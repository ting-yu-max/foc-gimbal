#include "foc.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include "math.h"
#include <stdint.h>
#include "as5600.h"
#include "usart.h"

#define PI 3.14159265f
#define TAU 6.2831853f  /*2pi*/

float zero_electric_angle = 0;
float mech_angle = 0;
float vel_filt = 0;
float uq_out = 0;
float vel_target = 0;
float angle_target = 2.0f;
float pitch = 0;

/*反Park变换：dq->a,b*/
void inv_park(float Vd, float Vq, float theta, float *Valpha, float *Vbeta)
{
    float ct = cosf(theta);
    float st = sinf(theta);
    *Valpha = Vd *ct - Vq * st;
    *Vbeta = Vd * st + Vq *ct;
}

/*SVPWM:a,b->三相占空比（0~1）*/
void svpwm(float Valpha, float Vbeta, float *da, float *db, float *dc)
{
    /*Clarke反变换：a,b ->三项正弦abc*/
    float Va = Valpha;
    float Vb = -0.5f * Valpha + 0.8660254f * Vbeta;
    float Vc = -0.5f *Valpha - 0.8660254f *Vbeta;

    /*零序注入：让三相对称，提高母线电压利用率*/
    float Vmax = fmaxf(fmaxf(Va, Vb), Vc);
    float Vmin = fminf(fminf(Va, Vb), Vc);
    float Vn = -0.5f*(Vmax + Vmin);

    /*归一化到占空比*/
    *da = (Va + Vn) / VDC + 0.5f;
    *db = (Vb + Vn) / VDC + 0.5f;
    *dc = (Vc + Vn) / VDC + 0.5f;
}

/*开环FOC步进（放TIM6中断调用*/
static float _elec_angle = 0.0f;

void foc_openloop_step(void)
{
    _elec_angle += OPEN_LOOP_DELTA;
    if(_elec_angle >=TAU) _elec_angle -= TAU;

    float Valpha, Vbeta;
    inv_park(0.0f, OPEN_LOOP_VQ, _elec_angle, &Valpha, &Vbeta);

    float da, db, dc;
    svpwm(Valpha, Vbeta, &da, &db, &dc);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(da * PWM_ARR));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(db * PWM_ARR));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dc * PWM_ARR));
}

void foc_align_sensor(void)
{
    float Valpha, Vbeta, da, db, dc;

    inv_park(0.0f, ALIGN_UQ, 0.0f, &Valpha, &Vbeta);
    svpwm(Valpha, Vbeta, &da, &db, &dc);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, da * PWM_ARR);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, db * PWM_ARR);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dc * PWM_ARR);

    HAL_Delay(700);

    float sum = 0;
    int valid = 0;
    for (int i = 0; i < 20; i++) {
        float a = AS5600_GetAngleRad();
        if (a < 100.0f) { sum += a; valid++;}
        HAL_Delay(2);
    }
    zero_electric_angle = (valid > 0 ? sum / valid : 0) * POLE_PAIRS;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
}

void foc_closedloop_step(void)
{
    float elec = -mech_angle * POLE_PAIRS - zero_electric_angle + PI / 2;
    elec = fmodf(elec, TAU);
    if (elec < 0.0f) elec += TAU;

    float Valpha, Vbeta, da, db, dc;
    inv_park(0.0f, uq_out, elec, &Valpha, &Vbeta);
    svpwm(Valpha, Vbeta, &da, &db, &dc);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(da * PWM_ARR));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(db * PWM_ARR));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dc * PWM_ARR));
    
}

/*测速：mech_angle差分+一阶低通滤波*/
static float _mech_last = 0;
void foc_velocity_update(void)
{
    float dtheta = mech_angle - _mech_last;
    if (dtheta > PI) dtheta -= TAU;
    if (dtheta < -PI) dtheta +=TAU;
    _mech_last = mech_angle;

    float vel_raw = dtheta / DT;
    vel_filt += VEL_LP_ALPHA * (vel_raw - vel_filt);
}

void foc_velocity_loop(void)
{
    static float integ = 0;
    float err = vel_target - vel_filt;
    integ += err * DT;
    if (integ > UQ_LIMIT) integ = UQ_LIMIT;
    if (integ < -UQ_LIMIT) integ = -UQ_LIMIT;

    uq_out = VEL_KP * err + VEL_KI * integ;
    if (uq_out > UQ_LIMIT) uq_out = UQ_LIMIT;
    if (uq_out < -UQ_LIMIT) uq_out = -UQ_LIMIT;
}

void foc_angle_loop(void)
{
    float err = angle_target - mech_angle;
    if(err > PI) err -= TAU;
    if(err < -PI) err +=TAU;

    float cmd = ANGLE_KP *err;
    if (cmd > VEL_LIMIT) cmd = VEL_LIMIT;
    if (cmd < -VEL_LIMIT) cmd = -VEL_LIMIT;
    vel_target = cmd;
}

void foc_attitude_loop(void) {
    float err = -pitch * 0.017453f;
    vel_target = ATTITUDE_KP * err;
    if (vel_target > VEL_LIMIT) vel_target = VEL_LIMIT;
    if (vel_target < -VEL_LIMIT) vel_target = -VEL_LIMIT;
}
