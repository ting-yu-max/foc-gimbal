#ifndef FOC_H
#define FOC_H

#include "main.h"

/* ===== FOC 参数 ===== */
#define PWM_ARR          4200.0f   /* TIM1 ARR（中心对齐，PWM=20kHz） */

#define VDC              12.0f      /* 母线电压 VM（电机电源伏特，按实际改） */
#define OPEN_LOOP_VQ     1.0f      /* 开环 Vq（伏特），先小值防过流，转不动再调大 */
#define OPEN_LOOP_DELTA  0.02f     /* 每周期电角度增量(rad)，决定开环转速 */
#define POLE_PAIRS       7
#define ALIGN_UQ         3.0f
#define CLOSED_LOOP_VQ   1.0f
#define DT               0.001f
#define VEL_LP_ALPHA     0.1f
#define VEL_KP           0.3f
#define VEL_KI           6.0f
#define ANGLE_KP         0.5f
#define VEL_LIMIT        3.0f
#define UQ_LIMIT         1.0f
#define ATTITUDE_KP      5.0f

/* ===== FOC 接口 ===== */
void inv_park(float Vd, float Vq, float theta, float *Valpha, float *Vbeta);
void svpwm(float Valpha, float Vbeta, float *da, float *db, float *dc);
void foc_openloop_step(void);
extern float zero_electric_angle;
void foc_align_sensor(void);
extern float mech_angle;
void foc_closedloop_step(void);
extern float vel_filt;
void foc_velocity_update(void);
extern float uq_out;
void foc_velocity_loop(void);
extern float vel_target;
extern float angle_target;
void foc_angle_loop(void);
extern float pitch;
void foc_attitude_loop(void);

#endif /* FOC_H */
