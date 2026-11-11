#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

#include <stdint.h>

/* =====================================================================
 * app_chassis.h —— 底盘应用层对外的四个动作
 *
 * 文件作用：
 *   给 main.c / 遥控 / 上位机提供唯一入口：初始化、周期任务、设速度、急停。
 *   调用方不需要知道下面有 CAN、有电机、有运动学。
 *
 * 命名对照：
 *   App       = Application（应用层）
 *   Chassis   = 底盘
 *   Init / Task / SetVelocity / Stop = 初始化 / 周期任务 / 设速度 / 停车
 *   v = 线速度 m/s     w = 角速度 rad/s
 *   Rpm = 电机转子转速（转/分）
 * ===================================================================== */

/* 作用：建电机对象、建运动学对象、开 CAN、发使能。上电调一次，返回 0 表示成功 */
uint8_t App_Chassis_Init(void);

/* 作用：底盘主任务，需要在 while(1) 里不停调用。
 * 内部自己做 5 ms 周期门控，所以调用频率高一点也没关系 */
void App_Chassis_Task(void);

/* 作用：设置底盘目标速度，只写变量不做运算，不会阻塞。
 * v 单位 m/s（前进为正），w 单位 rad/s（左转为正、逆时针） */
void App_Chassis_SetVelocity(float v, float w);

/* 作用：急停。目标清零并立即下发 0 转速，不等下一个控制周期 */
void App_Chassis_Stop(void);

/* 作用：读当前左右轮转子实测转速 rpm，调试/看波形用 */
float App_Chassis_GetLeftRpm(void);
float App_Chassis_GetRightRpm(void);

#endif
