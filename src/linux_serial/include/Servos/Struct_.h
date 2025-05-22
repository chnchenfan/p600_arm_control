#pragma once
#include<string.h>
#include<iostream>
#include <stdint.h>
#ifndef PKG_INFO_H
#define PKG_INFO_H
#pragma pack(1)
typedef enum {
	S_VER   = 0,			/* 读取固件版本和对应的硬件版本 */
	S_RL    = 1,			/* 读取读取相电阻和相电感 */
	S_PID   = 2,			/* 读取PID参数 */
	S_VBUS  = 3,			/* 读取总线电压 */
	S_CPHA  = 5,			/* 读取相电流 */
	S_ENCL  = 7,			/* 读取经过线性化校准后的编码器值 */
	S_TPOS  = 8,			/* 读取电机目标位置角度 */
	S_VEL   = 9,			/* 读取电机实时转速 */
	S_CPOS  = 10,			/* 读取电机实时位置角度 */
	S_PERR  = 11,			/* 读取电机位置误差角度 */
	S_FLAG  = 13,			/* 读取使能/到位/堵转状态标志位 */
	S_Conf  = 14,			/* 读取驱动参数 */
	S_State = 15,			/* 读取系统状态参数 */
	S_ORG   = 16,           /* 读取正在回零/回零失败状态标志位 */
}SysParams_t;

typedef struct State_pkg{
    uint8_t addr_;  // 电机编号
    uint16_t bus_voltage; // 总线电压
    uint16_t bus_phase_current; // 相电流
	uint16_t encoder_value; // 编码器值
	uint8_t direction_tp; // 方向，表示正负
    uint32_t target_position; // 目标位置
	uint8_t direction_tv; // 方向，表示正负
    uint16_t target_velocity; // 实时速度
	uint8_t direction_cp; // 方向，表示正负
    uint32_t current_position; // 实时位置
	uint8_t direction_pe; // 方向，表示正负
    uint32_t position_error; // 位置误差
	uint8_t ready_status; // 就绪状态
	uint8_t motor_status; // 电机状态
}State_pkg;


#pragma pack()
#endif 