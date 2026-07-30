#ifndef MY_STEP_MOTOR_H_
#define MY_STEP_MOTOR_H_

#define STEP_MOTOR_UART       UART_2_INST  // 步进电机使用UART2（原Zigbee）
#define STEP_MOTOR_ADDR       0x01U      // 电机地址
#define PULSES_PER_REV        3200U      // 16细分：3200脉冲/圈
#define MOTOR_RPM             500U       // 转速 RPM
#define MOTOR_ACC             50U        // 加速度档位 0~255
#define MOTOR_TIMEOUT_US      200000U    // 超时 200ms

#include "ti_msp_dl_config.h"
#include "board.h"
#include "STEP_MOTOR.h"

#ifdef __cplusplus
extern "C" {
#endif

void StepMotor_Init(void);

/**
 * @brief 相对当前位置旋转指定角度（0.1度单位）
 *
 * @param dir       旋转方向 CW / CCW
 * @param deg_x10   角度 ×10，例如 900 = 90.0°，3600 = 360.0°
 * @return          命令执行结果，ZDT_EMM_RESULT_OK 表示定位完成
 */
zdt_emm_result_t StepMotor_MoveRelativeAngle(zdt_emm_dir_t dir, uint32_t deg_x10);

/**
 * @brief 旋转到绝对零点位置（需先回零建立零点参考）
 *
 * 方向由电机根据最短路径自动决定。
 *
 * @param deg_x10   绝对角度 ×10，范围 0~3600 对应 0°~360°
 * @return          命令执行结果，ZDT_EMM_RESULT_OK 表示定位完成
 */
zdt_emm_result_t StepMotor_MoveToAbsoluteAngle(uint32_t deg_x10);

/**
 * @brief 立即停止电机运动（保持力矩不释放）
 *
 * @return 命令执行结果
 */
zdt_emm_result_t StepMotor_Stop(void);

/**
 * @brief 将当前电机位置清零（设为新的绝对零点）
 *
 * @return 命令执行结果
 */
zdt_emm_result_t StepMotor_ZeroPosition(void);

/**
 * @brief 执行回零操作（以最近碰撞点为参考）
 *
 * 回零完成前会阻塞，超时默认 5 秒。
 *
 * @return 命令执行结果
 */
zdt_emm_result_t StepMotor_Home(void);

/**
 * @brief 配置快速位置模式参数（配合 StepMotor_QuickMoveByPulses 使用）
 *
 * 配置一次后可反复调用 QuickMove，适合连续多次定位。
 *
 * @param mode  相对/绝对位置模式
 * @return      命令执行结果
 */
zdt_emm_result_t StepMotor_QuickConfig(zdt_emm_move_mode_t mode);

/**
 * @brief 快速位置运动（需先调用 StepMotor_QuickConfig）
 *
 * @param pulses  有符号脉冲数，正=正向，负=反向
 * @return        命令执行结果
 */
zdt_emm_result_t StepMotor_QuickMoveByPulses(int32_t pulses);

/* ========== 读取函数（PID 闭环必需） ========== */

/**
 * @brief 读取单圈编码器原始值
 *
 * @param encoder  输出：0~65535 对应 0°~360°
 * @return         读取结果
 */
zdt_emm_result_t StepMotor_ReadEncoder(uint16_t *encoder);

/**
 * @brief 读取当前角度（0.1° 单位，基于零点）
 *
 * @param deg_x10  输出：角度 ×10，如 900 = 90.0°
 * @return         读取结果
 */
zdt_emm_result_t StepMotor_ReadAngle_x10(uint32_t *deg_x10);

/**
 * @brief 读取电机状态标志
 *
 * @param flags  输出：状态结构体（含 reached / stall / limit 等）
 * @return       读取结果
 */
zdt_emm_result_t StepMotor_ReadStatus(zdt_emm_status_flags_t *flags);

/**
 * @brief 快速查询是否已到达目标位置
 *
 * @param reached  输出：true = 已到位
 * @return         读取结果
 */
zdt_emm_result_t StepMotor_IsReached(bool *reached);

/* ========== 速度模式（PID 连续输出） ========== */

/**
 * @brief 速度模式运行（阻塞，等待回复）
 *
 * @param dir  旋转方向
 * @param rpm  转速 0~3000
 * @return     命令执行结果
 */
zdt_emm_result_t StepMotor_RunSpeed(zdt_emm_dir_t dir, uint16_t rpm);

/**
 * @brief 速度模式运行（非阻塞，不等待回复）
 *
 * 适合 PID 内环高频调速，配合 _no_reply 版本减少通信延迟。
 *
 * @param dir  旋转方向
 * @param rpm  转速 0~3000
 * @return     命令帧已发送成功
 */
zdt_emm_result_t StepMotor_RunSpeed_NoReply(zdt_emm_dir_t dir, uint16_t rpm);

/* ========== 非阻塞位置运动（PID 内环） ========== */

/**
 * @brief 相对角度运动（非阻塞，不等待回复）
 *
 * 发完命令立即返回，配合 StepMotor_IsReached 轮询到位状态。
 *
 * @param dir       旋转方向
 * @param deg_x10   角度 ×10
 * @return          命令帧已发送成功
 */
zdt_emm_result_t StepMotor_MoveRelativeAngle_NoReply(zdt_emm_dir_t dir, uint32_t deg_x10);

/**
 * @brief 绝对角度运动（非阻塞，不等待回复）
 *
 * @param deg_x10  绝对角度 ×10
 * @return         命令帧已发送成功
 */
zdt_emm_result_t StepMotor_MoveToAbsoluteAngle_NoReply(uint32_t deg_x10);

#ifdef __cplusplus
}
#endif

#endif