#include "My_STEPMOTOR.h"

/**
 * @brief 步进电机 UART2 初始化（轮询模式，不禁用中断的 NVIC）
 *
 * 与 UART2_Init() 的区别：不清除 NVIC 挂起位、不使能 NVIC，
 * 防止电机回复数据触发 Default_Handler 导致程序卡死。
 */
void StepMotor_Init(void)
{
    /* 清空 RX FIFO 残留数据 */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_2_INST)) 
    {
        (void)DL_UART_Main_receiveData(UART_2_INST);
    }

    /* 清除 UART 外设中断标志 */
    DL_UART_Main_clearInterruptStatus(UART_2_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);

    /* 使能电机（上力矩） */
    zdt_emm_enable_no_reply(STEP_MOTOR_UART, STEP_MOTOR_ADDR,
                            true, ZDT_EMM_EXEC_NOW);
    Delay_ms(200);

    /* 清空电机回复数据，避免 RX FIFO 累积溢出 */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_2_INST)) 
    {
        (void)DL_UART_Main_receiveData(UART_2_INST);
    }
}

/**
 * @brief 相对当前位置旋转指定角度（0.1度单位）
 */
zdt_emm_result_t StepMotor_MoveRelativeAngle(zdt_emm_dir_t dir, uint32_t deg_x10)
{
    uint32_t pulses = zdt_emm_degrees_to_pulses_x10(deg_x10, PULSES_PER_REV);

    return zdt_emm_position_pulses(
        STEP_MOTOR_UART, STEP_MOTOR_ADDR,
        dir, MOTOR_RPM, MOTOR_ACC,
        pulses,
        ZDT_EMM_MOVE_REL_CURRENT,
        ZDT_EMM_EXEC_NOW,
        MOTOR_TIMEOUT_US);
}

/**
 * @brief 旋转到绝对零点位置（需先回零建立零点参考）
 */
zdt_emm_result_t StepMotor_MoveToAbsoluteAngle(uint32_t deg_x10)
{
    uint32_t pulses = zdt_emm_degrees_to_pulses_x10(deg_x10, PULSES_PER_REV);

    return zdt_emm_position_pulses(
        STEP_MOTOR_UART, STEP_MOTOR_ADDR,
        ZDT_EMM_DIR_CW,
        MOTOR_RPM, MOTOR_ACC,
        pulses,
        ZDT_EMM_MOVE_ABS_ZERO,
        ZDT_EMM_EXEC_NOW,
        MOTOR_TIMEOUT_US);
}

/**
 * @brief 立即停止电机运动（保持力矩不释放）
 */
zdt_emm_result_t StepMotor_Stop(void)
{
    return zdt_emm_stop(
        STEP_MOTOR_UART, STEP_MOTOR_ADDR,
        ZDT_EMM_EXEC_NOW, MOTOR_TIMEOUT_US);
}

zdt_emm_result_t StepMotor_Stop_NoReply(void)
{
    return zdt_emm_stop_no_reply(
        STEP_MOTOR_UART, STEP_MOTOR_ADDR,
        ZDT_EMM_EXEC_NOW);
}

/**
 * @brief 将当前电机位置清零（设为新的绝对零点）
 */
zdt_emm_result_t StepMotor_ZeroPosition(void)
{
    return zdt_emm_zero_position(
        STEP_MOTOR_UART, STEP_MOTOR_ADDR, MOTOR_TIMEOUT_US);
}

/**
 * @brief 配置快速位置模式参数（配合 StepMotor_QuickMoveByPulses 使用）
 */
zdt_emm_result_t StepMotor_QuickConfig(zdt_emm_move_mode_t mode)
{
    return zdt_emm_quick_position_config(
        STEP_MOTOR_UART, STEP_MOTOR_ADDR,
        MOTOR_RPM, MOTOR_ACC,
        mode,
        ZDT_EMM_EXEC_NOW,
        MOTOR_TIMEOUT_US);
}

/**
 * @brief 快速位置运动（需先调用 StepMotor_QuickConfig）
 */
zdt_emm_result_t StepMotor_QuickMoveByPulses(int32_t pulses)
{
    return zdt_emm_quick_position_run(
        STEP_MOTOR_UART, STEP_MOTOR_ADDR,
        pulses, MOTOR_TIMEOUT_US);
}

/* ================================================================
 * 读取函数 —— PID 闭环反馈
 * ================================================================ */

/**
 * @brief 读取单圈编码器原始值（0~65535 = 0°~360°）
 */
zdt_emm_result_t StepMotor_ReadEncoder(uint16_t *encoder)
{
    return zdt_emm_read_encoder_raw(
        STEP_MOTOR_UART, STEP_MOTOR_ADDR,
        encoder, MOTOR_TIMEOUT_US);
}

/**
 * @brief 读取逻辑位置（受 ZeroPosition 影响）
 *
 * 65536 单位 = 360°，正转增加，反转减少。
 */
zdt_emm_result_t StepMotor_ReadPosition(int64_t *position)
{
    return zdt_emm_read_position_raw(
        STEP_MOTOR_UART, STEP_MOTOR_ADDR,
        position, MOTOR_TIMEOUT_US);
}

/**
 * @brief 读取基于零点的当前角度（0.1° 单位，有符号）
 *
 * 换算：deg_x10 = position * 3600 / 65536
 */
zdt_emm_result_t StepMotor_ReadAngle_x10(int32_t *deg_x10)
{
    int64_t position;
    zdt_emm_result_t ret;

    ret = zdt_emm_read_position_raw(
        STEP_MOTOR_UART, STEP_MOTOR_ADDR,
        &position, MOTOR_TIMEOUT_US);
    if (ret == ZDT_EMM_RESULT_OK) {
        /* position * 3600 / 65536，四舍五入 */
        *deg_x10 = (int32_t)((position * 3600 + (position >= 0 ? 32768 : -32768)) / 65536);
    }
    return ret;
}

/**
 * @brief 读取电机状态标志（enabled / reached / stall / limit 等）
 */
zdt_emm_result_t StepMotor_ReadStatus(zdt_emm_status_flags_t *flags)
{
    return zdt_emm_read_status(
        STEP_MOTOR_UART, STEP_MOTOR_ADDR,
        flags, MOTOR_TIMEOUT_US);
}

/**
 * @brief 快速查询是否已到达目标位置
 */
zdt_emm_result_t StepMotor_IsReached(bool *reached)
{
    zdt_emm_status_flags_t flags;
    zdt_emm_result_t ret;

    ret = zdt_emm_read_status(
        STEP_MOTOR_UART, STEP_MOTOR_ADDR,
        &flags, MOTOR_TIMEOUT_US);
    if (ret == ZDT_EMM_RESULT_OK) {
        *reached = flags.reached;
    }
    return ret;
}

/* ================================================================
 * 速度模式 —— PID 连续输出
 * ================================================================ */

/**
 * @brief 速度模式运行（阻塞）
 */
zdt_emm_result_t StepMotor_RunSpeed(zdt_emm_dir_t dir, uint16_t rpm)
{
    return zdt_emm_speed(
        STEP_MOTOR_UART, STEP_MOTOR_ADDR,
        dir, rpm, MOTOR_ACC,
        ZDT_EMM_EXEC_NOW, MOTOR_TIMEOUT_US);
}

/**
 * @brief 速度模式运行（非阻塞，PID 内环推荐）
 */
zdt_emm_result_t StepMotor_RunSpeed_NoReply(zdt_emm_dir_t dir, uint16_t rpm)
{
    return zdt_emm_speed_no_reply(
        STEP_MOTOR_UART, STEP_MOTOR_ADDR,
        dir, rpm, MOTOR_ACC,
        ZDT_EMM_EXEC_NOW);
}

/* ================================================================
 * 非阻塞位置运动 —— PID 内环不下发完整脉冲，改为微调
 * ================================================================ */

/**
 * @brief 相对角度运动（非阻塞）
 */
zdt_emm_result_t StepMotor_MoveRelativeAngle_NoReply(zdt_emm_dir_t dir, uint32_t deg_x10)
{
    uint32_t pulses = zdt_emm_degrees_to_pulses_x10(deg_x10, PULSES_PER_REV);

    return zdt_emm_position_pulses_no_reply(
        STEP_MOTOR_UART, STEP_MOTOR_ADDR,
        dir, MOTOR_RPM, MOTOR_ACC,
        pulses,
        ZDT_EMM_MOVE_REL_CURRENT,
        ZDT_EMM_EXEC_NOW);
}

/**
 * @brief 绝对角度运动（非阻塞）
 */
zdt_emm_result_t StepMotor_MoveToAbsoluteAngle_NoReply(uint32_t deg_x10)
{
    uint32_t pulses = zdt_emm_degrees_to_pulses_x10(deg_x10, PULSES_PER_REV);

    return zdt_emm_position_pulses_no_reply(
        STEP_MOTOR_UART, STEP_MOTOR_ADDR,
        ZDT_EMM_DIR_CW,
        MOTOR_RPM, MOTOR_ACC,
        pulses,
        ZDT_EMM_MOVE_ABS_ZERO,
        ZDT_EMM_EXEC_NOW);
}