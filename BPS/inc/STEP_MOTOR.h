
/*
 * ZDT X42S Emm 固件串口驱动。
 *
 * 默认自由协议格式：
 *   地址 + 功能码 + 数据 + 0x6B
 *
 * 名称带 no_reply 的接口只表示主机不读取回复，不会修改电机的 Response 参数。
 * 电机配置为默认 Receive 时，应使用等待回复版本；广播或明确配置为 None 时，
 * 才使用 no_reply 版本。
 */
#ifndef STEP_MOTOR_H_
#define STEP_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ZDT_EMM_DEFAULT_CHECK   (0x6BU)
#define ZDT_EMM_BROADCAST_ADDR  (0x00U)

#define ZDT_EMM_CMD_CALIBRATE_ENCODER   (0x06U)
#define ZDT_EMM_CMD_RESTART             (0x08U)
#define ZDT_EMM_CMD_ENABLE              (0xF3U)
#define ZDT_EMM_CMD_SPEED               (0xF6U)
#define ZDT_EMM_CMD_POSITION            (0xFDU)
#define ZDT_EMM_CMD_QUICK_POS_CONFIG    (0xF1U)
#define ZDT_EMM_CMD_QUICK_POS_RUN       (0xFCU)
#define ZDT_EMM_CMD_STOP                (0xFEU)
#define ZDT_EMM_CMD_SYNC_START          (0xFFU)
#define ZDT_EMM_CMD_ZERO_POSITION       (0x0AU)
#define ZDT_EMM_CMD_CLEAR_PROTECTION    (0x0EU)
#define ZDT_EMM_CMD_FACTORY_RESET       (0x0FU)
#define ZDT_EMM_CMD_SET_HOME_ZERO       (0x93U)
#define ZDT_EMM_CMD_HOME                (0x9AU)
#define ZDT_EMM_CMD_HOME_ABORT          (0x9CU)
#define ZDT_EMM_CMD_READ_VERSION        (0x1FU)
#define ZDT_EMM_CMD_READ_PHASE_RL       (0x20U)
#define ZDT_EMM_CMD_READ_VOLTAGE        (0x24U)
#define ZDT_EMM_CMD_READ_BUS_CURRENT    (0x26U)
#define ZDT_EMM_CMD_READ_PHASE_CURRENT  (0x27U)
#define ZDT_EMM_CMD_READ_ENCODER        (0x31U)
#define ZDT_EMM_CMD_READ_INPUT_PULSES   (0x32U)
#define ZDT_EMM_CMD_READ_TARGET         (0x33U)
#define ZDT_EMM_CMD_READ_SET_TARGET     (0x34U)
#define ZDT_EMM_CMD_READ_SPEED          (0x35U)
#define ZDT_EMM_CMD_READ_POSITION       (0x36U)
#define ZDT_EMM_CMD_READ_POSITION_ERROR (0x37U)
#define ZDT_EMM_CMD_READ_TEMPERATURE    (0x39U)
#define ZDT_EMM_CMD_READ_STATUS         (0x3AU)
#define ZDT_EMM_CMD_READ_HOME_STATUS    (0x3BU)
#define ZDT_EMM_CMD_READ_ALL_STATUS     (0x3CU)
#define ZDT_EMM_CMD_READ_IO_STATUS      (0x3DU)

#define ZDT_EMM_STATUS_OK               (0x02U)
#define ZDT_EMM_STATUS_LIMIT_LEFT       (0x12U)
#define ZDT_EMM_STATUS_LIMIT_RIGHT      (0x22U)
/* 参数范围或执行条件不满足，也用于低压、堵转、过热和过流等条件错误。 */
#define ZDT_EMM_STATUS_DEVICE_ERROR     (0xE2U)
#define ZDT_EMM_STATUS_FORMAT_ERROR     (0xEEU)
#define ZDT_EMM_STATUS_ACTION_DONE      (0x9FU)

#define ZDT_EMM_AUX_ENABLE              (0xABU)
#define ZDT_EMM_AUX_CALIBRATE_ENCODER   (0x45U)
#define ZDT_EMM_AUX_RESTART             (0x97U)
#define ZDT_EMM_AUX_STOP                (0x98U)
#define ZDT_EMM_AUX_SYNC_START          (0x66U)
#define ZDT_EMM_AUX_ZERO_POSITION       (0x6DU)
#define ZDT_EMM_AUX_CLEAR_PROTECTION    (0x52U)
#define ZDT_EMM_AUX_FACTORY_RESET       (0x5FU)
#define ZDT_EMM_AUX_SET_HOME_ZERO       (0x88U)
#define ZDT_EMM_AUX_HOME_ABORT          (0x48U)

#define ZDT_EMM_MAX_RPM                 (3000U)

typedef enum {
    ZDT_EMM_RESULT_OK = 0,
    ZDT_EMM_RESULT_TIMEOUT,
    ZDT_EMM_RESULT_BAD_FRAME,
    ZDT_EMM_RESULT_BAD_PARAM,
    ZDT_EMM_RESULT_LIMITED,
    ZDT_EMM_RESULT_DEVICE_ERROR,
    ZDT_EMM_RESULT_FORMAT_ERROR
} zdt_emm_result_t;

typedef enum {
    ZDT_EMM_DIR_CW  = 0x00,
    ZDT_EMM_DIR_CCW = 0x01
} zdt_emm_dir_t;

typedef enum {
    ZDT_EMM_EXEC_NOW  = 0x00,
    ZDT_EMM_EXEC_SYNC = 0x01
} zdt_emm_sync_t;

typedef enum {
    ZDT_EMM_MOVE_REL_LAST_TARGET = 0x00,
    ZDT_EMM_MOVE_ABS_ZERO        = 0x01,
    ZDT_EMM_MOVE_REL_CURRENT     = 0x02
} zdt_emm_move_mode_t;

typedef enum {
    ZDT_EMM_HOME_NEAREST            = 0x00,
    ZDT_EMM_HOME_DIRECTION          = 0x01,
    ZDT_EMM_HOME_COLLISION          = 0x02,
    ZDT_EMM_HOME_LIMIT              = 0x03,
    ZDT_EMM_HOME_ABSOLUTE_ZERO      = 0x04,
    ZDT_EMM_HOME_POWER_LOSS_ANGLE   = 0x05
} zdt_emm_home_mode_t;

typedef struct {
    bool enabled;
    bool reached;
    bool stall;
    bool stall_protect;
    bool left_limit_high;
    bool right_limit_high;
    bool power_loss;
} zdt_emm_status_flags_t;

typedef struct {
    bool encoder_ready;
    bool calibration_ready;
    bool homing;
    bool homing_failed;
    bool over_temperature;
    bool over_current;
} zdt_emm_home_flags_t;

typedef struct {
    uint16_t firmware_version;
    uint8_t hardware_series;
    uint8_t hardware_type;
    uint8_t hardware_version;
} zdt_emm_version_t;

typedef struct {
    uint16_t resistance_milliohm;
    uint16_t inductance_uh;
} zdt_emm_phase_parameters_t;

typedef struct {
    bool enable_pin_high;
    bool step_pin_high;
    bool direction_pin_high;
    bool direction_output;
} zdt_emm_io_status_t;

/**
 * @brief 清空 UART 接收 FIFO 中当前残留的所有字节。
 *
 * @param uart 与电机通信使用的 UART 实例。
 */
void zdt_emm_clear_rx(UART_Regs *uart);

/**
 * @brief 通过电机 UART 发送原始字节数据。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param data 待发送数据缓冲区。
 * @param len 待发送字节数。
 */
void zdt_emm_write(UART_Regs *uart, const uint8_t *data, uint8_t len);

/**
 * @brief 带微秒级超时地从 UART 读取 1 个字节。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param data 接收字节输出指针。
 * @param timeout_us 超时时间，单位为微秒。
 * @return 读取到 1 个字节返回 true，超时返回 false。
 */
bool zdt_emm_read_byte(UART_Regs *uart, uint8_t *data, uint32_t timeout_us);

/**
 * @brief 从 UART 读取固定数量的字节。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param data 接收数据输出缓冲区。
 * @param len 需要读取的字节数。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 全部读取成功返回 true，任意字节超时返回 false。
 */
bool zdt_emm_read(UART_Regs *uart, uint8_t *data, uint8_t len, uint32_t timeout_us);

/**
 * @brief 等待标准 4 字节动作命令回复。
 *
 * 期望回复格式：地址 + 功能码 + 状态码 + 0x6B。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。地址 0 仅允许用于多机同步触发命令。
 * @param code 期望匹配的功能码。
 * @param status 可选输出指针，用于返回原始状态码。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_wait_reply(
    UART_Regs *uart, uint8_t addr, uint8_t code, uint8_t *status, uint32_t timeout_us);

/**
 * @brief 使能或失能电机保持力矩，并等待回复。
 *
 * Emm 命令帧：Addr F3 AB Enable Sync 6B。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param enable true 表示使能，false 表示失能。
 * @param sync 立即执行，或缓存为同步执行。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_enable(
    UART_Regs *uart, uint8_t addr, bool enable, zdt_emm_sync_t sync, uint32_t timeout_us);

/**
 * @brief 使能或失能电机保持力矩，不等待回复。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param enable true 表示使能，false 表示失能。
 * @param sync 立即执行，或缓存为同步执行。
 * @return 参数合法且命令帧已发送时返回 ZDT_EMM_RESULT_OK。
 */
zdt_emm_result_t zdt_emm_enable_no_reply(
    UART_Regs *uart, uint8_t addr, bool enable, zdt_emm_sync_t sync);

/**
 * @brief 运行 Emm 速度模式，并等待回复。
 *
 * Emm 命令帧：Addr F6 Dir Speed Acc Sync 6B。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param dir 旋转方向。
 * @param rpm 转速，单位 RPM，有效范围 0-3000。
 * @param acc 加速度档位，有效范围 0-255。
 * @param sync 立即执行，或缓存为同步执行。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_speed(
    UART_Regs *uart, uint8_t addr, zdt_emm_dir_t dir, uint16_t rpm, uint8_t acc,
    zdt_emm_sync_t sync, uint32_t timeout_us);

/**
 * @brief 运行 Emm 速度模式，不等待回复。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param dir 旋转方向。
 * @param rpm 转速，单位 RPM，有效范围 0-3000。
 * @param acc 加速度档位，有效范围 0-255。
 * @param sync 立即执行，或缓存为同步执行。
 * @return 参数合法且命令帧已发送时返回 ZDT_EMM_RESULT_OK。
 */
zdt_emm_result_t zdt_emm_speed_no_reply(
    UART_Regs *uart, uint8_t addr, zdt_emm_dir_t dir, uint16_t rpm, uint8_t acc,
    zdt_emm_sync_t sync);

/**
 * @brief 按脉冲数运行 Emm 位置模式，并等待回复。
 *
 * Emm 命令帧：Addr FD Dir Speed Acc Pulses Mode Sync 6B。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param dir 旋转方向。
 * @param rpm 最大转速，单位 RPM，有效范围 0-3000。
 * @param acc 加速度档位，有效范围 0-255。
 * @param pulses 运动脉冲数。默认 16 细分时，3200 脉冲为 360 度。
 * @param mode 相对/绝对位置模式。
 * @param sync 立即执行，或缓存为同步执行。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_position_pulses(
    UART_Regs *uart, uint8_t addr, zdt_emm_dir_t dir, uint16_t rpm, uint8_t acc,
    uint32_t pulses, zdt_emm_move_mode_t mode, zdt_emm_sync_t sync, uint32_t timeout_us);

/**
 * @brief 按脉冲数运行 Emm 位置模式，不等待回复。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param dir 旋转方向。
 * @param rpm 最大转速，单位 RPM，有效范围 0-3000。
 * @param acc 加速度档位，有效范围 0-255。
 * @param pulses 运动脉冲数。
 * @param mode 相对/绝对位置模式。
 * @param sync 立即执行，或缓存为同步执行。
 * @return 参数合法且命令帧已发送时返回 ZDT_EMM_RESULT_OK。
 */
zdt_emm_result_t zdt_emm_position_pulses_no_reply(
    UART_Regs *uart, uint8_t addr, zdt_emm_dir_t dir, uint16_t rpm, uint8_t acc,
    uint32_t pulses, zdt_emm_move_mode_t mode, zdt_emm_sync_t sync);

/**
 * @brief 配置 Emm 快速位置模式，并等待回复。
 *
 * 按手册说明，该功能需要 V2.0.0 或更新版本固件。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param rpm 最大转速，单位 RPM，有效范围 0-3000。
 * @param acc 加速度档位，有效范围 0-255。
 * @param mode 相对/绝对位置模式。
 * @param sync 立即执行，或缓存为同步执行。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_quick_position_config(
    UART_Regs *uart, uint8_t addr, uint16_t rpm, uint8_t acc, zdt_emm_move_mode_t mode,
    zdt_emm_sync_t sync, uint32_t timeout_us);

/**
 * @brief 配置 Emm 快速位置模式，不等待回复。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param rpm 最大转速，单位 RPM，有效范围 0-3000。
 * @param acc 加速度档位，有效范围 0-255。
 * @param mode 相对/绝对位置模式。
 * @param sync 立即执行，或缓存为同步执行。
 * @return 参数合法且命令帧已发送时返回 ZDT_EMM_RESULT_OK。
 */
zdt_emm_result_t zdt_emm_quick_position_config_no_reply(
    UART_Regs *uart, uint8_t addr, uint16_t rpm, uint8_t acc, zdt_emm_move_mode_t mode,
    zdt_emm_sync_t sync);

/**
 * @brief 运行 Emm 快速位置运动，并等待回复。
 *
 * 脉冲数按手册定义为有符号 int32_t。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param pulses 基于快速位置模式配置的有符号目标脉冲数。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_quick_position_run(
    UART_Regs *uart, uint8_t addr, int32_t pulses, uint32_t timeout_us);

/**
 * @brief 运行 Emm 快速位置运动，不等待回复。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param pulses 基于快速位置模式配置的有符号目标脉冲数。
 * @return 命令帧发送后返回 ZDT_EMM_RESULT_OK。
 */
zdt_emm_result_t zdt_emm_quick_position_run_no_reply(
    UART_Regs *uart, uint8_t addr, int32_t pulses);

/**
 * @brief 立即停止电机运动，并等待回复。
 *
 * Emm 命令帧：Addr FE 98 Sync 6B。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param sync 立即执行，或缓存为同步执行。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_stop(
    UART_Regs *uart, uint8_t addr, zdt_emm_sync_t sync, uint32_t timeout_us);

/**
 * @brief 立即停止电机运动，不等待回复。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param sync 立即执行，或缓存为同步执行。
 * @return 参数合法且命令帧已发送时返回 ZDT_EMM_RESULT_OK。
 */
zdt_emm_result_t zdt_emm_stop_no_reply(UART_Regs *uart, uint8_t addr, zdt_emm_sync_t sync);

/**
 * @brief 触发回零运动，并等待回复。
 *
 * Emm 命令帧：Addr 9A Mode Sync 6B。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param mode 回零模式。
 * @param sync 立即执行，或缓存为同步执行。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_home(
    UART_Regs *uart, uint8_t addr, zdt_emm_home_mode_t mode, zdt_emm_sync_t sync,
    uint32_t timeout_us);

/**
 * @brief 触发回零运动，不等待回复。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param mode 回零模式。
 * @param sync 立即执行，或缓存为同步执行。
 * @return 参数合法且命令帧已发送时返回 ZDT_EMM_RESULT_OK。
 */
zdt_emm_result_t zdt_emm_home_no_reply(
    UART_Regs *uart, uint8_t addr, zdt_emm_home_mode_t mode, zdt_emm_sync_t sync);

/**
 * @brief 中断并退出回零操作，并等待回复。
 *
 * Emm 命令帧：Addr 9C 48 6B。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_home_abort(UART_Regs *uart, uint8_t addr, uint32_t timeout_us);

/**
 * @brief 中断并退出回零操作，不等待回复。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 */
void zdt_emm_home_abort_no_reply(UART_Regs *uart, uint8_t addr);

/**
 * @brief 触发已缓存的同步运动命令，并等待回复。
 *
 * 使用广播地址 0 和命令 FF 66。该等待回复版本适用于单电机，或总线上
 * 只有一个设备会回复的场景。多电机广播控制建议使用
 * zdt_emm_sync_start_no_reply()，避免多个设备同时回复造成总线冲突。
 *
 * @param uart 与电机总线通信使用的 UART 实例。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_sync_start(UART_Regs *uart, uint32_t timeout_us);

/**
 * @brief 触发所有已缓存的同步运动命令，不等待回复。
 *
 * @param uart 与电机总线通信使用的 UART 实例。
 */
void zdt_emm_sync_start_no_reply(UART_Regs *uart);

/**
 * @brief 将当前电机位置角度清零，并等待回复。
 *
 * Emm 命令帧：Addr 0A 6D 6B。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_zero_position(UART_Regs *uart, uint8_t addr, uint32_t timeout_us);

/**
 * @brief 将当前电机位置角度清零，不等待回复。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 */
void zdt_emm_zero_position_no_reply(UART_Regs *uart, uint8_t addr);

/**
 * @brief 解除堵转、过热或过流保护，并等待回复。
 *
 * Emm 命令帧：Addr 0E 52 6B。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_clear_protection(
    UART_Regs *uart, uint8_t addr, uint32_t timeout_us);

/**
 * @brief 解除堵转、过热或过流保护，不等待回复。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 */
void zdt_emm_clear_protection_no_reply(UART_Regs *uart, uint8_t addr);

/**
 * @brief 触发编码器线性化校准，并等待接收确认。
 *
 * 校准时电机会缓慢正反转。仅允许使用单机地址。
 */
zdt_emm_result_t zdt_emm_calibrate_encoder(
    UART_Regs *uart, uint8_t addr, uint32_t timeout_us);

/**
 * @brief 复位并重启电机驱动板，并等待接收确认。
 *
 * 该命令仅适用于 X42S/Y42，且仅允许使用单机地址。
 */
zdt_emm_result_t zdt_emm_restart(UART_Regs *uart, uint8_t addr, uint32_t timeout_us);

/**
 * @brief 恢复出厂设置，并等待接收确认。
 *
 * 执行后需要断电重启并重新空载校准编码器。仅允许使用单机地址。
 */
zdt_emm_result_t zdt_emm_factory_reset(
    UART_Regs *uart, uint8_t addr, uint32_t timeout_us);

/**
 * @brief 将当前位置设置为单圈回零零点。
 *
 * @param store true 表示写入非易失存储，false 表示仅本次上电有效。
 */
zdt_emm_result_t zdt_emm_set_home_zero(
    UART_Regs *uart, uint8_t addr, bool store, uint32_t timeout_us);

/**
 * @brief 读取电机状态标志。
 *
 * Emm 命令帧：Addr 3A 6B。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param status 电机状态标志输出结构体。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_read_status(
    UART_Regs *uart, uint8_t addr, zdt_emm_status_flags_t *status, uint32_t timeout_us);

/**
 * @brief 读取回零状态标志。
 *
 * Emm 命令帧：Addr 3B 6B。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param status 回零状态标志输出结构体。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_read_home_status(
    UART_Regs *uart, uint8_t addr, zdt_emm_home_flags_t *status, uint32_t timeout_us);

/**
 * @brief 读取电机实时转速，单位 RPM。
 *
 * Emm 命令帧：Addr 35 6B。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param rpm 有符号实时转速输出，单位 RPM。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_read_speed_rpm(
    UART_Regs *uart, uint8_t addr, int32_t *rpm, uint32_t timeout_us);

/**
 * @brief 读取 Emm 实时原始位置值。
 *
 * 协议返回 32 位位置幅值，并用单独字节表示正负号。对于常见的 Emm
 * 单圈位置含义，0-65535 对应一圈：角度 = value * 360 / 65536。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param position 有符号原始位置值输出。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_read_position_raw(
    UART_Regs *uart, uint8_t addr, int64_t *position, uint32_t timeout_us);

/**
 * @brief 读取电机母线电压，单位毫伏。
 *
 * Emm 命令帧：Addr 24 6B。
 *
 * @param uart 与电机通信使用的 UART 实例。
 * @param addr 电机地址。
 * @param voltage_mv 母线电压输出，单位 mV。
 * @param timeout_us 单字节超时时间，单位为微秒。
 * @return 解析后的命令结果。
 */
zdt_emm_result_t zdt_emm_read_bus_voltage_mv(
    UART_Regs *uart, uint8_t addr, uint16_t *voltage_mv, uint32_t timeout_us);

/** @brief 读取固件版本和硬件版本信息。 */
zdt_emm_result_t zdt_emm_read_version(
    UART_Regs *uart, uint8_t addr, zdt_emm_version_t *version, uint32_t timeout_us);

/** @brief 读取相电阻（mΩ）和相电感（uH）。 */
zdt_emm_result_t zdt_emm_read_phase_parameters(UART_Regs *uart, uint8_t addr,
    zdt_emm_phase_parameters_t *parameters, uint32_t timeout_us);

/** @brief 读取估算的总线电流，单位 mA。 */
zdt_emm_result_t zdt_emm_read_bus_current_ma(
    UART_Regs *uart, uint8_t addr, uint16_t *current_ma, uint32_t timeout_us);

/** @brief 读取电机实际相电流，单位 mA。 */
zdt_emm_result_t zdt_emm_read_phase_current_ma(
    UART_Regs *uart, uint8_t addr, uint16_t *current_ma, uint32_t timeout_us);

/** @brief 读取单圈线性化编码器值，0-65535 对应 0-360 度。 */
zdt_emm_result_t zdt_emm_read_encoder_raw(
    UART_Regs *uart, uint8_t addr, uint16_t *encoder, uint32_t timeout_us);

/** @brief 读取累计输入脉冲数。 */
zdt_emm_result_t zdt_emm_read_input_pulses(
    UART_Regs *uart, uint8_t addr, int64_t *pulses, uint32_t timeout_us);

/** @brief 读取电机目标位置原始值。 */
zdt_emm_result_t zdt_emm_read_target_position_raw(
    UART_Regs *uart, uint8_t addr, int64_t *position, uint32_t timeout_us);

/** @brief 读取电机实时设定的目标位置原始值。 */
zdt_emm_result_t zdt_emm_read_set_target_position_raw(
    UART_Regs *uart, uint8_t addr, int64_t *position, uint32_t timeout_us);

/** @brief 读取驱动板温度，单位摄氏度。 */
zdt_emm_result_t zdt_emm_read_temperature_c(
    UART_Regs *uart, uint8_t addr, int16_t *temperature_c, uint32_t timeout_us);

/** @brief 读取电机位置误差原始值。 */
zdt_emm_result_t zdt_emm_read_position_error_raw(
    UART_Regs *uart, uint8_t addr, int64_t *error, uint32_t timeout_us);

/** @brief 一次读取回零状态和电机状态。 */
zdt_emm_result_t zdt_emm_read_all_status(UART_Regs *uart, uint8_t addr,
    zdt_emm_home_flags_t *home_status, zdt_emm_status_flags_t *motor_status,
    uint32_t timeout_us);

/** @brief 读取使能、脉冲和方向引脚的电平及方向输出模式。 */
zdt_emm_result_t zdt_emm_read_io_status(
    UART_Regs *uart, uint8_t addr, zdt_emm_io_status_t *status, uint32_t timeout_us);

/**
 * @brief 将 0.1 度单位的角度值换算为脉冲数。
 *
 * @param degrees_x10 角度值，单位为 0.1 度。例如 3600 表示 360.0 度。
 * @param pulses_per_rev 每 360 度对应的脉冲数。Emm 默认示例为 3200。
 * @return 四舍五入后的脉冲数。
 */
uint32_t zdt_emm_degrees_to_pulses_x10(uint32_t degrees_x10, uint16_t pulses_per_rev);

#ifdef __cplusplus
}
#endif
#endif/* ZDT_EMM_H */