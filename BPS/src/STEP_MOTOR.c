#include "STEP_MOTOR.h"
static void zdt_emm_put_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t) (value >> 8);
    dst[1] = (uint8_t) value;
}

static void zdt_emm_put_u32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t) (value >> 24);
    dst[1] = (uint8_t) (value >> 16);
    dst[2] = (uint8_t) (value >> 8);
    dst[3] = (uint8_t) value;
}

static uint16_t zdt_emm_get_u16(const uint8_t *src)
{
    return (uint16_t) (((uint16_t) src[0] << 8) | src[1]);
}

static uint32_t zdt_emm_get_u32(const uint8_t *src)
{
    return ((uint32_t) src[0] << 24) | ((uint32_t) src[1] << 16) |
           ((uint32_t) src[2] << 8) | src[3];
}

static void zdt_emm_send_frame(UART_Regs *uart, uint8_t *frame, uint8_t len)
{
    frame[len - 1U] = ZDT_EMM_DEFAULT_CHECK;
    zdt_emm_write(uart, frame, len);
}

static zdt_emm_result_t zdt_emm_status_to_result(uint8_t status)
{
    if ((status == ZDT_EMM_STATUS_OK) || (status == ZDT_EMM_STATUS_ACTION_DONE)) {
        return ZDT_EMM_RESULT_OK;
    }
    if ((status == ZDT_EMM_STATUS_LIMIT_LEFT) || (status == ZDT_EMM_STATUS_LIMIT_RIGHT)) {
        return ZDT_EMM_RESULT_LIMITED;
    }
    if (status == ZDT_EMM_STATUS_DEVICE_ERROR) {
        return ZDT_EMM_RESULT_DEVICE_ERROR;
    }
    if (status == ZDT_EMM_STATUS_FORMAT_ERROR) {
        return ZDT_EMM_RESULT_FORMAT_ERROR;
    }

    return ZDT_EMM_RESULT_BAD_FRAME;
}

static bool zdt_emm_sync_valid(zdt_emm_sync_t sync)
{
    return (sync == ZDT_EMM_EXEC_NOW) || (sync == ZDT_EMM_EXEC_SYNC);
}

static bool zdt_emm_dir_valid(zdt_emm_dir_t dir)
{
    return (dir == ZDT_EMM_DIR_CW) || (dir == ZDT_EMM_DIR_CCW);
}

static bool zdt_emm_move_mode_valid(zdt_emm_move_mode_t mode)
{
    return (mode == ZDT_EMM_MOVE_REL_LAST_TARGET) || (mode == ZDT_EMM_MOVE_ABS_ZERO) ||
           (mode == ZDT_EMM_MOVE_REL_CURRENT);
}

static bool zdt_emm_home_mode_valid(zdt_emm_home_mode_t mode)
{
    return (mode == ZDT_EMM_HOME_NEAREST) || (mode == ZDT_EMM_HOME_DIRECTION) ||
           (mode == ZDT_EMM_HOME_COLLISION) || (mode == ZDT_EMM_HOME_LIMIT) ||
           (mode == ZDT_EMM_HOME_ABSOLUTE_ZERO) || (mode == ZDT_EMM_HOME_POWER_LOSS_ANGLE);
}

static bool zdt_emm_unicast_addr_valid(uint8_t addr)
{
    return addr != ZDT_EMM_BROADCAST_ADDR;
}

static void zdt_emm_decode_status(uint8_t flags, zdt_emm_status_flags_t *status)
{
    status->enabled          = (flags & 0x01U) != 0U;
    status->reached          = (flags & 0x02U) != 0U;
    status->stall            = (flags & 0x04U) != 0U;
    status->stall_protect    = (flags & 0x08U) != 0U;
    status->left_limit_high  = (flags & 0x10U) != 0U;
    status->right_limit_high = (flags & 0x20U) != 0U;
    status->power_loss       = (flags & 0x80U) != 0U;
}

static void zdt_emm_decode_home_status(uint8_t flags, zdt_emm_home_flags_t *status)
{
    status->encoder_ready     = (flags & 0x01U) != 0U;
    status->calibration_ready = (flags & 0x02U) != 0U;
    status->homing            = (flags & 0x04U) != 0U;
    status->homing_failed     = (flags & 0x08U) != 0U;
    status->over_temperature  = (flags & 0x10U) != 0U;
    status->over_current      = (flags & 0x20U) != 0U;
}

static zdt_emm_result_t zdt_emm_read_request(UART_Regs *uart, uint8_t addr, uint8_t code,
    uint8_t *reply, uint8_t reply_len, uint32_t timeout_us)
{
    uint8_t frame[3] = { addr, code, 0U };

    if (!zdt_emm_unicast_addr_valid(addr)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_clear_rx(uart);
    zdt_emm_send_frame(uart, frame, sizeof(frame));
    if (!zdt_emm_read(uart, reply, reply_len, timeout_us)) {
        return ZDT_EMM_RESULT_TIMEOUT;
    }
    if ((reply[0] != addr) || (reply[1] != code) ||
        (reply[reply_len - 1U] != ZDT_EMM_DEFAULT_CHECK)) {
        return ZDT_EMM_RESULT_BAD_FRAME;
    }

    return ZDT_EMM_RESULT_OK;
}

static zdt_emm_result_t zdt_emm_read_u16_value(UART_Regs *uart, uint8_t addr, uint8_t code,
    uint16_t *value, uint32_t timeout_us)
{
    uint8_t reply[5];
    zdt_emm_result_t result;

    if (value == NULL) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    result = zdt_emm_read_request(uart, addr, code, reply, sizeof(reply), timeout_us);
    if (result == ZDT_EMM_RESULT_OK) {
        *value = zdt_emm_get_u16(&reply[2]);
    }
    return result;
}

static zdt_emm_result_t zdt_emm_read_signed_u32(UART_Regs *uart, uint8_t addr, uint8_t code,
    int64_t *value, uint32_t timeout_us)
{
    uint8_t reply[8];
    uint32_t magnitude;
    zdt_emm_result_t result;

    if (value == NULL) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    result = zdt_emm_read_request(uart, addr, code, reply, sizeof(reply), timeout_us);
    if (result != ZDT_EMM_RESULT_OK) {
        return result;
    }
    if (reply[2] > 0x01U) {
        return ZDT_EMM_RESULT_BAD_FRAME;
    }

    magnitude = zdt_emm_get_u32(&reply[3]);
    *value    = (reply[2] == 0x01U) ? -(int64_t) magnitude : (int64_t) magnitude;
    return ZDT_EMM_RESULT_OK;
}

static zdt_emm_result_t zdt_emm_action_with_aux(UART_Regs *uart, uint8_t addr, uint8_t code,
    uint8_t aux, uint32_t timeout_us)
{
    uint8_t frame[4] = { addr, code, aux, 0U };

    if (!zdt_emm_unicast_addr_valid(addr)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_clear_rx(uart);
    zdt_emm_send_frame(uart, frame, sizeof(frame));
    return zdt_emm_wait_reply(uart, addr, code, NULL, timeout_us);
}

void zdt_emm_clear_rx(UART_Regs *uart)
{
    while (!DL_UART_Main_isRXFIFOEmpty(uart)) {
        (void) DL_UART_Main_receiveData(uart);
    }
}

void zdt_emm_write(UART_Regs *uart, const uint8_t *data, uint8_t len)
{
    uint8_t i;

    for (i = 0; i < len; i++) {
        DL_UART_Main_transmitDataBlocking(uart, data[i]);
    }
}

bool zdt_emm_read_byte(UART_Regs *uart, uint8_t *data, uint32_t timeout_us)
{
    while (timeout_us > 0U) {
        if (!DL_UART_Main_isRXFIFOEmpty(uart)) {
            *data = DL_UART_Main_receiveData(uart);
            return true;
        }

        delay_cycles(CPUCLK_FREQ / 1000000U);
        timeout_us--;
    }

    return false;
}

bool zdt_emm_read(UART_Regs *uart, uint8_t *data, uint8_t len, uint32_t timeout_us)
{
    uint8_t i;

    for (i = 0; i < len; i++) {
        if (!zdt_emm_read_byte(uart, &data[i], timeout_us)) {
            return false;
        }
    }

    return true;
}

zdt_emm_result_t zdt_emm_wait_reply(
    UART_Regs *uart, uint8_t addr, uint8_t code, uint8_t *status, uint32_t timeout_us)
{
    uint8_t reply[4];

    if ((addr == ZDT_EMM_BROADCAST_ADDR) && (code != ZDT_EMM_CMD_SYNC_START)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    if (!zdt_emm_read(uart, reply, sizeof(reply), timeout_us)) {
        return ZDT_EMM_RESULT_TIMEOUT;
    }

    if ((reply[3] != ZDT_EMM_DEFAULT_CHECK) || (reply[1] != code)) {
        return ZDT_EMM_RESULT_BAD_FRAME;
    }
    if ((addr != ZDT_EMM_BROADCAST_ADDR) && (reply[0] != addr)) {
        return ZDT_EMM_RESULT_BAD_FRAME;
    }

    if (status != NULL) {
        *status = reply[2];
    }

    return zdt_emm_status_to_result(reply[2]);
}

zdt_emm_result_t zdt_emm_enable(
    UART_Regs *uart, uint8_t addr, bool enable, zdt_emm_sync_t sync, uint32_t timeout_us)
{
    zdt_emm_result_t result;

    if (!zdt_emm_unicast_addr_valid(addr)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_clear_rx(uart);
    result = zdt_emm_enable_no_reply(uart, addr, enable, sync);
    if (result != ZDT_EMM_RESULT_OK) {
        return result;
    }

    return zdt_emm_wait_reply(uart, addr, ZDT_EMM_CMD_ENABLE, NULL, timeout_us);
}

zdt_emm_result_t zdt_emm_enable_no_reply(
    UART_Regs *uart, uint8_t addr, bool enable, zdt_emm_sync_t sync)
{
    uint8_t frame[6] = {
        addr, ZDT_EMM_CMD_ENABLE, ZDT_EMM_AUX_ENABLE, enable ? 0x01U : 0x00U,
        (uint8_t) sync, 0U
    };

    if (!zdt_emm_sync_valid(sync)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_send_frame(uart, frame, sizeof(frame));
    return ZDT_EMM_RESULT_OK;
}

zdt_emm_result_t zdt_emm_speed(
    UART_Regs *uart, uint8_t addr, zdt_emm_dir_t dir, uint16_t rpm, uint8_t acc,
    zdt_emm_sync_t sync, uint32_t timeout_us)
{
    zdt_emm_result_t result;

    if (!zdt_emm_unicast_addr_valid(addr)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_clear_rx(uart);
    result = zdt_emm_speed_no_reply(uart, addr, dir, rpm, acc, sync);
    if (result != ZDT_EMM_RESULT_OK) {
        return result;
    }

    return zdt_emm_wait_reply(uart, addr, ZDT_EMM_CMD_SPEED, NULL, timeout_us);
}

zdt_emm_result_t zdt_emm_speed_no_reply(
    UART_Regs *uart, uint8_t addr, zdt_emm_dir_t dir, uint16_t rpm, uint8_t acc,
    zdt_emm_sync_t sync)
{
    uint8_t frame[8] = {
        addr, ZDT_EMM_CMD_SPEED, (uint8_t) dir, 0U, 0U, acc, (uint8_t) sync, 0U
    };

    if ((rpm > ZDT_EMM_MAX_RPM) || !zdt_emm_dir_valid(dir) || !zdt_emm_sync_valid(sync)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_put_u16(&frame[3], rpm);
    zdt_emm_send_frame(uart, frame, sizeof(frame));
    return ZDT_EMM_RESULT_OK;
}

zdt_emm_result_t zdt_emm_position_pulses(
    UART_Regs *uart, uint8_t addr, zdt_emm_dir_t dir, uint16_t rpm, uint8_t acc,
    uint32_t pulses, zdt_emm_move_mode_t mode, zdt_emm_sync_t sync, uint32_t timeout_us)
{
    zdt_emm_result_t result;

    if (!zdt_emm_unicast_addr_valid(addr)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_clear_rx(uart);
    result = zdt_emm_position_pulses_no_reply(uart, addr, dir, rpm, acc, pulses, mode, sync);
    if (result != ZDT_EMM_RESULT_OK) {
        return result;
    }

    return zdt_emm_wait_reply(uart, addr, ZDT_EMM_CMD_POSITION, NULL, timeout_us);
}

zdt_emm_result_t zdt_emm_position_pulses_no_reply(
    UART_Regs *uart, uint8_t addr, zdt_emm_dir_t dir, uint16_t rpm, uint8_t acc,
    uint32_t pulses, zdt_emm_move_mode_t mode, zdt_emm_sync_t sync)
{
    uint8_t frame[13] = {
        addr, ZDT_EMM_CMD_POSITION, (uint8_t) dir, 0U, 0U, acc, 0U, 0U, 0U, 0U, (uint8_t) mode,
        (uint8_t) sync, 0U
    };

    if ((rpm > ZDT_EMM_MAX_RPM) || !zdt_emm_dir_valid(dir) || !zdt_emm_move_mode_valid(mode) ||
        !zdt_emm_sync_valid(sync)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_put_u16(&frame[3], rpm);
    zdt_emm_put_u32(&frame[6], pulses);
    zdt_emm_send_frame(uart, frame, sizeof(frame));
    return ZDT_EMM_RESULT_OK;
}

zdt_emm_result_t zdt_emm_quick_position_config(
    UART_Regs *uart, uint8_t addr, uint16_t rpm, uint8_t acc, zdt_emm_move_mode_t mode,
    zdt_emm_sync_t sync, uint32_t timeout_us)
{
    zdt_emm_result_t result;

    if (!zdt_emm_unicast_addr_valid(addr)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_clear_rx(uart);
    result = zdt_emm_quick_position_config_no_reply(uart, addr, rpm, acc, mode, sync);
    if (result != ZDT_EMM_RESULT_OK) {
        return result;
    }

    return zdt_emm_wait_reply(uart, addr, ZDT_EMM_CMD_QUICK_POS_CONFIG, NULL, timeout_us);
}

zdt_emm_result_t zdt_emm_quick_position_config_no_reply(
    UART_Regs *uart, uint8_t addr, uint16_t rpm, uint8_t acc, zdt_emm_move_mode_t mode,
    zdt_emm_sync_t sync)
{
    uint8_t frame[8] = {
        addr, ZDT_EMM_CMD_QUICK_POS_CONFIG, 0U, 0U, acc, (uint8_t) mode, (uint8_t) sync, 0U
    };

    if ((rpm > ZDT_EMM_MAX_RPM) || !zdt_emm_move_mode_valid(mode) || !zdt_emm_sync_valid(sync)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_put_u16(&frame[2], rpm);
    zdt_emm_send_frame(uart, frame, sizeof(frame));
    return ZDT_EMM_RESULT_OK;
}

zdt_emm_result_t zdt_emm_quick_position_run(
    UART_Regs *uart, uint8_t addr, int32_t pulses, uint32_t timeout_us)
{
    zdt_emm_result_t result;

    if (!zdt_emm_unicast_addr_valid(addr)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_clear_rx(uart);
    result = zdt_emm_quick_position_run_no_reply(uart, addr, pulses);
    if (result != ZDT_EMM_RESULT_OK) {
        return result;
    }

    return zdt_emm_wait_reply(uart, addr, ZDT_EMM_CMD_QUICK_POS_RUN, NULL, timeout_us);
}

zdt_emm_result_t zdt_emm_quick_position_run_no_reply(
    UART_Regs *uart, uint8_t addr, int32_t pulses)
{
    uint8_t frame[7] = { addr, ZDT_EMM_CMD_QUICK_POS_RUN, 0U, 0U, 0U, 0U, 0U };

    zdt_emm_put_u32(&frame[2], (uint32_t) pulses);
    zdt_emm_send_frame(uart, frame, sizeof(frame));
    return ZDT_EMM_RESULT_OK;
}

zdt_emm_result_t zdt_emm_stop(
    UART_Regs *uart, uint8_t addr, zdt_emm_sync_t sync, uint32_t timeout_us)
{
    zdt_emm_result_t result;

    if (!zdt_emm_unicast_addr_valid(addr)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_clear_rx(uart);
    result = zdt_emm_stop_no_reply(uart, addr, sync);
    if (result != ZDT_EMM_RESULT_OK) {
        return result;
    }

    return zdt_emm_wait_reply(uart, addr, ZDT_EMM_CMD_STOP, NULL, timeout_us);
}

zdt_emm_result_t zdt_emm_stop_no_reply(UART_Regs *uart, uint8_t addr, zdt_emm_sync_t sync)
{
    uint8_t frame[5] = { addr, ZDT_EMM_CMD_STOP, ZDT_EMM_AUX_STOP, (uint8_t) sync, 0U };

    if (!zdt_emm_sync_valid(sync)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_send_frame(uart, frame, sizeof(frame));
    return ZDT_EMM_RESULT_OK;
}

zdt_emm_result_t zdt_emm_home(
    UART_Regs *uart, uint8_t addr, zdt_emm_home_mode_t mode, zdt_emm_sync_t sync,
    uint32_t timeout_us)
{
    zdt_emm_result_t result;

    if (!zdt_emm_unicast_addr_valid(addr)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_clear_rx(uart);
    result = zdt_emm_home_no_reply(uart, addr, mode, sync);
    if (result != ZDT_EMM_RESULT_OK) {
        return result;
    }

    return zdt_emm_wait_reply(uart, addr, ZDT_EMM_CMD_HOME, NULL, timeout_us);
}

zdt_emm_result_t zdt_emm_home_no_reply(
    UART_Regs *uart, uint8_t addr, zdt_emm_home_mode_t mode, zdt_emm_sync_t sync)
{
    uint8_t frame[5] = { addr, ZDT_EMM_CMD_HOME, (uint8_t) mode, (uint8_t) sync, 0U };

    if (!zdt_emm_home_mode_valid(mode) || !zdt_emm_sync_valid(sync)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_send_frame(uart, frame, sizeof(frame));
    return ZDT_EMM_RESULT_OK;
}

zdt_emm_result_t zdt_emm_home_abort(UART_Regs *uart, uint8_t addr, uint32_t timeout_us)
{
    if (!zdt_emm_unicast_addr_valid(addr)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_clear_rx(uart);
    zdt_emm_home_abort_no_reply(uart, addr);
    return zdt_emm_wait_reply(uart, addr, ZDT_EMM_CMD_HOME_ABORT, NULL, timeout_us);
}

void zdt_emm_home_abort_no_reply(UART_Regs *uart, uint8_t addr)
{
    uint8_t frame[4] = { addr, ZDT_EMM_CMD_HOME_ABORT, ZDT_EMM_AUX_HOME_ABORT, 0U };

    zdt_emm_send_frame(uart, frame, sizeof(frame));
}

zdt_emm_result_t zdt_emm_sync_start(UART_Regs *uart, uint32_t timeout_us)
{
    zdt_emm_clear_rx(uart);
    zdt_emm_sync_start_no_reply(uart);
    return zdt_emm_wait_reply(uart, ZDT_EMM_BROADCAST_ADDR, ZDT_EMM_CMD_SYNC_START, NULL, timeout_us);
}

void zdt_emm_sync_start_no_reply(UART_Regs *uart)
{
    uint8_t frame[4] = {
        ZDT_EMM_BROADCAST_ADDR, ZDT_EMM_CMD_SYNC_START, ZDT_EMM_AUX_SYNC_START, 0U
    };

    zdt_emm_send_frame(uart, frame, sizeof(frame));
}

zdt_emm_result_t zdt_emm_zero_position(UART_Regs *uart, uint8_t addr, uint32_t timeout_us)
{
    if (!zdt_emm_unicast_addr_valid(addr)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_clear_rx(uart);
    zdt_emm_zero_position_no_reply(uart, addr);
    return zdt_emm_wait_reply(uart, addr, ZDT_EMM_CMD_ZERO_POSITION, NULL, timeout_us);
}

void zdt_emm_zero_position_no_reply(UART_Regs *uart, uint8_t addr)
{
    uint8_t frame[4] = { addr, ZDT_EMM_CMD_ZERO_POSITION, ZDT_EMM_AUX_ZERO_POSITION, 0U };

    zdt_emm_send_frame(uart, frame, sizeof(frame));
}

zdt_emm_result_t zdt_emm_clear_protection(
    UART_Regs *uart, uint8_t addr, uint32_t timeout_us)
{
    if (!zdt_emm_unicast_addr_valid(addr)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_clear_rx(uart);
    zdt_emm_clear_protection_no_reply(uart, addr);
    return zdt_emm_wait_reply(uart, addr, ZDT_EMM_CMD_CLEAR_PROTECTION, NULL, timeout_us);
}

void zdt_emm_clear_protection_no_reply(UART_Regs *uart, uint8_t addr)
{
    uint8_t frame[4] = {
        addr, ZDT_EMM_CMD_CLEAR_PROTECTION, ZDT_EMM_AUX_CLEAR_PROTECTION, 0U
    };

    zdt_emm_send_frame(uart, frame, sizeof(frame));
}

zdt_emm_result_t zdt_emm_calibrate_encoder(
    UART_Regs *uart, uint8_t addr, uint32_t timeout_us)
{
    return zdt_emm_action_with_aux(uart, addr, ZDT_EMM_CMD_CALIBRATE_ENCODER,
        ZDT_EMM_AUX_CALIBRATE_ENCODER, timeout_us);
}

zdt_emm_result_t zdt_emm_restart(UART_Regs *uart, uint8_t addr, uint32_t timeout_us)
{
    return zdt_emm_action_with_aux(
        uart, addr, ZDT_EMM_CMD_RESTART, ZDT_EMM_AUX_RESTART, timeout_us);
}

zdt_emm_result_t zdt_emm_factory_reset(
    UART_Regs *uart, uint8_t addr, uint32_t timeout_us)
{
    return zdt_emm_action_with_aux(uart, addr, ZDT_EMM_CMD_FACTORY_RESET,
        ZDT_EMM_AUX_FACTORY_RESET, timeout_us);
}

zdt_emm_result_t zdt_emm_set_home_zero(
    UART_Regs *uart, uint8_t addr, bool store, uint32_t timeout_us)
{
    uint8_t frame[5] = {
        addr, ZDT_EMM_CMD_SET_HOME_ZERO, ZDT_EMM_AUX_SET_HOME_ZERO,
        store ? 0x01U : 0x00U, 0U
    };

    if (!zdt_emm_unicast_addr_valid(addr)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    zdt_emm_clear_rx(uart);
    zdt_emm_send_frame(uart, frame, sizeof(frame));
    return zdt_emm_wait_reply(uart, addr, ZDT_EMM_CMD_SET_HOME_ZERO, NULL, timeout_us);
}

zdt_emm_result_t zdt_emm_read_status(
    UART_Regs *uart, uint8_t addr, zdt_emm_status_flags_t *status, uint32_t timeout_us)
{
    uint8_t reply[4];
    zdt_emm_result_t result;

    if (status == NULL) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    result = zdt_emm_read_request(
        uart, addr, ZDT_EMM_CMD_READ_STATUS, reply, sizeof(reply), timeout_us);
    if (result == ZDT_EMM_RESULT_OK) {
        zdt_emm_decode_status(reply[2], status);
    }
    return result;
}

zdt_emm_result_t zdt_emm_read_home_status(
    UART_Regs *uart, uint8_t addr, zdt_emm_home_flags_t *status, uint32_t timeout_us)
{
    uint8_t reply[4];
    zdt_emm_result_t result;

    if (status == NULL) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    result = zdt_emm_read_request(
        uart, addr, ZDT_EMM_CMD_READ_HOME_STATUS, reply, sizeof(reply), timeout_us);
    if (result == ZDT_EMM_RESULT_OK) {
        zdt_emm_decode_home_status(reply[2], status);
    }
    return result;
}

zdt_emm_result_t zdt_emm_read_speed_rpm(
    UART_Regs *uart, uint8_t addr, int32_t *rpm, uint32_t timeout_us)
{
    uint8_t reply[6];
    uint16_t value;
    zdt_emm_result_t result;

    if (rpm == NULL) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    result = zdt_emm_read_request(
        uart, addr, ZDT_EMM_CMD_READ_SPEED, reply, sizeof(reply), timeout_us);
    if (result != ZDT_EMM_RESULT_OK) {
        return result;
    }
    if (reply[2] > 0x01U) {
        return ZDT_EMM_RESULT_BAD_FRAME;
    }

    value = zdt_emm_get_u16(&reply[3]);
    *rpm  = (reply[2] == 0x01U) ? -(int32_t) value : (int32_t) value;

    return ZDT_EMM_RESULT_OK;
}

zdt_emm_result_t zdt_emm_read_position_raw(
    UART_Regs *uart, uint8_t addr, int64_t *position, uint32_t timeout_us)
{
    return zdt_emm_read_signed_u32(
        uart, addr, ZDT_EMM_CMD_READ_POSITION, position, timeout_us);
}

zdt_emm_result_t zdt_emm_read_bus_voltage_mv(
    UART_Regs *uart, uint8_t addr, uint16_t *voltage_mv, uint32_t timeout_us)
{
    return zdt_emm_read_u16_value(
        uart, addr, ZDT_EMM_CMD_READ_VOLTAGE, voltage_mv, timeout_us);
}

zdt_emm_result_t zdt_emm_read_version(
    UART_Regs *uart, uint8_t addr, zdt_emm_version_t *version, uint32_t timeout_us)
{
    uint8_t reply[7];
    zdt_emm_result_t result;

    if (version == NULL) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    result = zdt_emm_read_request(
        uart, addr, ZDT_EMM_CMD_READ_VERSION, reply, sizeof(reply), timeout_us);
    if (result != ZDT_EMM_RESULT_OK) {
        return result;
    }

    version->firmware_version = zdt_emm_get_u16(&reply[2]);
    version->hardware_series  = (uint8_t) (reply[4] >> 4);
    version->hardware_type    = reply[4] & 0x0FU;
    version->hardware_version = reply[5];
    return ZDT_EMM_RESULT_OK;
}

zdt_emm_result_t zdt_emm_read_phase_parameters(UART_Regs *uart, uint8_t addr,
    zdt_emm_phase_parameters_t *parameters, uint32_t timeout_us)
{
    uint8_t reply[7];
    zdt_emm_result_t result;

    if (parameters == NULL) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    result = zdt_emm_read_request(
        uart, addr, ZDT_EMM_CMD_READ_PHASE_RL, reply, sizeof(reply), timeout_us);
    if (result != ZDT_EMM_RESULT_OK) {
        return result;
    }

    parameters->resistance_milliohm = zdt_emm_get_u16(&reply[2]);
    parameters->inductance_uh       = zdt_emm_get_u16(&reply[4]);
    return ZDT_EMM_RESULT_OK;
}

zdt_emm_result_t zdt_emm_read_bus_current_ma(
    UART_Regs *uart, uint8_t addr, uint16_t *current_ma, uint32_t timeout_us)
{
    return zdt_emm_read_u16_value(
        uart, addr, ZDT_EMM_CMD_READ_BUS_CURRENT, current_ma, timeout_us);
}

zdt_emm_result_t zdt_emm_read_phase_current_ma(
    UART_Regs *uart, uint8_t addr, uint16_t *current_ma, uint32_t timeout_us)
{
    return zdt_emm_read_u16_value(
        uart, addr, ZDT_EMM_CMD_READ_PHASE_CURRENT, current_ma, timeout_us);
}

zdt_emm_result_t zdt_emm_read_encoder_raw(
    UART_Regs *uart, uint8_t addr, uint16_t *encoder, uint32_t timeout_us)
{
    return zdt_emm_read_u16_value(
        uart, addr, ZDT_EMM_CMD_READ_ENCODER, encoder, timeout_us);
}

zdt_emm_result_t zdt_emm_read_input_pulses(
    UART_Regs *uart, uint8_t addr, int64_t *pulses, uint32_t timeout_us)
{
    return zdt_emm_read_signed_u32(
        uart, addr, ZDT_EMM_CMD_READ_INPUT_PULSES, pulses, timeout_us);
}

zdt_emm_result_t zdt_emm_read_target_position_raw(
    UART_Regs *uart, uint8_t addr, int64_t *position, uint32_t timeout_us)
{
    return zdt_emm_read_signed_u32(
        uart, addr, ZDT_EMM_CMD_READ_TARGET, position, timeout_us);
}

zdt_emm_result_t zdt_emm_read_set_target_position_raw(
    UART_Regs *uart, uint8_t addr, int64_t *position, uint32_t timeout_us)
{
    return zdt_emm_read_signed_u32(
        uart, addr, ZDT_EMM_CMD_READ_SET_TARGET, position, timeout_us);
}

zdt_emm_result_t zdt_emm_read_temperature_c(
    UART_Regs *uart, uint8_t addr, int16_t *temperature_c, uint32_t timeout_us)
{
    uint8_t reply[5];
    zdt_emm_result_t result;

    if (temperature_c == NULL) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    result = zdt_emm_read_request(
        uart, addr, ZDT_EMM_CMD_READ_TEMPERATURE, reply, sizeof(reply), timeout_us);
    if (result != ZDT_EMM_RESULT_OK) {
        return result;
    }
    if (reply[2] > 0x01U) {
        return ZDT_EMM_RESULT_BAD_FRAME;
    }

    /*
     * 手册 5.5.12 的符号说明与其他命令矛盾；0x43 示例明确以 00 22 表示 +34℃。
     * 这里按全协议一致的 00=正、01=负解释。
     */
    *temperature_c = (reply[2] == 0x01U) ? -(int16_t) reply[3] : (int16_t) reply[3];
    return ZDT_EMM_RESULT_OK;
}

zdt_emm_result_t zdt_emm_read_position_error_raw(
    UART_Regs *uart, uint8_t addr, int64_t *error, uint32_t timeout_us)
{
    return zdt_emm_read_signed_u32(
        uart, addr, ZDT_EMM_CMD_READ_POSITION_ERROR, error, timeout_us);
}

zdt_emm_result_t zdt_emm_read_all_status(UART_Regs *uart, uint8_t addr,
    zdt_emm_home_flags_t *home_status, zdt_emm_status_flags_t *motor_status,
    uint32_t timeout_us)
{
    uint8_t reply[5];
    zdt_emm_result_t result;

    if ((home_status == NULL) || (motor_status == NULL)) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    result = zdt_emm_read_request(
        uart, addr, ZDT_EMM_CMD_READ_ALL_STATUS, reply, sizeof(reply), timeout_us);
    if (result == ZDT_EMM_RESULT_OK) {
        zdt_emm_decode_home_status(reply[2], home_status);
        zdt_emm_decode_status(reply[3], motor_status);
    }
    return result;
}

zdt_emm_result_t zdt_emm_read_io_status(
    UART_Regs *uart, uint8_t addr, zdt_emm_io_status_t *status, uint32_t timeout_us)
{
    uint8_t reply[4];
    uint8_t flags;
    zdt_emm_result_t result;

    if (status == NULL) {
        return ZDT_EMM_RESULT_BAD_PARAM;
    }

    result = zdt_emm_read_request(
        uart, addr, ZDT_EMM_CMD_READ_IO_STATUS, reply, sizeof(reply), timeout_us);
    if (result != ZDT_EMM_RESULT_OK) {
        return result;
    }

    flags                       = reply[2];
    status->enable_pin_high     = (flags & 0x01U) != 0U;
    status->step_pin_high       = (flags & 0x04U) != 0U;
    status->direction_pin_high  = (flags & 0x10U) != 0U;
    status->direction_output    = (flags & 0x20U) != 0U;
    return ZDT_EMM_RESULT_OK;
}

uint32_t zdt_emm_degrees_to_pulses_x10(uint32_t degrees_x10, uint16_t pulses_per_rev)
{
    if (pulses_per_rev == 0U) {
        return 0U;
    }

    return (uint32_t) ((((uint64_t) degrees_x10 * pulses_per_rev) + 1800U) / 3600U);
}

