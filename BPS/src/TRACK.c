#include "TRACK.h"

/* ==================== 模块级静态状态 ==================== */
static TRACK_SAMPLE g_track_last_sample;

/* 稳定滤波：每路独立滞回计数 */
#define TRACK_FILTER_COUNT_MAX  8U
#define TRACK_FILTER_ON_COUNT   6U
#define TRACK_FILTER_OFF_COUNT  2U
static uint8_t g_track_filter_count[6];
static uint8_t g_track_stable_bits;

/* 6路传感器 pin mask（用于单次原子读取） */
#define TRACK_ALL_PINS_MASK \
    (TRACKING_PIN_1_PIN | TRACKING_PIN_2_PIN | TRACKING_PIN_3_PIN | \
     TRACKING_PIN_4_PIN | TRACKING_PIN_5_PIN | TRACKING_PIN_6_PIN)

/* ==================== 内部辅助函数 ==================== */

/*
 * 从单次 GPIO 端口读取值中提取 6 路原始位。
 * 高电平有效(TRACK_ACTIVE_LOW=0)：pin 为高 → 检测到黑线。
 */
static uint8_t Track_RawBitsFromPort(uint32_t port_value)
{
    uint8_t bits = 0;
    if (port_value & TRACKING_PIN_1_PIN) bits |= (1U << 0);
    if (port_value & TRACKING_PIN_2_PIN) bits |= (1U << 1);
    if (port_value & TRACKING_PIN_3_PIN) bits |= (1U << 2);
    if (port_value & TRACKING_PIN_4_PIN) bits |= (1U << 3);
    if (port_value & TRACKING_PIN_5_PIN) bits |= (1U << 4);
    if (port_value & TRACKING_PIN_6_PIN) bits |= (1U << 5);
    return bits;
}

/*
 * 根据 RawBits 和 ActiveBits 填充 TRACK_SAMPLE。
 * PositionX10 = 加权平均中心 * 10，丢线时为 0。
 */
static void Track_SampleFromBits(TRACK_SAMPLE *sample)
{
    uint8_t i;
    uint8_t count = 0;
    uint16_t sum = 0;

    for (i = 0; i < 6; i++)
    {
        if (sample->ActiveBits & (uint8_t)(1U << i))
        {
            sum = (uint16_t)(sum + (uint16_t)(i + 1U) * 10U);
            count++;
        }
    }
    sample->ActiveCount = count;
    if (count == 0)
    {
        sample->PositionX10 = 0;
    }
    else
    {
        sample->PositionX10 = (uint8_t)(sum / count);
    }
}

/* ==================== 20s 模式：直接读取 ==================== */

/*
 * 单次原子读取全部 6 路 GPIO，返回原始采样。
 * 这是 20s 无球模式的主接口。
 */
TRACK_SAMPLE Track_ReadRaw(void)
{
    TRACK_SAMPLE sample;
    uint32_t port_value;

    port_value = DL_GPIO_readPins(TRACKING_PORT, TRACK_ALL_PINS_MASK);
    sample.RawBits = Track_RawBitsFromPort(port_value);

#if TRACK_ACTIVE_LOW
    sample.ActiveBits = (uint8_t)(~sample.RawBits & TRACK_ALL_ACTIVE_BITS);
#else
    sample.ActiveBits = sample.RawBits;
#endif

    Track_SampleFromBits(&sample);
    g_track_last_sample = sample;
    return sample;
}

TRACK_SAMPLE Track_Read(void)
{
    return Track_ReadRaw();
}

/* ==================== 30s 模式：滞回滤波 ==================== */

/*
 * 每 1ms 调用一次。对每路传感器做独立滞回计数：
 *   - 当前 GPIO 为高 → 计数+1（最高到 MAX）
 *   - 当前 GPIO 为低 → 计数-1（最低到 0）
 *   - 计数 >= ON_COUNT  → 该路判定为有效
 *   - 计数 <= OFF_COUNT → 该路判定为无效
 * 中间状态保持上一拍结果，避免跳变。
 */
void Track_Update1ms(void)
{
    uint32_t port_value;
    uint8_t raw_bits;
    uint8_t i;

    port_value = DL_GPIO_readPins(TRACKING_PORT, TRACK_ALL_PINS_MASK);
    raw_bits = Track_RawBitsFromPort(port_value);

    for (i = 0; i < 6; i++)
    {
        uint8_t mask = (uint8_t)(1U << i);
        if (raw_bits & mask)
        {
            if (g_track_filter_count[i] < TRACK_FILTER_COUNT_MAX)
                g_track_filter_count[i]++;
        }
        else
        {
            if (g_track_filter_count[i] > 0)
                g_track_filter_count[i]--;
        }

        if (g_track_filter_count[i] >= TRACK_FILTER_ON_COUNT)
            g_track_stable_bits |= mask;
        else if (g_track_filter_count[i] <= TRACK_FILTER_OFF_COUNT)
            g_track_stable_bits &= (uint8_t)(~mask);
        /* 中间状态保持上一拍值不变 */
    }
}

/*
 * 返回经过滞回滤波后的稳定快照。
 * 调用前需确保 Track_Update1ms() 已在 1ms 周期运行。
 */
TRACK_SAMPLE Track_ReadStable(void)
{
    TRACK_SAMPLE sample;

    sample.RawBits = g_track_stable_bits;
#if TRACK_ACTIVE_LOW
    sample.ActiveBits = (uint8_t)(~g_track_stable_bits & TRACK_ALL_ACTIVE_BITS);
#else
    sample.ActiveBits = g_track_stable_bits;
#endif
    Track_SampleFromBits(&sample);
    g_track_last_sample = sample;
    return sample;
}

/*
 * 清零所有滞回滤波状态。
 */
void Track_ResetStable(void)
{
    uint8_t i;
    for (i = 0; i < 6; i++)
        g_track_filter_count[i] = 0;
    g_track_stable_bits = 0;
}

/* ==================== 向后兼容接口 ==================== */

/*
 * 返回当前黑线中心位置（10~60），丢线返回 0。
 * 内部调用 Track_ReadRaw()，保持与原接口相同语义。
 */
uint8_t Track_Get(void)
{
    return Track_ReadRaw().PositionX10;
}

/*
 * 返回 6 路传感器当前有效位：bit0..bit5，1=检测到黑线。
 */
uint8_t Track_GetBits(void)
{
    Track_ReadRaw();  /* 更新 g_track_last_sample */
    return g_track_last_sample.ActiveBits;
}
