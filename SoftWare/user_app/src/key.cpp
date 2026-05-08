/*********************************************************************************************************************
 * 文件名称：key.cpp
 * 功能描述：按键 + 拨码开关控制模块实现
 *
 * ─────────────────────────────────────────────────────────────────────
 * 按键状态机（每个按键独立维护一份）：
 *
 *   ┌────────────┐  电平=0       ┌────────────┐
 *   │  IDLE      │ ──────────►   │  PRESSED   │
 *   │ (未按下)   │  消抖通过      │ (已按下)   │
 *   └────────────┘                └────────────┘
 *         ▲                              │
 *         │                              │ 持续 ≥ 长按阈值
 *         │ 电平=1 + 消抖通过              │
 *         │                              ▼
 *         │                       ┌────────────┐
 *         │                       │ LONG_HOLD  │
 *         │ ◄─────────────────────│ (长按中)   │
 *         │                       └────────────┘
 *         │
 *      释放时根据 hold_count 判定事件：
 *        hold_count < LONG_PRESS_TICKS → 触发 SHORT
 *        hold_count ≥ LONG_PRESS_TICKS → 已经触发过 LONG，不再发 SHORT
 *
 *      双击检测：
 *        每次 SHORT 事件检查与上次 SHORT 的时间差
 *        差 ≤ DOUBLE_GAP_TICKS → 触发 DOUBLE，撤销上一次的 SHORT
 *
 * ─────────────────────────────────────────────────────────────────────
 * 与原版本的差异：
 *   1. 原版本 key_loop() 自动清零 flag，与文档矛盾。新版本严格按文档：业务清
 *   2. 原版本只有"边沿检测"，没消抖。新版本要求连续 N 次同电平才稳定
 *   3. 增加长按、双击事件
 *   4. 增加事件队列和回调机制
 ********************************************************************************************************************/

#include "key.h"
#include <stdio.h>
#include <string.h>

// ══════════════════════════════════════════════════════════════
// 硬件实例（延迟初始化）
// ══════════════════════════════════════════════════════════════
static ls_gpio *p_keys[KEY_NUM] = {nullptr, nullptr, nullptr};
static ls_gpio *p_boma1 = nullptr;
static ls_gpio *p_boma2 = nullptr;

// ══════════════════════════════════════════════════════════════
// 全局变量定义
// ══════════════════════════════════════════════════════════════

// 短按
uint8_t key1_flag = 0;
uint8_t key2_flag = 0;
uint8_t key3_flag = 0;

// 长按
uint8_t key1_long_flag = 0;
uint8_t key2_long_flag = 0;
uint8_t key3_long_flag = 0;

// 双击
uint8_t key1_double_flag = 0;
uint8_t key2_double_flag = 0;
uint8_t key3_double_flag = 0;

// 拨码
uint8_t key_mode = 0;
uint8_t key_mode_changed = 0;

// ══════════════════════════════════════════════════════════════
// 内部状态：按键状态机
// ══════════════════════════════════════════════════════════════
typedef enum
{
    KS_IDLE = 0,      // 未按下
    KS_DEBOUNCE = 1,  // 检测到按下，消抖中
    KS_PRESSED = 2,   // 已确认按下
    KS_LONG_HELD = 3, // 长按已触发（按住中）
    KS_RELEASING = 4  // 检测到释放，消抖中
} KeyState_t;

typedef struct
{
    KeyState_t state;
    uint16_t debounce_cnt; // 消抖计数
    uint16_t hold_cnt;     // 持续按下时长（tick 数）
    uint16_t gap_cnt;      // 距上次释放的间隔（用于双击判断）
    bool waiting_double;   // 上次刚 SHORT，正在等第二次按下
    bool long_fired;       // 长按事件是否已发出（防重复）
} KeyContext_t;

static KeyContext_t s_ctx[KEY_NUM] = {{KS_IDLE, 0, 0, 0, false, false}};

// ══════════════════════════════════════════════════════════════
// 内部状态：事件队列（环形缓冲）
// ══════════════════════════════════════════════════════════════
static KeyEvent_t s_evt_queue[KEY_EVENT_QUEUE_SIZE];
static int s_evt_head = 0; // 写入位置
static int s_evt_tail = 0; // 读取位置
static int s_evt_count = 0;

// ══════════════════════════════════════════════════════════════
// 内部状态：回调
// ══════════════════════════════════════════════════════════════
static key_event_cb_t s_short_cb[KEY_NUM] = {nullptr};
static key_event_cb_t s_long_cb[KEY_NUM] = {nullptr};
static key_event_cb_t s_double_cb[KEY_NUM] = {nullptr};

// 拨码上次值（用于变化检测）
static uint8_t s_last_mode = 0xFF; // 0xFF = 还没读过

// ══════════════════════════════════════════════════════════════
// 内部辅助：将事件加入队列
// ══════════════════════════════════════════════════════════════
static void enqueue_event(uint8_t key_idx, KeyEventType_t evt)
{
    if (s_evt_count >= KEY_EVENT_QUEUE_SIZE)
    {
        // 队列已满，丢弃最早的（保持最新事件）
        s_evt_tail = (s_evt_tail + 1) % KEY_EVENT_QUEUE_SIZE;
        s_evt_count--;
    }
    s_evt_queue[s_evt_head].key_idx = key_idx;
    s_evt_queue[s_evt_head].evt = evt;
    s_evt_head = (s_evt_head + 1) % KEY_EVENT_QUEUE_SIZE;
    s_evt_count++;
}

// ══════════════════════════════════════════════════════════════
// 内部辅助：触发事件（设标志位 + 入队 + 调回调）
// ══════════════════════════════════════════════════════════════
static void fire_event(uint8_t idx, KeyEventType_t evt)
{
    // 1. 设置对应标志位（向下兼容旧 API）
    if (evt == KEY_EVT_SHORT)
    {
        if (idx == KEY1_INDEX)
            key1_flag = 1;
        if (idx == KEY2_INDEX)
            key2_flag = 1;
        if (idx == KEY3_INDEX)
            key3_flag = 1;
    }
    else if (evt == KEY_EVT_LONG)
    {
        if (idx == KEY1_INDEX)
            key1_long_flag = 1;
        if (idx == KEY2_INDEX)
            key2_long_flag = 1;
        if (idx == KEY3_INDEX)
            key3_long_flag = 1;
    }
    else if (evt == KEY_EVT_DOUBLE)
    {
        if (idx == KEY1_INDEX)
            key1_double_flag = 1;
        if (idx == KEY2_INDEX)
            key2_double_flag = 1;
        if (idx == KEY3_INDEX)
            key3_double_flag = 1;
    }

    // 2. 入事件队列
    enqueue_event(idx, evt);

    // 3. 触发回调
    key_event_cb_t cb = nullptr;
    if (evt == KEY_EVT_SHORT)
        cb = s_short_cb[idx];
    else if (evt == KEY_EVT_LONG)
        cb = s_long_cb[idx];
    else if (evt == KEY_EVT_DOUBLE)
        cb = s_double_cb[idx];
    if (cb != nullptr)
        cb(idx);

    // 4. 调试打印
    const char *name = (idx == 0 ? "KEY1" : idx == 1 ? "KEY2"
                                                     : "KEY3");
    const char *etyp = (evt == KEY_EVT_SHORT ? "SHORT" : evt == KEY_EVT_LONG ? "LONG"
                                                     : evt == KEY_EVT_DOUBLE ? "DOUBLE"
                                                                             : "?");
    printf("[Key] %s %s\n", name, etyp);
}

// ══════════════════════════════════════════════════════════════
// 内部辅助：单个按键状态机更新
// 由 key_loop 每周期调用一次
// ══════════════════════════════════════════════════════════════
static void update_one_key(uint8_t idx, uint8_t level)
{
    KeyContext_t *c = &s_ctx[idx];

    // 双击间隔计数（不论按键状态，每周期 +1）
    if (c->waiting_double)
    {
        c->gap_cnt++;
        if (c->gap_cnt > KEY_DOUBLE_GAP_TICKS)
        {
            // 超过双击间隔，确认是单击：补发 SHORT 事件
            fire_event(idx, KEY_EVT_SHORT);
            c->waiting_double = false;
            c->gap_cnt = 0;
        }
    }

    switch (c->state)
    {
    // ── IDLE：未按下 ─────────────────────────────────────
    case KS_IDLE:
        if (level == 0) // 检测到按下信号
        {
            c->state = KS_DEBOUNCE;
            c->debounce_cnt = 1;
        }
        break;

    // ── DEBOUNCE：消抖中 ─────────────────────────────────
    case KS_DEBOUNCE:
        if (level == 0)
        {
            c->debounce_cnt++;
            if (c->debounce_cnt >= KEY_DEBOUNCE_TICKS)
            {
                // 确认按下，进入 PRESSED
                c->state = KS_PRESSED;
                c->hold_cnt = 0;
                c->long_fired = false;
            }
        }
        else
        {
            // 中途松开，是抖动，回到 IDLE
            c->state = KS_IDLE;
            c->debounce_cnt = 0;
        }
        break;

    // ── PRESSED：已按下，等待释放或长按 ──────────────────
    case KS_PRESSED:
        if (level == 0)
        {
            c->hold_cnt++;
            if (c->hold_cnt >= KEY_LONG_PRESS_TICKS && !c->long_fired)
            {
                // 达到长按阈值，立即触发 LONG（不等释放）
                fire_event(idx, KEY_EVT_LONG);
                c->long_fired = true;
                c->state = KS_LONG_HELD;
            }
        }
        else
        {
            // 释放：根据 hold_cnt 决定是 SHORT 还是双击候选
            if (c->waiting_double)
            {
                // 这是第二次按下且释放 → 双击
                fire_event(idx, KEY_EVT_DOUBLE);
                c->waiting_double = false;
                c->gap_cnt = 0;
            }
            else
            {
                // 第一次释放，先不发 SHORT，等可能的双击
                c->waiting_double = true;
                c->gap_cnt = 0;
            }
            c->state = KS_IDLE;
        }
        break;

    // ── LONG_HELD：长按已触发，等待释放 ──────────────────
    case KS_LONG_HELD:
        if (level == 1) // 释放
        {
            // 长按结束，不再发 SHORT 也不参与双击
            c->state = KS_IDLE;
            c->waiting_double = false;
            c->gap_cnt = 0;
        }
        break;

    default:
        c->state = KS_IDLE;
        break;
    }
}

// ══════════════════════════════════════════════════════════════
// 公共接口：初始化
// ══════════════════════════════════════════════════════════════
void key_init(void)
{
    if (p_keys[0] != nullptr)
        return;

    static ls_gpio inst_key1(KEY1_PIN, GPIO_MODE_IN);
    static ls_gpio inst_key2(KEY2_PIN, GPIO_MODE_IN);
    static ls_gpio inst_key3(KEY3_PIN, GPIO_MODE_IN);
    static ls_gpio inst_boma1(KEY_BOMA1_PIN, GPIO_MODE_IN);
    static ls_gpio inst_boma2(KEY_BOMA2_PIN, GPIO_MODE_IN);

    p_keys[0] = &inst_key1;
    p_keys[1] = &inst_key2;
    p_keys[2] = &inst_key3;
    p_boma1 = &inst_boma1;
    p_boma2 = &inst_boma2;

    // 清空状态机
    memset(s_ctx, 0, sizeof(s_ctx));

    // 清空事件队列
    s_evt_head = s_evt_tail = s_evt_count = 0;

    // 读一次拨码作为初始值
    key_boma();
    s_last_mode = key_mode;

    printf("[Key] Init done. mode=%d\n", key_mode);
}

// ══════════════════════════════════════════════════════════════
// 实时电平检测
// ══════════════════════════════════════════════════════════════
int8_t key_scan(void)
{
    if (p_keys[0] == nullptr)
        return -1;

    if (p_keys[0]->gpio_level_get() == 0)
        return 0;
    if (p_keys[1]->gpio_level_get() == 0)
        return 1;
    if (p_keys[2]->gpio_level_get() == 0)
        return 2;
    return -1;
}

// ══════════════════════════════════════════════════════════════
// 拨码开关读取
// ══════════════════════════════════════════════════════════════
uint8_t key_boma(void)
{
    if (p_boma1 == nullptr)
        return 0;

    uint8_t b1 = p_boma1->gpio_level_get();
    uint8_t b2 = p_boma2->gpio_level_get();
    key_mode = (uint8_t)((b1 << 1) | b2);
    return key_mode;
}

// ══════════════════════════════════════════════════════════════
// 按键扫描循环（100ms 调用一次）
// ══════════════════════════════════════════════════════════════
void key_loop(void)
{
    if (p_keys[0] == nullptr)
        return;

    // ── 1. 更新各按键状态机 ──────────────────────────────────
    for (uint8_t i = 0; i < KEY_NUM; i++)
    {
        uint8_t lvl = p_keys[i]->gpio_level_get();
        update_one_key(i, lvl);
    }

    // ── 2. 拨码变化检测 ─────────────────────────────────────
    uint8_t now_mode = key_boma();
    if (now_mode != s_last_mode)
    {
        printf("[Key] BOMA changed: %d → %d\n", s_last_mode, now_mode);
        key_mode_changed = 1;
        s_last_mode = now_mode;
    }
}

// ══════════════════════════════════════════════════════════════
// 事件队列
// ══════════════════════════════════════════════════════════════
bool key_get_event(KeyEvent_t *out_evt)
{
    if (out_evt == nullptr)
        return false;
    if (s_evt_count == 0)
        return false;

    *out_evt = s_evt_queue[s_evt_tail];
    s_evt_tail = (s_evt_tail + 1) % KEY_EVENT_QUEUE_SIZE;
    s_evt_count--;
    return true;
}

void key_clear_events(void)
{
    s_evt_head = s_evt_tail = s_evt_count = 0;
}

// ══════════════════════════════════════════════════════════════
// 回调注册
// ══════════════════════════════════════════════════════════════
void key_register_short_cb(uint8_t key_idx, key_event_cb_t cb)
{
    if (key_idx >= KEY_NUM)
        return;
    s_short_cb[key_idx] = cb;
}

void key_register_long_cb(uint8_t key_idx, key_event_cb_t cb)
{
    if (key_idx >= KEY_NUM)
        return;
    s_long_cb[key_idx] = cb;
}

void key_register_double_cb(uint8_t key_idx, key_event_cb_t cb)
{
    if (key_idx >= KEY_NUM)
        return;
    s_double_cb[key_idx] = cb;
}