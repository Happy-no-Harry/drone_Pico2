#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// ===================== 硬件配置（按实际接线修改）=====================
#define SERVO1_PIN 14       // S1舵机引脚
#define SERVO2_PIN 15       // S2舵机引脚
#define IR1_PIN    7      // 红外1
#define IR2_PIN    8      // 红外2
#define IR3_PIN    9      // 红外3
#define FLY_TRIGGER_PIN 16 // 飞控升空触发引脚
#define LED_D1 2       // LED（红）
#define LED_D2 3       // LED（黄）
#define LED_D3 4       // LED（蓝）
#define LED_D4 5       // LED（绿）
#define ONLY_IR3

// ===================== 舵机参数配置 =====================
// 常见舵机使用 50Hz PWM，0~180 度通常映射到 0.5ms~2.5ms 高电平脉宽。
#define SERVO_PWM_FREQ 50       // 标准50Hz
#define SERVO_PWM_WRAP 19999    // 20ms 对应 20000 计数（wrap+1）
#define SERVO_TARGET_ANGLE 90   // 取货时旋转角度
#define SERVO_RESET_ANGLE  0    // 投递时复位角度
#define SERVO_DELAY_MS 1000     // 舵机动作等待时间
#define SERVO_GRAB_GAP_MS 300   // 取货时两舵机动作间隔
#define SERVO_MIN_CMD_INTERVAL_MS 250   // 同一舵机两次命令最小间隔
#define SERVO_OVERFREQ_WINDOW_MS 1200   // 过频统计窗口
#define SERVO_OVERFREQ_LIMIT 5          // 窗口内最大允许动作次数
#define SERVO_PROTECT_COOLDOWN_MS 2000  // 触发保护后的冷却时间

// ===================== 红外参数配置 =====================
// 本项目约定：红外被遮挡时输出低电平。
#define OBSTACLE_LEVEL  0       // 有障碍物=低电平
#define CLEAR_LEVEL     1       // 无障碍物=高电平
#define IR_DEBOUNCE_MS  10      // 防抖时间

// ===================== 状态机定义 =====================
// 取货与投递按阶段推进，每次只处理一个状态，避免动作互相重叠。
typedef enum {
    STATE_IDLE,                 // 空闲：等待指令
    STATE_GRAB_CHECK,           // 取货检测：判断红外组合
    STATE_GRAB_ACTUATE,         // 取货动作：驱动舵机
    STATE_WAIT_ASCEND,          // 等待ascend指令
    STATE_GRAB_VERIFY,          // 抓取验证：检查IR3
    STATE_HOLDING,              // 持货等待
    STATE_RELEASE_ACTUATE,      // 投递动作：S1->S2
    STATE_WAIT_TAKEOFF,         // 等待takeoff指令
    STATE_RELEASE_VERIFY,       // 投递验证：检查全部红外
    STATE_ERROR
} SystemState;

// ===================== 全局变量 =====================
// 串口缓冲只做短命令接收（grab/ascend/release/takeoff）。
static SystemState current_state = STATE_IDLE;
static char uart_buf[64] = {0};
static uint8_t buf_idx = 0;
// 错误恢复计数器 - 避免ERROR状态无限循环
static uint8_t error_recovery_count = 0;
static const uint8_t ERROR_RECOVERY_MAX = 1;  // ERROR状态最多执行1次恢复

// ===================== 红外状态结构体 =====================
typedef struct {
    bool ir1;
    bool ir2;
    bool ir3;
} IRState;

// ===================== 舵机保护结构体 =====================
// 无电流传感器条件下的软件防堵转：
// 1) 过滤重复角度命令，避免无意义顶死输出；
// 2) 限制最小动作间隔，避免短时间频繁反复；
// 3) 过频后进入冷却，降低持续堵转风险。
typedef struct {
    uint8_t commanded_angle;
    absolute_time_t last_cmd_time;
    absolute_time_t window_start_time;
    absolute_time_t protect_until;
    uint8_t window_cmd_count;
    bool cooldown_notified;
    bool initialized;
    // 状态恢复机制 - 记录该舵机的失败重试状态
    uint8_t fail_retry_count;
} ServoGuard;

typedef enum {
    SERVO_ACTION_DONE,
    SERVO_ACTION_SKIPPED,
    SERVO_ACTION_BLOCKED
} ServoActionResult;

static ServoGuard servo1_guard = {0};
static ServoGuard servo2_guard = {0};
static absolute_time_t actuate_start = {0};  // 使用 {0} 初始化为nil_time
static absolute_time_t state_enter_time = {0};
static SystemState last_state_observed = (SystemState)-1;
static bool fly_pulse_active = false;
static absolute_time_t fly_pulse_until = {0};

// ===================== LED 灯效系统 =====================
// 若你的LED是低电平点亮，请改为 1。
#define LED_ACTIVE_LOW 0
#define LED_FX_TICK_MS 50

typedef struct {
    absolute_time_t last_tick;
    uint32_t tick;
    uint32_t lfsr;
    SystemState last_state;
} LedFxState;

static LedFxState led_fx = {0};

static inline void led_write_pin(uint pin, bool on) {
    gpio_put(pin, LED_ACTIVE_LOW ? !on : on);
}

static void led_apply_mask(uint8_t mask) {
    led_write_pin(LED_D1, (mask & 0x01u) != 0);
    led_write_pin(LED_D2, (mask & 0x02u) != 0);
    led_write_pin(LED_D3, (mask & 0x04u) != 0);
    led_write_pin(LED_D4, (mask & 0x08u) != 0);
}

static uint32_t led_rand32(void) {
    uint32_t x = led_fx.lfsr;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    led_fx.lfsr = x;
    return x;
}

void led_fx_init(void) {
    gpio_init(LED_D1); gpio_set_dir(LED_D1, GPIO_OUT);
    gpio_init(LED_D2); gpio_set_dir(LED_D2, GPIO_OUT);
    gpio_init(LED_D3); gpio_set_dir(LED_D3, GPIO_OUT);
    gpio_init(LED_D4); gpio_set_dir(LED_D4, GPIO_OUT);

    led_fx.last_tick = get_absolute_time();
    led_fx.tick = 0;
    led_fx.lfsr = 0xA5A55A5Au;
    led_fx.last_state = current_state;

    // 开机灯序：点亮扫描 + 全亮收束
    const uint8_t boot_seq[] = {0x1, 0x2, 0x4, 0x8, 0x4, 0x2, 0x1, 0xF, 0x0};
    for (uint i = 0; i < sizeof(boot_seq); i++) {
        led_apply_mask(boot_seq[i]);
        sleep_ms(55);
    }
}

void led_fx_update(void) {
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(led_fx.last_tick, now) < (int64_t)LED_FX_TICK_MS * 1000) {
        return;
    }
    led_fx.last_tick = now;

    if (led_fx.last_state != current_state) {
        led_fx.tick = 0;
        led_fx.last_state = current_state;
    }

    uint8_t mask = 0;
    switch (current_state) {
        case STATE_IDLE: {
            // 骑士流光：左右来回扫描
            const uint8_t seq[] = {0x1, 0x2, 0x4, 0x8, 0x4, 0x2};
            mask = seq[led_fx.tick % (sizeof(seq) / sizeof(seq[0]))];
            break;
        }
        case STATE_GRAB_CHECK:
        case STATE_GRAB_ACTUATE:
        case STATE_RELEASE_ACTUATE: {
            // 彗星推进：逐步拉满再熄灭
            const uint8_t seq[] = {0x1, 0x3, 0x7, 0xF, 0xE, 0xC, 0x8, 0x0};
            mask = seq[led_fx.tick % (sizeof(seq) / sizeof(seq[0]))];
            break;
        }
        case STATE_WAIT_ASCEND:
        case STATE_WAIT_TAKEOFF: {
            // 双闪心跳：提醒等待外部命令
            uint8_t phase = led_fx.tick % 20u;
            mask = (phase < 2u || (phase >= 5u && phase < 7u)) ? 0xFu : 0x0u;
            break;
        }
        case STATE_GRAB_VERIFY:
        case STATE_RELEASE_VERIFY: {
            // 交叉闪烁：验证阶段
            mask = (led_fx.tick & 1u) ? 0x5u : 0xAu;
            break;
        }
        case STATE_HOLDING: {
            // 持货巡航：常亮 + 细微抖动
            const uint8_t seq[] = {0xF, 0xF, 0xF, 0xF, 0x7, 0xF, 0xB, 0xF, 0xD, 0xF, 0xE, 0xF};
            mask = seq[led_fx.tick % (sizeof(seq) / sizeof(seq[0]))];
            break;
        }
        case STATE_ERROR: {
            // 错误态：爆闪 + 随机火花
            if ((led_fx.tick & 1u) == 0u) {
                mask = 0xFu;
            } else {
                uint32_t r = led_rand32();
                mask = (uint8_t)(r & 0xFu);
                if (mask == 0) {
                    mask = (uint8_t)(1u << ((r >> 4) & 0x3u));
                }
            }
            break;
        }
        default:
            mask = 0;
            break;
    }

    led_apply_mask(mask);
    led_fx.tick++;
}

static inline int64_t state_elapsed_ms(void) {
    if (is_nil_time(state_enter_time)) {
        return 0;
    }
    return absolute_time_diff_us(state_enter_time, get_absolute_time()) / 1000;
}

void fly_trigger_pulse_start(uint32_t pulse_ms) {
    gpio_put(FLY_TRIGGER_PIN, 1);
    fly_pulse_active = true;
    fly_pulse_until = delayed_by_ms(get_absolute_time(), pulse_ms);
}

void fly_trigger_update(void) {
    if (fly_pulse_active && absolute_time_diff_us(get_absolute_time(), fly_pulse_until) <= 0) {
        gpio_put(FLY_TRIGGER_PIN, 0);
        fly_pulse_active = false;
    }
}

// ===================== 舵机驱动函数 =====================
// 角度转 PWM 计数值：
// - PWM 时基配置为 1MHz（1 计数=1us）
// - 周期 20ms（wrap=19999）
// - 角度线性映射到 500us~2500us
uint16_t angle_to_pwm(uint8_t angle) {
    if (angle > 180) angle = 180;
    // 使用浮点数除法避免整数截断导致角度精度丢失
    float pulse_us = 500.0f + ((float)angle / 180.0f) * 2000.0f;
    float period_us = 1000000.0f / (float)SERVO_PWM_FREQ;
    uint32_t level = (uint32_t)((pulse_us / period_us) * (float)(SERVO_PWM_WRAP + 1u));
    if (level > SERVO_PWM_WRAP) level = SERVO_PWM_WRAP;
    return (uint16_t)level;
}

// ===================== 通用的安全驱动内核 =====================
// 返回值：
// - DONE: 已执行动作
// - SKIPPED: 目标角度与当前角度一致，属于正常空操作
// - BLOCKED: 被保护逻辑拦截
ServoActionResult servo_safe_set_internal(ServoGuard *guard, uint pin, uint8_t target_angle) {
    absolute_time_t now = get_absolute_time();

    // 1. 懒初始化
    if (!guard->initialized) {
        guard->commanded_angle = SERVO_RESET_ANGLE;
        guard->last_cmd_time = nil_time;
        guard->window_start_time = now;
        guard->protect_until = now;
        guard->window_cmd_count = 0;
        guard->cooldown_notified = false;
        guard->initialized = true;
    }

    // 2. 冷却期检查
    // absolute_time_diff_us(from, to) = to - from
    // 若 now < protect_until，则 (protect_until - now) > 0，表示仍在冷却期内
    if (absolute_time_diff_us(now, guard->protect_until) > 0) {
        if (!guard->cooldown_notified) {
            printf("[WARN] Servo locked. Cooldown remaining: %lld ms\r\n", 
                   absolute_time_diff_us(now, guard->protect_until) / 1000);
            guard->cooldown_notified = true;
        }
        printf("[WARN] Command to pin %d blocked due to cooldown.\r\n", pin);
        return SERVO_ACTION_BLOCKED;
    }
    guard->cooldown_notified = false;

    // 3. 相同指令去重
    if (target_angle == guard->commanded_angle) {
        return SERVO_ACTION_SKIPPED;
    }

    // 4. 最小间隔限制 (如果太快，直接拒绝，不做 sleep 阻塞)
    if (!is_nil_time(guard->last_cmd_time)) {
        int64_t dt_us = absolute_time_diff_us(guard->last_cmd_time, now);
        if (dt_us < (int64_t)SERVO_MIN_CMD_INTERVAL_MS * 1000) {
            printf("[WARN] Command to pin %d blocked due to minimum interval. Time since last command: %lld ms\r\n", 
                   pin, dt_us / 1000);
            return SERVO_ACTION_BLOCKED;
        }
    }

    // 5. 滑动窗口过频保护
    int64_t win_us = absolute_time_diff_us(guard->window_start_time, now);
    if (win_us > (int64_t)SERVO_OVERFREQ_WINDOW_MS * 1000) {
        guard->window_start_time = now;
        guard->window_cmd_count = 0;
    }
    
    guard->window_cmd_count++;
    if (guard->window_cmd_count > SERVO_OVERFREQ_LIMIT) {
        guard->protect_until = delayed_by_ms(now, SERVO_PROTECT_COOLDOWN_MS);
        printf("[ERROR] Servo overheated! Locking for %d ms.\r\n", SERVO_PROTECT_COOLDOWN_MS);
        return SERVO_ACTION_BLOCKED;
    }

    // 6. 执行动作 (这里只发令，不 sleep!)
    pwm_set_gpio_level(pin, angle_to_pwm(target_angle));
    
    // 7. 更新状态
    guard->commanded_angle = target_angle;
    guard->last_cmd_time = now;
    
    printf("[SERVO] Set to %d deg\r\n", target_angle);
    // 移除阻塞sleep - servo_safe_set_internal应该是非阻塞的
    // 舵机的实际运行延迟由调用端(状态机)处理
    return SERVO_ACTION_DONE;
}

// 舵机初始化：两路舵机统一配置为 50Hz，并默认回到复位角。
void servo_init(void) {
    gpio_set_function(SERVO1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(SERVO2_PIN, GPIO_FUNC_PWM);

    uint slice1 = pwm_gpio_to_slice_num(SERVO1_PIN);
    uint slice2 = pwm_gpio_to_slice_num(SERVO2_PIN);

    // 按当前系统时钟计算分频，避免不同默认时钟(如 125MHz/150MHz)导致频率偏差。
    uint32_t clk_hz = clock_get_hz(clk_sys);
    float clkdiv = (float)clk_hz / ((float)(SERVO_PWM_WRAP + 1u) * (float)SERVO_PWM_FREQ);
    if (clkdiv < 1.0f) clkdiv = 1.0f;
    if (clkdiv > 255.0f) clkdiv = 255.0f;

    pwm_set_clkdiv(slice1, clkdiv);
    pwm_set_wrap(slice1, SERVO_PWM_WRAP);
    pwm_set_clkdiv(slice2, clkdiv);
    pwm_set_wrap(slice2, SERVO_PWM_WRAP);

    float actual_freq = (float)clk_hz / (clkdiv * (float)(SERVO_PWM_WRAP + 1u));
    printf("[PWM] clk_sys=%lu Hz, clkdiv=%.3f, wrap=%u, target=%d Hz, actual=%.3f Hz\r\n",
           (unsigned long)clk_hz, clkdiv, SERVO_PWM_WRAP, SERVO_PWM_FREQ, actual_freq);

    pwm_set_gpio_level(SERVO1_PIN, angle_to_pwm(SERVO_RESET_ANGLE));
    pwm_set_gpio_level(SERVO2_PIN, angle_to_pwm(SERVO_RESET_ANGLE));
    pwm_set_enabled(slice1, true);
    pwm_set_enabled(slice2, true);
}

// 独立控制 S1，动作后阻塞等待舵机到位。
ServoActionResult servo1_set(uint8_t angle) {
    return servo_safe_set_internal(&servo1_guard, SERVO1_PIN, angle);
}
// 独立控制 S2，动作后阻塞等待舵机到位。
ServoActionResult servo2_set(uint8_t angle) {
    return servo_safe_set_internal(&servo2_guard, SERVO2_PIN, angle);
}

// ===================== 红外检测函数 =====================
// 红外输入上拉，配合“遮挡=低电平”的接线方式。
void ir_gpio_init(void) {
    #ifdef ONLY_IR3
    gpio_init(IR3_PIN); gpio_set_dir(IR3_PIN, GPIO_IN); gpio_pull_up(IR3_PIN);
    #else
    gpio_init(IR1_PIN); gpio_set_dir(IR1_PIN, GPIO_IN); gpio_pull_up(IR1_PIN);
    gpio_init(IR2_PIN); gpio_set_dir(IR2_PIN, GPIO_IN); gpio_pull_up(IR2_PIN);
    gpio_init(IR3_PIN); gpio_set_dir(IR3_PIN, GPIO_IN); gpio_pull_up(IR3_PIN);
    #endif
}

// 防抖读取：20ms 内采样 10 次，超过半数判定为“有障碍”。
// 优化防抖逻辑 - 使用更快的采样间隔减少总延迟
bool gpio_read_debounce(uint pin) {
    uint count = 0;
    for (int i = 0; i < 10; i++) {
        if (gpio_get(pin) == OBSTACLE_LEVEL) count++;
        sleep_us(100*IR_DEBOUNCE_MS);  // 改为 1000us，总防抖时间从 20ms 降为 10ms
    }
    return count > 5;
}

// 读取所有红外状态并打印调试信息。
// 日志中 X 表示被遮挡，O 表示未遮挡。
IRState ir_read_all(void) {
    IRState s;
    #ifdef ONLY_IR3
    s.ir1 = false;
    s.ir2 = false;
    s.ir3 = gpio_read_debounce(IR3_PIN);
    printf("[IR] IR3:%s\r\n", s.ir3 ? "X" : "O");
    #else
    s.ir1 = gpio_read_debounce(IR1_PIN);
    s.ir2 = gpio_read_debounce(IR2_PIN);
    s.ir3 = gpio_read_debounce(IR3_PIN);
    printf("[IR] IR1:%s IR2:%s IR3:%s\r\n", 
           s.ir1 ? "X" : "O", s.ir2 ? "X" : "O", s.ir3 ? "X" : "O");
    #endif  
    return s;
}

// ===================== 串口指令解析 =====================
// 采用非阻塞读取：仅在“状态匹配”时接受对应命令，避免误触发流程。
void uart_process_command(void) {
    int c = getchar_timeout_us(0);
    while (c != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            uart_buf[buf_idx] = '\0';
            buf_idx = 0;

            // 指令匹配：同一命令在错误状态下会被忽略。
            // 添加命令拒绝反馈，避免无声失败
            if (strcmp(uart_buf, "grab") == 0) {
                if (current_state == STATE_IDLE) {
                    printf("[CMD] Received: grab\r\n");
                    current_state = STATE_GRAB_CHECK;
                } else {
                    printf("[CMD] Rejected: grab (invalid state %d)\r\n", current_state);
                }
            } 
            else if (strcmp(uart_buf, "ascend") == 0) {
                if (current_state == STATE_WAIT_ASCEND) {
                    printf("[CMD] Received: ascend\r\n");
                    current_state = STATE_GRAB_VERIFY;
                } else {
                    printf("[CMD] Rejected: ascend (invalid state %d)\r\n", current_state);
                }
            }
            else if (strcmp(uart_buf, "release") == 0) {
                if (current_state == STATE_HOLDING || current_state == STATE_IDLE) {
                    printf("[CMD] Received: release\r\n");
                    current_state = STATE_RELEASE_ACTUATE;
                } else {
                    printf("[CMD] Rejected: release (invalid state %d)\r\n", current_state);
                }
            }
            else if (strcmp(uart_buf, "takeoff") == 0) {
                if (current_state == STATE_WAIT_TAKEOFF) {
                    printf("[CMD] Received: takeoff\r\n");
                    current_state = STATE_RELEASE_VERIFY;
                } else {
                    printf("[CMD] Rejected: takeoff (invalid state %d)\r\n", current_state);
                }
            }
            
            memset(uart_buf, 0, sizeof(uart_buf));
        } else if (buf_idx < sizeof(uart_buf)-1) {
            uart_buf[buf_idx++] = c;
        }else {
            // 溢出保护
            printf("[WARN] UART buffer overflow\r\n");
            buf_idx = 0;
            memset(uart_buf, 0, sizeof(uart_buf));
        }
        c = getchar_timeout_us(0);
    }
}

// ===================== 主状态机 =====================
// 规则摘要：
// 1) 取货检测时，IR 至少有两个触发且不能是“仅 IR1+IR2”。
// 2) 取货后必须收到 ascend，再用 IR3 校验是否抓稳。
// 3) 投递后必须收到 takeoff，再确认三路红外全部清空。
void state_machine_run(void) {
    bool entered = false;
    if (current_state != last_state_observed) {
        last_state_observed = current_state;
        state_enter_time = get_absolute_time();
        entered = true;
    }

    switch (current_state) {
        case STATE_IDLE:
            if (entered) {
                printf("[STATE] IDLE (Send 'grab' or 'release')\r\n");
            }
            break;

        // --- 取货流程 ---
        case STATE_GRAB_CHECK: {
            printf("[STATE] Checking IR pattern...\r\n");
            #ifdef ONLY_IR3
            printf("[ACT] Drive S1 90deg, then S2 90deg\r\n");
            ServoActionResult res1 = servo1_set(SERVO_TARGET_ANGLE);
            if (res1 == SERVO_ACTION_BLOCKED) {
                printf("[WARN] S1 blocked, will retry\r\n");
                break;  // 保持当前状态，下次循环重试
            }
            sleep_ms(SERVO_GRAB_GAP_MS);
            ServoActionResult res2 = servo2_set(SERVO_TARGET_ANGLE);
            if (res2 == SERVO_ACTION_BLOCKED) {
                printf("[WARN] S2 blocked, will retry\r\n");
                break;  // 保持当前状态，下次循环重试
            }
            current_state = STATE_GRAB_ACTUATE;
            #else
            IRState ir = ir_read_all();
            int cnt = ir.ir1 + ir.ir2 + ir.ir3;

            // 错误情况：
            // - 没有触发
            // - 仅一个触发
            // - 仅 IR1+IR2（业务上视为无效姿态）
            if (cnt == 0 || cnt == 1 || (ir.ir1 && ir.ir2 && !ir.ir3)) {
                printf("[ERROR] Invalid IR pattern. Task failed.\r\n");
                current_state = STATE_ERROR;
            }
            // 有效情况 A：IR1+IR3 或三路全触发。
            // 先 S1 后 S2，全部完成后再进入下一阶段。
            else if ((ir.ir1 && ir.ir3 && !ir.ir2) || (ir.ir1 && ir.ir2 && ir.ir3)) {
                printf("[ACT] Drive S1 90deg, then S2 90deg\r\n");
                // 改进舵机失败处理 - BLOCKED可能是暂时阻塞，不应直接ERROR
                ServoActionResult res1 = servo1_set(SERVO_TARGET_ANGLE);
                if (res1 == SERVO_ACTION_BLOCKED) {
                    printf("[WARN] S1 blocked, will retry\r\n");
                    break;  // 保持当前状态，下次循环重试
                }
                sleep_ms(SERVO_GRAB_GAP_MS);
                ServoActionResult res2 = servo2_set(SERVO_TARGET_ANGLE);
                if (res2 == SERVO_ACTION_BLOCKED) {
                    printf("[WARN] S2 blocked, will retry\r\n");
                    break;  // 保持当前状态，下次循环重试
                }
                current_state = STATE_GRAB_ACTUATE;
            }
            // 有效情况 B：IR2+IR3。
            // 先 S2 后 S1，全部完成后再进入下一阶段。
            else if (ir.ir2 && ir.ir3 && !ir.ir1) {
                printf("[ACT] Drive S2 90deg, then S1 90deg\r\n");
                // 改进舵机失败处理 - BLOCKED可能是暂时阻塞，不应直接ERROR
                ServoActionResult res2 = servo2_set(SERVO_TARGET_ANGLE);
                if (res2 == SERVO_ACTION_BLOCKED) {
                    printf("[WARN] S2 blocked, will retry\r\n");
                    break;  // 保持当前状态，下次循环重试
                }
                sleep_ms(SERVO_GRAB_GAP_MS);
                ServoActionResult res1 = servo1_set(SERVO_TARGET_ANGLE);
                if (res1 == SERVO_ACTION_BLOCKED) {
                    printf("[WARN] S1 blocked, will retry\r\n");
                    break;  // 保持当前状态，下次循环重试
                }
                current_state = STATE_GRAB_ACTUATE;
            }
            #endif
            break;
        }

        case STATE_GRAB_ACTUATE: 
            if (is_nil_time(actuate_start)) {
                actuate_start = get_absolute_time();
            }
            
            if (absolute_time_diff_us(actuate_start, get_absolute_time()) 
                > SERVO_DELAY_MS * 1000) {
                // 舵机动作完成后给上位机反馈，再等待升空指令。
                printf("[FEEDBACK] grab_finished\r\n");
                // grab 动作结束后立即通知飞控，避免把升空信号延后到命令阶段。
                fly_trigger_pulse_start(1000);
                actuate_start = nil_time;
                current_state = STATE_WAIT_ASCEND;
            }
            break;

        case STATE_WAIT_ASCEND:
            if (entered) {
                printf("[STATE] Waiting for 'ascend'...\r\n");
            }
            break;

        case STATE_GRAB_VERIFY: {
            printf("[STATE] Verifying grab (IR3)...\r\n");
            IRState ir = ir_read_all();
            // 约定 IR3 为抓取成功关键位。
            if (ir.ir3) {
                printf("[FEEDBACK] grab_success\r\n");
                current_state = STATE_HOLDING;
            } else {
                printf("[FEEDBACK] grab_failed\r\n");
                current_state = STATE_ERROR;
            }
            break;
        }

        case STATE_HOLDING:
            // 持货等待阶段，不主动动作，仅等待 release 命令。
            if (entered) {
                printf("[STATE] Holding (Send 'release')\r\n");
            }
            break;

        // --- 投递流程 ---
        case STATE_RELEASE_ACTUATE:
            printf("[ACT] Release sequence: S1 then S2\r\n");
            // 按既定机械顺序先 S1 后 S2，避免机构干涉。
            // 改进舵机失败处理 - BLOCKED可能是暂时阻塞，不应直接ERROR
            ServoActionResult s1_res = servo1_set(SERVO_RESET_ANGLE);
            if (s1_res == SERVO_ACTION_BLOCKED) {
                printf("[WARN] S1 reset blocked, will retry\r\n");
                break;  // 重试
            }
            ServoActionResult s2_res = servo2_set(SERVO_RESET_ANGLE);
            if (s2_res == SERVO_ACTION_BLOCKED) {
                printf("[WARN] S2 reset blocked, will retry\r\n");
                break;  // 重试
            }
            printf("[FEEDBACK] release_finished\r\n");
            // release 动作结束后立即通知飞控。
            fly_trigger_pulse_start(1000);
            current_state = STATE_WAIT_TAKEOFF;
            break;

        case STATE_WAIT_TAKEOFF:
            if (entered) {
                printf("[STATE] Waiting for 'takeoff'...\r\n");
            }
            break;

        case STATE_RELEASE_VERIFY: {
            printf("[STATE] Verifying release (All IR)...\r\n");
            IRState ir = ir_read_all();
            #ifdef ONLY_IR3
            if (!ir.ir3) {
                printf("[FEEDBACK] release_success\r\n");
                current_state = STATE_IDLE;
            }
            #else 
            // 投递成功要求三路都未触发
            if (!ir.ir1 && !ir.ir2 && !ir.ir3) {
                printf("[FEEDBACK] release_success\r\n");
                current_state = STATE_IDLE;
            }
            #endif 
            else {
                printf("[FEEDBACK] release_failed\r\n");
                current_state = STATE_ERROR;
            }
            break;
        }

        case STATE_ERROR:
            // 错误恢复：执行安全复位并回到空闲态。
            if (entered) {
                printf("[STATE] ERROR. Resetting to IDLE...\r\n");
                servo1_set(SERVO_RESET_ANGLE);
                servo2_set(SERVO_RESET_ANGLE);
                gpio_put(FLY_TRIGGER_PIN, 0);
                fly_pulse_active = false;
                error_recovery_count++;
            }

            if (error_recovery_count >= ERROR_RECOVERY_MAX && state_elapsed_ms() >= 2000) {
                printf("[STATE] ERROR recovery complete. Back to IDLE.\r\n");
                error_recovery_count = 0;
                current_state = STATE_IDLE;
            }
            break;

        default:
            current_state = STATE_IDLE;
            break;
    }
}

// ===================== 主函数 =====================
// 主循环采用“收命令 + 跑状态机”的轮询模式。
int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("\r\n=== drone_Pico2 Grab/Release System Ready ===\r\n");

    // 初始化飞控触发引脚，默认低电平。
    gpio_init(FLY_TRIGGER_PIN);
    gpio_set_dir(FLY_TRIGGER_PIN, GPIO_OUT);
    gpio_put(FLY_TRIGGER_PIN, 0);

    servo_init();
    ir_gpio_init();
    led_fx_init();
    printf("Init done.\r\n");

    while (1) {
        uart_process_command();
        fly_trigger_update();
        state_machine_run();
        led_fx_update();
        sleep_ms(20);
    }
}