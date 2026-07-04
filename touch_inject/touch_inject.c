/*
 * touch_inject.c — FTS 触控注入内核模块 (拟人化增强版 v2.1)
 *
 * 目标: 小米13 (fuxi), GKI 5.15.178, STM FTS 触控IC
 *   内核 uname -r: 5.15.178-android13-8-g362d545d31a5
 *   GKI vermagic:  5.15.78 (系统所有 .ko 均用此简化版本号)
 *
 * 工作原理:
 *   1. 通过 input_handler 自动匹配名为 "fts" 的 input_dev
 *   2. 创建 /dev/touch_inject 字符设备
 *   3. 用户态 daemon write() 坐标数据 -> 内核调用 input_event() 注入
 *
 * v2.0 拟人化改进 (所有改进均在内核内部完成, 协议不变):
 *   1. 高斯分布抖动 (CLT近似) 替代均匀分布 -- 坐标偏移更接近人类
 *   2. AR(1) 时序相关抖动 -- 连续帧间抖动有记忆性, 非独立
 *   3. TOUCH_MAJOR/MINOR 随机化 -- 模拟指腹接触面积变化
 *   4. ABS_MT_PRESSURE 支持 -- hack补充absinfo, 上报压力值
 *   5. 多帧压力渐变点击 -- 按下渐入->保持->抬起渐出, 非瞬变
 *   6. 三次贝塞尔曲线滑动 -- 2个控制点, 比二次更自然
 *   7. Smoothstep缓动 -- 三阶段运动模型(加速->匀速->减速)
 *
 * v2.1 工程修复 (不改协议, 不改拟人化算法):
 *   1. touch_fops.owner = THIS_MODULE  (修复 use-after-free 崩溃风险)
 *   2. input_handler 正确创建 input_handle (修复悬垂指针, disconnect 生效)
 *   3. sscanf 返回4时 dur 初始化默认值 (修复未定义行为)
 *   4. tracking ID 全局递增 (符合 Type-B 协议, 避免固定模式)
 *   5. (void*)0 统一为 NULL (代码风格)
 *
 * 协议格式 (全部 ASCII, 以换行结束) -- 与 v1.0 完全兼容:
 *   T x y              -- tap (点击, 内部多帧压力渐变)
 *   D x y              -- down (按下, 保持)
 *   M x y              -- move (移动)
 *   U                  -- up (抬起)
 *   S x1 y1 x2 y2 ms   -- swipe (滑动, 三次贝塞尔+缓动)
 *   W                  -- wait 0.5s
 *
 * 坐标映射: 屏幕 1080x2400 -> 触控 10800x24000 (x10)
 *           本模块统一使用屏幕坐标, 内核自动 x10 转换
 *
 * 加载: su -c insmod touch_inject.ko
 * 测试: echo "T 540 1200" > /dev/touch_inject
 * 卸载: su -c rmmod touch_inject
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/mm.h>
#include <linux/bitops.h>
#include <generated/utsrelease.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("buddy");
MODULE_DESCRIPTION("STM FTS touch injector for Xiaomi 13 (fuxi) - humanized v2.1");
MODULE_VERSION("2.1");
/* Manually set name and vermagic (normally added by modpost) */
MODULE_INFO(name, "touch_inject");
/* vermagic: GKI 5.15.78 是系统所有 .ko 的统一简化版本号
 * 实际内核 uname -r = 5.15.178-android13-8-g362d545d31a5
 * 经核验 /vendor_dlkm/lib/modules/*.ko 的 vermagic 均为 5.15.78 */
MODULE_INFO(vermagic, "5.15.78 SMP preempt mod_unload modversions aarch64");

/* ============================================================
 * Configuration
 * ============================================================ */

/* 屏幕坐标 -> 触控坐标 缩放因子 */
#define X_SCALE 10
#define Y_SCALE 10

/* 触控坐标范围 */
#define X_MAX 10799
#define Y_MAX 23999

/* 多点触控 slot 数量 */
#define MAX_SLOTS 10

/* 设备名、类名 */
#define DEVICE_NAME "touch_inject"
#define CLASS_NAME  "touch_inject"

/* 字符设备写缓冲区大小 */
#define BUF_SIZE 256

/* --- 拟人化参数 --- */

/* 随机抖动范围 (像素, 高斯分布 sigma) */
#define JITTER_RANGE 3

/* 压力参数 (0-255, 真实手指典型范围) */
#define PRESSURE_LIGHT  15   /* 刚接触时的轻压力 */
#define PRESSURE_MIN    30   /* 按压下限 */
#define PRESSURE_MAX    80   /* 按压上限 */
#define PRESSURE_DEFAULT 55  /* D/M 命令默认压力 */

/* 触摸面积参数 (模拟指腹接触面) */
#define TOUCH_MAJOR_MIN 7
#define TOUCH_MAJOR_MAX 11
#define TOUCH_MINOR_MIN 5
#define TOUCH_MINOR_MAX 8

/* ============================================================
 * Global state
 * ============================================================ */

static struct input_dev *fts_input_dev = NULL;
static struct class   *touch_class  = NULL;
static struct device  *touch_device = NULL;
static dev_t           dev_num;
static struct cdev     touch_cdev;

static int    current_slot = 0;
static int    touch_down   = 0;
static bool   module_ready = false;

/* AR(1) 时序相关抖动状态 (滑动时使用, 使连续帧抖动有记忆性) */
static int    ar1_state_x = 0;
static int    ar1_state_y = 0;

/* v2.1: tracking ID 全局递增计数器 (符合 Type-B 协议)
 * 真人每次触摸的 tracking ID 必须唯一且递增,
 * 固定值模式容易被行为分析检测 */
static int    tracking_id_counter = 0;

/* ============================================================
 * Random humanization helpers
 * ============================================================ */

/*
 * 高斯分布随机数 (CLT近似)
 *
 * 三个均匀分布[-1000,1000]求和, sigma ~= 1414
 * 返回值: gaussian_rand(sigma) ~ N(0, sigma)
 *
 * 为什么不用 Box-Muller: 内核空间避免浮点运算,
 * CLT近似在 sigma < 20 时精度足够 (3个样本即可)
 */
static int gaussian_rand(int sigma)
{
    int sum;
    sum =  (prandom_u32() % 2001) - 1000;
    sum += (prandom_u32() % 2001) - 1000;
    sum += (prandom_u32() % 2001) - 1000;
    /* sum 范围 [-3000, 3000], sigma ~= 1414 */
    return (sum * sigma) / 1414;
}

/*
 * 高斯抖动 (独立, 无记忆)
 * 用于点击等单次操作
 */
static int jitter(int val, int range)
{
    return val + gaussian_rand(range);
}

/*
 * AR(1) 时序相关抖动
 * 模型: x[n] = 0.7 * x[n-1] + w[n]
 *   其中 w[n] ~ N(0, range)
 *   0.7 系数控制记忆性 (越大越平滑)
 *
 * 用于滑动轨迹, 使相邻帧的偏移有连续性,
 * 而非每帧独立跳变 (真实手指颤动有惯性)
 */
static int ar1_jitter(int val, int range, int *state)
{
    int w = gaussian_rand(range);
    /* x[n] = 0.7 * x[n-1] + w[n], 整数运算: *7/10 */
    *state = (*state * 7) / 10 + w;
    return val + *state;
}

/* ============================================================
 * Touch protocol helpers
 * ============================================================ */

/*
 * 发送单点触摸事件。
 * pressure: 按压力度 0-255 (若设备支持 ABS_MT_PRESSURE 则上报)
 * first=true 时带 tracking ID 分配
 *
 * 触面 TOUCH_MAJOR/MINOR 每帧随机化, 模拟指腹面积自然变化
 */
static void fts_report_abs(int x, int y, int slot, bool first, int pressure)
{
    int touch_major, touch_minor;

    if (!fts_input_dev)
        return;

    /* Slot 切换 */
    input_event(fts_input_dev, EV_ABS, ABS_MT_SLOT, slot);

    if (first) {
        /* v2.1: tracking ID 全局递增 (符合 Type-B 协议)
         * 原 v2.0 用固定值 (slot+1)*100+0x45, 模式高度规律,
         * 同一 slot 抬起后再次按下会复用相同 ID, 易被检测 */
        input_event(fts_input_dev, EV_ABS, ABS_MT_TRACKING_ID,
                    tracking_id_counter++);
    }

    /* 坐标 (屏幕坐标 x 比例 -> 触控坐标) */
    input_event(fts_input_dev, EV_ABS, ABS_MT_POSITION_X,
                clamp(x * X_SCALE, 0, X_MAX));
    input_event(fts_input_dev, EV_ABS, ABS_MT_POSITION_Y,
                clamp(y * Y_SCALE, 0, Y_MAX));

    /* 触面: 随机化手指接触面 (模拟指腹面积变化) */
    touch_major = TOUCH_MAJOR_MIN +
                  prandom_u32() % (TOUCH_MAJOR_MAX - TOUCH_MAJOR_MIN + 1);
    touch_minor = TOUCH_MINOR_MIN +
                  prandom_u32() % (TOUCH_MINOR_MAX - TOUCH_MINOR_MIN + 1);
    input_event(fts_input_dev, EV_ABS, ABS_MT_TOUCH_MAJOR, touch_major);
    input_event(fts_input_dev, EV_ABS, ABS_MT_TOUCH_MINOR, touch_minor);

    /* 方向角 (竖直) */
    input_event(fts_input_dev, EV_ABS, ABS_MT_ORIENTATION, 0);

    /*
     * 压力: 若 absinfo 已注册则上报
     * FTS 驱动原生不支持 ABS_MT_PRESSURE, 在 fts_finder_connect()
     * 中通过 hack 补充了 absinfo, 此处条件检查后上报
     */
    if (fts_input_dev->absinfo &&
        test_bit(ABS_MT_PRESSURE, fts_input_dev->absbit)) {
        input_event(fts_input_dev, EV_ABS, ABS_MT_PRESSURE,
                    clamp(pressure, 0, 255));
    }
}

static void fts_report_sync(void)
{
    if (!fts_input_dev)
        return;
    input_event(fts_input_dev, EV_SYN, SYN_REPORT, 0);
}

static void fts_touch_down(int x, int y, int pressure)
{
    touch_down = 1;

    /* 按下 */
    input_event(fts_input_dev, EV_KEY, BTN_TOUCH, 1);
    input_event(fts_input_dev, EV_KEY, BTN_TOOL_FINGER, 1);

    fts_report_abs(x, y, current_slot, true, pressure);
    fts_report_sync();
}

static void fts_touch_move(int x, int y, int pressure)
{
    if (!touch_down) {
        fts_touch_down(x, y, pressure);
        return;
    }
    fts_report_abs(x, y, current_slot, false, pressure);
    fts_report_sync();
}

static void fts_touch_up(void)
{
    if (!touch_down)
        return;

    /* 释放 tracking ID */
    input_event(fts_input_dev, EV_ABS, ABS_MT_SLOT, current_slot);
    input_event(fts_input_dev, EV_ABS, ABS_MT_TRACKING_ID, -1);
    fts_report_sync();

    /* 抬起 */
    input_event(fts_input_dev, EV_KEY, BTN_TOUCH, 0);
    input_event(fts_input_dev, EV_KEY, BTN_TOOL_FINGER, 0);
    fts_report_sync();

    touch_down = 0;
    current_slot = (current_slot + 1) % MAX_SLOTS;
}

/* ============================================================
 * High-level touch actions (humanized)
 * ============================================================ */

/*
 * 拟人化点击: 四阶段压力渐变
 *
 * 真实手指点击的压力曲线:
 *   轻触(15-25) -> 渐增(30-45) -> 稳定(50-65) -> 渐减(20-30) -> 抬起
 *
 * 相比 v1.0 的 "按下->等50-120ms->抬起" 两帧,
 * v2.0 发出 4 帧事件, 每帧压力不同, 模拟力度渐入渐出
 */
static void do_tap(int x, int y)
{
    int hold_ms;

    if (!module_ready || !fts_input_dev)
        return;

    /* 高斯抖动坐标 (人手不可能精确点到同一像素) */
    x = jitter(x, JITTER_RANGE);
    y = jitter(y, JITTER_RANGE);

    /* Phase 1: 轻触 (手指刚接触屏幕, 压力低) */
    fts_touch_down(x, y, PRESSURE_LIGHT + prandom_u32() % 10);
    usleep_range_state(8000, 16000, 2);  /* 8-16ms */

    /* Phase 2: 渐增 (手指用力压下, 压力上升) */
    fts_touch_move(x, y, PRESSURE_MIN + prandom_u32() % 15);
    usleep_range_state(8000, 16000, 2);  /* 8-16ms */

    /* Phase 3: 保持 (稳定按压, 主体停留时间) */
    fts_touch_move(x, y,
                   (PRESSURE_MIN + PRESSURE_MAX) / 2 + prandom_u32() % 10);
    hold_ms = 30 + prandom_u32() % 60;  /* 30-90ms 保持 */
    usleep_range_state(hold_ms * 1000, (hold_ms + 30) * 1000, 2);

    /* Phase 4: 渐减 (手指准备抬起, 压力回落) */
    fts_touch_move(x, y, PRESSURE_LIGHT + prandom_u32() % 10);
    usleep_range_state(4000, 8000, 2);  /* 4-8ms */

    fts_touch_up();

    /* 总时长: 50-130ms (与 v1.0 的 50-120ms 一致) */
}

/*
 * 拟人化滑动: 三次贝塞尔 + Smoothstep缓动 + AR(1)抖动 + 压力变化
 *
 * 改进点:
 *   1. 三次贝塞尔 (2个控制点) 替代二次贝塞尔 (1个控制点)
 *      B(t) = (1-t)^3*P0 + 3(1-t)^2*t*P1 + 3(1-t)*t^2*P2 + t^3*P3
 *   2. Smoothstep 缓动: t' = 3t^2 - 2t^3
 *      使滑动有加速->匀速->减速的三阶段特征
 *   3. AR(1) 抖动: 相邻帧偏移有记忆性, 模拟肌肉颤动惯性
 *   4. 压力变化: 起始渐增->中段稳定->末尾渐减
 *
 * 整数定点运算, 无浮点 (内核空间)
 */
static void do_swipe(int x1, int y1, int x2, int y2, int duration_ms)
{
    int steps, i;
    int cx1, cy1, cx2, cy2;     /* 三次贝塞尔的两个控制点 */
    int px, py;
    int progress, base_p, swipe_p;
    int t_raw, t2, t_eased;     /* Smoothstep 缓动参数 */
    int ti, mt, mt2, mt3, t2b, t3b;  /* 贝塞尔系数 */
    int b0, b1, b2, b3;

    if (!module_ready || !fts_input_dev)
        return;
    if (duration_ms < 50)
        duration_ms = 50;
    if (duration_ms > 5000)
        duration_ms = 5000;

    steps = duration_ms / 8;  /* ~8ms/frame ~ 125Hz */
    if (steps < 4)
        steps = 4;
    if (steps > 200)
        steps = 200;

    /*
     * 三次贝塞尔的两个控制点
     * P1 在路径 1/3 处, P2 在 2/3 处, 各加 +-20px 随机偏移
     * 两个控制点独立偏移, 产生比二次贝塞尔更自然的弧线
     */
    cx1 = x1 + (x2 - x1) / 3 + (prandom_u32() % 41 - 20);
    cy1 = y1 + (y2 - y1) / 3 + (prandom_u32() % 41 - 20);
    cx2 = x1 + 2 * (x2 - x1) / 3 + (prandom_u32() % 41 - 20);
    cy2 = y1 + 2 * (y2 - y1) / 3 + (prandom_u32() % 41 - 20);

    /* 重置 AR(1) 状态 (每次滑动从零开始) */
    ar1_state_x = 0;
    ar1_state_y = 0;

    /* 起始: 轻触 */
    fts_touch_down(x1, y1, PRESSURE_LIGHT + prandom_u32() % 10);
    usleep_range_state(20000, 40000, 2);

    for (i = 1; i <= steps; i++) {
        /*
         * Smoothstep 缓动: S(t) = 3t^2 - 2t^3
         * 使 t 在起始和结束处变化慢 (加速/减速段),
         * 中段变化快 (匀速段), 模拟三阶段运动
         */
        t_raw = (i * 1000) / steps;       /* [0, 1000] */
        t2 = t_raw * t_raw / 1000;        /* t^2, [0, 1000] */
        t_eased = 3 * t2 - 2 * t2 * t_raw / 1000;  /* S(t), [0, 1000] */

        /*
         * 三次贝塞尔系数 (定点整数, scale=1000)
         * b0 = (1-t)^3, b1 = 3(1-t)^2*t, b2 = 3(1-t)*t^2, b3 = t^3
         * b0+b1+b2+b3 = 1000 (=1.0)
         */
        ti = t_eased;
        mt = 1000 - ti;
        mt2 = mt * mt / 1000;
        mt3 = mt2 * mt / 1000;
        t2b = ti * ti / 1000;
        t3b = t2b * ti / 1000;

        b0 = mt3;                        /* (1-t)^3 * 1000 */
        b1 = 3 * mt2 * ti / 1000;        /* 3(1-t)^2*t * 1000 */
        b2 = 3 * mt * t2b / 1000;        /* 3(1-t)*t^2 * 1000 */
        b3 = t3b;                        /* t^3 * 1000 */

        /* B(t) = (b0*P0 + b1*P1 + b2*P2 + b3*P3) / 1000 */
        px = (int)((s64)(b0 * x1 + b1 * cx1 + b2 * cx2 + b3 * x2) / 1000);
        py = (int)((s64)(b0 * y1 + b1 * cy1 + b2 * cy2 + b3 * y2) / 1000);

        /* AR(1) 相关抖动 (连续帧有记忆性, 模拟肌肉颤动) */
        px = ar1_jitter(px, 2, &ar1_state_x);
        py = ar1_jitter(py, 2, &ar1_state_y);

        /*
         * 滑动压力变化: 三段式
         *   0-15%:  渐增 (手指按压下去)
         *   15-85%: 稳定 (主体滑动, 略有波动)
         *   85-100%: 渐减 (手指准备抬起)
         */
        progress = (i * 100) / steps;
        if (progress < 15) {
            base_p = PRESSURE_LIGHT +
                     (PRESSURE_MIN - PRESSURE_LIGHT) * progress / 15;
        } else if (progress > 85) {
            base_p = PRESSURE_LIGHT +
                     (PRESSURE_MIN - PRESSURE_LIGHT) * (100 - progress) / 15;
        } else {
            base_p = (PRESSURE_MIN + PRESSURE_MAX) / 2;
        }
        swipe_p = base_p + gaussian_rand(5);
        if (swipe_p < PRESSURE_LIGHT)
            swipe_p = PRESSURE_LIGHT;
        if (swipe_p > PRESSURE_MAX + 10)
            swipe_p = PRESSURE_MAX + 10;

        fts_touch_move(px, py, swipe_p);
        usleep_range_state(6000, 10000, 2);
    }

    usleep_range_state(30000, 60000, 2);
    fts_touch_up();
}

/* ============================================================
 * Character device operations
 * ============================================================ */

static ssize_t touch_write(struct file *file, const char __user *buf,
                           size_t count, loff_t *ppos)
{
    char kbuf[BUF_SIZE];
    char cmd;
    /* v2.1: dur 初始化默认值, 避免 sscanf 返回4时读取未初始化变量 (UB) */
    int x, y, x2, y2, dur = 500;

    if (!module_ready || !fts_input_dev)
        return -ENODEV;

    if (count >= BUF_SIZE)
        count = BUF_SIZE - 1;
    if (count == 0)
        return 0;

    if (copy_from_user(kbuf, buf, count))
        return -EFAULT;
    kbuf[count] = '\0';

    cmd = kbuf[0];

    switch (cmd) {
    case 'T': /* Tap: T x y */
        if (sscanf(kbuf, "T %d %d", &x, &y) == 2)
            do_tap(x, y);
        break;

    case 'D': /* Down: D x y (使用默认压力) */
        if (sscanf(kbuf, "D %d %d", &x, &y) == 2)
            fts_touch_down(jitter(x, 2), jitter(y, 2),
                           PRESSURE_DEFAULT + gaussian_rand(5));
        break;

    case 'M': /* Move: M x y (使用默认压力) */
        if (sscanf(kbuf, "M %d %d", &x, &y) == 2)
            fts_touch_move(jitter(x, 2), jitter(y, 2),
                           PRESSURE_DEFAULT + gaussian_rand(5));
        break;

    case 'U': /* Up */
        fts_touch_up();
        break;

    case 'S': /* Swipe: S x1 y1 x2 y2 dur_ms */
        if (sscanf(kbuf, "S %d %d %d %d %d", &x, &y, &x2, &y2, &dur) >= 4) {
            if (dur <= 0 || dur > 5000)
                dur = 500;
            do_swipe(x, y, x2, y2, dur);
        }
        break;

    case 'W': /* Wait */
        usleep_range_state(400000, 600000, 2);
        break;

    default:
        break;
    }

    return count;
}

static ssize_t touch_read(struct file *file, char __user *buf,
                          size_t count, loff_t *ppos)
{
    const char *msg;

    if (*ppos > 0)
        return 0;

    if (module_ready && fts_input_dev)
        msg = "OK ready fts v2.1\n";
    else if (fts_input_dev)
        msg = "INIT fts_found\n";
    else
        msg = "WAIT no_fts\n";

    return simple_read_from_buffer(buf, count, ppos, msg, strlen(msg));
}

static int touch_open(struct inode *inode, struct file *file)
{
    if (!fts_input_dev) {
        pr_warn("[touch_inject] open: FTS device not ready yet\n");
        return -ENODEV;
    }
    touch_down = 0;
    current_slot = 0;
    return 0;
}

static int touch_release(struct inode *inode, struct file *file)
{
    /* 安全: 如果还有触摸保持, 强制释放 */
    if (touch_down) {
        pr_warn("[touch_inject] closing while touch still down, releasing\n");
        fts_touch_up();
    }
    return 0;
}

static const struct file_operations touch_fops = {
    /* v2.1: owner 必须为 THIS_MODULE
     * 原 v2.0 为 NULL, 导致模块被打开使用时引用计数不加,
     * rmmod 会直接卸载正在使用的模块, 触发 use-after-free 内核崩溃 */
    .owner   = THIS_MODULE,
    .open    = touch_open,
    .release = touch_release,
    .read    = touch_read,
    .write   = touch_write,
};

/* ============================================================
 * Input handler -- 自动找到 FTS 的 input_dev
 *
 * v2.1 修复: 正确创建 input_handle
 *   原 v2.0/v1.0 的 connect 回调只保存 dev 指针就返回0,
 *   没有调用 input_register_handle(), 导致:
 *     1. disconnect 永远不被触发 (死代码)
 *     2. FTS 设备热拔时 fts_input_dev 变悬垂指针
 *   v2.1 在 connect 里分配 input_handle 并注册,
 *   disconnect 里注销并释放, 符合 input 子系统约定.
 * ============================================================ */

/* 保存我们的 handle 指针, 用于 disconnect 时清理 */
static struct input_handle *fts_handle = NULL;

static int fts_finder_connect(struct input_handler *handler,
                              struct input_dev *dev,
                              const struct input_device_id *id)
{
    struct input_handle *handle;
    int ret;

    /* 只匹配名为 "fts" 的设备 */
    if (!dev->name || strcmp(dev->name, "fts") != 0)
        return -ENODEV;  /* 不匹配, 明确拒绝 */

    /* 已有 handle 则跳过 (避免重复) */
    if (fts_handle) {
        pr_warn("[touch_inject] FTS already connected, skip\n");
        return 0;
    }

    pr_info("[touch_inject] FTS input_dev captured: %px\n", dev);
    pr_info("[touch_inject]   phys=%s\n",
            dev->phys ? dev->phys : "(null)");
    pr_info("[touch_inject]   ABS_X: 0..%d ABS_Y: 0..%d\n",
            dev->absinfo ? dev->absinfo[ABS_MT_POSITION_X].maximum : -1,
            dev->absinfo ? dev->absinfo[ABS_MT_POSITION_Y].maximum : -1);

    /*
     * Hack: 补充 ABS_MT_PRESSURE 轴支持
     *
     * FTS 驱动未注册 ABS_MT_PRESSURE, 但 Android MotionEvent
     * 可读取压力值用于反检测。直接修改 input_dev 的 absinfo
     * 和 absbit, 使后续 input_event() 发送压力不被内核丢弃。
     */
    if (dev->absinfo) {
        if (!test_bit(ABS_MT_PRESSURE, dev->absbit)) {
            dev->absinfo[ABS_MT_PRESSURE].minimum = 0;
            dev->absinfo[ABS_MT_PRESSURE].maximum = 255;
            dev->absinfo[ABS_MT_PRESSURE].fuzz = 0;
            dev->absinfo[ABS_MT_PRESSURE].flat = 0;
            dev->absinfo[ABS_MT_PRESSURE].resolution = 0;
            __set_bit(ABS_MT_PRESSURE, dev->absbit);
            pr_info("[touch_inject] Added ABS_MT_PRESSURE axis (0-255)\n");
        } else {
            pr_info("[touch_inject] ABS_MT_PRESSURE already supported\n");
        }
    }

    /* v2.1: 分配并注册 input_handle
     * 注意: 用 kmalloc+memset 替代 kzalloc, 因为 Termux 编译环境中
     * kzalloc 是 inline 函数, 实际解析为 __kmalloc, 不需要单独的 CRC */
    handle = kmalloc(sizeof(*handle), GFP_KERNEL | __GFP_ZERO);
    if (!handle) {
        pr_err("[touch_inject] FAILED to alloc input_handle\n");
        return -ENOMEM;
    }

    handle->dev = dev;
    handle->handler = handler;
    handle->name = "touch_inject";

    ret = input_register_handle(handle);
    if (ret) {
        pr_err("[touch_inject] FAILED input_register_handle: %d\n", ret);
        kfree(handle);
        return ret;
    }

    fts_handle = handle;
    fts_input_dev = dev;

    if (!module_ready) {
        module_ready = true;
        pr_info("[touch_inject] Module ready for injection (v2.1).\n");
    }

    return 0;
}

static void fts_finder_disconnect(struct input_handle *handle)
{
    /* v2.1: 正确注销 handle, 清理悬垂指针 */
    if (handle == fts_handle) {
        pr_warn("[touch_inject] FTS device disconnecting!\n");
        input_unregister_handle(handle);
        kfree(handle);
        fts_handle = NULL;
        fts_input_dev = NULL;
        module_ready = false;
    }
}

static const struct input_device_id fts_finder_ids[] = {
    { .driver_info = 1 },
    { }
};

static struct input_handler fts_finder_handler = {
    .name       = "fts_finder",
    .id_table   = fts_finder_ids,
    .connect    = fts_finder_connect,
    .disconnect = fts_finder_disconnect,
};

/* ============================================================
 * Module init / exit
 * ============================================================ */

static int __init touch_inject_init(void)
{
    int ret;

    pr_info("[touch_inject] ====================================\n");
    pr_info("[touch_inject] Loading v2.1 (humanized+fixed) on Xiaomi 13 (fuxi)\n");
    pr_info("[touch_inject] Kernel: %s\n", UTS_RELEASE);
    pr_info("[touch_inject] Features: gaussian jitter, AR(1), pressure gradient,\n");
    pr_info("[touch_inject]   cubic bezier, smoothstep easing\n");
    pr_info("[touch_inject] v2.1 fixes: owner, input_handle, dur init, tracking ID\n");

    /* 步骤1: 注册 input handler, 自动匹配 fts 设备 */
    ret = input_register_handler(&fts_finder_handler);
    if (ret) {
        pr_err("[touch_inject] FAILED to register input handler: %d\n", ret);
        return ret;
    }

    if (!fts_input_dev) {
        pr_warn("[touch_inject] WARNING: No 'fts' device found immediately\n");
        pr_warn("[touch_inject] It may appear later (driver not loaded yet?)\n");
        /* 不返回错误, 因为 FTS 驱动可能在这个模块之后加载 */
    }

    /* 步骤2: 创建字符设备 /dev/touch_inject */
    ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    if (ret) {
        pr_err("[touch_inject] FAILED alloc_chrdev_region: %d\n", ret);
        goto err_handler;
    }

    cdev_init(&touch_cdev, &touch_fops);
    ret = cdev_add(&touch_cdev, dev_num, 1);
    if (ret) {
        pr_err("[touch_inject] FAILED cdev_add: %d\n", ret);
        goto err_region;
    }

    touch_class = class_create(NULL, CLASS_NAME);
    if (IS_ERR(touch_class)) {
        ret = PTR_ERR(touch_class);
        pr_err("[touch_inject] FAILED class_create: %d\n", ret);
        goto err_cdev;
    }

    touch_device = device_create(touch_class, NULL, dev_num,
                                 NULL, DEVICE_NAME);
    if (IS_ERR(touch_device)) {
        ret = PTR_ERR(touch_device);
        pr_err("[touch_inject] FAILED device_create: %d\n", ret);
        goto err_class;
    }

    if (fts_input_dev)
        module_ready = true;

    pr_info("[touch_inject] ====================================\n");
    pr_info("[touch_inject] /dev/touch_inject created (major=%d)\n",
            MAJOR(dev_num));
    pr_info("[touch_inject] FTS device: %s\n",
            fts_input_dev ? "CONNECTED" : "NOT FOUND (will retry)");
    pr_info("[touch_inject] Module v2.1 loaded successfully.\n");
    pr_info("[touch_inject] ====================================\n");

    return 0;

err_class:
    class_destroy(touch_class);
err_cdev:
    cdev_del(&touch_cdev);
err_region:
    unregister_chrdev_region(dev_num, 1);
err_handler:
    input_unregister_handler(&fts_finder_handler);
    return ret;
}

static void __exit touch_inject_exit(void)
{
    /* 清理触摸状态 */
    if (touch_down)
        fts_touch_up();

    module_ready = false;

    /* v2.1: 清理 input_handle (如果还存在的) */
    if (fts_handle) {
        input_unregister_handle(fts_handle);
        kfree(fts_handle);
        fts_handle = NULL;
    }

    device_destroy(touch_class, dev_num);
    class_destroy(touch_class);
    cdev_del(&touch_cdev);
    unregister_chrdev_region(dev_num, 1);
    input_unregister_handler(&fts_finder_handler);
    fts_input_dev = NULL;  /* v2.1: (void*)0 统一为 NULL */

    pr_info("[touch_inject] Module v2.1 unloaded.\n");
}

module_init(touch_inject_init);
module_exit(touch_inject_exit);
