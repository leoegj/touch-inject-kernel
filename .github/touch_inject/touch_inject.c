/*
 * touch_inject.c — FTS 触控注入内核模块
 *
 * 目标: 小米13 (fuxi), GKI 5.15.178, STM FTS 触控IC
 *
 * 工作原理:
 *   1. 通过 input_handler 自动匹配名为 "fts" 的 input_dev
 *   2. 创建 /dev/touch_inject 字符设备
 *   3. 用户态 daemon write() 坐标数据 → 内核调用 input_event() 注入
 *
 * 加载: su -c insmod touch_inject.ko
 * 测试: echo "T 540 1200" > /dev/touch_inject
 * 卸载: su -c rmmod touch_inject
 *
 * 坐标映射: 屏幕 1080x2400 → 触控 10800x24000 (×10)
 *           本模块统一使用屏幕坐标，内核自动 ×10 转换
 *
 * 协议格式 (全部 ASCII，以换行结束):
 *   T x y              — tap (点击)
 *   D                  — down (按下, 保持)
 *   M x y              — move (移动)
 *   U                  — up (抬起)
 *   S x1 y1 x2 y2 ms   — swipe (滑动)
 *   W                   — wait 0.5s
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
#include <generated/utsrelease.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("buddy");
MODULE_DESCRIPTION("STM FTS touch injector for Xiaomi 13 (fuxi)");
MODULE_VERSION("1.0");
/* Manually set name and vermagic (normally added by modpost) */
MODULE_INFO(name, "touch_inject");
/* vermagic must match phone's kernel: 5.15.78 from existing modules */
MODULE_INFO(vermagic, "5.15.78 SMP preempt mod_unload modversions aarch64");

/* ============================================================
 * Configuration
 * ============================================================ */

/* 屏幕坐标 → 触控坐标 缩放因子 */
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

/* 随机抖动范围 (像素) */
#define JITTER_RANGE 3

/* ============================================================
 * Touch protocol helpers
 * ============================================================ */

/*
 * 发送单点触摸事件。
 * slim=0 时带触面数据（TOUCH_MAJOR/MINOR），
 * 真机 FTS 驱动也上报这些字段。
 */
static void fts_report_abs(int x, int y, int slot, bool first)
{
    if (!fts_input_dev)
        return;

    /* Slot 切换 */
    input_event(fts_input_dev, EV_ABS, ABS_MT_SLOT, slot);

    if (first) {
        /* 首次触摸：分配 tracking ID */
        input_event(fts_input_dev, EV_ABS, ABS_MT_TRACKING_ID,
                    (slot + 1) * 100 + 0x45);
    }

    /* 坐标 (屏幕坐标 × 比例 → 触控坐标) */
    input_event(fts_input_dev, EV_ABS, ABS_MT_POSITION_X,
                clamp(x * X_SCALE, 0, X_MAX));
    input_event(fts_input_dev, EV_ABS, ABS_MT_POSITION_Y,
                clamp(y * Y_SCALE, 0, Y_MAX));

    /* 触面: 模拟手指接触面 ~8-10 单位 */
    input_event(fts_input_dev, EV_ABS, ABS_MT_TOUCH_MAJOR, 8);
    input_event(fts_input_dev, EV_ABS, ABS_MT_TOUCH_MINOR, 6);

    /* 方向角 (竖直) */
    input_event(fts_input_dev, EV_ABS, ABS_MT_ORIENTATION, 0);

    /* FTS 驱动不支持 ABS_MT_PRESSURE, 跳过 */
}

static void fts_report_sync(void)
{
    if (!fts_input_dev)
        return;
    input_event(fts_input_dev, EV_SYN, SYN_REPORT, 0);
}

static void fts_touch_down(int x, int y)
{
    touch_down = 1;

    /* 按下 */
    input_event(fts_input_dev, EV_KEY, BTN_TOUCH, 1);
    input_event(fts_input_dev, EV_KEY, BTN_TOOL_FINGER, 1);

    fts_report_abs(x, y, current_slot, true);
    fts_report_sync();
}

static void fts_touch_move(int x, int y)
{
    if (!touch_down) {
        fts_touch_down(x, y);
        return;
    }
    fts_report_abs(x, y, current_slot, false);
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

/* 添加随机微抖动 (模拟真人手指自然颤抖) */
static int jitter(int val, int range)
{
    int r = (prandom_u32() % (range * 2 + 1)) - range;
    return val + r;
}

/*
 * 模拟真人的触摸流程：按下 → 短留 → 抬起
 */
static void do_tap(int x, int y)
{
    if (!module_ready || !fts_input_dev)
        return;

    x = jitter(x, JITTER_RANGE);
    y = jitter(y, JITTER_RANGE);

    fts_touch_down(x, y);

    /* 模拟按压延迟 50-120ms */
    usleep_range_state(50000, 120000, 2);

    fts_touch_up();
}

/*
 * 整数贝塞尔曲线滑动（无浮点运算）
 * 使用二次贝塞尔: B(t) = (1-t)^2*P0 + 2(1-t)t*Pc + t^2*P1
 * 用固定点整数: 乘 1000 运算, 最后除 1000000
 */
static void do_swipe(int x1, int y1, int x2, int y2, int duration_ms)
{
    int steps, i, ti, t2, mt, mt2, px, py;
    int cx, cy;

    if (!module_ready || !fts_input_dev)
        return;
    if (duration_ms < 50)
        duration_ms = 50;
    if (duration_ms > 5000)
        duration_ms = 5000;

    steps = duration_ms / 8;  /* ~8ms/frame */
    if (steps < 4)
        steps = 4;
    if (steps > 200)
        steps = 200;

    /* 控制点：起点和终点中间加入微小偏移 */
    cx = (x1 + x2) / 2 + (prandom_u32() % 21 - 10);
    cy = (y1 + y2) / 2 + (prandom_u32() % 21 - 10);

    fts_touch_down(x1, y1);
    usleep_range_state(20000, 40000, 2);

    for (i = 1; i <= steps; i++) {
        /* ti = i/steps * 1000 (固定点整数, 范围 0-1000) */
        ti = (i * 1000) / steps;
        /* mt = 1000 - ti */
        mt = 1000 - ti;
        /* t2 = ti^2 / 1000, mt2 = mt^2 / 1000 */
        t2 = (ti * ti) / 1000;
        mt2 = (mt * mt) / 1000;

        /* B(t) = (mt2*P0 + 2*mt*ti*Pc + t2*P1) / 1000000 */
        px = (mt2 * x1 + 2 * mt * ti / 1000 * cx + t2 * x2) / 1000;
        py = (mt2 * y1 + 2 * mt * ti / 1000 * cy + t2 * y2) / 1000;

        px = jitter(px, 2);
        py = jitter(py, 2);

        fts_touch_move(px, py);
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
    int x, y, x2, y2, dur;

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

    case 'D': /* Down: D x y */
        if (sscanf(kbuf, "D %d %d", &x, &y) == 2)
            fts_touch_down(jitter(x, 2), jitter(y, 2));
        break;

    case 'M': /* Move: M x y */
        if (sscanf(kbuf, "M %d %d", &x, &y) == 2)
            fts_touch_move(jitter(x, 2), jitter(y, 2));
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
        msg = "OK ready fts\n";
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
    /* 安全：如果还有触摸保持，强制释放 */
    if (touch_down) {
        pr_warn("[touch_inject] closing while touch still down, releasing\n");
        fts_touch_up();
    }
    return 0;
}

static const struct file_operations touch_fops = {
    .owner   = NULL,
    .open    = touch_open,
    .release = touch_release,
    .read    = touch_read,
    .write   = touch_write,
};

/* ============================================================
 * Input handler — 自动找到 FTS 的 input_dev
 * ============================================================ */

static int fts_finder_connect(struct input_handler *handler,
                              struct input_dev *dev,
                              const struct input_device_id *id)
{
    if (dev->name && strcmp(dev->name, "fts") == 0) {
        fts_input_dev = dev;
        pr_info("[touch_inject] FTS input_dev captured: %px\n", dev);
        pr_info("[touch_inject]   phys=%s\n",
                dev->phys ? dev->phys : "(null)");
        pr_info("[touch_inject]   ABS_X: 0..%d ABS_Y: 0..%d\n",
                dev->absinfo ? dev->absinfo[ABS_MT_POSITION_X].maximum : -1,
                dev->absinfo ? dev->absinfo[ABS_MT_POSITION_Y].maximum : -1);

        if (!module_ready) {
            module_ready = true;
            pr_info("[touch_inject] Module ready for injection.\n");
        }
    }
    return 0;
}

static void fts_finder_disconnect(struct input_handle *handle)
{
    if (handle->dev == fts_input_dev) {
        pr_warn("[touch_inject] FTS device disconnecting!\n");
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
    pr_info("[touch_inject] Loading on Xiaomi 13 (fuxi)\n");
    pr_info("[touch_inject] Kernel: %s\n", UTS_RELEASE);

    /* 步骤1: 注册 input handler，自动匹配 fts 设备 */
    ret = input_register_handler(&fts_finder_handler);
    if (ret) {
        pr_err("[touch_inject] FAILED to register input handler: %d\n", ret);
        return ret;
    }

    if (!fts_input_dev) {
        pr_warn("[touch_inject] WARNING: No 'fts' device found immediately\n");
        pr_warn("[touch_inject] It may appear later (driver not loaded yet?)\n");
        /* 不返回错误，因为 FTS 驱动可能在这个模块之后加载 */
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
    pr_info("[touch_inject] Module loaded successfully.\n");
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

    device_destroy(touch_class, dev_num);
    class_destroy(touch_class);
    cdev_del(&touch_cdev);
    unregister_chrdev_region(dev_num, 1);
    input_unregister_handler(&fts_finder_handler);
    fts_input_dev = (void *)0;

    pr_info("[touch_inject] Module unloaded.\n");
}

module_init(touch_inject_init);
module_exit(touch_inject_exit);
