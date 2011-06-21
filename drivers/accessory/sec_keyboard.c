
#include <linux/input.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/earlysuspend.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <asm/gpio.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <mach/gpio-sec.h>
#include <linux/slab.h>
#include <linux/30pin_con.h>
#include <linux/sec_keyboard_struct.h>
#include "sec_keyboard.h"

struct dock_keyboard_drvdata *g_data;


extern struct class *sec_class;
extern void dock_keyboard_tx(u8 val);
extern int change_console_baud_rate(unsigned int baud);

static void timer_work(struct work_struct *work)
{
	struct dock_keyboard_drvdata *data = container_of(work,
			struct dock_keyboard_drvdata, work_timer);

	int error;
	if (data->kl == UNKOWN_KEYLAYOUT ) {
		data->acc_power(0, false);

		/* Set baud rate for the keyboard uart */
		error = change_console_baud_rate(115200);
		if (error<0)
			printk(KERN_ERR "[Keyboard] Couldn't modify the baud rate.\n");

		/* if the uart is set as cp path, the path should be switched to ap path.*/
		if (!data->pre_uart_path) {
			gpio_direction_output(GPIO_UART_SEL, 0);
			pr_info("[Keyboard] console uart path is switched to CP.\n");
		}
	}

}

static void keyboard_timer(unsigned long _data)
{
/* this part will be run in the disable() func */
	struct dock_keyboard_drvdata *data = (struct dock_keyboard_drvdata *)_data;

	if (!work_pending(&data->work_timer))
		schedule_work(&data->work_timer);
}

void remapkey_timer(unsigned long _data)
{
	struct dock_keyboard_drvdata *data = (struct dock_keyboard_drvdata *)_data;
	unsigned int keycode = 0;
	if (data->pressed[0x45] || data->pressed[0x48]) {
		keycode = data->keycode[data->remap_key];
		input_report_key(data->input_dev, keycode, 1);
		input_sync(data->input_dev);
	} else {

		if (0x48 == data->remap_key)
			keycode = KEY_NEXTSONG;
		else
			keycode = KEY_PREVIOUSSONG;

		input_report_key(data->input_dev, keycode, 1);
		input_report_key(data->input_dev, keycode, 0);
		input_sync(data->input_dev);
	}
	data->remap_key = 0;
}

void release_all_keys(struct dock_keyboard_drvdata *data)
{
	int i;
	pr_info("[Keyboard] Release the pressed keys.\n");
	for (i = 0; i < KEYBOARD_MAX; i++) {
		if (data->pressed[i]) {
			input_report_key(data->input_dev, data->keycode[i], 0);
			data->pressed[i] = false;
		}
		input_sync(data->input_dev);
	}
}

static void key_event_work(struct work_struct *work)
{
	struct dock_keyboard_drvdata *data = container_of(work,
			struct dock_keyboard_drvdata, work_msg);
	bool press;
	unsigned int keycode;
	unsigned char scan_code;

	mutex_lock(&data->mutex);
	while (data->buf_front != data->buf_rear) {
		scan_code = data->key_buf[data->buf_front];
		data->buf_front++;
		if (data->buf_front > MAX_BUF)
			data->buf_front = 0;

		/* keyboard driver need the contry code*/
		if (data->kl == UNKOWN_KEYLAYOUT) {
			switch (scan_code) {
			case US_KEYBOARD:
			data->kl = US_KEYLAYOUT;
			data->keycode[49] = KEY_BACKSLASH;
			/* for the wakeup state*/
			data->pre_kl = data->kl;
			pr_info("[Keyboard] US keyboard is attacted.\n");
			break;

			case UK_KEYBOARD:
			data->kl = UK_KEYLAYOUT;
			data->keycode[49] = KEY_NUMERIC_POUND;
			/* for the wakeup state*/
			data->pre_kl = data->kl;
			pr_info("[Keyboard] UK keyboard is attacted.\n");
			break;

			default:

			pr_info("[Keyboard] Unkown key layout : %x\n", scan_code);
			break;
			}
		} else {
			/* Caps lock led on/off */
			if (scan_code == 0xca || scan_code == 0xcb || scan_code == 0xeb || scan_code == 0xec)
				;/* Ignore */
			else {
				press = ((scan_code & 0x80) != 0x80);
				keycode = (scan_code & 0x7f);

				if (keycode >= KEYBOARD_MIN || keycode <= KEYBOARD_MAX) {
					data->pressed[keycode] = press;

					/* for the remap keys*/
					if (keycode == 0x45 || keycode == 0x48) {
						if (press) {
							data->remap_key = keycode;
							mod_timer(&data->key_timer, jiffies + HZ/3);
						} else {
							if ( 0 == data->remap_key) {
								input_report_key(data->input_dev, data->keycode[keycode], press);
								input_sync(data->input_dev);
							}
						}
					} else {
						input_report_key(data->input_dev, data->keycode[keycode], press);
						input_sync(data->input_dev);
					}
				}
			}
		}
	}
	mutex_unlock(&data->mutex);
}

static void led_work(struct work_struct *work)
{
	struct dock_keyboard_drvdata *data = container_of(work,
			struct dock_keyboard_drvdata, work_led);

	if (data->led_on)
		dock_keyboard_tx(0xca);
	else
		dock_keyboard_tx(0xcb);
}

#if 0
static void get_scancode(struct kbd_callbacks *cb, unsigned int key_code)
{
	struct dock_keyboard_drvdata *data = container_of(cb,
			struct dock_keyboard_drvdata, callbacks);
#else
void send_keyevent(unsigned int key_code)
{
	struct dock_keyboard_drvdata *data = g_data;
	//pr_info("[Keyboard] key_code : %x\n", key_code);
#endif
	data->key_buf[data->buf_rear]  = key_code;
	data->buf_rear++;
	if (data->buf_rear > MAX_BUF)
		data->buf_rear = 0;

	if (!work_pending(&data->work_msg))
		schedule_work(&data->work_msg);
}

#if 0
static int check_handshake(struct kbd_callbacks *cb, int val)
{
	struct dock_keyboard_drvdata *data = container_of(cb,
			struct dock_keyboard_drvdata, callbacks);
#else
int check_keyboard_dock(bool val)
{
	struct dock_keyboard_drvdata *data = g_data;
#endif

	int try_cnt = 0;
	int error = 0;
	int max_cnt = 6;

	pr_info("[Keyboard] %s : %s \n", __func__, val? "attach":"dettach");

	if (!val)
		data->dockconnected = false;
	else {
		data->pre_connected = true;

		/*for checking handshake*/
		data->connected_time = jiffies_to_msecs(jiffies);

		/* wakeup by keyboard dock */
		if ((data->connected_time - data->disconnected_time) < 1000) {
			data->kl = data->pre_kl;
			pr_info("[Keyboard] kl : %d\n", data->pre_kl);
		} else
			data->pre_kl = UNKOWN_KEYLAYOUT;
#if 0
		if (data->enable) {
			error = data->enable();
			if (error<0)
				pr_err("[Keyboard] Failed to init the keyboard dock!!\n");
			else	{
				data->task = kthread_run(dock_keyboard_thread, data, "dock_keyboard_thread");
				if (data->task == NULL) {
					pr_err("[Keyboard] Failed to create keyboard thread\n");
					return 0;
				}
			}
		} else
			return 0;
#endif

/* this part will be run in the enable() func */
#if 1
		data->acc_power(0, false);
		msleep(300);

		/* if the uart is set as cp path, the path should be switched to ap path.*/
		data->pre_uart_path = gpio_get_value(GPIO_UART_SEL);
		if (!data->pre_uart_path)	{
			gpio_direction_output(GPIO_UART_SEL, 1);
			pr_info("[Keyboard] console uart path is switched to AP.\n");
		}
		/* Set baud rate for the keyboard uart */
		error = change_console_baud_rate(9600);
		if (error<0)
			printk(KERN_ERR "[Keyboard] Couldn't modify the baud rate.\n");

		msleep(10);
		data->acc_power(0, true);

#endif

		/* try to get handshake data */
		for (try_cnt=0; try_cnt<max_cnt; try_cnt++) {
			msleep(100);
			if (data->kl != UNKOWN_KEYLAYOUT) {
				data->dockconnected = true;
				break;
			}
#if 0
			/* the accessory is dettached. */
			if (gpio_get_value(data->gpio)) {
				data->dockconnected = false;
				break;
			} else
				pr_info("[Keyboard] test\n");
#endif
		}
	}

	if (data->dockconnected)
		return 1;
	else	{
		if (data->pre_connected) {
			/* stop the thread and clear the buffer*/
			data->buf_front = data->buf_rear = 0;

			data->dockconnected = false;
			mod_timer(&data->timer, jiffies + HZ/2);

			data->kl = UNKOWN_KEYLAYOUT;
			data->pre_connected = false;
			data->disconnected_time = jiffies_to_msecs(jiffies);
			release_all_keys(data);
		}
		return 0;
	}
}

EXPORT_SYMBOL(send_keyevent);
EXPORT_SYMBOL(check_keyboard_dock);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void keyboard_early_suspend(struct early_suspend *early_sus)
{
	struct dock_keyboard_drvdata *data = container_of(early_sus,
		struct dock_keyboard_drvdata, early_suspend);

	if (data->kl != UNKOWN_KEYLAYOUT )
		dock_keyboard_tx(0x10);
}

static void keyboard_late_resume(struct early_suspend *early_sus)
{
	struct dock_keyboard_drvdata *data = container_of(early_sus,
		struct dock_keyboard_drvdata, early_suspend);

	if (data->kl != UNKOWN_KEYLAYOUT )
		pr_info("[Keyboard] %s", __func__);
}
#endif

static int sec_keyboard_event(struct input_dev *dev,
			unsigned int type, unsigned int code, int value)
{
	/*
	struct sec_keyboard_drvdata *data = input_get_drvdata(dev);
	*/

	switch (type) {
	case EV_LED:
		if (value)
			dock_keyboard_tx(0xca);
		else
			dock_keyboard_tx(0xcb);
		return 0;
	}
	return -1;
}

static int __devinit dock_keyboard_probe(struct platform_device *pdev)
{
	struct dock_keyboard_platform_data *pdata = pdev->dev.platform_data;
	struct dock_keyboard_drvdata *ddata;
	struct input_dev *input;
	int i, error;

	ddata = kzalloc(sizeof(struct dock_keyboard_drvdata), GFP_KERNEL);
	if (NULL == ddata) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	input = input_allocate_device();
	if (NULL == input) {
		printk(KERN_ERR "[Keyboard] Fail to allocate input device.\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	ddata->input_dev = input;
	ddata->acc_power = pdata->acc_power;
	ddata->buf_front=ddata->buf_rear=0;
	ddata->disconnected_time=0;
	ddata->led_on=false;
	ddata->dockconnected=false;
	ddata->pre_connected=false;
	ddata->remap_key=0;
	ddata->kl = UNKOWN_KEYLAYOUT;
	memcpy(ddata->keycode, dock_keycodes, sizeof(dock_keycodes));

	mutex_init(&ddata->mutex);
	INIT_WORK(&ddata->work_msg, key_event_work);
	INIT_WORK(&ddata->work_led, led_work);
	INIT_WORK(&ddata->work_timer, timer_work);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdev->name;
	input->dev.parent = &pdev->dev;
	input->id.bustype = BUS_RS232;
	input->event 		= sec_keyboard_event;

	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(EV_LED, input->evbit);
	set_bit(LED_CAPSL, input->ledbit);
	/* framework doesn't use repeat event */
	/* set_bit(EV_REP, input->evbit); */

	for (i = 0; i < KEYBOARD_SIZE; i++) {
		if (KEY_RESERVED != ddata->keycode[i])
			input_set_capability(input, EV_KEY, ddata->keycode[i]);
	}

	/* for the UK keyboard */
	input_set_capability(input, EV_KEY, KEY_NUMERIC_POUND);

	/* for the remaped keys */
	input_set_capability(input, EV_KEY, KEY_NEXTSONG);
	input_set_capability(input, EV_KEY, KEY_PREVIOUSSONG);

	error = input_register_device(input);
	if (error < 0) {
		printk(KERN_ERR "[Keyboard] Fail to register input device.\n");
		error = -ENOMEM;
		goto err_input_allocate_device;
	}


#ifdef CONFIG_HAS_EARLYSUSPEND
	ddata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ddata->early_suspend.suspend = keyboard_early_suspend;
	ddata->early_suspend.resume = keyboard_late_resume;
	register_early_suspend(&ddata->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	init_timer(&ddata->timer);
	ddata->timer.expires = jiffies + HZ;
	ddata->timer.function = keyboard_timer;
	ddata->timer.data = (unsigned long)ddata;

	init_timer(&ddata->key_timer);
	ddata->key_timer.expires = jiffies + HZ/3;
	ddata->key_timer.function = remapkey_timer;
	ddata->key_timer.data = (unsigned long)ddata;

	g_data = ddata;

	return 0;
	err_input_allocate_device:
	input_free_device(input);
	err_free_mem:
	kfree(ddata);
	return error;

}

static int __devexit dock_keyboard_remove(struct platform_device *pdev)
{
	struct dock_keyboard_drvdata *data = pdev->dev.platform_data;
	input_unregister_device(data->input_dev);
	return 0;
}

static struct platform_driver dock_keyboard_device_driver =
{
	.probe		= dock_keyboard_probe,
	.remove	= __devexit_p(dock_keyboard_remove),
	.driver		=
	{
		.name	= "sec_keyboard",
		.owner	= THIS_MODULE,
	}
};

static int __init dock_keyboard_init(void)
{
	return platform_driver_register(&dock_keyboard_device_driver);
}

static void __exit dock_keyboard_exit(void)
{
	platform_driver_unregister(&dock_keyboard_device_driver);
}

module_init(dock_keyboard_init);
module_exit(dock_keyboard_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SEC P series Dock Keyboard driver");
