

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/30pin_con.h>
#include <linux/switch.h>
#include <linux/wakelock.h>

#include <asm/irq.h>
#include <linux/mfd/tps6586x.h>

#if defined(CONFIG_MACH_SAMSUNG_P5) || (CONFIG_MACH_SAMSUNG_P5WIFI)
#include <mach/gpio-p5.h>
#endif


#define SUBJECT "CONNECTOR_DRIVER"

#define ACC_CONDEV_DBG(format, ...) \
	pr_info("[ "SUBJECT " (%s,%d) ] " format "\n", \
		__func__, __LINE__, ## __VA_ARGS__);

#define DETECTION_INTR_DELAY	(get_jiffies_64() + (HZ*15)) /* 20s */

enum accessory_type {
	ACCESSORY_NONE = 0,
	ACCESSORY_OTG,
	ACCESSORY_LINEOUT,
	ACCESSORY_CARMOUNT,
	ACCESSORY_UNKNOWN,
};

enum dock_type {
	DOCK_NONE = 0,
	DOCK_DESK,
	DOCK_KEYBOARD,
};

enum uevent_dock_type {
	UEVENT_DOCK_NONE = 0,
	UEVENT_DOCK_DESK,
	UEVENT_DOCK_CAR,
	UEVENT_DOCK_KEYBOARD = 9,
};

struct acc_con_info {
	struct device *acc_dev;
	struct acc_con_platform_data *pdata;
	struct switch_dev dock_switch;
	struct switch_dev ear_jack_switch;
	enum accessory_type current_accessory;
	enum dock_type current_dock;
	int accessory_irq;
	int dock_irq;
	int mhl_irq;
	bool mhl_pwr_state;
	struct wake_lock wake_lock;
};

extern	s16 stmpe811_adc_get_value(u8 channel);

#ifdef CONFIG_MHL_SII9234
#include "sii9234.h"
#endif
static int connector_detect_change(void)
{
	int i;
	u32 adc = 0, adc_sum = 0;
	u32 adc_buff[5] = {0};
	u32 mili_volt;
	u32 adc_min = 0;
	u32 adc_max = 0;

	for (i = 0; i < 5; i++) {
		/*change this reading ADC function  */
		mili_volt =  (u32)stmpe811_adc_get_value(7);
		adc_buff[i] = mili_volt;
		adc_sum += adc_buff[i];
		if (i == 0) {
			adc_min = adc_buff[0];
			adc_max = adc_buff[0];
		} else {
			if (adc_max < adc_buff[i])
				adc_max = adc_buff[i];
			else if (adc_min > adc_buff[i])
				adc_min = adc_buff[i];
		}
		msleep(20);
	}
	adc = (adc_sum - adc_max - adc_min)/3;
	ACC_CONDEV_DBG("ACCESSORY_ID : ADC value = %d\n", adc);
	return (int)adc;
}


static ssize_t MHD_check_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	int res;
//	TVout_LDO_ctrl(true);
	if(!MHD_HW_IsOn())
	{
		sii9234_tpi_init();
		res = MHD_Read_deviceID();
		MHD_HW_Off();
	}
	else
	{
		sii9234_tpi_init();
		res = MHD_Read_deviceID();
	}

	count = sprintf(buf,"%d\n", res );
//	TVout_LDO_ctrl(false);
	return count;
}

static ssize_t MHD_check_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk("input data --> %s\n", buf);

	return size;
}

static DEVICE_ATTR(MHD_file, 0660, MHD_check_read, MHD_check_write);


static void acc_dock_check(struct acc_con_info *acc, bool connected)
{
	char *env_ptr;
	char *stat_ptr;
	char *envp[3];

	if (acc->current_dock == DOCK_KEYBOARD)
		env_ptr = "DOCK=keyboard";
	else if (acc->current_dock == DOCK_DESK)
		env_ptr = "DOCK=desk";
	else
		env_ptr = "DOCK=unknown";

	if (!connected) {
		stat_ptr = "STATE=offline";
		acc->current_dock = DOCK_NONE;
	} else {
		stat_ptr = "STATE=online";
	}

	envp[0] = env_ptr;
	envp[1] = stat_ptr;
	envp[2] = NULL;
	kobject_uevent_env(&acc->acc_dev->kobj, KOBJ_CHANGE, envp);
	ACC_CONDEV_DBG("%s : %s", env_ptr, stat_ptr);
}

#if defined(CONFIG_MACH_SAMSUNG_P5) || (CONFIG_MACH_SAMSUNG_P5WIFI)
void hpd_force_low(void)
{
	if (system_rev >= 6) {
		tegra_gpio_enable(GPIO_HDMI_HPD);
		gpio_request(GPIO_HDMI_HPD, "hdmi_hpd");
		gpio_direction_output(GPIO_HDMI_HPD, 0);
		gpio_direction_input(GPIO_HDMI_HPD);
		gpio_free(GPIO_HDMI_HPD);
	} else {
		tegra_gpio_enable(GPEX_GPIO_P1);
		gpio_request(GPEX_GPIO_P1, "hdmi_hpd");
		gpio_direction_output(GPEX_GPIO_P1, 0);
		gpio_direction_input(GPEX_GPIO_P1);
		gpio_free(GPEX_GPIO_P1);
	}

}
#endif

#ifdef CONFIG_SEC_KEYBOARD_DOCK
extern int check_keyboard_dock(bool);
#endif

irqreturn_t acc_con_interrupt(int irq, void *ptr)
{
	struct acc_con_info *acc = ptr;
	int cur_state;

	ACC_CONDEV_DBG("");

	/* check the flag MHL or keyboard */
	cur_state = gpio_get_value(acc->pdata->accessory_irq_gpio);
	if (cur_state == 1) {
		if (acc->current_dock == DOCK_NONE) {
			return;
		}

		ACC_CONDEV_DBG("[30PIN] dock station detached!!!");
		switch_set_state(&acc->dock_switch, UEVENT_DOCK_NONE);
#ifdef CONFIG_SEC_KEYBOARD_DOCK
		check_keyboard_dock(false);
#endif
#ifdef CONFIG_MHL_SII9234
		/*call MHL deinit */
		if (acc->mhl_pwr_state) {
			MHD_HW_Off();
#if defined(CONFIG_MACH_SAMSUNG_P5) || (CONFIG_MACH_SAMSUNG_P5WIFI)
			hpd_force_low();
#endif
			acc->mhl_pwr_state = false;
		}
		/*TVout_LDO_ctrl(false); */
#endif
		acc_dock_check(acc, false);
		set_irq_type(irq,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT);
	} else if (0 == cur_state) {

#ifdef CONFIG_SEC_KEYBOARD_DOCK
		if (check_keyboard_dock(true)) {
			acc->current_dock = DOCK_KEYBOARD;
			ACC_CONDEV_DBG("[30PIN] keyboard dock station attached!!!");
			switch_set_state(&acc->dock_switch, UEVENT_DOCK_KEYBOARD);
		}
		else
#endif
		{
#ifdef CONFIG_MHL_SII9234
			ACC_CONDEV_DBG("[30PIN] desktop dock station attached!!!");
			switch_set_state(&acc->dock_switch, UEVENT_DOCK_DESK);
			acc->current_dock = DOCK_DESK;
#endif
			if(acc->pdata->acc_power) {
				ACC_CONDEV_DBG("[30PIN] otg 5v up!!!");
				acc->pdata->acc_power(2, true);
			} else
				printk(KERN_ERR"[30PIN][ERROR] acc_power is null !!!");
		}
		if (!acc->mhl_pwr_state) {
			sii9234_tpi_init();
			acc->mhl_pwr_state = true;
		}
		acc_dock_check(acc, true);
		set_irq_type(irq,
			IRQF_TRIGGER_HIGH | IRQF_ONESHOT);
	}

	return IRQ_HANDLED;
}

static int acc_con_interrupt_init(struct acc_con_info *acc)
{
	int ret;
	ACC_CONDEV_DBG("");

	gpio_request(acc->pdata->accessory_irq_gpio, "accessory");
	gpio_direction_input(acc->pdata->accessory_irq_gpio);
	tegra_gpio_enable(acc->pdata->accessory_irq_gpio);
	acc->accessory_irq = gpio_to_irq(acc->pdata->accessory_irq_gpio);
	ret = request_threaded_irq(acc->accessory_irq, NULL, acc_con_interrupt,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			"accessory_detect", acc);
	if (ret) {
		ACC_CONDEV_DBG("request_irq(accessory_irq) return : %d\n", ret);
	}

	return ret;
}

void acc_notified(struct acc_con_info *acc, int acc_adc)
{
	char *env_ptr;
	char *stat_ptr;
	char *envp[3];

	if (acc_adc != false) {
		if ((2600 < acc_adc) && (2800 > acc_adc)) {
			env_ptr = "ACCESSORY=OTG";
			acc->current_accessory = ACCESSORY_OTG;
		} else if ((1100 < acc_adc) && (1300 > acc_adc)) {
			env_ptr = "ACCESSORY=lineout";
			acc->current_accessory = ACCESSORY_LINEOUT;
			switch_set_state(&acc->ear_jack_switch, 1);
		} else if ((3000 < acc_adc) && (4100 > acc_adc)) { /* for nonideal adc value when insert earjeck */
			env_ptr = "ACCESSORY=lineout";
			acc->current_accessory = ACCESSORY_LINEOUT;
			switch_set_state(&acc->ear_jack_switch, 1);
		} else if ((1700 < acc_adc) && (2300 > acc_adc)) { /* for nonideal adc value when insert earjeck */
			env_ptr = "ACCESSORY=lineout";
			acc->current_accessory = ACCESSORY_LINEOUT;
			switch_set_state(&acc->ear_jack_switch, 1);
		}
		else if ((1300 < acc_adc) && (1500 > acc_adc)) {
			env_ptr = "ACCESSORY=carmount";
			acc->current_accessory = ACCESSORY_CARMOUNT;
			ACC_CONDEV_DBG("[30PIN] car dock station attached!!!");
			switch_set_state(&acc->dock_switch, UEVENT_DOCK_CAR);
		} else {
//			env_ptr = "ACCESSORY=unknown";
//			acc->current_accessory = ACCESSORY_UNKNOWN;
			ACC_CONDEV_DBG("wrong adc range filter !!");
			return;
		}
		stat_ptr = "STATE=online";
		envp[0] = env_ptr;
		envp[1] = stat_ptr;
		envp[2] = NULL;
		if (acc->current_accessory == ACCESSORY_OTG) {
			if (acc->pdata->usb_ldo_en)
				acc->pdata->usb_ldo_en(1, 0);
			msleep(50);
			if (acc->pdata->otg_en)
				acc->pdata->otg_en(1);
		}
		kobject_uevent_env(&acc->acc_dev->kobj, KOBJ_CHANGE, envp);
		ACC_CONDEV_DBG("%s : %s", env_ptr, stat_ptr);
	} else {
		if (acc->current_accessory == ACCESSORY_OTG)
			env_ptr = "ACCESSORY=OTG";
		else if (acc->current_accessory == ACCESSORY_LINEOUT) {
			env_ptr = "ACCESSORY=lineout";
			switch_set_state(&acc->ear_jack_switch, 0);
		}
		else if (acc->current_accessory == ACCESSORY_CARMOUNT)
			env_ptr = "ACCESSORY=carmount";
		else
			env_ptr = "ACCESSORY=unknown";

		if ((acc->current_accessory == ACCESSORY_OTG) &&
			acc->pdata->otg_en) {
			acc->pdata->otg_en(0);
		} else {
			if (acc->pdata->acc_power)
				acc->pdata->acc_power(2, false);
		}

		acc->current_accessory = ACCESSORY_NONE;
		stat_ptr = "STATE=offline";
		envp[0] = env_ptr;
		envp[1] = stat_ptr;
		envp[2] = NULL;
		kobject_uevent_env(&acc->acc_dev->kobj, KOBJ_CHANGE, envp);
		ACC_CONDEV_DBG("%s : %s", env_ptr, stat_ptr);
	}
}

irqreturn_t acc_ID_interrupt(int irq, void *ptr)
{
	struct acc_con_info *acc = ptr;
	int acc_ID_val, adc_val;
	static int post_state = -1;

	ACC_CONDEV_DBG("");

	acc_ID_val = gpio_get_value(acc->pdata->dock_irq_gpio);
	ACC_CONDEV_DBG("IRQ_DOCK_GPIO is %d", acc_ID_val);

	if (acc_ID_val == 1) {
		if (post_state != 1) {
			post_state = 1;
			ACC_CONDEV_DBG("Accessory detached");
	//		switch_set_state(&acc->ear_jack_switch, 0);
			acc_notified(acc, false);
			set_irq_type(irq, IRQF_TRIGGER_LOW | IRQF_ONESHOT);
		} else
			ACC_CONDEV_DBG("[WARNING] duplication detache --> ignore");
	} else if (acc_ID_val == 0) {
		if (post_state != 0) {
			wake_lock(&acc->wake_lock);
			post_state = 0;
			msleep(420); /* workaround for jack */
			ACC_CONDEV_DBG("Accessory attached");
	//		switch_set_state(&acc->ear_jack_switch, 1);
			adc_val = connector_detect_change();
			acc_notified(acc, adc_val);
			set_irq_type(irq, IRQF_TRIGGER_HIGH | IRQF_ONESHOT);
			wake_unlock(&acc->wake_lock);
		} else {
			adc_val = connector_detect_change();
			ACC_CONDEV_DBG("[WARNING] duplication attache (adc = %d) --> ignore", adc_val);
		}
	}

	return IRQ_HANDLED;
}

static int acc_ID_interrupt_init(struct acc_con_info *acc)
{
	int ret;

	ACC_CONDEV_DBG("");

	gpio_request(acc->pdata->dock_irq_gpio, "dock");
	gpio_direction_input(acc->pdata->dock_irq_gpio);
	tegra_gpio_enable(acc->pdata->dock_irq_gpio);
	acc->dock_irq = gpio_to_irq(acc->pdata->dock_irq_gpio);
	ret = request_threaded_irq(acc->dock_irq, NULL, acc_ID_interrupt,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			"dock_detect", acc);
	if (ret) {
		ACC_CONDEV_DBG("request_irq(dock_irq) return : %d\n", ret);
	}
	enable_irq_wake(acc->dock_irq);
	return ret;
}

static int acc_con_probe(struct platform_device *pdev)
{
	struct acc_con_info *acc;
	struct acc_con_platform_data *pdata = pdev->dev.platform_data;
	int	retval;

	ACC_CONDEV_DBG("");

	if (pdata == NULL) {
		pr_err("%s: no pdata\n", __func__);
		return -ENODEV;
	}

	acc = kzalloc(sizeof(*acc), GFP_KERNEL);
	if (!acc)
		return -ENOMEM;

	acc->pdata = pdata;
	acc->current_dock = DOCK_NONE;
	acc->current_accessory = ACCESSORY_NONE;
	acc->mhl_irq = gpio_to_irq(pdata->mhl_irq_gpio);
	acc->mhl_pwr_state = false;

	dev_set_drvdata(&pdev->dev, acc);

	acc->acc_dev = &pdev->dev;
#ifdef CONFIG_MHL_SII9234
	retval = i2c_add_driver(&SII9234A_i2c_driver);
	if (retval) {
		pr_err("[MHL SII9234A] can't add i2c driver\n");
		goto err_i2c_a;
	} else {
		pr_info("[MHL SII9234A] add i2c driver\n");
	}

	retval = i2c_add_driver(&SII9234B_i2c_driver);
	if (retval) {
		pr_err("[MHL SII9234B] can't add i2c driver\n");
		goto err_i2c_b;
	} else {
		pr_info("[MHL SII9234B] add i2c driver\n");
	}

	retval = i2c_add_driver(&SII9234C_i2c_driver);
	if (retval) {
		pr_err("[MHL SII9234C] can't add i2c driver\n");
		goto err_i2c_c;
	} else {
		pr_info("[MHL SII9234C] add i2c driver\n");
	}

	retval = i2c_add_driver(&SII9234_i2c_driver);
	if (retval) {
		pr_err("[MHL SII9234] can't add i2c driver\n");
		goto err_i2c;
	} else {
		pr_info("[MHL SII9234] add i2c driver\n");
	}

	/*MHD_HW_Off(); */
	/*sii9234_tpi_init(); */
	/*MHD_Read_deviceID(); */
#endif

	acc->dock_switch.name = "dock";
	retval = switch_dev_register(&acc->dock_switch);
	if (retval < 0)
		goto err_sw_dock;

	acc->ear_jack_switch.name = "usb_audio";
	retval = switch_dev_register(&acc->ear_jack_switch);
	if (retval < 0)
		goto err_sw_jack;

	wake_lock_init(&acc->wake_lock, WAKE_LOCK_SUSPEND, "30pin_con");
	retval = acc_con_interrupt_init(acc);
	if (retval != 0)
		goto err_irq_dock;

	retval = acc_ID_interrupt_init(acc);
	if (retval != 0)
		goto err_irq_acc;
	if (device_create_file(acc->acc_dev, &dev_attr_MHD_file) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_MHD_file.attr.name);

	return 0;

err_irq_acc:
	free_irq(acc->accessory_irq, acc);
err_irq_dock:
	switch_dev_unregister(&acc->ear_jack_switch);
err_sw_jack:
	switch_dev_unregister(&acc->dock_switch);
err_sw_dock:
	i2c_del_driver(&SII9234_i2c_driver);
err_i2c:
	i2c_del_driver(&SII9234C_i2c_driver);
err_i2c_c:
	i2c_del_driver(&SII9234B_i2c_driver);
err_i2c_b:
	i2c_del_driver(&SII9234A_i2c_driver);
err_i2c_a:
	kfree(acc);
}

static int acc_con_remove(struct platform_device *pdev)
{
	struct acc_con_info *acc = platform_get_drvdata(pdev);
	ACC_CONDEV_DBG("");

	free_irq(acc->accessory_irq, acc);
	free_irq(acc->dock_irq, acc);
#ifdef CONFIG_MHL_SII9234
	i2c_del_driver(&SII9234A_i2c_driver);
	i2c_del_driver(&SII9234B_i2c_driver);
	i2c_del_driver(&SII9234C_i2c_driver);
	i2c_del_driver(&SII9234_i2c_driver);
#endif
	switch_dev_unregister(&acc->dock_switch);
	switch_dev_unregister(&acc->ear_jack_switch);
	kfree(acc);
	return 0;
}

static int acc_con_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct acc_con_info *acc = platform_get_drvdata(pdev);
	ACC_CONDEV_DBG("");
#ifdef CONFIG_MHL_SII9234
	if (acc->mhl_pwr_state) {
		MHD_HW_Off();   /*call MHL deinit */
		acc->mhl_pwr_state = false;
	}
#endif
	return 0;
}

static int acc_con_resume(struct platform_device *pdev)
{
	struct acc_con_info *acc = platform_get_drvdata(pdev);
	ACC_CONDEV_DBG("");
#ifdef CONFIG_MHL_SII9234
	if (acc->current_dock != DOCK_NONE) {
		sii9234_tpi_init();  /* call MHL init */
		acc->mhl_pwr_state = true;
	}
#endif
	return 0;
}

static struct platform_driver acc_con_driver = {
	.probe		= acc_con_probe,
	.remove		= acc_con_remove,
	.suspend	= acc_con_suspend,
	.resume		= acc_con_resume,
	.driver		= {
		.name		= "acc_con",
		.owner		= THIS_MODULE,
	},
};

static int __init acc_con_init(void)
{
	ACC_CONDEV_DBG("");

	return platform_driver_register(&acc_con_driver);
}

static void __exit acc_con_exit(void)
{
	platform_driver_unregister(&acc_con_driver);
}

late_initcall(acc_con_init);
module_exit(acc_con_exit);

MODULE_AUTHOR("Kyungrok Min <gyoungrok.min@samsung.com>");
MODULE_DESCRIPTION("acc connector driver");
MODULE_LICENSE("GPL");
