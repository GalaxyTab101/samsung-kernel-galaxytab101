/*
 * arch/arm/mach-tegra/tegra2_mc.c
 *
 * Memory controller bandwidth profiling interface
 *
 * Copyright (c) 2009-2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/sysdev.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/parser.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/init.h>
#include <mach/iomap.h>
#include <asm/uaccess.h>

#include "tegra2_mc.h"

static void stat_start(void);
static void stat_stop(void);
static void stat_log(void);

static struct hrtimer sample_timer;

#define MC_COUNTER_INITIALIZER()			\
	{						\
		.enabled = false,			\
		.reschedule = false,			\
		.period = 10,				\
		.mode = FILTER_CLIENT,			\
		.address_low = 0,			\
		.address_length_1 = 0xfffffffful,	\
		.address_window_size_1 = PAGE_SIZE,	\
		.client_number = 0,			\
	}

static struct tegra_mc_counter mc_counter0 = MC_COUNTER_INITIALIZER();
static struct tegra_mc_counter mc_counter1 = MC_COUNTER_INITIALIZER();
static struct tegra_mc_counter emc_llp_counter = MC_COUNTER_INITIALIZER();

/* /sys/devices/system/tegra_mc */
static bool sample_enable	= SAMPLE_ENABLE_DEFAULT;
static u16 sample_quantum	= SAMPLE_QUANTUM_DEFAULT;
static u8 sample_log[SAMPLE_LOG_SIZE];

static DEFINE_SPINLOCK(sample_enable_lock);
static DEFINE_SPINLOCK(sample_log_lock);

static u8 *sample_log_wptr = sample_log, *sample_log_rptr = sample_log;
static int sample_log_size = SAMPLE_LOG_SIZE - 1;

static bool sampling(void)
{
	bool ret;

	spin_lock_bh(&sample_enable_lock);
	ret = (sample_enable == true)? true : false;
	spin_unlock_bh(&sample_enable_lock);

	return ret;
}

static struct sysdev_class tegra_mc_sysclass = {
	.name = "tegra_mc",
};

static ssize_t tegra_mc_enable_show(struct sysdev_class *class,
	struct sysdev_class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sample_enable);
}

static ssize_t tegra_mc_enable_store(struct sysdev_class *class,
	struct sysdev_class_attribute *attr,
	const char *buf, size_t count)
{
	int value, i;
	struct tegra_mc_counter *counters[] = {
		&mc_counter0,
		&mc_counter1,
		&emc_llp_counter
	};

	sscanf(buf, "%d", &value);

	if (value == 0 || value == 1)
		sample_enable = value;
	else
		return -EINVAL;

	if (!sample_enable) {
		stat_stop();
		hrtimer_cancel(&sample_timer);
		return count;
	}

	hrtimer_cancel(&sample_timer);

	/* we need to initialize variables that change during sampling */
	sample_log_wptr = sample_log_rptr = sample_log;
	sample_log_size = SAMPLE_LOG_SIZE - 1;

	for (i = 0; i < ARRAY_SIZE(counters); i++) {
		struct tegra_mc_counter *c = counters[i];

		if (!c->enabled)
			continue;

		c->current_address_low = c->address_low;
		c->current_address_high = c->address_low;
		c->address_range_change = (c->mode == FILTER_ADDR);
		if (c->address_range_change)
			c->current_address_high += c->address_window_size_1;
		else
			c->current_address_high += c->address_length_1;

		c->current_client = 0;
		c->sample_count = 0;
	}

	stat_start();

	hrtimer_start(&sample_timer,
		ktime_add_ns(ktime_get(), (u64)sample_quantum * 1000000),
		HRTIMER_MODE_ABS);

	return count;
}

static ssize_t tegra_mc_log_show(struct sysdev_class *class,
	struct sysdev_class_attribute *attr, char *buf)
{
	int index = 0, count = 0;
	unsigned long flags;

	spin_lock_irqsave(&sample_log_lock, flags);

	while (sample_log_rptr != sample_log_wptr) {
		if (sample_log_rptr < sample_log_wptr) {
			count = sample_log_wptr - sample_log_rptr;
			memcpy(buf + index, sample_log_rptr, count);
			sample_log_rptr = sample_log_wptr;
			sample_log_size += count;
		} else {
			count = SAMPLE_LOG_SIZE -
				(sample_log_rptr - sample_log);
			memcpy(buf + index, sample_log_rptr, count);
			sample_log_rptr = sample_log;
			sample_log_size += count;
		}
		index += count;
	}

	spin_unlock_irqrestore(&sample_log_lock, flags);

	return index;
}

static ssize_t tegra_mc_log_store(struct sysdev_class *class,
	struct sysdev_class_attribute *attr,
	const char *buf, size_t count)
{
	return -EPERM;
}

static ssize_t tegra_mc_quantum_show(struct sysdev_class *class,
	struct sysdev_class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sample_quantum);
}

static ssize_t tegra_mc_quantum_store(struct sysdev_class *class,
	struct sysdev_class_attribute *attr,
	const char *buf, size_t count)
{
	int value;

	if (sampling())
		return -EINVAL;

	sscanf(buf, "%d", &value);
	sample_quantum = value;

	return count;
}

#define TEGRA_MC_EXPAND(_attr,_mode) \
	static SYSDEV_CLASS_ATTR( \
	  _attr, _mode, tegra_mc_##_attr##_show, tegra_mc_##_attr##_store);

#define TEGRA_MC_ATTRIBUTES(_attr1,_mode1,_attr2,_mode2,_attr3,_mode3) \
	TEGRA_MC_EXPAND(_attr1,_mode1) \
	TEGRA_MC_EXPAND(_attr2,_mode2) \
	TEGRA_MC_EXPAND(_attr3,_mode3)

TEGRA_MC_ATTRIBUTES(enable,0666,log,0444,quantum,0666)

#undef TEGRA_MC_EXPAND

#define TEGRA_MC_EXPAND(_attr,_mode) \
	&attr_##_attr,

static struct sysdev_class_attribute *tegra_mc_attrs[] = {
	TEGRA_MC_ATTRIBUTES(enable,0666,log,0444,quantum,0666)
	NULL
};

/* /sys/devices/system/tegra_mc/client */
static bool tegra_mc_client_0_enabled = CLIENT_ENABLED_DEFAULT;
static u8 tegra_mc_client_0_on_schedule_buffer[CLIENT_ON_SCHEDULE_LENGTH];
static struct kobject *tegra_mc_client_kobj, *tegra_mc_client_0_kobj;

struct match_mode {
	const char *name;
	int mode;
};

static const struct match_mode mode_list[] = {
	[0] = {
		.name = "none",
		.mode = FILTER_NONE,
	},
	[1] = {
		.name = "address",
		.mode = FILTER_ADDR,
	},
	[2] = {
		.name = "client",
		.mode = FILTER_CLIENT,
	},
};

static int tegra_mc_parse_mode(const char* str) {
	int i;

	for (i = 0; i < ARRAY_SIZE(mode_list); i++) {
		if (!strncmp(str, mode_list[i].name, strlen(mode_list[i].name)))
			return mode_list[i].mode;
	}
	return -EINVAL;
}

static int tegra_mc_client_parse(const char *buf, size_t count,
	tegra_mc_counter_t *counter0, tegra_mc_counter_t *counter1,
	tegra_mc_counter_t *llp)
{
	char *options, *p, *ptr;
	tegra_mc_counter_t *counter;
	substring_t args[MAX_OPT_ARGS];
	enum {
		opt_period,
		opt_mode,
		opt_client,
		opt_address_low,
		opt_address_length,
		opt_address_window_size,
		opt_err,
	};
	const match_table_t tokens = {
		{opt_period, "period=%s"},
		{opt_mode, "mode=%s"},
		{opt_client, "client=%s"},
		{opt_address_low, "address_low=%s"},
		{opt_address_length, "address_length=%s"},
		{opt_address_window_size, "address_window_size=%s"},
		{opt_err, NULL},
	};
	int ret = 0, i, token, client_number;
	bool aggregate = false;
	int  period, *client_ids, mode;
	bool fperiod = false, fmode = false, fclient = false;
	u64 address_low = 0;
	u64 address_length = 1ull<<32;
	u64 address_window_size = PAGE_SIZE;

	client_ids = kmalloc(sizeof(int) * (MC_COUNTER_CLIENT_SIZE + 1),
		GFP_KERNEL);
	if (!client_ids)
		return -ENOMEM;

	options = kstrdup(buf, GFP_KERNEL);
	if (!options) {
		ret = -ENOMEM;
		goto end;
	}

	while ((p = strsep(&options, " ")) != NULL) {
		if (!*p)
			continue;

		pr_debug("\t %s\n", p);

		token = match_token(p, tokens, args);
		switch (token) {
		case opt_period:
			if (match_int(&args[0], &period) || period<=0) {
				ret = -EINVAL;
				goto end;
			}
			fperiod = true;
			break;

		case opt_mode:
			mode = tegra_mc_parse_mode(args[0].from);
			if (mode<0) {
				ret = mode;
				goto end;
			}
			fmode = true;
			break;

		case opt_client:
			client_ids[0] = 0;

			ptr = get_options(args[0].from,
				MC_COUNTER_CLIENT_SIZE+1 , client_ids);

			if (client_ids[0] <= 0) {
				ret = -EINVAL;
				goto end;
			}

			for (i = 1; i <= client_ids[0]; i++) {
				if (client_ids[i] < MC_STAT_END)
					continue;

				if ((client_ids[i] != MC_STAT_AGGREGATE) ||
				    client_ids[0] != 1) {
					ret = -EINVAL;
					goto end;
				} else
					aggregate = true;
			}

			client_number = client_ids[0];
			fclient = true;
			break;

		case opt_address_low:
			address_low = simple_strtoull(args[0].from, NULL, 0);
			break;

		case opt_address_length:
			address_length = simple_strtoull(args[0].from, NULL, 0);
			break;

		case opt_address_window_size:
			address_window_size = simple_strtoull(args[0].from,
				NULL, 0);
			break;

		default:
			ret = -EINVAL;
			goto end;
		}
	}

	if (!fmode || !fclient || (mode == FILTER_CLIENT && aggregate)) {
		ret = -EINVAL;
		goto end;
	}

	address_low &= PAGE_MASK;
	address_length += PAGE_SIZE-1;
	address_length &= ~((1ull << PAGE_SHIFT)-1ull);

	address_window_size += PAGE_SIZE-1;
	address_window_size &= ~((1ull << PAGE_SHIFT)-1ull);

	if (mode == FILTER_CLIENT) {
		counter = counter0;
		counter->reschedule = (client_number>1);
		counter->client_number = client_number;
		llp->enabled = false;
		counter1->enabled = false;
		for (i = 1; i <= client_number && i < MC_COUNTER_CLIENT_SIZE; i++)
			counter->clients[i - 1] = client_ids[i];
	} else if (mode == FILTER_ADDR || mode == FILTER_NONE) {
		if (aggregate) {
			counter = counter1;
			llp->enabled = true;
			counter0->enabled = false;
		} else {
			counter = counter0;
			counter1->enabled = false;
			llp->enabled = false;
		}
		counter->client_number = 1;
		counter->clients[0] = client_ids[1];
		counter->reschedule = (mode != FILTER_NONE);
	} else {
		ret = -EINVAL;
		goto end;
	}

	counter->mode = mode;
	counter->enabled = true;
	counter->address_low = (u32)address_low;
	counter->address_length_1 = (u32)(address_length-1);
	counter->address_window_size_1 = (u32)(address_window_size-1);

	if (llp->enabled) {
		llp->mode = counter->mode;
		llp->reschedule = counter->reschedule;
		llp->period = counter->period;
		llp->address_low = counter->address_low;
		llp->address_length_1 = counter->address_length_1;
		llp->address_window_size_1 = counter->address_window_size_1;
	}

end:
	if (options)
		kfree(options);
	if (client_ids)
		kfree(client_ids);

	return ret;
}

static ssize_t tegra_mc_client_0_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	if (strcmp(attr->attr.name, "enable") == 0)
		return sprintf(buf, "%d\n", tegra_mc_client_0_enabled);
	else if (strcmp(attr->attr.name, "on_schedule") == 0)
		return sprintf(buf, "%s", tegra_mc_client_0_on_schedule_buffer);
	else
		return -EINVAL;
}

static ssize_t tegra_mc_client_0_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sampling())
		return -EINVAL;

	if (strcmp(attr->attr.name, "enable") == 0) {
		sscanf(buf, "%d\n", &value);
		if (value == 0 || value == 1)
			tegra_mc_client_0_enabled = value;
		else
			return -EINVAL;

		return count;
	} else if (strcmp(attr->attr.name, "on_schedule") == 0) {
		if (tegra_mc_client_parse(buf, count,
			&mc_counter0, &mc_counter1,
			&emc_llp_counter)== 0) {

			strncpy(tegra_mc_client_0_on_schedule_buffer,
				buf, count);

			return count;
		} else
			return -EINVAL;
	} else
		return -EINVAL;
}

static struct kobj_attribute tegra_mc_client_0_enable =
	__ATTR(enable, 0660, tegra_mc_client_0_show, tegra_mc_client_0_store);

static struct kobj_attribute tegra_mc_client_0_on_schedule =
	__ATTR(on_schedule, 0660, tegra_mc_client_0_show, tegra_mc_client_0_store);

static struct attribute *tegra_mc_client_0_attrs[] = {
	&tegra_mc_client_0_enable.attr,
	&tegra_mc_client_0_on_schedule.attr,
	NULL,
};

static struct attribute_group tegra_mc_client_0_attr_group = {
	.attrs = tegra_mc_client_0_attrs
};

/* /sys/devices/system/tegra_mc/dram */
#define dram_counters(_x)					 \
	_x(activate_cnt, ACTIVATE_CNT)				 \
	_x(read_cnt, READ_CNT)					 \
	_x(write_cnt, WRITE_CNT)				 \
	_x(ref_cnt, REF_CNT)					 \
	_x(cumm_banks_active_cke_eq1, CUMM_BANKS_ACTIVE_CKE_EQ1) \
	_x(cumm_banks_active_cke_eq0, CUMM_BANKS_ACTIVE_CKE_EQ0) \
	_x(cke_eq1_clks, CKE_EQ1_CLKS)				 \
	_x(extclks_cke_eq1, EXTCLKS_CKE_EQ1)			 \
	_x(extclks_cke_eq0, EXTCLKS_CKE_EQ0)			 \
	_x(no_banks_active_cke_eq1, NO_BANKS_ACTIVE_CKE_EQ1)	 \
	_x(no_banks_active_cke_eq0, NO_BANKS_ACTIVE_CKE_EQ0)

#define DEFINE_COUNTER(_name, _val) { .enabled = false, .device_mask = 0, },

static tegra_emc_dram_counter_t dram_counters[] = {
	dram_counters(DEFINE_COUNTER)
};

#define DEFINE_SYSFS(_name, _val)					\
									\
static struct kobject *tegra_mc_dram_##_name##_kobj;			\
									\
static ssize_t tegra_mc_dram_##_name##_show(struct kobject *kobj,	\
	struct kobj_attribute *attr, char *buf)				\
{									\
	return tegra_mc_dram_show(kobj, attr, buf,			\
				  _val - EMC_DRAM_STAT_BEGIN);		\
}									\
									\
static ssize_t tegra_mc_dram_##_name##_store(struct kobject *kobj,	\
	struct kobj_attribute *attr, const char *buf, size_t count)	\
{									\
	if (sampling())							\
		return 0;						\
									\
	return tegra_mc_dram_store(kobj, attr, buf, count,		\
				   _val - EMC_DRAM_STAT_BEGIN);		\
}									\
									\
									\
static struct kobj_attribute tegra_mc_dram_##_name##_enable =		\
	       __ATTR(enable, 0660, tegra_mc_dram_##_name##_show,	\
		      tegra_mc_dram_##_name##_store);			\
									\
static struct kobj_attribute tegra_mc_dram_##_name##_device_mask =	\
	       __ATTR(device_mask, 0660, tegra_mc_dram_##_name##_show,	\
		      tegra_mc_dram_##_name##_store);			\
									\
static struct attribute *tegra_mc_dram_##_name##_attrs[] = {		\
	&tegra_mc_dram_##_name##_enable.attr,				\
	&tegra_mc_dram_##_name##_device_mask.attr,			\
	NULL,								\
};									\
									\
static struct attribute_group tegra_mc_dram_##_name##_attr_group = {	\
	.attrs = tegra_mc_dram_##_name##_attrs,				\
};

static struct kobject *tegra_mc_dram_kobj;

static ssize_t tegra_mc_dram_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf, int index)
{
	if (index >= EMC_DRAM_STAT_END - EMC_DRAM_STAT_BEGIN)
		return -EINVAL;

	if (strcmp(attr->attr.name, "enable") == 0)
		return sprintf(buf, "%d\n", dram_counters[index].enabled);
	else if (strcmp(attr->attr.name, "device_mask") == 0)
		return sprintf(buf, "%d\n", dram_counters[index].device_mask);
	else
		return -EINVAL;
}
static ssize_t tegra_mc_dram_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count, int index)
{
	int value;

	if (index >= EMC_DRAM_STAT_END - EMC_DRAM_STAT_BEGIN)
		return -EINVAL;

	if (strcmp(attr->attr.name, "enable") == 0) {
		sscanf(buf, "%d\n", &value);
		if (value == 0 || value == 1)
			dram_counters[index].enabled = value;
		else
			return -EINVAL;

		return count;
	} else if (strcmp(attr->attr.name, "device_mask") == 0) {
		sscanf(buf, "%d\n", &value);
		dram_counters[index].device_mask = (u8)value;

		return count;
	} else
		return -EINVAL;
}

dram_counters(DEFINE_SYSFS)

/* Tegra Statistics */
typedef struct {
	void __iomem *mmio;
} tegra_device_t;

static tegra_device_t mc = {
	.mmio = IO_ADDRESS(TEGRA_MC_BASE),
};

static tegra_device_t emc = {
	.mmio = IO_ADDRESS(TEGRA_EMC_BASE),
};

void mc_stat_start(tegra_mc_counter_t *counter0, tegra_mc_counter_t *counter1)
{
	struct tegra_mc_counter *c;
	u32 filter_client = ARMC_STAT_CONTROL_FILTER_CLIENT_DISABLE;
	u32 filter_addr = ARMC_STAT_CONTROL_FILTER_ADDR_DISABLE;

	if (!tegra_mc_client_0_enabled)
		return;

	c = (counter0->enabled) ? counter0 : counter1;

	/* disable statistics */
	writel((MC_STAT_CONTROL_0_EMC_GATHER_DISABLE << MC_STAT_CONTROL_0_EMC_GATHER_SHIFT),
		mc.mmio + MC_STAT_CONTROL_0);

	if (c->enabled && c->mode == FILTER_ADDR)
		filter_addr = ARMC_STAT_CONTROL_FILTER_ADDR_ENABLE;
	else if (c->enabled && c->mode == FILTER_CLIENT)
		filter_client = ARMC_STAT_CONTROL_FILTER_CLIENT_ENABLE;

	filter_addr <<= ARMC_STAT_CONTROL_FILTER_ADDR_SHIFT;
	filter_client <<= ARMC_STAT_CONTROL_FILTER_CLIENT_SHIFT;

	if (c->enabled) {
		u32 reg = 0;
		reg |= (ARMC_STAT_CONTROL_MODE_BANDWIDTH <<
			ARMC_STAT_CONTROL_MODE_SHIFT);
		reg |= (ARMC_STAT_CONTROL_EVENT_QUALIFIED <<
			ARMC_STAT_CONTROL_EVENT_SHIFT);
		reg |= (ARMC_STAT_CONTROL_FILTER_PRI_DISABLE <<
			ARMC_STAT_CONTROL_FILTER_PRI_SHIFT);
		reg |= (ARMC_STAT_CONTROL_FILTER_COALESCED_DISABLE <<
			ARMC_STAT_CONTROL_FILTER_COALESCED_SHIFT);
		reg |= filter_client;
		reg |= filter_addr;
		reg |= (c->clients[c->current_client] <<
			ARMC_STAT_CONTROL_CLIENT_ID_SHIFT);

		/* note these registers are shared */
		writel(c->current_address_low,
		       mc.mmio + MC_STAT_EMC_ADDR_LOW_0);
		writel(c->current_address_high,
		       mc.mmio + MC_STAT_EMC_ADDR_HIGH_0);
		writel(0xFFFFFFFF, mc.mmio + MC_STAT_EMC_CLOCK_LIMIT_0);

		writel(reg, mc.mmio + MC_STAT_EMC_CONTROL_0_0);
	}

	/* reset then enable statistics */
	writel((MC_STAT_CONTROL_0_EMC_GATHER_CLEAR << MC_STAT_CONTROL_0_EMC_GATHER_SHIFT),
		mc.mmio + MC_STAT_CONTROL_0);

	writel((MC_STAT_CONTROL_0_EMC_GATHER_ENABLE << MC_STAT_CONTROL_0_EMC_GATHER_SHIFT),
		mc.mmio + MC_STAT_CONTROL_0);
}

void mc_stat_stop(tegra_mc_counter_t *counter0,
	tegra_mc_counter_t *counter1)
{
	/* Disable statistics */
	writel((MC_STAT_CONTROL_0_EMC_GATHER_DISABLE << MC_STAT_CONTROL_0_EMC_GATHER_SHIFT),
		mc.mmio + MC_STAT_CONTROL_0);

	if (counter0->enabled)
		counter0->value = readl(mc.mmio + MC_STAT_EMC_COUNT_0_0);
	else
		counter1->value = readl(mc.mmio + MC_STAT_EMC_COUNT_1_0);
}

void emc_stat_start(tegra_mc_counter_t *llp_counter,
	tegra_emc_dram_counter_t *dram_counter)
{
	u32 llmc_stat = 0;
	u32 llmc_ctrl =
		(AREMC_STAT_CONTROL_MODE_BANDWIDTH <<
			AREMC_STAT_CONTROL_MODE_SHIFT) |
		(AREMC_STAT_CONTROL_CLIENT_TYPE_MPCORER <<
			AREMC_STAT_CONTROL_CLIENT_TYPE_SHIFT) |
		(AREMC_STAT_CONTROL_EVENT_QUALIFIED <<
			AREMC_STAT_CONTROL_EVENT_SHIFT);

	/* disable statistics */
	llmc_stat |= (EMC_STAT_CONTROL_0_LLMC_GATHER_DISABLE <<
			EMC_STAT_CONTROL_0_LLMC_GATHER_SHIFT);
	llmc_stat |= (EMC_STAT_CONTROL_0_DRAM_GATHER_DISABLE <<
			EMC_STAT_CONTROL_0_DRAM_GATHER_SHIFT);
	writel(llmc_stat, emc.mmio + EMC_STAT_CONTROL_0);

	if (tegra_mc_client_0_enabled && llp_counter->enabled) {
		if (llp_counter->mode == FILTER_ADDR) {
			llmc_ctrl |=
				(AREMC_STAT_CONTROL_FILTER_ADDR_ENABLE <<
				 AREMC_STAT_CONTROL_FILTER_ADDR_SHIFT);
			llmc_ctrl |=
				(AREMC_STAT_CONTROL_FILTER_CLIENT_DISABLE <<
				 AREMC_STAT_CONTROL_FILTER_CLIENT_SHIFT);
		} else if (llp_counter->mode == FILTER_CLIENT) {
			/* not allow aggregate client in client mode */
			llmc_ctrl |=
				(AREMC_STAT_CONTROL_FILTER_ADDR_DISABLE <<
				 AREMC_STAT_CONTROL_FILTER_ADDR_SHIFT);
			llmc_ctrl |=
				(AREMC_STAT_CONTROL_FILTER_CLIENT_DISABLE <<
				 AREMC_STAT_CONTROL_FILTER_CLIENT_SHIFT);
		} else if (llp_counter->mode == FILTER_NONE) {
			llmc_ctrl |=
				(AREMC_STAT_CONTROL_FILTER_ADDR_DISABLE <<
				 AREMC_STAT_CONTROL_FILTER_ADDR_SHIFT);
			llmc_ctrl |=
				(AREMC_STAT_CONTROL_FILTER_CLIENT_DISABLE <<
				 AREMC_STAT_CONTROL_FILTER_CLIENT_SHIFT);
		}

		writel(llp_counter->current_address_low,
		       emc.mmio + EMC_STAT_LLMC_ADDR_LOW_0);
		writel(llp_counter->current_address_high,
		       emc.mmio + EMC_STAT_LLMC_ADDR_HIGH_0);
		writel(0xFFFFFFFF, emc.mmio + EMC_STAT_LLMC_CLOCK_LIMIT_0);
		writel(llmc_ctrl, emc.mmio + EMC_STAT_LLMC_CONTROL_0_0);
	}

	writel(0xFFFFFFFF, emc.mmio + EMC_STAT_DRAM_CLOCK_LIMIT_LO_0);
	writel(0xFF, emc.mmio + EMC_STAT_DRAM_CLOCK_LIMIT_HI_0);

	/* Reset then enable statistics */
	llmc_stat |= (EMC_STAT_CONTROL_0_LLMC_GATHER_CLEAR <<
			EMC_STAT_CONTROL_0_LLMC_GATHER_SHIFT);
	llmc_stat |= (EMC_STAT_CONTROL_0_DRAM_GATHER_CLEAR <<
			EMC_STAT_CONTROL_0_DRAM_GATHER_SHIFT);
	writel(llmc_stat, emc.mmio + EMC_STAT_CONTROL_0);

	llmc_stat |= (EMC_STAT_CONTROL_0_LLMC_GATHER_ENABLE <<
			EMC_STAT_CONTROL_0_LLMC_GATHER_SHIFT);
	llmc_stat |= (EMC_STAT_CONTROL_0_DRAM_GATHER_ENABLE <<
			EMC_STAT_CONTROL_0_DRAM_GATHER_SHIFT);
	writel(llmc_stat, emc.mmio + EMC_STAT_CONTROL_0);
}

void emc_stat_stop(tegra_mc_counter_t *llp_counter,
	tegra_emc_dram_counter_t *dram_counter)
{
	u32 llmc_stat = 0;
	int i;
	int dev0_offsets_lo[] = {
		EMC_STAT_DRAM_DEV0_ACTIVATE_CNT_LO_0,
		EMC_STAT_DRAM_DEV0_READ_CNT_LO_0,
		EMC_STAT_DRAM_DEV0_WRITE_CNT_LO_0,
		EMC_STAT_DRAM_DEV0_REF_CNT_LO_0,
		EMC_STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ1_LO_0,
		EMC_STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ0_LO_0,
		EMC_STAT_DRAM_DEV0_CKE_EQ1_CLKS_LO_0,
		EMC_STAT_DRAM_DEV0_EXTCLKS_CKE_EQ1_LO_0,
		EMC_STAT_DRAM_DEV0_EXTCLKS_CKE_EQ0_LO_0,
		EMC_STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ1_LO_0,
		EMC_STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ0_LO_0,
	};
	int dev0_offsets_hi[] = {
		EMC_STAT_DRAM_DEV0_ACTIVATE_CNT_HI_0,
		EMC_STAT_DRAM_DEV0_READ_CNT_HI_0,
		EMC_STAT_DRAM_DEV0_WRITE_CNT_HI_0,
		EMC_STAT_DRAM_DEV0_REF_CNT_HI_0,
		EMC_STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ1_HI_0,
		EMC_STAT_DRAM_DEV0_CUMM_BANKS_ACTIVE_CKE_EQ0_HI_0,
		EMC_STAT_DRAM_DEV0_CKE_EQ1_CLKS_HI_0,
		EMC_STAT_DRAM_DEV0_EXTCLKS_CKE_EQ1_HI_0,
		EMC_STAT_DRAM_DEV0_EXTCLKS_CKE_EQ0_HI_0,
		EMC_STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ1_HI_0,
		EMC_STAT_DRAM_DEV0_NO_BANKS_ACTIVE_CKE_EQ0_HI_0,
	};
	int dev1_offsets_lo[] = {
		EMC_STAT_DRAM_DEV1_ACTIVATE_CNT_LO_0,
		EMC_STAT_DRAM_DEV1_READ_CNT_LO_0,
		EMC_STAT_DRAM_DEV1_WRITE_CNT_LO_0,
		EMC_STAT_DRAM_DEV1_REF_CNT_LO_0,
		EMC_STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ1_LO_0,
		EMC_STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ0_LO_0,
		EMC_STAT_DRAM_DEV1_CKE_EQ1_CLKS_LO_0,
		EMC_STAT_DRAM_DEV1_EXTCLKS_CKE_EQ1_LO_0,
		EMC_STAT_DRAM_DEV1_EXTCLKS_CKE_EQ0_LO_0,
		EMC_STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ1_LO_0,
		EMC_STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ0_LO_0,
	};
	int dev1_offsets_hi[] = {
		EMC_STAT_DRAM_DEV1_ACTIVATE_CNT_HI_0,
		EMC_STAT_DRAM_DEV1_READ_CNT_HI_0,
		EMC_STAT_DRAM_DEV1_WRITE_CNT_HI_0,
		EMC_STAT_DRAM_DEV1_REF_CNT_HI_0,
		EMC_STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ1_HI_0,
		EMC_STAT_DRAM_DEV1_CUMM_BANKS_ACTIVE_CKE_EQ0_HI_0,
		EMC_STAT_DRAM_DEV1_CKE_EQ1_CLKS_HI_0,
		EMC_STAT_DRAM_DEV1_EXTCLKS_CKE_EQ1_HI_0,
		EMC_STAT_DRAM_DEV1_EXTCLKS_CKE_EQ0_HI_0,
		EMC_STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ1_HI_0,
		EMC_STAT_DRAM_DEV1_NO_BANKS_ACTIVE_CKE_EQ0_HI_0,
	};

	/* Disable statistics */
	llmc_stat |= (EMC_STAT_CONTROL_0_LLMC_GATHER_DISABLE <<
			EMC_STAT_CONTROL_0_LLMC_GATHER_SHIFT);
	llmc_stat |= (EMC_STAT_CONTROL_0_DRAM_GATHER_DISABLE <<
			EMC_STAT_CONTROL_0_DRAM_GATHER_SHIFT);
	writel(llmc_stat, emc.mmio + EMC_STAT_CONTROL_0);

	if (tegra_mc_client_0_enabled == true)
		llp_counter->value = readl(emc.mmio + EMC_STAT_LLMC_COUNT_0_0);

	for (i = 0; i < EMC_DRAM_STAT_END - EMC_DRAM_STAT_BEGIN; i++) {
		if (dram_counter[i].enabled) {
			dram_counter[i].value = 0;
			if (!(dram_counter[i].device_mask & 0x1)) {
				if (readl(emc.mmio + dev0_offsets_hi[i]) != 0) {
					dram_counter[i].value = 0xFFFFFFFF;
					continue;
				}
				dram_counter[i].value +=
					readl(emc.mmio + dev0_offsets_lo[i]);
			}
			if (!(dram_counter[i].device_mask & 0x2)) {
				if (readl(emc.mmio + dev1_offsets_hi[i]) != 0) {
					dram_counter[i].value = 0xFFFFFFFF;
					continue;
				}
				dram_counter[i].value +=
					readl(emc.mmio + dev1_offsets_lo[i]);
			}
		}
	}
}

static void stat_reschedule(tegra_mc_counter_t *counter0,
	tegra_mc_counter_t *counter1,
	tegra_mc_counter_t *llp)
{
	int i;
	struct tegra_mc_counter *counters[] = {
		counter0,
		counter1,
		llp
	};

	if (!tegra_mc_client_0_enabled)
		return;

	for (i = 0; i < ARRAY_SIZE(counters); i++) {
		struct tegra_mc_counter *c = counters[i];

		c->address_range_change = false;
		if (!c->enabled || !c->reschedule)
			continue;

		c->sample_count++;

		if (c->sample_count < c->period)
			continue;

		c->sample_count = 0;

		if (c->mode == FILTER_CLIENT) {
			c->current_client++;
			if (c->current_client == c->client_number)
				c->current_client = 0;
			continue;
		}

		c->address_range_change = true;
		c->current_address_low = c->current_address_high+1;

		if (c->current_address_low >= c->address_low+c->address_length_1)
			c->current_address_low = c->address_low;

		c->current_address_high = c->current_address_low +
			c->address_window_size_1;
	}
}

static void stat_start(void)
{
	mc_stat_start(&mc_counter0, &mc_counter1);
	emc_stat_start(&emc_llp_counter, dram_counters);
}

static void stat_stop(void)
{
	mc_stat_stop(&mc_counter0, &mc_counter1);
	emc_stat_stop(&emc_llp_counter, dram_counters);
}

static size_t stat_log_counter(struct tegra_mc_counter *c,
	struct tegra_mc_counter *l, log_event_t *e, u32* value)
{
	size_t size = 0;

	*value = c->value;
	if (l)
		*value += l->value;

	if (!c->enabled || (l && !l->enabled) || !*value)
		return 0;

	e->word0.enabled = 1;
	e->word0.address_range_change = c->address_range_change;
	e->word0.event_id = (l) ? MC_STAT_AGGREGATE :
		c->clients[c->current_client];
	e->word0.address_range_low_pfn = __phys_to_pfn(c->current_address_low);
	size += sizeof(e->word0);

	if (c->address_range_change) {
		e->word1.address_range_length_pfn =
			__phys_to_pfn(c->address_window_size_1+1);
		size += sizeof(e->word1);
	}

	size += sizeof(*value);
	return size;
}

#define statcpy(_buf, _bufstart, _buflen, _elem)	\
	do {						\
		size_t s = sizeof(_elem);		\
		memcpy(_buf, &_elem, s);		\
		_buf += s;				\
		if (_buf >= _bufstart + _buflen)	\
			_buf = _bufstart;		\
	} while (0);

static void stat_log(void)
{
	log_header_t	header = {0, 0};
	log_event_t	event[LOG_EVENT_NUMBER_MAX];
	u32		value[LOG_EVENT_NUMBER_MAX];
	int		i, count = 0;
	unsigned long	flags;
	size_t elem;
	int	required_log_size = 0;

	required_log_size += sizeof(header);

	if (tegra_mc_client_0_enabled) {
		elem = stat_log_counter(&mc_counter0, NULL, &event[count],
					&value[count]);
		if (elem) {
			required_log_size += elem;
			count++;
		}

		elem = stat_log_counter(&mc_counter1, &emc_llp_counter,
					&event[count], &value[count]);

		if (elem) {
			required_log_size += elem;
			count++;
		}
	}

	for (i = 0; i < EMC_DRAM_STAT_END - EMC_DRAM_STAT_BEGIN &&
		count < LOG_EVENT_NUMBER_MAX; i++) {
		if (dram_counters[i].enabled && dram_counters[i].value != 0) {
			event[count].word0.enabled = 1;
			event[count].word0.address_range_change = false;
			event[count].word0.event_id = i + EMC_DRAM_STAT_BEGIN;
			event[count].word0.address_range_low_pfn = 0;
			required_log_size += sizeof(event[count].word0);

			event[count].word1.address_range_length_pfn =
				0xFFFFFFFFUL >> SHIFT_4K;

			value[count] = dram_counters[i].value;
			required_log_size += sizeof(value[count]);

			count++;
		}
	}

	header.time_quantum = sample_quantum * MILLISECONDS_TO_TIME_QUANTUM;
	for (i = 0; i < count; i++) {
		header.event_state_change |= 1 << i;
	}

	if (header.event_state_change != 0) {
		spin_lock_irqsave(&sample_log_lock, flags);
		if (unlikely(required_log_size > sample_log_size)) {
			pr_err("%s: sample log too small!\n", __func__);
			WARN_ON(1);
			spin_unlock_irqrestore(&sample_log_lock, flags);
			goto reschedule;
		}

		statcpy(sample_log_wptr, sample_log, SAMPLE_LOG_SIZE, header);

		for (i=0; i<count; i++) {
			statcpy(sample_log_wptr, sample_log,
				SAMPLE_LOG_SIZE, event[i].word0);
			if (!event[i].word0.address_range_change)
				continue;
			statcpy(sample_log_wptr, sample_log,
				SAMPLE_LOG_SIZE, event[i].word1);
		}

		for (i=0; i<count; i++) {
			statcpy(sample_log_wptr, sample_log,
				SAMPLE_LOG_SIZE, value[i]);
		}

		sample_log_size -= required_log_size;
		spin_unlock_irqrestore(&sample_log_lock, flags);
	}

reschedule:
	stat_reschedule(&mc_counter0, &mc_counter1, &emc_llp_counter);
}

static enum hrtimer_restart sample_timer_function(struct hrtimer *handle)
{
	stat_stop();
	stat_log();

	if (!sample_enable)
		return HRTIMER_NORESTART;

	stat_start();

	hrtimer_add_expires_ns(&sample_timer, (u64)sample_quantum * 1000000);
	return HRTIMER_RESTART;
}

/* module init */
#define REGISTER_SYSFS(_name, _val)					\
	tegra_mc_dram_##_name##_kobj =					\
		kobject_create_and_add(#_name, tegra_mc_dram_kobj);	\
	sysfs_create_group(tegra_mc_dram_##_name##_kobj,		\
			   &tegra_mc_dram_##_name##_attr_group);

static int tegra_mc_init(void)
{
	int i;
	int rc;

	/* /sys/devices/system/tegra_mc */
	rc = sysdev_class_register(&tegra_mc_sysclass);
	if(rc)
		goto out;

	for (i = 0;  i < ARRAY_SIZE(tegra_mc_attrs)-1; i++) {
		rc = sysdev_class_create_file(&tegra_mc_sysclass,
			tegra_mc_attrs[i]);
		if(rc) {
			printk("\n sysdev_class_create_file : failed \n");
			goto out_unreg_class;
		}
	}

	/* /sys/devices/system/tegra_mc/client */
	tegra_mc_client_kobj = kobject_create_and_add("client",
		&tegra_mc_sysclass.kset.kobj);
	if(!tegra_mc_client_kobj)
		goto out_remove_sysdev_files;

	tegra_mc_client_0_kobj = kobject_create_and_add("0",
		tegra_mc_client_kobj);
	if(!tegra_mc_client_0_kobj)
		goto out_put_kobject_client;

	rc = sysfs_create_group(tegra_mc_client_0_kobj,
		&tegra_mc_client_0_attr_group);
	if(rc)
		goto out_put_kobject_client_0;

	/* /sys/devices/system/tegra_mc/dram */
	tegra_mc_dram_kobj = kobject_create_and_add("dram",
		&tegra_mc_sysclass.kset.kobj);
	if(!tegra_mc_dram_kobj)
		goto out_remove_group_client_0;

	dram_counters(REGISTER_SYSFS)

	/* hrtimer */
	hrtimer_init(&sample_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	sample_timer.function = sample_timer_function;

	return 0;

out_remove_group_client_0:
	sysfs_remove_group(tegra_mc_client_0_kobj, &tegra_mc_client_0_attr_group);

out_put_kobject_client_0:
	kobject_put(tegra_mc_client_0_kobj);

out_put_kobject_client:
	kobject_put(tegra_mc_client_kobj);

out_remove_sysdev_files:
	for (i = 0;  i < ARRAY_SIZE(tegra_mc_attrs)-1; i++) {
		sysdev_class_remove_file(&tegra_mc_sysclass, tegra_mc_attrs[i]);
	}

out_unreg_class:
	sysdev_class_unregister(&tegra_mc_sysclass);

out:
	return rc;
}

/* module deinit */
#define REMOVE_SYSFS(_name, _val)					\
	sysfs_remove_group(tegra_mc_dram_##_name##_kobj,		\
			   &tegra_mc_dram_##_name##_attr_group);	\
	kobject_put(tegra_mc_dram_##_name##_kobj);

static void tegra_mc_exit(void)
{
	int i;

	stat_stop();

	/* hrtimer */
	hrtimer_cancel(&sample_timer);

	/* /sys/devices/system/tegra_mc/client */
	sysfs_remove_group(tegra_mc_client_0_kobj,
		&tegra_mc_client_0_attr_group);
	kobject_put(tegra_mc_client_0_kobj);
	kobject_put(tegra_mc_client_kobj);

	/* /sys/devices/system/tegra_mc/dram */
	dram_counters(REMOVE_SYSFS)
	kobject_put(tegra_mc_dram_kobj);

	/* /sys/devices/system/tegra_mc */
	for (i = 0;  i < ARRAY_SIZE(tegra_mc_attrs)-1; i++) {
		sysdev_class_remove_file(&tegra_mc_sysclass, tegra_mc_attrs[i]);
	}
	sysdev_class_unregister(&tegra_mc_sysclass);
}

module_init(tegra_mc_init);
module_exit(tegra_mc_exit);
MODULE_LICENSE("Dual BSD/GPL");
