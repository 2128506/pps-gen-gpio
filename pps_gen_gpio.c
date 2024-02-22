/*
 * pps_gen_gpio.c -- kernel GPIO PPS signal generator
 *
 * Copyright (C)  2009   Alexander Gordeev <lasaine@lvk.cs.msu.su>
 *                2018   Juan Solano <jsm@jsolano.com>
 *                2024   Victor Kirhenshtein <victor@netxms.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define DRVDESC "GPIO PPS signal generator"
MODULE_AUTHOR("Victor Kirhenshtein <victor@netxms.com>");
MODULE_DESCRIPTION(DRVDESC);
MODULE_LICENSE("GPL");

#define SAFETY_INTERVAL_NS             (10 * NSEC_PER_USEC)   /* 10us */
#define RESET_EDGE_EVENT_TOLERANCE_NS  (10 * NSEC_PER_MSEC)   /* 10ms */

enum pps_sync_mode
{
   PPS_SYNC_RAISING_EDGE = 0,
   PPS_SYNC_FALLING_EDGE = 1
};

/* Module parameters. */
static unsigned int pps_sync_mode = PPS_SYNC_RAISING_EDGE;
MODULE_PARM_DESC(mode, "Synchronization mode (0 = raising edge, 1 = falling edge)");
module_param_named(mode, pps_sync_mode, uint, 0000);

static unsigned int pps_pulse_width = 200;
MODULE_PARM_DESC(width, "Pulse width in milliseconds");
module_param_named(width, pps_pulse_width, uint, 0000);

enum pps_gen_gpio_state
{
   PPS_GPIO_INACTIVE = 0,
   PPS_GPIO_ACTIVATING = 1,
   PPS_GPIO_DEACTIVATING = 2,
   PPS_GPIO_ACTIVE = 3
};

/* Device private data structure. */
struct pps_gen_gpio_devdata
{
	struct gpio_desc *pps_gpio;     /* GPIO port descriptor */
	struct hrtimer timer_sync_edge;
	struct hrtimer timer_reset_edge;
	long gpio_write_time;           /* measured port write time (ns) */
   int state;
};

/* Average of hrtimer interrupt latency. */
static long hrtimer_avg_latency = SAFETY_INTERVAL_NS;

/* Levels for sync and reset edges */
static int gpio_sync = 1;
static int gpio_reset = 0;

static ssize_t get_active_flag(struct device *dev, struct device_attribute *attr, char *buf)
{
   struct pps_gen_gpio_devdata *devdata = dev_get_drvdata(dev);
   buf[0] = (devdata->state == PPS_GPIO_ACTIVE) ? '1' : '0';
   buf[1] = 0;
   return 1;
}

static ssize_t set_active_flag(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
   struct pps_gen_gpio_devdata *devdata = dev_get_drvdata(dev);
   long v;

   if (kstrtol(buf, 10, &v) != 0)
      v = 0;

   if (v && devdata->state != PPS_GPIO_ACTIVE)
      devdata->state = PPS_GPIO_ACTIVATING;
   else if (!v && devdata->state != PPS_GPIO_INACTIVE)
      devdata->state = PPS_GPIO_DEACTIVATING;

   return count;
}

static ssize_t get_gpio_write_time(struct device *dev, struct device_attribute *attr, char *buf)
{
   struct pps_gen_gpio_devdata *devdata = dev_get_drvdata(dev);
   return sprintf(buf, "%ld", devdata->gpio_write_time);
}

static ssize_t get_pulse_width(struct device *dev, struct device_attribute *attr, char *buf)
{
   return sprintf(buf, "%u", pps_pulse_width);
}

static ssize_t get_sync_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
   return sprintf(buf, "%s-edge", pps_sync_mode == PPS_SYNC_RAISING_EDGE ? "raising" : "falling");
}

static ssize_t get_timer_latency(struct device *dev, struct device_attribute *attr, char *buf)
{
   return sprintf(buf, "%ld", hrtimer_avg_latency);
}

static DEVICE_ATTR(active, S_IRUGO | S_IWUSR, get_active_flag, set_active_flag);
static DEVICE_ATTR(gpio_write_time, S_IRUGO, get_gpio_write_time, NULL);
static DEVICE_ATTR(mode, S_IRUGO, get_sync_mode, NULL);
static DEVICE_ATTR(pulse_width, S_IRUGO, get_pulse_width, NULL);
static DEVICE_ATTR(timer_latency, S_IRUGO, get_timer_latency, NULL);
static struct attribute *state_attrs[] = {
   &dev_attr_active.attr,
   &dev_attr_gpio_write_time.attr,
   &dev_attr_mode.attr,
   &dev_attr_pulse_width.attr,
   &dev_attr_timer_latency.attr,
   NULL
};
static struct attribute_group state_group = {
   .name = "state",
   .attrs = state_attrs,
};

static inline void update_hrtimer_latency(struct timespec64 ts_expire_real, struct timespec64 ts_expire_req)
{
   struct timespec64 ts_hrtimer_latency;
	long hrtimer_latency;

	/* Update the average hrtimer latency. */
	ts_hrtimer_latency = timespec64_sub(ts_expire_real, ts_expire_req);
	hrtimer_latency = timespec64_to_ns(&ts_hrtimer_latency);

   /* Ignore negative values (seen this on boot, could be that expire_real < expire_req ?) */
   if (hrtimer_latency >= 0)
   {
      /* If the new latency value is bigger then the old, use the new
      * value, if not then slowly move towards the new value. This
      * way it should be safe in bad conditions and efficient in
      * good conditions.
      */
      if (hrtimer_latency > hrtimer_avg_latency)
         hrtimer_avg_latency = hrtimer_latency;
      else
         hrtimer_avg_latency = (3 * hrtimer_avg_latency + hrtimer_latency) / 4;
   }
}

/* hrtimer front edge event callback */
static enum hrtimer_restart hrtimer_callback_reset_edge(struct hrtimer *timer)
{
	unsigned long irq_flags;
	struct pps_gen_gpio_devdata *devdata = container_of(timer, struct pps_gen_gpio_devdata, timer_reset_edge);
	const long time_gpio_change_ns =
      (pps_sync_mode == PPS_SYNC_RAISING_EDGE) ?
         pps_pulse_width * NSEC_PER_MSEC - devdata->gpio_write_time :
         (1000 - pps_pulse_width) * NSEC_PER_MSEC - devdata->gpio_write_time;
	struct timespec64 ts_expire_req, ts_expire_real, ts_gpio_write_time, ts1, ts2;

   if (devdata->state == PPS_GPIO_INACTIVE)
   {
   	ktime_get_real_ts64(&ts_expire_real);
   	ts_expire_req = ktime_to_timespec64(hrtimer_get_softexpires(timer));
      goto done;
   }

	/* We have to disable interrupts here. The idea is to prevent
	 * other interrupts on the same processor to introduce random
	 * lags while polling the clock; ktime_get_real_ts64() takes <1us on
	 * most machines while other interrupt handlers can take much
	 * more potentially.
	 */
	local_irq_save(irq_flags);

	/* Get current timestamp and requested time to check if we are late. Much higher tolerance can be used for raise event. */
	ktime_get_real_ts64(&ts_expire_real);
	ts_expire_req = ktime_to_timespec64(hrtimer_get_softexpires(timer));
	if (ts_expire_req.tv_sec != ts_expire_real.tv_sec || ts_expire_real.tv_nsec > time_gpio_change_ns + RESET_EDGE_EVENT_TOLERANCE_NS)
   {
		local_irq_restore(irq_flags);
		pr_err("pps_gen_gpio: reset edge event is too late [%lld.%09ld]\n", ts_expire_real.tv_sec, ts_expire_real.tv_nsec);
		goto done;
	}

	/* Busy loop until the time is right for a GPIO assert. */
	do
   {
		ktime_get_real_ts64(&ts1);
   } 
   while ((ts_expire_req.tv_sec == ts1.tv_sec) && (ts1.tv_nsec < time_gpio_change_ns));

	/* Assert PPS GPIO. */
	gpiod_set_value(devdata->pps_gpio, gpio_reset);

	ktime_get_real_ts64(&ts2);
	local_irq_restore(irq_flags);

	/* Update the calibrated GPIO set instruction time. */
	ts_gpio_write_time = timespec64_sub(ts2, ts1);
	devdata->gpio_write_time = (devdata->gpio_write_time + timespec64_to_ns(&ts_gpio_write_time)) / 2;

   if (pps_sync_mode == PPS_SYNC_RAISING_EDGE)
   {
      if (devdata->state == PPS_GPIO_ACTIVATING)
      {
         devdata->state = PPS_GPIO_ACTIVE;
         pr_info("pps_gen_gpio: state changed to ACTIVE\n");
      }
      else if (devdata->state == PPS_GPIO_DEACTIVATING)
      {
         devdata->state = PPS_GPIO_INACTIVE;
         pr_info("pps_gen_gpio: state changed to INACTIVE\n");
      }
   }

done:
	/* Update the average hrtimer latency. */
   update_hrtimer_latency(ts_expire_real, ts_expire_req);

	/* Update the hrtimer expire time. */
	hrtimer_set_expires(timer, ktime_set(ts_expire_req.tv_sec + 1, time_gpio_change_ns - hrtimer_avg_latency - SAFETY_INTERVAL_NS));
	return HRTIMER_RESTART;
}

/* hrtimer sync edge event callback */
static enum hrtimer_restart hrtimer_callback_sync_edge(struct hrtimer *timer)
{
	unsigned long irq_flags;
	struct pps_gen_gpio_devdata *devdata = container_of(timer, struct pps_gen_gpio_devdata, timer_sync_edge);
	const long time_gpio_change_ns = NSEC_PER_SEC - devdata->gpio_write_time;
	struct timespec64 ts_expire_req, ts_expire_real, ts_gpio_write_time, ts1, ts2;

   if ((devdata->state == PPS_GPIO_INACTIVE) ||
       (devdata->state == PPS_GPIO_ACTIVATING) ||
       ((devdata->state == PPS_GPIO_DEACTIVATING) && (pps_sync_mode == PPS_SYNC_RAISING_EDGE)))
   {
   	ktime_get_real_ts64(&ts_expire_real);
   	ts_expire_req = ktime_to_timespec64(hrtimer_get_softexpires(timer));
      goto done;
   }

	/* We have to disable interrupts here. The idea is to prevent
	 * other interrupts on the same processor to introduce random
	 * lags while polling the clock; ktime_get_real_ts64() takes <1us on
	 * most machines while other interrupt handlers can take much
	 * more potentially.
	 */
	local_irq_save(irq_flags);

	/* Get current timestamp and requested time to check if we are late. */
	ktime_get_real_ts64(&ts_expire_real);
	ts_expire_req = ktime_to_timespec64(hrtimer_get_softexpires(timer));
	if (ts_expire_req.tv_sec != ts_expire_real.tv_sec || ts_expire_real.tv_nsec > time_gpio_change_ns)
   {
		local_irq_restore(irq_flags);
		pr_err("pps_gen_gpio: sync edge event is too late [%lld.%09ld]\n", ts_expire_real.tv_sec, ts_expire_real.tv_nsec);
		goto done;
	}

	/* Busy loop until the time is right for a GPIO change. */
	do
   {
		ktime_get_real_ts64(&ts1);
   }
	while ((ts_expire_req.tv_sec == ts1.tv_sec) && (ts1.tv_nsec < time_gpio_change_ns));

	/* Change PPS GPIO. */
	gpiod_set_value(devdata->pps_gpio, gpio_sync);

	ktime_get_real_ts64(&ts2);
	local_irq_restore(irq_flags);

	/* Update the calibrated GPIO set instruction time. */
	ts_gpio_write_time = timespec64_sub(ts2, ts1);
	devdata->gpio_write_time = (devdata->gpio_write_time + timespec64_to_ns(&ts_gpio_write_time)) / 2;

done:
   if (pps_sync_mode == PPS_SYNC_FALLING_EDGE)
   {
      if (devdata->state == PPS_GPIO_ACTIVATING)
      {
         devdata->state = PPS_GPIO_ACTIVE;
         pr_info("pps_gen_gpio: state changed to ACTIVE\n");
      }
      else if (devdata->state == PPS_GPIO_DEACTIVATING)
      {
         devdata->state = PPS_GPIO_INACTIVE;
         pr_info("pps_gen_gpio: state changed to INACTIVE\n");
      }
   }

	/* Update the average hrtimer latency. */
   update_hrtimer_latency(ts_expire_real, ts_expire_req);

	/* Update the hrtimer expire time. */
	hrtimer_set_expires(timer, ktime_set(ts_expire_req.tv_sec + 1, time_gpio_change_ns - hrtimer_avg_latency - SAFETY_INTERVAL_NS));
	return HRTIMER_RESTART;
}

/* Initial calibration of GPIO set instruction time. */
#define PPS_GEN_CALIBRATE_LOOPS 100
static void pps_gen_calibrate(struct pps_gen_gpio_devdata *devdata)
{
	int i;
	long time_acc = 0;

	for (i = 0; i < PPS_GEN_CALIBRATE_LOOPS; i++) {
		struct timespec64 ts1, ts2, ts_delta;
		unsigned long irq_flags;

		local_irq_save(irq_flags);
		ktime_get_real_ts64(&ts1);
		gpiod_set_value(devdata->pps_gpio, 0);
		ktime_get_real_ts64(&ts2);
		local_irq_restore(irq_flags);

		ts_delta = timespec64_sub(ts2, ts1);
		time_acc += timespec64_to_ns(&ts_delta);
	}

	devdata->gpio_write_time = time_acc / PPS_GEN_CALIBRATE_LOOPS;
	pr_info("PPS GPIO set takes %ldns\n", devdata->gpio_write_time);
}

/**
 * Driver probe
 */
static int pps_gen_gpio_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct pps_gen_gpio_devdata *devdata;
	struct timespec64 ts;

   if ((pps_sync_mode != PPS_SYNC_RAISING_EDGE) && (pps_sync_mode != PPS_SYNC_FALLING_EDGE))
   {
		dev_err(dev, "pps_gen_gpio: invalid mode value %d\n", pps_sync_mode);
		ret = -EINVAL;
		goto err_alloc;
   }

   if (pps_sync_mode == PPS_SYNC_RAISING_EDGE)
   {
      gpio_sync = 1;
      gpio_reset = 0;
   }
   else
   {
      gpio_sync = 0;
      gpio_reset = 1;
   }

	/* Allocate space for device info. */
	devdata = devm_kzalloc(dev, sizeof(struct pps_gen_gpio_devdata), GFP_KERNEL);
	if (!devdata)
   {
		ret = -ENOMEM;
		goto err_alloc;
	}

	/* There should be a single PPS generator GPIO pin defined in DT. */
	if (of_gpio_named_count(dev->of_node, "pps-gen-gpio") != 1)
   {
		dev_err(dev, "pps_gen_gpio: there should be exactly one pps-gen GPIO defined in DT\n");
		ret = -EINVAL;
		goto err_dt;
	}

	devdata->pps_gpio = devm_gpiod_get(dev, "pps-gen", GPIOD_OUT_LOW);
	if (IS_ERR(devdata->pps_gpio))
   {
		ret = PTR_ERR(devdata->pps_gpio);
		dev_err(dev, "pps_gen_gpio: cannot get PPS GPIO [%d]\n", ret);
		goto err_gpio_get;
	}

	platform_set_drvdata(pdev, devdata);

	ret = gpiod_direction_output(devdata->pps_gpio, GPIOD_OUT_LOW);
	if (ret < 0)
   {
		dev_err(dev, "pps_gen_gpio: cannot configure PPS GPIO\n");
		goto err_gpio_dir;
	}

	pps_gen_calibrate(devdata);

   ret = sysfs_create_group(&pdev->dev.kobj, &state_group);
   if (ret != 0)
   {
      dev_err(dev, "pps_gen_gpio: sysfs creation failed\n");
		goto err_gpio_dir;
   }

	hrtimer_init(&devdata->timer_sync_edge, CLOCK_REALTIME, HRTIMER_MODE_ABS);
	devdata->timer_sync_edge.function = hrtimer_callback_sync_edge;

	hrtimer_init(&devdata->timer_reset_edge, CLOCK_REALTIME, HRTIMER_MODE_ABS);
	devdata->timer_reset_edge.function = hrtimer_callback_reset_edge;

	ktime_get_real_ts64(&ts);

	hrtimer_start(&devdata->timer_reset_edge,
		      ktime_set(ts.tv_sec + 1, 
               (pps_sync_mode == PPS_SYNC_FALLING_EDGE) ?
                  pps_pulse_width * NSEC_PER_MSEC - devdata->gpio_write_time - SAFETY_INTERVAL_NS :
                  (1000 - pps_pulse_width) * NSEC_PER_MSEC - devdata->gpio_write_time - SAFETY_INTERVAL_NS),
		      HRTIMER_MODE_ABS);
	hrtimer_start(&devdata->timer_sync_edge,
		      ktime_set(ts.tv_sec + 1, NSEC_PER_SEC - devdata->gpio_write_time - SAFETY_INTERVAL_NS),
		      HRTIMER_MODE_ABS);
	return 0;

err_gpio_dir:
	devm_gpiod_put(dev, devdata->pps_gpio);
err_gpio_get:
err_dt:
	devm_kfree(dev, devdata);
err_alloc:
	return ret;
}

static int pps_gen_gpio_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pps_gen_gpio_devdata *devdata = platform_get_drvdata(pdev);
   sysfs_remove_group(&pdev->dev.kobj, &state_group);
	devm_gpiod_put(dev, devdata->pps_gpio);
	hrtimer_cancel(&devdata->timer_sync_edge);
	hrtimer_cancel(&devdata->timer_reset_edge);
	return 0;
}

/* The compatible property here defined is searched for in the DT */
static const struct of_device_id pps_gen_gpio_dt_ids[] =
{
	{ .compatible = "pps-gen-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pps_gen_gpio_dt_ids);

static struct platform_driver pps_gen_gpio_driver =
{
	.driver			= {
		.name		= "pps_gen_gpio",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(pps_gen_gpio_dt_ids),
	},
	.probe			= pps_gen_gpio_probe,
	.remove			= pps_gen_gpio_remove,
};

static int __init pps_gen_gpio_init(void)
{
	pr_info(DRVDESC "\n");
	platform_driver_register(&pps_gen_gpio_driver);
	return 0;
}

static void __exit pps_gen_gpio_exit(void)
{
	pr_info("pps_gen_gpio: hrtimer average latency is %ldns\n", hrtimer_avg_latency);
	platform_driver_unregister(&pps_gen_gpio_driver);
}

module_init(pps_gen_gpio_init);
module_exit(pps_gen_gpio_exit);
