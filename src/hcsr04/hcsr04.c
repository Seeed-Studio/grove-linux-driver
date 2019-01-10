/*
 * Experimental Kernel module for HC-SR04 Ultrasonic Range Finder
 *
 * Copyright (C) 2014 James Ward
 *
 * Port to support single signal pin, and device tree configuration
 *
 * Copyright (C) 2019 Seeed Studio
 * Peter Yang <turmary@126.com>
 *
 * Released under the GPL
 *
 */

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#define DEV_NAME	"hcsr04"
#define DEV_ALERT	KERN_ALERT DEV_NAME ": "

/*---------------------------------------------------------------------------*/

/* The Kernel object */
static struct kobject *s_dev_obj = NULL;

/* GPIO pin used for trigger out */
static int trig = 57;
module_param(trig, int, S_IRUGO);

/* GPIO pin used for echo in */
static int echo = 57;
module_param(echo, int, S_IRUGO);

/* High level pulse width, in us */
static int pulse = 20;
module_param(pulse, int, S_IRUGO);

/* Timeout for range finding operation in milliseconds */
const long TIMEOUT_RANGE_FINDING = 60;

/*---------------------------------------------------------------------------*/

typedef struct {
	int count;                   /* Number of interrupts received */
	ktime_t time_stamp[2];       /* Time-stamp of interrupts */
} irq_data_t;

static irq_data_t irq_data;

/* Declare a wait queue to wait for interrupts */
static DECLARE_WAIT_QUEUE_HEAD(wait);

/* Mutex used to prevent simultaneous access */
static DEFINE_MUTEX(mutex);

/*---------------------------------------------------------------------------*/

/* Interrupt handler: called on rising/falling edge */
static irqreturn_t gpioInterruptHandler( int irq, void *dev_id )
{
	/* Get the kernel time */
	ktime_t time_stamp = ktime_get();

	/* Check the cookie */
	if ( dev_id != &irq_data )
		return IRQ_HANDLED;

	/* For the first two interrupts received, store the time-stamp */
	if ( irq_data.count < 2 )
		irq_data.time_stamp[irq_data.count] = time_stamp;

	/* Count the number of interrupts received */
	++irq_data.count;

	/* If we have received two interrupts, wake up */
	if ( irq_data.count > 1 )
		wake_up_interruptible( &wait );

	return IRQ_HANDLED;
}

/*---------------------------------------------------------------------------*/

/* Measures the range, returning the time period in microseconds, and the
 * range in millimetres. The return value is 1 for success, and 0 for failure.
 */
int hcsr04_measure_range(
	long *us,	/* out: time in us */
	long *mm	/* out: range in mm */
) {
	/* The speed of sound in mm/s */
	const long speed_sound_mms = 340270;

	ktime_t elapsed_kt;          /* used to store elapsed time */
	struct timeval elapsed_tv;   /* elapsed time as timeval */
	int irq = 0;                 /* the IRQ number */
	int range_complete = 0;      /* indicates successful range finding */

	/* Acquire the mutex before entering critical section */
	mutex_lock( &mutex );

	/* Initialise variables used by interrupt handler */
	irq_data.count = 0;
	memset( &irq_data.time_stamp, 0, sizeof(irq_data.time_stamp) );

	if (echo == trig) {
		/* make GPIO an output */
		if ( gpio_direction_output(trig, 0) != 0 ) {
			printk( DEV_ALERT "Failed to make sig/trig pin %d an output\n", trig );
			mutex_unlock( &mutex );
			return -1;
		}
	}

	/* Transmit trigger pulse, lasting at least 20us */
	gpio_set_value(trig, 0);
	udelay(2);
	gpio_set_value(trig, 1);
	udelay(pulse);
	gpio_set_value(trig, 0);

	if (echo == trig) {
		/* make GPIO an input */
		if ( gpio_direction_input(echo) ) {
			printk( DEV_ALERT "Failed to make sig/echo pin %d an input\n", echo );
			mutex_unlock( &mutex );
			return -1;
		}
	}

	/* Request an IRQ for the echo GPIO pin, so that we can measure the rising
	 * and falling edge of the pulse from the ranger.
	 */
	irq = gpio_to_irq( echo );
	if ( request_irq(
		irq,
		gpioInterruptHandler,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED,
		"hcsr04_irq",
		&irq_data
	) ) {
		/* Release the mutex */
		mutex_unlock( &mutex );
		return -1;
	}

	/* Wait until we have received two interrupts (indicating that
	 * range finding has completed), or we have timed out
	 */
	wait_event_interruptible_timeout(
		wait,
		irq_data.count == 2,
		msecs_to_jiffies(TIMEOUT_RANGE_FINDING)
	);

	/* Free the interrupt */
	free_irq( irq, &irq_data );

	/* Have we successfully completed ranging? */
	range_complete = (irq_data.count == 2);

	if ( range_complete ) {
		/* Calculate pulse length */
		elapsed_kt = ktime_sub( irq_data.time_stamp[1], irq_data.time_stamp[0] );
		elapsed_tv = ktime_to_timeval( elapsed_kt );
	}

	/* Release the mutex */
	mutex_unlock( &mutex );

	if ( range_complete ) {
		/* Return the time period in microseconds. We ignore the tv_sec,
		 * because the maximum delay should be less than 60ms
		 */
		if ( us != NULL )
			*us = elapsed_tv.tv_usec;

		/* Return the distance, based on speed of sound and the elapsed
		 * time in microseconds.
		 */
		if ( mm != NULL )
			*mm = elapsed_tv.tv_usec * (speed_sound_mms / 10) / 200000/*0*/;

		/* Success */
		return 1;
	} else {
		/* Failure */
		if ( us != NULL ) *us = 0;
		if ( mm != NULL ) *mm = 0;
		return 0;
	}
}

/*---------------------------------------------------------------------------*/

/* This function is called when the 'range' kernel object is read */
static ssize_t range_show(
	struct kobject *object,
	struct kobj_attribute *attribute,
	char *buffer
) {
	long us = 0;
	long mm = 0;

	/* Outputs: <mm> <us> <good> where:
	 * <mm> = Distance in millimetres
	 * <us> = Time in microseconds
	 * <good> = 1 for success, 0 for failure
	 */
	if ( hcsr04_measure_range( &us, &mm ) != 0 )
		return sprintf( buffer, "%ld %ld 1\n", mm, us );
	else
		return sprintf( buffer, "0 0 0\n" );
}

/*---------------------------------------------------------------------------*/

/* Attribute representing the 'range' kernel object, which is read only */
static struct kobj_attribute range_attr = __ATTR_RO(range);

/*---------------------------------------------------------------------------*/

/* List of all attributes */
static struct attribute *attrs[] = {
	&range_attr.attr,
	NULL    /* terminate the list */
};

/*---------------------------------------------------------------------------*/

/* Attribute group */
static struct attribute_group attr_group = {
	.attrs = attrs
};

/*---------------------------------------------------------------------------*/

/* Initialise GPIO */
static int hcsr04_gpio_init( void )
{
	/* check that trigger GPIO is valid */
	if ( !gpio_is_valid(trig) ){
		printk( DEV_ALERT "trig GPIO %d is not valid\n", trig );
		return -EINVAL;
	}

	/* request the GPIO pin */
	if ( gpio_request(trig, DEV_NAME) != 0 ) {
		printk( DEV_ALERT "Unable to request trig GPIO %d\n", trig );
		return -EINVAL;
	}

	/* make GPIO an output */
	if ( gpio_direction_output(trig, 0) != 0 ) {
		printk( DEV_ALERT "Failed to make trig GPIO %d an output\n", trig );
		return -EINVAL;
	}

	/* check if trigger and echo are the same GPIO */
	if ( trig == echo ) {
		printk( DEV_ALERT "sig/echo GPIO %d is same as sig/trig\n", echo);
		return 0;
	}

	/* check that echo GPIO is valid */
	if ( !gpio_is_valid(echo) ){
		printk( DEV_ALERT "echo GPIO %d is not valid\n", echo );
		return -EINVAL;
	}

	/* request the GPIO pin */
	if ( gpio_request(echo, DEV_NAME) != 0 ) {
		printk( DEV_ALERT "Unable to request echo GPIO %d\n", echo );
		return -EINVAL;
	}

	/* make GPIO an input */
	if ( gpio_direction_input(echo) != 0 ) {
		printk( DEV_ALERT "Failed to make echo GPIO %d an input\n", echo );
		return -EINVAL;
	}

	return 0;
}

/*---------------------------------------------------------------------------*/

/* Free GPIO */
static void hcsr04_gpio_free( void )
{
	gpio_free(trig);
	gpio_free(echo);
}

/*---------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id hcsr04_dt_ids[] = {
	{ .compatible = DEV_NAME, },
	{}
};
MODULE_DEVICE_TABLE(of, hcsr04_dt_ids);
#endif

/* device initialisation function */
static int hcsr04_probe(struct platform_device* pdev)
{
	int ret = -1;
	struct device *dev = &pdev->dev;
	#ifdef CONFIG_OF
	struct device_node *node = dev->of_node;

	if (node) {
		/*
		 * first  gpio: trigger pin
		 * second gpio: echo    pin
		 */
		ret = of_get_gpio(node, 0);
		if (ret >= 0) {
			trig = ret;
			ret = of_get_gpio(node, 1);
			if (ret >= 0) {
				echo = ret;
			} else {
				echo = trig;
			}
		}
	}
	#endif

	/* Create /sys/kernel/hcsr04 representing the HCSR04 sensor */
	s_dev_obj = kobject_create_and_add( DEV_NAME, kernel_kobj );
	if ( s_dev_obj == NULL )
		return -ENOMEM;

	/* Create the files associated with this kobject */
	ret = sysfs_create_group( s_dev_obj, &attr_group );
	if ( ret ) {
		/* Failed: clean up */
		kobject_put( s_dev_obj );
		return ret;
	}

	/* Set up the GPIO */
	if ( hcsr04_gpio_init() < 0 )
		return -EINVAL;

	return ret;
}

/*---------------------------------------------------------------------------*/

/* Module exit function */
static int hcsr04_remove(struct platform_device* pdev)
{
	/* Decrement refcount and clean up if zero */
	kobject_put( s_dev_obj );

	/* Free GPIO */
	hcsr04_gpio_free();
	return 0;
}

/*---------------------------------------------------------------------------*/

static struct platform_driver hcsr04_driver = {
	.driver = {
		.name  = DEV_NAME,
		.owner = THIS_MODULE,
		#ifdef CONFIG_OF
		.of_match_table = hcsr04_dt_ids,
		#endif
	},
	.probe  = hcsr04_probe,
	.remove = hcsr04_remove,
};

module_platform_driver(hcsr04_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("James Ward");
MODULE_AUTHOR("Peter Yang <turmary@126.com>");

/*---------------------------------------------------------------------------*/
