/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
//#include <linux/fb.h>
//#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/gpio.h>
//#include <linux/pwm_backlight.h>
//#include <linux/slab.h>

#define PWM_120MS				2
#define PWM_38KHz				3

static struct pwm_device	*pwm_120;
static struct pwm_device	*pwm_38k;

static int	ir_enabled = 1;


static int im300b_ir_update_status(void)
{

	pr_err(" *** Update iM300B IrDA status :: enabled = %d ***\n", ir_enabled);

	if (ir_enabled) {
		pwm_config(pwm_120, 120000000, 500000000);
		pwm_enable(pwm_120);

		pwm_config(pwm_38k, 13158, 26316);
		pwm_enable(pwm_38k);
	} else {
		pwm_config(pwm_120, 0, 500000000);
		pwm_disable(pwm_120);

		pwm_config(pwm_38k, 0, 26316);
		pwm_disable(pwm_38k);
	}
	return 0;
}

static int im300b_ir_probe(struct platform_device *pdev)
{
	int ret;

	dev_err(&pdev->dev, " *** iM300B IrDA init! ***\n");

	pwm_120 = pwm_request(PWM_120MS, "ir_120ms");
	if (IS_ERR(pwm_120)) {
		dev_err(&pdev->dev, "unable to request PWM for IrDA 120ms\n");
		ret = PTR_ERR(pwm_120);
		return ret;
	}

	pwm_38k = pwm_request(PWM_38KHz, "ir_120ms");
	if (IS_ERR(pwm_38k)) {
		dev_err(&pdev->dev, "unable to request PWM for IrDA 38khz\n");
		ret = PTR_ERR(pwm_38k);
		pwm_free(pwm_120);
		return ret;
	}

	gpio_direction_input(IMX_GPIO_NR(6, 11));

	im300b_ir_update_status();

	return 0;
}

static int im300b_ir_remove(struct platform_device *pdev)
{

	pwm_config(pwm_120, 0, 500000000);
	pwm_disable(pwm_120);
	pwm_free(pwm_120);

	pwm_config(pwm_38k, 0, 26316);
	pwm_disable(pwm_38k);
	pwm_free(pwm_38k);

	return 0;
}

#ifdef CONFIG_PM
static int im300b_ir_suspend(struct platform_device *pdev,
				 pm_message_t state)
{

	pwm_config(pwm_120, 0, 500000000);
	pwm_disable(pwm_120);

	pwm_config(pwm_38k, 0, 26316);
	pwm_disable(pwm_38k);

	return 0;
}

static int im300b_ir_resume(struct platform_device *pdev)
{
	im300b_ir_update_status();
	return 0;
}
#else
#define im300b_ir_suspend	NULL
#define im300b_ir_resume	NULL
#endif

static struct platform_driver im300b_ir_driver = {
	.driver		= {
		.name	= "im300b_ir",
		.owner	= THIS_MODULE,
	},
	.probe		= im300b_ir_probe,
	.remove		= im300b_ir_remove,
	.suspend	= im300b_ir_suspend,
	.resume		= im300b_ir_resume,
};

static int __init im300b_ir_init(void)
{
	return platform_driver_register(&im300b_ir_driver);
}
module_init(im300b_ir_init);

static void __exit im300b_ir_exit(void)
{
	platform_driver_unregister(&im300b_ir_driver);
}
module_exit(im300b_ir_exit);

MODULE_DESCRIPTION("PWM based IrDA Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:im300b_ir");

