/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @defgroup Framebuffer Framebuffer Driver for SDC and ADC.
 */

/*!
 * @file mxcfb_innolux_wvga.c
 *
 * @brief MXC Frame buffer driver for SDC
 *
 * @ingroup Framebuffer
 */

/*!
 * Include files
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/fsl_devices.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mxcfb.h>
#include <linux/regulator/consumer.h>
#include <mach/hardware.h>

static void lcd_poweron(void);
static void lcd_poweroff(void);

static struct platform_device *plcd_dev;

static struct fb_videomode video_modes[] = {
	{
	/* 800x480 @ 60 Hz , pixel clk @ 33.3MHz */
	"INNOLUX-WVGA", 60, 800, 480, 30030, 46, 210, 23, 22, 1, 1,
	FB_SYNC_CLK_LAT_FALL,
	FB_VMODE_NONINTERLACED,
	0,},
};

static void lcd_init_fb(struct fb_info *info)
{
	struct fb_var_screeninfo var;

	memset(&var, 0, sizeof(var));

	fb_videomode_to_var(&var, &video_modes[0]);

	var.activate = FB_ACTIVATE_ALL;
	var.yres_virtual = var.yres * 3;

	console_lock();
	info->flags |= FBINFO_MISC_USEREVENT;
	fb_set_var(info, &var);
	info->flags &= ~FBINFO_MISC_USEREVENT;
	console_unlock();
}

static int lcd_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct fb_event *event = v;

	if (strcmp(event->info->fix.id, "mxc_elcdif_fb"))
		return 0;

	switch (val) {
	case FB_EVENT_BLANK:
		if ((event->info->var.xres != 800) ||
		    (event->info->var.yres != 480)) {
			break;
		}
		if (*((int *)event->data) == FB_BLANK_UNBLANK)
			lcd_poweron();
		else
			lcd_poweroff();
		break;
	}
	return 0;
}

static struct notifier_block nb = {
	.notifier_call = lcd_fb_event,
};

/*!
 * This function is called whenever the platform device is detected.
 *
 * @param	pdev	the platform device
 *
 * @return 	Returns 0 on SUCCESS and error on FAILURE.
 */
static int __devinit lcd_probe(struct platform_device *pdev)
{
	int i;
	struct fsl_mxc_lcd_platform_data *plat = pdev->dev.platform_data;

	if (plat) {
		if (plat->reset)
			plat->reset();
	}

	for (i = 0; i < num_registered_fb; i++) {
		if (strcmp(registered_fb[i]->fix.id, "mxc_elcdif_fb") == 0) {
			lcd_init_fb(registered_fb[i]);
			fb_show_logo(registered_fb[i], 0);
			lcd_poweron();
		}
	}

	fb_register_client(&nb);

	plcd_dev = pdev;

	return 0;
}

static int __devexit lcd_remove(struct platform_device *pdev)
{
	fb_unregister_client(&nb);

	return 0;
}

#ifdef CONFIG_PM
static int lcd_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int lcd_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define lcd_suspend NULL
#define lcd_resume NULL
#endif

/*!
 * platform driver structure for SEIKO WVGA
 */
static struct platform_driver lcd_driver = {
	.driver = {
		   .name = "lcd_innolux"},
	.probe = lcd_probe,
	.remove = __devexit_p(lcd_remove),
	.suspend = lcd_suspend,
	.resume = lcd_resume,
};

/*
 * Send Power
 *
 */
static void lcd_poweron(void)
{
}

/*
 * Send Power Off
 *
 */
static void lcd_poweroff(void)
{
}

static int __init innolux_wvga_lcd_init(void)
{
	return platform_driver_register(&lcd_driver);
}

static void __exit innolux_wvga_lcd_exit(void)
{
	platform_driver_unregister(&lcd_driver);
}

module_init(innolux_wvga_lcd_init);
module_exit(innolux_wvga_lcd_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("INNOLUX WVGA LCD init driver");
MODULE_LICENSE("GPL");
