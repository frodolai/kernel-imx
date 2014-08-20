/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
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

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <sound/wm8960.h>
#include <linux/mfd/mxc-hdmi-core.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_ar6mxcs.h"
#include "board-mx6solo_ar6mxcs.h"

#define AR6MXCS_SW_RST        IMX_GPIO_NR(1, 0)
#define AR6MXCS_RTC_INT       IMX_GPIO_NR(6, 7)
#define AR6MXCS_SPK_DET	      IMX_GPIO_NR(1, 20)
#define AR6MXCS_MIC_DET	      IMX_GPIO_NR(1, 16)
#define AR6MXCS_LVDS0_PWR     IMX_GPIO_NR(4, 8)
#define AR6MXCS_BL0_PWR       IMX_GPIO_NR(4, 9)
#define AR6MXCS_PHY_INT	      IMX_GPIO_NR(1, 28)

#define AR6MXCS_SD3_CD      	IMX_GPIO_NR(7, 0)

#define AR6MXCS_PCIE_DIS_B	  IMX_GPIO_NR(1, 9)
#define AR6MXCS_PCIE_RST_B	  IMX_GPIO_NR(7, 11)

static struct clk *clko;
static int caam_enabled;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

static const struct esdhc_platform_data mx6q_ar6mxcs_sd3_data __initconst = {
	.cd_gpio = AR6MXCS_SD3_CD,
	//.wp_gpio = AR6MXCS_SD3_WP,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
};

static const struct esdhc_platform_data mx6q_ar6mxcs_sd4_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct anatop_thermal_platform_data
	mx6q_ar6mxcs_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_ar6mxcs_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(3, NULL);
}

static inline void imx6q_ar6mxcs_init_ldb(void)
{
	gpio_request(AR6MXCS_LVDS0_PWR, "lvds0");
	gpio_direction_output(AR6MXCS_LVDS0_PWR, 1);
	gpio_free(AR6MXCS_LVDS0_PWR);
	gpio_request(AR6MXCS_BL0_PWR, "bl0_pwr");
	gpio_direction_output(AR6MXCS_BL0_PWR, 1);
	gpio_free(AR6MXCS_BL0_PWR);
}

static int mx6q_ar6mxcs_fec_phy_init(struct phy_device *phydev)
{
	/* RX Data Pad Skew Register */
	phy_write(phydev, 0xd, 0x0002);
	phy_write(phydev, 0xe, 0x0005);
	phy_write(phydev, 0xd, 0xc002);
	phy_write(phydev, 0xe, 0x7777);

	/* TX Data Pad Skew Register */
	phy_write(phydev, 0xd, 0x0002);
	phy_write(phydev, 0xe, 0x0006);
	phy_write(phydev, 0xd, 0xc002);
	phy_write(phydev, 0xe, 0x7777);

	/* rx/tx data delay no changed, clock set max */
	phy_write(phydev, 0xd, 0x0002);
	phy_write(phydev, 0xe, 0x0008);
	phy_write(phydev, 0xd, 0xc002);
	phy_write(phydev, 0xe, 0x7fff);

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_ar6mxcs_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
	//.gpio_irq = AR6MXCS_PHY_INT,
};

static struct imx_ssi_platform_data mx6_ar6mxcs_ssi1_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct platform_device mx6_ar6mxcs_audio_wm8960_device = {
	.name = "imx-wm8960",
};

static struct mxc_audio_platform_data wm8960_data;

static int wm8960_clk_enable(int enable)
{
	if (enable)
		clk_enable(clko);
	else
		clk_disable(clko);
	return 0;
}

static int mxc_wm8960_init(void)
{
	struct clk *new_parent;
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	new_parent = clk_get(NULL, "clko2_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko, 24000000);
	pr_err("WM8960 mclk freq %d!\n", rate);
	clk_set_rate(clko, rate);
	wm8960_data.sysclk = rate;
	clk_enable(clko);

	return 0;
}

static struct mxc_audio_platform_data wm8960_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_gpio = -1,
	.mic_gpio = -1,
	.init = mxc_wm8960_init,
	.clock_enable = wm8960_clk_enable,
};

static int __init imx6q_init_audio(void)
{
	mxc_register_device(&mx6_ar6mxcs_audio_wm8960_device, &wm8960_data);
	imx6q_add_imx_ssi(1, &mx6_ar6mxcs_ssi1_pdata);

	mxc_wm8960_init();

	return 0;
}

static struct imxi2c_platform_data mx6q_ar6mxcs_i2c_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("wm8960", 0x1a),
		.platform_data = &wm8960_data,
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("s35390a", 0x30),
		.irq  = gpio_to_irq(AR6MXCS_RTC_INT),
	},
};

static void imx6q_ar6mxcs_usbotg_vbus(bool on)
{
}

static void imx6q_ar6mxcs_host1_vbus(bool on)
{
}

static void __init imx6q_ar6mxcs_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	mx6_set_otghost_vbus_func(imx6q_ar6mxcs_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6q_ar6mxcs_host1_vbus);
}

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data ar6mxcs_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "hdmi",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "1920x1080M@60",
	.default_bpp = 32,
	.int_clk = false,
	.late_init = false,
	},
	{
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On mx6x ar6mxcs board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6solo_ar6mxcs_hdmi_ddc_pads,
			ARRAY_SIZE(mx6solo_ar6mxcs_hdmi_ddc_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_ar6mxcs_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_ar6mxcs_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6solo_ar6mxcs_i2c2_pads,
			ARRAY_SIZE(mx6solo_ar6mxcs_i2c2_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_ar6mxcs_i2c2_pads,
			ARRAY_SIZE(mx6q_ar6mxcs_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
	.phy_reg_vlev = 0x0294,
	.phy_reg_cksymtx = 0x800d,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
	.disp_id = 0,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 0,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SEP0,
	.sec_ipu_id = 1,
	.sec_disp_id = 0,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	},
};

struct imx_vout_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_vout_mem vout_mem __initdata = {
	.res_msize = SZ_128M,
};

static void ar6mxcs_suspend_enter(void)
{
}

static void ar6mxcs_suspend_exit(void)
{
}

static const struct pm_platform_data mx6q_ar6mxcs_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = ar6mxcs_suspend_enter,
	.suspend_exit = ar6mxcs_suspend_exit,
};

static struct regulator_consumer_supply ar6mxcs_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data ar6mxcs_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(ar6mxcs_vmmc_consumers),
	.consumer_supplies = ar6mxcs_vmmc_consumers,
};

static struct fixed_voltage_config ar6mxcs_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &ar6mxcs_vmmc_init,
};

static struct platform_device ar6mxcs_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &ar6mxcs_vmmc_reg_config,
	},
};

static struct mxc_dvfs_platform_data ar6mxcs_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{

	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit*/
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

static const struct imx_pcie_platform_data mx6_ar6mxcs_pcie_data __initconst = {
	.pcie_pwr_en	= -EINVAL,
	.pcie_rst	= AR6MXCS_PCIE_RST_B,
	.pcie_wake_up	= -EINVAL,
	.pcie_dis	= AR6MXCS_PCIE_DIS_B,
};

/*!
 * Board specific initialization.
 */
static void __init mx6_ar6mxcs_board_init(void)
{
	int i;
	int ret;
	struct clk *clko;
	struct clk *new_parent;
	int rate;
	struct platform_device *voutdev;

	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_ar6mxcs_pads,
			ARRAY_SIZE(mx6q_ar6mxcs_pads));
	} else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6solo_ar6mxcs_pads,
			ARRAY_SIZE(mx6solo_ar6mxcs_pads));
	}

	gp_reg_id = ar6mxcs_dvfscore_data.reg_id;
	soc_reg_id = ar6mxcs_dvfscore_data.soc_id;
	mx6q_ar6mxcs_init_uart();

	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.sec_ipu_id = 0;
	}
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);
	imx6q_ar6mxcs_init_ldb();
	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(ar6mxcs_fb_data); i++)
			imx6q_add_ipuv3fb(i, &ar6mxcs_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(ar6mxcs_fb_data); i++)
			imx6q_add_ipuv3fb(i, &ar6mxcs_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);

	imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_imx_i2c(0, &mx6q_ar6mxcs_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_ar6mxcs_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_ar6mxcs_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_ar6mxcs_anatop_thermal_data);

	imx6_init_fec(fec_data);

	imx6q_add_pm_imx(0, &mx6q_ar6mxcs_pm_data);

	/* Move sd4 to first because sd4 connect to emmc.
	   Mfgtools want emmc is mmcblk0 and other sd card is mmcblk1.
	*/
	imx6q_add_sdhci_usdhc_imx(3, &mx6q_ar6mxcs_sd4_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_ar6mxcs_sd3_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_ar6mxcs_init_usb();

	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&ar6mxcs_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

#if 0
	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
#endif

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&ar6mxcs_dvfscore_data);

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	/* Add PCIe RC interface support */
	imx6q_add_pcie(&mx6_ar6mxcs_pcie_data);

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
}

extern void __iomem *twd_base;
static void __init mx6_ar6mxcs_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART4_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_ar6mxcs_timer = {
	.init   = mx6_ar6mxcs_timer_init,
};

static void __init mx6q_ar6mxcs_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
			SZ_4K, SZ_2G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif
}

/*
 * initialize __mach_desc_MX6Q_AR6MXCS data structure.
 */
MACHINE_START(MX6Q_AR6MXCS, "AR6MXCS Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_ar6mxcs_board_init,
	.timer = &mx6_ar6mxcs_timer,
	.reserve = mx6q_ar6mxcs_reserve,
MACHINE_END
