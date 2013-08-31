/*
 * Copyright (C) 2007 Sascha Hauer, Pengutronix
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 */

#include <common.h>
#include <net.h>
#include <init.h>
#include <environment.h>
#include <asm/armlinux.h>
#include <generated/mach-types.h>
#include <partition.h>
#include <fs.h>
#include <fcntl.h>
#include <io.h>
#include <asm/hardware.h>
#include <nand.h>
#include <sizes.h>
#include <linux/mtd/nand.h>
#include <mach/cpu.h>
#include <mach/at91_pmc.h>
#include <mach/board.h>
#include <gpio.h>
#include <mach/io.h>
#include <mach/iomux.h>
#include <mach/at91sam9_smc.h>
#include <dm9000.h>
#include <gpio_keys.h>
#include <readkey.h>
#include <led.h>
#include <spi/spi.h>

#ifdef CONFIG_CPODC2_MSP430
#include "msp430.h"
#endif

static struct atmel_nand_data nand_pdata = {
	.ale		= 22,
	.cle		= 21,
	.det_pin	= -EINVAL,
	.rdy_pin	= AT91_PIN_PC15,
	.enable_pin	= AT91_PIN_PC14,
#if defined(CONFIG_MTD_NAND_ATMEL_BUSWIDTH_16)
	.bus_width_16	= 1,
#else
	.bus_width_16	= 0,
#endif
	.on_flash_bbt	= 1,
};

static struct sam9_smc_config dc_nand_smc_config = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 1,
	.ncs_write_setup	= 0,
	.nwe_setup		= 1,

	.ncs_read_pulse		= 3,
	.nrd_pulse		= 3,
	.ncs_write_pulse	= 3,
	.nwe_pulse		= 3,

	.read_cycle		= 5,
	.write_cycle		= 5,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE,
	.tdf_cycles		= 2,
};

static void dc_add_device_nand(void)
{
	/* setup bus-width (8 or 16) */
	if (nand_pdata.bus_width_16)
		dc_nand_smc_config.mode |= AT91_SMC_DBW_16;
	else
		dc_nand_smc_config.mode |= AT91_SMC_DBW_8;

	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(0, 3, &dc_nand_smc_config);

	at91_add_device_nand(&nand_pdata);
}

/*
 * USB OHCI Host port
 */
#ifdef CONFIG_USB_OHCI_AT91
static struct at91_usbh_data  __initdata usbh_data = {
	.ports		= 1,
	//.vbus_pin	= { AT91_PIN_PD0,  -EINVAL },
};

static void __init dc_add_device_usbh(void)
{
	if (cpu_is_at91sam9g10())  // add it just for boards with 9g10
		at91_add_device_usbh_ohci(&usbh_data);
}
#else
static void __init dc_add_device_usbh(void) {}
#endif


#if defined(CONFIG_USB_GADGET_DRIVER_AT91)
/*
 * USB Device port
 */
static struct at91_udc_data __initdata dc_udc_data = {
	.vbus_pin	= AT91_PIN_PB29,
	.pullup_pin	= 0,
};

static void dc_add_device_udc(void)
{
	at91_add_device_udc(&dc_udc_data);
}
#else
static void dc_add_device_udc(void) {}
#endif

/*
 * LCD Controller
 */
#if defined(CONFIG_DRIVER_VIDEO_ATMEL)
static int dc_gpio_request_output(int gpio, const char *name)
{
	int ret;

	ret = gpio_request(gpio, name);
	if (ret) {
		pr_err("%s: can not request gpio %d (%d)\n", name, gpio, ret);
		return ret;
	}

	ret = gpio_direction_output(gpio, 1);
	if (ret)
		pr_err("%s: can not configure gpio %d as output (%d)\n", name, gpio, ret);
	return ret;
}

/* TFT */
static struct fb_videomode at91_tft_vga_modes[] = {
	{
		.name		= "320x240@60",
		.refresh	= 60,
		.xres		= 320,		.yres		= 240,
		.pixclock	= KHZ2PICOS(6210),

		.left_margin	= 62,		.right_margin	= 14,
		.upper_margin	= 18,		.lower_margin	= 3,
		.hsync_len	= 5,		.vsync_len	= 1,

		.sync		= FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};

#define AT91SAM9261_DEFAULT_TFT_LCDCON2	(ATMEL_LCDC_MEMOR_LITTLE \
					| ATMEL_LCDC_DISTYPE_TFT \
					| ATMEL_LCDC_CLKMOD_ALWAYSACTIVE \
					| ATMEL_LCDC_INVVD_INVERTED \
					| ATMEL_LCDC_INVFRAME_INVERTED \
					| ATMEL_LCDC_INVLINE_INVERTED \
					| ATMEL_LCDC_INVCLK_INVERTED \
					| ATMEL_LCDC_INVDVAL_INVERTED \
					)

static void at91_lcdc_tft_power_control(int on)
{
	if (on)
		gpio_set_value(AT91_PIN_PA12, 0);	/* power up */
	else
		gpio_set_value(AT91_PIN_PA12, 1);	/* power down */
}

static struct atmel_lcdfb_platform_data dc_lcdc_data = {
	.lcdcon_is_backlight	= false,
	.default_bpp			= 16,
	.default_dmacon			= ATMEL_LCDC_DMAEN,
	.default_lcdcon2		= AT91SAM9261_DEFAULT_TFT_LCDCON2,
	.guard_time			= 1,
	.atmel_lcdfb_power_control	= at91_lcdc_tft_power_control,
	.mode_list			= at91_tft_vga_modes,
	.num_modes			= ARRAY_SIZE(at91_tft_vga_modes),
};

static int at91_lcdc_gpio(void)
{
	return dc_gpio_request_output(AT91_PIN_PA12, "lcdc_tft_power");
}

static void dc_add_device_lcdc(void)
{
	if (at91_lcdc_gpio())
		return;

	dc_lcdc_data.have_intensity_bit = cpu_is_at91sam9261();
	
	dc_lcdc_data.lcd_wiring_mode = cpu_is_at91sam9261() ? ATMEL_LCDC_WIRING_RGB
	                                                    : ATMEL_LCDC_WIRING_BGR;

	at91_add_device_lcdc(&dc_lcdc_data);
}

#else
static void dc_add_device_lcdc(void) {}
#endif

static void __init dc_add_device_buttons(void)
{
	// this first call is necessary to turn on clocks for gpio...
	gpio_request(AT91_PIN_PB30,"go_button");
	gpio_direction_input(AT91_PIN_PB30);
	// don't use pullup here.. so don't need this hack.
	at91_set_gpio_input(AT91_PIN_PB30, 0);
	//at91_set_deglitch(AT91_PIN_PB30, 1);

	gpio_request(AT91_PIN_PC2, "ts_pen");
	gpio_direction_input(AT91_PIN_PC2);
	at91_set_gpio_input (AT91_PIN_PC2, 1); // pullup
	at91_set_deglitch   (AT91_PIN_PC2, 1);
	
	export_env_ull("dfu_button", AT91_PIN_PB30);
}

/*
 * SPI related devices
 */
#if defined(CONFIG_DRIVER_SPI_ATMEL)
static struct spi_board_info dc_spi_devices[] = {
	{	/* DataFlash chip */
		.name		= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
		.bus_num	= 0,
	},
};

static unsigned spi0_standard_cs[] = { AT91_PIN_PA3, AT91_PIN_PA6};
static struct at91_spi_platform_data spi_pdata = {
	.chipselect = spi0_standard_cs,
	.num_chipselect = ARRAY_SIZE(spi0_standard_cs),
};

static void dc_add_device_spi(void)
{
	spi_register_board_info(dc_spi_devices,
				ARRAY_SIZE(dc_spi_devices));
	at91_add_device_spi(0, &spi_pdata);
}
#else
static void dc_add_device_spi(void) {}
#endif

static int cpodc2_mem_init(void)
{
	at91_add_device_sdram(0);

	return 0;
}
mem_initcall(cpodc2_mem_init);

static int __init main_clock(void)
{
	int tmp;
	static int main_clock = 0;

	// this works for both boards, but only if at91boostrap was used first to setup the PLL lock.
	if (!main_clock) {
		do { // wait for PLL lock..
			tmp = at91_pmc_read(AT91_CKGR_MCFR);
		} while (!(tmp & AT91_PMC_MAINRDY));
		tmp = (tmp & AT91_PMC_MAINF) * (AT91_SLOW_CLOCK / 16);
		main_clock = (tmp > 19500000) ? 20000000 : 18432000;
	}

	return main_clock;
}

static int cpodc2_devices_init(void)
{
	u32 board_revision = 0;

	dc_add_device_spi();
	dc_add_device_nand();
	dc_add_device_udc();
	dc_add_device_usbh();
	dc_add_device_buttons();
	dc_add_device_lcdc();

	if (! IS_ENABLED(CONFIG_MTD_DATAFLASH)) {
		if (IS_ENABLED(CONFIG_AT91_LOAD_BAREBOX_SRAM)) {
			devfs_add_partition("nand0", 0, SZ_256K + SZ_128K, DEVFS_PARTITION_FIXED, "self_raw");
			export_env_ull("borebox_first_stage", 1);
		} else {
			devfs_add_partition("nand0", 0x00000, SZ_128K, DEVFS_PARTITION_FIXED, "bootstrap_raw");
			dev_add_bb_dev("bootstrap_raw","bootstrap");
			devfs_add_partition("nand0", SZ_128K, SZ_256K, DEVFS_PARTITION_FIXED, "self_raw");
		}
		dev_add_bb_dev("self_raw", "self0");
		devfs_add_partition("nand0", SZ_256K + SZ_128K, SZ_128K, DEVFS_PARTITION_FIXED, "env_raw");
		dev_add_bb_dev("env_raw", "env0");
		devfs_add_partition("nand0", SZ_512K, SZ_128K, DEVFS_PARTITION_FIXED, "env_raw1");
		dev_add_bb_dev("env_raw1", "env1");
	} else { // dataflash partitions
		devfs_add_partition("dataflash0", 0x00000, 0x4200,  DEVFS_PARTITION_FIXED, "bootstrap");
		devfs_add_partition("dataflash0", 0x04200, 0x4200,  DEVFS_PARTITION_FIXED, "env0");
		devfs_add_partition("dataflash0", 0x08400, 0x39C00, DEVFS_PARTITION_FIXED, "self0");
	}

	// we should probably also get revision data from the msp430
	// but it takes a while to load...
	// just fix based on whether CPU is 9g10
	
	if (nand_pdata.bus_width_16)
		board_revision |= (1<<31);

	if (cpu_is_at91sam9g10())
		board_revision |= (0x01<<8);

	if (main_clock() > 19500000)
		board_revision |= (0x01<<10);

	armlinux_set_revision(board_revision);
	armlinux_set_bootparams((void *)(AT91_CHIPSELECT_1 + 0x100));
	armlinux_set_architecture(MACH_TYPE_CPODC2);

	return 0;
}
device_initcall(cpodc2_devices_init);

static int cpodc2_console_init(void)
{
	barebox_set_model("CPO Science DataCollector II");
	barebox_set_hostname("cpodc2");
	
	at91_register_uart(0, 0);
#ifdef CONFIG_CPODC2_MSP430
	at91_register_uart(1, 0);
	// deactivate console and use it for the msp command
	cpodc2_msp430_init_console("atmel_usart1");
#endif
	return 0;
}
console_initcall(cpodc2_console_init);

static int cpodc2_main_clock(void)
{
	// bootloader should have set the frequency:
	at91_set_main_clock(main_clock());
	
	return 0;
}
pure_initcall(cpodc2_main_clock);

