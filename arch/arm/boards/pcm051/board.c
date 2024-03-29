/*
 * pcm051 - phyCORE-AM335x Board Initalization Code
 *
 * Copyright (C) 2012 Teresa Gámez, Phytec Messtechnik GmbH
 *
 * Based on arch/arm/boards/omap/board-beagle.c
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
 */

#include <common.h>
#include <init.h>
#include <io.h>
#include <sizes.h>
#include <ns16550.h>
#include <asm/armlinux.h>
#include <generated/mach-types.h>
#include <linux/phy.h>
#include <mach/am33xx-devices.h>
#include <mach/am33xx-generic.h>
#include <mach/am33xx-mux.h>
#include <mach/am33xx-silicon.h>
#include <mach/cpsw.h>
#include <spi/spi.h>
#include <spi/flash.h>
#include <i2c/i2c.h>
#include <i2c/at24.h>

#include "mux.h"

/**
 * @brief UART serial port initialization
 * arch
 *
 * @return result of device registration
 */
static int pcm051_console_init(void)
{
	/* Register the serial port */
	am33xx_add_uart0();

	return 0;
}
console_initcall(pcm051_console_init);

static int pcm051_mem_init(void)
{
	omap_add_ram0(SZ_512M);

	return 0;
}
mem_initcall(pcm051_mem_init);

static struct flash_platform_data pcm051_spi_flash = {
	.name	= "nor",
	.type	= "w25q64",
};

/*
* SPI Flash works at 80Mhz however the SPI controller runs with 48MHz.
* So setup Max speed to be less than the controller speed.
*/
static struct spi_board_info pcm051_spi_board_info[] = {
	{
		.name		= "m25p80",
		.platform_data	= &pcm051_spi_flash,
		.max_speed_hz	= 24000000,
		.bus_num	= 0,
		.chip_select	= 0,
	},
};

static struct cpsw_slave_data cpsw_slaves[] = {
	{
		.phy_id		= 0,
		.phy_if		= PHY_INTERFACE_MODE_RMII,
	},
};

static struct cpsw_platform_data cpsw_data = {
	.slave_data	= cpsw_slaves,
	.num_slaves	= ARRAY_SIZE(cpsw_slaves),
};

static struct i2c_board_info i2c0_devices[] = {
	{
		I2C_BOARD_INFO("24c32", 0x52),
	},
};

static void pcm051_spi_init(void)
{
	int ret;

	am33xx_enable_spi0_pin_mux();

	ret = spi_register_board_info(pcm051_spi_board_info,
					ARRAY_SIZE(pcm051_spi_board_info));
	am33xx_add_spi0();
}

static void pcm051_eth_init(void)
{
	am33xx_register_ethaddr(0, 0);

	writel(0x49, AM33XX_MAC_MII_SEL);

	am33xx_enable_rmii1_pin_mux();

	am33xx_add_cpsw(&cpsw_data);
}

static void pcm051_i2c_init(void)
{
	am33xx_enable_i2c0_pin_mux();

	i2c_register_board_info(0, i2c0_devices, ARRAY_SIZE(i2c0_devices));

	am33xx_add_i2c0(NULL);
}

static int pcm051_devices_init(void)
{
	pcm051_enable_mmc0_pin_mux();

	am33xx_add_mmc0(NULL);

	pcm051_spi_init();
	pcm051_eth_init();
	pcm051_i2c_init();

	devfs_add_partition("nor0", 0x00000, SZ_128K,
					DEVFS_PARTITION_FIXED, "xload");
	devfs_add_partition("nor0", SZ_128K, SZ_512K,
					DEVFS_PARTITION_FIXED, "self0");
	devfs_add_partition("nor0", SZ_128K + SZ_512K, SZ_128K,
					DEVFS_PARTITION_FIXED, "env0");

	armlinux_set_bootparams((void *)(AM33XX_DRAM_ADDR_SPACE_START + 0x100));
	armlinux_set_architecture(MACH_TYPE_PCM051);

	return 0;
}
device_initcall(pcm051_devices_init);
