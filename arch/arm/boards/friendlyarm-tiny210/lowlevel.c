/*
 * Copyright (C) 2012 Alexey Galakhov
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <config.h>
#include <common.h>
#include <init.h>
#include <io.h>
#include <sizes.h>
#include <asm/barebox-arm.h>
#include <asm/barebox-arm-head.h>
#include <asm/sections.h>
#include <mach/s3c-iomap.h>
#include <mach/s3c-clocks.h>
#include <mach/s3c-generic.h>

#define IRAM_CODE_BASE		0xD0020010

/*
 * iROM boot from MMC
 * TODO: replace this by native boot
 */

#define ADDR_V210_SDMMC_BASE	0xD0037488
#define ADDR_CopySDMMCtoMem	0xD0037F98

int __bare_init s5p_irom_load_mmc(void *dest, uint32_t start_block, uint16_t block_count)
{
	typedef uint32_t (*func_t) (int32_t, uint32_t, uint16_t, uint32_t*, int8_t);
	uint32_t chbase = readl(ADDR_V210_SDMMC_BASE);
	func_t func = (func_t)readl(ADDR_CopySDMMCtoMem);
	int chan = (chbase - 0xEB000000) >> 20;
	if (chan != 0 && chan != 2)
		return 0;
	return func(chan, start_block, block_count, (uint32_t*)dest, 0) ? 1 : 0;
}

static __bare_init __naked void jump_sdram(unsigned long offset)
{
	__asm__ __volatile__ (
			"sub lr, lr, %0;"
			"mov pc, lr;" : : "r"(offset)
			);
}

static __bare_init bool load_stage2(void *dest, size_t size)
{
	/* TODO add other ways to boot */
	return s5p_irom_load_mmc(dest, 1, (size+ 511) / 512);
}

void __bare_init barebox_arm_reset_vector(void)
{
	uint32_t r;

	arm_cpu_lowlevel_init();

#ifdef CONFIG_S3C_PLL_INIT
	s5p_init_pll();
#endif

	if (get_pc() < IRAM_CODE_BASE) /* Are we running from iRAM? */
		/* No, we don't. */
		goto boot;

	s5p_init_dram_bank_ddr2(S5P_DMC0_BASE, 0x20E00323, 0, 0);

	if (! load_stage2((void*)(ld_var(_text) - 16),
				ld_var(_barebox_image_size) + 16))
		while (1) { } /* hang */

	jump_sdram(IRAM_CODE_BASE - ld_var(_text));

boot:
	barebox_arm_entry(S3C_SDRAM_BASE, SZ_256M, 0);
}
