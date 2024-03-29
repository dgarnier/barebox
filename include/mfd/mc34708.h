/*
 * Copyright (C) 2009 Marc Kleine-Budde <mkl@pengutronix.de>
 *
 * This file is released under the GPLv2
 *
 * Derived from:
 * - arch-mxc/pmic_external.h --  contains interface of the PMIC protocol driver
 *   Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 */

#ifndef __ASM_ARCH_MC34708_H
#define __ASM_ARCH_MC34708_H

enum mc34708_reg {
	MC34708_REG_INT_STATUS0		= 0x00,
	MC34708_REG_INT_MASK0		= 0x01,
	MC34708_REG_INT_SENSE0		= 0x02,
	MC34708_REG_INT_STATUS1		= 0x03,
	MC34708_REG_INT_MASK1		= 0x04,
	MC34708_REG_INT_SENSE1		= 0x05,
	MC34708_REG_PU_MODE_S		= 0x06,
	MC34708_REG_IDENTIFICATION	= 0x07,
	MC34708_REG_REG_FAULT_S		= 0x08,
	MC34708_REG_ACC0		= 0x09,
	MC34708_REG_ACC1		= 0x0a,
	MC34708_REG_ACC2		= 0x0b,
	MC34708_REG_UNUSED0		= 0x0c,
	MC34708_REG_POWER_CTL0		= 0x0d,
	MC34708_REG_POWER_CTL1		= 0x0e,
	MC34708_REG_POWER_CTL2		= 0x0f,
	MC34708_REG_MEM_A		= 0x10,
	MC34708_REG_MEM_B		= 0x11,
	MC34708_REG_MEM_C		= 0x12,
	MC34708_REG_MEM_D		= 0x13,
	MC34708_REG_RTC_TIME		= 0x14,
	MC34708_REG_RTC_ALARM		= 0x15,
	MC34708_REG_RTC_DAY		= 0x16,
	MC34708_REG_RTC_DAY_ALARM	= 0x17,
	MC34708_REG_1			= 0x18,
	MC34708_REG_2_3			= 0x19,
	MC34708_REG_4			= 0x1a,
	MC34708_REG_5			= 0x1b,
	MC34708_REG_1_2_MODE		= 0x1c,
	MC34708_REG_3_4_5_MODE		= 0x1d,
	MC34708_REG_SETTING_0		= 0x1e,
	MC34708_REG_SWBST_CTRL		= 0x1f,
	MC34708_REG_MODE_0		= 0x20,
	MC34708_REG_GPIOLV0_CTRL	= 0x21,
	MC34708_REG_GPIOLV1_CTRL	= 0x22,
	MC34708_REG_GPIOLV2_CTRL	= 0x23,
	MC34708_REG_GPIOLV3_CTRL	= 0x24,
	MC34708_REG_USB_TIMING		= 0x25,
	MC34708_REG_USB_BUTTON		= 0x26,
	MC34708_REG_USB_CTRL		= 0x27,
	MC34708_REG_USB_DEVTYPE		= 0x28,
	MC34708_REG_UNUSED1		= 0x29,
	MC34708_REG_UNUSED2		= 0x2a,
	MC34708_REG_ADC0		= 0x2b,
	MC34708_REG_ADC1		= 0x2c,
	MC34708_REG_ADC2		= 0x2d,
	MC34708_REG_ADC3		= 0x2e,
	MC34708_REG_ADC4		= 0x2f,
	MC34708_REG_ADC5		= 0x30,
	MC34708_REG_ADC6		= 0x31,
	MC34708_REG_ADC7		= 0x32,
	MC34708_REG_BAT_PROFILE		= 0x33,
	MC34708_REG_CHRG_DEBOUNCE	= 0x34,
	MC34708_REG_CHRG_SOURCE		= 0x35,
	MC34708_REG_CHRG_LED_CTRL	= 0x36,
	MC34708_REG_PWM_CTRL		= 0x37,
	MC34708_REG_UNUSED3		= 0x38,
	MC34708_REG_UNUSED4		= 0x39,
	MC34708_REG_UNUSED5		= 0x3a,
	MC34708_REG_UNUSED6		= 0x3b,
	MC34708_REG_UNUSED7		= 0x3c,
	MC34708_REG_UNUSED8		= 0x3d,
	MC34708_REG_UNUSED9		= 0x3e,
	MC34708_REG_UNUSED10		= 0x3f,
};


enum mc34708_mode {
	MC34708_MODE_I2C,
	MC34708_MODE_SPI,
};

struct mc34708 {
	struct cdev		cdev;
	struct i2c_client	*client;
	struct spi_device	*spi;
	enum mc34708_mode	mode;
	unsigned int		revision;
};

#ifdef CONFIG_MFD_MC34708
struct mc34708 *mc34708_get(void);
#else
static inline struct mc34708 *mc34708_get(void)
{
	return NULL;
}
#endif

extern int mc34708_reg_read(struct mc34708 *mc34708, enum mc34708_reg reg, u32 *val);
extern int mc34708_reg_write(struct mc34708 *mc34708, enum mc34708_reg reg, u32 val);
extern int mc34708_set_bits(struct mc34708 *mc34708, enum mc34708_reg reg, u32 mask, u32 val);

#endif /* __ASM_ARCH_MC34708_H */
