/*
 *  AD semiconductor asp01 grip sensor driver
 *
 *  Copyright (C) 2012 Samsung Electronics Co.Ltd
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/asp01.h>
#include "core/sensors_core.h"

/* For Debugging */
#define asp01_dbgmsg(str, args...) pr_debug("%s: " str, __func__, ##args)
#undef GRIP_DEBUG
#define USE_EEPROM
#define FEATURE_INIT_TOUCH_OFF

#define VENDOR		"ADSEMICON"
#define CHIP_ID		"ASP01"
#define CALIBRATION_FILE_PATH	"/efs/grip_cal_data"
#define CAL_DATA_NUM	24
#define MFM_DATA_NUM	16
#define MFM_REF_NUM	(MFM_DATA_NUM / 2)
#define SLAVE_ADDR	0x48
#define SET_MFM_DONE	(0x1 << 4)
#define EN_INIT_CAL	(0x1 << 3)
#define DIV	1000
#define BYTE_MSK	0xff
#define BYTE_SFT	8
#define RESET_MSK 0xfe
#define CAL_CHECK_VAL	40/* CR percent : 40 = 0.5% / 0.125%(step) */
#define GRIP_CLOSE	1
#define GRIP_FAR	2

enum asp01_I2C_IF {
	REG = 0,
	CMD,
};

/* register address */
enum asp01_REG {
	REG_PROM_EN1 = 0x01,
	REG_PROM_EN2,	/* 0x02 */
	REG_SETMFM,	/* 0x03 */
	REG_CR_CNT,	/* 0x04 */
	REG_CR_CNT_H,	/* 0x05 */
	REG_CR_REF0,	/* 0x06 */
	REG_CR_REF1,	/* 0x07 */
	REG_CR_PER_L,	/* 0x08 */
	REG_CR_PER_H,	/* 0x09 */
	REG_CS_REF0 = 0x0c,
	REG_CS_REF1,	/* 0x0d */
	REG_CS_PERL,	/* 0x0e */
	REG_CS_PERH,	/* 0x0f */
	REG_UNLOCK = 0x11,
	REG_RST_ERR,	/* 0x12 */
	REG_PROX_PER,	/* 0x13 */
	REG_PAR_PER,	/* 0x14 */
	REG_TOUCH_PER,	/* 0x15 */
	REG_HI_CAL_PER = 0x19,
	REG_BSMFM_SET,	/*0x1A */
	REG_ERR_MFM_CYC,	/* 0x1B */
	REG_TOUCH_MFM_CYC,	/* 0x1C */
	REG_HI_CAL_SPD,	/* 0x1D */
	REG_CAL_SPD,	/* 0x1E */
	REG_INIT_REF,	/* 0x1F */
	REG_BFT_MOT,	/* 0x20 */
	REG_TOU_RF_EXT = 0x22,
	REG_SYS_FUNC,	/* 0x23 */
	REG_MFM_INIT_REF0,	/* 0x24 */
	REG_EEP_ST_CON = 0x34,
	REG_EEP_ADDR0,	/* 0x35 */
	REG_EEP_ADDR1,	/* 0x36 */
	REG_EEP_DATA,	/* 0x37 */
	REG_ID_CHECK,	/* 0x38 */
	REG_OFF_TIME,	/* 0x39 */
	REG_SENSE_TIME,	/* 0x3A */
	REG_DUTY_TIME,	/* 0x3B */
	REG_HW_CON1,	/* 0x3C */
	REG_HW_CON2,	/* 0x3D */
	REG_HW_CON3,	/* 0x3E */
	REG_HW_CON4,	/* 0x3F */
	REG_HW_CON5,	/* 0x40 */
	REG_HW_CON6,	/* 0x41 */
	REG_HW_CON7,	/* 0x42 */
	REG_HW_CON8,	/* 0x43 */
	REG_HW_CON9,	/* 0x44 */
	REG_HW_CON10,	/* 0x45 */
	REG_HW_CON11,	/* 0x46 */
};

enum asp01_CMD {
	CMD_CLK_OFF = 0,
	CMD_CLK_ON,
	CMD_RESET,
	CMD_INIT_TOUCH_OFF,
	CMD_INIT_TOUCH_ON,
	CMD_NUM,
};

static const u8 control_reg[CMD_NUM][2] = {
	{REG_SYS_FUNC, 0x16}, /* clock off */
	{REG_SYS_FUNC, 0x10}, /* clock on */
	{REG_SYS_FUNC, 0x01}, /* sw reset */
	{REG_INIT_REF, 0x00}, /* disable initial touch */
	{REG_INIT_REF, EN_INIT_CAL}, /* enable initial touch */
};

static u8 init_reg[SET_REG_NUM][2] = {
	/*{ REG_PROM_EN1, 0x00}, RST */
	{ REG_UNLOCK,		0x5a},
	{ REG_RST_ERR,		0x33},
	{ REG_PROX_PER,		0x34},
	{ REG_PAR_PER,		0x34},
	{ REG_TOUCH_PER,	0x3c},
	{ REG_HI_CAL_PER,	0x08},
	{ REG_BSMFM_SET,	0x31},
	{ REG_ERR_MFM_CYC,	0x33},
	{ REG_TOUCH_MFM_CYC,	0x25},
	{ REG_HI_CAL_SPD,	0x19},
	{ REG_CAL_SPD,		0x03},
	{ REG_INIT_REF,		0x00},
	{ REG_BFT_MOT,		0x40},
	{ REG_TOU_RF_EXT,	0x00},
	{ REG_SYS_FUNC,		0x10},
	{ REG_OFF_TIME,		0x50},
	{ REG_SENSE_TIME,	0x50},
	{ REG_DUTY_TIME,	0x50},
	{ REG_HW_CON1,		0x78},
	{ REG_HW_CON2,		0x27},
	{ REG_HW_CON3,		0x20},
	{ REG_HW_CON4,		0x27},
	{ REG_HW_CON5,		0x83},
	{ REG_HW_CON6,		0x3f},
	{ REG_HW_CON7,		0x48},
	{ REG_HW_CON8,		0x20},
	{ REG_HW_CON10,		0x27},
	{ REG_HW_CON11,		0x00},
};

struct asp01_data {
	struct i2c_client *client;
	struct device *dev;
	struct input_dev *input;
	struct work_struct work; /* for grip sensor */
#ifdef FEATURE_INIT_TOUCH_OFF
	struct delayed_work d_work;
#endif
	struct mutex data_mutex;
	atomic_t enable;
	struct asp01_platform_data *pdata;
	struct wake_lock gr_wake_lock;
	u8 cal_data[CAL_DATA_NUM];
	u8 default_mfm[MFM_DATA_NUM];
	u16 cr_per[4];
	int cr_cosnt;
	int cs_cosnt;
#ifdef FEATURE_INIT_TOUCH_OFF
	bool init_touch_off_needed;
	bool first_close_check;
	bool is_first_close;
#endif
	bool skip_data;
};

static int usb_jig_on;

enum {
	CABLE_TYPE2_JIG_UART_OFF_MUIC = 5,
	CABLE_TYPE2_JIG_UART_OFF_VB_MUIC,
	CABLE_TYPE2_JIG_UART_ON_MUIC,
	CABLE_TYPE2_JIG_UART_ON_VB_MUIC,
};

extern int usb_switch_register_notify(struct notifier_block *nb);
extern int usb_switch_unregister_notify(struct notifier_block *nb);
extern void muic_attached_accessory_inquire(void);

static int jig_check(struct notifier_block *self, unsigned long action,
			void *dev)
{
	pr_info("%s : action = %ld\n", __func__, action);
	switch (action) {
		case CABLE_TYPE2_JIG_UART_OFF_MUIC:
		case CABLE_TYPE2_JIG_UART_OFF_VB_MUIC:
		case CABLE_TYPE2_JIG_UART_ON_MUIC:
		case CABLE_TYPE2_JIG_UART_ON_VB_MUIC:
			pr_info("%s : jig on! disable grip sensor\n", __func__);
			usb_jig_on = true;
		break;
		default:
			pr_info("%s : jig off! enable grip sensor\n", __func__);
			usb_jig_on = false;
	}
	return NOTIFY_OK;
}

static struct notifier_block jig_notify = {
	.notifier_call =	jig_check,
};


static int asp01_reset(struct asp01_data *data)
{
	int err;
	u8 reg;

	/* sw reset */
	reg = i2c_smbus_read_byte_data(data->client,
			control_reg[CMD_RESET][REG]);
	if (reg < 0) {
		pr_err("%s : i2c read fail, err=%d, %d line\n",
			__func__, reg, __LINE__);
		err = -EIO;
		return err;
	}
	err = i2c_smbus_write_byte_data(data->client,
			control_reg[CMD_RESET][REG],
			(RESET_MSK & reg)
			| control_reg[CMD_RESET][CMD]);
	if (err) {
		pr_err("%s: i2c write fail, err=%d, %d line\n",
			__func__, err, __LINE__);
		goto done;
	}
	err = i2c_smbus_write_byte_data(data->client,
			control_reg[CMD_RESET][REG],
			RESET_MSK & reg);
	if (err) {
		pr_err("%s: i2c write fail, err=%d, %d line\n",
			__func__, err, __LINE__);
		goto done;
	}
#ifdef USE_EEPROM
	msleep(250);
#else
	msleep(40);
#endif
done:
	return err;
}

static int asp01_init_touch_onoff(struct asp01_data *data, bool onoff)
{
	int err;

	if (onoff) {
		err = i2c_smbus_write_byte_data(data->client,
			control_reg[CMD_INIT_TOUCH_ON][REG],
			control_reg[CMD_INIT_TOUCH_ON][CMD]);
		if (err) {
				pr_err("%s: i2c write fail, err=%d, %d line\n",
					__func__, err, __LINE__);
				goto done;
		}
	} else {
		err = i2c_smbus_write_byte_data(data->client,
			control_reg[CMD_INIT_TOUCH_OFF][REG],
			control_reg[CMD_INIT_TOUCH_OFF][CMD]);
		if (err) {
				pr_err("%s: i2c write fail, err=%d, %d line\n",
					__func__, err, __LINE__);
				goto done;
		}
	}
done:
	return err;
}

static int asp01_init_code_set(struct asp01_data *data, int count)
{
	int err, i, j = 0;
#ifdef GRIP_DEBUG
	u8 reg;
#endif

#ifdef USE_EEPROM
	msleep(400);
#else
	for (j = 0; j < 20; j++) {/* need to write six time */
#endif
		/* clock on */
		err = i2c_smbus_write_byte_data(data->client,
				control_reg[CMD_CLK_ON][REG],
				control_reg[CMD_CLK_ON][CMD]);
		if (err) {
			pr_err("%s: i2c write fail, err=%d, %d line\n",
				__func__, err, __LINE__);
			goto done;
		}

		/* write Initializing code */
		for (i = 0; i < SET_REG_NUM; i++) {
			err = i2c_smbus_write_byte_data(data->client,
				init_reg[i][REG], init_reg[i][CMD]);
			if (err) {
				pr_err("%s: i2c write fail, err=%d, %d line\n",
					__func__, err, __LINE__);
				goto done;
			}
			pr_debug("%s: reg: %x, data: %x\n", __func__,
				init_reg[i][REG], init_reg[i][CMD]);
		}

		if (!count) {
			/* disable initial touch mode */
			asp01_init_touch_onoff(data, false);
			if (!j)
				pr_info("%s: cal disable\n", __func__);
		} else {
			/* wtire initial touch ref setting */
			for (i = 0; i < MFM_DATA_NUM; i++) {
				pr_debug("%s: reg(0x%x) = 0x%x\n", __func__,
					REG_MFM_INIT_REF0 + i,
					data->cal_data[i]);
				err = i2c_smbus_write_byte_data(data->client,
					REG_MFM_INIT_REF0 + i,
					data->cal_data[i]);
				if (err)
					pr_err("%s: i2c write fail, err=%d, %d line\n",
						__func__, err, __LINE__);
			};
			/* enable initial touch mode */
			asp01_init_touch_onoff(data, true);
			if (!j)
				pr_info("%s: cal enable\n", __func__);
		}
		usleep_range(5000, 5100);
#ifndef USE_EEPROM
	}
#endif
#ifdef GRIP_DEBUG
	/* verifying register set */
	for (i = 0; i < SET_REG_NUM; i++) {
		reg = i2c_smbus_read_byte_data(data->client,
					 init_reg[i][REG);
		if (reg < 0) {
			pr_err("%s : i2c read fail, err=%d, %d line\n",
				__func__, reg, __LINE__);
			err = -EIO;
			return err;
		}
		pr_info("%s: reg: %x, data: %x\n", __func__,
			init_reg[i][REG], reg);
	}
#endif
done:
	return err;
}

static void asp01_open_calibration(struct asp01_data *data)
{
	struct file *cal_filp = NULL;
	int i;
	int err = 0;
	int count = CAL_DATA_NUM;
	mm_segment_t old_fs;
#ifdef GRIP_DEBUG
	u8 reg;
#endif

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY,
			S_IRUGO | S_IWUSR | S_IWGRP);
	if (IS_ERR(cal_filp)) {
		err = PTR_ERR(cal_filp);
		if (err != -ENOENT)
			pr_err("%s: Can't open calibration file\n", __func__);
		else {
			pr_info("%s: need calibration\n", __func__);
			/* calibration status init */
			for (i = 0; i < CAL_DATA_NUM; i++)
				data->cal_data[i] = 0;
		}
		set_fs(old_fs);
		if (err == -ENOENT)
			goto set_init_code;
		else
			return;
	}

	err = cal_filp->f_op->read(cal_filp,
		(char *)data->cal_data,
		CAL_DATA_NUM * sizeof(u8), &cal_filp->f_pos);
	if (err != CAL_DATA_NUM * sizeof(u8))
		pr_err("%s: Can't read the cal data from file\n", __func__);

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

set_init_code:
	for (i = 0; i < CAL_DATA_NUM; i++) {
		if (!data->cal_data[i])
			count -= 1;
	}

	mutex_lock(&data->data_mutex);

	err = asp01_reset(data);
	if (err) {
		pr_err("%s: asp01_reset, err=%d\n",
			__func__, err);
		mutex_unlock(&data->data_mutex);
		return;
	}
#ifdef USE_EEPROM
	if (!count) {
#endif
		err = asp01_init_code_set(data, count);
		if (err < 0) {
			pr_err("%s: asp01_init_code_set, err=%d\n",
				__func__, err);
			mutex_unlock(&data->data_mutex);
			return;
		}
#ifdef USE_EEPROM
	} else
		pr_info("%s: now use eeprom\n", __func__);
#endif
#ifdef GRIP_DEBUG
	/* verifiying MFM value*/
	for (i = 0; i < MFM_DATA_NUM; i++) {
		reg = i2c_smbus_read_byte_data(data->client,
				 REG_MFM_INIT_REF0 + i);
		if (reg < 0) {
			pr_err("%s : i2c read fail, err=%d, %d line\n",
				__func__, reg, __LINE__);
			err = -EIO;
			mutex_unlock(&data->data_mutex);
			return;
		}
		pr_info("%s: verify reg(0x%x) = 0x%x\n", __func__,
			REG_MFM_INIT_REF0 + i, reg);
	}
#endif
	mutex_unlock(&data->data_mutex);

	return;
}

static int asp01_do_calibrate(struct device *dev, bool do_calib)
{
	struct asp01_data *data = dev_get_drvdata(dev);
	struct file *cal_filp = NULL;
	int err = 0;
	int i;
	int count = 100;
	u8 reg;
	u32 cr_ref[4] = {0,};
	u32 cs_ref[4] = {0,};
	mm_segment_t old_fs;
	u16 cr_per[4] = {0,};
	u32 cr_per_sum = 0;
	u16 cr_per_avg = 0;
	bool calibration_success = true;

	pr_info("%s: do_calib=%d\n", __func__, do_calib);

	if (do_calib) {
		for (i = 0; i < 4; i++) {
			/* 1. set MFM unmber (MFM0~MFM3)*/
			err = i2c_smbus_write_byte_data(data->client,
					REG_SETMFM, i);
			if (err) {
				pr_err("%s: i2c write fail, err=%d, %d line\n",
					__func__, err, __LINE__);
				return err;
			}
			/* 2. wait until data ready */
			do {
				reg = i2c_smbus_read_byte_data(data->client,
					REG_SETMFM);
				if (reg < 0) {
					pr_err("%s : i2c read fail, err=%d, %d line\n",
						__func__, reg, __LINE__);
					err = -EIO;
					return err;
				}
				msleep(20);
				if (reg & SET_MFM_DONE) {
					pr_info("%s: count is %d\n", __func__,
						100 - count);
					count = 100;
					break;
				}
				if (!count)
					pr_err("%s: full count state\n",
						__func__);
			} while (count--);

			/* 3. read each CR CS ref value */
			err = i2c_smbus_write_byte_data(data->client,
					control_reg[CMD_CLK_OFF][REG],
					control_reg[CMD_CLK_OFF][CMD]);
			if (err) {
				pr_err("%s: i2c write fail, err=%d, %d line\n",
					__func__, err, __LINE__);
				return err;
			}
			err = i2c_smbus_read_i2c_block_data(data->client,
				REG_CR_REF0, sizeof(u8)*2,
				&data->cal_data[i * 4]);
			if (err != sizeof(u8)*2) {
				pr_err("%s : i2c read fail, err=%d, %d line\n",
					__func__, err, __LINE__);
				err = -EIO;
				return err;
			}
			err = i2c_smbus_read_i2c_block_data(data->client,
				REG_CS_REF0, sizeof(u8)*2,
				&data->cal_data[(i * 4) + 2]);
			if (err != sizeof(u8)*2) {
				pr_err("%s : i2c read fail, err=%d, %d line\n",
					__func__, err, __LINE__);
				err = -EIO;
				return err;
			}
			cr_ref[i] =
				(data->cal_data[(i * 4) + 1] << BYTE_SFT
					| data->cal_data[(i * 4)]);
			cs_ref[i] =
				(data->cal_data[(i * 4) + 3] << BYTE_SFT
					| data->cal_data[(i * 4) + 2]);

			/* 4. multiply cr, cs constant*/
			/* cs ref x cs cont */
			cs_ref[i] =
				(cs_ref[i] * data->cs_cosnt) / DIV;
			/* cr ref x cr cont */
			cr_ref[i] =
				(cr_ref[i] * data->cr_cosnt) / DIV;

			/* 5. separate high low reg of cr cs ref */
			data->cal_data[(i * 4)] =
				(u8)(BYTE_MSK & cr_ref[i]);

			data->cal_data[(i * 4) + 1] =
				(u8)(BYTE_MSK & (cr_ref[i] >> BYTE_SFT));

			data->cal_data[(i * 4) + 2] =
				(u8)(BYTE_MSK & cs_ref[i]);

			data->cal_data[(i * 4) + 3] =
				(u8)(BYTE_MSK & (cs_ref[i] >> BYTE_SFT));

			/* 6. read pr percent data */
			err = i2c_smbus_read_i2c_block_data(data->client,
			REG_CR_PER_L, sizeof(u8)*2,
			&data->cal_data[MFM_DATA_NUM + i*2]);
			if (err != sizeof(u8)*2) {
				pr_err("%s : i2c read fail, err=%d, %d line\n",
					__func__, err, __LINE__);
				err = -EIO;
				mutex_unlock(&data->data_mutex);
				return err;
			}
			cr_per[i] =
				data->cal_data[MFM_DATA_NUM + 1 + i*2] << 8|\
				data->cal_data[MFM_DATA_NUM + i*2];
			cr_per_sum += cr_per[i];
			pr_info("%s, cr percent = %d\n", __func__, cr_per[i]);

			err = i2c_smbus_write_byte_data(data->client,
				control_reg[CMD_CLK_ON][REG],
				control_reg[CMD_CLK_ON][CMD]);
			if (err) {
				pr_err("%s: i2c write fail, err=%d, %d line\n",
					__func__, err, __LINE__);
				mutex_unlock(&data->data_mutex);
				return err;
			}
		}

		/* Check if cr percent is under 0.5%. */
		cr_per_avg = (u16)(cr_per_sum / 4);
		if (cr_per_avg > CAL_CHECK_VAL) {
			memset(data->cal_data, 0, ARRAY_SIZE(data->cal_data));
			pr_err("%s, calibeation fail. cr percent avg = %d\n",
				__func__, cr_per_avg);
			calibration_success = false;
		} else
			pr_info("%s, cr percent avg = %d\n", __func__,
				cr_per_avg);

		/* write MFM ref value */
		for (i = 0; i < MFM_DATA_NUM; i++) {
			err = i2c_smbus_write_byte_data(data->client,
					REG_MFM_INIT_REF0 + i,
					data->cal_data[i]);
			if (err) {
				pr_err("%s: i2c write fail, err=%d, %d line\n",
					__func__, err, __LINE__);
				return err;
			}
			pr_debug("%s: wr reg(0x%x) = 0x%x\n", __func__,
				REG_MFM_INIT_REF0 + i,
				data->cal_data[i]);
		}
		if (calibration_success) {
			asp01_init_touch_onoff(data, true);
			data->init_touch_off_needed = true;
			pr_info("%s, init_touch_off_needed is set\n", __func__);
		} else
			asp01_init_touch_onoff(data, false);
	} else {
		/* reset MFM ref value */
		memset(data->cal_data, 0, ARRAY_SIZE(data->cal_data));
		for (i = 0; i < MFM_DATA_NUM; i++) {
			err = i2c_smbus_write_byte_data(data->client,
					REG_MFM_INIT_REF0 + i, 0);
			if (err) {
				pr_err("%s: i2c write fail, err=%d, %d line\n",
					__func__, err, __LINE__);
				return err;
			}
			pr_debug("%s: reset reg(0x%x) = 0x%x\n", __func__,
				REG_MFM_INIT_REF0 + i, data->cal_data[i]);
		}
		asp01_init_touch_onoff(data, false);
	}

#ifdef USE_EEPROM
	/* write into eeprom */
	err = i2c_smbus_write_byte_data(data->client,
			REG_EEP_ST_CON, 0x02);
	if (err) {
		pr_err("%s: i2c write fail, err=%d, %d line\n",
			__func__, err, __LINE__);
		return err;
	}
	count = 20;
	do {
		reg = i2c_smbus_read_byte_data(data->client, REG_EEP_ST_CON);
		if (reg < 0) {
			pr_err("%s : i2c read fail, err=%d, %d line\n",
				__func__, reg, __LINE__);
			err = -EIO;
			return err;
		}
		pr_debug("%s: check eeprom reg(0x%x) = 0x%x count=%d\n",
			__func__, REG_EEP_ST_CON, reg, count);
		msleep(50);
	} while ((reg & 0x1) && count--);
	if (!count)
		pr_err("%s: use all count! need to check eeprom\n",
			__func__);
#endif
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY | O_SYNC,
			S_IRUGO | S_IWUSR | S_IWGRP);
	if (IS_ERR(cal_filp)) {
		pr_err("%s: Can't open calibration file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		return err;
	}

	err = cal_filp->f_op->write(cal_filp,
		(char *)&data->cal_data,
		CAL_DATA_NUM * sizeof(u8), &cal_filp->f_pos);
	if (err != CAL_DATA_NUM * sizeof(u8)) {
		pr_err("%s: Can't write the cal data to file\n", __func__);
		err = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return err;
}

static int asp01_grip_enable(struct asp01_data *data)
{
	int err;

	mutex_lock(&data->data_mutex);
	err = i2c_smbus_write_byte_data(data->client,
			control_reg[CMD_CLK_ON][REG],
			control_reg[CMD_CLK_ON][CMD]);
	if (err)
		pr_err("%s: i2c write fail, err=%d, %d line\n",
			__func__, err, __LINE__);
	mutex_unlock(&data->data_mutex);

#ifdef GRIP_DEBUG
	pr_debug("%s: reg: %x, data: %x\n", __func__,
		control_reg[CMD_CLK_ON][REG],
		control_reg[CMD_CLK_ON][CMD]);
#endif

	return err;
}

static int asp01_grip_disable(struct asp01_data *data)
{
	int err, grip;
	int count = 10;
	mutex_lock(&data->data_mutex);
	err = i2c_smbus_write_byte_data(data->client,
			REG_SYS_FUNC, 0x14);
	if (err)
		pr_err("%s: i2c write fail, err=%d, %d line\n",
			__func__, err, __LINE__);
	do {
		usleep_range(5000, 51000);
		grip = gpio_get_value(data->pdata->t_out);
		pr_info("%s: grip=%d, count=%d\n",
			__func__, grip, count);
	} while (!grip && count--);

	err = i2c_smbus_write_byte_data(data->client,
			control_reg[CMD_CLK_OFF][REG],
			control_reg[CMD_CLK_OFF][CMD]);
	if (err)
		pr_err("%s: i2c write fail, err=%d, %d line\n",
			__func__, err, __LINE__);
	mutex_unlock(&data->data_mutex);

#ifdef GRIP_DEBUG
	pr_debug("%s: reg: %x, data: %x\n", __func__,
		control_reg[CMD_CLK_OFF][REG],
		control_reg[CMD_CLK_OFF][CMD]);
#endif

	return err;
}

static int asp01_suspend(struct device *dev)
{
	int err = 0;
	struct asp01_data *data = dev_get_drvdata(dev);

	pr_info("%s, enable = %d\n", __func__, atomic_read(&data->enable));
	if (atomic_read(&data->enable))
		enable_irq_wake(data->pdata->irq);
	else {
		mutex_lock(&data->data_mutex);
		err = i2c_smbus_write_byte_data(data->client,
				control_reg[CMD_CLK_OFF][REG],
				control_reg[CMD_CLK_OFF][CMD]);
		if (err)
			pr_err("%s: i2c write fail, err=%d, %d line\n",
				__func__, err, __LINE__);
		mutex_unlock(&data->data_mutex);
	}
	return err;
}

static int asp01_resume(struct device *dev)
{
	int err = 0;
	struct asp01_data *data = dev_get_drvdata(dev);

	pr_info("%s, enable = %d\n", __func__, atomic_read(&data->enable));
	if (atomic_read(&data->enable))
		disable_irq_wake(data->pdata->irq);

	return err;
}

static const struct dev_pm_ops asp01_pm_ops = {
	.suspend = asp01_suspend,
	.resume = asp01_resume,
};

static ssize_t asp01_enable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct asp01_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", atomic_read(&data->enable));
}

static ssize_t asp01_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct asp01_data *data = input_get_drvdata(input);
	unsigned long enable = 0;
	int err;
	static bool isFirst = true;
	int touch, jig_on;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable && !atomic_read(&data->enable)) {
		if (isFirst) {
			asp01_open_calibration(data);
			isFirst = false;
#ifdef FEATURE_INIT_TOUCH_OFF
			data->init_touch_off_needed = true;
			pr_info("%s, init_touch_off_needed is set\n", __func__);
#endif
		}
#ifdef FEATURE_INIT_TOUCH_OFF
		if (data->init_touch_off_needed) {
			data->first_close_check = true;
			pr_info("%s, first_close_check is set\n", __func__);
		}
#endif
		err = asp01_grip_enable(data);
		if (err < 0)
			goto done;
		/* turn on time is needed. */
		msleep(250);

		jig_on = usb_jig_on;
		/* gpio jig check 
		jig_on = gpio_get_value(data->pdata->adj_det);
		*/
		touch = gpio_get_value(data->pdata->t_out);
		if (!jig_on && !data->skip_data) {
			pr_info("%s: touch INT : %d!!\n", __func__, touch);
			input_report_rel(data->input, REL_MISC,
				(touch ? GRIP_FAR : GRIP_CLOSE));
			input_sync(data->input);
		} else
			pr_info("%s: skipped input report jig_on: %d, skip cmd: %d\n",
			__func__, jig_on, data->skip_data);

		enable_irq(data->pdata->irq);
#ifdef FEATURE_INIT_TOUCH_OFF
		if (data->init_touch_off_needed)
			schedule_delayed_work(&data->d_work,
				msecs_to_jiffies(700));
#endif
	} else if (!enable && atomic_read(&data->enable)) {
#ifdef FEATURE_INIT_TOUCH_OFF
		cancel_delayed_work_sync(&data->d_work);
#endif
		disable_irq(data->pdata->irq);
		err = asp01_grip_disable(data);
		if (err < 0)
			goto done;
	} else
		pr_err("%s: Invalid enable status\n", __func__);

	atomic_set(&data->enable, enable);
	pr_info("%s, enable = %ld\n", __func__, enable);
done:
	return count;
}
static DEVICE_ATTR(enable,
		   S_IRUGO | S_IWUSR | S_IWGRP,
		   asp01_enable_show, asp01_enable_store);

static struct attribute *asp01_attributes[] = {
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group asp01_attribute_group = {
	.attrs = asp01_attributes
};

static void asp01_work_func(struct work_struct *work)
{
	int touch, jig_on;
	struct asp01_data *data
		= container_of(work, struct asp01_data, work);

	disable_irq(data->pdata->irq);

	if (unlikely(!data->pdata)) {
		pr_err("%s: can't get pdata\n", __func__);
		goto done;
	}
	jig_on = usb_jig_on;
	/* gpio_get_value returns 256, when the pin is high */
	/* gpio jig check 
	jig_on = gpio_get_value(data->pdata->adj_det);
	*/
	touch = gpio_get_value(data->pdata->t_out);
#ifdef FEATURE_INIT_TOUCH_OFF
	if (data->init_touch_off_needed) {
		if (!touch && data->first_close_check) {
			data->is_first_close = true;
			data->first_close_check = false;
			pr_info("%s, first close!!\n", __func__);
		}

		if (touch && data->is_first_close) {
			data->is_first_close = false;
			data->init_touch_off_needed = false;
			asp01_init_touch_onoff(data, false);
			pr_info("%s: init touch off\n", __func__);
		} else
			pr_info("%s: skipped init touch off\n", __func__);
	}
#endif
	/* check adj_det pin & skip data cmd for factory test */
	if (!jig_on && !data->skip_data) {
		pr_info("%s: touch INT : %d!!\n", __func__, touch);
		input_report_rel(data->input, REL_MISC,
			(touch ? GRIP_FAR : GRIP_CLOSE));
		input_sync(data->input);
	} else
		pr_info("%s: skipped input report jig_on: %d, skip cmd: %d\n",
			__func__, jig_on, data->skip_data);
done:
	enable_irq(data->pdata->irq);
}

static irqreturn_t asp01_irq_thread(int irq, void *dev)
{
	struct asp01_data *data = (struct asp01_data *)dev;

	pr_info("%s: irq created\n", __func__);
	wake_lock_timeout(&data->gr_wake_lock, 3 * HZ);
	schedule_work(&data->work);
	return IRQ_HANDLED;
}

#ifdef FEATURE_INIT_TOUCH_OFF
static void asp01_delay_work_func(struct work_struct *work)
{
	int err = 0;
	struct asp01_data *data
		= container_of(work, struct asp01_data, d_work.work);

	/* disable initial touch mode */
	if (data->init_touch_off_needed && !data->is_first_close) {
		data->init_touch_off_needed = false;
		data->first_close_check = false;
		err = asp01_init_touch_onoff(data, false);
		if (err)
			pr_err("%s: failed to write, err=%d\n",
				__func__, err);
		pr_info("%s: disable init touch mode\n", __func__);
	} else
		pr_info("%s: skip disable init touch mode\n", __func__);
}
#endif

static ssize_t asp01_calibration_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct asp01_data *data = dev_get_drvdata(dev);
	int err = -1;
	int i;
	int count = MFM_REF_NUM;
	u16 cr_per[4] = {0,};
	u16 mfm_ref[MFM_REF_NUM];

	asp01_open_calibration(data);

	for (i = 0; i < MFM_REF_NUM; i++) {
		mfm_ref[i] =
			(data->cal_data[2 * i + 1] << BYTE_SFT) |
				data->cal_data[2 * i];
		if (!mfm_ref[i])
			count -= 1;
	}

	/* If MFM is not calibrated. */
	if (count) {
		err = 1;
		for (i = 0; i < 4; i++)
			cr_per[i] =
				data->cal_data[MFM_DATA_NUM + 1 + i*2] << 8 |\
				data->cal_data[MFM_DATA_NUM + i*2];
	}

	return sprintf(buf, "%d, %d, %d, %d, %d\n",
		err, cr_per[0], cr_per[1], cr_per[2], cr_per[3]);
}

static ssize_t asp01_calibration_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int err;
	bool do_calib;
	struct asp01_data *data = dev_get_drvdata(dev);

	if (sysfs_streq(buf, "1"))
		do_calib = true;
	else if (sysfs_streq(buf, "0"))
		do_calib = false;
	else {
		pr_info("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	mutex_lock(&data->data_mutex);

	asp01_reset(data);
	/* write init code with disable init touch (0)*/
	asp01_init_code_set(data, 0);

	err = asp01_do_calibrate(dev, do_calib);

	mutex_unlock(&data->data_mutex);

	if (err < 0) {
		pr_err("%s: asp01_do_calibrate() failed\n", __func__);
		return err;
	}

	return count;
}
static DEVICE_ATTR(calibration, S_IRUGO | S_IWUSR | S_IWGRP,
	asp01_calibration_show, asp01_calibration_store);

static ssize_t asp01_grip_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VENDOR);
}
static DEVICE_ATTR(vendor, S_IRUGO,
	asp01_grip_vendor_show, NULL);

static ssize_t asp01_grip_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", CHIP_ID);
}
static DEVICE_ATTR(name, S_IRUGO,
	asp01_grip_name_show, NULL);

static ssize_t asp01_grip_raw_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 reg;
	u8 cs_per[2] = {0,};
	u8 cr_per[2] = {0,};
	u16 mlt_cs_per[4] = {0,};
	u16 mlt_cr_per[4] = {0,};
	int count = 100;
	int err, i;
	struct asp01_data *data = dev_get_drvdata(dev);

	if (!atomic_read(&data->enable)) {
		asp01_grip_enable(data);
		usleep_range(10000, 11000);
	}

	mutex_lock(&data->data_mutex);
	for (i = 0; i < 4; i++) {
		/* 1. set MFM unmber (MFM0~MFM3)*/
		err = i2c_smbus_write_byte_data(data->client,
				REG_SETMFM, i);
		if (err) {
			pr_err("%s: i2c write fail, err=%d, %d line\n",
				__func__, err, __LINE__);
			mutex_unlock(&data->data_mutex);
			return err;
		}
		/* 2. wait until data ready */
		do {
			reg = i2c_smbus_read_byte_data(data->client,
					 REG_SETMFM);
			if (reg < 0) {
				pr_err("%s : i2c read fail, err=%d, %d line\n",
					__func__, reg, __LINE__);
				err = -EIO;
				mutex_unlock(&data->data_mutex);
				return err;
			}
			usleep_range(10000, 11000);
			if (reg & SET_MFM_DONE) {
				pr_debug("%s: count is %d\n", __func__,
					100 - count);
				count = 100;
				break;
			}
			if (!count)
				pr_err("%s: full count state\n",
					__func__);
		} while (count--);
		/* 3. read CS percent value */
		err = i2c_smbus_write_byte_data(data->client,
				control_reg[CMD_CLK_OFF][REG],
				control_reg[CMD_CLK_OFF][CMD]);
		if (err) {
			pr_err("%s: i2c write fail, err=%d, %d line\n",
				__func__, err, __LINE__);
			mutex_unlock(&data->data_mutex);
			return err;
		}
		err = i2c_smbus_read_i2c_block_data(data->client,
			REG_CS_PERL, sizeof(u8)*2,
			cs_per);
		if (err != sizeof(reg)*2) {
			pr_err("%s : i2c read fail, err=%d, %d line\n",
				__func__, err, __LINE__);
			err = -EIO;
			mutex_unlock(&data->data_mutex);
			return err;
		}

		mlt_cs_per[i] = (cs_per[1] << 8) | cs_per[0];
		/* 4. read CR percent value */
		err = i2c_smbus_read_i2c_block_data(data->client,
			REG_CR_PER_L, sizeof(u8)*2,
			cr_per);
		if (err != sizeof(reg)*2) {
			pr_err("%s :i2c read fail. err=%d, %d line\n",
				__func__, err, __LINE__);
			err = -EIO;
			mutex_unlock(&data->data_mutex);
			return err;
		}
		err = i2c_smbus_write_byte_data(data->client,
				control_reg[CMD_CLK_ON][REG],
				control_reg[CMD_CLK_ON][CMD]);
		if (err) {
			pr_err("%s: i2c write fail, err=%d, %d line\n",
				__func__, err, __LINE__);
			mutex_unlock(&data->data_mutex);
			return err;
		}
		data->cr_per[i] =
			mlt_cr_per[i] = (cr_per[1] << 8) | cr_per[0];
	}
	mutex_unlock(&data->data_mutex);

#ifdef GRIP_DEBUG
		for (i = 0; i < SET_REG_NUM; i++) {
			reg = i2c_smbus_read_byte_data(data->client,
					 init_reg[i][REG]);
			if (reg < 0) {
				pr_err("%s : i2c read fail, err=%d, %d line\n",
					__func__, reg, __LINE__);
				return reg;
			}
			pr_info("%s: reg(0x%x) = 0x%x, default 0x%x\n",
				__func__,
				init_reg[i][REG], reg, init_reg[i][CMD]);
		}
		/* verifiying MFM value*/
		for (i = 0; i < MFM_DATA_NUM; i++) {
			reg = i2c_smbus_read_byte_data(data->client,
					 REG_MFM_INIT_REF0 + i);
			if (reg < 0) {
				pr_err("%s : i2c read fail, err=%d, %d line\n",
					__func__, reg, __LINE__);
				return reg;
			}
			pr_info("%s: verify reg(0x%x) = 0x%x, data: 0x%x\n",
				__func__, REG_MFM_INIT_REF0 + i,
				reg, data->cal_data[i]);
		}
#endif

	if (!atomic_read(&data->enable))
		asp01_grip_disable(data);

	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d\n",
		mlt_cs_per[0], mlt_cs_per[1], mlt_cs_per[2], mlt_cs_per[3],
		mlt_cr_per[0], mlt_cr_per[1], mlt_cr_per[2], mlt_cr_per[3]);
}
static DEVICE_ATTR(raw_data, S_IRUGO,
	asp01_grip_raw_data_show, NULL);

static ssize_t asp01_grip_threshold_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", init_reg[SET_PROX_PER][CMD]);
}
static DEVICE_ATTR(threshold, S_IRUGO,
	asp01_grip_threshold_show, NULL);

static ssize_t asp01_grip_check_crcs_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 reg;
	u8 crcs[48] = {0,};
	int count = 100;
	int err, i, j;
	struct asp01_data *data = dev_get_drvdata(dev);

	for (i = 0; i < 4; i++) {
		/* 1. set MFM unmber (MFM0~MFM3)*/
		err = i2c_smbus_write_byte_data(data->client,
				REG_SETMFM, i);
		if (err) {
			pr_err("%s: i2c write fail, err=%d, %d line\n",
				__func__, err, __LINE__);
			return err;
		}
		/* 2. wait until data ready */
		do {
			reg = i2c_smbus_read_byte_data(data->client,
				REG_SETMFM);
			if (reg < 0) {
				pr_err("%s : i2c read fail, err=%d, %d line\n",
					__func__, reg, __LINE__);
				err = -EIO;
				return err;
			}
			usleep_range(10000, 11000);
			if (reg & SET_MFM_DONE) {
				pr_debug("%s: count is %d\n", __func__,
					100 - count);
				count = 100;
				break;
			}
			if (!count)
				pr_err("%s: full count state\n",
					__func__);
		} while (count--);
		/* 3. read CR CS registers */
		for (j = REG_CR_CNT; j < (REG_CS_PERH + 1); j++) {
			crcs[(12 * i) + j - REG_CR_CNT] =
				i2c_smbus_read_byte_data(data->client, j);
			if (crcs[(12 * i) + j - REG_CR_CNT] < 0) {
				pr_err("%s : i2c read fail, err=%d, %d line\n",
					__func__,
					crcs[(12 * i) + j - REG_CR_CNT],
					__LINE__);
				err = -EIO;
				return err;
			}
			pr_info("%s: SEL %d, reg (0x%x) = 0x%x",
				__func__, i, j,
				crcs[(12 * i) + j - REG_CR_CNT]);
		}
	}

	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n"
		"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n"
		"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n"
		"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		crcs[0], crcs[1], crcs[2], crcs[3], crcs[4], crcs[5],
		crcs[6], crcs[7], crcs[8], crcs[9], crcs[10], crcs[11],
		crcs[12], crcs[13], crcs[14], crcs[15], crcs[16], crcs[17],
		crcs[18], crcs[19], crcs[20], crcs[21], crcs[22], crcs[23],
		crcs[24], crcs[25], crcs[26], crcs[27], crcs[28], crcs[29],
		crcs[30], crcs[31], crcs[32], crcs[33], crcs[34], crcs[35],
		crcs[36], crcs[37], crcs[38], crcs[39], crcs[40], crcs[41],
		crcs[42], crcs[43], crcs[44], crcs[45], crcs[46], crcs[47]);
}
static DEVICE_ATTR(check_crcs, S_IRUGO,
	asp01_grip_check_crcs_show, NULL);

static ssize_t asp01_grip_onoff_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct asp01_data *data = dev_get_drvdata(dev);

	pr_info("%s, onoff = %d\n", __func__, !data->skip_data);

	return sprintf(buf, "%d\n", !data->skip_data);
}

static ssize_t asp01_grip_onoff_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct asp01_data *data = dev_get_drvdata(dev);

	if (sysfs_streq(buf, "1"))
		data->skip_data = false;
	else if (sysfs_streq(buf, "0"))
		data->skip_data = true;
	else {
		pr_info("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
	pr_info("%s, onoff = %d\n", __func__, !data->skip_data);

	return count;
}

static DEVICE_ATTR(onoff, S_IRUGO | S_IWUSR | S_IWGRP,
		   asp01_grip_onoff_show, asp01_grip_onoff_store);

static ssize_t asp01_grip_reset_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct asp01_data *data = dev_get_drvdata(dev);
	int err = 0;

	if (sysfs_streq(buf, "1")) {
		pr_info("%s is called\n", __func__);
		if (atomic_read(&data->enable)) {
			disable_irq(data->pdata->irq);
			err = asp01_grip_disable(data);
			if (err < 0)
				pr_err("%s: diable fail(%d)\n", __func__, err);
			msleep(250);
		}

		mutex_lock(&data->data_mutex);
		err = asp01_reset(data);
		if (err) {
			pr_err("%s: asp01_reset, err = %d\n",
				__func__, err);
		}

		err = asp01_init_code_set(data, 0);
		if (err < 0) {
			pr_err("%s: asp01_init_code_set, err = %d\n",
				__func__, err);
		}
		mutex_unlock(&data->data_mutex);

		if (atomic_read(&data->enable)) {
			err = asp01_grip_enable(data);
			if (err < 0)
				pr_err("%s: enable fail(%d)\n", __func__, err);
			msleep(250);
			enable_irq(data->pdata->irq);
		}
	} else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
	}

	return count;
}
static DEVICE_ATTR(reset,    S_IWUSR | S_IWGRP,
		   NULL, asp01_grip_reset_store);

static ssize_t asp01_grip_cscr_const_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct asp01_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "cs const = %d, cr_const= %d\n",
		data->cs_cosnt, data->cr_cosnt);
}

static ssize_t asp01_grip_cscr_const_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int cs_divnd, cs_divsr, cr_divnd, cr_divsr;
	struct asp01_data *data = dev_get_drvdata(dev);

	sscanf(buf, "%d %d %d %d",
		&cs_divnd, &cs_divsr, &cr_divnd, &cr_divsr);

	if (cs_divsr && cs_divsr) {
		data->cs_cosnt = (cs_divnd * DIV) / cs_divsr;
		data->cr_cosnt = (cr_divnd * DIV) / cr_divsr;
	}
	return count;
}
static DEVICE_ATTR(cscr_const, S_IRUGO | S_IWUSR | S_IWGRP,
		   asp01_grip_cscr_const_show, asp01_grip_cscr_const_store);

static int asp01_apply_hw_dep_set(struct asp01_data *data)
{
	u8 reg, reg_prom_en1, i;
	u8 reg_hwcon10, reg_hwcon6;
	int err = 0;

	if (data->pdata == NULL) {
		err = -EINVAL;
		pr_err("%s: data->pdata is NULL\n", __func__);
		goto done;
	}

	if (!data->pdata->cr_divsr)
		data->cr_cosnt = DIV;
	else
		data->cr_cosnt =
			((data->pdata->cr_divnd * DIV) / data->pdata->cr_divsr);

	if (!data->pdata->cs_divsr)
		data->cs_cosnt = 0;
	else
		data->cs_cosnt =
			((data->pdata->cs_divnd * DIV) / data->pdata->cs_divsr);

	for (i = 0; i < MFM_DATA_NUM; i++) {
		reg = i2c_smbus_read_byte_data(data->client,
			REG_MFM_INIT_REF0 + i);
		if (reg < 0) {
			pr_err("%s : i2c read fail, err=%d, %d line\n",
				__func__, reg, __LINE__);
			err = -EIO;
			goto done;
		}
		data->default_mfm[i] = reg;

		pr_info("%s: default_mfm[%d] = %x\n",
			__func__, i, data->default_mfm[i]);
	}

	reg_prom_en1 = i2c_smbus_read_byte_data(data->client,
		REG_PROM_EN1);
	if (reg_prom_en1 < 0) {
		pr_err("%s : i2c read fail, err=%d, %d line\n",
			__func__, reg_prom_en1, __LINE__);
		err = -EIO;
		goto done;
	}

	for (i = 0; i < SET_REG_NUM; i++) {
		if (i < SET_HW_CON1)
			init_reg[i][CMD] = data->pdata->init_code[i];
		else {
			/* keep reset value if PROM_EN1 is 0xaa */
			if (reg_prom_en1 != 0xaa)
				init_reg[i][CMD] =
					data->pdata->init_code[i];
			else {
				reg = i2c_smbus_read_byte_data(data->client,
					init_reg[i][REG]);
				if (reg < 0) {
					pr_err("%s : i2c read fail, err=%d, %d line\n",
						__func__, reg, __LINE__);
					err = -EIO;
					goto done;
				}
				init_reg[i][CMD] = reg;
				pr_info("%s: skip HW_CONs, reset value = %x\n",
					__func__, reg);
			}
		}
	}

	/* 3.3V init code check */
	reg = i2c_smbus_read_byte_data(data->client, REG_HW_CON7);
	if (reg == 0x48)
		goto done;

	reg_hwcon10 = i2c_smbus_read_byte_data(data->client, REG_HW_CON10);
	reg_hwcon6 = i2c_smbus_read_byte_data(data->client, REG_HW_CON6);
	if (reg_hwcon10 == 0x37) {
		i2c_smbus_write_byte_data(data->client,
				REG_HW_CON6, init_reg[SET_HW_CON6][REG]-2);
		i2c_smbus_write_byte_data(data->client,
				REG_HW_CON7, 0x48);
	} else if ((reg_hwcon6 & 0x0f) == 0x0f) {
		i2c_smbus_write_byte_data(data->client,
				REG_HW_CON6, reg_hwcon6 & 0xfe);
		i2c_smbus_write_byte_data(data->client,
				REG_HW_CON7, 0x48);
	} else {
		i2c_smbus_write_byte_data(data->client,
				REG_HW_CON7, 0x48);
	}
done:
	return err;
}

static int asp01_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct asp01_data *data;
	struct asp01_platform_data *pdata = client->dev.platform_data;
	int err;
	u8 reg;

	pr_info("is started.\n");
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
		pr_err("%s: i2c functionality check failed!\n", __func__);
		err = -ENODEV;
		goto exit;
	}

	data = kzalloc(sizeof(struct asp01_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto exit;
	}

	if (pdata) {
		data->pdata = pdata;
	} else {
		pr_err("%s, pdata is NULL.\n", __func__);
		err = -EINVAL;
		goto err_read_reg;
	}

	/* check device */
	reg = i2c_smbus_read_byte_data(client,
		REG_ID_CHECK);
	if (reg < 0) {
		pr_err("%s : i2c read fail, err=%d, %d line\n",
			__func__, reg, __LINE__);
		err = -EIO;
		goto err_read_reg;
	}
	if (reg != SLAVE_ADDR) {
		pr_err("%s : Invalid slave address(0x%x)\n", __func__, reg);
		err = -EIO;
		goto err_read_reg;
	}

#ifdef USE_EEPROM
	/* eeprom reset */
	err = i2c_smbus_write_byte_data(client,
		REG_UNLOCK, 0x5A);
	if (err) {
		pr_err("%s: i2c write fail, err=%d, %d line\n",
			__func__, err, __LINE__);
		goto err_read_reg;
	}
	err = i2c_smbus_write_byte_data(client,
		REG_EEP_ADDR1, 0xFF);
	if (err) {
		pr_err("%s: i2c write fail, err=%d, %d line\n",
			__func__, err, __LINE__);
		goto err_read_reg;
	}
	err = i2c_smbus_write_byte_data(client,
		REG_EEP_ST_CON, 0x42);
	if (err) {
		pr_err("%s: i2c write fail, err=%d, %d line\n",
			__func__, err, __LINE__);
		goto err_read_reg;
	}
	usleep_range(15000, 15100);
	err = i2c_smbus_write_byte_data(client,
		REG_EEP_ST_CON, 0x06);
	if (err) {
		pr_err("%s: i2c write fail, err=%d, %d line\n",
			__func__, err, __LINE__);
		goto err_read_reg;
	}
	usleep_range(5000, 5100);
#endif
	data->client = client;
	i2c_set_clientdata(client, data);

	if (asp01_apply_hw_dep_set(data) < 0)
		goto err_read_reg;

	atomic_set(&data->enable, 0);
	data->skip_data = false;

	data->input = input_allocate_device();
	if (!data->input) {
		pr_err("%s: count not allocate input device\n", __func__);
		err = -ENOMEM;
		goto err_read_reg;
	}
	data->input->name = "grip_sensor";
	data->input->id.bustype = BUS_I2C;
	input_set_capability(data->input, EV_REL, REL_MISC);
	input_set_drvdata(data->input, data);

	err = input_register_device(data->input);
	if (err < 0) {
		input_free_device(data->input);
		pr_err("%s: could not register input device\n", __func__);
		goto err_read_reg;
	}

	mutex_init(&data->data_mutex);

	/* wake lock init */
	wake_lock_init(&data->gr_wake_lock, WAKE_LOCK_SUSPEND,
		       "grip_wake_lock");

#ifdef FEATURE_INIT_TOUCH_OFF
	INIT_DELAYED_WORK(&data->d_work, asp01_delay_work_func);
#endif

	/* Setup sysfs */
	err =
	    sysfs_create_group(&data->input->dev.kobj,
			       &asp01_attribute_group);
	if (err < 0) {
		pr_err("%s: sysfs_create_group failed(%d)\n", __func__, err);
		goto err_sysfs_create_group;
	}

	/* creating device for test & calibration */
	data->dev = sensors_classdev_register("grip_sensor");
	if (IS_ERR(data->dev)) {
		pr_err("%s: class create failed(grip_sensor)\n",
			__func__);
		err = PTR_ERR(data->dev);
		goto err_acc_device_create;
	}

	err = device_create_file(data->dev, &dev_attr_raw_data);
	if (err < 0) {
		pr_err("%s: Failed to create device file(%s)\n",
				__func__, dev_attr_raw_data.attr.name);
		goto err_acc_device_create_file;
	}

	err = device_create_file(data->dev, &dev_attr_vendor);
	if (err < 0) {
		pr_err("%s: Failed to create device file(%s)\n",
				__func__, dev_attr_vendor.attr.name);
		goto err_vendor_device_create_file;
	}

	err = device_create_file(data->dev, &dev_attr_name);
	if (err < 0) {
		pr_err("%s: Failed to create device file(%s)\n",
				__func__, dev_attr_name.attr.name);
		goto err_name_device_create_file;
	}

	err = device_create_file(data->dev, &dev_attr_calibration);
	if (err < 0) {
		pr_err("%s: Failed to create device file(%s)\n",
				__func__, dev_attr_calibration.attr.name);
		goto err_calibration_device_create_file;
	}

	err = device_create_file(data->dev, &dev_attr_threshold);
	if (err < 0) {
		pr_err("%s: Failed to create device file(%s)\n",
				__func__, dev_attr_threshold.attr.name);
		goto err_threshold_device_create_file;
	}

	err = device_create_file(data->dev, &dev_attr_check_crcs);
	if (err < 0) {
		pr_err("%s: Failed to create device file(%s)\n",
				__func__, dev_attr_check_crcs.attr.name);
		goto err_check_crcs_device_create_file;
	}

	err = device_create_file(data->dev, &dev_attr_onoff);
	if (err < 0) {
		pr_err("%s: Failed to create device file(%s)\n",
				__func__, dev_attr_onoff.attr.name);
		goto err_onoff_device_create_file;
	}

	err = device_create_file(data->dev, &dev_attr_reset);
	if (err < 0) {
		pr_err("%s: Failed to create device file(%s)\n",
				__func__, dev_attr_reset.attr.name);
		goto err_reset_device_create_file;
	}

	err = device_create_file(data->dev, &dev_attr_cscr_const);
	if (err < 0) {
		pr_err("%s: Failed to create device file(%s)\n",
				__func__, dev_attr_cscr_const.attr.name);
		goto err_cscr_const_device_create_file;
	}
	dev_set_drvdata(data->dev, data);

	INIT_WORK(&data->work, asp01_work_func);

	if (data->pdata->irq > 0) {
		err = request_threaded_irq(data->pdata->irq,
				NULL, asp01_irq_thread,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"asp01", data);
		if (err < 0) {
			dev_err(&client->dev,
				"irq request failed %d, error %d\n",
				data->pdata->irq, err);
				goto err_irq_initialize;
		}
		disable_irq(data->pdata->irq);
	} else
		dev_err(&client->dev,
			"irq request failed %d, error %d\n",
			data->pdata->irq, err);
	usb_switch_register_notify(&jig_notify);
	muic_attached_accessory_inquire();
	pr_info("%s is successful.\n", __func__);

	return 0;

err_irq_initialize:
	device_remove_file(data->dev, &dev_attr_cscr_const);
err_cscr_const_device_create_file:
	device_remove_file(data->dev, &dev_attr_reset);
err_reset_device_create_file:
	device_remove_file(data->dev, &dev_attr_onoff);
err_onoff_device_create_file:
	device_remove_file(data->dev, &dev_attr_check_crcs);
err_check_crcs_device_create_file:
	device_remove_file(data->dev, &dev_attr_threshold);
err_threshold_device_create_file:
	device_remove_file(data->dev, &dev_attr_calibration);
err_calibration_device_create_file:
	device_remove_file(data->dev, &dev_attr_name);
err_name_device_create_file:
	device_remove_file(data->dev, &dev_attr_vendor);
err_vendor_device_create_file:
	device_remove_file(data->dev, &dev_attr_raw_data);
err_acc_device_create_file:
	sensors_classdev_unregister(data->dev);
err_acc_device_create:
	sysfs_remove_group(&data->input->dev.kobj,
			   &asp01_attribute_group);
err_sysfs_create_group:
	wake_lock_destroy(&data->gr_wake_lock);
	mutex_destroy(&data->data_mutex);
	input_unregister_device(data->input);
err_read_reg:
	kfree(data);
exit:
	usb_switch_unregister_notify(&jig_notify);
	return err;
}

static int asp01_remove(struct i2c_client *client)
{
	struct asp01_data *data = i2c_get_clientdata(client);
	usb_switch_unregister_notify(&jig_notify);
	device_remove_file(data->dev, &dev_attr_cscr_const);
	device_remove_file(data->dev, &dev_attr_reset);
	device_remove_file(data->dev, &dev_attr_onoff);
	device_remove_file(data->dev, &dev_attr_check_crcs);
	device_remove_file(data->dev, &dev_attr_threshold);
	device_remove_file(data->dev, &dev_attr_calibration);
	device_remove_file(data->dev, &dev_attr_name);
	device_remove_file(data->dev, &dev_attr_vendor);
	device_remove_file(data->dev, &dev_attr_raw_data);
	sensors_classdev_unregister(data->dev);
	sysfs_remove_group(&data->input->dev.kobj,
		&asp01_attribute_group);
	wake_lock_destroy(&data->gr_wake_lock);
	mutex_destroy(&data->data_mutex);
	input_unregister_device(data->input);
	kfree(data);

	return 0;
}

static const struct i2c_device_id asp01_id[] = {
	{ "asp01", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, asp01_id);

static struct i2c_driver asp01_driver = {
	.probe = asp01_probe,
	.remove = __devexit_p(asp01_remove),
	.id_table = asp01_id,
	.driver = {
		.pm = &asp01_pm_ops,
		.owner = THIS_MODULE,
		.name = "asp01",
	},
};

static int __init asp01_init(void)
{
	return i2c_add_driver(&asp01_driver);
}

static void __exit asp01_exit(void)
{
	i2c_del_driver(&asp01_driver);
}

module_init(asp01_init);
module_exit(asp01_exit);

MODULE_DESCRIPTION("asp01 grip sensor driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
