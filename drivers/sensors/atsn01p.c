/*
 *  AD semiconductor atsn01p grip sensor driver
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

#include <linux/module.h>
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
#include <linux/atsn01p.h>
#include "core/sensors_core.h"

/* For Debugging */
#define atsn01p_dbgmsg(str, args...) pr_debug("%s: " str, __func__, ##args)
#define atsn01p_infomsg(str, args...) pr_info("%s: " str, __func__, ##args)
#undef GRIP_DEBUG
#define USE_EEPROM

#define VENDOR		"ADSEMICON"
#define CHIP_ID		"ASP01"
#define CALIBRATION_FILE_PATH	"/efs/grip_cal_data"
#define CAL_DATA_NUM	16
#define MFM_REF_NUM	(CAL_DATA_NUM / 2)
#define SLAVE_ADDR	0x48
#define SET_MFM_DONE	(0x1 << 4)
#define EN_INIT_CAL	(0x1 << 3)
#define DIV	1000
#define BYTE_MSK	0xff
#define BYTE_SFT	8
#define RESET_MSK 0xfe

enum ATSN01P_I2C_IF {
	REG = 0,
	CMD,
};

/* register address */
enum ATSN01P_REG {
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

enum ATSN01P_CMD {
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
	{ REG_UNLOCK,		0x5a},
	{ REG_RST_ERR,		0x33},
	{ REG_PROX_PER,		0x38},
	{ REG_PAR_PER,		0x38},
	{ REG_TOUCH_PER,	0x3c},
	{ REG_HI_CAL_PER,	0x30},
	{ REG_BSMFM_SET,	0x31},
	{ REG_ERR_MFM_CYC,	0x33},
	{ REG_TOUCH_MFM_CYC,	0x24},
	{ REG_HI_CAL_SPD,	0x21},
	{ REG_CAL_SPD,		0x04},
	{ REG_INIT_REF,		0x00},
	{ REG_BFT_MOT,		0x40},
	{ REG_TOU_RF_EXT,	0x00},
	{ REG_SYS_FUNC,		0x10},
	{ REG_OFF_TIME,		0x30},
	{ REG_SENSE_TIME,	0x48},
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

struct atsn01p_data {
	struct i2c_client *client;
	struct device *dev;
	struct input_dev *input;
	struct work_struct work; /* for grip sensor */
	struct mutex data_mutex;
	atomic_t enable;
	struct atsn10p_platform_data *pdata;
	struct wake_lock gr_wake_lock;
	u8 cal_data[CAL_DATA_NUM];
	u16 mfm_ref_raw[MFM_REF_NUM];
	u8 default_mfm[CAL_DATA_NUM];
	u16 cr_per[4];
	int cr_cosnt;
	int cs_cosnt;
	bool skip_data;
};

static struct atsn01p_data *g_atsn01p;



static int atsn01p_reset(struct atsn01p_data *data)
{
	int err;
	u8 reg;

	/* sw reset */
	err = i2c_smbus_read_i2c_block_data(
		data->client, control_reg[CMD_RESET][REG],
		sizeof(reg), &reg);
	if (err != sizeof(reg)) {
		pr_err("%s : i2c read fail. err=%d\n",
			__func__, err);
		err = -EIO;
		return err;
	}

	err = i2c_smbus_write_byte_data(data->client,
			control_reg[CMD_RESET][REG],
			(RESET_MSK & reg)
			| control_reg[CMD_RESET][CMD]);
	if (err) {
		pr_err("%s: failed to write, err = %d\n",
			__func__, err);
		goto done;
	}
	err = i2c_smbus_write_byte_data(data->client,
			control_reg[CMD_RESET][REG],
			RESET_MSK & reg);
	if (err) {
		pr_err("%s: failed to write, err = %d\n",
			__func__, err);
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

static int atsn01p_init_touch_onoff(struct atsn01p_data *data, bool onoff)
{
	int err;

	if (onoff) {
		err = i2c_smbus_write_byte_data(data->client,
			control_reg[CMD_INIT_TOUCH_ON][REG],
			control_reg[CMD_INIT_TOUCH_ON][CMD]);
		if (err) {
				pr_err("%s: failed to write, err = %d\n",
					__func__, err);
				goto done;
		}
	} else {
		err = i2c_smbus_write_byte_data(data->client,
			control_reg[CMD_INIT_TOUCH_OFF][REG],
			control_reg[CMD_INIT_TOUCH_OFF][CMD]);
		if (err) {
				pr_err("%s: failed to write, err = %d\n",
					__func__, err);
				goto done;
		}
	}
done:
	return err;
}

static int atsn01p_init_code_set(struct atsn01p_data *data, int count)
{
	int err, i, j;
#ifdef GRIP_DEBUG
	u8 reg;
#endif
	for (j = 0; j < 20; j++) {/* need to write six time */
		/* clock on */
		err = i2c_smbus_write_byte_data(data->client,
				control_reg[CMD_CLK_ON][REG],
				control_reg[CMD_CLK_ON][CMD]);
		if (err) {
			pr_err("%s: failed to write, err = %d\n",
				__func__, err);
			goto done;
		}

		/* write Initializing code */
		for (i = 0; i < SET_REG_NUM; i++) {
			err = i2c_smbus_write_byte_data(data->client,
				init_reg[i][REG], init_reg[i][CMD]);
			if (err) {
				pr_err("%s: failed to write, err = %d\n",
					__func__, err);
				goto done;
			}
			pr_debug("%s: reg: %x, data: %x\n", __func__,
				init_reg[i][REG], init_reg[i][CMD]);
		}

		if (!count) {
			/* disable initial touch mode */
			atsn01p_init_touch_onoff(data, false);
			if (!j)
				pr_info("%s: cal disable\n", __func__);
		} else {
			/* wtire initial touch ref setting */
			for (i = 0; i < CAL_DATA_NUM; i++) {
				pr_debug("%s: reg(0x%x) = 0x%x\n", __func__,
					REG_MFM_INIT_REF0 + i,
					data->cal_data[i]);
				err = i2c_smbus_write_byte_data(data->client,
					REG_MFM_INIT_REF0 + i,
					data->cal_data[i]);
				if (err)
					pr_err("%s: failed to write, err = %d\n",
						__func__, err);
			};
			/* enable initial touch mode */
			atsn01p_init_touch_onoff(data, true);
			if (!j)
				pr_info("%s: cal enable\n", __func__);
		}
		usleep_range(5000, 5100);
	}
#ifdef GRIP_DEBUG
	/* verifying register set */
	for (i = 0; i < SET_REG_NUM; i++) {
		err = i2c_smbus_read_i2c_block_data(
			data->client, init_reg[i][REG],
			sizeof(reg), &reg);
		if (err != sizeof(reg)) {
			pr_err("%s : i2c read fail. err=%d\n",
				__func__, err);
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

static int atsn01p_open_calibration(struct atsn01p_data *data)
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
			return err;
	}

	err = cal_filp->f_op->read(cal_filp,
		(char *)data->cal_data,
		CAL_DATA_NUM * sizeof(u8), &cal_filp->f_pos);
	if (err != CAL_DATA_NUM * sizeof(u8)) {
		pr_err("%s: Can't read the cal data from file\n", __func__);
		err = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

set_init_code:
	for (i = 0; i < CAL_DATA_NUM; i++) {
		if (!data->cal_data[i])
			count -= 1;
	}

	mutex_lock(&data->data_mutex);

	err = atsn01p_reset(data);
	if (err) {
		pr_err("%s: atsn01p_reset, err = %d\n",
			__func__, err);
		return err;
	}
#ifdef USE_EEPROM
	if (!count) {
#endif
		err = atsn01p_init_code_set(data, count);
		if (err < 0) {
			pr_err("%s: atsn01p_init_code_set, err = %d\n",
				__func__, err);
			return err;
		}
#ifdef USE_EEPROM
	} else
		pr_info("%s: now use eeprom\n", __func__);
#endif
#ifdef GRIP_DEBUG
	/* verifiying MFM value*/
	for (i = 0; i < CAL_DATA_NUM; i++) {
		err = i2c_smbus_read_i2c_block_data(
			data->client, REG_MFM_INIT_REF0 + i,
			sizeof(reg), &reg);
		if (err != sizeof(reg)) {
			pr_err("%s : i2c read fail. err=%d\n",
				__func__, err);
			err = -EIO;
			return err;
		}
		pr_info("%s: verify reg(0x%x) = 0x%x\n", __func__,
			REG_MFM_INIT_REF0 + i, reg);
	}
#endif
	mutex_unlock(&data->data_mutex);

	return err;
}


static int atsn01p_do_calibrate(struct device *dev, bool do_calib)
{
	struct atsn01p_data *data = dev_get_drvdata(dev);
	struct file *cal_filp = NULL;
	int err = 0;
	int i;
	int count = 100;
	u8 reg;
	u32 cr_ref[4] = {0,};
	u32 cs_ref[4] = {0,};
	mm_segment_t old_fs;

	pr_info("%s: do_calib=%d\n", __func__, do_calib);

	if (do_calib) {
		for (i = 0; i < 4; i++) {
			/* 1. set MFM unmber (MFM0~MFM3)*/
			err = i2c_smbus_write_byte_data(data->client,
					REG_SETMFM, i);
			if (err) {
				pr_err("%s: failed to write, err = %d\n",
						__func__, err);
				return err;
			}
			/* 2. wait until data ready */
			do {
				err = i2c_smbus_read_i2c_block_data(
					data->client, REG_SETMFM,
					sizeof(reg), &reg);
				if (err != sizeof(reg)) {
					pr_err("%s : i2c read fail. err=%d\n",
						__func__, err);
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
				pr_err("%s: failed to write, err = %d\n",
					__func__, err);
				return err;
			}
			err = i2c_smbus_read_i2c_block_data(data->client,
				REG_CR_REF0, sizeof(u8),
				&data->cal_data[i * 4]);
			if (err != sizeof(reg)) {
				pr_err("%s :i2c read fail. err=%d, %d line\n",
					__func__, __LINE__, err);
				err = -EIO;
				return err;
			}
			err = i2c_smbus_read_i2c_block_data(data->client,
				REG_CR_REF1, sizeof(u8),
				&data->cal_data[(i * 4) + 1]);
			if (err != sizeof(reg)) {
				pr_err("%s : i2c read fail. err=%d, %d line\n",
					__func__, __LINE__, err);
				err = -EIO;
				return err;
			}
			err = i2c_smbus_read_i2c_block_data(data->client,
				REG_CS_REF0, sizeof(u8),
				&data->cal_data[(i * 4) + 2]);
			if (err != sizeof(reg)) {
				pr_err("%s : i2c read fail. err=%d, %d line\n",
					__func__, __LINE__, err);
				err = -EIO;
				return err;
			}
			err = i2c_smbus_read_i2c_block_data(data->client,
				REG_CS_REF1, sizeof(u8),
				&data->cal_data[(i * 4) + 3]);
			if (err != sizeof(reg)) {
				pr_err("%s : i2c read fail. err=%d, %d line\n",
					__func__, __LINE__, err);
				err = -EIO;
				return err;
			}
			err = i2c_smbus_write_byte_data(data->client,
					control_reg[CMD_CLK_ON][REG],
					control_reg[CMD_CLK_ON][CMD]);
			if (err) {
				pr_err("%s: failed to write, err = %d\n",
					__func__, err);
				return err;
			}
			cr_ref[i] =
				(data->cal_data[(i * 4) + 1] << BYTE_SFT
					| data->cal_data[(i * 4)]);
			cs_ref[i] =
				(data->cal_data[(i * 4) + 3] << BYTE_SFT
					| data->cal_data[(i * 4) + 2]);

			data->mfm_ref_raw[i * 2] = cr_ref[i];
			data->mfm_ref_raw[(i * 2) + 1] = cs_ref[i];

			/* 4. multiply cr, cs constant*/
			/* cs ref + cr ref x cs cont */
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
		}

		/* write MFM ref value */
		for (i = 0; i < CAL_DATA_NUM; i++) {
			err = i2c_smbus_write_byte_data(data->client,
					REG_MFM_INIT_REF0 + i,
					data->cal_data[i]);
			if (err) {
				pr_err("%s: failed to write, err = %d\n",
						__func__, err);
				return err;
			}
			pr_debug("%s: wr reg(0x%x) = 0x%x\n", __func__,
				REG_MFM_INIT_REF0 + i,
				data->cal_data[i]);
		}
		atsn01p_init_touch_onoff(data, true);
	} else {
		/* reset MFM ref value */
		for (i = 0; i < CAL_DATA_NUM; i++) {
			data->cal_data[i] = 0;
			err = i2c_smbus_write_byte_data(data->client,
					REG_MFM_INIT_REF0 + i, 0);
			if (err) {
				pr_err("%s: failed to write, err = %d\n",
						__func__, err);
				return err;
			}
			pr_debug("%s: reset reg(0x%x) = 0x%x\n", __func__,
				REG_MFM_INIT_REF0 + i, data->cal_data[i]);
		}
		atsn01p_init_touch_onoff(data, false);
	}

#ifdef USE_EEPROM
	/* write into eeprom */
	err = i2c_smbus_write_byte_data(data->client,
			REG_EEP_ST_CON, 0x02);
	if (err) {
		pr_err("%s: failed to write, err = %d\n",
			__func__, err);
		return err;
	}
	count = 20;
	do {
		err = i2c_smbus_read_i2c_block_data(
			data->client, REG_EEP_ST_CON,
			sizeof(reg), &reg);
		if (err != sizeof(reg)) {
			pr_err("%s : i2c read fail. err=%d\n",
				__func__, err);
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

static int atsn01p_grip_enable(struct atsn01p_data *data)
{
	int err;

	mutex_lock(&data->data_mutex);
	err = i2c_smbus_write_byte_data(data->client,
			control_reg[CMD_CLK_ON][REG],
			control_reg[CMD_CLK_ON][CMD]);
	if (err)
		pr_err("%s: failed to write, err = %d\n",
			__func__, err);
	mutex_unlock(&data->data_mutex);

	pr_info("%s: reg: %x, data: %x\n", __func__,
		control_reg[CMD_CLK_ON][REG],
		control_reg[CMD_CLK_ON][CMD]);

	return err;
}

static int atsn01p_grip_disable(struct atsn01p_data *data)
{
	int err, grip;
	int count = 10;
	mutex_lock(&data->data_mutex);
	err = i2c_smbus_write_byte_data(data->client,
			REG_SYS_FUNC, 0x14);
	if (err)
		pr_err("%s: failed to write, err = %d\n",
			__func__, err);
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
		pr_err("%s: failed to write, err = %d\n",
			__func__, err);
	mutex_unlock(&data->data_mutex);

	pr_info("%s: reg: %x, data: %x\n", __func__,
		control_reg[CMD_CLK_OFF][REG],
		control_reg[CMD_CLK_OFF][CMD]);

	return err;
}

static int atsn01p_suspend(struct device *dev)
{
	int err = 0;
	struct atsn01p_data *data = dev_get_drvdata(dev);

	if (atomic_read(&data->enable))
		enable_irq_wake(g_atsn01p->client->irq);
	else {
		mutex_lock(&data->data_mutex);
		err = i2c_smbus_write_byte_data(data->client,
				control_reg[CMD_CLK_OFF][REG],
				control_reg[CMD_CLK_OFF][CMD]);
		if (err)
			pr_err("%s: failed to write, err = %d\n",
				__func__, err);
		mutex_unlock(&data->data_mutex);
	}
	return err;
}

static int atsn01p_resume(struct device *dev)
{
	int err = 0;
	struct atsn01p_data *data = dev_get_drvdata(dev);

	if (atomic_read(&data->enable))
		disable_irq_wake(g_atsn01p->client->irq);

	return err;
}

static const struct dev_pm_ops atsn01p_pm_ops = {
	.suspend = atsn01p_suspend,
	.resume = atsn01p_resume,
};

static ssize_t atsn01p_enable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct atsn01p_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", atomic_read(&data->enable));
}

static ssize_t atsn01p_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct atsn01p_data *data = input_get_drvdata(input);
	unsigned long enable = 0;
	int err;
	static bool isFirst = true;

	if (strict_strtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable && !atomic_read(&data->enable)) {
		if (isFirst) {
			atsn01p_open_calibration(data);
			isFirst = false;
		}

		err = atsn01p_grip_enable(data);
		if (err < 0)
			goto done;

		msleep(250);

		enable_irq(g_atsn01p->client->irq);
	} else if (!enable && atomic_read(&data->enable)) {
		disable_irq(g_atsn01p->client->irq);
		err = atsn01p_grip_disable(data);
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
		   atsn01p_enable_show, atsn01p_enable_store);

static struct attribute *atsn01p_attributes[] = {
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group atsn01p_attribute_group = {
	.attrs = atsn01p_attributes
};


static void atsn01p_work_func(struct work_struct *work)
{
	int touch, jig_on;
	pr_info("%s", __func__);
	disable_irq(g_atsn01p->client->irq);

	if (unlikely(!g_atsn01p->pdata)) {
		pr_err("%s: can't get pdata\n", __func__);
		goto done;
	}
	jig_on = gpio_get_value(g_atsn01p->pdata->adj_det);
	touch = gpio_get_value(g_atsn01p->pdata->t_out);

	/* check adj_det pin & skip data cmd for factory test */
	if (!jig_on && !g_atsn01p->skip_data) {
		pr_info("%s: touch INT : %d!!\n", __func__, touch);
		input_report_rel(g_atsn01p->input, REL_MISC, touch + 1);
		input_sync(g_atsn01p->input);
	} else
		pr_info("%s: skipped input report jig_on: %d, skip cmd: %d\n",
			__func__, jig_on, g_atsn01p->skip_data);
done:
	enable_irq(g_atsn01p->client->irq);
}

static irqreturn_t atsn01p_irq_thread(int irq, void *dev)
{
	pr_info("%s: irq created\n", __func__);
	wake_lock_timeout(&g_atsn01p->gr_wake_lock, 3 * HZ);
	schedule_work(&g_atsn01p->work);
	return IRQ_HANDLED;
}

static ssize_t atsn01p_calibration_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct atsn01p_data *data = dev_get_drvdata(dev);
	int err = -1;
	int i;
	int count = MFM_REF_NUM;
	u16 mfm_ref[MFM_REF_NUM];

	atsn01p_open_calibration(data);

	for (i = 0; i < MFM_REF_NUM; i++) {
		mfm_ref[i] =
			(data->cal_data[2 * i + 1] << BYTE_SFT) |
				data->cal_data[2 * i];
		if (!mfm_ref[i])
			count -= 1;
	}

	/* if all data zero, cal data is initialized*/
	if (count)
		err = 1;

	return sprintf(buf, "%d, %d, %d, %d, %d\n",
		err, data->cr_per[0], data->cr_per[1],
		data->cr_per[2], data->cr_per[3]);
}

static ssize_t atsn01p_calibration_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int err;
	bool do_calib;

	if (sysfs_streq(buf, "1"))
		do_calib = true;
	else if (sysfs_streq(buf, "0"))
		do_calib = false;
	else {
		pr_info("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	mutex_lock(&g_atsn01p->data_mutex);

	atsn01p_reset(g_atsn01p);
	/* write init code with disable init touch (0)*/
	atsn01p_init_code_set(g_atsn01p, 0);

	err = atsn01p_do_calibrate(dev, do_calib);

	mutex_unlock(&g_atsn01p->data_mutex);

	if (err < 0) {
		pr_err("%s: atsn01p_do_calibrate() failed\n", __func__);
		return err;
	}

	return count;
}
static DEVICE_ATTR(calibration, 0664,
		   atsn01p_calibration_show, atsn01p_calibration_store);

static ssize_t atsn01p_grip_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VENDOR);
}
static DEVICE_ATTR(vendor, 0664, atsn01p_grip_vendor_show, NULL);

static ssize_t atsn01p_grip_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", CHIP_ID);
}
static DEVICE_ATTR(name, 0664, atsn01p_grip_name_show, NULL);

static ssize_t atsn01p_grip_raw_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 reg;
	u8 cs_per[2] = {0,};
	u8 cr_per[2] = {0,};
	u16 mlt_cs_per[4] = {0,};
	u16 mlt_cr_per[4] = {0,};
	int count = 100;
	int err, i;

	if (!atomic_read(&g_atsn01p->enable)) {
		atsn01p_grip_enable(g_atsn01p);
		usleep_range(10000, 11000);
	}

	mutex_lock(&g_atsn01p->data_mutex);
	for (i = 0; i < 4; i++) {
		/* 1. set MFM unmber (MFM0~MFM3)*/
		err = i2c_smbus_write_byte_data(g_atsn01p->client,
				REG_SETMFM, i);
		if (err) {
			pr_err("%s: failed to write, err = %d, %d line\n",
				__func__, __LINE__, err);
			return err;
		}
		/* 2. wait until data ready */
		do {
			err = i2c_smbus_read_i2c_block_data(
					g_atsn01p->client, REG_SETMFM,
					sizeof(reg), &reg);
			if (err != sizeof(reg)) {
				pr_err("%s : i2c read fail. err=%d, %d line\n",
					__func__, __LINE__, err);
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
		/* 3. read CS percent value */
		err = i2c_smbus_write_byte_data(g_atsn01p->client,
				control_reg[CMD_CLK_OFF][REG],
				control_reg[CMD_CLK_OFF][CMD]);
		if (err) {
			pr_err("%s: failed to write, err = %d\n",
				__func__, err);
			return err;
		}
		err = i2c_smbus_read_i2c_block_data(g_atsn01p->client,
			REG_CS_PERL, sizeof(u8),
			&cs_per[0]);
		if (err != sizeof(reg)) {
			pr_err("%s :i2c read fail. err=%d, %d line\n",
				__func__, __LINE__, err);
			err = -EIO;
			return err;
		}
		err = i2c_smbus_read_i2c_block_data(g_atsn01p->client,
			REG_CS_PERH, sizeof(u8),
			&cs_per[1]);
		if (err != sizeof(reg)) {
			pr_err("%s : i2c read fail. err=%d, %d line\n",
				__func__, __LINE__, err);
			err = -EIO;
			return err;
		}
		mlt_cs_per[i] = (cs_per[1] << 8) | cs_per[0];
		/* 4. read CR percent value */
		err = i2c_smbus_read_i2c_block_data(g_atsn01p->client,
			REG_CR_PER_L, sizeof(u8),
			&cr_per[0]);
		if (err != sizeof(reg)) {
			pr_err("%s :i2c read fail. err=%d, %d line\n",
				__func__, __LINE__, err);
			err = -EIO;
			return err;
		}
		err = i2c_smbus_read_i2c_block_data(g_atsn01p->client,
			REG_CR_PER_H, sizeof(u8),
			&cr_per[1]);
		if (err != sizeof(reg)) {
			pr_err("%s : i2c read fail. err=%d, %d line\n",
				__func__, __LINE__, err);
			err = -EIO;
			return err;
		}
		err = i2c_smbus_write_byte_data(g_atsn01p->client,
				control_reg[CMD_CLK_ON][REG],
				control_reg[CMD_CLK_ON][CMD]);
		if (err) {
			pr_err("%s: failed to write, err = %d\n",
				__func__, err);
			return err;
		}
		g_atsn01p->cr_per[i] =
			mlt_cr_per[i] = (cr_per[1] << 8) | cr_per[0];
	}
	mutex_unlock(&g_atsn01p->data_mutex);

#ifdef GRIP_DEBUG
		for (i = 0; i < SET_REG_NUM; i++) {
			i2c_smbus_read_i2c_block_data(
				g_atsn01p->client, init_reg[i][REG],
				sizeof(reg), &reg);
			pr_info("%s: reg(0x%x) = 0x%x, default 0x%x\n",
				__func__,
				init_reg[i][REG], reg, init_reg[i][CMD]);
		}
		/* verifiying MFM value*/
		for (i = 0; i < CAL_DATA_NUM; i++) {
			i2c_smbus_read_i2c_block_data(
				g_atsn01p->client, REG_MFM_INIT_REF0 + i,
				sizeof(reg), &reg);
			pr_info("%s: verify reg(0x%x) = 0x%x, data: 0x%x\n",
				__func__, REG_MFM_INIT_REF0 + i,
				reg, g_atsn01p->cal_data[i]);
		}
#endif

	if (!atomic_read(&g_atsn01p->enable))
		atsn01p_grip_disable(g_atsn01p);

	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d\n",
		mlt_cs_per[0], mlt_cs_per[1], mlt_cs_per[2], mlt_cs_per[3],
		mlt_cr_per[0], mlt_cr_per[1], mlt_cr_per[2], mlt_cr_per[3]);
}
static DEVICE_ATTR(raw_data, 0664, atsn01p_grip_raw_data_show, NULL);

static ssize_t atsn01p_grip_threshold_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", init_reg[SET_PROX_PER][CMD]);
}
static DEVICE_ATTR(threshold, 0664, atsn01p_grip_threshold_show, NULL);

static ssize_t atsn01p_grip_check_crcs_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 reg;
	u8 crcs[48] = {0,};
	int count = 100;
	int err, i, j;

	for (i = 0; i < 4; i++) {
		/* 1. set MFM unmber (MFM0~MFM3)*/
		err = i2c_smbus_write_byte_data(g_atsn01p->client,
				REG_SETMFM, i);
		if (err) {
			pr_err("%s: failed to write, err = %d, %d line\n",
				__func__, __LINE__, err);
			return err;
		}
		/* 2. wait until data ready */
		do {
			err = i2c_smbus_read_i2c_block_data(
					g_atsn01p->client, REG_SETMFM,
					sizeof(reg), &reg);
			if (err != sizeof(reg)) {
				pr_err("%s : i2c read fail. err=%d, %d line\n",
					__func__, __LINE__, err);
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
			err = i2c_smbus_read_i2c_block_data(g_atsn01p->client,
				j, sizeof(u8),
				&crcs[(12 * i) + j - REG_CR_CNT]);
			if (err != sizeof(reg)) {
				pr_err("%s :i2c read fail. err=%d, %d line\n",
					__func__, __LINE__, err);
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
static DEVICE_ATTR(check_crcs, 0664, atsn01p_grip_check_crcs_show, NULL);

static ssize_t atsn01p_grip_onoff_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%d\n", !g_atsn01p->skip_data);
}

static ssize_t atsn01p_grip_onoff_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	if (sysfs_streq(buf, "1"))
		g_atsn01p->skip_data = false;
	else if (sysfs_streq(buf, "0"))
		g_atsn01p->skip_data = true;
	else {
		pr_info("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR(onoff, 0664,
		   atsn01p_grip_onoff_show, atsn01p_grip_onoff_store);

/* ----------------- *
   Input device interface
 * ------------------ */
static int atsn01p_input_init(struct atsn01p_data *data)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;
	dev->name = "grip_sensor";
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_REL, REL_MISC);
	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0)
		goto done;
	data->input = dev;
done:
	return 0;
}

static int atsn01p_apply_hw_dep_set(struct atsn01p_data *data)
{
	u8 reg, i;
	u8 reg_prom_en1;
	int err;

	err = i2c_smbus_read_i2c_block_data(data->client,
			REG_PROM_EN1, sizeof(reg_prom_en1), &reg_prom_en1);
	if (err != sizeof(reg_prom_en1)) {
		pr_err("%s : i2c read fail. err=%d\n", __func__, err);
		err = -EIO;
		goto done;
	}

	if (data->pdata == NULL) {
		pr_err("%s: set default init code\n", __func__);
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

	for (i = 0; i < CAL_DATA_NUM; i++) {
		err = i2c_smbus_read_i2c_block_data
				(data->client,
				REG_MFM_INIT_REF0 + i,
				sizeof(reg), &reg);
		if (err != sizeof(reg)) {
			pr_err("%s : i2c read fail. err=%d\n",
				__func__, err);
			err = -EIO;
			goto done;
		}
		data->default_mfm[i] = reg;

		pr_info("%s: default_mfm[%d] = %x\n",
			__func__, i, data->default_mfm[i]);
	}

	for (i = 0; i < SET_REG_NUM; i++) {
		if (i < SET_HW_CON1)
			init_reg[i][CMD] = data->pdata->init_code[i];
		else {
			/* keep reset value if PROM_EN1 is 0xaa*/
			if (reg_prom_en1 != 0xaa)
				init_reg[i][CMD] =
					data->pdata->init_code[i];
			else {
				err = i2c_smbus_read_i2c_block_data
						(data->client,
						init_reg[i][REG],
						sizeof(reg), &reg);
				if (err != sizeof(reg)) {
					pr_err("%s : i2c read fail. err=%d\n",
						__func__, err);
					err = -EIO;
					goto done;
				}
				init_reg[i][CMD] = reg;
				pr_info("%s: skip HW_CONs, reset value = %x\n",
					__func__, reg);
			}
		}
	}
done:
	return err;
}

static int atsn01p_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct atsn01p_data *data;
	struct atsn10p_platform_data *pdata = client->dev.platform_data;
	int err;
	u8 reg;

	atsn01p_infomsg("is started.\n");

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
		pr_err("%s: i2c functionality check failed!\n", __func__);
		err = -ENODEV;
		goto exit;
	}

	data = kzalloc(sizeof(struct atsn01p_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto exit;
	}

	/* check device */
	err = i2c_smbus_read_i2c_block_data(client,
			REG_ID_CHECK, sizeof(reg), &reg);
	if (err != sizeof(reg)) {
		pr_err("%s : there is no such device. err=%d\n", __func__, err);
		err = -EIO;
		goto err_read_reg;
	}
	if (reg != SLAVE_ADDR) {
		pr_err("%s : Invalid slave address\n", __func__);
		goto err_read_reg;
	}

#ifdef USE_EEPROM
	/* eeprom reset */
	err = i2c_smbus_write_byte_data(client,
		REG_UNLOCK, 0x5A);
	if (err) {
		pr_err("%s: failed to write, err = %d\n",
			__func__, err);
		goto err_read_reg;
	}
	err = i2c_smbus_write_byte_data(client,
		REG_EEP_ADDR1, 0xFF);
	if (err) {
		pr_err("%s: failed to write, err = %d\n",
			__func__, err);
		goto err_read_reg;
	}
	err = i2c_smbus_write_byte_data(client,
		REG_EEP_ST_CON, 0x42);
	if (err) {
		pr_err("%s: failed to write, err = %d\n",
			__func__, err);
		goto err_read_reg;
	}
	usleep_range(15000, 15100);
	err = i2c_smbus_write_byte_data(client,
		REG_EEP_ST_CON, 0x06);
	if (err) {
		pr_err("%s: failed to write, err = %d\n",
			__func__, err);
		goto err_read_reg;
	}
	usleep_range(5000, 5100);
#endif

	data->pdata = pdata;
	data->client = client;
	g_atsn01p = data;
	i2c_set_clientdata(client, data);

	if (atsn01p_apply_hw_dep_set(data) < 0)
			goto err_read_reg;

	atomic_set(&data->enable, 0);
	data->skip_data = false;
	atsn01p_input_init(data);

	mutex_init(&data->data_mutex);

	/* wake lock init */
	wake_lock_init(&data->gr_wake_lock, WAKE_LOCK_SUSPEND,
		       "grip_wake_lock");

	/* Setup sysfs */
	err =
	    sysfs_create_group(&data->input->dev.kobj,
			       &atsn01p_attribute_group);
	if (err < 0)
		goto err_sysfs_create_group;

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

	dev_set_drvdata(data->dev, data);

	INIT_WORK(&data->work, atsn01p_work_func);

	if (client->irq > 0) {
		err = request_threaded_irq(client->irq,
				NULL, atsn01p_irq_thread,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"atsn01p", data);
		if (err < 0) {
			dev_err(&client->dev,
				"irq request failed %d, error %d\n",
				client->irq, err);
				goto err_irq_initialize;
			}
		disable_irq(client->irq);
	}

	atsn01p_infomsg("is successful.\n");

	return 0;


err_irq_initialize:
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
			   &atsn01p_attribute_group);
err_sysfs_create_group:
	wake_lock_destroy(&data->gr_wake_lock);
	mutex_destroy(&data->data_mutex);
	input_free_device(data->input);
err_read_reg:
	kfree(data);
exit:
	return err;
}

static int atsn01p_remove(struct i2c_client *client)
{
	struct atsn01p_data *data = i2c_get_clientdata(client);
	device_remove_file(data->dev, &dev_attr_onoff);
	device_remove_file(data->dev, &dev_attr_check_crcs);
	device_remove_file(data->dev, &dev_attr_threshold);
	device_remove_file(data->dev, &dev_attr_calibration);
	device_remove_file(data->dev, &dev_attr_name);
	device_remove_file(data->dev, &dev_attr_vendor);
	device_remove_file(data->dev, &dev_attr_raw_data);
	sensors_classdev_unregister(data->dev);
	sysfs_remove_group(&data->input->dev.kobj,
		&atsn01p_attribute_group);
	wake_lock_destroy(&data->gr_wake_lock);
	mutex_destroy(&data->data_mutex);
	input_free_device(data->input);
	kfree(data);

	return 0;
}

static const struct i2c_device_id atsn01p_id[] = {
	{ "atsn01p", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, atsn01p_id);

static struct i2c_driver atsn01p_driver = {
	.probe = atsn01p_probe,
	.remove = __devexit_p(atsn01p_remove),
	.id_table = atsn01p_id,
	.driver = {
		.pm = &atsn01p_pm_ops,
		.owner = THIS_MODULE,
		.name = "atsn01p",
	},
};

static int __init atsn01p_init(void)
{
	return i2c_add_driver(&atsn01p_driver);
}

static void __exit atsn01p_exit(void)
{
	i2c_del_driver(&atsn01p_driver);
}

module_init(atsn01p_init);
module_exit(atsn01p_exit);

MODULE_DESCRIPTION("atsn01p grip sensor driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
