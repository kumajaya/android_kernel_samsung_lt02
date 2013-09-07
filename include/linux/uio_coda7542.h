/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * (C) Copyright 2012 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef _UIO_CODA7542_H_
#define _UIO_CODA7542_H_

#define CODA7542_IOC_MAGIC 'C'
#define CODA7542_POWER_ON	_IO(CODA7542_IOC_MAGIC, 1)
#define CODA7542_POWER_OFF	_IO(CODA7542_IOC_MAGIC, 2)
#define CODA7542_CLK_ON		_IO(CODA7542_IOC_MAGIC, 3)
#define CODA7542_CLK_OFF	_IO(CODA7542_IOC_MAGIC, 4)
#define CODA7542_LOCK		_IOW(CODA7542_IOC_MAGIC, 5, unsigned int)
#define CODA7542_UNLOCK		_IO(CODA7542_IOC_MAGIC, 6)
#define CODA7542_GETSET_INFO	_IOW(CODA7542_IOC_MAGIC, 7, unsigned int)
#define UIO_CODA7542_NAME	"pxa-coda7542"

extern void coda7542_power_switch(int on);

#endif /* _UIO_CODA7542_H_ */
