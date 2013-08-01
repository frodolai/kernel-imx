/*******************************************************
 *
 * PenMount TouchScreen Driver for Android
 *
 * Copyright (c) 2012 PenMount Touch Solutions <penmount@seed.net.tw>
 *
 *******************************************************/

/*******************************************************
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *******************************************************/

////////////////////////////////////////////////////////
// Include
////////////////////////////////////////////////////////
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
#include <linux/input/mt.h>
#else
#include <linux/input.h>
#endif
#include <linux/usb.h>
#include "penmount.h"
////////////////////////////////////////////////////////
// Global Variable
////////////////////////////////////////////////////////
struct proc_dir_entry * PMDRIVER_PROCFS_DIRENTRY;
////////////////////////////////////////////////////////
// Define
////////////////////////////////////////////////////////
#define PMMODULE_DESC                    "PenMount Touchscreen Driver"
#define PMDRIVER_PROCFS_DIRNAME          "PenMount"
#define PMDRIVER_PROCFS_NODE_FEATURE     "HidFeatureReport"
#define PMDRIVER_PROCFS_NODE_RELOADCALIB "ReloadCalib"
#define PMDRIVER_PROCFS_NODE_ENABLECALIB "EnableCalib"
////////////////////////////////////////////////////////
// Calibration
////////////////////////////////////////////////////////
int RawX = 0;
int RawY = 0;
module_param(RawX, int, S_IRUSR | S_IRGRP | S_IROTH);
module_param(RawY, int, S_IRUSR | S_IRGRP | S_IROTH);
//------------------------------------------------------
static
int pmDevice_SendCommand(struct strPENMOUNT *pPenMount, unsigned char *pCommand,
		unsigned char *pResponse)
{
	switch (pPenMount->pInputDev->id.bustype)
	{
	case BUS_USB:
		pmUsbIF_SetFeatureReport(pPenMount->USB.pDevice, pCommand,
				PMDRIVER_SIZE_FEATUREREPORT);
		if (pResponse == NULL)
			break;
		pmUsbIF_GetFeatureReport(pPenMount->USB.pDevice, pResponse,
				PMDRIVER_SIZE_FEATUREREPORT);
		break;
	}
	return 1;
}
//------------------------------------------------------
static
int pmDriver_LoadEEPROM(struct strPENMOUNT *pPenMount, unsigned char index,
		unsigned char *pData0, unsigned char *pData1)
{
	unsigned char pCommand[PMDRIVER_SIZE_FEATUREREPORT];
	unsigned char pResponse[PMDRIVER_SIZE_FEATUREREPORT];

	memset(pCommand, 0, PMDRIVER_SIZE_FEATUREREPORT);
	memset(pResponse, 0, PMDRIVER_SIZE_FEATUREREPORT);

	pCommand[0] = 0xF3;
	pCommand[2] = index;

	if (!pmDevice_SendCommand(pPenMount, pCommand, pResponse))
		return 0;

	if (pData0)
		*pData0 = pResponse[3];
	if (pData1)
		*pData1 = pResponse[4];

	return 1;
}
//------------------------------------------------------
static
int pmDriver_DisableDevice(struct strPENMOUNT *pPenMount)
{
	unsigned char pCommand[PMDRIVER_SIZE_FEATUREREPORT];
	unsigned char pResponse[PMDRIVER_SIZE_FEATUREREPORT];

	memset(pCommand, 0, PMDRIVER_SIZE_FEATUREREPORT);
	memset(pResponse, 0, PMDRIVER_SIZE_FEATUREREPORT);

	pCommand[0] = 0xF0;
	if (!pmDevice_SendCommand(pPenMount, pCommand, pResponse))
		return 0;

	return 1;
}
//------------------------------------------------------
static
int pmDriver_EnableDevice(struct strPENMOUNT *pPenMount)
{
	unsigned char pCommand[PMDRIVER_SIZE_FEATUREREPORT];
	unsigned char pResponse[PMDRIVER_SIZE_FEATUREREPORT];

	memset(pCommand, 0, PMDRIVER_SIZE_FEATUREREPORT);
	memset(pResponse, 0, PMDRIVER_SIZE_FEATUREREPORT);

	pCommand[0] = 0xF1;
	if (!pmDevice_SendCommand(pPenMount, pCommand, pResponse))
		return 0;

	return 1;
}
//------------------------------------------------------
static
int pmDriver_LoadCalibData(struct strPENMOUNT *pPenMount)
{
	unsigned char EEPROM[128];
	unsigned char i = 0;
	unsigned char offset = 0;
	memset(EEPROM, 0, 128);
	for (i = 0; i < 64; i += 2)
	{
		if (!pmDriver_LoadEEPROM(pPenMount, i, EEPROM + i, EEPROM + i + 1))
		{
			printk(
					"[PENMOUNT] Unable to load data from controller! Please check the cable connection!\n");
			return 0;
		}
	}

	pPenMount->Calib.mode = EEPROM[1];
	printk("[PENMOUNT] Calib Mode : %d\n", pPenMount->Calib.mode);
	switch (pPenMount->Calib.mode)
	{
	case PENMOUNT_CALIBMODE_TSLIB:
		pPenMount->Calib.bEnable = 1;
		for (i = 0, offset = 0x0C; i < 7; i++, offset += 4)
		{
			pPenMount->Calib.param[i] = *(int *) (EEPROM + offset);
			printk("[PENMOUNT] Calibration parameter %d : %d\n", i, pPenMount->Calib.param[i]);
		}
		break;
	default:
		pPenMount->Calib.bEnable = 0;
		printk("[PENMOUNT] Driver calibration disabled !\n");
		break;
	}
	return 1;
}
//------------------------------------------------------
void pmDriver_CalibData(struct strPENMOUNT *pPenMount, unsigned short *pX,
		unsigned short *pY)
{
	int xtemp = (int) *pX;
	int ytemp = (int) *pY;

	if (pPenMount->Calib.mode != PENMOUNT_CALIBMODE_TSLIB)
		return;

	if (!pPenMount->Calib.bEnable)
		return;

	*pX = (unsigned short) ((pPenMount->Calib.param[0]
			+ pPenMount->Calib.param[1] * xtemp
			+ pPenMount->Calib.param[2] * ytemp) / pPenMount->Calib.param[6]);
	*pY = (unsigned short) ((pPenMount->Calib.param[3]
			+ pPenMount->Calib.param[4] * xtemp
			+ pPenMount->Calib.param[5] * ytemp) / pPenMount->Calib.param[6]);

	if (*pX > 1023)
		*pX = 1023;

	if (*pY > 1023)
		*pY = 1023;

	return;
}
////////////////////////////////////////////////////////
// PROCFS
////////////////////////////////////////////////////////
static
int pmDriver_ProcfsRead(char * pBuffer, char **pBufLoc, off_t offset,
		int cbData, int * pEof, void * pConext)
{
	struct strPENMOUNT *pPenMount = NULL;
	ssize_t rc = 0;

	pPenMount = (struct strPENMOUNT *) pConext;

	if ((pPenMount == NULL) || (pBuffer == NULL))
		return -EINVAL;

	switch (pPenMount->pInputDev->id.bustype)
	{
	case BUS_USB:
		pmUsbIF_GetFeatureReport(pPenMount->USB.pDevice, pPenMount->pResponse,
				PMDRIVER_SIZE_FEATUREREPORT);
		break;
	}

	rc = sprintf(pBuffer, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
			pPenMount->pResponse[0], pPenMount->pResponse[1],
			pPenMount->pResponse[2], pPenMount->pResponse[3],
			pPenMount->pResponse[4]);

	return rc;
}
//------------------------------------------------------
static ssize_t pmDriver_ProcfsWrite(struct file * pFile, const char * pBuffer,
		unsigned long count, void * pConext)
{
	struct strPENMOUNT *pPenMount = NULL;
	int i = 0;
	unsigned int pData[6] =
	{ 0, 0, 0, 0, 0, 0 };
	unsigned char pCommand[6] =
	{ 0, 0, 0, 0, 0, 0 };

	pPenMount = (struct strPENMOUNT *) pConext;
	if ((pPenMount == NULL) || (pBuffer == NULL))
		return -EINVAL;

	sscanf(pBuffer, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", &pData[0], &pData[1],
			&pData[2], &pData[3], &pData[4]);
	for (i = 0; i < PMDRIVER_SIZE_FEATUREREPORT; i++)
		pCommand[i] = pData[i];

	switch (pPenMount->pInputDev->id.bustype)
	{
	case BUS_USB:
		pmUsbIF_SetFeatureReport(pPenMount->USB.pDevice, pCommand,
				PMDRIVER_SIZE_FEATUREREPORT);
		break;
	}

	return count;
}
//------------------------------------------------------
static ssize_t pmDriver_ReloadCalib(struct file * pFile, const char * pBuffer,
		unsigned long count, void * pConext)
{
	struct strPENMOUNT *pPenMount = NULL;

	pPenMount = (struct strPENMOUNT *) pConext;
	if ((pPenMount == NULL) || (pBuffer == NULL))
		return -EINVAL;

	printk("[PENMOUNT] Reloading calibration settings ...\n");
	pmDriver_DisableDevice(pPenMount);
	pmDriver_LoadCalibData(pPenMount);
	pmDriver_EnableDevice(pPenMount);

	return count;
}
//------------------------------------------------------
static ssize_t pmDriver_EnableCalib(struct file * pFile, const char * pBuffer,
		unsigned long count, void * pConext)
{
	struct strPENMOUNT *pPenMount = NULL;
	signed int bEnable = 0 ;
	pPenMount = (struct strPENMOUNT *) pConext;
	if ((pPenMount == NULL) || (pBuffer == NULL))
		return -EINVAL;

	sscanf(pBuffer, "%d", &bEnable);
	if (bEnable)
		printk("[PENMOUNT] Calibration is enabled !\n");
	else
		printk("[PENMOUNT] Calibration is disabled !\n");

	pPenMount->Calib.bEnable = (unsigned char)bEnable;

	return count;
}
//------------------------------------------------------
int pmDriver_Procfsmkdir(void)
{
	PMDRIVER_PROCFS_DIRENTRY = proc_mkdir(PMDRIVER_PROCFS_DIRNAME, NULL);
	return 0;
}
//------------------------------------------------------
void pmDriver_Procfsrmdir(void)
{
	remove_proc_entry(PMDRIVER_PROCFS_DIRNAME, NULL);
	return;
}
//------------------------------------------------------
int pmDriver_ProcfsInit(struct strPENMOUNT * pPenMount,
		struct proc_dir_entry * pProcRoot)
{
	pPenMount->pProcfs = create_proc_entry(PMDRIVER_PROCFS_NODE_FEATURE, 0x666,
			pProcRoot);
	if (pPenMount->pProcfs == NULL)
		return -ENOMEM;

	pPenMount->pProcfs->read_proc = pmDriver_ProcfsRead;
	pPenMount->pProcfs->write_proc = pmDriver_ProcfsWrite;

	pPenMount->pProcfs->mode = 0x666;
	pPenMount->pProcfs->uid = 0;
	pPenMount->pProcfs->gid = 0;
	pPenMount->pProcfs->size = 40;
	pPenMount->pProcfs->data = pPenMount;

	pPenMount->Calib.pProcfsReload = create_proc_entry(
			PMDRIVER_PROCFS_NODE_RELOADCALIB, 0x666, pProcRoot);
	if (pPenMount->Calib.pProcfsReload == NULL)
		return -ENOMEM;

	pPenMount->Calib.pProcfsReload->write_proc = pmDriver_ReloadCalib;

	pPenMount->Calib.pProcfsReload->mode = 0x666;
	pPenMount->Calib.pProcfsReload->uid = 0;
	pPenMount->Calib.pProcfsReload->gid = 0;
	pPenMount->Calib.pProcfsReload->size = 40;
	pPenMount->Calib.pProcfsReload->data = pPenMount;

	pPenMount->Calib.pProcfsEnable = create_proc_entry(
			PMDRIVER_PROCFS_NODE_ENABLECALIB, 0x666, pProcRoot);
	if (pPenMount->Calib.pProcfsEnable == NULL)
		return -ENOMEM;

	pPenMount->Calib.pProcfsEnable->write_proc = pmDriver_EnableCalib;

	pPenMount->Calib.pProcfsEnable->mode = 0x666;
	pPenMount->Calib.pProcfsEnable->uid = 0;
	pPenMount->Calib.pProcfsEnable->gid = 0;
	pPenMount->Calib.pProcfsEnable->size = 40;
	pPenMount->Calib.pProcfsEnable->data = pPenMount;

	return 0;
}
//------------------------------------------------------
void pmDriver_ProcfsExit(struct strPENMOUNT *pPenMount)
{
	remove_proc_entry(PMDRIVER_PROCFS_NODE_FEATURE, PMDRIVER_PROCFS_DIRENTRY);
	remove_proc_entry(PMDRIVER_PROCFS_NODE_RELOADCALIB, PMDRIVER_PROCFS_DIRENTRY);
	remove_proc_entry(PMDRIVER_PROCFS_NODE_ENABLECALIB, PMDRIVER_PROCFS_DIRENTRY);
	return;
}
////////////////////////////////////////////////////////
// InputDev
////////////////////////////////////////////////////////
int pmDriver_InputDevOpen(struct input_dev *pInputDev)
{
	struct strPENMOUNT *pPenMount = NULL;

	pPenMount = (struct strPENMOUNT *) input_get_drvdata(pInputDev);
	if (pPenMount == NULL)
		return -ENXIO;

	if (pInputDev->id.bustype != BUS_USB)
		return 0;

	if (!pPenMount->OpenCount++)
	{
		pPenMount->USB.pUrb->dev = pPenMount->USB.pDevice;
		usb_submit_urb(pPenMount->USB.pUrb, GFP_KERNEL);
	}

	return 0;
}
//------------------------------------------------------
void pmDriver_InputDevClose(struct input_dev *pInputDev)
{
	struct strPENMOUNT *pPenMount = NULL;

	pPenMount = (struct strPENMOUNT *) input_get_drvdata(pInputDev);

	if (pPenMount == NULL)
		return;

	if (pInputDev->id.bustype != BUS_USB)
		return;

	if (!--pPenMount->OpenCount)
		usb_kill_urb(pPenMount->USB.pUrb);

	return;
}
//------------------------------------------------------
struct strPENMOUNT * pmDriver_InitContext(void)
{
	struct strPENMOUNT *pPenMount = NULL;
	int i = 0;

	pPenMount = kzalloc(sizeof(struct strPENMOUNT), GFP_KERNEL);
	if (pPenMount == NULL)
		return NULL;

	for (i = 0; i < PMDRIVER_MAXTOUCH; i++)
		pPenMount->Touch[i].Slot = i;

	return pPenMount;
}
//------------------------------------------------------
int pmDriver_InputDevInit(struct strPENMOUNT *pPenMount,
		struct device *pParentDev, struct input_id *pInputID)
{
	int rc = 0;
	struct input_dev *pInputDev = NULL;

	pInputDev = input_allocate_device();
	if (pInputDev == NULL)
		return -ENOMEM;

	pInputDev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	pInputDev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	memcpy(&pInputDev->id, pInputID, sizeof(struct input_id));

	switch (pPenMount->Model)
	{
	case PMDEVICE_MODEL_P2:
		pPenMount->MaxTouch = 2;
		input_set_abs_params(pInputDev, ABS_MT_TOUCH_MAJOR, 0, 5, 0, 0);
		input_set_abs_params(pInputDev, ABS_MT_POSITION_X, 0, 0x7FF, 0, 0);
		input_set_abs_params(pInputDev, ABS_MT_POSITION_Y, 0, 0x7FF, 0, 0);
		input_set_abs_params(pInputDev, ABS_X, 0, 0x7FF, 0, 0);
		input_set_abs_params(pInputDev, ABS_Y, 0, 0x7FF, 0, 0);
		break;
	case PMDEVICE_MODEL_6000:
		pPenMount->MaxTouch = 1;
		input_set_abs_params(pInputDev, ABS_X, 0, 0x3FF, 0, 0);
		input_set_abs_params(pInputDev, ABS_Y, 0, 0x3FF, 0, 0);
		break;
	}

	printk("[PENMOUNT] Registering input device for %s\n",
			pPenMount->szDeviceName);

	pInputDev->name = pPenMount->szDeviceName;
	pInputDev->phys = pPenMount->szPhysDevice;
	pInputDev->open = pmDriver_InputDevOpen;
	pInputDev->close = pmDriver_InputDevClose;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
	set_bit(INPUT_PROP_DIRECT, pInputDev->propbit);
#endif

	// Set up "dev" field
	pInputDev->dev.parent = pParentDev;

	input_set_drvdata(pInputDev, pPenMount);

	// Register input device
	rc = input_register_device(pInputDev);

	pPenMount->pInputDev = pInputDev;

	pmDriver_DisableDevice(pPenMount);
	pmDriver_LoadCalibData(pPenMount);
	pmDriver_EnableDevice(pPenMount);

	return rc;
}
//------------------------------------------------------
void pmDriver_InputDevExit(struct strPENMOUNT *pPenMount)
{
	if (pPenMount == NULL)
		return;

	input_set_drvdata(pPenMount->pInputDev, NULL);
	input_unregister_device(pPenMount->pInputDev);

	return;
}
////////////////////////////////////////////////////////
// Event Processing
////////////////////////////////////////////////////////
void pmDriver_ProcessEvent(struct input_dev *pInputDev,
		struct strPENMOUNT *pPenMount, struct strPMTOUCH *pTouch)
{
	if (pTouch->bTouch)
	{
		if (!pTouch->bTouching)
		{
			if (pPenMount->MaxTouch == 1)
				input_report_key(pInputDev, BTN_TOUCH, 1);
			pTouch->bTouching = 1;
		}
	}
	else
	{
		if (pTouch->bTouching)
		{
			if (pPenMount->MaxTouch == 1)
				input_report_key(pInputDev, BTN_TOUCH, 0);
			pTouch->bTouching = 0;
		}
	}

	if (pPenMount->MaxTouch == 1)
	{
		input_report_abs(pInputDev, ABS_X, pTouch->X);
		input_report_abs(pInputDev, ABS_Y, pTouch->Y);
		input_sync(pInputDev);
	}

	pTouch->bTouch = 0;

	return;
}
//------------------------------------------------------
void pmDriver_ProcessMTEvent(struct input_dev *pInputDev,
		struct strPENMOUNT *pPenMount)
{
	__u8 i = 0;
	__u8 TouchCount = 0;

	if (pPenMount->MaxTouch == 1)
		return;

	// Android 2 and 3 uses Linux MT Protocol A
	for (i = 0; i < pPenMount->MaxTouch; i++)
	{
		if ((pPenMount->pMainTouch == NULL) && (pPenMount->Touch[i].bTouching))
		pPenMount->pMainTouch = &pPenMount->Touch[i];

		if (!pPenMount->Touch[i].bTouching)
		continue;

		input_report_abs(pInputDev, ABS_MT_TOUCH_MAJOR, 2);
		input_report_abs(pInputDev, ABS_MT_POSITION_X, pPenMount->Touch[i].X);
		input_report_abs(pInputDev, ABS_MT_POSITION_Y, pPenMount->Touch[i].Y);
		input_mt_sync(pInputDev);
		TouchCount++;
	}

	if (!TouchCount)
	input_mt_sync(pInputDev);

	// Single-Touch Emulation

	if (pPenMount->pMainTouch)
	{
		if (pPenMount->pMainTouch->bTouching)
		{
			input_report_key(pInputDev, BTN_TOUCH, 1);
			input_report_abs(pInputDev, ABS_X, pPenMount->pMainTouch->X);
			input_report_abs(pInputDev, ABS_Y, pPenMount->pMainTouch->Y);
		}
		else
		{
			input_report_key(pInputDev, BTN_TOUCH, 0);
			pPenMount->pMainTouch = NULL;
		}
	}

	input_sync(pInputDev);

	for (i = 0; i < pPenMount->MaxTouch; i++)
		pPenMount->Touch[i].bUpdated = 0;

	return;
}
////////////////////////////////////////////////////////
// Module
////////////////////////////////////////////////////////
static
int __init pmModule_Init ( void )
{
	int rc = 0;
	pmDriver_Procfsmkdir();

	rc = usb_register ( &PMDRIVER_USBIF );

	return rc;
}
//------------------------------------------------------
static
void __exit pmModule_Exit ( void )
{
	usb_deregister ( &PMDRIVER_USBIF );

	pmDriver_Procfsrmdir(); // Child nodes need to be removed first
	return;
}
//------------------------------------------------------
module_init (pmModule_Init);
module_exit (pmModule_Exit);
////////////////////////////////////////////////////////
MODULE_AUTHOR("PenMount Touch Solutions <penmount@seed.net.tw>");
MODULE_DESCRIPTION(PMMODULE_DESC);
MODULE_LICENSE("GPL");
////////////////////////////////////////////////////////
