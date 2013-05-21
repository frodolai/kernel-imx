/*******************************************************
 *
 *  PenMount USB TouchScreen Driver
 *
 *  Copyright (c) 2012 PenMount Touch Solutions <penmount@seed.net.tw>
 *
 *******************************************************/

/*******************************************************
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *******************************************************/

////////////////////////////////////////////////////////
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/usb.h>
#include <linux/usb/input.h> // define usb_to_input_id()
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
#include <linux/input/mt.h>
#else
#include <linux/input.h>
#endif
#include <linux/hid.h>
#include "penmount.h"
////////////////////////////////////////////////////////
#define PMUSB_DRIVER_DESC "PenMount USB TouchScreen Driver"
////////////////////////////////////////////////////////
int pmUsbIF_SetFeatureReport(struct usb_device * pUsbDevice,
		unsigned char * pBuffer, unsigned long cbData)
{
	int ret = 0;

	ret = usb_control_msg(pUsbDevice, usb_sndctrlpipe(pUsbDevice, 0), // pipe
	HID_REQ_SET_REPORT, // request
			USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE, // requesttype
			((HID_FEATURE_REPORT + 1) << 8), // value
			0, // index
			pBuffer, // data
			cbData, // size
			USB_CTRL_SET_TIMEOUT);
	if (ret == cbData)
		return 1;

	return 0;
}
//------------------------------------------------------
int pmUsbIF_GetFeatureReport(struct usb_device * pUsbDevice,
		unsigned char * pBuffer, unsigned long cbData)
{
	int ret;

	ret = usb_control_msg(pUsbDevice, usb_rcvctrlpipe(pUsbDevice, 0), // pipe
	HID_REQ_GET_REPORT, // request
			USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE, // requesttype
			((HID_FEATURE_REPORT + 1) << 8), // value
			0, // index
			pBuffer, // data
			cbData, // size
			USB_CTRL_GET_TIMEOUT);
	if (ret == cbData)
		return 1;

	return 0;
}
//------------------------------------------------------
static
void pmUsbIF_ReadComplete(struct urb *pUrb)
{
	struct strPENMOUNT *pPenMount = pUrb->context;
	struct input_dev *pInputDev = pPenMount->pInputDev;
	__u8 Slot = 0;
	__u8 i = 0;
	__u8 bProcessEvents = 1;

	switch (pUrb->status)
	{
	case 0:
		switch (pPenMount->Model)
		{
		default:
		case PMDEVICE_MODEL_6000:
			pPenMount->Touch[0].bTouch = ((pPenMount->pInputBuffer[0] & 0xF0)
					== 0x70);
			pPenMount->Touch[0].X = pPenMount->pInputBuffer[1]
					+ (pPenMount->pInputBuffer[2] << 8);
			pPenMount->Touch[0].Y = pPenMount->pInputBuffer[3]
					+ (pPenMount->pInputBuffer[4] << 8);
			RawX = pPenMount->Touch[0].X;
			RawY = pPenMount->Touch[0].Y;
			pmDriver_CalibData(pPenMount, &pPenMount->Touch[0].X,
					&pPenMount->Touch[0].Y);
			pmDriver_ProcessEvent(pInputDev, pPenMount, &pPenMount->Touch[0]);
			break;
		case PMDEVICE_MODEL_P2:
			Slot = (pPenMount->pInputBuffer[0] & 0x0F);
			pPenMount->Touch[Slot].bTouch = ((pPenMount->pInputBuffer[0] & 0xF0)
					== 0x70);
			pPenMount->Touch[Slot].X = pPenMount->pInputBuffer[1]
					+ (pPenMount->pInputBuffer[2] << 8);
			pPenMount->Touch[Slot].Y = pPenMount->pInputBuffer[3]
					+ (pPenMount->pInputBuffer[4] << 8);
			if (pPenMount->Touch[Slot].bUpdated)
				pmDriver_ProcessMTEvent(pInputDev, pPenMount);
			pPenMount->Touch[Slot].bUpdated = 1;
			pmDriver_ProcessEvent(pInputDev, pPenMount,
					&pPenMount->Touch[Slot]);
			for (i = 0; i < pPenMount->MaxTouch; i++)
				if ((!pPenMount->Touch[i].bUpdated)
						&& (pPenMount->Touch[i].bTouching))
					bProcessEvents = 0;
			if (bProcessEvents)
				pmDriver_ProcessMTEvent(pInputDev, pPenMount);
			break;
		}
	default:
		usb_submit_urb(pUrb, GFP_ATOMIC);
		break;
	case -ENOENT: /* -2   */
	case -ETIME: /* -62 URB Time Out */
	case -ECONNRESET: /* -104 */
	case -ESHUTDOWN: /* -108 */
		break;

	}

	return;
}
//------------------------------------------------------
static
int pmUsbIF_Probe(struct usb_interface *pUsbInterface,
		const struct usb_device_id *pUsbDeviceID)
{
	int rc = 0;
	struct usb_device *pUsbDevice = NULL;
	struct usb_endpoint_descriptor *pUsbEndPoint = NULL;
	struct strPENMOUNT *pPenMount = NULL;
	struct input_id InputID;

	pPenMount = pmDriver_InitContext();
	if (pPenMount == NULL)
		return -ENOMEM;

	// Initialize the USB Device
	pUsbDevice = interface_to_usbdev(pUsbInterface);
	pUsbEndPoint = &pUsbInterface->cur_altsetting->endpoint[0].desc;

	pPenMount->USB.pDevice = pUsbDevice;

	usb_make_path(pUsbDevice, pPenMount->szPhysDevice,
			sizeof(pPenMount->szPhysDevice));
	strlcat(pPenMount->szPhysDevice, "/input0",
			sizeof(pPenMount->szPhysDevice));

	pPenMount->USB.pUrb = usb_alloc_urb(0, GFP_KERNEL);
	if (pPenMount->USB.pUrb == NULL)
	{
		kfree(pPenMount);
		return -ENOMEM;
	}
	pPenMount->cbPacket = 5;

	pPenMount->pInputBuffer = (unsigned char *)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
			usb_alloc_coherent
#else
			usb_buffer_alloc
#endif
			( pUsbDevice ,
					pPenMount->cbPacket ,
					GFP_KERNEL ,
					&pPenMount->USB.DmaAddress );
	if (pPenMount->pInputBuffer == NULL)
	{
		kfree(pPenMount);
		return -ENOMEM;
	}

	usb_fill_int_urb(pPenMount->USB.pUrb, pUsbDevice,
			usb_rcvintpipe(pUsbDevice, pUsbEndPoint->bEndpointAddress),
			pPenMount->pInputBuffer, pPenMount->cbPacket, pmUsbIF_ReadComplete,
			pPenMount, pUsbEndPoint->bInterval);

	pPenMount->USB.pUrb->dev = pUsbDevice;
	pPenMount->USB.pUrb->transfer_dma = pPenMount->USB.DmaAddress;
	pPenMount->USB.pUrb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	usb_set_intfdata(pUsbInterface, pPenMount);

	// Initialize the Input Device
	usb_to_input_id(pUsbDevice, &InputID);
	pPenMount->Model = InputID.product;
	switch (pPenMount->Model)
	{
	case PMDEVICE_MODEL_P2:
		snprintf(pPenMount->szDeviceName, sizeof(pPenMount->szDeviceName),
				"PenMount P2 USB TouchScreen");
		break;
	case PMDEVICE_MODEL_6000:
		snprintf(pPenMount->szDeviceName, sizeof(pPenMount->szDeviceName),
				"PenMount 6000 USB TouchScreen");
		break;
	}

	rc = pmDriver_InputDevInit(pPenMount, &pUsbInterface->dev, &InputID);
	if (rc)
	{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		usb_free_coherent
#else
		usb_buffer_free
#endif
		( pUsbDevice ,
				pPenMount->cbPacket ,
				pPenMount->pInputBuffer ,
				pPenMount->USB.DmaAddress );

		usb_free_urb(pPenMount->USB.pUrb);
		kfree(pPenMount);
		return -ENOMEM;
	}

	pmDriver_ProcfsInit(pPenMount, PMDRIVER_PROCFS_DIRENTRY);

	return rc;
}
//------------------------------------------------------
static
void pmUsbIF_Disconnect(struct usb_interface *pUsbInterface)
{
	struct strPENMOUNT *pPenMount = NULL;

	pPenMount = (struct strPENMOUNT *) usb_get_intfdata(pUsbInterface);
	if (pPenMount == NULL)
		return;

	pmDriver_ProcfsExit(pPenMount);
	pmDriver_InputDevExit(pPenMount);

	if (pPenMount->USB.pUrb)
		usb_free_urb(pPenMount->USB.pUrb);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
	usb_free_coherent
#else
	usb_buffer_free
#endif
	( interface_to_usbdev(pUsbInterface) ,
			pPenMount->cbPacket ,
			pPenMount->pInputBuffer ,
			pPenMount->USB.DmaAddress );

	usb_set_intfdata(pUsbInterface, NULL);

	kfree(pPenMount);

	return;
}
////////////////////////////////////////////////////////
static struct usb_device_id PMUSBIF_DEVICES[] =
{
{ USB_DEVICE(PMDEVICE_VENDORID_USB, PMDEVICE_MODEL_6000) },
{ USB_DEVICE(PMDEVICE_VENDORID_USB, PMDEVICE_MODEL_P2) },
{ } };
//------------------------------------------------------
struct usb_driver PMDRIVER_USBIF =
{
	.name = "USB-PenMount" ,
	.probe = pmUsbIF_Probe ,
	.disconnect = pmUsbIF_Disconnect ,
	.id_table = PMUSBIF_DEVICES ,
};
//------------------------------------------------------
MODULE_DEVICE_TABLE(usb, PMUSBIF_DEVICES);
