////////////////////////////////////////////////////////
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/usb.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
#include <linux/input/mt.h>
#else
#include <linux/input.h>
#endif
////////////////////////////////////////////////////////
#define PMDRIVER_MAXTOUCH       2
#define PENMOUNT_MAXTRACKID     0xFFFF
//------------------------------------------------------
#ifndef PMDEVICE_VENDORID_USB
#define PMDEVICE_VENDORID_USB   0x14E1
#endif
//------------------------------------------------------
#ifndef PMDEVICE_MODEL_P2
#define PMDEVICE_MODEL_P2       0x3500
#endif
//------------------------------------------------------
#ifndef PMDEVICE_MODEL_6000
#define PMDEVICE_MODEL_6000     0x6000
#endif
//------------------------------------------------------
#ifdef  USB_CTRL_SET_TIMEOUT
#undef  USB_CTRL_SET_TIMEOUT
#endif
#define USB_CTRL_SET_TIMEOUT    5000
//------------------------------------------------------
#ifdef  USB_CTRL_GET_TIMEOUT
#undef  USB_CTRL_GET_TIMEOUT
#endif
#define USB_CTRL_GET_TIMEOUT    5000
//------------------------------------------------------
#define PMDRIVER_SIZE_FEATUREREPORT  5
#define PENMOUNT_CALIBMODE_TSLIB     5
////////////////////////////////////////////////////////
// Structure
////////////////////////////////////////////////////////
struct strPENMOUNT;
//------------------------------------------------------
struct strPMTOUCH
{
	__u8 bUpdated;
	__s32 TrackID;
	__u8 Slot;
	__u8 bTouch;
	__u8 bTouching;
	__u16 X;
	__u16 Y;
	__u16 LastX;
	__u16 LastY;
	__u8 LastState;
};
//------------------------------------------------------
struct strPMUSB
{
	struct urb * pUrb;
	struct usb_device * pDevice;
	dma_addr_t DmaAddress;
};
//------------------------------------------------------
struct strPMCALIB
{
	int bEnable;
	int mode;
	int param[7];
	struct proc_dir_entry * pProcfsReload;
	struct proc_dir_entry * pProcfsEnable;
};
//------------------------------------------------------
struct strPENMOUNT
{
	char szPhysDevice[64];
	char szDeviceName[64];
	__u8 MaxTouch;
	__u16 Model;
	__u8 cbPacket;
	__u8 *pInputBuffer;
	__u8 pResponse[6];
	__u8 OpenCount;
	struct input_dev * pInputDev;
	struct proc_dir_entry * pProcfs;
	struct strPMTOUCH * pMainTouch;
	struct strPMTOUCH Touch[PMDRIVER_MAXTOUCH];
	struct strPMUSB USB;
	struct strPMCALIB Calib;
};
////////////////////////////////////////////////////////
// Extern Variables
////////////////////////////////////////////////////////
extern struct usb_driver PMDRIVER_USBIF;
extern struct proc_dir_entry * PMDRIVER_PROCFS_DIRENTRY;
extern int RawX;
extern int RawY;
////////////////////////////////////////////////////////
// Function prototype
////////////////////////////////////////////////////////
struct strPENMOUNT * pmDriver_InitContext(void);
int pmDriver_InputDevInit(struct strPENMOUNT *, struct device *,
		struct input_id *);
void pmDriver_InputDevExit(struct strPENMOUNT *);
int pmDriver_InputDevOpen(struct input_dev *);
void pmDriver_InputDevClose(struct input_dev *);
void pmDriver_ProcessEvent(struct input_dev *, struct strPENMOUNT *,
		struct strPMTOUCH *);
void pmDriver_ProcessMTEvent(struct input_dev *, struct strPENMOUNT *);
int pmDriver_ProcfsInit(struct strPENMOUNT *, struct proc_dir_entry *);
void pmDriver_ProcfsExit(struct strPENMOUNT *);
int pmUsbIF_GetFeatureReport(struct usb_device *, unsigned char *,
		unsigned long);
int pmUsbIF_SetFeatureReport(struct usb_device *, unsigned char *,
		unsigned long);
void pmDriver_CalibData(struct strPENMOUNT *, unsigned short *,
		unsigned short *);
