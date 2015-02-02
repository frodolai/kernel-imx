#include <linux/i2c.h>
#include <linux/interrupt.h>
//#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/firmware.h>
//#include <asm/intel_vlv2.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define X_MIN                                 0x00
#define Y_MIN                                 0x00
#define X_MAX                                 0xFFF
#define Y_MAX                                 0xFFF
#define NUTOUCH_MAX_REPORTED_PRESSURE         10
#define NUTOUCH_MAX_TOUCH_SIZE                100

#define NUTOUCH_ReadResponseMsg_Noaddress       1
#define NUTOUCH_ReadResponseMsg                 2
#define NUTOUCH_ReadSendCMDgetCRCMsg            3
#define NUTOUCH_HandleTouchMsg                  4

/* Debug levels */
#define DEBUG_DETAIL                          2
#define DEBUG_BASIC                           1
#define DEBUG_ERROR                           0
#define NUM_FINGERS_SUPPORTED                 5

#define OFFSET_CID		0
#define OFFSET_STATUS	0
#define OFFSET_X_L		1
#define OFFSET_Y_L		2
#define OFFSET_X_H		3
#define OFFSET_Y_H		3
#define OFFSET_PRESSURE		4
#define OFFSET_AREA		5

/* Firmware */
#define NUTOUCH_FW_NAME         	    "nutouch.cb"
#define NUTOUCH_FWRESET_TIME    	    200		     /* msec */
#define NUTOUCH_WAIT_BOOTLOADER_TIME	    100		     /* msec */

#define NUTOUCH_WAIT_FRAME_DATA		    0x80
#define NUTOUCH_FRAME_CRC_PASS		    0x04

#define TAG_DATA_TYPE	12
#define TAG_SIZE	12
#define TAG_VER		12
#define TAG_DATE	12

struct point_data {
        short Status;
        short X;
        short Y;
        short Pressure;
        short Area;
};

static int debug = DEBUG_ERROR;
static int LastUpdateID = 0;
static u8 i2cfail = 0;
static int irq_time = 0;

#define nutouch_debug(level, ...) \
	do { \
		if (debug >= (level)) \
			printk(__VA_ARGS__); \
	} while (0)


struct nutouch_data
{
	struct i2c_client    *client;
	struct input_dev     *input;
	struct semaphore     sema;
	struct workqueue_struct *wq;
	//struct delayed_work  dwork;
	struct work_struct dwork;
	int irq;
	unsigned		gp;
	short irq_type;
	struct point_data PointBuf[NUM_FINGERS_SUPPORTED];
	#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
	#endif
	int dev_major;
	int dev_minor;
	struct cdev *dev_cdevp;
	struct class *dev_classp;
	int update_mode;
	int x_range;
	int y_range;
	int orient;
	int backup_rc;
	int en_water_proof;
	int fw_ver[16];
	int delta;
};

struct nutouch_data  *nutouch;
struct nutouch_data *nutouch_suspend;
static int suspend_resume = 0;
static int NUTOUCH_Deepsleep(struct nutouch_data *nutouch);
static int NUTOUCH_Resume(struct nutouch_data *nutouch);
static int NUTOUCH_AutoTune(struct nutouch_data *nutouch);
static int NUTOUCH_SoftwareReset(struct nutouch_data *nutouch);
static int NUTOUCH_SyncWithThreadToReadMsg(struct nutouch_data *nutouch, int boot);
static int NUTOUCH_ResetToBootloader(struct nutouch_data *nutouch);
static int NUTOUCH_GetBootloaderVersion(struct nutouch_data *nutouch);
static int NUTOUCH_UnlockBootloader(struct nutouch_data *nutouch);
static int NUTOUCH_UnlockBootloader_CFG(struct nutouch_data *nutouch);
static int NUTOUCH_GetBootloaderStatus(struct nutouch_data *nutouch, unsigned int state);
static int NUTOUCH_FW_WRITE(struct nutouch_data *nutouch, const u8 *data, unsigned int frame_size);
static int NUTOUCH_FW_UPDATE_FINISHED(struct nutouch_data *nutouch);
static int NUTOUCH_SKIP_INIT(struct nutouch_data *nutouch);
static int NUTOUCH_GET_RESOLUTION(struct nutouch_data *nutouch);
static int NUTOUCH_SET_X_RESOLUTION(struct nutouch_data *nutouch);
static int NUTOUCH_SET_Y_RESOLUTION(struct nutouch_data *nutouch);
static int NUTOUCH_GET_ORIENT(struct nutouch_data *nutouch);
static int NUTOUCH_SET_ORIENT(struct nutouch_data *nutouch);
static int NUTOUCH_BACKUP_RC(struct nutouch_data *nutouch);
static int NUTOUCH_EN_WATER_PROOF(struct nutouch_data *nutouch);
static int NUTOUCH_GET_WATER_PROOF_SATAUS(struct nutouch_data *nutouch);
static int NUTOUCH_GET_FW_VER(struct nutouch_data *nutouch);
static int NUTOUCH_GET_DELTA(struct nutouch_data *nutouch);
static int NUTOUCH_SET_DELTA(struct nutouch_data *nutouch);

#ifdef CONFIG_HAS_EARLYSUSPEND
void nutouch_early_suspend(struct early_suspend *h);
void nutouch_late_resume(struct early_suspend *h);
#endif

static int nutouch_load_fw(struct device *dev, const char *fn);

/*--------------------------------------------------------------*/
/* VFS for I2C_RW */
#define DEV_NAME    		"nutouch-i2c"
#define DEV_IOCTLID		0xD0
#define IOCTL_I2C_SLAVE_FORCE	_IOW(DEV_IOCTLID, 1, int)
#define IOCTL_MAXNR	1

int nutouch_i2c_open(struct inode *inode, struct file *filp)
{
	nutouch_debug(DEBUG_DETAIL, "Nutouch: nutouch_i2c_open\n");
	return 0;
}

int nutouch_i2c_release(struct inode *inode, struct file *filp)
{
	nutouch_debug(DEBUG_DETAIL, "Nutouch: nutouch_i2c_release\n");
	return 0;
}

ssize_t nutouch_i2c_write(struct file *filp, const char *buff, size_t count, loff_t *offp)
{
	int rc = 0;
	char *txbuf;

	if (count > 8192) {
		nutouch_debug(DEBUG_ERROR, "Nutouch: nutouch_i2c_write - count > 8192\n");
		return -EFAULT;
	}

	txbuf = kmalloc(count, GFP_KERNEL);

	if (txbuf == NULL) {
		nutouch_debug(DEBUG_ERROR, "Nutouch: nutouch_i2c_write - kmalloc error\n");
	        return -ENOMEM;
	}

	if (copy_from_user(txbuf, (u8 *) buff, count)) {
		nutouch_debug(DEBUG_ERROR, "Nutouch: nutouch_i2c_write - copy_from_user error\n");
		kfree(txbuf);
		rc = -EFAULT;
		return rc;
	}

	rc = i2c_master_send(nutouch->client, txbuf, count);

	if (rc != count)
		nutouch_debug(DEBUG_ERROR, "Nutouch: nutouch_i2c_write - i2c write error\n");

	return rc;
}

ssize_t nutouch_i2c_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
	int rc = 0;
	char *rxbuf;

	if (count > 8192) {
		nutouch_debug(DEBUG_ERROR, "Nutouch: nutouch_i2c_write - count > 8192\n");
		return -EFAULT;
	}

	rxbuf = kmalloc(count, GFP_KERNEL);

	if (rxbuf == NULL) {
		nutouch_debug(DEBUG_ERROR, "Nutouch: nutouch_i2c_write - kmalloc error\n");
	        return -ENOMEM;
	}

	rc = i2c_master_recv(nutouch->client, rxbuf, count);

	if (rc != count)
		nutouch_debug(DEBUG_ERROR, "Nutouch: nutouch_i2c_read - i2c read error\n");

	if (copy_to_user((u8 *) buff, rxbuf, count)) {
		nutouch_debug(DEBUG_ERROR, "Nutouch: nutouch_i2c_write - copy_to_user error\n");
		kfree(rxbuf);
		rc = -EFAULT;
		return rc;
	}

	kfree(rxbuf);

	return rc;
}

int nutouch_i2c_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	return 0;
	if (_IOC_TYPE(cmd) != DEV_IOCTLID) {
		nutouch_debug(DEBUG_ERROR, "Nutouch: nutouch_i2c_ioctl - IOC_TYPE(cmd) != DEV_IOCTLID\n");
		return -ENOTTY;
	}

	if (_IOC_NR(cmd) > IOCTL_MAXNR) {
		nutouch_debug(DEBUG_ERROR, "Nutouch: nutouch_i2c_ioctl - _IOC_NR(cmd) > IOCTL_MAXNR\n");
		return -ENOTTY;
	}

	switch (cmd) {
		case IOCTL_I2C_SLAVE_FORCE:
			nutouch->client->addr = (int __user *)arg;
			break;
		default:
			return -ENOTTY;
	}
	return 0;
}

struct file_operations nutouch_i2c_rw_fops = {
	.open		= nutouch_i2c_open,
	.write		= nutouch_i2c_write,
	.read		= nutouch_i2c_read,
	.release 	= nutouch_i2c_release,
	.unlocked_ioctl	= nutouch_i2c_ioctl,
};
/* VFS for I2C_RW */
/*--------------------------------------------------------------*/
/* SYSFS for FW Version */
static int nutouch_atoi(const char *a)
{
	int s = 0;

	while(*a >= '0' && *a <= '9')
		s = (s << 3) + (s << 1) + *a++ - '0';
	return s;
}

static ssize_t show_fw_version_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	NUTOUCH_GET_FW_VER(nutouch);
	return sprintf(buf, "0x%02x, 0x%02x, %d\n", 0x02, 0x00, irq_time);
}

static ssize_t show_debug_level(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", debug);
}

static ssize_t show_xy_range(struct device *dev, struct device_attribute *attr, char *buf)
{
	NUTOUCH_GET_RESOLUTION(nutouch);
	return sprintf(buf, "XRange: %d, YRange: %d\n", nutouch->x_range, nutouch->y_range);
}

static ssize_t show_orient(struct device *dev, struct device_attribute *attr, char *buf)
{
	NUTOUCH_GET_ORIENT(nutouch);
	return sprintf(buf, "Orient: %d\n", nutouch->orient);
}

static ssize_t show_water_proof(struct device *dev, struct device_attribute *attr, char *buf)
{
	NUTOUCH_GET_WATER_PROOF_SATAUS(nutouch);
	return sprintf(buf, "en_water_proof: %d\n", nutouch->en_water_proof);
}

static ssize_t show_delta(struct device *dev, struct device_attribute *attr, char *buf)
{
	NUTOUCH_GET_DELTA(nutouch);
	return sprintf(buf, "delta: %d\n", nutouch->delta);
}

static ssize_t store_debug_level(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	debug = nutouch_atoi(buf);
	printk("Nutouch : debug level is %d\n", debug);
        return count;
}

static ssize_t store_nutouch_cmd(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int cmd;
	cmd = nutouch_atoi(buf);
	if (cmd == 1) {
		printk("Nutouch : nutouch command is auto_tune\n");
		NUTOUCH_AutoTune(nutouch);
	} else if (cmd == 2) {
		printk("Nutouch : nutouch command is software_reset\n");
		NUTOUCH_SoftwareReset(nutouch);
	} else if (cmd == 3) {
		printk("Nutouch : nutouch command is sleep\n");
		NUTOUCH_Deepsleep(nutouch);
	} else if (cmd == 4) {
		printk("Nutouch : nutouch command is resume\n");
		NUTOUCH_Resume(nutouch);
	} else if (cmd == 5) {
		printk("Nutouch : nutouch command is backup rc\n");
		NUTOUCH_BACKUP_RC(nutouch);
	} else {
		printk("Nutouch : nutouch command is invalid\n");
	}

        return count;
}

static ssize_t store_nutouch_update_fw(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        int error = 0;

        disable_irq(nutouch->irq);
        error = nutouch_load_fw(dev, NUTOUCH_FW_NAME);

        if (error) {
                nutouch_debug(DEBUG_ERROR, "The firmware update failed(%d)\n", error);
                count = error;
        } else {
                nutouch_debug(DEBUG_ERROR, "The firmware update succeeded\n");

                /* Wait for reset */
                msleep(NUTOUCH_FWRESET_TIME);
        }

        enable_irq(nutouch->irq);

        return count;
}

static ssize_t store_nutouch_update_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int mode;
	mode = nutouch_atoi(buf);

	if (mode == 1) {
		printk("Nutouch : nutouch update fw only\n");
		nutouch->update_mode = 1;
	} else if (mode == 2) {
		printk("Nutouch : nutouch update config only\n");
		nutouch->update_mode = 2;
	} else if (mode == 3) {
		printk("Nutouch : nutouch update fw and config\n");
		nutouch->update_mode = 3;
	} else {
		printk("Nutouch : nutouch command is invalid\n");
		nutouch->update_mode = 0;
	}

        return count;
}

static ssize_t store_orient(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int orient;
	orient = (nutouch_atoi(buf) & 0x07);

	nutouch->orient = orient;

	NUTOUCH_SET_ORIENT(nutouch);

	printk("Nutouch : set nutouch orient to %d\n", nutouch->orient);

        return count;
}

static ssize_t store_x_range(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	nutouch->x_range = nutouch_atoi(buf);

	if (nutouch->x_range <= 0) {
		printk("Nutouch : X Range value invalid\n");
	}

	NUTOUCH_SET_X_RESOLUTION(nutouch);

	printk("Nutouch : set nutouch x range to %d\n", nutouch->x_range);

        return count;
}

static ssize_t store_y_range(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	nutouch->y_range = nutouch_atoi(buf);

	NUTOUCH_SET_Y_RESOLUTION(nutouch);

	printk("Nutouch : set nutouch y range to %d\n", nutouch->y_range);

        return count;
}

static ssize_t store_water_proof(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	nutouch->en_water_proof = (nutouch_atoi(buf) & 0x01);

	NUTOUCH_EN_WATER_PROOF(nutouch);

	printk("Nutouch : set en_water_proof to %d\n", nutouch->en_water_proof);

        return count;
}

static ssize_t store_delta(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	nutouch->delta = nutouch_atoi(buf);

	NUTOUCH_SET_DELTA(nutouch);

	printk("Nutouch : set delta to %d\n", nutouch->delta);

        return count;
}

static DEVICE_ATTR(fw_version, S_IRUGO, show_fw_version_value, NULL);
static DEVICE_ATTR(debug_level, S_IWUSR|S_IRUGO, show_debug_level, store_debug_level);
static DEVICE_ATTR(nutouch_cmd, S_IWUSR, NULL, store_nutouch_cmd);
static DEVICE_ATTR(update_fw, S_IWUSR, NULL, store_nutouch_update_fw);
static DEVICE_ATTR(update_mode, S_IWUSR, NULL, store_nutouch_update_mode);
static DEVICE_ATTR(xy_range, S_IRUGO, show_xy_range, NULL);
static DEVICE_ATTR(orient, S_IWUSR|S_IRUGO, show_orient, store_orient);
static DEVICE_ATTR(set_x_range, S_IWUGO, NULL, store_x_range);
static DEVICE_ATTR(set_y_range, S_IWUGO, NULL, store_y_range);
static DEVICE_ATTR(water_proof, S_IWUSR|S_IWUGO, show_water_proof, store_water_proof);
static DEVICE_ATTR(delta, S_IWUSR|S_IWUGO, show_delta, store_delta);

static struct attribute *nutouch_attributes[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_debug_level.attr,
	&dev_attr_nutouch_cmd.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_update_mode.attr,
	&dev_attr_xy_range.attr,
	&dev_attr_orient.attr,
	&dev_attr_set_x_range.attr,
	&dev_attr_set_y_range.attr,
	&dev_attr_water_proof.attr,
	&dev_attr_delta.attr,
	NULL
};

static struct attribute_group nutouch_attribute_group = {
	.name	= "Nutouch",
	.attrs	= nutouch_attributes
};
/* SYSFS for FW Version */
/*--------------------------------------------------------------*/

static int nutouch_write_block(struct i2c_client *client, u16 addr, u16 length, u8 *value)
{
	int i, j;
	struct {
		__le16  le_addr;
		u8      data[256];
	} i2c_block_transfer;
	struct nutouch_data *nutouch;

	if (length > 256)
		return -EINVAL;

	nutouch = i2c_get_clientdata(client);
	for (i = 0; i < length; i++)
		i2c_block_transfer.data[i] = *value++;
	i2c_block_transfer.le_addr = cpu_to_le16(addr);
	i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);
	if (i == (length + 2)) {
		return length;
	} else {
		for(j=0; j<10; j++) {
			mdelay(10);
			i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);
			if (i == (length + 2)) {
				nutouch_debug(DEBUG_ERROR, "NuTouch: i2c write %d time\n", j+2);
				return length;
			}
		}
		nutouch_debug(DEBUG_ERROR, "Nutouch: i2c write failed\n");
		return -1;
	}
}

static inline int nutouch_read_block_onetime(struct i2c_client *client, u16 length, u8 *value)
{
	struct nutouch_data *nutouch;
	int rc = 0;

	nutouch = i2c_get_clientdata(client);

	rc = i2c_master_recv(nutouch->client, value, length);

	if (rc == length) {
		return length;
	} else {
		mdelay(10);
		nutouch_debug(DEBUG_ERROR, "NuTouch: i2c read failed\n");
		return -1;
	}
}

static inline int nutouch_parser_packet(u8 *buff)
{
	int i, FID;
	short ContactID = 0;

	for (i=0;i<10; i++) {
		nutouch->PointBuf[ContactID].Status = -1;
	}

	for (i=0; i<10; i++) {
		FID = i*6;

		if (buff[FID] == 0xFF) {
			LastUpdateID = i;
			nutouch_debug(DEBUG_DETAIL, "\nFID = %d, 0x%02x\n", FID, buff[FID]);
			if(FID == 0) {
				for (i=0; i<60; i++) {
					if(i==0 || i==10 || i==20 || i==30 || i==40 || i==50)
						nutouch_debug(DEBUG_ERROR, "\nbuff[%02d] : ", i);
					nutouch_debug(DEBUG_ERROR,"0x%02x ", buff[i]);
				}
				goto ERR_PACKET;
			}
			break;
		}

		ContactID = ( (buff[FID + OFFSET_CID]&0xF8) >> 3) - 1;
		if (ContactID < 0 || ContactID > 9)	goto ERR_PACKET;

		nutouch->PointBuf[ContactID].Status = (buff[FID + OFFSET_STATUS] & 0x01);
		nutouch->PointBuf[ContactID].X = ( ((buff[FID + OFFSET_X_H] & 0x0F) << 8) | (buff[FID + OFFSET_X_L] ) );
		nutouch->PointBuf[ContactID].Y = ( ((buff[FID + OFFSET_Y_H] & 0xF0) << 4) | (buff[FID + OFFSET_Y_L] ) );
		nutouch->PointBuf[ContactID].Pressure = buff[FID + OFFSET_PRESSURE];
		nutouch->PointBuf[ContactID].Area = buff[FID + OFFSET_AREA];

		#if 1
		nutouch_debug(DEBUG_DETAIL,"\n[%d] : (%d, %d) (%d) (%d, %d)\n",
			ContactID,
			nutouch->PointBuf[ContactID].X,
			nutouch->PointBuf[ContactID].Y,
			nutouch->PointBuf[ContactID].Status,
			nutouch->PointBuf[ContactID].Pressure,
			nutouch->PointBuf[ContactID].Area);
		#endif
	}

	return 0;

ERR_PACKET:
	printk("ERR PACKET\n");
	return -1;
}

static inline void nutouch_report_coordinate(void)
{
	int i;
	int finger_num = 0;

	for(i=0; i<NUM_FINGERS_SUPPORTED; i++) {
		if (nutouch->PointBuf[i].Status < 0)
			continue;

		if (LastUpdateID < i)
			break;

		if (nutouch->PointBuf[i].Status == 1)
			finger_num++;

		if (finger_num > 0) {
			if(finger_num == 1)	input_report_key(nutouch->input, BTN_TOUCH, 1);
			input_report_abs(nutouch->input, ABS_MT_TRACKING_ID, i);
			input_report_abs(nutouch->input, ABS_MT_POSITION_X, nutouch->PointBuf[i].X);
			input_report_abs(nutouch->input, ABS_MT_POSITION_Y, nutouch->PointBuf[i].Y);
			input_report_abs(nutouch->input, ABS_PRESSURE, nutouch->PointBuf[i].Pressure);
			input_report_abs(nutouch->input, ABS_MT_TOUCH_MAJOR, nutouch->PointBuf[i].Area);
			input_mt_sync(nutouch->input);
		} else {
			//input_report_abs(nutouch->input, ABS_MT_TOUCH_MAJOR, 0);
			input_mt_sync(nutouch->input);
		}

		if (nutouch->PointBuf[i].Status == 0)
			nutouch->PointBuf[i].Status--;
	}
	nutouch_debug(DEBUG_DETAIL,"[NuTouch] : Finger_Number : %d\n", finger_num);
	input_sync(nutouch->input);
}

static inline void nutouch_worker(struct work_struct *work)
{
	struct nutouch_data *nutouch;
	u8 buffer[60] = {0};
	int i;

	nutouch = container_of(work, struct nutouch_data, dwork);
	if(gpio_get_value(nutouch->gp) == 1) {
	//if(gpio_get_value(S3C64XX_GPN(8)) == 1) {
	//if(mt_get_gpio_in(124) == 1) {
		goto next_irq;
	}
	if (nutouch->irq_type == NUTOUCH_HandleTouchMsg) {
		if (nutouch_read_block_onetime(nutouch->client, 60, &buffer[0]) < 0) {
			i2cfail = 1;
			nutouch_debug(DEBUG_ERROR, "NuTouch: nutouch_read_block failed, try again\n");
			printk("NuTouch: nutouch_read_block failed, try again\n");
			if (nutouch_read_block_onetime(nutouch->client, 60, &buffer[0]) < 0) {
				nutouch_debug(DEBUG_ERROR, "NuTouch: nutouch_read_block fail\n");
				goto next_irq;
			}
		}

		if(i2cfail == 1) {
			for(i=0; i<NUM_FINGERS_SUPPORTED; i++) {
				nutouch->PointBuf[i].X = 0;
				nutouch->PointBuf[i].Y = 0;
				nutouch->PointBuf[i].Status = -1;
			}
			i2cfail = 0;
		}

		#if 0
		for (i=0; i<60; i++) {
			if(i==0 || i==10 || i==20 || i==30 || i==40 || i==50)
				nutouch_debug(DEBUG_DETAIL, "\nbuff[%02d] : ", i);
			nutouch_debug(DEBUG_DETAIL, "0x%02x ", buffer[i]);
		}
		#endif


		if (nutouch_parser_packet(buffer) != 0)	goto next_irq;
		nutouch_report_coordinate();
	}

next_irq:
	enable_irq(nutouch->irq);

	return;
}

static irqreturn_t nutouch_irq_handler(int irq, void *_nutouch)
{
	struct nutouch_data *nutouch = _nutouch;

	//nutouch_debug(DEBUG_DETAIL, "\n nutouch_irq_handler\n");
	irq_time++;
	disable_irq_nosync(nutouch->irq);
	queue_work(nutouch->wq, &nutouch->dwork);

	return IRQ_HANDLED;
}

static int nutouch_load_fw(struct device *dev, const char *fn)
{
        const struct firmware *fw = NULL;
        unsigned int frame_size;
        unsigned int pos = 0;
	unsigned int fw_size = 0, cfg_size = 0;
	unsigned int fw_block_len = 0;
        int ret;

	if (nutouch->update_mode == 0) {
		nutouch_debug(DEBUG_ERROR, "NUTOUCH: Invalid update mode %d\n", nutouch->update_mode);
		return 0;
	}

	ret = request_firmware(&fw, fn, dev);
        if (ret) {
                nutouch_debug(DEBUG_ERROR, "NUTOUCH: Unable to open firmware %s\n", fn);
                return ret;
        }

        /* Change to the bootloader mode */
        ret = NUTOUCH_ResetToBootloader(nutouch);
	if (ret) {
		goto out;
	}

	/* Wait NCB3051 reset to bootloader */
	msleep(NUTOUCH_WAIT_BOOTLOADER_TIME);

	/* Get bootloader status and version information */
	ret = NUTOUCH_GetBootloaderVersion(nutouch);
	if (ret) {
		goto out;
	}

	fw_size = ( *(fw->data + TAG_DATA_TYPE + 6) << 24 |
		    *(fw->data + TAG_DATA_TYPE + 7) << 16 |
		    *(fw->data + TAG_DATA_TYPE + 8) <<  8 |
		    *(fw->data + TAG_DATA_TYPE + 9) );

	fw_block_len = TAG_DATA_TYPE + TAG_SIZE + TAG_VER + fw_size;

	cfg_size = ( *(fw->data + fw_block_len + TAG_DATA_TYPE + 6) << 24 |
		     *(fw->data + fw_block_len + TAG_DATA_TYPE + 7) << 16 |
		     *(fw->data + fw_block_len + TAG_DATA_TYPE + 8) <<  8 |
		     *(fw->data + fw_block_len + TAG_DATA_TYPE + 9) );

	printk("=========================================\n");
	printk("NUTOUCH: CB File Size = %d Bytes\n", fw->size);
	printk("NUTOUCH: FW Size      = %d Bytes\n", fw_size);

	printk("NUTOUCH: FW Ver       = %d.%d.%d.%d\n",
			*(fw->data + TAG_DATA_TYPE + TAG_SIZE + 6),
			*(fw->data + TAG_DATA_TYPE + TAG_SIZE + 7),
			*(fw->data + TAG_DATA_TYPE + TAG_SIZE + 8),
			*(fw->data + TAG_DATA_TYPE + TAG_SIZE + 9) );

	printk("NUTOUCH: FW Block Len = %d Bytes\n", fw_block_len);

	printk("NUTOUCH: CFG Size     = %d Bytes\n", cfg_size);

	printk("NUTOUCH: CFG TIME     = %d.%d.%d.%d\n",
			*(fw->data + fw_block_len + TAG_DATA_TYPE + TAG_SIZE + 6),
			*(fw->data + fw_block_len + TAG_DATA_TYPE + TAG_SIZE + 7),
			*(fw->data + fw_block_len + TAG_DATA_TYPE + TAG_SIZE + 8),
			*(fw->data + fw_block_len + TAG_DATA_TYPE + TAG_SIZE + 9) );
	printk("=========================================\n");

	if (nutouch->update_mode == 1)
		goto update_fw;
	else if (nutouch->update_mode == 2)
		goto update_cfg;

update_fw:
	nutouch_debug(DEBUG_DETAIL,"Update FW\n");
        /* Unlock bootloader */
        NUTOUCH_UnlockBootloader(nutouch);
	if (ret) {
		goto out;
	}

	pos = (TAG_DATA_TYPE + TAG_SIZE + TAG_VER);
	/* Write fw to NCB3051 */
	while (pos < fw_block_len) {
                ret = NUTOUCH_GetBootloaderStatus(nutouch, NUTOUCH_WAIT_FRAME_DATA);
                if (ret) {
                        goto out;
		}

                frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

                /* We should add 2 at frame size as the the firmware data is not
                 * included the CRC bytes.
                 */
                frame_size += 2;

                /* Write one frame to NCB3051 */
                NUTOUCH_FW_WRITE(nutouch, fw->data + pos, frame_size);
                ret = NUTOUCH_GetBootloaderStatus(nutouch, NUTOUCH_FRAME_CRC_PASS);
                if (ret) {
                        goto out;
		}

                pos += frame_size;

                nutouch_debug(DEBUG_DETAIL, "NUTOUCH: FW Updated %d bytes / %zd bytes\n", pos, fw_block_len);
        }

	if (nutouch->update_mode == 1)
		goto update_finished;

update_cfg:
	nutouch_debug(DEBUG_DETAIL,"Update CFG\n");
        /* Unlock bootloader CFG */
        NUTOUCH_UnlockBootloader_CFG(nutouch);
	if (ret) {
		goto out;
	}

	/* Write config to NCB3051 */
	pos =  fw_block_len+(TAG_DATA_TYPE + TAG_SIZE + TAG_VER);
	while (pos < fw->size) {
                ret = NUTOUCH_GetBootloaderStatus(nutouch, NUTOUCH_WAIT_FRAME_DATA);
                if (ret) {
                        goto out;
		}

                frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

                /* We should add 2 at frame size as the the firmware data is not
                 * included the CRC bytes.
                 */
                frame_size += 2;

                /* Write one frame to NCB3051 */
                NUTOUCH_FW_WRITE(nutouch, fw->data + pos, frame_size);
                ret = NUTOUCH_GetBootloaderStatus(nutouch, NUTOUCH_FRAME_CRC_PASS);
                if (ret) {
                        goto out;
		}

                pos += frame_size;

                nutouch_debug(DEBUG_DETAIL, "NUTOUCH: CFG Updated %d bytes / %zd bytes\n", (pos-fw_block_len), (fw->size-fw_block_len));
        }

update_finished:
	NUTOUCH_FW_UPDATE_FINISHED(nutouch);
	NUTOUCH_SKIP_INIT(nutouch);
out:
        release_firmware(fw);

        return ret;
}

extern int aka_irq;

/*  boot argument is for determine when to use request_irq or not*/
static int NUTOUCH_SyncWithThreadToReadMsg(struct nutouch_data *nutouch, int boot)
{
	int error = 0, ret = 0;

	nutouch_debug(DEBUG_DETAIL, "NUTOUCH_SyncWithThreadToReadMsg, boot is %d\n", boot);

	if(boot) {
		nutouch_debug(DEBUG_DETAIL, "NuTouch: boot request_irq\n");

		ret = down_interruptible(&nutouch->sema);

		error = request_threaded_irq(nutouch->irq,
				NULL,
				nutouch_irq_handler,
				IRQF_ONESHOT |
				IRQF_TRIGGER_FALLING,
				nutouch->client->name,
				nutouch);

		if (error < 0) {
			nutouch_debug(DEBUG_ERROR, "NuTouch: failed to allocate irq %d\n", nutouch->irq);
			up(&nutouch->sema);
			return -1;
		}
	} else {
		nutouch_debug(DEBUG_DETAIL, "NuTouch: enable_irq\n");
		enable_irq(nutouch->irq);
	}

	return 0;
}

static int NUTOUCH_Initial(struct nutouch_data *nutouch)
{
	nutouch->irq_type = NUTOUCH_ReadResponseMsg_Noaddress;
	if (NUTOUCH_SyncWithThreadToReadMsg(nutouch,1) < 0) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_SyncWithThreadToReadMsg failed\n");
		return -1;
	}

	nutouch->irq_type = NUTOUCH_HandleTouchMsg;

	enable_irq(nutouch->irq);

	return 0;
}

static int NUTOUCH_Deepsleep(struct nutouch_data *nutouch)
{
	u8 cmd_deepsleep[2] = {0x00, 0x00};

	nutouch_debug(DEBUG_DETAIL, "NuTouch: NUTOUCH_Deepsleep\n");

	if (nutouch_write_block(nutouch->client, 0x0108, 2, cmd_deepsleep) < 0)
		return -1;

	return 0;

}

static int NUTOUCH_Resume(struct nutouch_data *nutouch)
{
	u8 cmd_resume[2] = {0x0A,0x0A};

	nutouch_debug(DEBUG_DETAIL, "NuTouch: NUTOUCH_Resume\n");

	if (nutouch_write_block(nutouch->client, 0x0108, 2, cmd_resume) < 0)
		return -1;

	return 0;

}

static int NUTOUCH_AutoTune(struct nutouch_data *nutouch)
{
	u8 cmd_autotune[1] = {0x20};

	nutouch_debug(DEBUG_DETAIL, "NuTouch: NUTOUCH_AutoTune\n");

	if (nutouch_write_block(nutouch->client, 0x0104, 1, cmd_autotune) < 0)
		return -1;

	return 0;

}

static int NUTOUCH_SoftwareReset(struct nutouch_data *nutouch)
{
	u8 cmd_softwarereset[1] = {0x01};

	nutouch_debug(DEBUG_DETAIL, "NuTouch: NUTOUCH_SoftwareReset\n");

	if (nutouch_write_block(nutouch->client, 0x0102, 1, cmd_softwarereset) < 0)
		return -1;

	return 0;

}

static int NUTOUCH_ResetToBootloader(struct nutouch_data *nutouch)
{
	u8 cmd_bootloader[1] = {0xA5};

	nutouch_debug(DEBUG_DETAIL, "NuTouch: NUTOUCH_ResetToBootloader\n");

	if (nutouch_write_block(nutouch->client, 0x0102, 1, cmd_bootloader) < 0)
		return -1;

	return 0;

}

static int NUTOUCH_GetBootloaderVersion(struct nutouch_data *nutouch)
{
        u8 val[5];

ver_recheck:
        if (i2c_master_recv(nutouch->client, &val[0], 5) != 5) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_GetBootloaderVersion error\n");
                return -1;
        }

        if (val[0] != 0xC0) {
		goto ver_recheck;
	}

	printk("NuTouch : Bootloader ver : 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n", val[0], val[1], val[2], val[3], val[4]);

        return 0;

}

static int NUTOUCH_UnlockBootloader(struct nutouch_data *nutouch)
{
	u8 val[2];

        val[0] = 0xDC;
        val[1] = 0xAA;

        if (i2c_master_send(nutouch->client, val, 2) != 2) {
                nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_UnlockBootloader error\n");
                return -1;
        }

        return 0;
}

static int NUTOUCH_UnlockBootloader_CFG(struct nutouch_data *nutouch)
{
	u8 val[2];

        val[0] = 0xDC;
        val[1] = 0xBB;

        if (i2c_master_send(nutouch->client, val, 2) != 2) {
                nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_UnlockBootloader_CFG error\n");
                return -1;
        }

        return 0;
}

static int GetBootloaderStatus_timeout = 0;
static int NUTOUCH_GetBootloaderStatus(struct nutouch_data *nutouch, unsigned int state)
{
        u8 val;
	GetBootloaderStatus_timeout = 0;
recheck:
        if (i2c_master_recv(nutouch->client, &val, 1) != 1) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_GetBootloaderStatus error\n");
                return -1;
        }

	if (GetBootloaderStatus_timeout > 20) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_GetBootloaderStatus timeout %d\n", GetBootloaderStatus_timeout);
		return -1;
	}

        if (val != state) {
		nutouch_debug(DEBUG_DETAIL, "state = 0x%02x, val = 0x%02x\n", state, val);
		msleep(1);
		GetBootloaderStatus_timeout++;
		goto recheck;
	}

        return 0;
}

static int NUTOUCH_FW_WRITE(struct nutouch_data *nutouch,
                             const u8 *data, unsigned int frame_size)
{
        if (i2c_master_send(nutouch->client, data, frame_size) != frame_size) {
                nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_FW_WRITE error\n");
                return -1;
        }

	nutouch_debug(DEBUG_DETAIL, "NUTOUCH_FW_WRITE : frame_size = %d\n", frame_size);

        return 0;
}

static int NUTOUCH_FW_UPDATE_FINISHED(struct nutouch_data *nutouch)
{
	u8 val[12] = {0xAD, 0xAD, 0xAD, 0xAD, 0xAD, 0xAD, 0xAD, 0xAD, 0xAD, 0xAD, 0xAD, 0xAD};

        if (i2c_master_send(nutouch->client, val, 12) != 12) {
                nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_FW_UPDATE_FINISHED error\n");
                return -1;
        }

	nutouch_debug(DEBUG_DETAIL, "NUTOUCH_FW_UPDATE_FINISHED\n");

        return 0;
}

static int NUTOUCH_SKIP_INIT(struct nutouch_data *nutouch)
{
	u8 val[3] = {0x07, 0x01, 0x40};

        if (i2c_master_send(nutouch->client, val, 3) != 3) {
                nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_SKIP_INIT error\n");
                return -1;
        }

	nutouch_debug(DEBUG_DETAIL, "NUTOUCH_SKIP_INIT\n");

        return 0;
}

static int NUTOUCH_GET_RESOLUTION(struct nutouch_data *nutouch)
{
	u8 val[4];

	val[0] = 0x0C;
	val[1] = 0x01;

	if (i2c_master_send(nutouch->client, val, 2) != 2) {
                nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_GET_RESOLUTION write address error, try again\n");
		if (i2c_master_send(nutouch->client, val, 2) != 2) {
			nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_GET_RESOLUTION write address error\n");
                	return -1;
		}
        }

	if (i2c_master_recv(nutouch->client, &val[0], 4) != 4) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_GET_RESOLUTION error\n");
                return -1;
        }

	nutouch->x_range = ((val[1] << 8) | val[0]) + 1;
	nutouch->y_range = ((val[3] << 8) | val[2]) + 1;

	printk("NuTouch: NUTOUCH_GET_RESOLUTION = (%d, %d)\n", nutouch->x_range, nutouch->y_range);
	nutouch_debug(DEBUG_DETAIL, "NUTOUCH_GET_RESOLUTION\n");

        return 0;
}

static int NUTOUCH_SET_X_RESOLUTION(struct nutouch_data *nutouch)
{
	u8 val[4];

	val[0] = 0x0C;
	val[1] = 0x01;
	val[2] = ((nutouch->x_range - 1) & 0x00FF);		// X Range Low Byte
	val[3] = (((nutouch->x_range - 1) & 0xFF00) >> 8);	// X Range High Byte

	if (i2c_master_send(nutouch->client, val, 4) != 4) {
                nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_SET_X_RESOLUTION error, try again\n");
		if (i2c_master_send(nutouch->client, val, 6) != 6) {
			nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_SET_X_RESOLUTION error\n");
                	return -1;
		}
        }

	NUTOUCH_GET_RESOLUTION(nutouch);

        return 0;
}

static int NUTOUCH_SET_Y_RESOLUTION(struct nutouch_data *nutouch)
{
	u8 val[4];

	val[0] = 0x0E;
	val[1] = 0x01;
        val[2] = ((nutouch->y_range - 1) & 0x00FF);		// Y Range Low Byte
	val[3] = (((nutouch->y_range -1) & 0xFF00) >> 8);	// Y Range High Byte

	if (i2c_master_send(nutouch->client, val, 4) != 4) {
                nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_SET_Y_RESOLUTION error, try again\n");
		if (i2c_master_send(nutouch->client, val, 4) != 4) {
			nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_SET_Y_RESOLUTION error\n");
                	return -1;
		}
        }

	NUTOUCH_GET_RESOLUTION(nutouch);

        return 0;
}

static int NUTOUCH_GET_ORIENT(struct nutouch_data *nutouch)
{
	u8 val[2];

	val[0] = 0x0B;
	val[1] = 0x01;

	if (i2c_master_send(nutouch->client, val, 2) != 2) {
                nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_GET_ORIENT write address error\n");
                return -1;
        }

	if (i2c_master_recv(nutouch->client, &val[0], 1) != 1) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_GET_ORIENT error\n");
                return -1;
        }

	nutouch->orient = (val[0] & 0x07);

	nutouch_debug(DEBUG_DETAIL, "NUTOUCH_GET_ORIENT\n");

        return 0;
}

static int NUTOUCH_SET_ORIENT(struct nutouch_data *nutouch)
{
	u8 val[3];

	val[0] = 0x0B;
	val[1] = 0x01;
	val[2] = nutouch->orient;

	if (i2c_master_send(nutouch->client, val, 3) != 3) {
                nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_SET_ORIENT error\n");
                return -1;
        }

	nutouch_debug(DEBUG_DETAIL, "NUTOUCH_SET_ORIENT\n");

        return 0;
}

static int NUTOUCH_BACKUP_RC(struct nutouch_data *nutouch)
{
	u8 val[3];

	val[0] = 0x03;
	val[1] = 0x01;
	val[2] = 0x20;

	if (i2c_master_send(nutouch->client, val, 3) != 3) {
                nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_BACKUP_RC error\n");
                return -1;
        }

	nutouch_debug(DEBUG_DETAIL, "NUTOUCH_BACKUP_RC\n");

        return 0;
}

static int NUTOUCH_EN_WATER_PROOF(struct nutouch_data *nutouch)
{
	u8 val[3];

	val[0] = 0x50;
	val[1] = 0x01;
	val[2] = nutouch->en_water_proof;

	if (i2c_master_send(nutouch->client, val, 3) != 3) {
                nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_EN_WATER_PROOF error\n");
                return -1;
        }

	nutouch_debug(DEBUG_DETAIL, "NUTOUCH_EN_WATER_PROOF\n");

	return 0;
}

static int NUTOUCH_GET_WATER_PROOF_SATAUS(struct nutouch_data *nutouch)
{
	u8 val[2];

	val[0] = 0x50;
	val[1] = 0x01;

	if (i2c_master_send(nutouch->client, val, 2) != 2) {
                nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_GET_WATER_PROOF_STATUS write addr error\n");
                return -1;
        }

	if (i2c_master_recv(nutouch->client, &val[0], 1) != 1) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_GET_WATER_PROOF_STATUS error\n");
                return -1;
        }

	nutouch->en_water_proof = (val[0] & 0x01);

	nutouch_debug(DEBUG_DETAIL, "NuTouch: NUTOUCH_GET_WATER_PROOF_STATUS = %d\n", nutouch->en_water_proof);

	return 0;
}

static int NUTOUCH_SET_DELTA(struct nutouch_data *nutouch)
{
	u8 val[3];

	val[0] = 0x87;
	val[1] = 0x01;
	val[2] = nutouch->delta;

	if (i2c_master_send(nutouch->client, val, 3) != 3) {
                nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_SET_DELTA error\n");
                return -1;
        }

	nutouch_debug(DEBUG_DETAIL, "NUTOUCH_SET_DELTA\n");

	return 0;
}

static int NUTOUCH_GET_DELTA(struct nutouch_data *nutouch)
{
	u8 val[2];

	val[0] = 0x87;
	val[1] = 0x01;

	if (i2c_master_send(nutouch->client, val, 2) != 2) {
                nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_GET_DELTA write addr error\n");
                return -1;
        }

	if (i2c_master_recv(nutouch->client, &val[0], 1) != 1) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_GET_DELTA error\n");
                return -1;
        }

	nutouch->delta = val[0];

	nutouch_debug(DEBUG_DETAIL, "NuTouch: NUTOUCH_GET_DELTA = %d\n", nutouch->delta);

	return 0;
}

static int NUTOUCH_GET_FW_VER(struct nutouch_data *nutouch)
{
	int i = 0;
	u8 val[16];

	val[0] = 0x00;
	val[1] = 0x04;

	if (i2c_master_send(nutouch->client, val, 2) != 2) {
                nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_GET_FW_VER write addr error\n");
                return -1;
        }

	if (i2c_master_recv(nutouch->client, &val[0], 16) != 16) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_GET_FW_VER error\n");
                return -1;
        }

	printk("NuTouch: FW ver = ");
	for (i=0; i<16; i++) {
		nutouch->fw_ver[i] = val[i];
		if((i==7)||(i==15) )
			printk("0x%02x\n", nutouch->fw_ver[i]);
		else
			printk("0x%02x, ", nutouch->fw_ver[i]);
	}

	printk("\n");

	return 0;
}

static int nutouch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct input_dev *input;
	int error, i;
	dev_t dev;

	nutouch_debug(DEBUG_DETAIL, "NuTouch: nutouch_probe\n");

	if (client == NULL) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: client == NULL\n");
		return	-EINVAL;
	} else if (client->adapter == NULL) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: client->adapter == NULL\n");
		return	-EINVAL;
	} else if (&client->dev == NULL) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: client->dev == NULL\n");
		return	-EINVAL;
	} else if (&client->adapter->dev == NULL) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: client->adapter->dev == NULL\n");
		return	-EINVAL;
	} else if (id == NULL) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: id == NULL\n");
		return	-EINVAL;
	}

	nutouch = kzalloc(sizeof(struct nutouch_data), GFP_KERNEL);
	if (nutouch == NULL) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: insufficient memory\n");
		error = -ENOMEM;
		goto err_nutouch_alloc;
	}

	input = input_allocate_device();
	if (!input) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: error allocating input device\n");
		error = -ENOMEM;
		goto err_input_dev_alloc;
	}

	input->name = "NuTouch";
	nutouch->input  = input;
	nutouch->client = client;
	nutouch->irq = client->irq;
	nutouch->gp = irq_to_gpio(client->irq);
//aka	nutouch->irq = aka_irq;

	nutouch->wq = create_singlethread_workqueue("nutouch_wq");

	INIT_WORK(&nutouch->dwork, nutouch_worker);

	set_bit(BTN_TOUCH, input->keybit);

	//set_bit(INPUT_PROP_DIRECT, input->propbit);

	set_bit(EV_ABS, input->evbit);
	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);

	set_bit(ABS_MT_TRACKING_ID, input->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, input->absbit);
	set_bit(ABS_MT_TOUCH_MINOR, input->absbit);
	set_bit(ABS_MT_POSITION_X, input->absbit);
	set_bit(ABS_MT_POSITION_Y, input->absbit);
	set_bit(ABS_PRESSURE, input->absbit);
	set_bit(ABS_X, input->absbit);
	set_bit(ABS_Y, input->absbit);

	/* single touch */
	input_set_abs_params(input, ABS_X, X_MIN, X_MAX, 0, 0);
	input_set_abs_params(input, ABS_Y, Y_MIN, Y_MAX, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, NUTOUCH_MAX_REPORTED_PRESSURE, 0, 0);
	input_abs_set_res(input, ABS_X, X_MAX);
	input_abs_set_res(input, ABS_Y, X_MAX);

	/* multiple touch */
	input_set_abs_params(input, ABS_MT_POSITION_X, X_MIN, X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, Y_MIN, Y_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, NUTOUCH_MAX_TOUCH_SIZE, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, NUTOUCH_MAX_TOUCH_SIZE, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, NUM_FINGERS_SUPPORTED,0, 0);

	i2c_set_clientdata(client, nutouch);

	error = input_register_device(nutouch->input);
	if (error < 0) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: Failed to register input device\n");
		goto err_register_device;
	}

	sema_init(&nutouch->sema,1);

	for(i=0;i<NUM_FINGERS_SUPPORTED;i++) {
		nutouch->PointBuf[i].X = 0;
		nutouch->PointBuf[i].Y = 0;
		nutouch->PointBuf[i].Status = -1;
	}

	error = NUTOUCH_Initial(nutouch);
	if (error < 0) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_Initial failed\n");
		goto err_irq;
	}

	#ifdef CONFIG_HAS_EARLYSUSPEND
	nutouch->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	nutouch->early_suspend.suspend = nutouch_early_suspend;
	nutouch->early_suspend.resume = nutouch_late_resume;
	register_early_suspend(&nutouch->early_suspend);
	#endif

	/* VFS For I2C R/W */
	nutouch->dev_cdevp = NULL;
	error = alloc_chrdev_region(&dev, 0, 1, DEV_NAME);
    	if (error < 0) {
        	nutouch_debug(DEBUG_ERROR, "NuTouch: can't alloc chrdev\n");
        	goto err_irq;
    	}
    	nutouch->dev_major = MAJOR(dev);
    	nutouch->dev_minor = MINOR(dev);
    	nutouch_debug(DEBUG_DETAIL, "NuTouch: register chrdev(%d,%d)\n", nutouch->dev_major, nutouch->dev_minor);

    	nutouch->dev_cdevp = kmalloc(sizeof(struct cdev), GFP_KERNEL);
    	if (nutouch->dev_cdevp == NULL) {
        	nutouch_debug(DEBUG_ERROR, "NuTouch: kmalloc failed\n");
        	goto err_cdev;
		}
    	cdev_init(nutouch->dev_cdevp, &nutouch_i2c_rw_fops);
    	nutouch->dev_cdevp->owner = THIS_MODULE;
    	error = cdev_add(nutouch->dev_cdevp, MKDEV(nutouch->dev_major, nutouch->dev_minor), 1);
    	if (error < 0) {
        	nutouch_debug(DEBUG_ERROR, "NuTouch: add chr dev failed\n");
        	goto err_cdev;
    	}

		nutouch->dev_classp = class_create(THIS_MODULE, "nutouch_class");
		device_create( nutouch->dev_classp, NULL, MKDEV(nutouch->dev_major, nutouch->dev_minor), NULL, DEV_NAME);

	/* SYSFS */
	if (sysfs_create_group(&nutouch->client->dev.kobj, &nutouch_attribute_group))
		nutouch_debug(DEBUG_ERROR, "NuTouch: sysfs create group error\n");

	return 0;

err_cdev:
    	if (nutouch->dev_cdevp) {
        	kfree(nutouch->dev_cdevp);
          	nutouch->dev_cdevp = NULL;
	}
err_irq:
	if (nutouch->irq)
		free_irq(nutouch->irq, nutouch);
err_register_device:
	input_free_device(input);
err_input_dev_alloc:
	kfree(nutouch);
err_nutouch_alloc:
	return error;
}

static int nutouch_remove(struct i2c_client *client)
{
	struct nutouch_data *nutouch;
	dev_t dev;

	nutouch = i2c_get_clientdata(client);

	if (nutouch != NULL) {
		if (nutouch->irq)
			free_irq(nutouch->irq, nutouch);

		//cancel_delayed_work_sync(&nutouch->dwork);
		input_unregister_device(nutouch->input);
	}
	kfree(nutouch);
	i2c_set_clientdata(client, NULL);

	/* VFS For I2C R/W */
	dev = MKDEV(nutouch->dev_major, nutouch->dev_minor);

	if (nutouch->dev_cdevp) {
		cdev_del(nutouch->dev_cdevp);
		kfree(nutouch->dev_cdevp);
	}

	unregister_chrdev_region(dev, 1);

	device_destroy(nutouch->dev_classp, MKDEV(nutouch->dev_major, nutouch->dev_minor));
	class_destroy(nutouch->dev_classp);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void nutouch_early_suspend(struct early_suspend *h)
{
	struct nutouch_data *data = container_of(h, struct nutouch_data, early_suspend);
	nutouch_suspend = data;
	suspend_resume = 0;
	nutouch_debug(DEBUG_ERROR, "NuTouch: nutouch_early_suspend\n");

	disable_irq(data->irq);

	if (NUTOUCH_Deepsleep(data) == 0) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: Deepsleep OK!\n");
	}
}

void nutouch_late_resume(struct early_suspend *h)
{
	struct nutouch_data *data = container_of(h, struct nutouch_data, early_suspend);
	int i;
	nutouch_suspend = data;
	suspend_resume = 1;
	nutouch_debug(DEBUG_ERROR, "NuTouch: nutouch_late_resume\n");

	for(i=0;i<NUM_FINGERS_SUPPORTED;i++) {
		data->PointBuf[i].X = 0;
		data->PointBuf[i].Y = 0;
		data->PointBuf[i].Status = -1;
	}
	/*
	system will still resume back if i2c error,
	but it will be blocked if resume configs are not written to touch successfully
	*/
	if (NUTOUCH_Resume(data) == 0) {
		nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_Resume OK!\n");
	} else {
		nutouch_debug(DEBUG_ERROR, "NuTouch: NUTOUCH_Resume Fail\n");
	}

	enable_irq(data->irq);
}
#endif
#ifndef CONFIG_HAS_EARLYSUSPEND
static int nutouch_suspend1(struct i2c_client *client, pm_message_t mesg)
{
	nutouch_debug(DEBUG_ERROR, "NuTouch: nutouch_suspend\n");
	return 0;
}

static int nutouch_resume(struct i2c_client *client)
{
	nutouch_debug(DEBUG_ERROR, "NuTouch: nutouch_resume\n");
	return 0;
}
#endif

static const struct i2c_device_id nutouch_id[] =
{
	{ "NuTouch", 0 },
	{},
};

static struct i2c_driver nutouch_driver =
{
	.probe      = nutouch_probe,
	.remove     = nutouch_remove,
	#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend    = nutouch_suspend1,
	.resume     = nutouch_resume,
	#endif
	.id_table   = nutouch_id,
	.driver     = {
		.name   = "NuTouch",
	},
};

//aka
module_i2c_driver(nutouch_driver);

static int __init nutouch_init(void)
{
	int ret;
	ret = i2c_add_driver(&nutouch_driver);

	return ret;
//	struct i2c_board_info info = {
//		I2C_BOARD_INFO("NuTouch", 0x55),
//		.irq = VV_GPIO_IRQBASE + VV_NGPIO_SCORE + VV_NGPIO_NCORE + 12,
//	};

//	i2c_register_board_info(6, &info, 1);
//	return 0;
}

static void __exit nutouch_exit(void)
{
	i2c_del_driver(&nutouch_driver);
}

module_init(nutouch_init);
module_exit(nutouch_exit);

//aka

MODULE_DESCRIPTION("Driver for NuTouch Touchscreen Controller");
MODULE_LICENSE("GPL");
