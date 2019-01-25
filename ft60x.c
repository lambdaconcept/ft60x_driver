/*
 *
 * Copyright (C) 2019 ramtin@lambdaconcept.com
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/cdev.h>

/* Define these values to match your devices */
#define USB_FT60X_VENDOR_ID	0x0403
#define USB_FT601_PRODUCT_ID	0x601f
#define USB_FT600_PRODUCT_ID	0x601e

#define FT60X_EP_PAIR_MAX	4
#define FT60X_DEVICE_NAME	"ft60x"
#define FT60X_MAX_MINORS	256

/* table of devices that work with this driver */
static const struct usb_device_id ft60x_table[] = {
	{ USB_DEVICE(USB_FT60X_VENDOR_ID, USB_FT600_PRODUCT_ID) },
	{ USB_DEVICE(USB_FT60X_VENDOR_ID, USB_FT601_PRODUCT_ID) },
	{ }			/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, ft60x_table);

/* Get a minor  */
#ifdef CONFIG_USB_DYNAMIC_MINORS
#define USB_FT60X_MINOR_BASE	0
#else
#define USB_FT60X_MINOR_BASE	192
#endif

/* our private defines. if this grows any larger, use your own .h file */
#define MAX_TRANSFER		2048
#define WRITES_IN_FLIGHT	8

struct ft60x_config {
	/* Device Descriptor */
	u16	VendorID;
	u16	ProductID;
	/* String Descriptors */
	u8	StringDescriptors[128];
	/* Configuration Descriptor */
	u8	Reserved;
	u8	PowerAttributes;
	u16	PowerConsumption;
	/* Data Transfer Configuration */
	u8	Reserved2;
	u8	FIFOClock;
	u8	FIFOMode;
	u8	ChannelConfig;
	/* Optional Feature Support */
	u16	OptionalFeatureSupport;
	u8	BatteryChargingGPIOConfig;
	u8	FlashEEPROMDetection;	/* Read-only */
	/* MSIO and GPIO Configuration */
	u32	MSIO_Control;
	u32	GPIO_Control;
} __attribute__((packed));

static LIST_HEAD(ft60x_ctrl_list);
static LIST_HEAD(ft60x_data_list);
static struct usb_driver ft60x_driver;

static int ft60x_open(struct inode *inode, struct file *file);
static int ft60x_release(struct inode *inode, struct file *file);

static int ft60x_data_open(struct inode *inode, struct file *file);
static int ft60x_data_release(struct inode *inode, struct file *file);
static ssize_t ft60x_data_read(struct file *file, char *user_buffer,
			       size_t count, loff_t * ppos);
static ssize_t ft60x_data_write(struct file *file, const char *user_buffer,
				size_t count, loff_t * ppos);
static __poll_t ft60x_data_poll(struct file *file, poll_table *wait);

static struct class *class = NULL;	/* The device-driver class struct pointer */
static dev_t devt;		/* Global variable for the first device number */
static char ft60x_minors[FT60X_MAX_MINORS / FT60X_EP_PAIR_MAX];

static const struct file_operations ft60x_ctrl_fops = {
	.owner =	THIS_MODULE,
	.read =		NULL,
	.write =	NULL,
	.open =		ft60x_open,
	.release =	ft60x_release,
	.flush =	NULL,
	.poll =		NULL,
	.unlocked_ioctl = NULL,
	.llseek =	noop_llseek,
};

static const struct file_operations ft60x_data_fops = {
	.owner =	THIS_MODULE,
	.read =		ft60x_data_read,
	.write =	ft60x_data_write,
	.open =		ft60x_data_open,
	.release =	ft60x_data_release,
	.flush =	NULL,
	.poll =		ft60x_data_poll,
	.unlocked_ioctl = NULL,
	.llseek =	noop_llseek,
};

static void ft60x_print_usb_log(struct usb_interface *intf,
				const struct usb_device_id *id)
{
	int i;
	struct usb_host_interface *interface;
	struct usb_endpoint_descriptor *endpoint;
	if (!intf || !id) {
		printk(KERN_ERR "Invalid arg\n");
		return;
	}
	interface = intf->cur_altsetting;
	printk(KERN_INFO "FT60x i/f %d now probed: (%04X:%04X)\n",
	       interface->desc.bInterfaceNumber, id->idVendor, id->idProduct);
	printk(KERN_INFO "ID->bNumEndpoints: %02X\n",
	       interface->desc.bNumEndpoints);
	printk(KERN_INFO "ID->bInterfaceClass: %02X\n",
	       interface->desc.bInterfaceClass);

	for (i = 0; i < interface->desc.bNumEndpoints; i++) {
		endpoint = &interface->endpoint[i].desc;
		printk(KERN_INFO "ED[%d]->bEndpointAddress: 0x%02X\n",
		       i, endpoint->bEndpointAddress);
		printk(KERN_INFO "ED[%d]->bmAttributes: 0x%02X\n",
		       i, endpoint->bmAttributes);
		printk(KERN_INFO "ED[%d]->wMaxPacketSize: 0x%04X (%d)\n",
		       i, endpoint->wMaxPacketSize, endpoint->wMaxPacketSize);
	}
}

struct ft60x_ctrlreq {
	u32	idx;
	u8	ep;
	u8	cmd;
	u8	unk1;
	u8	unk2;
	u16	len;
	u16	unk3;
	u32	unk4;
	u32	unk5;
} __attribute__((packed));

struct ft60x_irqresp {
	u32	idx;
	u8	ep;
	u8	cmd;
	u16	len;
	u32	unk1;
} __attribute__((packed));

struct ft60x_node_s {
	unsigned char		*bulk_in_buffer;
	size_t			len;		/* total length of data in the buffer */
	size_t			used;		/* how much has been consumed already */
	struct ft60x_node_s	*next;
	struct ft60x_node_s	*prev;
};

struct ft60x_ring_s {
	struct ft60x_node_s	first;
	struct ft60x_node_s	*wr;
	struct ft60x_node_s	*rd;
	size_t ep_size;				/* node buffer size */
};

struct ft60x_endpoint {
	int			used;			/* endpoint use depends on ft60x current config */
	struct ft60x_ring_s	ring;			/* Our RING structure to add data in */
	struct ft60x_data_dev	*data_dev;		/* pointer to parent data_dev */
	struct cdev		cdev;
	atomic_t		opened;			/* restrict access to only one user */
	struct semaphore	limit_sem;		/* limiting the number of writes in progress */
	size_t			bulk_in_size;
	struct urb		*bulk_in_urb;		/* the urb to read data with */
	wait_queue_head_t	bulk_in_wait;		/* to wait for an ongoing read */
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
	__u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint */
	bool			busy_write;		/* for the writing poll */
	bool			ongoing_read;		/* a read is going on */
	bool			waiting_notif;		/* waiting for data available notification  */
	int			errors;			/* the last request tanked */
	spinlock_t		err_lock;		/* lock for errors */
	struct mutex		io_rd_mutex;		/* synchronize I/O with disconnect */
};

struct ft60x_data_dev {
	struct usb_device	*device;
	struct usb_interface	*interface;
	struct list_head	data_list;
	struct ft60x_ctrl_dev	*ctrl_dev;		/* the corresponding control device */
	u32			devnum;
	struct usb_device	*udev;			/* the usb device for this device */
	struct usb_anchor	submitted;		/* in case we need to retract our submissions */
	int			major;			/* major of the data chardev */
	int			baseminor;		/* first minor in the data ep group */
	struct kref		kref;
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */
	struct ft60x_endpoint	ep_pair[FT60X_EP_PAIR_MAX];	/* 4 different endpoint max */
};

struct ft60x_ctrl_dev {
	struct usb_device	*device;
	struct usb_interface	*interface;
	struct list_head	ctrl_list;
	struct ft60x_data_dev	*data_dev;		/* the corresponding data device */
	u32			devnum;
	struct usb_device	*udev;			/* the usb device for this device */
	__u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint */
	struct urb		*int_in_urb;		/* the urb to read data with */
	size_t			int_in_size;		/* the size of the receive buffer */
	signed char		*int_in_buffer;		/* the buffer to receive int data */
	dma_addr_t		int_in_data_dma;	/* the dma buffer to receive int data */
	struct kref		kref;
	struct ft60x_config	ft60x_cfg;		/* store the chip current config */
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */
	struct ft60x_ctrlreq	ctrlreq;
};

static int ft60x_do_data_read_io(struct ft60x_endpoint *ep, size_t count);

static int ft60x_ring_add_node(struct ft60x_ring_s *r)
{
	struct ft60x_node_s *tmp;

	if (!r || !r->wr) {
		return -EINVAL;
	}

	/* use current node if available */
	if (!r->wr->len) {
		return 0;
	}

	/* use next node is available */
	if (!r->wr->next->len) {
		r->wr = r->wr->next;
		return 0;
	}

	/* add a new node */
	tmp = kmalloc(sizeof(struct ft60x_node_s), GFP_NOIO);
	if (!tmp) {
		return -ENOMEM;
	}
	memset(tmp, 0, sizeof(struct ft60x_node_s));

	tmp->bulk_in_buffer = kmalloc(r->ep_size, GFP_KERNEL);
	if (!tmp->bulk_in_buffer) {
		kfree(tmp);
		return -ENOMEM;
	}

	tmp->prev = r->wr;
	tmp->next = r->wr->next;
	r->wr->next->prev = tmp;
	r->wr->next = tmp;
	r->wr = tmp;

	return 0;
}

static int ft60x_ring_init(struct ft60x_ring_s *r, size_t ep_size)
{
	memset(r, 0, sizeof(struct ft60x_ring_s));

	r->first.next = &r->first;
	r->first.prev = &r->first;
	r->wr = &r->first;
	r->rd = &r->first;
	r->ep_size = ep_size;

	/* allocate the first buffer in the ring */
	r->first.bulk_in_buffer = kmalloc(r->ep_size, GFP_KERNEL);
	if (!r->first.bulk_in_buffer) {
		return -ENOMEM;
	}

	return 0;
}

static ssize_t ft60x_ring_read(struct ft60x_ring_s *r, char *user_buffer,
			       size_t len)
{
	char *pnt = user_buffer;
	size_t rlen = len;

	printk(KERN_INFO "IN_FUNCTION %s\n", __func__);

	if (!r || !r->rd) {
		return -EINVAL;
	}

	while (r->rd->len) {
		printk(KERN_INFO "Ring read: using node %p, %ld, %ld\n", r->rd,
		       r->rd->len, r->rd->used);
		if (r->rd->len < rlen) {
			/* 
			 * we are asked for more data than we have on the
			 * current node.
			 * copy any available data, empty the node and go to
			 * the next node until the ring is exhausted.
			 */
			printk(KERN_INFO "copy_to_user 1\n");
			if (copy_to_user(pnt,
					 r->rd->bulk_in_buffer + r->rd->used,
					 r->rd->len))
				return -EFAULT;

			pnt += r->rd->len;
			rlen -= r->rd->len;
			r->rd->len = 0;
			r->rd->used = 0;

			if (r->rd == r->wr)
				return len - rlen;
		} else {
			/* 
			 * enough data.
			 * copy the requested amount.
			 */
			printk(KERN_INFO "copy_to_user 2\n");
			if (copy_to_user(pnt,
					 r->rd->bulk_in_buffer + r->rd->used,
					 rlen))
				return -EFAULT;

			pnt += rlen;
			r->rd->used += rlen;
			r->rd->len -= rlen;

			/* 
			 * if this node is now empty, reset it for later use
			 * and jump to the next.
			 */
			if (r->rd->len == 0) {
				r->rd->used = 0;
				r->rd = r->rd->next;
			}

			return len;
		}
		r->rd = r->rd->next;
	}
	return len - rlen;
}

static int ft60x_ring_has_data(struct ft60x_ring_s *r)
{
	printk(KERN_INFO "IN_FUNCTION %s\n", __func__);
	printk("return %d\n", (r->rd->len > 0));

	return (r->rd->len > 0);
}

void ft60x_ring_free(struct ft60x_ring_s *r)
{
	struct ft60x_node_s *p = &r->first;
	struct ft60x_node_s *o;

	if (!p->next)
		return;

	do {
		o = p->next;
		p->next = o->next;
		if (o->bulk_in_buffer) {
			kfree(o->bulk_in_buffer);
			o->bulk_in_buffer = NULL;
		}
		if (o != p) {
			kfree(o);
		}
	} while (o != p);

	r->wr = NULL;
	r->rd = NULL;
}

static struct ft60x_endpoint *ft60x_find_endpoint(struct ft60x_data_dev
						  *data_dev, u8 addr)
{
	int i;
	struct ft60x_endpoint *ep;

	for (i = 0; i < FT60X_EP_PAIR_MAX; i++) {
		ep = &data_dev->ep_pair[i];
		printk("FIND: %02x, %02x, %02x", addr, ep->bulk_in_endpointAddr,
		       ep->bulk_out_endpointAddr);
		if ((ep->bulk_in_endpointAddr == addr) ||
		    (ep->bulk_out_endpointAddr == addr)) {
			return ep;
		}
	}

	return NULL;
}

static int ft60x_endpoint_has_notification(struct ft60x_endpoint *ep)
{
	int notif;

	/* Get the "Notification Message Feature" which tells if we receive
	   an interruption via ctrl endpoint when message are available.
	   This bit exist for each endpoint.
	 */
	notif = (ep->data_dev->ctrl_dev->ft60x_cfg.OptionalFeatureSupport >> ep->bulk_out_endpointAddr) & 1;

	return notif;
}

static void ft60x_ctrlreq_callback(struct urb *urb)
{
	printk(KERN_INFO "%s called\n", __func__);

	switch (urb->status) {
	case 0:
		/* nothing to do */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		printk(KERN_INFO "%s - urb shutting down with status: %d\n",
		       __func__, urb->status);
		break;
	default:
		printk(KERN_INFO "%s - nonzero urb status received: %d\n",
		       __func__, urb->status);
		break;
	}
}

static void ft60x_int_callback(struct urb *urb)
{
	int retval;
	struct ft60x_ctrl_dev *ctrl_dev;
	struct ft60x_endpoint *ep;
	struct ft60x_irqresp *resp;

	printk(KERN_INFO "%s called\n", __func__);
	ctrl_dev = urb->context;

	switch (urb->status) {
	case 0:
		/*
		 * we received a notification about the IN data pipes that have
		 * pending data, submit a read request on the matching endpoint
		 */
		resp = (struct ft60x_irqresp *)ctrl_dev->int_in_buffer;
		printk(KERN_INFO
		       "struct ft60x_irqresp\nidx: %d, ep: 0x%02x, len: %d\n",
		       resp->idx, resp->ep, resp->len);

		ep = ft60x_find_endpoint(ctrl_dev->data_dev, resp->ep);
		if (ep) {
			/* 
			 * this is just submitting a read request but not waiting,
			 * fine to do it in callback context
			 */
			ft60x_do_data_read_io(ep, resp->len);

			spin_lock_irq(&ep->err_lock);
			ep->waiting_notif = 0;
			spin_unlock_irq(&ep->err_lock);

			wake_up_interruptible(&ep->bulk_in_wait);

		} else {
			dev_err(&ctrl_dev->interface->dev,
				"%s - could not find notified endpoint: 0x%02x\n",
				__func__, resp->ep);
			goto exit;
		}
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		printk(KERN_INFO "%s - urb shutting down with status: %d\n",
		       __func__, urb->status);
		return;
	default:
		printk(KERN_INFO "%s - nonzero urb status received: %d\n",
		       __func__, urb->status);
		goto exit;
	}

exit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval) {
		printk(KERN_ERR "%s - usb_submit_urb failed with result: %d\n",
		       __func__, retval);
	}
}

static void ft60x_print_config(struct ft60x_config *cfg)
{
	printk(KERN_INFO "vendorid: %04x\n", cfg->VendorID);
	printk(KERN_INFO "productid: %04x\n", cfg->ProductID);
	printk(KERN_INFO "fifoclock: %02x\n", cfg->FIFOClock);
	printk(KERN_INFO "fifomode: %02x\n", cfg->FIFOMode);
	printk(KERN_INFO "channel: %02x\n", cfg->ChannelConfig);
	printk(KERN_INFO "optional: %04x\n", cfg->OptionalFeatureSupport);
	printk(KERN_INFO "msiocontrol: %04x\n", cfg->MSIO_Control);
}

static int ft60x_set_config(struct ft60x_ctrl_dev *ctrl_dev,
			    struct ft60x_config *ft60x_cfg)
{
	struct ft60x_config *cfg = NULL;
	int ret;

	if (!ctrl_dev || !ft60x_cfg) {
		ret = -EINVAL;
		goto exit;
	}

	/* Make sure we are not sending the same thing because it'd disconnect us */
	if (!memcmp(ft60x_cfg, &ctrl_dev->ft60x_cfg,
			       sizeof(struct ft60x_config))) {
		return 0;
	}

	/* Changing configuration */
	cfg = kmalloc(sizeof(struct ft60x_config), GFP_NOIO);
	if (!cfg) {
		ret = -ENOMEM;
		goto exit;
	}

	memcpy(cfg, ft60x_cfg, sizeof(struct ft60x_config));

	ret = usb_control_msg(ctrl_dev->udev, usb_sndctrlpipe(ctrl_dev->udev, 0),
			      0xcf, USB_TYPE_VENDOR | USB_DIR_OUT, 0, 0, cfg,
			      sizeof(struct ft60x_config), USB_CTRL_SET_TIMEOUT);

	printk("RETURN FROM CONFIG\n");

	if (ret < 0) {
		ret = -EIO;
		goto exit;
	}
exit:
	if (cfg)
		kfree(cfg);

	return ret;
}

static int ft60x_get_config(struct ft60x_ctrl_dev *ctrl_dev)
{
	struct ft60x_config *cfg = NULL;
	int retval = 0;
	int ret;

	if (!ctrl_dev) {
		retval = -EINVAL;
		goto exit;
	}

	cfg = kmalloc(sizeof(struct ft60x_config), GFP_NOIO);
	if (!cfg) {
		retval = -ENOMEM;
		goto exit;
	}

	ret = usb_control_msg(ctrl_dev->udev, usb_rcvctrlpipe(ctrl_dev->udev, 0),
			      0xcf, USB_TYPE_VENDOR | USB_DIR_IN, 1, 0, cfg,
			      sizeof(struct ft60x_config), USB_CTRL_SET_TIMEOUT);
	if (ret < 0) {
		retval = -EIO;
		goto exit;
	}

	if (ret <= sizeof(struct ft60x_config)) {
		memcpy(&ctrl_dev->ft60x_cfg, cfg, ret);
	}

exit:
	if (cfg) {
		kfree(cfg);
	}
	return retval;
}

static int ft60x_get_unknown(struct ft60x_ctrl_dev *ctrl_dev)
{
	int retval = 0;
	int ret;

	unsigned int *val = NULL;

	if (!ctrl_dev) {
		retval = -EINVAL;
		goto exit;
	}

	val = kmalloc(sizeof(unsigned int), GFP_NOIO);
	if (!val) {
		retval = -ENOMEM;
		goto exit;
	}

	ret = usb_control_msg(ctrl_dev->udev, usb_rcvctrlpipe(ctrl_dev->udev, 0),
			      0xf1, USB_TYPE_VENDOR | USB_DIR_IN, 0, 0, val, 4,
			      USB_CTRL_SET_TIMEOUT);
	if (ret < 0) {
		retval = -EIO;
		printk(KERN_ERR "GOT ERROR1: %d\n", retval);
		goto exit;
	}

exit:
	if (val) {
		kfree(val);
	}
	return retval;
}

static void ft60x_delete_ctrl(struct kref *kref)
{
	struct ft60x_ctrl_dev *ctrl_dev = container_of(kref, struct ft60x_ctrl_dev, kref);

	if (ctrl_dev->int_in_buffer) {
		usb_free_coherent(ctrl_dev->udev,
				  ctrl_dev->int_in_size,
				  ctrl_dev->int_in_buffer,
				  ctrl_dev->int_in_data_dma);
	}
	// XXX free int_in_urb

	kfree(ctrl_dev);
}

static void ft60x_delete_data(struct kref *kref)
{
	struct ft60x_data_dev *data_dev = container_of(kref, struct ft60x_data_dev, kref);
	int i;

	printk(KERN_INFO "%s called\n", __func__);

	for (i = 0; i < FT60X_EP_PAIR_MAX; i++) {
		if (data_dev->ep_pair[i].bulk_in_urb) {
			usb_free_urb(data_dev->ep_pair[i].bulk_in_urb);
		}
	}

	kfree(data_dev);
}

static int ft60x_allocate_ctrl_interface(struct usb_interface *interface,
					 const struct usb_device_id *id)
{
	struct usb_device *device;
	struct usb_host_interface *host_interface;
	struct ft60x_ctrl_dev *ctrl_dev = NULL;
	struct ft60x_data_dev *data_dev = NULL;
	struct usb_endpoint_descriptor *endpoint;

	int i;
	int maxp, pipe;
	int retval;

	if (!interface && !id)
		return -EINVAL;

	host_interface = interface->cur_altsetting;
	device = interface_to_usbdev(interface);

	/* allocate memory for our device state and initialize it */

	ctrl_dev = kzalloc(sizeof(struct ft60x_ctrl_dev), GFP_KERNEL);
	if (!ctrl_dev)
		return -ENOMEM;

	kref_init(&ctrl_dev->kref);

	/*
	   We place the device in a list, in order to match the data interface later on
	 */

	list_add_tail(&(ctrl_dev->ctrl_list), &(ft60x_ctrl_list));

	ctrl_dev->device = device;
	ctrl_dev->udev = usb_get_dev(device);
	ctrl_dev->interface = interface;
	ctrl_dev->devnum = device->devnum;

	mutex_init(&ctrl_dev->io_mutex);

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, ctrl_dev);

	/* 
	   Attempt to find the data interface for this ctrl intf
	   Normally, the Control interface is enumerated before data intf
	   So this function should never find anything, but in case in the future
	   the data interface shows up before, we can still match.
	 */

	list_for_each_entry(data_dev, &ft60x_data_list, data_list) {
		if (ctrl_dev->devnum == data_dev->devnum) {
			printk(KERN_INFO "Found data: %d\n", data_dev->devnum);
			ctrl_dev->data_dev = data_dev;
			data_dev->ctrl_dev = ctrl_dev;
			break;
		}
	}

	for (i = 0; i < host_interface->desc.bNumEndpoints; ++i) {
		endpoint = &host_interface->endpoint[i].desc;

		if (usb_endpoint_is_int_in(endpoint)) {

			/* we found a interrupt in endpoint */

			ctrl_dev->int_in_size = usb_endpoint_maxp(endpoint);
			printk("ctrl_dev->int_in_size: %ld\n",
			       ctrl_dev->int_in_size);
			ctrl_dev->int_in_buffer = usb_alloc_coherent(
					ctrl_dev->udev,
					ctrl_dev->int_in_size,
					GFP_ATOMIC,
					&ctrl_dev->int_in_data_dma);

			if (!ctrl_dev->int_in_buffer) {
				retval = -ENOMEM;
				goto error;
			}

			ctrl_dev->int_in_urb = usb_alloc_urb(0, GFP_KERNEL);
			if (!ctrl_dev->int_in_urb) {
				retval = -ENOMEM;
				goto error;
			}

			/* get a handle to the interrupt data pipe */

			pipe = usb_rcvintpipe(ctrl_dev->udev,
					      endpoint->bEndpointAddress);

			maxp = usb_maxpacket(ctrl_dev->udev, pipe,
					     usb_pipeout(pipe));

			usb_fill_int_urb(ctrl_dev->int_in_urb, ctrl_dev->udev,
					 pipe, ctrl_dev->int_in_buffer, maxp,
					 ft60x_int_callback, ctrl_dev,
					 endpoint->bInterval);

			ctrl_dev->int_in_urb->transfer_dma = ctrl_dev->int_in_data_dma;
			ctrl_dev->int_in_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

			/* register our interrupt URB with the USB system */

			if (usb_submit_urb(ctrl_dev->int_in_urb, GFP_KERNEL)) {
				retval = -EIO;
				goto error;
			}
		}

		if (usb_endpoint_is_bulk_out(endpoint)) {

			/* we found a bulk out endpoint */
			ctrl_dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
		}
	}

	/* 
	 * The chip stores its current configuration on a internal EEPROM.
	 * Get it now
	 */

	retval = ft60x_get_config(ctrl_dev);
	if (retval)
		goto error;
	ft60x_print_config(&ctrl_dev->ft60x_cfg);

	retval = ft60x_get_unknown(ctrl_dev);
	if (retval)
		goto error;

	return retval;
error:
	printk(KERN_INFO "ERRROOOOOOOOOR\n");

	list_del(&ctrl_dev->ctrl_list);

	/* this frees allocated memory */
	kref_put(&ctrl_dev->kref, ft60x_delete_ctrl);

	return retval;
}

static int ft60x_add_device(struct ft60x_data_dev *data_dev, int minor, int idx)
{
	struct device *device = NULL;
	dev_t newdevt;
	int ret = 0;

	/* Create and add a new device */
	newdevt = MKDEV(data_dev->major, data_dev->baseminor + idx);
	printk(KERN_INFO "device_create: %d %d %d\n", data_dev->major, minor,
	       idx);
	printk(KERN_INFO "drvdata: %p\n", &data_dev->ep_pair[idx]);

	device = device_create(class, NULL, newdevt, NULL,
			       "ft60x%d%c", minor, idx + 'a');
	if (IS_ERR(device)) {
		ret = PTR_ERR(device);
		dev_err(&data_dev->interface->dev,
			"Error %d creating device for port %u\n",
			ret, data_dev->baseminor + idx);
		goto error_dev;
	}

	cdev_init(&data_dev->ep_pair[idx].cdev, &ft60x_data_fops);
	ret = cdev_add(&data_dev->ep_pair[idx].cdev, newdevt, 1);
	if (ret < 0) {
		dev_err(&data_dev->interface->dev,
			"Error %d adding cdev for port %u\n",
			ret, data_dev->baseminor + idx);
		goto error_cdev;
	}

	printk(KERN_INFO "device class created correctly\n");
	return 0;

error_cdev:
	device_destroy(class, newdevt);
error_dev:
	return ret;
}

static int ft60x_register_baseminor(void)
{
	int i;

	/* Reserve a group of minors */
	for (i = 0; i < sizeof(ft60x_minors); i++) {
		if (!ft60x_minors[i]) {
			ft60x_minors[i] = 1;
			return i * FT60X_EP_PAIR_MAX;
		}
	}

	return -1;
}

static void ft60x_unregister_baseminor(int baseminor)
{
	int idx = baseminor / FT60X_EP_PAIR_MAX;

	if (idx < sizeof(ft60x_minors))
		ft60x_minors[idx] = 0;
}

static int ft60x_allocate_data_interface(struct usb_interface *interface,
					 const struct usb_device_id *id)
{
	struct usb_device *device;
	struct usb_host_interface *host_interface;
	struct ft60x_data_dev *data_dev = NULL;
	struct ft60x_ctrl_dev *ctrl_dev = NULL;
	struct usb_endpoint_descriptor *endpoint;
	struct ft60x_endpoint *p;
	int retval;
	int i;
	int ep_pair_num;

	host_interface = interface->cur_altsetting;
	device = interface_to_usbdev(interface);

	/* allocate memory for our device state and initialize it */
	data_dev = kzalloc(sizeof(struct ft60x_data_dev), GFP_KERNEL);
	if (!data_dev)
		return -ENOMEM;

	kref_init(&data_dev->kref);
	mutex_init(&data_dev->io_mutex);
	init_usb_anchor(&data_dev->submitted);

	list_add_tail(&(data_dev->data_list), &(ft60x_data_list));

	for (i = 0; i < FT60X_EP_PAIR_MAX; i++) {
		p = &data_dev->ep_pair[i];

		p->data_dev = data_dev;
		atomic_set(&p->opened, 0);
		sema_init(&p->limit_sem, WRITES_IN_FLIGHT);
		spin_lock_init(&p->err_lock);
		init_waitqueue_head(&p->bulk_in_wait);
		mutex_init(&p->io_rd_mutex);
	}

	data_dev->device = device;
	data_dev->udev = usb_get_dev(device);
	data_dev->interface = interface;
	data_dev->devnum = device->devnum;
	data_dev->major = MAJOR(devt);

	data_dev->baseminor = ft60x_register_baseminor();
	if (data_dev->baseminor < 0) {
		printk("ERR Alloc minor\n");
		goto error;
	}

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, data_dev);

	/* Attempt to find the ctrl interface of this data intf */
	list_for_each_entry(ctrl_dev, &ft60x_ctrl_list, ctrl_list) {
		if (ctrl_dev->devnum == data_dev->devnum) {
			printk(KERN_INFO "Found ctrl: %d\n", data_dev->devnum);
			ctrl_dev->data_dev = data_dev;
			data_dev->ctrl_dev = ctrl_dev;
			break;
		}
	}

	for (i = 0; i < host_interface->desc.bNumEndpoints; i++) {

		endpoint = &host_interface->endpoint[i].desc;
		ep_pair_num = i >> 1;
		p = &data_dev->ep_pair[ep_pair_num];
		p->used = 1;

		if (usb_endpoint_is_bulk_out(endpoint)) {
			printk("BULK OUT %d\n", i);
			/* we found a bulk out endpoint */
			p->bulk_out_endpointAddr = endpoint->bEndpointAddress;
		}

		if (usb_endpoint_is_bulk_in(endpoint)) {

			printk("BULK IN %d\n", i);
			/* we found a bulk in endpoint */
			retval = ft60x_add_device(data_dev,
					 data_dev->ctrl_dev->interface->minor,
					 ep_pair_num);
			if (retval < 0) {
				goto error;
			}
			printk(KERN_INFO "%d %d\n",
			       data_dev->ctrl_dev->interface->minor,
			       ep_pair_num);

			p->bulk_in_size = usb_endpoint_maxp(endpoint);
			p->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			p->bulk_in_urb = usb_alloc_urb(0, GFP_KERNEL);
			if (!p->bulk_in_urb) {
				retval = -ENOMEM;
				goto error;
			}

		}
	}

	return 0;
error:
	list_del(&data_dev->data_list);

	/* this frees allocated memory */
	kref_put(&data_dev->kref, ft60x_delete_data);

	return retval;
}

static int ft60x_data_open(struct inode *inode, struct file *file)
{
	int ret;
	unsigned int mj = imajor(inode);
	unsigned int mn = iminor(inode);
	struct ft60x_endpoint *ep;
	int opened;

	printk(KERN_INFO "%s called\n", __func__);
	printk(KERN_INFO "major: %i, minor: %i\n", mj, mn);

	ep = container_of(inode->i_cdev, struct ft60x_endpoint, cdev);

	/* Restrict access to only one instance */
	opened = atomic_cmpxchg(&ep->opened, 0, 1);
	if (opened) {
		return -EBUSY;
	}

	/* Initialize RX Ring Structure */
	ret = ft60x_ring_init(&ep->ring, ep->bulk_in_size);
	if (ret < 0) {
		goto error;
	}

	/* increment our usage count for the device */
	kref_get(&ep->data_dev->kref);

	/* save our object in the file's private structure */
	file->private_data = ep;
	printk(KERN_INFO "prvdata: %p\n", file->private_data);

	return 0;

error:
	atomic_dec(&ep->opened);
	return ret;
}

static int ft60x_data_release(struct inode *inode, struct file *file)
{
	struct ft60x_endpoint *ep;

	printk(KERN_INFO "%s called\n", __func__);

	ep = file->private_data;
	if (ep == NULL)
		return -ENODEV;

	file->private_data = NULL;

	/* Free Ring struture */
	ft60x_ring_free(&ep->ring);	// XXX maybe use after free if file is closed while ongoing read

	/* decrement the count on our device */
	kref_put(&ep->data_dev->kref, ft60x_delete_data);

	atomic_dec(&ep->opened);

	return 0;
}

static __poll_t ft60x_data_poll(struct file *file, poll_table *wait)
{
	__poll_t ret = 0;
	struct ft60x_endpoint *ep;

	printk(KERN_INFO "%s called\n", __func__);

	ep = file->private_data;
	if (ep == NULL)
		return -ENODEV;

	poll_wait(file, &ep->bulk_in_wait, wait);
	// XXX add poll_wait for bulk_out_wait ??

	// XXX need mutex / lock ??
	if (!ep->data_dev->interface) {
		return EPOLLHUP;
	}

	// XXX need mutex / lock ??
	if (ft60x_ring_has_data(&ep->ring)) {
		ret |= EPOLLIN | EPOLLRDNORM;
	}

	// XXX use down_timeout
	if (!down_trylock(&ep->limit_sem)) {
		/*
		 * we acquired the semaphore,
		 * write in flight limit no reached yet, release sem
		 */
		up(&ep->limit_sem);
		ret |= EPOLLOUT;
	}

	return ret;
}

static int ft60x_send_ctrlreq(struct ft60x_ctrl_dev *ctrl_dev,
			      bool asynchronous)
{
	int len = 0;
	u8 *buf = NULL;
	int retval = 0;
	struct urb *urb = NULL;

	printk(KERN_INFO "IN_FUNCTION %s\n", __func__);

	if (!ctrl_dev) {
		retval = -EINVAL;
		goto exit;
	}

	buf = kmalloc(sizeof(struct ft60x_ctrlreq), GFP_KERNEL);
	if (!buf) {
		retval = -ENOMEM;
		goto exit;
	}

	memcpy(buf, &ctrl_dev->ctrlreq, sizeof(struct ft60x_ctrlreq));

	if (asynchronous) {
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			retval = -ENOMEM;
			goto exit;
		}

		usb_fill_bulk_urb(urb, ctrl_dev->udev,
				  usb_sndbulkpipe(ctrl_dev->udev,
						  ctrl_dev->bulk_out_endpointAddr),
				  buf, sizeof(struct ft60x_ctrlreq),
				  ft60x_ctrlreq_callback, NULL);
		/// XXX anchor urb

		retval = usb_submit_urb(urb, GFP_KERNEL);
		if (retval) {
			dev_err(&ctrl_dev->interface->dev,
				"%s - failed submitting ctrlreq urb, error %d\n",
				__func__, retval);
			goto exit;
		}
	} else {
		retval = usb_bulk_msg(ctrl_dev->udev,
				      usb_sndbulkpipe(ctrl_dev->udev,
						      ctrl_dev->bulk_out_endpointAddr),
				      buf, sizeof(struct ft60x_ctrlreq), &len,
				      1000);
		if (retval) {
			printk("%s: command bulk message failed: error %d\n",
			       __func__, retval);
			goto exit;
		}
	}

exit:
	if (urb) {
		usb_free_urb(urb);
	}
	if (buf) {
		kfree(buf);
	}

	printk(KERN_INFO "EXIT from %s, async == %d\n", __func__, asynchronous);

	return retval;
}

static int ft60x_send_cmdread(struct ft60x_endpoint *ep, size_t reqlen,
			      bool asynchronous)
{
	int retval = 0;
	struct ft60x_ctrlreq *req;

	printk(KERN_INFO "IN_FUNCTION %s\n", __func__);

	if (!ep) {
		retval = -ENODEV;
		goto exit;
	}
	// XXX mutex this, maybe use data_dev io_mutex ??
	req = &ep->data_dev->ctrl_dev->ctrlreq;
	req->idx++;
	req->ep = ep->bulk_in_endpointAddr;
	req->cmd = 1; // XXX 1
	req->len = reqlen;

	retval = ft60x_send_ctrlreq(ep->data_dev->ctrl_dev, asynchronous);

	// dev->sent_cmd_read = 1; // XXX
	printk(KERN_INFO "EXIT %s\n", __func__);
exit:
	return retval;
}

static void ft60x_data_read_bulk_callback(struct urb *urb)
{
	struct ft60x_endpoint *ep;

	printk(KERN_INFO "IN_FUNCTION %s\n", __func__);

	ep = urb->context;
	spin_lock(&ep->err_lock);

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			dev_err(&ep->data_dev->interface->dev,
				"%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);

		ep->errors = urb->status;
	} else {
		/* update read length */
		printk("update ring: %p: len: %d\n", ep->ring.wr,
		       urb->actual_length);
		ep->ring.wr->len = urb->actual_length;
	}

	ep->ongoing_read = 0;
	spin_unlock(&ep->err_lock);

	wake_up_interruptible(&ep->bulk_in_wait);
}

static int ft60x_do_data_read_io(struct ft60x_endpoint *ep, size_t count)
{
	int ret;
	int rv = 0;
	int notif;
	int readlen;

	printk(KERN_INFO "IN_FUNCTION %s\n", __func__);

	notif = ft60x_endpoint_has_notification(ep);
	printk("NOTIF: %d\n", notif);

	/*
	 * when in notification mode,
	 * we should not ask more than what is available
	 */
	if (notif) {
		readlen = min(ep->bulk_in_size, count);
	} else {
		/* ignore count, read a full packet */
		readlen = ep->bulk_in_size;
	}

	/*
	 * we must inform the chip of how much data bytes we want to read.
	 * when in notification mode, we are called from callback context
	 * and thus sending the command asynchronously.
	 */
	if (notif) {
		ft60x_send_cmdread(ep, readlen, 1);
	} else {
		ft60x_send_cmdread(ep, readlen, 0);
	}

	/*
	 * in notification mode we may be called without the char dev opened,
	 */
	/* get the next empty node from the ring buffer */
	// XXX
	// if (!atomic_read(&ep->opened)) {
	// }
	ret = ft60x_ring_add_node(&ep->ring);
	if (ret < 0) {
		printk("cannot add node to ring\n");
		return ret;
	}

	printk(KERN_INFO "Ring add node: using node %p\n", ep->ring.wr);

	usb_fill_bulk_urb(ep->bulk_in_urb,
			  ep->data_dev->udev,
			  usb_rcvbulkpipe(ep->data_dev->udev,
					  ep->bulk_in_endpointAddr),
			  ep->ring.wr->bulk_in_buffer, readlen,
			  ft60x_data_read_bulk_callback, ep);

	printk(KERN_INFO "fill bulk in urb done %s\n", __func__);

	/* tell everybody to leave the URB alone */
	spin_lock_irq(&ep->err_lock);
	ep->ongoing_read = 1;
	spin_unlock_irq(&ep->err_lock);

	/* do it */
	rv = usb_submit_urb(ep->bulk_in_urb, GFP_KERNEL);

	printk(KERN_INFO "submitted ! %s\n", __func__);

	if (rv < 0) {
		dev_err(&ep->data_dev->interface->dev,
			"%s - failed submitting read urb, error %d\n",
			__func__, rv);
		rv = (rv == -ENOMEM) ? rv : -EIO;
		spin_lock_irq(&ep->err_lock);
		ep->ongoing_read = 0;
		spin_unlock_irq(&ep->err_lock);
	}
	return rv;
}

static ssize_t ft60x_data_read(struct file *file, char *user_buffer,
			       size_t count, loff_t * ppos)
{
	struct ft60x_endpoint *ep;
	int rv;
	bool ongoing_io;
	bool waiting;

	printk(KERN_INFO "IN_FUNCTION %s\n", __func__);

	ep = file->private_data;

	/* if we cannot read at all, return EOF */
	if (!ep || !ep->bulk_in_urb || !count)
		return 0;

	/* no concurrent readers */
	rv = mutex_lock_interruptible(&ep->io_rd_mutex);
	if (rv < 0)
		return rv;

	// XXX ? /* this lock makes sure we don't submit URBs to gone devices */
	// XXX ? mutex_lock(&ep->data_dev->io_mutex);
	/* disconnect() was called */
	if (!ep->data_dev->interface) {
		rv = -ENODEV;
		goto exit;
	}
	// ep->done_reading = 0; // XXX ?

	/* if IO is under way, we must not touch things */
retry:
	spin_lock_irq(&ep->err_lock);
	ongoing_io = ep->ongoing_read;
	waiting = ep->waiting_notif;
	spin_unlock_irq(&ep->err_lock);

	if (ongoing_io || waiting) {
		/* nonblocking IO shall not wait */
		if (file->f_flags & O_NONBLOCK) {
			rv = -EAGAIN;
			goto exit;
		}
		/*
		 * IO may take forever
		 * hence wait in an interruptible state
		 */
		rv = wait_event_interruptible(ep->bulk_in_wait,
					      (!ep->ongoing_read && !ep->waiting_notif));
		if (rv < 0)
			goto exit;
	}

	/* errors must be reported */
	rv = ep->errors;
	if (rv < 0) {
		/* any error is reported once */
		ep->errors = 0;
		/* to preserve notifications about reset */
		rv = (rv == -EPIPE) ? rv : -EIO;
		/* report it */
		goto exit;
	}

	/*
	 * if the buffer is filled we may satisfy the read
	 * else we need to start IO
	 */
	if (ft60x_ring_has_data(&ep->ring)) {
		/* data is available */
		printk(KERN_INFO "data is available\n");
		rv = ft60x_ring_read(&ep->ring, user_buffer, count);
		if (rv < 0)
			goto exit;

		/*
		 * if we are asked for more than we have,
		 * and we are not in notification mode,
		 * we start IO but don't wait
		 */
		if ((rv < count) && !ft60x_endpoint_has_notification(ep)) {
			ft60x_do_data_read_io(ep, count - rv);
		}
	} else {
		/*
		 * no data in the buffer,
		 * start IO only when not in notification mode
		 */
		printk(KERN_INFO "no data in the buffer\n");
		if (!ft60x_endpoint_has_notification(ep)) {
			rv = ft60x_do_data_read_io(ep, count);
			if (rv < 0)
				goto exit;
		} else {
			spin_lock_irq(&ep->err_lock);
			ep->waiting_notif = 1;
			spin_unlock_irq(&ep->err_lock);
		}
		goto retry;
	}
exit:
	// ep->done_reading = 1; // XXX ?
	mutex_unlock(&ep->io_rd_mutex);
	return rv;
}

static void ft60x_data_write_bulk_callback(struct urb *urb)
{
	struct ft60x_endpoint *ep;

	printk(KERN_INFO "IN_FUNCTION %s\n", __func__);

	ep = urb->context;

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		      urb->status == -ECONNRESET || urb->status == -ESHUTDOWN))
			dev_err(&ep->data_dev->interface->dev,
				"%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);

		spin_lock(&ep->err_lock);
		ep->busy_write = 0;
		ep->errors = urb->status;
		spin_unlock(&ep->err_lock);
		// wake_up_interruptible(&dev->bulk_out_wait);
	}

	/* free up our allocated buffer */
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			  urb->transfer_buffer, urb->transfer_dma);
	up(&ep->limit_sem);
}

static ssize_t ft60x_data_write(struct file *file, const char *user_buffer,
				size_t count, loff_t * ppos)
{
	struct ft60x_endpoint *ep;
	int retval = 0;
	struct urb *urb = NULL;
	char *buf = NULL;
	size_t writesize = min(count, (size_t)MAX_TRANSFER);

	printk(KERN_INFO "IN_FUNCTION %s\n", __func__);

	ep = file->private_data;

	/* verify that we actually have some data to write */
	if (count == 0)
		goto exit;

	/*
	 * limit the number of URBs in flight to stop a user from using up all
	 * RAM
	 */
	if (!(file->f_flags & O_NONBLOCK)) {
		if (down_interruptible(&ep->limit_sem)) {
			retval = -ERESTARTSYS;
			goto exit;
		}
	} else {
		if (down_trylock(&ep->limit_sem)) {
			retval = -EAGAIN;
			goto exit;
		}
	}

	spin_lock_irq(&ep->err_lock);
	retval = ep->errors;
	if (retval < 0) {
		/* any error is reported once */
		ep->errors = 0;
		/* to preserve notifications a bout reset */
		retval = (retval == -EPIPE) ? retval : -EIO;
	}
	spin_unlock_irq(&ep->err_lock);
	if (retval < 0)
		goto error;

	printk(KERN_INFO "alloc %s\n", __func__);

	/* create a urb, and a buffer for it, and copy the data to the urb */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		retval = -ENOMEM;
		goto error;
	}

	buf = usb_alloc_coherent(ep->data_dev->udev, writesize, GFP_KERNEL,
				 &urb->transfer_dma);
	if (!buf) {
		retval = -ENOMEM;
		goto error;
	}

	if (copy_from_user(buf, user_buffer, writesize)) {
		retval = -EFAULT;
		goto error;
	}

	/* this lock makes sure we don't submit URBs to gone devices */
	mutex_lock(&ep->data_dev->io_mutex);
	if (!ep->data_dev->interface) {	/* disconnect() was called */
		mutex_unlock(&ep->data_dev->io_mutex);
		retval = -ENODEV;
		goto error;
	}

	printk(KERN_INFO "urb init %s\n", __func__);

	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, ep->data_dev->udev,
			  usb_sndbulkpipe(ep->data_dev->udev,
					  ep->bulk_out_endpointAddr),
			  buf, writesize, ft60x_data_write_bulk_callback, ep);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &ep->data_dev->submitted);

	printk(KERN_INFO "send urb %s\n", __func__);

	/* send the data out the bulk port */
	retval = usb_submit_urb(urb, GFP_KERNEL);
	mutex_unlock(&ep->data_dev->io_mutex);
	if (retval) {
		dev_err(&ep->data_dev->interface->dev,
			"%s - failed submitting write urb, error %d\n",
			__func__, retval);
		goto error_unanchor;
	}
	spin_lock_irq(&ep->err_lock);
	ep->busy_write = 1;
	spin_unlock_irq(&ep->err_lock);

	/*
	 * release our reference to this urb, the USB core will eventually free
	 * it entirely
	 */
	usb_free_urb(urb);

	return writesize;

error_unanchor:
	usb_unanchor_urb(urb);
error:
	if (urb) {
		usb_free_coherent(ep->data_dev->udev, writesize, buf,
				  urb->transfer_dma);
		usb_free_urb(urb);
	}
	up(&ep->limit_sem);

exit:
	printk(KERN_INFO "error %s\n", __func__);
	return retval;
}

static int ft60x_open(struct inode *inode, struct file *file)
{
	struct usb_interface *interface;
	int subminor;
	int retval = 0;
	struct ft60x_ctrl_dev *ctrl_dev;
	struct ft60x_config cstm;

	printk(KERN_INFO "%s called\n", __func__);
	subminor = iminor(inode);

	interface = usb_find_interface(&ft60x_driver, subminor);
	if (!interface) {
		pr_err("%s - error, can't find device for minor %d\n",
		       __func__, subminor);
		retval = -ENODEV;
		goto exit;
	}

	ctrl_dev = usb_get_intfdata(interface);
	if (!ctrl_dev) {
		retval = -ENODEV;
		goto exit;
	}

	retval = usb_autopm_get_interface(interface);
	if (retval)
		goto exit;

	/* increment our usage count for the device */
	kref_get(&ctrl_dev->kref);

	/* save our object in the file's private structure */
	file->private_data = ctrl_dev;

	/*
	   memcpy(&cstm, &ctrl_dev->ft60x_cfg, sizeof(struct ft60x_config));
	   cstm.FIFOMode=1;
	   cstm.ChannelConfig=0;

	   retval = ft60x_set_config(ctrl_dev, &cstm);

	   ft60x_print_config(&ctrl_dev->ft60x_cfg);
	 */
	return retval;
exit:
	printk(KERN_INFO "ERROR OPEN\n");
	return retval;
}

static int ft60x_release(struct inode *inode, struct file *file)
{
	struct ft60x_ctrl_dev *ctrl_dev;

	ctrl_dev = file->private_data;
	if (ctrl_dev == NULL)
		return -ENODEV;

	/* decrement the count on our device */
	kref_put(&ctrl_dev->kref, ft60x_delete_ctrl);
	return 0;
}

/*                                                                           
 * usb class driver info in order to get a minor number from the usb core,   
 * and to have the device registered with the driver core                    
 */
static struct usb_class_driver ft60x_class = {
	.name = "ft60x%d",
	.fops = &ft60x_ctrl_fops,
	.minor_base = USB_FT60X_MINOR_BASE,
};

static int ft60x_probe(struct usb_interface *interface,
		       const struct usb_device_id *id)
{
	struct usb_device *device;
	struct usb_host_interface *host_interface;

	int retval;

	host_interface = interface->cur_altsetting;

	printk(KERN_INFO "%s called\n", __func__);
	device = interface_to_usbdev(interface);

	/*
	 * At this point, each FT60x is probed twice.
	 * Once for an interface with a bulkout EP for ctrl and INT EP for data notification
	 * Another time for an interface with 1 to 4 EP depending on the configuration of FT60x
	 * Both have the same devnum
	 */

	ft60x_print_usb_log(interface, id);
	printk(KERN_INFO "devnum: %d interface: %d\n", device->devnum,
	       host_interface->desc.bInterfaceNumber);

	/* First interface is the control/int interface, the other one is for data */

	if (host_interface->desc.bInterfaceNumber == 0) {

		retval = ft60x_allocate_ctrl_interface(interface, id);
		if (retval)
			goto error;
		retval = usb_register_dev(interface, &ft60x_class);
	} else {

		retval = ft60x_allocate_data_interface(interface, id);
	}

	if (retval)
		goto error;

	return 0;
error:
	return retval;
}

static int ft60x_suspend(struct usb_interface *intf, pm_message_t message)
{

	printk(KERN_INFO "%s called\n", __func__);
	return 0;
}

static int ft60x_resume(struct usb_interface *intf)
{
	printk(KERN_INFO "%s called\n", __func__);
	return 0;
}

static int ft60x_pre_reset(struct usb_interface *intf)
{
	printk(KERN_INFO "%s called\n", __func__);
	return 0;
}

static int ft60x_post_reset(struct usb_interface *intf)
{
	printk(KERN_INFO "%s called\n", __func__);
	return 0;
}

static void ft60x_disconnect(struct usb_interface *interface)
{
	struct usb_host_interface *host_interface;
	struct ft60x_ctrl_dev *ctrl_dev;
	struct ft60x_data_dev *data_dev;
	int i;

	host_interface = interface->cur_altsetting;

	printk(KERN_INFO "%s called\n", __func__);

	if (host_interface->desc.bInterfaceNumber == 0) {
		/* ctrl interface */

		ctrl_dev = usb_get_intfdata(interface);
		usb_set_intfdata(interface, NULL);

		/* Give back our minor */
		usb_deregister_dev(interface, &ft60x_class);

		/* Prevent more I/O from starting */
		mutex_lock(&ctrl_dev->io_mutex);
		ctrl_dev->interface = NULL;
		mutex_unlock(&ctrl_dev->io_mutex);

		list_del(&ctrl_dev->ctrl_list);

		kref_put(&ctrl_dev->kref, ft60x_delete_ctrl);

	} else {
		/* data interface */

		data_dev = usb_get_intfdata(interface);

		usb_set_intfdata(interface, NULL);

		for (i = 0; i < FT60X_EP_PAIR_MAX; i++) {

			if (data_dev->ep_pair[i].used) {
				cdev_del(&data_dev->ep_pair[i].cdev);
				device_destroy(class,
					       MKDEV(data_dev->major,
						     data_dev->baseminor + i));
			}
		}

		/* Prevent more I/O from starting */
		mutex_lock(&data_dev->io_mutex);
		data_dev->interface = NULL;
		mutex_unlock(&data_dev->io_mutex);

		ft60x_unregister_baseminor(data_dev->baseminor);
		list_del(&data_dev->data_list);

		kref_put(&data_dev->kref, ft60x_delete_data);
	}

	// XXX
	// usb_kill_anchored_urbs(&dev->submitted);

}

static struct usb_driver ft60x_driver = {
	.name =		"ft60x",
	.probe =	ft60x_probe,
	.disconnect =	ft60x_disconnect,
	.suspend =	ft60x_suspend,
	.resume =	ft60x_resume,
	.pre_reset =	ft60x_pre_reset,
	.post_reset =	ft60x_post_reset,
	.id_table =	ft60x_table,
	.supports_autosuspend = 1,
};

static int __init ft60x_init(void)
{
	int rc;

	class = class_create(THIS_MODULE, FT60X_DEVICE_NAME);
	if (IS_ERR(class)) {
		pr_err("couldn't create class\n");
		return PTR_ERR(class);
	}

	rc = alloc_chrdev_region(&devt, 0, FT60X_MAX_MINORS, FT60X_DEVICE_NAME);
	if (rc) {
		pr_err("failed to allocate char dev region\n");
		goto err_region;
	}

	rc = usb_register(&ft60x_driver);
	if (rc) {
		pr_err("failed to register usb driver\n");
		goto err_usb;
	}

	return rc;

err_usb:
	unregister_chrdev_region(devt, FT60X_MAX_MINORS);
err_region:
	class_destroy(class);
	class = NULL;

	return rc;
}

static void __exit ft60x_exit(void)
{
	usb_deregister(&ft60x_driver);
	unregister_chrdev_region(devt, FT60X_MAX_MINORS);
	class_destroy(class);
	class = NULL;
}

module_init(ft60x_init);
module_exit(ft60x_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Ramtin Amin <ramtin@lambdaconcept.com>");
MODULE_DESCRIPTION("FT60x Driver");
