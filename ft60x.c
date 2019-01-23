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
	{USB_DEVICE(USB_FT60X_VENDOR_ID, USB_FT600_PRODUCT_ID)},
	{USB_DEVICE(USB_FT60X_VENDOR_ID, USB_FT601_PRODUCT_ID)},
	{}			/* Terminating entry */
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
	u16 	VendorID;
	u16 	ProductID;
	/* String Descriptors */
	u8 	StringDescriptors[128];
	/* Configuration Descriptor */
	u8 	Reserved;
	u8 	PowerAttributes;
	u16 	PowerConsumption;
	/* Data Transfer Configuration */
	u8 	Reserved2;
	u8 	FIFOClock;
	u8 	FIFOMode;
	u8 	ChannelConfig;
	/* Optional Feature Support */
	u16 	OptionalFeatureSupport;
	u8 	BatteryChargingGPIOConfig;
	u8 	FlashEEPROMDetection;	/* Read-only */
	/* MSIO and GPIO Configuration */
	u32 	MSIO_Control;
	u32 	GPIO_Control;
} __attribute__ ((packed));

static LIST_HEAD(ft60x_ctrl_list);
static LIST_HEAD(ft60x_data_list);
static struct usb_driver ft60x_driver;

static int ft60x_open(struct inode *inode, struct file *file);
static int ft60x_release(struct inode *inode, struct file *file);

static int ft60x_data_open(struct inode *inode, struct file *file);
static int ft60x_data_release(struct inode *inode, struct file *file);
static ssize_t ft60x_data_write(struct file *file, const char *user_buffer,
				size_t count, loff_t * ppos);

static struct class* class = NULL; // The device-driver class struct pointer
static dev_t devt; // Global variable for the first device number
static char ft60x_minors[FT60X_MAX_MINORS / FT60X_EP_PAIR_MAX];

static const struct file_operations ft60x_fops = {
	.owner =        THIS_MODULE,
	.read =         NULL,
	.write =        NULL,
	.open =         ft60x_open,
	.release =      ft60x_release,
	.flush =        NULL,
	.poll =         NULL,
	.unlocked_ioctl =       NULL,
	.llseek =       noop_llseek,
};

static const struct file_operations ft60x_data_fops = {
	.owner =        THIS_MODULE,
	.read =         NULL,
	.write =        ft60x_data_write,
	.open =         ft60x_data_open,
	.release =      ft60x_data_release,
	.flush =        NULL,
	.poll =         NULL,
	.unlocked_ioctl =       NULL,
	.llseek =       noop_llseek,
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

struct ft60x_node_s {
	unsigned char           *bulk_in_buffer;
	size_t                   len;
	size_t                   used;
	struct ft60x_node_s     *next;
	struct ft60x_node_s     *prev;
};

struct ft60x_ring_s {
	struct ft60x_node_s      first;
	struct ft60x_node_s     *wr;
	struct ft60x_node_s     *rd;
};

struct ft60x_ctrl_dev {
	struct usb_device       *device;
	struct usb_interface    *interface;
	struct list_head         ctrl_list;
	struct ft60x_data_dev   *data_dev;
	u32                      devnum;
	struct usb_device       *udev;                   /* the usb device for this device */
	struct urb              *int_in_urb;             /* the urb to read data with */
	size_t                   int_in_size;            /* the size of the receive buffer */
	signed char             *int_in_buffer;          /* the buffer to receive int data */
	dma_addr_t               int_in_data_dma;        /* the dma buffer to receive int data */
	struct kref              kref;
	struct ft60x_config      ft60x_cfg;
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
	__u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint */
	bool 			busy_write;		/* for the writing poll */
	int 			errors;			/* the last request tanked */
	spinlock_t		err_lock;		/* lock for errors */
};

struct ft60x_data_dev {
	struct usb_device       *device;
	struct usb_interface    *interface;
	struct list_head         data_list;
	struct ft60x_ctrl_dev   *ctrl_dev;
	u32                      devnum;
	struct usb_device       *udev;                  /* the usb device for this device */
	struct usb_anchor 	submitted;		/* in case we need to retract our submissions */
	int			major;			/* major of the data chardev */
	int			baseminor;		/* first minor in the data ep group */
	struct kref		kref;
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */
	struct ft60x_endpoint	ep_pair[FT60X_EP_PAIR_MAX];	/* 4 different endpoint max */
};

static void ft60x_ring_add_elem(struct ft60x_ring_s *l)
{
	struct ft60x_node_s *tmp;
	
	if(!l->wr->len){
		return;
	}
	
	if(!l->wr->next->len){
		l->wr = l->wr->next;
		return;
	} 
	
	tmp = kmalloc(sizeof(struct ft60x_node_s), GFP_NOIO);
	memset(tmp, 0, sizeof(struct ft60x_node_s));
	//TODO ALLOC COHERENT
	tmp->prev = l->wr;
	tmp->next = l->wr->next;
	l->wr->next->prev = tmp;
	l->wr->next = tmp;
	l->wr = tmp;
}

static void ft60x_ring_init(struct ft60x_ring_s *r)
{
	
	memset(r, 0, sizeof(struct ft60x_ring_s));
	r->first.next = &r->first;
	r->first.prev = &r->first;
	r->wr = &r->first;
	r->rd = &r->first;
	/*
	  r->first.bulk_in_buffer = kmalloc(ep->bulk_in_size, GFP_KERNEL);
	  if(!r->first.bulk_in_buffer) {
	  return -ENOMEM;
	  }
	  return 0;
	*/
}

static size_t ft60x_ring_read(struct ft60x_ring_s *r, char *buf, size_t len)
{
	char *pnt = buf;
	size_t rlen=len;
	
	while(r->rd->len) {
		if(r->rd->len < rlen) {
			memcpy(pnt, r->rd->bulk_in_buffer + r->rd->used, r->rd->len);
			pnt += r->rd->len;
			rlen -= r->rd->len;
			r->rd->len = 0;
			r->rd->used = 0;
			if(r->rd == r->wr)
				return len - rlen;
		} else {
			memcpy(pnt, r->rd->bulk_in_buffer + r->rd->used, rlen);
			pnt += rlen;
			r->rd->used += rlen;
			r->rd->len -= rlen;
			rlen = 0;
			return len;
		}
		r->rd = r->rd->next;
	}
	return len - rlen;
}

void ft60x_ring_free(struct ft60x_ring_s *r)
{
	struct ft60x_node_s *p= &r->first;
	struct ft60x_node_s *o;

	if(!p->next)
		return;
	
	do{
		o = p->next;
		p->next = o->next;
		if(o->bulk_in_buffer){
			kfree(o->bulk_in_buffer);
			o->bulk_in_buffer=NULL;
		}
		if(o != p){
			kfree(o);
		}
	}while(o != p);
	
}

static void ft60x_int_callback(struct urb *urb)
{
	int retval;
	printk(KERN_INFO "%s called\n", __func__);

	switch (urb->status) {
	case 0:         /* success */
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
	printk(KERN_INFO "optionnal: %04x\n", cfg->OptionalFeatureSupport);
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
	if (!memcmp(ft60x_cfg, &ctrl_dev->ft60x_cfg, sizeof(struct ft60x_config))) {
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
                              0xcf, USB_TYPE_VENDOR | USB_DIR_OUT, 0, 0,
                              cfg, sizeof(struct ft60x_config),
                              USB_CTRL_SET_TIMEOUT);

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
        int retval=0;
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
			      0xcf, USB_TYPE_VENDOR | USB_DIR_IN, 1, 0,
                              cfg, sizeof(struct ft60x_config),
                              USB_CTRL_SET_TIMEOUT);
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
        int retval=0;
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
                              0xf1, USB_TYPE_VENDOR | USB_DIR_IN, 0, 0,
                              val, 4, USB_CTRL_SET_TIMEOUT);
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

	if(ctrl_dev->int_in_buffer){
		usb_free_coherent(
			ctrl_dev->udev,
			ctrl_dev->int_in_size,
			ctrl_dev->int_in_buffer,
			ctrl_dev->int_in_data_dma
			);
	}
	// XXX free int_in_urb

	kfree(ctrl_dev);
}

static void ft60x_delete_data(struct kref *kref)
{
	struct ft60x_data_dev *data_dev = container_of(kref, struct ft60x_data_dev, kref);
	int i;
	
	for(i=0; i< FT60X_EP_PAIR_MAX; i++){

		if(data_dev->ep_pair[i].bulk_in_urb){
			usb_free_urb(data_dev->ep_pair[i].bulk_in_urb);
		}
		
		ft60x_ring_free(&data_dev->ep_pair[i].ring);
	}

	kfree(data_dev);
}

static int ft60x_allocate_ctrl_interface(struct usb_interface *interface,
					 const struct usb_device_id *id)
{
	struct usb_device *device;
	struct usb_host_interface *host_interface;
	struct ft60x_ctrl_dev *ctrl_dev=NULL;
	struct ft60x_data_dev *data_dev=NULL;
	struct usb_endpoint_descriptor *endpoint;

	int i;
	int maxp, pipe;
	int retval;

	if(!interface && !id)
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

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, ctrl_dev);

	/* 
	   Attemp to find the ctrl interface of this ctrl intf 
	   Normally, the Control interface is enumerated before this one
	   So this function should never find anything, but in case in the future
	   the data interface shows up before, we can still match.
	 */

	list_for_each_entry(data_dev, &ft60x_data_list, data_list) {
		if(ctrl_dev->devnum ==  data_dev->devnum){
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
			ctrl_dev->int_in_buffer = usb_alloc_coherent(
				ctrl_dev->udev,
				ctrl_dev->int_in_size,
				GFP_ATOMIC,
				&ctrl_dev->int_in_data_dma
				);
			
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

			maxp = usb_maxpacket(ctrl_dev->udev, pipe, usb_pipeout(pipe));
			
			usb_fill_int_urb(ctrl_dev->int_in_urb, ctrl_dev->udev, pipe,
					 ctrl_dev->int_in_buffer, maxp,
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

			//dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
		}
	}

	retval = ft60x_get_config(ctrl_dev);
	if(retval)
		goto error;
	ft60x_print_config(&ctrl_dev->ft60x_cfg);
	retval = ft60x_get_unknown(ctrl_dev);

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
	struct device* device = NULL;
	dev_t newdev;
	int ret;

	// Register the device driver
	newdev = MKDEV(data_dev->major, data_dev->baseminor + idx);
	printk(KERN_INFO "device_create: %d %d %d\n", data_dev->major, minor , idx);
	printk(KERN_INFO "drvdata: %p\n", &data_dev->ep_pair[idx]);
	device = device_create(class, NULL, newdev, NULL, "ft60x%d%c", minor, idx + 'a');
	if (IS_ERR(device)){
		printk(KERN_ALERT "Failed to create the device\n");
		goto error;
	}

	cdev_init(&data_dev->ep_pair[idx].cdev, &ft60x_data_fops);
	if ((ret = cdev_add(&data_dev->ep_pair[idx].cdev, newdev, 1)) < 0) {
		printk(KERN_ALERT "Failed to add the device\n");
		return ret;
	}
	
	printk(KERN_INFO "EBBChar: device class created correctly\n"); // Made it! device was initialized
	return 0;

error:
	return -1;
}
 
static int ft60x_register_baseminor(void)
{
	int i;

	for (i=0; i<sizeof(ft60x_minors); i++) {
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
	struct ft60x_data_dev *data_dev=NULL;
	struct ft60x_ctrl_dev *ctrl_dev=NULL;
	struct usb_endpoint_descriptor *endpoint;
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

	for(i=0; i< FT60X_EP_PAIR_MAX; i++){
		data_dev->ep_pair[i].data_dev = data_dev;
		atomic_set(&data_dev->ep_pair[i].opened, 0);
		sema_init(&data_dev->ep_pair[i].limit_sem, WRITES_IN_FLIGHT);
		spin_lock_init(&data_dev->ep_pair[i].err_lock);
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
	
	/* Attemp to find the ctrl interface of this ctrl intf */
	list_for_each_entry(ctrl_dev, &ft60x_ctrl_list, ctrl_list) {
		if(ctrl_dev->devnum ==  data_dev->devnum){
			printk(KERN_INFO "Found ctrl: %d\n", data_dev->devnum);
			ctrl_dev->data_dev = data_dev;
			data_dev->ctrl_dev = ctrl_dev;
			break;
		}
	}

	for (i = 0; i < host_interface->desc.bNumEndpoints; i++) {

		endpoint = &host_interface->endpoint[i].desc;
		ep_pair_num =  i >> 1;
		data_dev->ep_pair[ep_pair_num].used = 1;

		if (usb_endpoint_is_bulk_out(endpoint)) {
			printk("BULK OUT %d\n", i);
			/* we found a bulk out endpoint */
			data_dev->ep_pair[ep_pair_num].bulk_out_endpointAddr = endpoint->bEndpointAddress;
		}
		
		if (usb_endpoint_is_bulk_in(endpoint)) {

			printk("BULK IN %d\n", i);
			ft60x_add_device(data_dev, data_dev->ctrl_dev->interface->minor, ep_pair_num);
			// XXX check retval
			printk(KERN_INFO "%d %d\n", data_dev->ctrl_dev->interface->minor, ep_pair_num);

			data_dev->ep_pair[ep_pair_num].bulk_in_size = usb_endpoint_maxp(endpoint);
			data_dev->ep_pair[ep_pair_num].bulk_in_urb = usb_alloc_urb(0, GFP_KERNEL);
			if (!data_dev->ep_pair[ep_pair_num].bulk_in_urb) {
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

	/* increment our usage count for the device */
	kref_get(&ep->data_dev->kref);

        /* save our object in the file's private structure */
	file->private_data = ep;
	printk(KERN_INFO "prvdata: %p\n", file->private_data);

	ft60x_ring_init(&ep->ring);

	return 0;
}

static int ft60x_data_release(struct inode *inode, struct file *file)
{
	struct ft60x_endpoint *ep;

	printk(KERN_INFO "%s called\n", __func__);

	ep = file->private_data;
	if (ep == NULL)
		return -ENODEV;

	atomic_dec(&ep->opened);

	file->private_data = NULL;

	return 0;
}

static void ft60x_data_write_bulk_callback(struct urb *urb)
{
	struct ft60x_endpoint *ep;

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
	size_t writesize = min(count, (size_t) MAX_TRANSFER);

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

	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, ep->data_dev->udev,
			  usb_sndbulkpipe(ep->data_dev->udev,
					  ep->bulk_out_endpointAddr),
			  buf, writesize, ft60x_data_write_bulk_callback, ep);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &ep->data_dev->submitted);

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
		usb_free_coherent(ep->data_dev->udev, writesize, buf, urb->transfer_dma);
		usb_free_urb(urb);
	}
	up(&ep->limit_sem);

exit:
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

	memcpy(&cstm, &ctrl_dev->ft60x_cfg, sizeof(struct ft60x_config));
	cstm.FIFOMode=1;
	cstm.ChannelConfig=0;
	
	retval = ft60x_set_config(ctrl_dev, &cstm);

	ft60x_print_config(&ctrl_dev->ft60x_cfg);
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
	.fops = &ft60x_fops,
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
	printk(KERN_INFO "devnum: %d interface: %d\n", device->devnum, host_interface->desc.bInterfaceNumber);

	/* First interface is the control/int interface, the other one is for data */

	if(host_interface->desc.bInterfaceNumber == 0) {

		retval = ft60x_allocate_ctrl_interface(interface, id);
		if (retval)
			goto error;
		retval = usb_register_dev(interface, &ft60x_class);
	} else {

		retval = ft60x_allocate_data_interface(interface, id);
	}
	
	if(retval)
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

	if(host_interface->desc.bInterfaceNumber == 0) {
		/* ctrl interface */

		ctrl_dev = usb_get_intfdata(interface);
		usb_set_intfdata(interface, NULL);

		usb_deregister_dev(interface, &ft60x_class);

		list_del(&ctrl_dev->ctrl_list);

		kref_put(&ctrl_dev->kref, ft60x_delete_ctrl);

	} else {
		/* data interface */

		data_dev = usb_get_intfdata(interface);
		usb_set_intfdata(interface, NULL);

		for(i = 0; i < FT60X_EP_PAIR_MAX; i++) {
			if(data_dev->ep_pair[i].used) {
				cdev_del(&data_dev->ep_pair[i].cdev);
				device_destroy(class, MKDEV(data_dev->major, data_dev->baseminor + i));
			}
		}

		ft60x_unregister_baseminor(data_dev->baseminor);
		list_del(&data_dev->data_list);

		kref_put(&data_dev->kref, ft60x_delete_data);
	}
}

static struct usb_driver ft60x_driver = {
	.name =         "ft60x",
	.probe =        ft60x_probe,
	.disconnect =   ft60x_disconnect,
	.suspend =      ft60x_suspend,
	.resume =       ft60x_resume,
	.pre_reset =    ft60x_pre_reset,
	.post_reset =   ft60x_post_reset,
	.id_table =     ft60x_table,
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
