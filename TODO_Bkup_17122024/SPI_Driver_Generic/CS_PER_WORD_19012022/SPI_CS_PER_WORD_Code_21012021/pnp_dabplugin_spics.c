/*
 *  pnp_dabplugin.c
 *  Copyright (C) 2020 Harman International Ltd,
 *  by Josmon Paul <josmon.paul@harman.com>
 *  Created on: 29-Dec-2020
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License v2.0 as published by
 * the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#undef DEBUG
#define pr_fmt(fmt) "pnp_dabplugin: "fmt

#include<linux/module.h>
/*#include <linux/sched/types.h>*/   /*TODO*/
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/kfifo.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/sched/rt.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/pm_runtime.h>

#include <linux/moduleparam.h>
#define REG_IRQ_DELAY     _IOW('b','b',int)

#define CREATE_TRACE_POINTS

#define DYNAMIC_MINORS          32      /* ... up to 256 */
#define CHECKSUM_SIZE		4
#define DAB_MAX_PKT_SIZE        2032   /*Data length*/
#define DAB_NUM_BOOT_MSG        2
#define DAB_MSG_HEADER_LEN      12   /*Header length*/
#define BUF_SIZE		(DAB_MAX_PKT_SIZE + \
					DAB_MSG_HEADER_LEN + CHECKSUM_SIZE)
#define FIFO_SIZE		(4 * BUF_SIZE)
#define DAB_RATELIMIT_BURST	2	/* no of messages emitted */
#define DAB_RATELIMIT_INTERVAL	(30*HZ)	/* 30 seconds */

//#define GPIO_CS

static int m_delay = 0;
module_param(m_delay, int, 0644); /*instead of perm flags, we can use like 0644 etc.*/
MODULE_PARM_DESC(myshort, "m_delay in ms to test or debug BH");


static DECLARE_BITMAP(minors, DYNAMIC_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

struct dabspi_data {
	dev_t			devt;
	struct spi_device	*spi;
	struct list_head	device_entry;
	struct spi_transfer	t;
	struct spi_message	m;
	u8			*tx_buf;
	u8			*rx_buf;
	struct mutex		rlock, wlock, txrx_lck;
	struct kfifo		fifo;
	struct completion	data_avail;
	wait_queue_head_t	wq_txrx;
	u32			speed_hz;
	u32			cs_gpio;
	u32			irq_gpio;
	u32			gpio_irq_num;
	u32			users;
	u32			txlen;
	s32			status;
};

static void dab_cs_assert(unsigned int gpio)
{
	#ifdef GPIO_CS
	gpio_set_value(gpio, 0);
	#endif
}

static void dab_cs_deassert(unsigned int gpio)
{
	#ifdef GPIO_CS
	gpio_set_value(gpio, 1);
	#endif
}

static int spi_xchange(struct spi_device *spi, unsigned char *txbuf,
		unsigned char *rxbuf, int len)
{
	struct spi_transfer t = {
		.tx_buf = txbuf,
		.rx_buf = rxbuf,
		.len    = len,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	dump_stack();

	return (!spi) ? -ESHUTDOWN : spi_sync(spi, &m);
}

static unsigned int is_valid_dabheader(unsigned char *dh)
{
	return dh && (dh[2] == 0x52) && (dh[1] == 0x44) && (dh[0] == 0x57)
		&& ((((dh[9] << 8) | dh[8])) <= DAB_MAX_PKT_SIZE);
}

static int dab_fill_kfifo(void *data)
{

	int res, pktsize, pktlen;
	struct dabspi_data *dabspi = data;
	unsigned char *pkt = NULL;

	static DEFINE_RATELIMIT_STATE(dab_rl, DAB_RATELIMIT_INTERVAL,
			DAB_RATELIMIT_BURST);

	res = is_valid_dabheader(dabspi->rx_buf);
	if (!res) {
		print_hex_dump_debug("PNP_Dabplugin: Invalid Packet: ",
					DUMP_PREFIX_OFFSET, 16, 1,
					dabspi->rx_buf, dabspi->txlen, false);
		goto ret;
	}

	pktlen = (dabspi->rx_buf[9] << 8) | dabspi->rx_buf[8];
	pktlen += CHECKSUM_SIZE; /*for checksum*/
	dev_dbg(&dabspi->spi->dev,"PNP_Dabplugin: RXD Len = %d DAB PKT LEN = %d\n ",
			dabspi->txlen, pktlen);
	/*if data is read during write operation*/
	if (dabspi->txlen >= (pktlen + DAB_MSG_HEADER_LEN)) {
		dev_dbg(&dabspi->spi->dev,"PNP_Dabplugin: Full Packed Received\n");

	} else {
		dev_dbg(&dabspi->spi->dev,"PNP_Dabplugin: Partial Pkt Rxd. Pending %d bytes\n",
                              pktlen - (dabspi->txlen - DAB_MSG_HEADER_LEN));
		res = spi_xchange(dabspi->spi, NULL,
				dabspi->rx_buf + dabspi->txlen,
				(pktlen -
				 (dabspi->txlen - DAB_MSG_HEADER_LEN)));
		if (res) {
			dev_dbg(&dabspi->spi->dev,"PNP_Dabplugin: Error reading pending data\n");
			goto ret;
		}
#ifdef CONFIG_DYNAMIC_DEBUG
		print_hex_dump_debug("PNP_Dabplugin: DAB PLUGIN: REM RX: ",
					DUMP_PREFIX_OFFSET, 16, 1,
					dabspi->rx_buf + dabspi->txlen, pktlen -
					(dabspi->txlen - DAB_MSG_HEADER_LEN),
					false);
#endif

	}
	pkt = dabspi->rx_buf;
	pktsize = pktlen + DAB_MSG_HEADER_LEN;

	if (kfifo_is_full(&dabspi->fifo)) {
		if (___ratelimit(&dab_rl, "dab_fill_kfifo"))
			pr_warn("PNP_Dabplugin: FIFO is full droping data!\n");
		res = -ENOBUFS;
		goto no_space;
	}
	res = kfifo_avail(&dabspi->fifo);
	if (res < pktsize) {
		if (___ratelimit(&dab_rl, "dab_fill_kfifo"))
			pr_warn("PNP_Dabplugin: No free space [A:%d - R:%d] droping data!\n",
					res, pktsize);
		goto no_space;
	}
	res = kfifo_in(&dabspi->fifo, pkt, pktsize);
	if (res != pktsize) {
		pr_debug("PNP_Dabplugin: Error in pushing data into fifo [pktsize=%d, bytes pushed=%d]\n",
				pktsize, res);
		goto end;
	}
	res = 0;

no_space:
	complete(&dabspi->data_avail);
ret:
	dabspi->status = res;
end:
	return res;
}


static irqreturn_t send_spi_data(int irq, void *data)
{
	#if 0 /*Commented by MIR for testing*/
	struct dabspi_data *dabspi = data;
	int res;

	static int irq_cnt = 0;
	irq_cnt++;
	
	pr_info("#########################In Threaded  IRQ BH - %d times !!\n",irq_cnt);
	
	mutex_lock(&dabspi->txrx_lck);

	#ifdef GPIO_CS
	dab_cs_assert(dabspi->cs_gpio);
        #endif	
	dabspi->txlen = DAB_MSG_HEADER_LEN;

	res = spi_xchange(dabspi->spi, NULL, dabspi->rx_buf, dabspi->txlen);
	if (res) {
		pr_debug("PNP_Dabplugin: xchange failed[%d]\n", res);
		goto ret;
	}

	res = dab_fill_kfifo(dabspi);
	if (res)
		pr_debug("PNP_Dabplugin: Error %s\n", __func__);
ret:

	#ifdef GPIO_CS
	dab_cs_deassert(dabspi->cs_gpio);
	#endif
	mutex_unlock(&dabspi->txrx_lck);
 	
	if(m_delay == 1000){
	        pr_err("Witing for %d msec in BH - %s\n",m_delay,__func__);  
		msleep(m_delay);/*Busy wait here intensionally for a long time.*/

		/*pr_info("In wait for completion\n");
		struct completion *comp = kzalloc(sizeof(*comp), GFP_KERNEL); 
		init_completion(comp);
		wait_for_completion(comp);
		pr_info("Out of completion\n");*/
	}
	
	#endif
	return IRQ_HANDLED;
}

static int send_write_data(void *data, size_t count)
{
	struct dabspi_data *dabspi = data;
	int res;

	mutex_lock(&dabspi->txrx_lck);

	#ifdef GPIO_CS
	dab_cs_assert(dabspi->cs_gpio);
	#endif
	dabspi->txlen = count;

	#if 1 /*Commented by MIR*/
	res = spi_xchange(dabspi->spi, dabspi->tx_buf, dabspi->rx_buf,
			dabspi->txlen);
	#endif
	/*Added by MIR*/
	
	#if 0
	res = spi_xchange(dabspi->spi, dabspi->tx_buf, NULL,
			dabspi->txlen);
	#endif

	if (res) {
		dev_dbg(&dabspi->spi->dev,"PNP_Dabplugin: xchange failed[%d]\n", res);
		goto ret;
	}

	#if 0 /*Commented by MIR*/
	res = dab_fill_kfifo(dabspi);
	if (res)
		pr_debug("Fill KFifo Error %s\n", __func__);
	#endif
ret:
	mutex_unlock(&dabspi->txrx_lck);
	
	#ifdef GPIO_CS
	dab_cs_deassert(dabspi->cs_gpio);
	#endif
	return res;
}

static irqreturn_t dab_irq_handler(int irq, void *data)
{
	static u64 irqs;

	pr_err("PNP_Dabplugin: IRQ == %llu\n", ++irqs);
	return IRQ_WAKE_THREAD;
}

static int dab_open(struct inode *i, struct file *f)
{
	struct dabspi_data *dabspi;
	int status = -ENXIO;

	mutex_lock(&device_list_lock);
	list_for_each_entry(dabspi, &device_list, device_entry) {
		if (dabspi->devt == i->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status) {
		pr_debug("PNP_Dabplugin: nothing for minor %d\n", iminor(i));
		goto err_find_dev;
	}
	if (dabspi->users != 0) {
		status = -EBUSY;
		goto err_find_dev;
	}
	dabspi->users++;
	f->private_data = dabspi;

err_find_dev:
	mutex_unlock(&device_list_lock);
	return status;
}

static ssize_t dab_read(struct file *f, char __user *buf, size_t count,
		loff_t *offset)
{
	struct dabspi_data *dabspi = f->private_data;
	ssize_t status = 0;
	unsigned int copied = 0;

	if (!(dabspi && dabspi->spi))
		return -ESHUTDOWN;
	if (count > BUF_SIZE)
		return -EMSGSIZE;

	mutex_lock(&dabspi->rlock);

	while (kfifo_is_empty(&dabspi->fifo)) {
		if (f->f_flags & O_NONBLOCK) {
			copied = -EAGAIN;
			goto ret;
		}
		//pr_err("PNP_Dabplugin: No Data available in Kfifo, going to wait\n");
		status = wait_for_completion_interruptible(&dabspi->data_avail);
		if (status) {
			pr_err("PNP_Dabplugin: Read was interrupted [%zd]\n",
				status);
			copied = status;
			goto ret;
		}
		if (!dabspi->spi) {
			copied = -ESHUTDOWN;
			goto ret;
		}
	}
	status = kfifo_to_user(&dabspi->fifo, buf, count, &copied);
	if (status) {
		pr_debug("PNP_Dabplugin: Error Reading %zu data from kfifo\n",
			count);
		copied = status;
		goto ret;
	}
ret:
	mutex_unlock(&dabspi->rlock);
	if (!copied) {
		pr_debug("PNP_Dabplugin: Error: Read Returning 0!!!\n");
		pr_debug("PNP_Dabplugin: buf[%p] count[%zu], offset[%p], res[%zd], copied[%d]\n",
				buf, count, offset, status, copied);
		pr_debug("PNP_Dabplugin: kfifo is %s\n",
				kfifo_is_empty(&dabspi->fifo) ?
					"empty" : "not empty");
		pr_debug("PNP_Dabplugin: kfifo_len = %d\n",
				kfifo_len(&dabspi->fifo));
		pr_debug("PNP_Dabplugin: kfifo.in = %d kfifo.out = %d kfifo.mask = %d kfifo.esize = %d kfifo.data = %p\n",
				dabspi->fifo.kfifo.in,
				dabspi->fifo.kfifo.out, dabspi->fifo.kfifo.mask,
				dabspi->fifo.kfifo.esize,
				dabspi->fifo.kfifo.data);
	}
	return copied;
}

static ssize_t dab_write(struct file *f, const char __user *buf, size_t count,
		loff_t *offset)
{
	struct dabspi_data *dabspi = f->private_data;
	int status = 0;

	if (!(dabspi && dabspi->spi))
		return -ESHUTDOWN;
	if (count > BUF_SIZE)
		return -EMSGSIZE;

	mutex_lock(&dabspi->wlock);

	status = copy_from_user(dabspi->tx_buf, buf, count);
	if (status) {
		pr_debug("PNP_Dabplugin: Error in accessing user buffer to write\n");
		goto unlock_n_ret;
	}

	status = send_write_data(dabspi, count);
	if (status) {
		pr_debug("PNP_Dabplugin: error while sending cmd\n");
		goto unlock_n_ret;
	}

unlock_n_ret:
	mutex_unlock(&dabspi->wlock);
	return (status) ? status : count;
}

static int dab_release(struct inode *i, struct file *f)
{
	struct dabspi_data *dabspi;

	mutex_lock(&device_list_lock);
	dabspi = f->private_data;
	f->private_data = NULL;
	dabspi->users--;
	mutex_unlock(&device_list_lock);
	return 0;
}

static struct class *dabspi_class;
static int dab_major;

static int dab_add_dev(struct spi_device *spi, struct dabspi_data *dabspi)
{
	int status;
	unsigned long minor;
	struct device *dev;

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, DYNAMIC_MINORS);
	if (minor >= DYNAMIC_MINORS) {
		dev_dbg(&spi->dev,
			"PNP_Dabplugin: no minor number available\n");
		status = -ENODEV;
		goto end;
	}
	dabspi->devt = MKDEV(dab_major, minor);
	dev = device_create(dabspi_class, &spi->dev, dabspi->devt, dabspi,
			"spi0");
	status = PTR_ERR_OR_ZERO(dev);
	if (status) {
		dev_dbg(&spi->dev,
		"PNP_Dabplugin: Couldn't create device file for minor%lu\n",
			minor);
		goto end;
	}
	set_bit(minor, minors);
	list_add(&dabspi->device_entry, &device_list);
end:
	mutex_unlock(&device_list_lock);
	return status;
}

static int dab_alloc_res(struct dabspi_data *dabspi)
{
	int status = 0;

	status = kfifo_alloc(&dabspi->fifo, FIFO_SIZE, GFP_KERNEL);
	if (status) {
		pr_debug("kfifo_alloc failed\n");
		return status;
	}

	dabspi->tx_buf = kzalloc(BUF_SIZE, GFP_DMA);
	if (!dabspi->tx_buf)
		goto err_txbuf_alloc;

	dabspi->rx_buf = kzalloc(BUF_SIZE, GFP_DMA);
	if (!dabspi->rx_buf)
		goto err_rxbuf_alloc;

	return status;

err_rxbuf_alloc:
	kfree(dabspi->tx_buf);
err_txbuf_alloc:
	kfifo_free(&dabspi->fifo);
	return -ENOMEM;
}

static void dab_dealloc_res(struct dabspi_data *dabspi)
{
	kfifo_free(&dabspi->fifo);
	kfree(dabspi->tx_buf);
	kfree(dabspi->rx_buf);
}

static int dab_gpio_alloc(struct dabspi_data *dabspi)
{
	int status;
	struct irq_desc *dab_irq_desc;
	struct sched_param param = { .sched_priority = 20}; //MAX_RT_PRIO - 1 };

	#ifdef GPIO_CS
	if (!gpio_is_valid(dabspi->cs_gpio)) {
		pr_debug("PNP_Dabplugin: Invalid cs gpio number %d\n",
			dabspi->cs_gpio);
		goto err_gpio_invalid;
	}
	#endif

	if (!gpio_is_valid(dabspi->irq_gpio)) {
		pr_debug("PNP_Dabplugin: Invalid irq gpio number %d\n",
				dabspi->irq_gpio);
		goto err_gpio_invalid;
	}
	
	#ifdef GPIO_CS
	status = gpio_request(dabspi->cs_gpio, "dab_cs_pin");
	if (status) {
		pr_debug("PNP_Dabplugin: Claiming cs gpio pin %d failed!\n",
				dabspi->cs_gpio);
		goto err_gpio_invalid;
	}
	#endif

	status = gpio_request(dabspi->irq_gpio, "dab_irq_pin");
	if (status) {
		pr_debug("PNP_Dabplugin: Claiming irq gpio pin %d failed!\n",
				dabspi->irq_gpio);
		goto err_gpio_invalid;
	}

	#ifdef GPIO_CS
	status = gpio_direction_output(dabspi->cs_gpio, 1);
	if (status) {
		pr_debug("PNP_Dabplugin: Setting direction cs gpio %d failed\n",
				dabspi->cs_gpio);
		goto err_gpio_dir;
	}
	#endif

	status = gpio_direction_input(dabspi->irq_gpio);
	if (status) {
		pr_debug("PNP_Dabplugin: Setting direction irq gpio %d failed\n",
				dabspi->irq_gpio);
		goto err_gpio_dir;
	}

	dabspi->gpio_irq_num = gpio_to_irq(dabspi->irq_gpio);
	pr_debug("PNP_Dabplugin: requesting irq  for irqnum = %d (gpio num = %d)\n",
			dabspi->gpio_irq_num, dabspi->irq_gpio);
	status = request_threaded_irq(dabspi->gpio_irq_num, dab_irq_handler,
			send_spi_data, IRQF_TRIGGER_FALLING, "pnp_dabplugin",
				dabspi);
	if (status) {
		pr_debug("PNP_Dabplugin: requesting irq failed for irqnum = %d (gpio num = %d)\n",
				dabspi->gpio_irq_num, dabspi->irq_gpio);
		goto err_gpio_dir;
	}

	dab_irq_desc = irq_to_desc(dabspi->gpio_irq_num);
        
	sched_setscheduler(dab_irq_desc->action->thread, SCHED_FIFO, &param);
	sched_setscheduler(dabspi->spi->master->kworker_task, SCHED_FIFO,
			&param);

	return status;

err_gpio_dir:

	#ifdef GPIO_CS
	gpio_free(dabspi->cs_gpio);
        #endif
	gpio_free(dabspi->irq_gpio);
err_gpio_invalid:
	status = -EIO;
	return status;
}

static void dab_gpio_dealloc(struct dabspi_data *dabspi)
{
	free_irq(dabspi->gpio_irq_num, dabspi);
	
	#ifdef GPIO_CS
	gpio_free(dabspi->cs_gpio);
	#endif
	gpio_free(dabspi->irq_gpio);
}

static u32 previous_mode;
static void __iomem *addr;
static void dab_set_pinmux(u32 paddr, u32 val)
{
	addr = ioremap(paddr, sizeof(val));
	if (!addr) {
		pr_err("can't ioremap\n");
		return;
	}
	previous_mode = readl(addr);
	writel(val, addr);
	pr_debug("Pinmux set Reg:%x, val:%x\n", paddr, val);
}

static void dab_restore_pinmux(void)
{
	if (addr) {
		writel(previous_mode, addr);
		iounmap(addr);
	}
}

static int dab_read_of_node(struct dabspi_data *dabspi)
{
	struct device_node *dn = of_find_node_by_name(NULL, "dabplugin");
	int rc;
	u32 value, cs_reg;
	const __be32 *p = NULL;
	struct property *prop;


	#ifdef GPIO_CS
	rc = of_property_read_u32(dn, "dab-cs-gpio", &value);
	if (rc) {
		pr_debug("PNP_Dabplugin: %s has no valid 'dab-cs-gpio' property (%d)\n",
		dn->full_name, rc);
		return rc;
	}
	dabspi->cs_gpio = value;
        #endif

	rc = of_property_read_u32(dn, "dab-irq-gpio", &value);
	if (rc) {
		pr_debug("PNP_Dabplugin: %s has no valid 'dab-irq-gpio' property (%d)\n",
				dn->full_name, rc);
		return rc;
	}
	dabspi->irq_gpio = value;


	#ifdef GPIO_CS
	prop = of_find_property(dn, "dab-cs-pinmux", NULL);
	if (prop) {
		p = of_prop_next_u32(prop, p, &cs_reg);
		if (p && of_prop_next_u32(prop, p, &value))
			dab_set_pinmux(cs_reg, value);
	}
        #endif

	return rc;
}

static int dab_spi_probe(struct spi_device *spi)
{
	int status;
	struct dabspi_data *dabspi;

	dabspi = kzalloc(sizeof(*dabspi), GFP_KERNEL);
	if (!dabspi)
		return -ENOMEM;

	/* Initialize the driver data */
	dabspi->spi = spi;
	mutex_init(&dabspi->rlock);
	mutex_init(&dabspi->wlock);
	mutex_init(&dabspi->txrx_lck);
	INIT_LIST_HEAD(&dabspi->device_entry);
	init_completion(&dabspi->data_avail);
	init_waitqueue_head(&dabspi->wq_txrx);

	dabspi->speed_hz = spi->max_speed_hz;
	status = dab_read_of_node(dabspi);
	if (status)
		goto err;

	/** setting private data to spi core **/
	spi_set_drvdata(spi, dabspi);
	status = dab_alloc_res(dabspi);
	if (status)
		goto err;

	status = dab_gpio_alloc(dabspi);
	if (status)
		goto err_res;

	status = dab_add_dev(spi, dabspi);
	if (status)
		goto err_dev;


	return status;

err_dev:
	dab_gpio_dealloc(dabspi);
err_res:
	dab_dealloc_res(dabspi);

err:
	kfree(dabspi);

	return status;
}

static int dab_spi_remove(struct spi_device *dev)
{
	struct dabspi_data *dabspi = spi_get_drvdata(dev);

	if (!dabspi)
		goto end;

	pm_runtime_allow(&dabspi->spi->master->dev);
	synchronize_irq(dabspi->gpio_irq_num);
	dabspi->spi = NULL;
	complete_all(&dabspi->data_avail);
	wake_up_interruptible_all(&dabspi->wq_txrx);
	mutex_lock(&device_list_lock);
	list_del(&dabspi->device_entry);
	device_destroy(dabspi_class, dabspi->devt);
	clear_bit(MINOR(dabspi->devt), minors);
	mutex_unlock(&device_list_lock);
	dab_dealloc_res(dabspi);
	dab_gpio_dealloc(dabspi);
	kfree(dabspi);
end:
	dab_restore_pinmux();
	return 0;
}

static long interrupt_delay_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
        long irq_delay = 1000;/*in ms*/

        if (cmd == REG_IRQ_DELAY)
        {
                printk(KERN_INFO "REG_IRQ_DELAY and delay set = %d\n",irq_delay);
                //interrupt_cnt_irq_num = arg;
                m_delay = irq_delay;

                //irq_cnt = interrupt_cnt_get_irq_statistics(unsigned int irq_num, int cpu);
                //signum = SIGMIR;
        }

        return 0;
}


static const struct of_device_id dabplugin_of_match[] = {
	{
		.compatible = "pnp_dabplugin",

	},
	{}

};
MODULE_DEVICE_TABLE(of, dabplugin_of_match);

static const struct spi_device_id dab_spi_id[] = {
	{ "pnp_dabplugin", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, dab_spi_id);

static struct spi_driver dab_spi_driver = {
	.driver			= {
		.name		= "pnp_dabplugin",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(dabplugin_of_match),
	},
	.probe			= dab_spi_probe,
	.remove			= dab_spi_remove,
	.id_table		= dab_spi_id,
};

static const struct file_operations dab_fops = {
	.owner		= THIS_MODULE,
	.open		= dab_open,
	.read		= dab_read,
	.write		= dab_write,
	.release	= dab_release,
	.llseek		= no_llseek,

	.unlocked_ioctl = interrupt_delay_ioctl,
};

static int __init dab_spi_init(void)
{
	int status;

	dab_major = register_chrdev(0, "pnp_dabspi", &dab_fops);
	if (dab_major < 0) {
		pr_err("PNP_Dabplugin: Device registration failed, \
				Major Number:%d\n", dab_major);
		return dab_major;
	}

	dabspi_class = class_create(THIS_MODULE, "pnp_dabspi");
	if (IS_ERR(dabspi_class)) {
		status = PTR_ERR(dabspi_class);
		pr_err("PNP_Dabplugin: class registration failed\n");
		goto class_fail;
	}

	status = spi_register_driver(&dab_spi_driver);
	if (status < 0) {
		pr_err("PNP_Dabplugin: driver registration failed\n");
		goto spi_reg_fail;
	}

	return status;

spi_reg_fail:
	class_destroy(dabspi_class);
class_fail:
	unregister_chrdev(dab_major, dab_spi_driver.driver.name);
	return status;
}
module_init(dab_spi_init);

static void __exit dab_spi_exit(void)
{
	spi_unregister_driver(&dab_spi_driver);
	class_destroy(dabspi_class);
	unregister_chrdev(dab_major, dab_spi_driver.driver.name);
}
module_exit(dab_spi_exit);

MODULE_AUTHOR("Josmon Paul  <josmon.paul@harman.com>");
MODULE_AUTHOR("Vivek Chidanandamurthy <Vivek.Chidanandamurthy@harman.com");
MODULE_AUTHOR("Sneha Mohan  <Sneha.Mohan@harman.com>");
MODULE_DESCRIPTION("Driver for DAB SPI interface for PNP tuner chip");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:pnpdabplugin");
