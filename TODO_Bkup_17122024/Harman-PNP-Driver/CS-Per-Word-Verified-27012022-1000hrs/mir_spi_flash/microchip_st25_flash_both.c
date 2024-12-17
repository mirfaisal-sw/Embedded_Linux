
#include <linux/module.h>
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

#define DYNAMIC_MINORS          32      /* ... up to 256 */
#define REG_IRQ_DELAY           _IOW('b','b',int)
#define BUF_SIZE		1024

#define SPI_BUS_NUM             1

static int m_delay = 0;
module_param(m_delay, int, 0644); /*instead of perm flags, we can use like 0644 etc.*/
MODULE_PARM_DESC(myshort, "m_delay in ms to test or debug BH");

static DECLARE_BITMAP(minors, DYNAMIC_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static struct class *mirspi_class;
static int mir_major;

struct mirspi_data {
	dev_t			devt;
	struct spi_device	*spi;
	struct list_head	device_entry;
	struct spi_transfer	t;
	struct spi_message	m;
	u8			*tx_buf;
	u8			*rx_buf;
	wait_queue_head_t	wq_txrx;
	u32			speed_hz;
	u32			users;
	u32			txlen;
	s32			status;
	u32                     size;
	u32                     page_size;
	u32			addr_width;
};

static struct spi_device *etx_spi_device;

struct spi_controller_config_data{
	unsigned turbo_mode:1;
	
	/*Toggle CS line for every word transfer.*/
	unsigned cs_per_word:1;
};

static struct spi_controller_config_data my_spi_controller_data = {
	.turbo_mode = 0,
	.cs_per_word = 1,
};

//Register information about your slave device
struct spi_board_info etx_spi_device_info =
{
	.modalias     = "mirflash",
	.max_speed_hz = 8000000,              // speed your device (slave) can handle
  	.bus_num      = SPI_BUS_NUM,          // SPI 1
  	.chip_select  = 0,                    // Use 0 Chip select (GPIO 18)
  	.mode         = SPI_MODE_0,           // SPI mode 0
	.controller_data = &my_spi_controller_data,
};

static int spi_xchange(struct spi_device *spi, unsigned char *txbuf,
		unsigned char *rxbuf, int len)
{
	struct spi_transfer t = {
		.tx_buf = txbuf,
		.rx_buf = rxbuf,
		.len    = len,
	};
	struct spi_message m;

	pr_info("MIR_SPI_FLASH: dump stack output.\n");
	dump_stack(); 

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return (!spi) ? -ESHUTDOWN : spi_sync(spi, &m);
}

#if 0
static ssize_t read(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{

	unsigned retval,i;
	u8 tx_buf[5];

	pr_info("read got invoked\n");

	tx_buf[0] = W25_READ;
	tx_buf[1] = obj->offset>>16;
	tx_buf[2] = obj->offset>>8;
	tx_buf[3] = obj->offset;
	
	retval = spi_write_then_read(obj->spi,tx_buf,4,buf,256);
	//retval = spi_read(obj->spi,tx_buf,5);
	if(retval)
		pr_info("read: spi_write_then_read got failed\n");
	return 256;
}

static ssize_t write(struct kobject *kobj,struct kobj_attribute *attr,const char *buf,size_t count) {
	u8 cmd = W25_WREN,tx_buf[260]={0};
	unsigned retval,i,j;

	tx_buf[0] = W25_WRITE;
	tx_buf[1] = obj->offset>>16;
	tx_buf[2] = obj->offset>>8;
	tx_buf[3] = obj->offset;

	j=4;
	for(i=0;i<count;i++)
	{
		tx_buf[j] = buf[i];
		j++;
	}

	retval = spi_write(obj->spi,&cmd,1);
	if(retval)
		pr_info("write (write enable): spi_write got failed\n");
	retval = spi_write(obj->spi,tx_buf,count+4);
	if(retval)
		pr_info("write (write enable): spi_write got failed\n");
	return count;

}

static ssize_t get_offset(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
	return sprintf(buf,"%ld",obj->offset);
}

static ssize_t set_offset(struct kobject *kobj,struct kobj_attribute *attr,const char *buf,size_t count) {

	unsigned block,sect,page;
	
	sscanf(buf,"%u:%u:%u",&block,&sect,&page);
	
	if((block>64) || (sect>16) || (page>16))
		return -EAGAIN;

	obj->offset = ((block*64*1024)+(sect*4*1024)+(page*256));
	return count;
}

static ssize_t erase(struct kobject *kobj,struct kobj_attribute *attr,const char *buf,size_t count) {

	unsigned blockno,sectno,offset,status;

	u8 cmd=W25_WREN,arr[4];

	sscanf(buf,"%u:%u",&blockno,&sectno);

	if((blockno>64)&&(sectno>16))
		return -EAGAIN;
		
	//offset = ((blockno*64*1024) + (sectno*4*1024));
	/* write enable latch */
		//status = spi_write(obj->spi,&cmd,1);
		//if(status)
		//	pr_info("spi_write got failed\n");
		//arr[0] = W25_SEC_ERASE;
		//arr[1] = offset>>16;
		//arr[2] = offset>>8;
		//arr[3] = offset;

		//status = spi_write(obj->spi,arr,4);
		//if(status)
		//	pr_info("spi_write got failed\n");

		//return count;
}
#endif

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

static int mir_open(struct inode *i, struct file *f)
{
	struct mirspi_data *mirspi;
	int status = -ENXIO;

	#if 1
	mutex_lock(&device_list_lock);
	list_for_each_entry(mirspi, &device_list, device_entry) {
		if (mirspi->devt == i->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status) {
		pr_debug("nothing for minor %d\n", iminor(i));
		goto err_find_dev;
	}
	if (mirspi->users != 0) {
		status = -EBUSY;
		goto err_find_dev;
	}
	
	#endif

	mirspi->users++;
	f->private_data = mirspi;

err_find_dev:
	mutex_unlock(&device_list_lock);
	
	return status;
}

static int send_write_data(void *data, size_t count)
{
	struct mirspi_data *mirspi = data;
	int res;

	//mutex_lock(&dabspi->txrx_lck);

	mirspi->txlen = count;

	#if 0 /*Commented by MIR*/
	res = spi_xchange(dabspi->spi, dabspi->tx_buf, dabspi->rx_buf,
			dabspi->txlen);
	#endif

	/*Added by MIR*/
	//res = spi_xchange(mirspi->spi, mirspi->tx_buf, NULL,
	//		mirspi->txlen);
	
	res = spi_write(mirspi->spi, mirspi->tx_buf,mirspi->txlen);
	if (res) {
		pr_err("MIR_SPI_FLASH: xchange failed[%d]\n", res);
		goto ret;
	}

ret:
	//mutex_unlock(&dabspi->txrx_lck);
	
	return res;
}

static ssize_t mir_write(struct file *f, const char __user *buf, size_t count,
		loff_t *offset)
{
	struct mirspi_data *mirspi = f->private_data;
	int status = 0;
	
	pr_info("MIR_SPI_FLASH: in finction - %s, line no - %d\n",__func__,__LINE__);
	
	if (!(mirspi && mirspi->spi))
		return -ESHUTDOWN;
	if (count > BUF_SIZE)
		return -EMSGSIZE;

	pr_info("MIR_SPI_FLASH: in finction - %s, line no - %d\n",__func__,__LINE__);
	//mutex_lock(&dabspi->wlock);

	status = copy_from_user(mirspi->tx_buf, buf, count);
	if (status) {
		pr_debug("Error in accessing user buffer to write\n");
		goto unlock_n_ret;
	}

	status = send_write_data(mirspi, count);
	if (status) {
		pr_debug("error while sending cmd\n");
		goto unlock_n_ret;
	}

	pr_info("MIR_SPI_FLASH: return from finction - %s\n and ret val - %d",
			__func__,(status) ? status : count);
unlock_n_ret:
	//mutex_unlock(&mirspi->wlock);
	return (status) ? status : count;
}

static int mir_release(struct inode *i, struct file *f)
{
	struct mirspi_data *mirspi;

	mutex_lock(&device_list_lock);
	if(f->private_data != NULL){
		mirspi = f->private_data;
		f->private_data = NULL;
		mirspi->users--;
	}
	mutex_unlock(&device_list_lock);
	return 0;
}

static const struct file_operations mir_fops = {
	.owner		= THIS_MODULE,
	.open		= mir_open,
	//.read		= mir_read,
	.write		= mir_write,
	.release	= mir_release,
	.llseek		= no_llseek,

	.unlocked_ioctl = interrupt_delay_ioctl,
};

static int mir_add_dev(struct spi_device *spi, struct mirspi_data *mirspi)
{
	int status;
	unsigned long minor;
	struct device *dev;

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, DYNAMIC_MINORS);
	if (minor >= DYNAMIC_MINORS) {
		pr_err("MIR_SPI_FLASH: no minor number available\n");
		status = -ENODEV;
		goto end;
	}
	mirspi->devt = MKDEV(mir_major, minor);
	dev = device_create(mirspi_class, &spi->dev, mirspi->devt, mirspi,
			"spi0");
	status = PTR_ERR_OR_ZERO(dev);
	if (status) {
		pr_err("MIR_SPI_FLASH: Couldn't create device file for minor%lu\n",
			minor);
		goto end;
	}
	set_bit(minor, minors);
	list_add(&mirspi->device_entry, &device_list);
end:
	mutex_unlock(&device_list_lock);
	return status;
}

static int mir_alloc_res(struct mirspi_data *mirspi)
{
	int status = 0;

	/*status = kfifo_alloc(&mirspi->fifo, FIFO_SIZE, GFP_KERNEL);
	if (status) {
		pr_debug("kfifo_alloc failed\n");
		return status;
	}*/

	mirspi->tx_buf = kzalloc(BUF_SIZE, GFP_DMA);
	if (!mirspi->tx_buf)
		goto err_txbuf_alloc;

	mirspi->rx_buf = kzalloc(BUF_SIZE, GFP_DMA);
	if (!mirspi->rx_buf)
		goto err_rxbuf_alloc;

	return status;

err_rxbuf_alloc:
	kfree(mirspi->tx_buf);
err_txbuf_alloc:
	//kfifo_free(&mirspi->fifo);
	return -ENOMEM;
}

static void mir_dealloc_res(struct mirspi_data *mirspi)
{
	//kfifo_free(&dabspi->fifo);
	kfree(mirspi->tx_buf);
	kfree(mirspi->rx_buf);
}

static int mir_spi_probe(struct spi_device *spi)
{
	int status = 0;
	struct mirspi_data *mirspi;
	
	pr_info("MIR_SPI_FLASH: %s function invoked\n",__func__);

	mirspi = kzalloc(sizeof(*mirspi), GFP_KERNEL);
	if (!mirspi)
		return -ENOMEM;

	spi->controller_data = &my_spi_controller_data;

	mirspi->spi = spi;

	mirspi->speed_hz = spi->max_speed_hz;
	
	if(device_property_read_u32(&spi->dev,"size",&mirspi->size)==0){
	}
	else{
		pr_err("MIR_SPI_FLASH : missing missing \"size\" property\n");
		goto err;
	}

	if(device_property_read_u32(&spi->dev,"pagesize",&mirspi->page_size)==0){
        }
        else{
                pr_err("MIR_SPI_FLASH : missing missing \"pagesize\" property\n");
		goto err;
        }

        if(device_property_read_u32(&spi->dev,"address-width",&mirspi->addr_width)==0){
        }
        else{
                 pr_err("MIR_SPI_FLASH : missing missing \"address-width\" property\n");
		 goto err;
        }

	/** setting private data to spi core **/
	spi_set_drvdata(spi, mirspi);
	mir_alloc_res(mirspi);

	#if 1
	mir_major = register_chrdev(0, "mir_spi", &mir_fops);
        if (mir_major < 0) {
                pr_info("MIR_SPI_FLASH: Device registration failed, \
                                Major Number:%d\n", mir_major);
                return mir_major;
        }

        mirspi_class = class_create(THIS_MODULE, "mir_spi");
        if (IS_ERR(mirspi_class)) {
                status = PTR_ERR(mirspi_class);
                pr_info("MIR_SPI_FLASH: class registration failed\n");
                //goto class_fail;
        }

	status = mir_add_dev(spi, mirspi);
	
	#endif
        
	pr_info("MIR_SPI_FLASH: prove successfule with return value - %d\n",status);

	return status;

err:
	pr_err("MIR spi flash probe failure\n");
	kfree(mirspi);

	return status;
}

static int mir_spi_remove(struct spi_device *dev)
{
	struct mirspi_data *mirspi = spi_get_drvdata(dev);

	if (!mirspi)
		pr_err("No mirspi\n");

	pm_runtime_allow(&mirspi->spi->master->dev);
	mirspi->spi = NULL;
	mutex_lock(&device_list_lock);
	list_del(&mirspi->device_entry);
	device_destroy(mirspi_class, mirspi->devt);
	clear_bit(MINOR(mirspi->devt), minors);
	mutex_unlock(&device_list_lock);
	mir_dealloc_res(mirspi);
	kfree(mirspi);
	
	return 0;
}

static const struct of_device_id mirs_spi_of_match[] = {
	{.compatible = "mir,mirflash"},
	{},
};
MODULE_DEVICE_TABLE(of, mirs_spi_of_match);

static const struct spi_device_id mir_spi_id_table[] = {
        { "mirflash",0 },
        {},
};
MODULE_DEVICE_TABLE(spi, mir_spi_id_table);

static struct spi_driver mir_spi_driver = {
        .driver = {
                .name = "mirflash",
		//.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(mirs_spi_of_match),
        },
        .probe = mir_spi_probe,
	.remove = mir_spi_remove,
        .id_table = mir_spi_id_table,
};
module_spi_driver(mir_spi_driver);

#if 0
static int __init mir_spi_init(void)
{
	int status = 0;

	int     ret;
	struct  spi_master *master;
	unsigned long minor;
	struct device *dev;  

	struct mirspi_data *mirspi;

        pr_info("MIR_SPI_FLASH: %s function invoked\n",__func__);

        mirspi = kzalloc(sizeof(*mirspi), GFP_KERNEL);
        if (!mirspi)
                 return -ENOMEM;

	mirspi->spi = etx_spi_device;
	pr_info("MIR_SPI_FLASH : init of driver invoked\n");
  	
	master = spi_busnum_to_master( etx_spi_device_info.bus_num );
  	if( master == NULL )
  	{
    		pr_err("SPI Master not found.\n");
    		return -ENODEV;
  	}
   
  	// create a new slave device, given the master and device info
  	etx_spi_device = spi_new_device( master, &etx_spi_device_info );
  	if( etx_spi_device == NULL ) 
 	{
    		pr_err("FAILED to create slave.\n");
    		return -ENODEV;
  	}
  
  	// 8-bits in a word
  	etx_spi_device->bits_per_word = 8;

  	// setup the SPI slave device
  	ret = spi_setup( etx_spi_device );
  	if( ret )
  	{
    		pr_err("FAILED to setup slave.\n");
    		spi_unregister_device( etx_spi_device );
    		return -ENODEV;
  	}	

	mir_major = register_chrdev(0, "mir_spi", &mir_fops);
	if (mir_major < 0) {
		pr_info("MIR_SPI_FLASH: Device registration failed, \
				Major Number:%d\n", mir_major);
		return mir_major;
	}

	mirspi_class = class_create(THIS_MODULE, "mir_spi");
	if (IS_ERR(mirspi_class)) {
		status = PTR_ERR(mirspi_class);
		pr_info("MIR_SPI_FLASH: class registration failed\n");
		goto class_fail;
	}

	minor = find_first_zero_bit(minors, DYNAMIC_MINORS);
	if (minor >= DYNAMIC_MINORS) {
		    pr_err("MIR_SPI_FLASH: no minor number available\n");
	            status = -ENODEV;
	            //goto end;
        }
        mirspi->devt = MKDEV(mir_major, minor);
        dev = device_create(mirspi_class, &etx_spi_device->dev, mirspi->devt, mirspi,
                          "spi0");
	
	status = spi_register_driver(&mir_spi_driver);
	if (status < 0) {
		pr_info("MIR_SPI_FLASH: driver registration failed\n");
		goto spi_reg_fail;
	}
	pr_info("MIR_SPI_FLASH : spi driver registration successful\n");
	

	return status;

spi_reg_fail:
	class_destroy(mirspi_class);
class_fail:
	unregister_chrdev(mir_major, mir_spi_driver.driver.name);
	return status;
}
module_init(mir_spi_init);

static void __exit mir_spi_exit(void)
{
	spi_unregister_driver(&mir_spi_driver);
	class_destroy(mirspi_class);
	unregister_chrdev(mir_major, mir_spi_driver.driver.name);
}
module_exit(mir_spi_exit);
#endif

MODULE_AUTHOR("Mir Faisal <mirfaisalece@gmail.com>");
MODULE_DESCRIPTION("sample spi slave drv");
MODULE_LICENSE("Dual MIT/GPL");

