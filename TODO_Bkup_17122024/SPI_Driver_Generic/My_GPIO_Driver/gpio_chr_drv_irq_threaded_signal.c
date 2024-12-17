/***************************************************************************//**
*  \file       driver.c
*
*  \details    Simple GPIO driver explanation (GPIO Interrupt) and notify 
*  	       application using signal.
*******************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/uaccess.h>  //copy_to/from_user()
#include <linux/gpio.h>     //GPIO
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/signalfd.h>

#define SIGMIR 29
 
#define REG_CURRENT_TASK _IOW('a','a',int32_t*)

//LED is connected to this GPIO
#define GPIO_OUT (10)

//LED is connected to this GPIO
#define GPIO_IN  (11)

//GPIO_11_IN value toggle
unsigned int led_toggle = 0; 

//This used for storing the IRQ number for the GPIO
unsigned int gpio_irqNumber;

/* Signaling to Application */
static struct task_struct *task = NULL;
static int signum = 0;

/*Tasklet function*/
static void tasklet_fun(unsigned long t_arg);

int tasklet_data = 100;

dev_t dev = 0;
static struct class *dev_class;
static struct cdev gpio_cdev;
 
static int __init gpio_driver_init(void);
static void __exit gpio_driver_exit(void);
 
 
/*************** Driver functions **********************/
static int gpio_open(struct inode *inode, struct file *file);
static int gpio_release(struct inode *inode, struct file *file);
static ssize_t gpio_read(struct file *filp, 
                char __user *buf, size_t len,loff_t * off);
static ssize_t gpio_write(struct file *filp, 
                const char *buf, size_t len, loff_t * off);
static long gpio_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
int send_sig_info(int sig, struct siginfo *info, struct task_struct *p);
/******************************************************/

/*File operation structure "fops"*/ 
static struct file_operations fops =
{
  .owner          = THIS_MODULE,
  .read           = gpio_read,
  .write          = gpio_write,
  .open           = gpio_open,
  .release        = gpio_release,
  .unlocked_ioctl = gpio_ioctl,
};

/*********************************************************************************
 * DECLARE_TASKLET is a macro, so any name in first argument a struct variable   *
 * of type - "struct tasklet_struct"                                             *
 * *******************************************************************************/
static DECLARE_TASKLET(tasklet_name, tasklet_fun, (unsigned long)&tasklet_data);

/*Interrupt handler for GPIO 11. This will be called whenever there is a raising edge detected.*/ 
static irqreturn_t gpio_irq_handler(int irq,void *dev_id) 
{
  printk(KERN_INFO "Shared IRQ: Interrupt Occurred");
  static unsigned long flags = 0;
  
  local_irq_save(flags);
  led_toggle = (0x01 ^ led_toggle);                             // toggle the old value
  gpio_set_value(GPIO_OUT, led_toggle);                      // toggle the GPIO_OUT
  pr_info("Interrupt Occurred : GPIO_OUT : %d ",gpio_get_value(GPIO_OUT));
  local_irq_restore(flags);

  tasklet_schedule(&tasklet_name);

  return IRQ_HANDLED;
}

static irqreturn_t my_threaded_irq_handler(int irq, void* dev_id)
{
printk(KERN_INFO "Shared IRQ: Interrupt Occurred");
	static unsigned long flags = 0;

   	local_irq_save(flags);
   	led_toggle = (0x01 ^ led_toggle);                             // toggle the old value
    	gpio_set_value(GPIO_OUT, led_toggle);                      // toggle the GPIO_OUT
    	pr_info("Interrupt Occurred : GPIO_OUT : %d ",gpio_get_value(GPIO_OUT));
 	local_irq_restore(flags);
  	
	msleep(10000);
	return IRQ_HANDLED;
}

static void tasklet_fun(unsigned long t_arg)
{
      struct siginfo info;
      static int sigInt = 0;
      
      /*Sending signal to app.*/
      memset(&info, 0, sizeof(struct siginfo));
      info.si_signo = SIGMIR;
      info.si_code = SI_QUEUE;
      info.si_int = (sigInt ^= 1);
 
      if (task != NULL) {
           printk(KERN_INFO "Sending signal to app\n");
           if(send_sig_info(SIGMIR, &info, task) < 0) {
               printk(KERN_INFO "Unable to send signal\n");
           }
      }
}
 
/*
** This function will be called when we open the Device file
*/ 
static int gpio_open(struct inode *inode, struct file *file)
{
  pr_info("Device File Opened...!!!\n");
  return 0;
}

/*
** This function will be called when we close the Device file
*/ 
static int gpio_release(struct inode *inode, struct file *file)
{
    struct task_struct *ref_task = get_current();
    printk(KERN_INFO "Device File Closed...!!!\n");

    //delete the task
    if(ref_task == task) {
        task = NULL;
    }

    return 0;
}

/*
** This function will be called when we read the Device file
*/ 
static ssize_t gpio_read(struct file *filp, 
                char __user *buf, size_t len, loff_t *off)
{
  uint8_t gpio_state = 0;
  
  //reading GPIO value
  gpio_state = gpio_get_value(GPIO_OUT);
  
  //write to user
  len = 1;
  if( copy_to_user(buf, &gpio_state, len) > 0) {
    pr_err("ERROR: Not all the bytes have been copied to user\n");
  }
  
  pr_info("Read function : GPIO_10 = %d \n", gpio_state);
  
  return 0;
}

/*
** This function will be called when we write the Device file
*/
static ssize_t gpio_write(struct file *filp, 
                const char __user *buf, size_t len, loff_t *off)
{
  uint8_t rec_buf[10] = {0};
  
  if( copy_from_user( rec_buf, buf, len ) > 0) {
    pr_err("ERROR: Not all the bytes have been copied from user\n");
  }
  
  pr_info("Write Function : GPIO_10 Set = %c\n", rec_buf[0]);
  
  if (rec_buf[0]=='1') {
    //set the GPIO value to HIGH
    gpio_set_value(GPIO_OUT, 1);
  } else if (rec_buf[0]=='0') {
    //set the GPIO value to LOW
    gpio_set_value(GPIO_OUT, 0);
  } else {
    pr_err("Unknown command : Please provide either 1 or 0 \n");
  }
  
  return len;
}

static long gpio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    if (cmd == REG_CURRENT_TASK) {
        printk(KERN_INFO "REG_CURRENT_TASK\n");
        task = get_current();
        signum = SIGMIR;
    }
    return 0;
}

/*
** Module Init function
*/ 
static int __init gpio_driver_init(void)
{
  /*Allocating Major number*/
  if((alloc_chrdev_region(&dev, 0, 1, "gpio_dev")) <0){
    pr_err("Cannot allocate major number\n");
    goto r_unreg;
  }
  pr_info("Major = %d Minor = %d \n",MAJOR(dev), MINOR(dev));

  /*Creating cdev structure*/
  cdev_init(&gpio_cdev,&fops);

  /*Adding character device to the system*/
  if((cdev_add(&gpio_cdev,dev,1)) < 0){
    pr_err("Cannot add the device to the system\n");
    goto r_del;
  }

  /*Creating struct class*/
  if((dev_class = class_create(THIS_MODULE,"gpio_class")) == NULL){
    pr_err("Cannot create the struct class\n");
    goto r_class;
  }

  /*Creating device*/
  if((device_create(dev_class,NULL,dev,NULL,"gpio_device")) == NULL){
    pr_err( "Cannot create the Device \n");
    goto r_device;
  }
  
  //Output GPIO configuration
  //Checking the GPIO is valid or not
  if(gpio_is_valid(GPIO_OUT) == false){
    pr_err("GPIO %d is not valid\n", GPIO_OUT);
    goto r_device;
  }
  
  //Requesting the GPIO
  if(gpio_request(GPIO_OUT,"GPIO_OUT") < 0){
    pr_err("ERROR: GPIO %d request\n", GPIO_OUT);
    goto r_gpio_out;
  }
  
  //configure the GPIO as output
  gpio_direction_output(GPIO_OUT, 0);
  
  //Input GPIO configuratioin
  //Checking the GPIO is valid or not
  if(gpio_is_valid(GPIO_IN) == false){
    pr_err("GPIO %d is not valid\n", GPIO_IN);
    goto r_gpio_in;
  }
  
  //Requesting the GPIO
  if(gpio_request(GPIO_IN,"GPIO_IN") < 0){
    pr_err("ERROR: GPIO %d request\n", GPIO_IN);
    goto r_gpio_in;
  }
  
  //configure the GPIO as input
  gpio_direction_input(GPIO_IN);
  
  //Debounce the button with a delay of 200ms
  if(gpio_set_debounce(GPIO_IN, 200) < 0){
    pr_err("ERROR: gpio_set_debounce - %d\n", GPIO_IN);
    //goto r_gpio_in;
  }
  
  //Get the IRQ number for our GPIO
  gpio_irqNumber = gpio_to_irq(GPIO_IN);
  pr_info("gpio_irqNumber = %d\n", gpio_irqNumber);
  
  if (request_threaded_irq(gpio_irqNumber,             //IRQ number
                  NULL,   //IRQ handler
                  my_threaded_irq_handler,
		  IRQF_TRIGGER_RISING | IRQF_ONESHOT,        //Handler will be called in raising edge
                  "gpio_device",               //used to identify the device name using this IRQ
                  NULL)) {                    //device id for shared IRQ
    pr_err("my_device: cannot register IRQ ");
    goto r_gpio_in;
  }
  
  
 
  pr_info("Device Driver Insert...Done!!!\n");
  return 0;

r_gpio_in:
  gpio_free(GPIO_IN);
r_gpio_out:
  gpio_free(GPIO_OUT);
r_device:
  device_destroy(dev_class,dev);
r_class:
  class_destroy(dev_class);
r_del:
  cdev_del(&gpio_cdev);
r_unreg:
  unregister_chrdev_region(dev,1);
  
  return -1;
}

/*
** Module exit function
*/
static void __exit gpio_driver_exit(void)
{
  free_irq(gpio_irqNumber,NULL);
  gpio_free(GPIO_IN);
  gpio_free(GPIO_OUT);
  device_destroy(dev_class,dev);
  class_destroy(dev_class);
  cdev_del(&gpio_cdev);
  unregister_chrdev_region(dev, 1);
  pr_info("Device Driver Remove...Done!!\n");
}
 
module_init(gpio_driver_init);
module_exit(gpio_driver_exit);
 
MODULE_LICENSE("GPL");
MODULE_AUTHOR("mirfaisalece@gmail.com>");
MODULE_DESCRIPTION("A simple device driver - GPIO Driver (GPIO Interrupt) and notify to user app");
MODULE_VERSION("1.0");

