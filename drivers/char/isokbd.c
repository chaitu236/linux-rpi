// SPDX-License-Identifier: GPL-2.0
/*
 * IsoKbd driver
 *
 * Author: Chaitanya Vadrevu <chaitanya.vadrevu@gmail.com>
 */
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/wait.h>

/* FIXME: Don't hardcode global gpio no. Use some API to figure it out */
#define FLOW_CONTROL_GPIO 537

static int keypress_spi_buffer_t_size;
module_param(keypress_spi_buffer_t_size, int, 0400);
MODULE_PARM_DESC(keypress_spi_buffer_t_size, "size of keypress_spi_buffer_t");

typedef void keypress_spi_buffer_t;

#define ADCBUF_SIZE 100
static keypress_spi_buffer_t* adcbuf;
static int bufsize = 0;
// head points to where next element should go to
// tail points to first element
static int head, tail;
struct mutex buf_lock;
wait_queue_head_t wait_queue;

static int isokbd_open(struct inode *inode, struct file *file)
{
	pr_info("%s: %d\n", __func__, __LINE__);
	return 0;
}

static ssize_t isokbd_read(struct file *file, char __user *usrbuf,
			size_t count, loff_t *offp)
{
	keypress_spi_buffer_t *buf;
	int ret = 0;
	unsigned long missing;

	if (count != keypress_spi_buffer_t_size)
		return -EINVAL;

	mutex_lock(&buf_lock);
	if (!bufsize) {
		mutex_unlock(&buf_lock);

		ret = wait_event_interruptible(wait_queue, bufsize);
		if (ret) {
			pr_err("wait_event_interruptible\n");
			goto err_wait_event;
		}

		mutex_lock(&buf_lock);
	}
	mutex_unlock(&buf_lock);

	buf = adcbuf + tail*keypress_spi_buffer_t_size;
	missing = copy_to_user(usrbuf, buf, keypress_spi_buffer_t_size);
	if (missing) {
		pr_err("Missing %ld\n", missing);
		ret = -EFAULT;
		goto err_copy_to_user;
	}
	ret = keypress_spi_buffer_t_size;

	// Now that copy_to_user has succeeded, mark buf as empty
	tail++;
	if (tail == ADCBUF_SIZE)
		tail = 0;
	mutex_lock(&buf_lock);
	bufsize--;
	mutex_unlock(&buf_lock);

err_wait_event:
err_copy_to_user:
	return ret;
}

static ssize_t isokbd_write(struct file *file, const char __user *buf,
			 size_t count, loff_t *offp)
{
	pr_info("%s: %d\n", __func__, __LINE__);
	return -EINVAL;
}

static const struct file_operations isokbd_fops = {
	.owner		= THIS_MODULE,
	.open		= isokbd_open,
	.read		= isokbd_read,
	.write		= isokbd_write,
};

static struct miscdevice isokbd_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = KBUILD_MODNAME,
	.fops = &isokbd_fops,
};

extern ssize_t global_spidev_read(u8 *buf, size_t count);

static irqreturn_t isokbd_isr(int irq, void *data)
{
	int ret;
	keypress_spi_buffer_t *buf;

	mutex_lock(&buf_lock);
	if (bufsize == ADCBUF_SIZE) {
		mutex_unlock(&buf_lock);
		pr_err("Buffer full\n");
		goto out;
	}
	mutex_unlock(&buf_lock);

	buf = adcbuf + head*keypress_spi_buffer_t_size;

	ret = global_spidev_read((u8*)buf, keypress_spi_buffer_t_size);

	if (ret != keypress_spi_buffer_t_size) {
		pr_err("Partial read %d\n", ret);
		goto out;
	}

	// Now that spi transfer is complete, mark the buffer as filled
	head++;
	if (head == ADCBUF_SIZE)
		head = 0;
	mutex_lock(&buf_lock);
	bufsize++;
	mutex_unlock(&buf_lock);

	// Let readers know there is a new element
	wake_up_all(&wait_queue);
out:
	return IRQ_HANDLED;
}

static int __init isokbd_init(void)
{
	int irq;
	int ret;

	pr_info("%s: keypress_spi_buffer_t_size=%d\n", __func__, keypress_spi_buffer_t_size);
	if (!keypress_spi_buffer_t_size) {
		pr_err("isokbd: keypress_spi_buffer_t_size=0. Not initializing\n");
		goto err_buffer_zero;
	}

	mutex_init(&buf_lock);
	init_waitqueue_head(&wait_queue);

	adcbuf = kmalloc(keypress_spi_buffer_t_size * ADCBUF_SIZE, GFP_KERNEL);
	irq = gpio_to_irq(FLOW_CONTROL_GPIO);
	pr_info("irq %d\n", irq);

	ret = request_threaded_irq(irq, NULL, isokbd_isr, IRQF_TRIGGER_RISING | IRQF_ONESHOT, "isokbd pico", NULL);

	if (ret) {
		pr_err("Error %d in request_irq\n", ret);
		goto err_request_irq;
	}

	return misc_register(&isokbd_miscdev);

err_request_irq:
	kfree(adcbuf);
err_buffer_zero:
	return -1;
}

static void __exit isokbd_exit(void)
{
	pr_info("%s: %d\n", __func__, __LINE__);

	free_irq(gpio_to_irq(FLOW_CONTROL_GPIO), NULL);
	kfree(adcbuf);

	misc_deregister(&isokbd_miscdev);
}

module_init(isokbd_init);
module_exit(isokbd_exit);

MODULE_AUTHOR("Chaitanya Vadrevu <chaitanya.vadrevu@gmail.com>");
MODULE_DESCRIPTION("IsoKbd driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
