/************************************************************
*
* Copyright (C) 2006-2007, Analog Devices. All Rights Reserved
*
* FILE twi_keypad.c
* PROGRAMMER(S): Michael Hennerich (Analog Devices Inc.)
*				 <hennerich@blackfin.uclinux.org>
*
*
* DATE OF CREATION: Feb. 24th 2006
*
* SYNOPSIS:
*
* DESCRIPTION: TWI Driver for an 4x4 Keybaord Matrix connected to
*              a PCF8574 I2C IO expander
* CAUTION:
**************************************************************
* MODIFICATION HISTORY:
* 24.02.2006 twi_keypad.c Created. (Michael Hennerich)
* 27.11.2007 twi_keypad.c cleanup (Michael Hennerich)
************************************************************
*
* This program is free software; you can distribute it and/or modify it
* under the terms of the GNU General Public License (Version 2) as
* published by the Free Software Foundation.
*
* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
* for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
*
************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/major.h>
#include <asm/uaccess.h>
#include <asm/blackfin.h>
#include <asm/irq.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("TWI Keypad input driver");
MODULE_LICENSE("GPL");

#undef SENDASCII

#define BUTTONS 16

#ifdef SENDASCII
static unsigned char twi_keypad_btncode[BUTTONS + 1] = {
	[0] = KEY_RESERVED,
	[1] = 'd',
	[2] = '#',
	[3] = '0',
	[4] = '*',
	[5] = 'c',
	[6] = '9',
	[7] = '8',
	[8] = '7',
	[9] = 'b',
	[10] = '6',
	[11] = '5',
	[12] = '4',
	[13] = 'a',
	[14] = '3',
	[15] = '2',
	[16] = '1'
};
#else
static unsigned char twi_keypad_btncode[BUTTONS + 1] = {
	[0] = KEY_RESERVED,
	[1] = KEY_ENTER,
	[2] = KEY_BACKSLASH,
	[3] = KEY_0,
	[4] = KEY_RIGHTBRACE,
	[5] = KEY_C,
	[6] = KEY_9,
	[7] = KEY_8,
	[8] = KEY_7,
	[9] = KEY_B,
	[10] = KEY_6,
	[11] = KEY_5,
	[12] = KEY_4,
	[13] = KEY_A,
	[14] = KEY_3,
	[15] = KEY_2,
	[16] = KEY_1
};
#endif

struct twikeypad {
	unsigned char *btncode;
	struct input_dev *idev;
	char name[64];
	char phys[32];
	unsigned char laststate;
	unsigned char statechanged;
	unsigned long irq_handled;
	unsigned long events_sended;
	unsigned long events_processed;
	struct work_struct twi_keypad_work;
};

#define	PCF8574_KP_DRV_NAME		"pcf8574_kp"
static struct i2c_driver pcf8574_kp_driver;
static struct i2c_client *pcf8574_kp_client;

static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short normal_addr[] = { 0x27, I2C_CLIENT_END };

static struct i2c_client_address_data addr_data = {
	.normal_i2c = normal_addr,
	.probe = ignore,
	.ignore = ignore,
};

static short read_state(struct twikeypad *lp)
{
	unsigned char x, y, a, b;

	if (pcf8574_kp_client) {
		i2c_smbus_write_byte(pcf8574_kp_client, 240);
		x = 0xF & (~(i2c_smbus_read_byte(pcf8574_kp_client) >> 4));

		i2c_smbus_write_byte(pcf8574_kp_client, 15);
		y = 0xF & (~i2c_smbus_read_byte(pcf8574_kp_client));

		for (a = 0; x > 0; a++)
			x = x >> 1;
		for (b = 0; y > 0; b++)
			y = y >> 1;

		return (((a - 1) * 4) + b);
	}

	return -1;
}

static void check_and_notify(struct work_struct *work)
{
	struct twikeypad *lp =
	    container_of(work, struct twikeypad, twi_keypad_work);
	unsigned char nextstate = read_state(lp);

	pr_debug("%s\n", __FUNCTION__);

	lp->statechanged = lp->laststate ^ nextstate;

	if (lp->statechanged) {
		input_report_key(lp->idev,
				 nextstate > 17 ? lp->btncode[lp->laststate] :
				 lp->btncode[nextstate],
				 nextstate > 17 ? 0 : 1);

		lp->events_sended++;
	}
	lp->laststate = nextstate;
	input_sync(lp->idev);
	enable_irq(CONFIG_BFIN_TWIKEYPAD_IRQ_PFX);

}

static irqreturn_t twi_keypad_irq_handler(int irq, void *dev_id)
{
	struct twikeypad *lp = dev_id;

	pr_debug("%s\n", __FUNCTION__);
	disable_irq(CONFIG_BFIN_TWIKEYPAD_IRQ_PFX);
	schedule_work(&lp->twi_keypad_work);

	return IRQ_HANDLED;
}

static int init_twikeypad(struct i2c_client *client)
{
	int i, ret;
	struct input_dev *idev;
	struct twikeypad *lp;

	lp = kzalloc(sizeof(struct twikeypad), GFP_KERNEL);
	idev = input_allocate_device();

	if (!idev || !lp) {
		ret = -ENOMEM;
		goto fail;
	}

	if (request_irq(CONFIG_BFIN_TWIKEYPAD_IRQ_PFX, twi_keypad_irq_handler,
			IRQF_TRIGGER_LOW, PCF8574_KP_DRV_NAME, lp)) {

		printk(KERN_WARNING "twikeypad: IRQ %d is not free.\n",
		       CONFIG_BFIN_TWIKEYPAD_IRQ_PFX);
		ret = -EBUSY;
		goto fail;
	}

	i2c_set_clientdata(client, lp);

	lp->idev = idev;
	lp->btncode = twi_keypad_btncode;

	idev->evbit[0] = 0;

	idev->evbit[0] |= BIT(EV_KEY);
	idev->keycode = lp->btncode;
	idev->keycodesize = sizeof(twi_keypad_btncode);
	idev->keycodemax = ARRAY_SIZE(twi_keypad_btncode);

	for (i = 0; i <= BUTTONS; i++)
		set_bit(lp->btncode[i], idev->keybit);


	sprintf(lp->name, "BF5xx twikeypad");
	sprintf(lp->phys, "twikeypad/input0");

	idev->name = lp->name;
	idev->phys = lp->phys;
	idev->id.bustype = BUS_I2C;
	idev->id.vendor = 0x0001;
	idev->id.product = 0x0001;
	idev->id.version = 0x0100;

	input_register_device(lp->idev);

	lp->statechanged = 0x0;

	lp->laststate = read_state(lp);

	/* Set up our workqueue. */
	INIT_WORK(&lp->twi_keypad_work, check_and_notify);

	printk(KERN_INFO "input: %s at %s IRQ %d\n", lp->name, lp->phys,
				 CONFIG_BFIN_TWIKEYPAD_IRQ_PFX);

	return 0;

fail:
	input_free_device(idev);
	kfree(lp);
	i2c_set_clientdata(client, NULL);

	return -1;
}

static int pcf8574_kp_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *client;
	int rc;

	client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	memset(client, 0, sizeof(struct i2c_client));
	strncpy(client->name, PCF8574_KP_DRV_NAME, I2C_NAME_SIZE);
	client->addr = addr;
	client->adapter = adap;
	client->driver = &pcf8574_kp_driver;

	rc = i2c_attach_client(client);

	if (rc != 0) {
		kfree(client);
		printk(KERN_WARNING"i2c_attach_client fail: %d\n", rc);
		return rc;
	}

	pcf8574_kp_client = client;

	if (i2c_smbus_write_byte(pcf8574_kp_client, 240) < 0) {
		printk(KERN_WARNING"in keypad probe: write fail\n");
		return -1;
	}

	return init_twikeypad(client);
}

static int pcf8574_kp_attach(struct i2c_adapter *adap)
{
	if (adap->algo->functionality)
		return i2c_probe(adap, &addr_data, pcf8574_kp_probe);
	else
		return pcf8574_kp_probe(adap, 0x27, 0);
}

static int pcf8574_kp_detach_client(struct i2c_client *client)
{
	struct twikeypad *lp = i2c_get_clientdata(client);
	int rc;

	free_irq(CONFIG_BFIN_TWIKEYPAD_IRQ_PFX, lp);
	input_unregister_device(lp->idev);
	kfree(lp);

	rc = i2c_detach_client(client);
	if (rc == 0)
		kfree(i2c_get_clientdata(client));
	return rc;
}

static struct i2c_driver pcf8574_kp_driver = {
	.driver = {
		   .name = PCF8574_KP_DRV_NAME,
		   },
	.id = 0x65,
	.attach_adapter = pcf8574_kp_attach,
	.detach_client = pcf8574_kp_detach_client,
};

static int __init twi_keypad_init(void)
{
	return i2c_add_driver(&pcf8574_kp_driver);
}

void __exit twi_keypad_exit(void)
{
	i2c_del_driver(&pcf8574_kp_driver);
}

module_init(twi_keypad_init);
module_exit(twi_keypad_exit);
