/*
 * File:         drivers/media/video/blackfin/ov9655.c
 * Based on:
 * Author:       Michael Hennerich <hennerich@blackfin.uclinux.org>
 *
 * Created:
 * Description:  Command driver for Omnivision OV9655 sensor
 *
 *
 * Modified:
 *               10/2007 Omnivision 9655 basic image driver
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/wait.h>
#include <linux/videodev.h>
#include <media/v4l2-dev.h>

#include "ov9655.h"

static char myconfig[] = {
	0x11, 0x81,

};

static DEFINE_MUTEX(ov9655_sysfs_lock);

static int ReadWord(struct i2c_client *client, unsigned char offset, u16 *data)
{
	u8 buf[2];

	BUG_ON(client == NULL);

	i2c_smbus_write_byte(client, offset);
	i2c_master_recv(client, buf, 1);

	*data = buf[0];

	return 0;
}

static int WriteWord(struct i2c_client *client,
				 unsigned char offset, unsigned short data)
{
	u8 buf[2];

	BUG_ON(client == NULL);

	buf[0] = offset;
	buf[1] = data >> 8;

	i2c_master_send(client, buf, 2);

	return 0;
}

static unsigned short get_reg(struct i2c_client *client, unsigned char offset)
{
	u16 buf[1];

	ReadWord(client, offset, &buf[0]);

	return buf[0];
}

static int my_probe(struct i2c_client *client)
{

/*
	u8 buf[2];

	ReadWord(client, DEVICEID_MSB, &buf[0]);

	if (((buf[0] << 8) | (buf[1] & 0xFF)) == MT9V022_ID)
		return 0;

	return -ENODEV;
*/
	return 0;

}

static int my_set_pixfmt(struct i2c_client *client, u32 arg)
{
	return 0;
}

static int my_set_framerate(struct i2c_client *client, u32 arg)
{
	return -EPERM;
}

static int my_set_window(struct i2c_client *client, u32 res)
{
	return 0;
}

static int my_set_resolution(struct i2c_client *client, u32 res)
{

	switch (res) {

	default:
		my_set_window(client, res);

	}

	return 0;
}

static int my_init(struct i2c_client *client, u32 arg)
{
	const char *b, *end;

	if (my_probe(client))
		return -ENODEV;

	WriteWord(client, 0x12, 0x80); /* reset */

	msleep(100);

	b = myconfig;
	end = &b[sizeof(myconfig)];

	while (b < end) {
		WriteWord(client, b[0], b[1]);
		b += 2;
	}

	return 0;

}

static int my_exit(struct i2c_client *client, u32 arg)
{
	return 0;
}

int cam_control(struct i2c_client *client, u32 cmd, u32 arg)
{
	switch (cmd) {
	case CAM_CMD_INIT:
		return my_init(client, arg);
	case CAM_CMD_SET_RESOLUTION:
		return my_set_resolution(client, arg);
	case CAM_CMD_SET_FRAMERATE:
		return my_set_framerate(client, arg);
	case CAM_CMD_SET_PIXFMT:
		return my_set_pixfmt(client, arg);
	case CAM_CMD_EXIT:
		return my_exit(client, arg);
	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

/****************************************************************************
 *  sysfs
 ***************************************************************************/

static u8 sysfs_strtou8(const char *buff, size_t len, ssize_t *count)
{
	char str[5];
	char *endp;
	unsigned long val;

	if (len < 4) {
		strncpy(str, buff, len);
		str[len + 1] = '\0';
	} else {
		strncpy(str, buff, 4);
		str[4] = '\0';
	}

	val = simple_strtoul(str, &endp, 0);

	*count = 0;
	if (val <= 0xff)
		*count = (ssize_t) (endp - str);
	if ((*count) && (len == *count + 1) && (buff[*count] == '\n'))
		*count += 1;

	return (u8) val;
}

static ssize_t sysfs_show_val(struct device *cd, struct device_attribute *attr, char *buf, int cmd)
{
	struct bcap_device_t *cam;
	ssize_t count;
	u8 val[1];

	if (mutex_lock_interruptible(&ov9655_sysfs_lock))
		return -ERESTARTSYS;

	cam = video_get_drvdata(to_video_device(cd));
	if (!cam) {
		mutex_unlock(&ov9655_sysfs_lock);
		return -ENODEV;
	}

	if (cam_control(cam->client, cmd, (u32) val) < 0) {
		mutex_unlock(&ov9655_sysfs_lock);
		return -EIO;
	}

	count = sprintf(buf, "%d\n", val[0]);

	mutex_unlock(&ov9655_sysfs_lock);

	return count;
}

static ssize_t
sysfs_store_val(struct device *cd, struct device_attribute *attr, const char *buf, size_t len,
		      int cmd)
{
	struct bcap_device_t *cam;
	u8 value;
	ssize_t count;
	int err;

	if (mutex_lock_interruptible(&ov9655_sysfs_lock))
		return -ERESTARTSYS;

	cam = video_get_drvdata(to_video_device(cd));

	if (!cam) {
		mutex_unlock(&ov9655_sysfs_lock);
		return -ENODEV;
	}

	value = sysfs_strtou8(buf, len, &count);

	if (!count) {
		mutex_unlock(&ov9655_sysfs_lock);
		return -EINVAL;
	}

	err = cam_control(cam->client, cmd, value);

	if (err) {
		mutex_unlock(&ov9655_sysfs_lock);
		return -EIO;
	}

	mutex_unlock(&ov9655_sysfs_lock);

	return count;
}

static ssize_t sysfs_fps_show(struct device *cd, struct device_attribute *attr, char *buf)
{

	return sysfs_show_val(cd, attr, buf, CAM_CMD_GET_FRAMERATE);
}

static ssize_t
sysfs_fps_store(struct device *cd, struct device_attribute *attr, const char *buf, size_t len)
{
	return sysfs_store_val(cd, attr, buf, len, CAM_CMD_SET_FRAMERATE);
}

static DEVICE_ATTR(fps, S_IRUGO | S_IWUSR,
			 sysfs_fps_show, sysfs_fps_store);

static ssize_t sysfs_flicker_show(struct device *cd, struct device_attribute *attr, char *buf)
{
	return sysfs_show_val(cd, attr, buf, CAM_CMD_GET_FLICKER_FREQ);
}

static ssize_t
sysfs_flicker_store(struct device *cd, struct device_attribute *attr, const char *buf, size_t len)
{
	return sysfs_store_val(cd, attr, buf, len, CAM_CMD_SET_FLICKER_FREQ);
}

static DEVICE_ATTR(flicker, S_IRUGO | S_IWUSR,
			 sysfs_flicker_show, sysfs_flicker_store);

static ssize_t sysfs_h_mirror_show(struct device *cd, struct device_attribute *attr, char *buf)
{
	return sysfs_show_val(cd, attr, buf, CAM_CMD_GET_HOR_MIRROR);
}

static ssize_t
sysfs_h_mirror_store(struct device *cd, struct device_attribute *attr, const char *buf, size_t len)
{
	return sysfs_store_val(cd, attr, buf, len, CAM_CMD_SET_HOR_MIRROR);
}

static DEVICE_ATTR(h_mirror, S_IRUGO | S_IWUSR,
			 sysfs_h_mirror_show, sysfs_h_mirror_store);

static ssize_t sysfs_v_mirror_show(struct device *cd, struct device_attribute *attr, char *buf)
{
	return sysfs_show_val(cd, attr, buf, CAM_CMD_GET_VERT_MIRROR);
}

static ssize_t
sysfs_v_mirror_store(struct device *cd, struct device_attribute *attr, const char *buf, size_t len)
{
	return sysfs_store_val(cd, attr, buf, len, CAM_CMD_SET_VERT_MIRROR);
}

static DEVICE_ATTR(v_mirror, S_IRUGO | S_IWUSR,
			 sysfs_v_mirror_show, sysfs_v_mirror_store);

static int ov9655_create_sysfs(struct video_device *v4ldev)
{

	int rc;

	rc = device_create_file(&v4ldev->dev, &dev_attr_fps);
	if (rc)
		goto err;
	rc = device_create_file(&v4ldev->dev, &dev_attr_flicker);
	if (rc)
		goto err_flicker;
	rc = device_create_file(&v4ldev->dev, &dev_attr_v_mirror);
	if (rc)
		goto err_v_mirror;
	rc = device_create_file(&v4ldev->dev, &dev_attr_h_mirror);
	if (rc)
		goto err_h_mirror;

	return 0;

err_h_mirror:
	device_remove_file(&v4ldev->dev, &dev_attr_v_mirror);
err_v_mirror:
	device_remove_file(&v4ldev->dev, &dev_attr_flicker);
err_flicker:
	device_remove_file(&v4ldev->dev, &dev_attr_fps);
err:
	return rc;
}

static struct bcap_camera_ops ov9655_ops = {
	cam_control,
	ov9655_create_sysfs,
	NULL,
};

struct bcap_camera_ops *get_camops(void)
{
	return (&ov9655_ops);

}
EXPORT_SYMBOL(get_camops);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
