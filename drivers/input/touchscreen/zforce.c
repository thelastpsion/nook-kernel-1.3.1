/* drivers/input/touchscreen/zforce.c
 *
 * Copyright (C) 2010 Barnes & Noble, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/platform_device.h>
#include <linux/zforce.h>
#include <linux/device.h>
#include <linux/sysfs.h>

#define WAIT_TIMEOUT		msecs_to_jiffies(500)

#define FRAME_START		0xEE
#undef ZF_USE_DEBUG

// Payload offsets
#define PAYLOAD_HEADER		0
#define PAYLOAD_LENGTH		1
#define PAYLOAD_BODY		2


// Response offsets
#define RESPONSE_ID		0
#define RESPONSE_DATA		1

// Commandszforce
#define COMMAND_DEACTIVATE	0x00
#define COMMAND_ACTIVATE	0x01
#define COMMAND_RESOLUTION	0x02
#define COMMAND_SETCONFIG	0x03
#define COMMAND_DATAREQUEST	0x04
#define COMMAND_SCANFREQ	0x08
#define COMMAND_VERSION		0x0A
#define COMMAND_PULSESTRENG	0x0F
#define COMMAND_FORCECAL	0X1A
#define COMMAND_LEVEL		0x1C
#define COMMAND_STATUS		0X1E
#define COMMAND_FORCED_LEVEL	0X20
#define COMMAND_OPEN_SHORT	0X21


// Responses
#define RESPONSE_DEACTIVATE	0x00
#define RESPONSE_ACTIVATE	0x01
#define RESPONSE_RESOLUTION	0x02
#define RESPONSE_SETCONFIG	0x03
#define RESPONSE_DATAREQUEST	0x04
#define RESPONSE_BOOTCOMPLETE	0x07
#define RESPONSE_SCANFREQ	0x08
#define RESPONSE_VERSION	0x0A
#define RESPONSE_PULSESTRENG	0x0F
#define RESPONSE_FORCECAL	0x1A
#define RESPONSE_LEVEL		0x1C
#define RESPONSE_STATUS		0X1E
#define RESPONSE_OPEN_SHORT	0X21
#define RESPONSE_OVERRUN	0X25
#define RESPONSE_INVALID	0xFE


// Size of data in Status Command response
#define ZF_STATUS_SIZE_V3	 64
#define ZF_STATUS_SIZE		128

// Platform specific
#define ZF_COORDATA_SIZE_V1 5
#define ZF_COORDATA_SIZE_V3 7
#define ZF_COORDATA_SIZE    9
#define X_AXIS              0
#define Y_AXIS              1

#define ZF_OPEN_SHORT_XDATA_LEN 7
#define ZF_OPEN_SHORT_YDATA_LEN 9

#define ZF_MAX_CMD_DATA_SIZE 16

#define I2C_M_WR                        0

#define ZF_SETCONFIG_DUALTOUCH 0x00000001

DEFINE_MUTEX(zForce_sysfs_mutex);

static int zforce_synchronized_wait_for_completion_timeout(struct completion *res);

static struct workqueue_struct *zforce_wq;

struct zforce {
	struct input_dev	*input;
	struct i2c_client	*client;
	struct completion	command_done;
	int			command_result;
	int			irq;
	struct work_struct	work;
	struct work_struct	reset_work;
	struct work_struct	initialization_work;
	char			phys[32];
	u8	zf_status_info[128];
	u8	zf_open_short_xsize;
	u8	zf_open_short_xresult[32];
	u8	zf_open_short_ysize;
	u8	zf_open_short_yresult[32];
	u16	idle_freq;
	u16	active_freq;
	u16	pen_freq;
	int	err_cnt;
};

struct zforce_comm_packet {

	unsigned char frame_start;
	unsigned char size;
	unsigned char command;
	unsigned char command_data[ZF_MAX_CMD_DATA_SIZE];
};

static u16 major = 0, minor = 0, build = 0, rev = 0;
static u8 reported_finger_count = 0;
static u8 modified_freqs = 0;

static int touch_data_size = 0;

#define MAX_ALLOWED_INVALID_FRAMES 20
static int invalid_frame_counter = 0;
static int invalid_frame_count_exceeded = 0;

static u8 fixps_stren	= 0;
static u8 fixps_time	= 0;

// 0 = no debugging messages printed
// 1 = some debugging messages
// 2 = all debugging messages
static int zf_debug = 0;

static void zforce_touch_hw_init(int);

#if defined (CONFIG_MACH_OMAP3621_GOSSAMER)
static int zforce_suspend(struct i2c_client *client, pm_message_t mesg);
static int zforce_resume(struct i2c_client *client);
#endif

static u8 reset_source = 0;
static u8 wdg_reset_state = 0;
static int irq_is_enabled = 0;


static int zforce_send_command ( struct i2c_client * client , int command , char * data , unsigned char size)
{
	struct zforce_comm_packet cmd_packet;
	int ret = -EINVAL;
	int i = 0;
	struct i2c_msg msg[1];

	cmd_packet.frame_start = FRAME_START;
	cmd_packet.size = 1 + size; // Size is the command byte plus data bytes
	cmd_packet.command = command;

	if ( (size <= ZF_MAX_CMD_DATA_SIZE )  && ( client != NULL ) ){
		if ( data != NULL ) {
			memcpy( &cmd_packet.command_data[0], data, size);
		}

		msg[0].addr = client->addr;
		msg[0].flags = I2C_M_WR;
		if (major <= 3) {
			// Firmware v3.x and earlier only requires the actual
			// command byte and its data
			msg[0].buf = (unsigned char*) &cmd_packet.command;
			msg[0].len = cmd_packet.size;
		}
		else {
			// Firmware v4.x requires header and byte count
			// before the command
			msg[0].buf = (unsigned char*) &cmd_packet;
			msg[0].len = 2 + cmd_packet.size;
		}

		if (zf_debug >= 2) dev_info(&client->dev, "%s: addr = 0x%X, cmd = 0x%02X; total len = %d; cmd_packet.size = %d\n", __FUNCTION__, msg[0].addr, cmd_packet.command, msg[0].len, cmd_packet.size);

		if (zf_debug >= 1) {
			printk("Packet OUT (COMMAND_%s) =",
			       (command == COMMAND_DEACTIVATE) ? "DEACTIVATE" :
			       (command == COMMAND_ACTIVATE) ? "ACTIVATE" :
			       (command == COMMAND_RESOLUTION) ? "RESOLUTION" :
			       (command == COMMAND_SETCONFIG) ? "SETCONFIG" :
			       (command == COMMAND_DATAREQUEST) ? "DATAREQUEST" :
			       (command == COMMAND_SCANFREQ) ? "SCANFREQ" :
			       (command == COMMAND_VERSION) ? "VERSION" :
			       (command == COMMAND_PULSESTRENG) ? "PULSESTRENG" :
			       (command == COMMAND_FORCECAL) ? "FORCECAL" :
			       (command == COMMAND_LEVEL) ? "LEVEL" :
			       (command == COMMAND_STATUS) ? "STATUS" :
			       (command == COMMAND_FORCED_LEVEL) ? "FORCED_LEVEL" :
			       (command == COMMAND_OPEN_SHORT) ? "OPEN_SHORT" :
			       "UNKNOWN");
			for (i = 0; i < msg[0].len; i++) {
				printk(" %02X", msg[0].buf[i]);
			}
			printk("\n");
		}

		ret = i2c_transfer(client->adapter, msg, 1);

		if ( ret >= 0 ) {
			ret = 0;
		}
	}

	return ret;
}

// DATA Request
// [1:cmd]
// #######
static int send_data_request(struct zforce *tsc)
{
	int ret = 0;
	int retry = 3;

	if (zf_debug >= 2) dev_info(&tsc->client->dev, "%s()\n", __FUNCTION__);

	while (retry-- > 0)
	{
		ret = zforce_send_command(tsc->client, COMMAND_DATAREQUEST, NULL, 0);
		if (ret >= 0)
			break;
	}
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send data request error: %d\n", ret);
	}
	return ret;
}

// RESOLUTION Request
// [1:cmd] [2:width] [2:height]
// ############################
static int send_resolution(struct zforce *tsc, u16 width, u16 height)
{
	u8 request[16];
	int ret = 0;

	dev_info(&tsc->client->dev, "%s(%d,%d)\n", __FUNCTION__, width, height);

	memcpy(&request[0], &width, sizeof(u16));
	memcpy(&request[2], &height, sizeof(u16));

	ret = zforce_send_command( tsc->client, COMMAND_RESOLUTION, request, 4);

	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send resolution error: %d\n", ret);
		return ret;
	}
	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0) {
		dev_err(&tsc->client->dev, "Timed out waiting for RESPONSE_RESOLUTION\n");
		return -ETIMEDOUT;
	}

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	return ret;
}

// SCANNING FREQUENCY Request
// [1:cmd] [2:idle] [2:active] [2:pen]
// ###################################
static int send_scan_freqs(struct zforce *tsc, u16 idle_freq, u16 active_freq, u16 pen_freq)
{
	u8 request[24];
	int ret = 0;

	if (zf_debug >= 2) dev_info(&tsc->client->dev, "%s(%d,%d,%d)\n", __FUNCTION__, idle_freq, active_freq, pen_freq);

	memcpy(&request[0], &idle_freq, sizeof(u16));
	memcpy(&request[2], &active_freq, sizeof(u16));
	memcpy(&request[4], &pen_freq, sizeof(u16));

	ret = zforce_send_command( tsc->client, COMMAND_SCANFREQ, request, 6);

	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send scan frequencies error: %d\n", ret);
		return ret;
	}
	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0) {
		dev_err(&tsc->client->dev, "Timed out waiting for RESPONSE_SCANFREQ\n");
		return -ETIMEDOUT;
	}

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	if (tsc->command_result)
		dev_err(&tsc->client->dev, "RESPONSE_SCANFREQ returned %d\n", tsc->command_result);
	return ret;
}

// SETCONFIGURATION Request
// [1:cmd] [2:width] [2:height]
// ############################
static int send_setconfig(struct zforce *tsc, u32 setconfig)
{
	u8 request[16];
	int ret = 0;

	dev_info(&tsc->client->dev, "%s(%d)\n", __FUNCTION__, setconfig);

	memcpy(&request[0], &setconfig, sizeof(u32));

	ret = zforce_send_command(tsc->client, COMMAND_SETCONFIG, request, 4);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send setconfig error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0) {
		dev_err(&tsc->client->dev, "Timed out waiting for RESPONSE_SETCONFIG\n");
		return -ETIMEDOUT;
	}

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	if (tsc->command_result)
		dev_err(&tsc->client->dev, "RESPONSE_SETCONFIG returned %d\n", tsc->command_result);
	return tsc->command_result;
}

// Fixed Pulse and Strength Request - firmware v2 and earlier
// ############################
static int send_pulsestreng_v2(struct zforce *tsc, u8 strength, u8 time)
{
	u8 request[16];
	int ret = 0;

	dev_info(&tsc->client->dev, "%s(%d,%d)\n", __FUNCTION__, strength, time);

	request[0] = (strength&0x0F) | ( time<<4 ) ;
	ret = zforce_send_command(tsc->client, COMMAND_PULSESTRENG, request, 1);

	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "send pulsestreng error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0) {
		dev_err(&tsc->client->dev, "Timed out waiting for RESPONSE_PULSESTRENG\n");
		return -ETIMEDOUT;
	}

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	return tsc->command_result;
}


// Fixed Pulse and Strength Request -firmware v 3.x and later
// ############################
static int send_pulsestreng(struct zforce *tsc, u8 strength, u8 axis)
{
	u8 request[16];
	int ret = 0;

	dev_info(&tsc->client->dev, "%s (0x%02X) %s axis\n", __FUNCTION__,
	         strength, (axis == X_AXIS) ? "X" : "Y");

	request[0] = axis;
	request[1] = strength;
	ret = zforce_send_command(tsc->client, COMMAND_PULSESTRENG, request, 2);

	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "send pulsestreng error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0) {
		dev_err(&tsc->client->dev, "Timed out waiting for RESPONSE_PULSESTRENG\n");
		return -ETIMEDOUT;
	}

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	return tsc->command_result;
}


// Status Request
// ############################
static int send_status_request(struct zforce *tsc)
{
	int ret;

	if (zf_debug >= 2) dev_info(&tsc->client->dev, "%s\n", __FUNCTION__);
	
	ret = zforce_send_command(tsc->client, COMMAND_STATUS, NULL, 0);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "send status request error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0) {
		dev_err(&tsc->client->dev, "Timed out waiting for RESPONSE_STATUS\n");
		return -ETIMEDOUT;
	}

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	if (tsc->command_result)
		dev_err(&tsc->client->dev, "RESPONSE_STATUS returned %d\n", tsc->command_result);
	return tsc->command_result;
}

// OPEN_SHORT Request
// ############################
static int send_open_short_request(struct zforce *tsc, u8 axis)
{
	u8 request[2];
	int ret;

	dev_info(&tsc->client->dev, "%s (%s axis)\n", __FUNCTION__, (axis == X_AXIS) ? "X" : "Y");

	request[0] = axis;

	ret = zforce_send_command(tsc->client, COMMAND_OPEN_SHORT, request, 1);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send OPEN_SHORT request error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0) {
		dev_err(&tsc->client->dev, "Timed out waiting for RESPONSE_OPEN_SHORT\n");
		return -ETIMEDOUT;
	}

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	if (tsc->command_result)
		dev_err(&tsc->client->dev, "RESPONSE_OPEN_SHORT returned %d\n", tsc->command_result);
	return tsc->command_result;
}

// Send Command Request
// [0:cmd]
// #######
static int send_cmd_request(struct zforce *tsc, u8 cmd)
{
	int ret;
	int retry = 3;

	while (retry-- > 0)
	{
		ret = zforce_send_command(tsc->client, cmd, NULL, 0);
		if (ret >= 0)
			break;
	}
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send cmd(%X) request error: %d\n", cmd, ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0) {
		dev_err(&tsc->client->dev, "Timed out waiting for response to MSP command 0x%02X\n", cmd);
		return -ETIMEDOUT;
	}

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	if (tsc->command_result)
		dev_err(&tsc->client->dev, "MSP command 0x%02X returned %d\n", cmd, tsc->command_result);
	return tsc->command_result;
}

// DEACTIVATE Request
// [0:cmd]
// #######
static int send_deactivate_request(struct zforce *tsc)
{
	int ret = 0;
	int retry = 3;

	if (zf_debug >= 2) dev_info(&tsc->client->dev, "%s()\n", __FUNCTION__);

	while (retry-- > 0)
	{
		ret = zforce_send_command(tsc->client, COMMAND_DEACTIVATE, NULL, 0);
		if (ret >= 0)
			break;
	}
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send deactivate request error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0) {
		dev_err(&tsc->client->dev, "Timed out waiting for RESPONSE_DEACTIVATE\n");
		return -ETIMEDOUT;
	}

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	if (tsc->command_result)
		dev_err(&tsc->client->dev, "RESPONSE_DEACTIVATE returned %d\n", tsc->command_result);
	return tsc->command_result;
}


// ACTIVATE Request
// [1:cmd]
// #######
static int send_activate_request(struct zforce *tsc)
{
	int ret = 0 ;
	int retry = 3;

	if (zf_debug >= 2) dev_info(&tsc->client->dev, "%s()\n", __FUNCTION__);

	while (retry-- > 0)
	{
		ret = zforce_send_command(tsc->client, COMMAND_ACTIVATE, NULL, 0);
		if (ret >= 0)
			break;
	}
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send activate request error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0) {
		dev_err(&tsc->client->dev, "Timed out waiting for RESPONSE_ACTIVATE\n");
		return -ETIMEDOUT;
	}

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	if (tsc->command_result)
		dev_err(&tsc->client->dev, "RESPONSE_ACTIVATE returned %d\n", tsc->command_result);
	return tsc->command_result;
}

// Force Calibration Request
// [1:cmd]
// #######
static int send_forcecal_request(struct zforce *tsc)
{
	int ret;

	if (zf_debug >= 2) dev_info(&tsc->client->dev, "%s()\n", __FUNCTION__);

	ret = zforce_send_command(tsc->client, COMMAND_FORCECAL, NULL, 0);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send force calibration request error: %d\n", ret);
		return ret;
	}

	return tsc->command_result;
}

// Scanning Frequencies Request
// [1:cmd]
// #######
static int get_scan_freqs(struct zforce *tsc)
{
	u8 *payload = (u8 *)tsc->zf_status_info;

	if (zf_debug >= 2) dev_info(&tsc->client->dev, "%s()\n", __FUNCTION__);

	if (major < 4) {
		printk("get_scan_freqs: not supported for FW versions before 4.x\n");
		return -1;
	}

	if (send_status_request(tsc))
	{
		dev_err(&tsc->client->dev, "Unable retrieve zforce status.\n");
		return -EINVAL;
	}

	// Extract values from the buffer
	memcpy(&tsc->idle_freq, &payload[21], sizeof(u16));
	memcpy(&tsc->active_freq, &payload[23], sizeof(u16));
	memcpy(&tsc->pen_freq, &payload[25], sizeof(u16));

	dev_info(&tsc->client->dev, "idle_freq = %d: active_freq = %d: pen_freq = %d\n", tsc->idle_freq, tsc->active_freq, tsc->pen_freq);

	return tsc->command_result;
}

// Version Request
// [1:cmd]
// #######
static int send_version_request(struct zforce *tsc)
{
	int ret;

	if (zf_debug >= 2) dev_info(&tsc->client->dev, "%s()\n", __FUNCTION__);

	if (send_status_request(tsc))
	{
		// The following is for backwards compatibility with zForce
		// firmware versions 2.X and earlier
		ret = zforce_send_command(tsc->client, COMMAND_VERSION, NULL, 0);
		if (ret < 0)
		{
		dev_err(&tsc->client->dev, "i2c send version request error: %d\n", ret);
			return ret;
		}

		if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0) {
			dev_err(&tsc->client->dev, "Timed out waiting for RESPONSE_VERSION\n");
			return -ETIMEDOUT;
		}
	}

	if (tsc->command_result)
		dev_err(&tsc->client->dev, "RESPONSE_VERSION returned %d\n", tsc->command_result);
	return tsc->command_result;
}

// Version Payload Results
// [2:major] [2:minor] [2:build] [2:rev]
// #####################################
static int process_version_response(struct zforce *tsc, u8* payload)
{
	memcpy(&major, &payload[0], sizeof(u16));
	memcpy(&minor, &payload[2], sizeof(u16));
	memcpy(&build, &payload[4], sizeof(u16));
	memcpy(&rev,   &payload[6], sizeof(u16));

	// Set the value of each touch data structure, which depends on the
	// firmware version
	touch_data_size = (major == 1) ? ZF_COORDATA_SIZE_V1 :
	                 ((major <= 3) ? ZF_COORDATA_SIZE_V3 : ZF_COORDATA_SIZE);

	dev_info(&tsc->client->dev, "Firmware Version: %d.%d.%d.%d\n", major, minor, build, rev);
	return 8;
}

// LED LEVEL Request - firmware v2 and earlier
// [1:cmd]
// #######
static int send_level_request_v2(struct zforce *tsc)
{
	int ret;

	if (zf_debug >= 2) dev_info(&tsc->client->dev, "%s()\n", __FUNCTION__);

	ret = zforce_send_command(tsc->client, COMMAND_LEVEL, NULL, 0);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send level request error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0) {
		dev_err(&tsc->client->dev, "Timed out waiting for RESPONSE_LEVEL\n");
		return -ETIMEDOUT;
	}

	return tsc->command_result;
}

// LED LEVEL Request - firmware v3.x and later
// [1:cmd]
// #######
// Fixed Pulse and Strength Request -firmware v 3.x and later
// ############################

static int send_level_request(struct zforce *tsc, u8 axis)
{
	u8 request[16];
	int ret = 0;

	if (zf_debug >= 2) dev_info(&tsc->client->dev, "%s(%d)\n", __FUNCTION__, axis);

	request[0] = axis;

	ret = zforce_send_command(tsc->client, COMMAND_FORCED_LEVEL, request, 1);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send level request error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0) {
		dev_err(&tsc->client->dev, "Timed out waiting for RESPONSE_LEVEL\n");
		return -ETIMEDOUT;
	}

	return tsc->command_result;
}

#define ZF_NUMX 11
#define ZF_NUMY 15
#define ZF_LEDDATA_LEN  256

static u8 ledlevel_x[ZF_LEDDATA_LEN];
static u8 ledlevel_y[ZF_LEDDATA_LEN];

// LED Level Payload Results
// v2 - [1:x] [1:y] [3*x:xdata] [3*y:ydata]
// v3+ - [1:axis] [1:num] [3*num:axisdata]
// #####################################
static int process_level_response(struct zforce *tsc, u8* payload, int payload_size)
{
	int i = 0;
	int payload_remain = payload_size;
	int x_data_start = 0;
	int y_data_start = 0;
	int x_data_count = 0;
	int y_data_count = 0;

	if (zf_debug >= 2) dev_info(&tsc->client->dev, "%s()\n", __FUNCTION__);

	if (payload_remain < 2) {
		dev_err(&tsc->client->dev, "ERROR: LED level payload too small\n");
		return -EINVAL;
	}

	if (major <= 2) {
		x_data_count = payload[0];
		y_data_count = payload[1];
		ledlevel_x[0] = payload[0];  // Number of X-axis signals
		ledlevel_y[0] = payload[1];  // Number of Y-axis signals
		x_data_start = 2;
		y_data_start = 2 + 3*ledlevel_x[0];
	}
	else {
		x_data_count = (payload[0] == X_AXIS) ? payload[1]/2 : 0;
		y_data_count = (payload[0] == Y_AXIS) ? payload[1]/2 : 0;
		ledlevel_x[0] = (x_data_count > 0) ? x_data_count : ledlevel_x[0];  // Number of X-axis signals
		ledlevel_y[0] = (y_data_count > 0) ? y_data_count : ledlevel_y[0];  // Number of Y-axis signals
		x_data_start = (payload[0] == X_AXIS) ? 2 : 0;
		y_data_start = (payload[0] == Y_AXIS) ? 2 : 0;
	}

	if (zf_debug >= 1) dev_info(&tsc->client->dev, "ledlevel_x[0] = %d, x_data_start = %d, ledlevel_y[0] = %d, y_data_start = %d\n", ledlevel_x[0], x_data_start, ledlevel_y[0], y_data_start);

	// Save data for each axis
	for (i = 0; i < (3 * x_data_count); i++ )
	{
		if (payload_remain < 1) {
			dev_err(&tsc->client->dev, "ERROR: LED level payload too small (x-axis)\n");
			return -EINVAL;
		}
		ledlevel_x[i + 1] = payload[x_data_start + i];
		payload_remain--;
	}
	for (i = 0; i < (3 * y_data_count); i++ )
	{
		if (payload_remain < 1) {
			dev_err(&tsc->client->dev, "ERROR: LED level payload too small (y-axis)\n");
			return -EINVAL;
		}
		ledlevel_y[i + 1] = payload[y_data_start + i];
		payload_remain--;
	}


	return (2+ 3*(ledlevel_x[0] + ledlevel_y[0]));
}


#define STATE_DOWN 0
#define STATE_MOVE 1
#define STATE_UP   2

struct touch_info_data_t tinfo[ZF_NUM_FINGER_SUPPORT];
//
// Clear touch info control block
//
void touchdata_clear(void)
{
	u8 i;
	for( i=0; i< ZF_NUM_FINGER_SUPPORT; i++ )
	{
		tinfo[i].x		= 0;
		tinfo[i].y		= 0;
		tinfo[i].id		= 0;
		tinfo[i].state	= 0;
		tinfo[i].valid	= 0;
		tinfo[i].rsvrd	= 0;
		tinfo[i].prblty	= 0;
		tinfo[i].z		= 0;
	}
}

//
// return the number of touches
// return error otherwise
static u32 framecounter = 0;
int touchdata_collect( u8* payload )
{
	u8 i;
	reported_finger_count = payload[0];
	framecounter++;
	if( reported_finger_count > ZF_NUM_FINGER_SUPPORT )
	{
		zforce_error("Detected (%d) more fingers the max(%d) number supported\n",
				reported_finger_count, ZF_NUM_FINGER_SUPPORT );
		return -EINVAL ;
	}

	for( i=0; i< reported_finger_count; i++ )
	{
		tinfo[i].x	= (u16)((payload[2+i*touch_data_size]<<8)|
					payload[1+i*touch_data_size]);
		tinfo[i].y	= (u16)((payload[4+i*touch_data_size]<<8)|
					payload[3+i*touch_data_size]);
		tinfo[i].id	= (u8)((major <= 3) ? ((payload[5+i*touch_data_size]&0x3C)>>2) :
		                                  ((payload[5+i*touch_data_size]&0xFC)>>2));
		tinfo[i].state	= (u8)((major <= 3) ? ((payload[5+i*touch_data_size]&0xC0)>>6) :
		                                  (payload[5+i*touch_data_size]&0x03));
		tinfo[i].rsvrd	= (u8)( payload[6+i*touch_data_size] );
		tinfo[i].prblty	= (u8)((major <= 3) ? payload[7+i*touch_data_size] :
		                                      payload[9+i*touch_data_size] );
		tinfo[i].valid	= 1;
		tinfo[i].z = reported_finger_count == 0 ? 0 : 20;
	}
	return reported_finger_count;
}

//
// display touch data buffer
//
#ifdef ZF_USE_DEBUG
void touchdata_show(void)
{
	int i = 0;

	if (major > 1)
	{
		printk("NumFingers=%02d\n", reported_finger_count);
		for(i=0; i<reported_finger_count; i++)
		{
			printk("[%05d](%03d, %03d, %02d) DMU%d ID%d R%02X P%02X V%02d\n",
				framecounter,
				tinfo[i].x,
				tinfo[i].y,
				tinfo[i].z,
				tinfo[i].state,
				tinfo[i].id,
				tinfo[i].rsvrd,
				tinfo[i].prblty,
				tinfo[i].valid );
		}
		printk("\n");
	}
}
#endif

// Fix Pulse Strength  Payload Results
// [1:x] [1:y] [3*x:xdata] [3*y:ydata]
// #####################################
#define ZF_FIXSP_BUFF_SIZE (ZF_NUMX*2+ZF_NUMY*2+2)
static u8 fixps_data_x[ZF_FIXSP_BUFF_SIZE];
static u8 fixps_data_y[ZF_FIXSP_BUFF_SIZE];

static int process_pulsestreng_response(struct zforce *tsc, u8* payload)
{
	int i = 0;
	int numx = -1, numy = -1;
	int x_offset = 0, y_offset = 0;
	int datasize;

	if (zf_debug >= 2) dev_info(&tsc->client->dev, "%s()\n", __FUNCTION__);

	if (major <= 2)
	{
		numx = payload[0];
		numy = payload[1];
		datasize = (numx + numy + 2);
	}
	else {
		numx = (payload[0] == X_AXIS) ? payload[1] : 0;
		numy = (payload[0] == Y_AXIS) ? payload[1] : 0;
		datasize = (numx + numy + 2);
	}
	if( (major <= 2) && (datasize != ZF_FIXSP_BUFF_SIZE) )
	{
		dev_err(&tsc->client->dev, "fixps buffer mismatch.(E%d, G%d)(TC%d).\n", ZF_FIXSP_BUFF_SIZE, datasize, ++(tsc->err_cnt) );
	}
	if( datasize > ZF_FIXSP_BUFF_SIZE )
	{
		dev_err(&tsc->client->dev, "fixps buff overflow:(E%d, G%d)(TC%d).\n", ZF_FIXSP_BUFF_SIZE, datasize, ++(tsc->err_cnt) );
		// Clamp datasize to prevent buffer overflow
		datasize = ZF_FIXSP_BUFF_SIZE;
		
	}

	// Save fix pulse strength data; first the byte count
	if (major <= 2)
	{
		fixps_data_x[0] = payload[0];
		fixps_data_y[0] = payload[1];
		x_offset = 1;
		y_offset = 1 + numx;
	}
	else
	{
		fixps_data_x[0] = (payload[0] == X_AXIS) ? payload[1] : fixps_data_x[0];
		fixps_data_y[0] = (payload[0] == Y_AXIS) ? payload[1] : fixps_data_y[0];
		x_offset = 1;
		y_offset = 1;
	}

	for(i = 1; i <= numx; i++)
	{
		fixps_data_x[i] = payload[i + x_offset];
	}
	for(i = 1; i <= numy; i++)
	{
		fixps_data_y[i] = payload[i + y_offset];
	}
	return ZF_FIXSP_BUFF_SIZE;
}

static int process_status_response(struct zforce *tsc, u8* payload)
{
	u8 *pyld =  NULL;
	
	if (zf_debug >= 2) dev_info(&tsc->client->dev, "%s()\n", __FUNCTION__);
	
	pyld = (u8 *)tsc->zf_status_info;
	if (*pyld <= 3) {
		memcpy(pyld, payload, ZF_STATUS_SIZE_V3);
	}
	else {
		memcpy(pyld, payload, ZF_STATUS_SIZE);
	}

	// The version command was replaced by the get status command.
	// We now parse the status response in order to obtain the version number
	process_version_response(tsc, payload);
	
	return ZF_STATUS_SIZE;
}

static int process_open_short_response(struct zforce *tsc, const u8* payload)
{
	u8 *pyld =  NULL;
	u8 len = 0;

	if (zf_debug >= 2) dev_info(&tsc->client->dev, "%s()\n", __FUNCTION__);

	if( payload[0] == X_AXIS )
	{
		pyld = (u8 *)tsc->zf_open_short_xresult;
		if (major <= 3) {
			len = ZF_OPEN_SHORT_XDATA_LEN;
		}
		else {
			// 1 byte for axis, 1 byte for signal count, plus
			// 2 bits per axis signal; calculate number of bytes
			len = 2 + (((2*payload[1]) + 7) / 8);
		}
		tsc->zf_open_short_xsize = len;
	}
	else if( payload[0] == Y_AXIS )
	{
		pyld = (u8 *)tsc->zf_open_short_yresult;
		if (major <= 3) {
			len = ZF_OPEN_SHORT_YDATA_LEN;
		}
		else {
			// 1 byte for axis, 1 byte for signal count, plus
			// 2 bits per axis signal; calculate number of bytes
			len = 2 + (((2*payload[1]) + 7) / 8);
		}
		tsc->zf_open_short_ysize = len;
	}
	else
	{
		return 1;
	}
	memcpy(pyld, payload, len);
	return len;
}

// Touch Payload Results
// [1:count] [2:x] [2:y] [1:state]
// ###############################
static int process_touch_event(struct zforce *tsc, u8* payload)
{
	u16 x,y;
	u8  status;
	int count;
	u8 id = 0;
	u8 state = 0;
#ifdef ZF_USE_DEBUG
	int retval = 0;
#endif

	// Request the next event ASAP.
	if (major == 1)
	{
		send_data_request(tsc);
	}

#ifdef ZF_USE_DEBUG
	// =-=-=-=-=-=-=-=-=-=
	//  Get touch data
	// =-=-=-=-=-=-=-=-=-=
	retval = touchdata_collect( payload ) ;
	if( retval < 0 )
		return retval;

	touchdata_show();
#endif

	count = payload[0];
	#ifdef ZF_USE_DEBUG
	if (major > 1)
	{
		int i;
		printk("NumFingers=%02d\n", count);
		for(i=0; i<count; i++) {
			printk("(%03d, %03d) %d %d %02X %02X ",
				(int)((payload[2+i*touch_data_size]<<8)|payload[1+i*touch_data_size]),
				(int)((payload[4+i*touch_data_size]<<8)|payload[3+i*touch_data_size]),
				(major <= 3) ? (int)((payload[5+i*touch_data_size]&0xC0)>>6) :
				               (int)(payload[5+i*touch_data_size]&0x03),
				(major <= 3) ? (int)((payload[5+i*touch_data_size]&0x3C)>>2) :
				               (int)((payload[5+i*touch_data_size]&0xFC)>>2),
				payload[6+i*touch_data_size],
				(major <= 3) ? payload[7+i*touch_data_size] :
				               payload[9+i*touch_data_size]);
			printk("\nTouch width = %d; height = %d\n", payload[6+i*touch_data_size], payload[7+i*touch_data_size]);
		}
		printk("\n");
	}
	#endif
	if (count != 1)
	{
		if (zf_debug >= 1) dev_info(&tsc->client->dev, "Invalid number of coordinates: %d\n", count);
	}
	memcpy(&x, &payload[1], sizeof(u16));
	memcpy(&y, &payload[3], sizeof(u16));
	status = payload[5];

	if (major == 1)
	{
		state = status & 0x03;
		id = 1;
	}
	else if (major <= 3)
	{
		state = (status & 0xC0) >> 6;
		id =    (status & 0x3C) >> 2;
	}
	else
	{
		state = (status & 0x03);
		id =    (status & 0xFC) >> 2;
	}

	if (major == 1)
	{
		y = 800 - y;
	}

	// Process
	switch (state)
	{
		case STATE_MOVE:
			if (zf_debug >= 1) dev_info(&tsc->client->dev, "%d move(%d,%d)\n", id, x, y);
			input_report_abs(tsc->input, ABS_X, x);
			input_report_abs(tsc->input, ABS_Y, y);
			break;

		case STATE_DOWN:
			if (zf_debug >= 1) dev_info(&tsc->client->dev, "%d down(%d,%d)\n", id, x, y);
			input_report_abs(tsc->input, ABS_X, x);
			input_report_abs(tsc->input, ABS_Y, y);
			input_report_key(tsc->input, BTN_TOUCH, 1);
			break;

		case STATE_UP:
			if (zf_debug >= 1) dev_info(&tsc->client->dev, "%d up(%d,%d)\n", id, x, y);
			input_report_abs(tsc->input, ABS_X, x);
			input_report_abs(tsc->input, ABS_Y, y);
			input_report_key(tsc->input, BTN_TOUCH, 0);
			break;

		default:
			dev_err(&tsc->client->dev, "Invalid state: %d\n", state);
			//Need to fake a release, since input_event system expects a realease for
			//every down event. With Invalid state release is never received.
			input_report_key(tsc->input, BTN_TOUCH, 0);
			break;
	}
	input_sync(tsc->input);
	return (count * touch_data_size) + 1;
}

static int read_packet(struct zforce *tsc, u8 *buffer)
{
	int ret;
	struct i2c_msg msg[2];

	msg[0].addr = tsc->client->addr;
	msg[0].flags = I2C_M_RD;
	msg[0].len = 2;
	msg[0].buf = buffer;

	// Read 2 byte header
	ret = i2c_transfer(tsc->client->adapter, msg, 1);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c header error: %d\n", ret);
		return -1;
	}

	if (buffer[PAYLOAD_HEADER] != FRAME_START)
	{
		if (zf_debug >= 1) dev_info(&tsc->client->dev, "invalid frame: %d\n", buffer[0]);
		invalid_frame_counter++;
		return -1;
	}

	if ((buffer[PAYLOAD_LENGTH] <= 0) || (buffer[PAYLOAD_LENGTH] > 255))
	{
		dev_err(&tsc->client->dev, "invalid payload length: %d\n", buffer[PAYLOAD_LENGTH]);
		return -1;
	}

	// Read payload
	msg[0].addr = tsc->client->addr;
	msg[0].flags = I2C_M_RD;
	msg[0].len = buffer[PAYLOAD_LENGTH];
	msg[0].buf = &buffer[PAYLOAD_BODY];
	memset(msg[0].buf, 0, msg[0].len);
	ret = i2c_transfer(tsc->client->adapter, msg, 1);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c payload error: %d\n", ret);
		return -1;
	}
	return 0;
}

static int zforce_synchronized_wait_for_completion_timeout(struct completion *res)
{
	int ret = 0;

	mutex_lock(&zForce_sysfs_mutex);
	ret = wait_for_completion_timeout(res, WAIT_TIMEOUT);
	mutex_unlock(&zForce_sysfs_mutex);

	if (ret)
		return 1;

	return 0;
}

static void zforce_tsc_reset_work_func(struct work_struct *reset_work)
{
	struct zforce *tsc = container_of(reset_work, struct zforce, reset_work);
	pm_message_t msg;
	msg.event = 0;
	zforce_suspend(tsc->client, msg);
	zforce_resume(tsc->client);
}

static void zforce_tsc_initialization_work_func(struct work_struct *initialization_work)
{
	struct zforce *tsc = container_of(initialization_work, struct zforce, initialization_work);
	struct zforce_platform_data *pdata = tsc->client->dev.platform_data;
	pm_message_t msg;
	msg.event = 0;


	if (send_deactivate_request(tsc))
	{
		dev_err(&tsc->client->dev, "Unable to request deactivate at first time\n");
		if (send_deactivate_request(tsc))
		{
			dev_err(&tsc->client->dev, "Unable to request deactivate at second time\n");
			return;
		}
	}
	if (send_activate_request(tsc))
	{
		dev_err(&tsc->client->dev, "Unable to request activate\n");
		return;
	}
	if (send_resolution(tsc, pdata->width, pdata->height))
	{
		dev_err(&tsc->client->dev, "Unable to set resolution\n");
		return;
	}
	if (send_setconfig(tsc, ZF_SETCONFIG_DUALTOUCH))
	{
		dev_err(&tsc->client->dev, "Unable to set config\n");
		return;
	}
	if (send_data_request(tsc))
	{
		dev_err(&tsc->client->dev, "Unable to request data\n");
		return;
	}
	// Allow time for initial cal to complete
	msleep(200);

	// Get Firmware version and frame size
	if (send_version_request(tsc))
	{
		dev_err(&tsc->client->dev, "Unable to request version\n");
		return;
	}

	// Restore the scan frequencies if they have been modified
	if (modified_freqs != 0)
	{
		if (send_scan_freqs(tsc, tsc->idle_freq, tsc->active_freq, tsc->pen_freq))
		{
			dev_err(&tsc->client->dev, "Unable to restore scan frequencies\n");
			return;
		}
	}
}

// Response Bytes
// [1:0xEE] [1:len] [len:payload]
// ##############################
static void zforce_tsc_work_func(struct work_struct *work)
{
	struct zforce *tsc = container_of(work, struct zforce, work);
	int cmd_len = 0;
	int payload_length = 0;
	u8 payload_buffer[512];
	u8* payload =  NULL;
	int i = 0;

	if (zf_debug >= 2) dev_info(&tsc->client->dev, "+%s()\n", __FUNCTION__);

	if (read_packet(tsc, payload_buffer) < 0) {
		if ((invalid_frame_count_exceeded == 0) && (invalid_frame_counter > MAX_ALLOWED_INVALID_FRAMES)) {
			invalid_frame_count_exceeded = 1;
			dev_err(&tsc->client->dev, "ERROR: invalid frame count exceeded (%d)\n",
				invalid_frame_counter);
		}
		return;
	}
	invalid_frame_counter = 0;  // Reset for each valid response

	payload_length = payload_buffer[PAYLOAD_LENGTH];
	payload =  &payload_buffer[PAYLOAD_BODY];

	while (payload_length > 0)
	{
		if (zf_debug >= 2) {
			dev_info(&tsc->client->dev, "%s: RESPONSE_ID = 0x%02X; payload_length = %d; cmd_len = %d\n",__FUNCTION__, payload[RESPONSE_ID], payload_length, cmd_len);
		}
		if (zf_debug >= 1) {
			printk("Packet IN (RESPONSE_%s) =",
			       (payload[RESPONSE_ID] == RESPONSE_DEACTIVATE) ? "DEACTIVATE" :
			       (payload[RESPONSE_ID] == RESPONSE_ACTIVATE) ? "ACTIVATE" :
			       (payload[RESPONSE_ID] == RESPONSE_RESOLUTION) ? "RESOLUTION" :
			       (payload[RESPONSE_ID] == RESPONSE_SETCONFIG) ? "SETCONFIG" :
			       (payload[RESPONSE_ID] == RESPONSE_DATAREQUEST) ? "DATAREQUEST" :
			       (payload[RESPONSE_ID] == RESPONSE_BOOTCOMPLETE) ? "BOOTCOMPLETE" :
			       (payload[RESPONSE_ID] == RESPONSE_SCANFREQ) ? "SCANFREQ" :
			       (payload[RESPONSE_ID] == RESPONSE_VERSION) ? "VERSION" :
			       (payload[RESPONSE_ID] == RESPONSE_PULSESTRENG) ? "PULSESTRENG" :
			       (payload[RESPONSE_ID] == RESPONSE_FORCECAL) ? "FORCECAL" :
			       (payload[RESPONSE_ID] == RESPONSE_LEVEL) ? "LEVEL" :
			       (payload[RESPONSE_ID] == RESPONSE_STATUS) ? "STATUS" :
			       (payload[RESPONSE_ID] == RESPONSE_OPEN_SHORT) ? "OPEN_SHORT" :
			       (payload[RESPONSE_ID] == RESPONSE_OVERRUN) ? "OVERRUN" :
			       (payload[RESPONSE_ID] == RESPONSE_INVALID) ? "INVALID" :
			       "UNKNOWN");
			for (i = 0; i < payload_length + 2; i++) {
				printk(" %02X", payload_buffer[i]);
			}
			printk("\n");
		}

		switch (payload[RESPONSE_ID])
		{
		case  RESPONSE_DATAREQUEST:
			cmd_len = process_touch_event(tsc, &payload[RESPONSE_DATA]);
			break;

		case  RESPONSE_BOOTCOMPLETE:
			tsc->command_result = payload[RESPONSE_DATA];
			if (tsc->command_result != 0) {
				dev_err(&tsc->client->dev, "RESPONSE_BOOTCOMPLETE: got error code 0x%02X\n",
				        tsc->command_result);
			}

			if (payload_length >= 4) {
				// FW Version >= 4.3.0.12; boot reason is included in payload
				reset_source = payload[RESPONSE_DATA + 1];
				wdg_reset_state = payload[RESPONSE_DATA + 2];

				dev_info(&tsc->client->dev, "BOOTCOMPLETE reset source: 0x%02X (%s)\n",
				         reset_source,
				         (reset_source == 0x01) ? "Watchdog Expired" :
				         (reset_source == 0x02) ? "Watchdog Violation" :
				         (reset_source == 0x04) ? "Flash Violation" :
				         (reset_source == 0x08) ? "Illegal Instruction" :
				         (reset_source == 0x10) ? "Power Up" :
				         (reset_source == 0x20) ? "RST Low" :
				         (reset_source == 0x40) ? "ACLK Fault" :
				         (reset_source == 0x80) ? "Unhandled NMI" : "UNKNOWN REASON");

				if (reset_source == 0x01)
				{
					dev_err(&tsc->client->dev, "Watchdog reset reason: 0x%02X (%s)\n",
					        wdg_reset_state,
					        (wdg_reset_state == 0x01) ? "Timed out waiting for NN1001 interrupt" :
					        (wdg_reset_state == 0x02) ? "SPI communication timeout" :
					        (wdg_reset_state == 0x04) ? "I2C communication timeout" :
					        (wdg_reset_state == 0x08) ? "Unhandled IRQ" : "UNKNOWN");
					if (payload_length >= 5) {
						dev_err(&tsc->client->dev, "NN1001 status = 0x%02X\n",
						        payload[RESPONSE_DATA + 3]);
					}
				}

				if ((reset_source != 0x10) && (reset_source != 0x20)) {
					// If the MSP430 was rebooted by itself (internal timeouot, etc),
					// we need to initialize again.
					dev_info(&tsc->client->dev, "Detected MSP430 reboot - need to re-initialize\n");
					schedule_work(&tsc->initialization_work);
					return;
				}
			}
			complete(&tsc->command_done);
			cmd_len = payload_length - 1;
			break;

		case  RESPONSE_ACTIVATE:
		case  RESPONSE_DEACTIVATE:
		case  RESPONSE_SETCONFIG:
		case  RESPONSE_RESOLUTION:
		case  RESPONSE_SCANFREQ:
		case  RESPONSE_FORCECAL:
			tsc->command_result = payload[RESPONSE_DATA];
			if (tsc->command_result != 0) {
				dev_err(&tsc->client->dev, "%s: got error code 0x%02X\n",
				        (payload[RESPONSE_ID] == RESPONSE_ACTIVATE)   ? "RESPONSE_ACTIVATE" :
				        (payload[RESPONSE_ID] == RESPONSE_DEACTIVATE) ? "RESPONSE_DEACTIVATE" :
				        (payload[RESPONSE_ID] == RESPONSE_SETCONFIG)  ? "RESPONSE_SETCONFIG" :
				        (payload[RESPONSE_ID] == RESPONSE_RESOLUTION) ? "RESPONSE_RESOLUTION" :
				        (payload[RESPONSE_ID] == RESPONSE_SCANFREQ)   ? "RESPONSE_SCANFREQ" :
				        (payload[RESPONSE_ID] == RESPONSE_FORCECAL)   ? "RESPONSE_FORCECAL" :
				                                                        "UNEXPECTED RESPONSE",
				        tsc->command_result);

			}
			complete(&tsc->command_done);
			cmd_len = 1;
			break;

		case  RESPONSE_VERSION:
			cmd_len = process_version_response(tsc, &payload[RESPONSE_DATA]);
			tsc->command_result = 0;
			complete(&tsc->command_done);
			break;

		case  RESPONSE_LEVEL:
			cmd_len = process_level_response(tsc, &payload[RESPONSE_DATA], payload_length);
			tsc->command_result = 0;
			complete(&tsc->command_done);
			break;

		case  RESPONSE_PULSESTRENG:
			cmd_len = process_pulsestreng_response(tsc, &payload[RESPONSE_DATA]);
			tsc->command_result = 0;
			complete(&tsc->command_done);
			break;

		case  RESPONSE_STATUS:
			cmd_len = process_status_response(tsc, &payload[RESPONSE_DATA]);
			tsc->command_result = 0;
			complete(&tsc->command_done);
			break;
			
		case  RESPONSE_OPEN_SHORT:
			cmd_len = process_open_short_response(tsc, &payload[RESPONSE_DATA]);
			tsc->command_result = 0;
			complete(&tsc->command_done);
			break;

		case  RESPONSE_OVERRUN:
			cmd_len = 1;
			dev_err(&tsc->client->dev, "Response from zForce: Command overrun; (err_cnt = %d)\n", ++(tsc->err_cnt) );
			return;

		case  RESPONSE_INVALID:
			cmd_len = 1;
			dev_err(&tsc->client->dev, "Response from zForce: Invalid command or argument; (err_cnt = %d)\n", ++(tsc->err_cnt) );
			return;

		default:
			dev_err(&tsc->client->dev, "Unrecognized Response ID: %d (TC%d)\n", payload[RESPONSE_ID], ++(tsc->err_cnt) );
			dev_err(&tsc->client->dev, "Response frame: " );
			for( i=0; i < (2+payload_length); i++ ){
				printk( " 0x%02X ", payload_buffer[i] );
			}
			printk( "\n" );
			return;
		}
		cmd_len += 1;  // Compensate for cmd byte.

		payload_length -= cmd_len;
		payload += cmd_len;
	}

	return;
}

static irqreturn_t zforce_tsc_irq_handler(int irq, void *dev)
{
	struct zforce *tsc = dev;

	queue_work(zforce_wq, &tsc->work);
	return IRQ_HANDLED;
}

#define DEBUG_CLK_GPIO   140
#define DEBUG_DATA_GPIO  141
#define HAPT_ENABLE_GPIO  37

static void zforce_touch_hw_init(int resume)
{
	u8 v;

	printk("zforce_touch_hw_init ...\n");
	if (!resume && gpio_request(DEBUG_CLK_GPIO, "DEBUG_CLK_GPIO") < 0)
	{
			printk(KERN_ERR "can't get DEBUG_CLK_GPIO\n");
			return;
	}
	gpio_direction_output(DEBUG_CLK_GPIO, 0);

	if (!resume && gpio_request(DEBUG_DATA_GPIO, "DEBUG_DATA_GPIO") < 0)
	{
			printk(KERN_ERR "can't get DEBUG_DATA_GPIO\n");
			return;
	}
	gpio_direction_output(DEBUG_DATA_GPIO, 0);  // Hold RST line low

	if (!resume && gpio_request(HAPT_ENABLE_GPIO, "HAPT_ENABLE_GPIO") < 0)
	{
			printk(KERN_ERR "can't get HAPT_ENABLE_GPIO\n");
			return;
	}
	gpio_direction_output(HAPT_ENABLE_GPIO, 0);  // Power OFF

	mdelay(10);
	gpio_set_value(HAPT_ENABLE_GPIO, 1); // Apply power to MSP430
	mdelay(10);
	gpio_set_value(DEBUG_DATA_GPIO, 1); // De-assert reset

	/* Clear events possibly logged in UART2 by
	   the init procedure - this is a hack but
	   pulling in the structures defined in omap-serial.c
	   would bloat the driver without reason */
	/* Reset UART fifos */
	v = omap_readb(OMAP_UART2_BASE + 0x08);
	v |= 0x06;
	omap_writeb(v, OMAP_UART2_BASE + 0x08);

	mdelay(50);
}
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-
// sysfs
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-

static ssize_t zforce_ledlevel_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int cnt = 0;
	u8 xcount, ycount;
	int offset = 0;
	int i = 0;
	u8* led_info = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);

	// Get LED Levels
	if (major <= 2) {
		if (send_level_request_v2(tsc))
		{
			dev_err(&client->dev, "Unable to request LED levels\n");
			return -EINVAL;
		}
	}
	else {
		// In firmware v3.x and later, each axis is done separately
		if (send_level_request(tsc, X_AXIS))
		{
			dev_err(&client->dev, "Unable to request LED levels in X axis\n");
			return -EINVAL;
		}
		if (send_level_request(tsc, Y_AXIS))
		{
			dev_err(&client->dev, "Unable to request LED levels in Y axis\n");
			return -EINVAL;
		}
	}

	xcount = ledlevel_x[0];
	ycount = ledlevel_y[0];
	// Bounds check
	if( ZF_NUMX != xcount || ZF_NUMY != ycount )
	{
		dev_err(&tsc->client->dev, "Warning: wrong touch dimensions: (%d, %d); was expecting (%d, %d)\n",
		        xcount, ycount, ZF_NUMX, ZF_NUMY);
	}
	zforce_info("LEDCount x=%d, y=%d\n", xcount, ycount);

	offset = 1;
	for (i = 0; i < xcount; i++)
	{
		led_info = &ledlevel_x[offset + (i*3)];
		if( PAGE_SIZE < (cnt+16) )
		{
			dev_err(&tsc->client->dev, "SYSFS buffer overflow predicted.\n");
			return -ENFILE;
		}

		cnt = cnt + sprintf(&buf[cnt], "%02d %02d %03d %03d ", (led_info[0]&0xF0)>>4, led_info[0]&0x0F, led_info[1], led_info[2]);
	}

	cnt = cnt + sprintf(&buf[cnt], "\n" );

	for (i = 0; i < ycount; i++)
	{
		led_info = &ledlevel_y[offset + (i*3)];
		if( PAGE_SIZE < (cnt+16) )
		{
			dev_err(&tsc->client->dev, "SYSFS Buffer overflow predicted.\n");
			return -ENFILE;
		}
		cnt = cnt + sprintf(&buf[cnt], "%02d %02d %03d %03d ",  (led_info[0]&0xF0)>>4, led_info[0]&0x0F, led_info[1], led_info[2]);
	}
	cnt = cnt + sprintf(&buf[cnt], "\n" );

	return cnt;
}


static ssize_t zforce_ledlevel_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	/* place holder for future use */
	return -ENOTSUPP;
}

static ssize_t zforce_reset_source_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int cnt = 0;

	cnt = sprintf(&buf[cnt], "0x%02X (%s)\n", reset_source,
	              ((reset_source == 0x01) && (wdg_reset_state == 0x01)) ? "Watchdog timeout: NN1001 interrupt" :
	              ((reset_source == 0x01) && (wdg_reset_state == 0x02)) ? "Watchdog timeout: SPI communication" :
		      ((reset_source == 0x01) && (wdg_reset_state == 0x04)) ? "Watchdog timeout: I2C communication" :
		      ((reset_source == 0x01) && (wdg_reset_state == 0x08)) ? "Watchdog timeout: unhandled IRQ" :
	              (reset_source == 0x01) ? "UNKNOWN Watchdog Timer Reset" :
	              (reset_source == 0x02) ? "Watchdog Violation" :
	              (reset_source == 0x04) ? "Flash Violation" :
	              (reset_source == 0x08) ? "Illegal Instruction" :
	              (reset_source == 0x10) ? "Power Up" :
	              (reset_source == 0x20) ? "RST Low" :
	              (reset_source == 0x40) ? "ACLK Fault" :
	              (reset_source == 0x80) ? "Unhandled NMI" : "UNKNOWN REASON");

	return cnt;
}


static ssize_t zforce_versions_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int cnt = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = NULL;
	int ret = 0;


	if (client){
		tsc  = i2c_get_clientdata(client);
	}

	if (!zforce_wq){/*DRIVER PROBE FAILED */
		cnt = cnt + sprintf(&buf[cnt],"%04x:%04x %04x:%04x\n", 0, 0, 0, 0);
		printk("sending touch firmware version 00000\n");
		return cnt;
	}
	else if (tsc){
		ret = send_version_request(tsc);
		if (ret){
			dev_err(&client->dev, "UnableToRequestVersion\n");
		return cnt;
		}
	}
	cnt = cnt + sprintf(&buf[cnt],"%d.%d.%d.%d\n", major, minor, build, rev);

	return cnt;
}

static ssize_t zforce_versions_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	/* place holder for future use */
	return -ENOTSUPP;
}


static ssize_t zforce_pulsestrength_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, cnt = 0;
	int numx, numy;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);

	zforce_info("Pulse width and led strength\n");
	if( fixps_time > 0xF && fixps_stren > 0xF )
	{
		return -EINVAL;
	}
	if (major <= 2) {
		if (send_pulsestreng_v2(tsc, fixps_stren, fixps_time))
		{
			dev_err(&client->dev, "Unable to fix pulse strenght\n");
			return -EINVAL;
		}
	}
	else {
		// In firmware v3.x and later, each axis is done separately
		if (send_pulsestreng(tsc, fixps_stren, X_AXIS))
		{
			dev_err(&client->dev, "Unable to fix pulse strenght in X axis\n");
			return -EINVAL;
		}
		if (send_pulsestreng(tsc, fixps_stren, Y_AXIS))
		{
			dev_err(&client->dev, "Unable to fix pulse strenght in Y axis\n");
			return -EINVAL;
		}
	}
	numx = fixps_data_x[0];
	numy = fixps_data_y[0];

	// print data
	// TODO: add check for PAGESIZE overflow
	printk("Num xdata:%d; Num ydata:%d\n", numx, numy);
	cnt = cnt + sprintf(&buf[cnt],"X PD:");

	for (i = 1; i < (1 + numx); i++ )
	{
		cnt = cnt + sprintf(&buf[cnt]," %03d", fixps_data_x[i]);
	}
	cnt = cnt + sprintf(&buf[cnt],"\nY PD:");
	for ( i = 1; i < (1 + numy); i++ )
	{
		cnt = cnt + sprintf(&buf[cnt]," %03d", fixps_data_y[i]);
	}
	cnt = cnt + sprintf(&buf[cnt],"\n");

	return cnt;
}

static ssize_t zforce_pulsestrength_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	unsigned long value;
	int err = 0;
	if (size > 3)
		return -EINVAL;
	err = strict_strtoul(buf, 16, &value);
	if (err != 0)
		return err;
	if (major <= 2) {
		fixps_stren = (u8)((value & 0xF0) >> 4);
		fixps_time  = value & 0x0F;
		printk("\n%s: %d %d; S%d, T%d\n", __FUNCTION__, (int)value, (int)size, (u8)fixps_stren, (u8)fixps_time);
	}
	else {
		// In FW v3.x and later, there is no time parameter
		fixps_stren = (u8)(value & 0xFF);
		fixps_time  = 0;
		printk("\n%s: %d %d; S%d\n", __FUNCTION__, (int)value, (int)size, (u8)fixps_stren);
	}
	return size;
}


static ssize_t zforce_on_off_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);
	int err = 0;
	unsigned long value;

	if (size > 2)
			return -EINVAL;

	err = strict_strtoul(buf, 10, &value);
	if (err != 0)
			return err;

	switch (value)
	{
	case 0:
		// request deactivate
		if (send_deactivate_request(tsc))
		{
			dev_err(&tsc->client->dev, "Unable to request zfdeactivate\n");
			return -1;
		}
		printk("%s: zforce deactivated.\n", __FUNCTION__);
		break;
	case 1:
		// request activate
		if (send_activate_request(tsc))
		{
			dev_err(&tsc->client->dev, "Unable to request zfactivate\n");
			return -1;
		}
		printk("%s: zforced activated\n", __FUNCTION__);
		break;

	default:
			break;
	}
	return size;

}


static ssize_t zforce_idle_freq_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);
	int err = 0;
	unsigned long value = 0;
	u16 idle_freq = 0;

	if (size > 4) {
		printk("%s: ERROR: input parameter must be a number between 0 and 255\n", __FUNCTION__);
		return -EINVAL;
	}

	err = strict_strtoul(buf, 10, &value);
	if (err != 0) {
		printk("%s: ERROR: input parameter must be a number between 0 and 255\n", __FUNCTION__);
		return err;
	}
	if ((value < 0) || (value > 255)) {
		printk("%s: ERROR: input parameter must be a number between 0 and 255\n", __FUNCTION__);
		return -EINVAL;
	}
	idle_freq = (u16)value;

	if (send_scan_freqs(tsc, idle_freq, tsc->active_freq, tsc->pen_freq))
	{
		dev_err(&tsc->client->dev, "Unable to modify idle scan frequency\n");
		return -1;
	}
	tsc->idle_freq = idle_freq;
	printk("%s: idle scan frequency set to %d Hz\n", __FUNCTION__, idle_freq);
	modified_freqs = 1;

	return size;
}


static ssize_t zforce_active_freq_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);
	int err = 0;
	unsigned long value = 0;
	u16 active_freq = 0;

	if (size > 4) {
		printk("%s: ERROR: input parameter must be a number between 0 and 255\n", __FUNCTION__);
		return -EINVAL;
	}

	err = strict_strtoul(buf, 10, &value);
	if (err != 0) {
		printk("%s: ERROR: input parameter must be a number between 0 and 255\n", __FUNCTION__);
		return err;
	}
	if ((value < 0) || (value > 255)) {
		printk("%s: ERROR: input parameter must be a number between 0 and 255\n", __FUNCTION__);
		return -EINVAL;
	}
	active_freq = (u16)value;

	if (send_scan_freqs(tsc, tsc->idle_freq, active_freq, tsc->pen_freq))
	{
		dev_err(&tsc->client->dev, "Unable to modify active scan frequency\n");
		return -1;
	}
	tsc->active_freq = active_freq;
	printk("%s: active scan frequency set to %d Hz\n", __FUNCTION__, active_freq);
	modified_freqs = 1;

	return size;
}


static ssize_t zforce_pen_freq_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);
	int err = 0;
	unsigned long value = 0;
	u16 pen_freq = 0;

	if (size > 4) {
		printk("%s: ERROR: input parameter must be a number between 0 and 255\n", __FUNCTION__);
		return -EINVAL;
	}

	err = strict_strtoul(buf, 10, &value);
	if (err != 0) {
		printk("%s: ERROR: input parameter must be a number between 0 and 255\n", __FUNCTION__);
		return err;
	}
	if ((value < 0) || (value > 255)) {
		printk("%s: ERROR: input parameter must be a number between 0 and 255\n", __FUNCTION__);
		return -EINVAL;
	}
	pen_freq = (u16)value;

	if (send_scan_freqs(tsc, tsc->idle_freq, tsc->active_freq, pen_freq))
	{
		dev_err(&tsc->client->dev, "Unable to modify pen scan frequency\n");
		return -1;
	}
	tsc->pen_freq = pen_freq;
	printk("%s: pen scan frequency set to %d Hz\n", __FUNCTION__, pen_freq);
	modified_freqs = 1;

	return size;
}


static ssize_t zforce_suspend_resume_device(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);
	int err = 0;
	unsigned long value;
	pm_message_t msg;

	if (size > 2)
			return -EINVAL;

	err = strict_strtoul(buf, 10, &value);
	if (err != 0)
			return err;

	switch (value) 
	{
	case 0:
		// Request power OFF
		msg.event = 0;
		if (zforce_suspend(client, msg))
		{
			dev_err(&tsc->client->dev, "Unable to request zforce_suspend\n");
			return -1;
		}
		printk("%s: zforce suspended\n", __FUNCTION__);
		break;
	case 1:
		// Request power ON
		if (zforce_resume(client))
		{
			dev_err(&tsc->client->dev, "Unable to request zforce_resume\n");
			return -1;
		}
		printk("%s: zforce resumed\n", __FUNCTION__);
		break;

	default:
			break;
	}
	return size;

}

static ssize_t zforce_start_touch_detection_store(struct device *dev, struct device_attribute *attr,
                                   const char *buf, size_t size)
{
	int err = 0;
	unsigned long value;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);

	if (size > 2)
		return -EINVAL;

	err = strict_strtoul(buf, 10, &value);
	if (err != 0)
		return err;

	switch (value)
	{
	case 1:
		send_data_request(tsc);
		break;
	default:
		break;
	}
	return size;
}


static ssize_t zforce_debug_store(struct device *dev, struct device_attribute *attr,
                                   const char *buf, size_t size)
{
	int err = 0;
	unsigned long value;

	if (size > 2)
		return -EINVAL;

	err = strict_strtoul(buf, 10, &value);
	if (err != 0)
		return err;

	switch (value)
	{
	case 0:
		// Request to turn debug OFF
		zf_debug = 0;
		printk("%s: zforce debugging deactivated\n", __FUNCTION__);
		break;
	case 1:
	case 2:
		// Request to turn debug ON
		zf_debug = value;
		printk("%s: zforce debugging level %d of 2 activated; set to 0 to deactivate it\n",
		       __FUNCTION__, (int) value);
		break;
	default:
		// Request to turn debug ON
		printk("%s: ERROR: zforce debugging values must be between 0 and 2\n", __FUNCTION__);
		break;
	}
	return size;
}


static ssize_t zforce_cmd_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);
	int err = 0;
	unsigned long value;

	zforce_info("Enter (%d)\n", size);

	if (size > 2)
			return -EINVAL;

	err = strict_strtoul(buf, 16, &value);
	
	if (err != 0)
			return err;
	zforce_info("Enter (%X)\n", (u8)value);

	if( value > 0xFF )
		return -EINVAL;
	if (send_cmd_request(tsc, (u8)value))
	{
		dev_err(&tsc->client->dev, "Unable to request cmd %X\n", (u8)value);
		return -1;
	}
	zforce_debug("zforce cmd %X send.\n", (u8)value);
	return size;
}

static ssize_t zforce_forcecal_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);
	int err = 0;
	unsigned long value;

	zforce_info("Enter\n");

	if (size > 2)
			return -EINVAL;

	err = strict_strtoul(buf, 10, &value);
	if (err != 0)
			return err;

	switch (value) {
	case 1:
		// request force calibration
		if (send_forcecal_request(tsc))
		{
			dev_err(&tsc->client->dev, "Unable to request forcecal\n");
			return -1;
		}
		printk("%s: Forcing TP cal NOW.\n", __FUNCTION__);
		break;
	default:
			break;
	}
	return size;

}


// =-=-=-=-=-=-=-=-=-=-=-=-=
//  Zforce internal status 
// =-=-=-=-=-=-=-=-=-=-=-=-=
static ssize_t zforce_zfstatus_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int cnt = 0;
	int rsvd_start = 0;
	int rsvd_end = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);
	u8 *payload = (u8 *)tsc->zf_status_info;

	// Move to tsc struct?
	u8 act = 0, cur_cont = 0, sa_x_start = 0, sa_x_end = 0, sa_y_start = 0, sa_y_end = 0;
	u8 max_strength = 0, ta_max_enbld = 0, ta_max_size = 0, ta_min_enbld = 0, ta_min_size = 0;
	u16 scancounter = 0, sr_idle = 0, sr_full = 0, sr_pen = 0, xres = 0, yres = 0;
	u16 prep_pckgs = 0, sent_pckgs = 0, invld_ctr = 0, phys_x = 0, phys_y = 0;
	u32 cfg = 0;
	
	if( send_status_request(tsc) )
	{
		dev_err(&client->dev, "Unable retrieve zforce status.\n");
		return -EINVAL;
	}

	// Extract values from the buffer
	if (major <= 2) {
		act = payload[8];
		memcpy(&scancounter, &payload[9], sizeof(u16));
		memcpy(&cfg, &payload[11], sizeof(u32));
		sr_idle = (u16)payload[15];
		sr_full = (u16)payload[16];
		memcpy(&xres, &payload[18], sizeof(u16));
		memcpy(&yres, &payload[20], sizeof(u16));
		sa_x_start = payload[22];
		sa_x_end   = payload[23];
		sa_y_start = payload[24];
		sa_y_end   = payload[25];
		rsvd_start = 26;
		rsvd_end   = 63;
	}
	else {
		act = (payload[8] >> 7) & 0x01;
		cur_cont = payload[8] & 0x7F;
		memcpy(&scancounter, &payload[9], sizeof(u16));
		memcpy(&prep_pckgs, &payload[11], sizeof(u16));
		memcpy(&sent_pckgs, &payload[13], sizeof(u16));
		memcpy(&invld_ctr, &payload[15], sizeof(u16));
		memcpy(&cfg, &payload[17], sizeof(u32));
		memcpy(&sr_idle, &payload[21], sizeof(u16));
		memcpy(&sr_full, &payload[23], sizeof(u16));
		memcpy(&sr_pen, &payload[25], sizeof(u16));
		memcpy(&xres, &payload[27], sizeof(u16));
		memcpy(&yres, &payload[29], sizeof(u16));
		memcpy(&phys_x, &payload[31], sizeof(u16));
		memcpy(&phys_y, &payload[33], sizeof(u16));
		sa_x_start   = payload[35];
		sa_x_end     = payload[36];
		sa_y_start   = payload[37];
		sa_y_end     = payload[38];
		max_strength = payload[39];
		ta_max_enbld = payload[40];
		ta_max_size  = payload[41];
		ta_min_enbld = payload[42];
		ta_min_size  = payload[43];
		rsvd_start = 44;
		rsvd_end   = 127;
	}


	// Total error count
	cnt = cnt + sprintf(&buf[cnt],"Transaction error count:: %0d \n", tsc->err_cnt);

	// Firmware Version
	cnt = cnt + sprintf(&buf[cnt],"MSP FW Version %d.%d.%d.%d\n", major, minor, build, rev);

	// Active status
	cnt = cnt + sprintf(&buf[cnt],"Scanning activated: %02d \n", act);

	// State machine information
	cnt = cnt + sprintf(&buf[cnt],"Current contacts: %02d \n", cur_cont);

	// Scan counter
	cnt = cnt + sprintf(&buf[cnt],"Scanning counter: %d \n", scancounter);
	
	// Configuration data
	cnt = cnt + sprintf(&buf[cnt],"Configuration: %04X \n", cfg);

	// Scan frequencies
	if (major <= 2) {
		cnt = cnt + sprintf(&buf[cnt],"Scan freq: idle = %d, full = %d\n", sr_idle, sr_full);
	}
	else {
		cnt = cnt + sprintf(&buf[cnt],"Scan freq: idle = %d, full = %d, pen = %d\n", sr_idle, sr_full, sr_pen);
	}
	
	// Resolution (x and y)
	cnt = cnt + sprintf(&buf[cnt],"XRES, YRES: %d, %d \n", xres, yres);
	
	// Active LED (scan area)
	cnt = cnt + sprintf(&buf[cnt],"Active LEDs: X 1st = %d, X last = %d, Y 1st = %d, Y last = %d \n", sa_x_start, sa_x_end, sa_y_start, sa_y_end);

	// Status information only available in firmware versions after 2.x
	if (major >= 3) {
		// Prepared touch packages
		cnt = cnt + sprintf(&buf[cnt],"PREP PCKGS: %d \n", prep_pckgs);

		// Sent touch packages
		cnt = cnt + sprintf(&buf[cnt],"SENT PCKGS: %d \n", sent_pckgs);

		// Invalid touch counter
		cnt = cnt + sprintf(&buf[cnt],"INVALID_TCH_CNT: %d \n", invld_ctr);

		// Physical dimensions (x and y, in mm)
		cnt = cnt + sprintf(&buf[cnt],"X size = %d mm, Y size = %d mm \n", phys_x, phys_y);

		// Maximum strength
		cnt = cnt + sprintf(&buf[cnt],"MAX_SIGNAL_STRENGTH: %d \n", max_strength);

		// Touch area size info - maximum size
		cnt = cnt + sprintf(&buf[cnt],"Max Size Enabled = %d, Max Size = %d mm \n", ta_max_enbld, ta_max_size);

		// Touch area size info - maximum size
		cnt = cnt + sprintf(&buf[cnt],"Min Size Enabled = %d, Min Size =  %d mm \n", ta_min_enbld, ta_min_size);
	}

	return cnt;
}

// =-=-=-=-=-=-=-=-=-=-=-=-=
//  Zforce OPEN_SHORT show
// =-=-=-=-=-=-=-=-=-=-=-=-=
static ssize_t zforce_open_short_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, cnt = 0, ret = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);
	u8 *payload = (u8 *)tsc->zf_open_short_xresult;

	zforce_info("Zforce internal status strength\n");

	// run open and short test for x axis
	ret = send_open_short_request(tsc, X_AXIS);
	if(ret)
	{
		dev_err(&client->dev, "Unable to retrieve zforce status: %x.\n", ret);
		return -EINVAL;
	}

	// run open and short test for y axis
	ret = send_open_short_request(tsc, Y_AXIS);
	if(ret)
	{
		dev_err(&client->dev, "Unable to retrieve zforce status: %x.\n", ret);
		return -EINVAL;
	}

	cnt = cnt + sprintf(&buf[cnt],"BISTX:");
	for( i=0; i < tsc->zf_open_short_xsize; i++ )
	{
		cnt = cnt + sprintf(&buf[cnt]," %02X", payload[i] );
	}

	payload = (u8 *)tsc->zf_open_short_yresult;
	cnt = cnt + sprintf(&buf[cnt],"\nBISTY:");
	for( i=0; i < tsc->zf_open_short_ysize; i++ )
	{
		cnt = cnt + sprintf(&buf[cnt]," %02X", payload[i] );
	}
	cnt = cnt + sprintf(&buf[cnt], "\n");
	return cnt;
}



static DEVICE_ATTR(scan_freq_idle, S_IWUSR, NULL, zforce_idle_freq_store);
static DEVICE_ATTR(scan_freq_active, S_IWUSR, NULL, zforce_active_freq_store);
static DEVICE_ATTR(scan_freq_pen, S_IWUSR, NULL, zforce_pen_freq_store);
static DEVICE_ATTR(ledlevel, S_IRUGO|S_IWUSR, zforce_ledlevel_show, zforce_ledlevel_store);
static DEVICE_ATTR(versions, S_IRUGO|S_IWUSR, zforce_versions_show, zforce_versions_store);
static DEVICE_ATTR(forcecal, S_IWUSR, NULL, zforce_forcecal_store);
static DEVICE_ATTR(fixps, S_IRUGO|S_IWUSR, zforce_pulsestrength_show, zforce_pulsestrength_store);
static DEVICE_ATTR(zfstatus, S_IRUGO, zforce_zfstatus_show, NULL);
static DEVICE_ATTR(bist, S_IRUGO, zforce_open_short_show, NULL);
static DEVICE_ATTR(on_off, S_IWUSR, NULL, zforce_on_off_store);
static DEVICE_ATTR(debug, S_IWUSR, NULL, zforce_debug_store);
static DEVICE_ATTR(cmd,	S_IWUSR, NULL, zforce_cmd_store);
static DEVICE_ATTR(suspend_resume, S_IWUSR, NULL, zforce_suspend_resume_device);
static DEVICE_ATTR(start_touch_detection, S_IWUSR, NULL, zforce_start_touch_detection_store);
static DEVICE_ATTR(reset_source, S_IRUGO, zforce_reset_source_show, NULL);

static struct attribute *zforce_attributes[] = {
	&dev_attr_scan_freq_idle.attr,
	&dev_attr_scan_freq_active.attr,
	&dev_attr_scan_freq_pen.attr,
	&dev_attr_ledlevel.attr,
	&dev_attr_versions.attr,
	&dev_attr_forcecal.attr,
	&dev_attr_fixps.attr,
	&dev_attr_zfstatus.attr,
	&dev_attr_bist.attr,
	&dev_attr_on_off.attr,
	&dev_attr_debug.attr,
	&dev_attr_cmd.attr,
	&dev_attr_suspend_resume.attr,
	&dev_attr_start_touch_detection.attr,
	&dev_attr_reset_source.attr,
	NULL
};

static struct attribute_group zforce_attribute_group = {
	.attrs = zforce_attributes
};

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-

static int zforce_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct zforce *tsc;
	struct zforce_platform_data *pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err;

	if (!pdata)
	{
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR "zforce_probe: need I2C_FUNC_I2C\n");
		return -ENODEV;
	}

	zforce_touch_hw_init(0);

	tsc = kzalloc(sizeof(struct zforce), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!tsc || !input_dev)
	{
		err = -ENOMEM;
		goto err_free_mem;
	}

	INIT_WORK(&tsc->work, zforce_tsc_work_func);
	INIT_WORK(&tsc->reset_work, zforce_tsc_reset_work_func);
	INIT_WORK(&tsc->initialization_work, zforce_tsc_initialization_work_func);
	init_completion(&tsc->command_done);

	//--== sysfs entries ==--
	err = sysfs_create_group(&client->dev.kobj, &zforce_attribute_group);
	if (err)
	{
		zforce_alert("sysfs_create_group() failed!!\n");
		goto err_free_mem;
	}

	tsc->client = client;
	i2c_set_clientdata(client, tsc);

	tsc->input = input_dev;

	input_dev->name = "zForce Touchscreen";
	input_dev->id.bustype = BUS_I2C;

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, pdata->width, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, pdata->height, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, pdata->width, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, pdata->height, 0, 0);

	tsc->irq = client->irq;
	tsc->err_cnt = 0;

	err = input_register_device(input_dev);
	if (err)
		goto err_free_mem;

	err = request_irq(client->irq, zforce_tsc_irq_handler, pdata->irqflags, client->name, tsc);
	if (err < 0)
	{
		dev_err(&client->dev, "Unable to register irq %d\n", tsc->irq);
		goto err_free_dev;
	}
	irq_is_enabled = 1;
	dev_info(&client->dev, "Registered irq %d (%s)\n", client->irq,
	                       (pdata->irqflags == IRQF_TRIGGER_FALLING) ? "IRQF_TRIGGER_FALLING" :
	                       (pdata->irqflags == IRQF_TRIGGER_RISING)  ? "IRQF_TRIGGER_RISING" :
	                       "UNKNOWN IRQ FLAG");

	// Read the result of Boot Complete, if applicable; otherwise, the
	// controller will not generate interrupts when receiving commands
	queue_work(zforce_wq, &tsc->work);
	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0) {
		// Looks like firmware version earlier than 4.x; version will
		// be read and set later
		major = 0;
		if (zforce_wq) flush_workqueue(zforce_wq);
		// Set the default fixed pulse strength parameters
		fixps_stren  = 12;
		fixps_time   = 3;
	}
	else {
		// Assume 4.x for now; version will be read and set later
		major = 4;
		// Set the default fixed pulse strength parameter
		fixps_stren  = 0xA1;
		// Some versions of firmware send a boot reason response
		// after the boot complete response; wait for it if needed
		zforce_synchronized_wait_for_completion_timeout(&tsc->command_done);
	}
	init_completion(&tsc->command_done);

	// We are now ready for some events..
	if (send_activate_request(tsc))
	{
		dev_err(&client->dev, "Unable to activate\n");
		goto err_free_irq;
	}

	// Set the touch panel width & height
	if (send_resolution(tsc, pdata->width, pdata->height))
	{
		dev_err(&client->dev, "Unable to set resolution\n");
		goto err_free_irq;
	}

	// Set configuration, enable dual touch
	if (send_setconfig(tsc, ZF_SETCONFIG_DUALTOUCH))
	{
		dev_err(&client->dev, "Unable to set config\n");
		goto err_free_irq;
	}

	if (major >= 4) {
		// Get the scanning frequencies; this will also print the firmware version
		if (get_scan_freqs(tsc))
		{
			dev_err(&client->dev, "Unable to request scanning frequencies\n");
		}
	}
	else {
		// Get Firmware version and frame size
		if (send_version_request(tsc))
		{
			dev_err(&client->dev, "Unable to request version\n");
		}
	}
	// This will start sending touch events.
	if (send_data_request(tsc))
	{
		dev_err(&client->dev, "Unable to request data\n");
		goto err_free_irq;
	}

	if (invalid_frame_count_exceeded != 0) {
		dev_err(&tsc->client->dev, "zForce not working properly\n");
		goto err_free_irq;
	}

	// Per NN, initial cal. take max. of 200msec.
	// Allow time to complete this calibration
	//msleep(200);
	// Using mdelay to prevent lockup of i2c bus when loading next driver
	// TODO: fix this at the high level; it is not ideal to add delays
	mdelay(100);

	return 0;

 err_free_irq:
	free_irq(tsc->irq, tsc);
	err = -EINVAL;

 err_free_dev:
	if (zforce_wq)
	{
		destroy_workqueue(zforce_wq);
		zforce_wq = NULL;
	}

	sysfs_remove_group(&client->dev.kobj, &zforce_attribute_group);
	input_unregister_device(input_dev);
	input_dev = NULL;
	i2c_set_clientdata(client, NULL);

 err_free_mem:
	input_free_device(input_dev);
	kfree(tsc);
	pr_err("%s: err = [%d]\n", __func__, err);
	return err;
}

static void zforce_shutdown(struct i2c_client *client)
{
	struct zforce *tsc = i2c_get_clientdata(client);
	if(!tsc){
		pr_err("%s: tsc is NULL!\n", __func__);
		return;
	}

	dev_info(&client->dev, "ZforceDrvrShtdwn\n");
	if (irq_is_enabled) {
		irq_is_enabled = 0;
		disable_irq(tsc->irq);
	}
	if (zforce_wq) flush_workqueue(zforce_wq);
	gpio_set_value(DEBUG_DATA_GPIO, 0); // data line doubles as 430 reset; pull reset low
	udelay(100);
	gpio_direction_output(HAPT_ENABLE_GPIO, 0);

}

static int zforce_remove(struct i2c_client *client)
{
	struct zforce *tsc = i2c_get_clientdata(client);
	struct zforce_platform_data *pdata;
	dev_info(&client->dev, "ZforceDrvrRemove\n");
	zforce_shutdown(client);

	pdata = client->dev.platform_data;
	/* housekeeping */
	sysfs_remove_group(&client->dev.kobj, &zforce_attribute_group);
	
	free_irq(tsc->irq, tsc);
	input_unregister_device(tsc->input);
	kfree(tsc);

	gpio_free(DEBUG_CLK_GPIO);
	gpio_free(DEBUG_DATA_GPIO);
	gpio_free(HAPT_ENABLE_GPIO);

	return 0;
}

#if defined (CONFIG_MACH_OMAP3621_GOSSAMER)
static int zforce_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct zforce *tsc = i2c_get_clientdata(client);
	
	if (irq_is_enabled) {
		irq_is_enabled = 0;
		disable_irq(tsc->irq);
	}
	if (zforce_wq) flush_workqueue(zforce_wq);
	gpio_set_value(DEBUG_DATA_GPIO, 0); // data line doubles as 430 reset; pull reset low
	udelay(100);
	gpio_direction_output(HAPT_ENABLE_GPIO, 0); // disable power

	return 0;
}

static int zforce_resume(struct i2c_client *client)
{
	struct zforce *tsc = i2c_get_clientdata(client);
	struct zforce_platform_data *pdata = client->dev.platform_data;

	if (!irq_is_enabled) {
		irq_is_enabled = 1;
		enable_irq(tsc->irq);
	}
	zforce_touch_hw_init(1);

	// Read the result of Boot Complete, if applicable; otherwise, the
	// controller will not generate interrupts when receiving commands
	queue_work(zforce_wq, &tsc->work);
	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0) {
		if (zforce_wq) flush_workqueue(zforce_wq);
	}
	init_completion(&tsc->command_done);

	if (send_activate_request(tsc))
	{
		dev_err(&client->dev, "Unable to request activate\n");
		return -1;
	}
	if (send_resolution(tsc, pdata->width, pdata->height))
	{
		dev_err(&client->dev, "Unable to set resolution\n");
		return -1;
	}
	if (send_setconfig(tsc, ZF_SETCONFIG_DUALTOUCH))
	{
		dev_err(&client->dev, "Unable to set config\n");
		return -1;
	}
	if (send_data_request(tsc))
	{
		dev_err(&client->dev, "Unable to request data\n");
		return -1;
	}
	// Allow time for initial cal to complete
	msleep(200);
	
	// Get Firmware version and frame size
	if (send_version_request(tsc))
	{
		dev_err(&client->dev, "Unable to request version\n");
		return -1;
	}

	// Restore the scan frequencies if they have been modified
	if (modified_freqs != 0)
	{
		if (send_scan_freqs(tsc, tsc->idle_freq, tsc->active_freq, tsc->pen_freq))
		{
			dev_err(&tsc->client->dev, "Unable to restore scan frequencies\n");
			return -1;
		}
	}

	return 0;
}
#endif /* CONFIG_MACH_OMAP3621_GOSSAMER */

static struct i2c_device_id zforce_idtable[] = {
	{ ZFORCE_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, zforce_idtable);

static struct i2c_driver zforce_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= ZFORCE_NAME
	},
	.id_table	= zforce_idtable,
	.probe		= zforce_probe,
	.remove		= zforce_remove,
#if defined(CONFIG_MACH_OMAP3621_GOSSAMER) || defined(CONFIG_MACH_OMAP3621_AVOCET)
	.suspend	= zforce_suspend,
	.resume		= zforce_resume,
	.shutdown	= zforce_shutdown,
#endif /* CONFIG_MACH_OMAP3621_GOSSAMER */
};

static int __init zforce_init(void)
{
	zforce_wq = create_singlethread_workqueue("zforce_wq");
	if (!zforce_wq)
		return -ENOMEM;
	return i2c_add_driver(&zforce_driver);
}

static void __exit zforce_exit(void)
{
	i2c_del_driver(&zforce_driver);
	if (zforce_wq)
		destroy_workqueue(zforce_wq);
}

module_init(zforce_init);
module_exit(zforce_exit);

MODULE_AUTHOR("Pieter Truter");
MODULE_DESCRIPTION("zForce TouchScreen Driver");
MODULE_LICENSE("GPL");

