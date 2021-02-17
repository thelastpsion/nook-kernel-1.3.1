/*
 * EPD EEPROM support
 *
 * Copyright (c) 2012 Barnes & Noble
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl4030.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>

#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/gpio.h>
#include <mach/mux.h>
#include <asm/mach-types.h>
#include <mach/control.h>

/* EPD EEPROM SPI lines */
#define EEPROM_PWR_GPIO     85
#define GPIO_SPI_CLK       156
#define GPIO_SPI_SIMO      158
#define GPIO_SPI_SOMI      159
#define GPIO_SPI_CS        161
#define SPI_DELAY

/* EPD EEPROM commands and addresses */
#define EEPROM_CMD_READ       0x03
#define EEPROM_CMD_FAST_READ  0x0B
#define EEPROM_CMD_RDID       0x9F
#define EEPROM_ID_SIZE        3
#define EPD_PART_NUM_SIZE     16
#define EPD_VCOM_SIZE         16
#define EPD_WVFM_VER_SIZE     32
#define EPD_FPL_VER_SIZE      16
#define EPD_BARCODE_SIZE      48
#define MAX_EPD_STRING_SIZE   64
#define EPD_INFO_MSB_NO_REAGL 0x03  // Pre-REAGL base address of wvfm data
#define EPD_INFO_MSB_REAGL    0x07  // Base address of wvfm data for REAGL
#define EPD_WAVEFORM_ADDR_ST  0x00886
#define EPD_WAVEFORM_ADDR_END_NO_REAGL 0x2FFFF
#define EPD_WAVEFORM_ADDR_END_REAGL    0x6FFFF
#define EPD_WAVEFORM_ADDR_END (epd_info_msb == EPD_INFO_MSB_REAGL ? EPD_WAVEFORM_ADDR_END_REAGL : EPD_WAVEFORM_ADDR_END_NO_REAGL)
#define EPD_WVFM_CHUNK        1024
#define MX25U2033E_MFG_ID     0xC2
#define MX25U2033E_DEV_ID_MSB 0x25
#define MX25U2033E_DEV_ID_LSB 0x32


static struct spi_device *avocet_spi_device;

// Globals for SPI data
u8 spi_buff[EPD_WVFM_CHUNK + 8];
u8 *spi_out = spi_buff;
u8 *spi_in = spi_buff;
u8 epd_data[MAX_EPD_STRING_SIZE];

static u8 epd_info_msb = 0;
static u32 max_epd_waveform_size = 0;
static u32 wvfm_addr = EPD_WAVEFORM_ADDR_ST;
static u32 last_wvfm_addr = 0;
static int wvfm_file_length = 0;

int  spi_xfer(int bitlen, u8 *dout, u8 *din) {
	u8 tmpdin  = 0;
	u8 tmpdout = 0;
	int   j;

	gpio_set_value(GPIO_SPI_CS, 0);  // Select the target chip
	SPI_DELAY;

	for(j = 0; j < bitlen; j++) {
		/*
		 * Check if it is time to work on a new byte.
		 */
		if((j % 8) == 0) {
			tmpdout = *dout++;
			if(j != 0) {
				*din++ = tmpdin;
			}
			tmpdin  = 0;
		}
		gpio_set_value(GPIO_SPI_SIMO, tmpdout & 0x80);
		gpio_set_value(GPIO_SPI_CLK, 0);
		SPI_DELAY;
		gpio_set_value(GPIO_SPI_CLK, 1);
		SPI_DELAY;
		tmpdin  <<= 1;
		tmpdin   |= gpio_get_value(GPIO_SPI_SOMI);
		tmpdout <<= 1;
	}
	/*
	 * If the number of bits isn't a multiple of 8, shift the last
	 * bits over to left-justify them.  Then store the last byte
	 * read in.
	 */
	if((bitlen % 8) != 0)
		tmpdin <<= 8 - (bitlen % 8);
	*din++ = tmpdin;

	gpio_set_value(GPIO_SPI_CLK, 0);  // SPI wants the clock left low for idle

	gpio_set_value(GPIO_SPI_CS, 1);  // Deselect the target chip
	SPI_DELAY;

	return(0);
}


// Translate from PVI's encoding to normal ASCII characters
// NOTE: make sure that the size of dest is at least max_chars + 1
void pvi_to_ascii(unsigned char *dest, unsigned char *source, int max_chars) {

    int i = 0;
    int s_ptr = 0;
    int d_ptr = 0;

    memset(dest, 0, max_chars + 1);
    // Skip leading 0's or spaces
    while ((source[s_ptr] == 0xFF) || (source[s_ptr] == 0x00)) {
        s_ptr++;
    }

    for (i = s_ptr; i < max_chars; i++) {
        if ((source[i] >= 0xE5) && (source[i] <= 0xFE)) {
            // Uppercase letter
            dest[d_ptr] = 'A' + (source[i] - 0xE5);
        }
        else if ((source[i] >= 0x00) && (source[i] <= 0x09)) {
            // Number
            dest[d_ptr] = '0' + source[i];
        }
        else if ((source[i] >= 0xCB) && (source[i] <= 0xE4)) {
            // Lowercase letter
            dest[d_ptr] = 'a' + (source[i] - 0xCB);
        }
        // Special chars
        else if (source[i] == 0x0A) dest[d_ptr] = '_';
        else if (source[i] == 0x0B) dest[d_ptr] = '.';
        else if (source[i] == 0x0C) dest[d_ptr] = '-';
        // If unknown, it has already been set to 0x00 by memset above
        // Note: 0xFF is a space in PVI's mapping, but it is also used to fill
        //       the end of strings.  We simply skip it because the strings
        //       themselves are not expected to have spaces.
        d_ptr++;
    }

}


// Translate the ASCII string for Vcom from Volts to Millivolts
// NOTE: translation is done on the same input string
void v_to_mv(unsigned char *vcom_str, int max_chars) {

    int mv_start = 0;
    int mv_digits = 0;
    int s_ptr = 0;
    int d_ptr = 0;
    int done = 0;

    while ((s_ptr <= max_chars) && (done == 0)) {
        if (vcom_str[s_ptr] == '.') {
            // Found the decimal point; skip it and start counting millivolts
            s_ptr++;
            mv_start = 1;
        }
        else if (mv_start != 0) {
            // Next millivolt digit
            if ((vcom_str[s_ptr] >= '0') && (vcom_str[s_ptr] <= '9')) {
                vcom_str[d_ptr++] = vcom_str[s_ptr++];
                mv_digits++;
            }
            else {
                // Fill the rest of the millivolt digits with 0's
                for (; mv_digits < 3; mv_digits++) {
                    vcom_str[d_ptr++] = '0';
                }
                done = 1;
            }
        }
        else if (((vcom_str[s_ptr] >= '0') && (vcom_str[s_ptr] <= '9')) ||
                 (vcom_str[s_ptr] == '-') || (vcom_str[s_ptr] == '+')) {
            // Volts digit or sign
            vcom_str[d_ptr++] = vcom_str[s_ptr++];
        }
        else {
            // Invalid character; just set the string to NULL and exit
            vcom_str[0] = '\0';
            done = 1;
        }
    }
    // Fill the rest of the destination with NULL characters
    for (; d_ptr < max_chars; d_ptr++) {
        vcom_str[d_ptr] = '\0';
    }
}


// Prepare EEPROM for reading
static int turn_on_eeprom(void)
{
	u8 new_epd_info_msb = 0;

	gpio_set_value(GPIO_SPI_CS, 1);
	gpio_set_value(EEPROM_PWR_GPIO, 1);
	msleep(1);

	// Determine the correct base address of the waveform information data.
	// Do this by reading VCOM and checking against the expected
	// data format: "-[0-9].[0-9][0-9]"
	memset(spi_buff, 0x00, EPD_VCOM_SIZE + 1);
	spi_out[0] = EEPROM_CMD_READ;
	spi_out[1] = EPD_INFO_MSB_REAGL;  // Try the REAGL waveform address
	spi_out[2] = 0x00;
	spi_out[3] = 0x10;
	spi_xfer(32 + (8 * EPD_VCOM_SIZE), (u8 *)spi_out, (u8 *)spi_buff);
	pvi_to_ascii(epd_data, &spi_buff[4], EPD_VCOM_SIZE);
	if ((epd_data[0] == '-') &&
		(epd_data[1] >= '0') && (epd_data[1] <= '9') &&
		(epd_data[2] == '.') &&
		(epd_data[3] >= '0') && (epd_data[3] <= '9') &&
		(epd_data[4] >= '0') && (epd_data[4] <= '9')) {
		new_epd_info_msb = EPD_INFO_MSB_REAGL;
	}
	else {
		memset(spi_buff, 0x00, EPD_VCOM_SIZE + 1);
		spi_out[1] = EPD_INFO_MSB_NO_REAGL;  // Try the pre-REAGL waveform address
		spi_xfer(32 + (8 * EPD_VCOM_SIZE), (u8 *)spi_out, (u8 *)spi_buff);
		pvi_to_ascii(epd_data, &spi_buff[4], EPD_VCOM_SIZE);
		if ((epd_data[0] == '-') &&
			(epd_data[1] >= '0') && (epd_data[1] <= '9') &&
			(epd_data[2] == '.') &&
			(epd_data[3] >= '0') && (epd_data[3] <= '9') &&
			(epd_data[4] >= '0') && (epd_data[4] <= '9')) {
			new_epd_info_msb = EPD_INFO_MSB_NO_REAGL;
		}
	}
	if (new_epd_info_msb == 0) {
		printk("WARNING: Could not read data from EPD EEPROM; may not be present\n");
		return -1;
	}

	if (new_epd_info_msb != epd_info_msb) {
		epd_info_msb = new_epd_info_msb;
		printk("Found EPD EEPROM; info base addr = 0x%02X0000\n", epd_info_msb);
	}

	// Recalculate max wvfm size in case a different EPD was plugged in without
	// rebooting the system.  This is done at the factory for IQC.
	max_epd_waveform_size = (EPD_WAVEFORM_ADDR_END - EPD_WAVEFORM_ADDR_ST + 1);

	return 0;
}


int eeprom_id_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0;
	int cnt = 0;
	int ret = 0;

	turn_on_eeprom();
	memset(spi_buff, 0x00, 16);
	spi_out[0] = EEPROM_CMD_RDID; // First byte has the command
	ret = spi_xfer(8 + (8 * EEPROM_ID_SIZE), (u8 *)spi_out, (u8 *)spi_buff);
	for (i = 0; i < EEPROM_ID_SIZE; i++) {
		cnt += sprintf(&buf[cnt],"%02X", spi_buff[1+i]);
	}
	return cnt;
}


static ssize_t eeprom_vcom_read(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int cnt = 0, ret = 0;

        // Read VCOM value - addresses offset 0x10 - 0x1F
	turn_on_eeprom();
        memset(spi_buff, 0x00, EPD_VCOM_SIZE + 1);
        spi_out[0] = EEPROM_CMD_READ;
        spi_out[1] = epd_info_msb;
        spi_out[2] = 0x00;
        spi_out[3] = 0x10;

	ret = spi_xfer(32 + (8 * EPD_VCOM_SIZE), (u8 *)spi_out, (u8 *)spi_buff);
        pvi_to_ascii(epd_data, &spi_buff[4], EPD_VCOM_SIZE);
        v_to_mv(epd_data, EPD_VCOM_SIZE);
	cnt = sprintf(&buf[cnt],"%s", epd_data);

	return cnt;
}


static ssize_t eeprom_waveform_filename_read(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int cnt = 0, ret = 0;

        // Read waveform version - addresses offset 0x20 - 0x3F
	turn_on_eeprom();
        memset(spi_buff, 0x00, EPD_WVFM_VER_SIZE + 1);
        spi_out[0] = EEPROM_CMD_READ;
        spi_out[1] = epd_info_msb;
        spi_out[2] = 0x00;
        spi_out[3] = 0x20;
        ret = spi_xfer(32 + (8 * EPD_WVFM_VER_SIZE), (u8 *)spi_out, (u8 *)spi_buff);
        pvi_to_ascii(epd_data, &spi_buff[4], EPD_WVFM_VER_SIZE);
	cnt = sprintf(&buf[cnt],"%s", epd_data);

	return cnt;
}


static ssize_t eeprom_epd_partnum_read(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int cnt = 0, ret = 0;

        // Read product part number - addresses offset 0x00 - 0x0F
	turn_on_eeprom();
        memset(spi_buff, 0x00, EPD_PART_NUM_SIZE + 1);
        spi_out[0] = EEPROM_CMD_READ;
        spi_out[1] = epd_info_msb;
        spi_out[2] = 0x00;
        spi_out[3] = 0x00;
        ret = spi_xfer(32 + (8 * EPD_PART_NUM_SIZE), (u8 *)spi_out, (u8 *)spi_buff);
        pvi_to_ascii(epd_data, &spi_buff[4], EPD_PART_NUM_SIZE);
	cnt = sprintf(&buf[cnt],"%s", epd_data);

	return cnt;
}


static ssize_t eeprom_epd_fpl_ver_read(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int cnt = 0, ret = 0;

        // Read FPL version - addresses offset 0x40 - 0x4F
	turn_on_eeprom();
        memset(spi_buff, 0x00, EPD_FPL_VER_SIZE + 1);
        spi_out[0] = EEPROM_CMD_READ;
        spi_out[1] = epd_info_msb;
        spi_out[2] = 0x00;
        spi_out[3] = 0x40;
        ret = spi_xfer(32 + (8 * EPD_FPL_VER_SIZE), (u8 *)spi_out, (u8 *)spi_buff);
        pvi_to_ascii(epd_data, &spi_buff[4], EPD_FPL_VER_SIZE);
	cnt = sprintf(&buf[cnt],"%s", epd_data);

	return cnt;
}


static ssize_t eeprom_epd_barcode_read(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int cnt = 0, ret = 0;

        // Read barcode - addresses offset 0x50 - 0x7F
	turn_on_eeprom();
        memset(spi_buff, 0x00, EPD_BARCODE_SIZE + 1);
        spi_out[0] = EEPROM_CMD_READ;
        spi_out[1] = epd_info_msb;
        spi_out[2] = 0x00;
        spi_out[3] = 0x50;
        ret = spi_xfer(32 + (8 * EPD_BARCODE_SIZE), (u8 *)spi_out, (u8 *)spi_buff);
        pvi_to_ascii(epd_data, &spi_buff[4], EPD_BARCODE_SIZE);
	cnt = sprintf(&buf[cnt],"%s", epd_data);

	return cnt;
}

static ssize_t eeprom_waveform_read(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int cnt = 0;
	int ret = 0;
	int byte_count = 0;

	// If we have reached the end of the waveform file in the previous call, return
	// an indicator that we are done.  Also, reset the global variables for the
	// next round.
	if (wvfm_addr > last_wvfm_addr) {
		wvfm_addr = EPD_WAVEFORM_ADDR_ST;
		last_wvfm_addr = EPD_WAVEFORM_ADDR_END;
		wvfm_file_length = 0;
		cnt = sprintf(&buf[cnt],"<EOF>");
		goto exit;
	}

	turn_on_eeprom();
	// Read 1 KBytes of the EPD waveform at a time (or less, if the file is ending)
	// This is a limitation of sysfs
	byte_count = ((wvfm_addr + EPD_WVFM_CHUNK) <= last_wvfm_addr) ?
	             EPD_WVFM_CHUNK : (last_wvfm_addr - wvfm_addr + 1);
	//printk("Reading %d bytes of EpdWaveform, from address 0x%05X\n", byte_count, wvfm_addr);
	memset(spi_buff, 0x00, sizeof(spi_buff));
	spi_out[0] = EEPROM_CMD_FAST_READ;
	spi_out[1] = (unsigned char)((wvfm_addr & 0x00FF0000) >> 16);
	spi_out[2] = (unsigned char)((wvfm_addr & 0x0000FF00) >> 8);
	spi_out[3] = (unsigned char)(wvfm_addr & 0x000000FF);
	spi_out[4] = 0x00; // For fast read dummy cycle
	ret = spi_xfer(40 + (8 * byte_count), (u8 *)spi_out, (u8 *)spi_buff);
	if (wvfm_addr == EPD_WAVEFORM_ADDR_ST) {
		// First chunk; obtain waveform size from data just downloaded.
		// Waveform file length is a 4-byte field starting at offset 0x04
		wvfm_file_length = *((u32 *)(&spi_buff[9]));
		// File length should not be greater than space allocated in EEPROM
		last_wvfm_addr = (wvfm_file_length <= (max_epd_waveform_size)) ?
		                 EPD_WAVEFORM_ADDR_ST + wvfm_file_length - 1 : EPD_WAVEFORM_ADDR_END;
		//printk("wvfm_file_length = %d; last_wvfm_addr = 0x%X\n", wvfm_file_length, last_wvfm_addr);
	}
	memcpy(&buf[cnt], &spi_buff[5], byte_count);
	wvfm_addr += EPD_WVFM_CHUNK;
	cnt += byte_count;

exit:
	return cnt;
}


static DEVICE_ATTR(EepromId, S_IRUGO, eeprom_id_read, NULL);
static DEVICE_ATTR(EpdVcom, S_IRUGO, eeprom_vcom_read, NULL);
static DEVICE_ATTR(EpdWaveform, S_IRUGO, eeprom_waveform_read, NULL);
static DEVICE_ATTR(EpdWaveformOriginalFilename, S_IRUGO, eeprom_waveform_filename_read, NULL);
static DEVICE_ATTR(EpdPartNum, S_IRUGO, eeprom_epd_partnum_read, NULL);
static DEVICE_ATTR(EpdFplVer, S_IRUGO, eeprom_epd_fpl_ver_read, NULL);
static DEVICE_ATTR(EpdBarcode, S_IRUGO, eeprom_epd_barcode_read, NULL);

static struct attribute *epd_eeprom_attributes[] = {
	&dev_attr_EepromId.attr,
	&dev_attr_EpdVcom.attr,
	&dev_attr_EpdWaveform.attr,
	&dev_attr_EpdWaveformOriginalFilename.attr,
	&dev_attr_EpdPartNum.attr,
	&dev_attr_EpdFplVer.attr,
	&dev_attr_EpdBarcode.attr,
	NULL
};

static struct attribute_group epd_eeprom_attribute_group = {
	.attrs = epd_eeprom_attributes
};


static int avocet_spi_probe(struct spi_device *spi)
{
	int err = 0;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi_setup(spi);

	/* If we don't request this pin. It will be autorequested by kernel.
	 * and cannot be released for pmic driver use again.*/
	err = gpio_request(EEPROM_PWR_GPIO,  "EEPROM_PWR_GPIO");
	gpio_direction_output(EEPROM_PWR_GPIO, 0);
	if (!err) {
		// Only free the GPIO if the request was successful; if another
		// driver already has this pin, it's OK.
		gpio_free(EEPROM_PWR_GPIO);
	}

	gpio_request(GPIO_SPI_CLK,  "EEPROM_SPI_CLK");
	gpio_direction_output(GPIO_SPI_CLK, 1);
	gpio_request(GPIO_SPI_SIMO, "EEPROM_SPI_SIMO");
	gpio_direction_output(GPIO_SPI_SIMO, 1);
	gpio_request(GPIO_SPI_SOMI, "EEPROM_SPI_SOMI");
	gpio_direction_input(GPIO_SPI_SOMI);
	gpio_request(GPIO_SPI_CS,   "EEPROM_SPI_CS");
	gpio_direction_output(GPIO_SPI_CS, 1);

	avocet_spi_device = spi;
	pr_debug("spi_probe mode : %x, per_word %d, chip_select %d, speed %d, master_bus %d,master_cs %d \n",spi->mode,spi->bits_per_word,spi->chip_select,spi->max_speed_hz,spi->master->bus_num, spi->master->num_chipselect);

	// SYSFS entries
	err = sysfs_create_group(&avocet_spi_device->dev.kobj, &epd_eeprom_attribute_group);
	if (err)
	{
		printk("sysfs_create_group() failed!!\n");
		goto error;
	}

	epd_info_msb = 0;
	if (turn_on_eeprom() != 0) {
		err = -1;
		goto error;
	}

	// Default values for waveform size
	max_epd_waveform_size = (EPD_WAVEFORM_ADDR_END - EPD_WAVEFORM_ADDR_ST + 1);
	last_wvfm_addr = EPD_WAVEFORM_ADDR_END;

	return 0;

error:
	pr_err("%s: err = [%d]\n", __func__, err);
	return err;
}


static int avocet_spi_remove(struct spi_device *spi)
{
	sysfs_remove_group(&spi->dev.kobj, &epd_eeprom_attribute_group);

	// Release GPIOs
	gpio_free(GPIO_SPI_CLK);
	gpio_free(GPIO_SPI_SIMO);
	gpio_free(GPIO_SPI_SOMI);
	gpio_free(GPIO_SPI_CS);

	return 0;
}

#ifdef CONFIG_PM
static int avocet_spi_suspend(struct spi_device *pdev, pm_message_t state)
{
	/*Bring down all the gpios to avoid residual voltage*/
	gpio_direction_output(EEPROM_PWR_GPIO, 0);
	gpio_direction_output(GPIO_SPI_CLK, 0);
	gpio_direction_output(GPIO_SPI_SIMO, 0);
	gpio_direction_output(GPIO_SPI_CS, 0);

	return 0;
}

static int avocet_spi_resume(struct spi_device *pdev)
{
	gpio_direction_output(GPIO_SPI_CLK, 1);
	gpio_direction_output(GPIO_SPI_SIMO, 1);
	gpio_direction_output(GPIO_SPI_CS, 1);

	return 0;
}
#endif

static struct spi_driver avocet_eeprom_spi_driver = {
	.probe           = avocet_spi_probe,
	.remove	= __devexit_p(avocet_spi_remove),
	.driver         = {
		.name   = "epd_eeprom_spi",
		.bus    = &spi_bus_type,
		.owner  = THIS_MODULE,
	},
#ifdef CONFIG_PM
	.suspend =avocet_spi_suspend,
	.resume  =avocet_spi_resume,
#endif
};

static int __init avocet_epdeeprom_init(void)
{

	return spi_register_driver(&avocet_eeprom_spi_driver);
}

static void __exit avocet_epdeeprom_exit(void)
{
    spi_unregister_driver(&avocet_eeprom_spi_driver);
}


module_init(avocet_epdeeprom_init);
module_exit(avocet_epdeeprom_exit);
MODULE_LICENSE("GPL");

