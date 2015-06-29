/* Lite-On LTR-553ALS Android / Linux Driver
 *
 * Copyright (C) 2013-2014 Lite-On Technology Corp (Singapore)
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */


#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/version.h>
#include <linux/regulator/consumer.h>
#include <linux/sensors.h>
#include <linux/of_gpio.h>
/* LTR-553 Registers */
#define LTR553_ALS_CONTR	0x80
#define LTR553_PS_CONTR		0x81
#define LTR553_PS_LED		0x82
#define LTR553_PS_N_PULSES	0x83
#define LTR553_PS_MEAS_RATE	0x84
#define LTR553_ALS_MEAS_RATE	0x85
#define LTR553_PART_ID		0x86
#define LTR553_MANUFACTURER_ID	0x87
#define LTR553_ALS_DATA_CH1_0	0x88
#define LTR553_ALS_DATA_CH1_1	0x89
#define LTR553_ALS_DATA_CH0_0	0x8A
#define LTR553_ALS_DATA_CH0_1	0x8B
#define LTR553_ALS_PS_STATUS	0x8C
#define LTR553_PS_DATA_0	0x8D
#define LTR553_PS_DATA_1	0x8E
#define LTR553_INTERRUPT	0x8F
#define LTR553_PS_THRES_UP_0	0x90
#define LTR553_PS_THRES_UP_1	0x91
#define LTR553_PS_THRES_LOW_0	0x92
#define LTR553_PS_THRES_LOW_1	0x93
#define LTR553_PS_OFFSET_1	0x94
#define LTR553_PS_OFFSET_0	0x95
#define LTR553_ALS_THRES_UP_0	0x97
#define LTR553_ALS_THRES_UP_1	0x98
#define LTR553_ALS_THRES_LOW_0	0x99
#define LTR553_ALS_THRES_LOW_1	0x9A
#define LTR553_INTERRUPT_PRST	0x9E
/* LTR-553 Registers */

/* POWER SUPPLY VOLTAGE RANGE */
#define LTR553_VDD_MIN_UV  2000000
#define LTR553_VDD_MAX_UV  3300000
#define LTR553_VIO_MIN_UV  1750000
#define LTR553_VIO_MAX_UV  1950000

#define SET_BIT 1
#define CLR_BIT 0

#define ALS 0
#define PS 1
#define ALSPS 2
/*#define PS_W_SATURATION_BIT	3*/

/* address 0x80 */
#define ALS_MODE_ACTIVE	(1 << 0)
#define ALS_MODE_STDBY		(0 << 0)
#define ALS_SW_RESET		(1 << 1)
#define ALS_SW_NRESET		(0 << 1)
#define ALS_GAIN_1x		(0 << 2)
#define ALS_GAIN_2x		(1 << 2)
#define ALS_GAIN_4x		(2 << 2)
#define ALS_GAIN_8x		(3 << 2)
#define ALS_GAIN_48x	(6 << 2)
#define ALS_GAIN_96x	(7 << 2)
#define ALS_MODE_RDBCK			0
#define ALS_SWRT_RDBCK			1
#define ALS_GAIN_RDBCK			2
#define ALS_CONTR_RDBCK		3

/* address 0x81 */
#define PS_MODE_ACTIVE		(3 << 0)
#define PS_MODE_STDBY		(0 << 0)
#define PS_GAIN_16x			(0 << 2)
#define PS_GAIN_32x			(2 << 2)
#define PS_GAIN_64x			(3 << 2)
#define PS_SATUR_INDIC_EN	(1 << 5)
#define PS_SATU_INDIC_DIS	(0 << 5)
#define PS_MODE_RDBCK		0
#define PS_GAIN_RDBCK		1
#define PS_SATUR_RDBCK		2
#define PS_CONTR_RDBCK		3

/* address 0x82 */
#define LED_CURR_5MA		(0 << 0)
#define LED_CURR_10MA		(1 << 0)
#define LED_CURR_20MA		(2 << 0)
#define LED_CURR_50MA		(3 << 0)
#define LED_CURR_100MA		(4 << 0)
#define LED_CURR_DUTY_25PC		(0 << 3)
#define LED_CURR_DUTY_50PC		(1 << 3)
#define LED_CURR_DUTY_75PC		(2 << 3)
#define LED_CURR_DUTY_100PC	(3 << 3)
#define LED_PUL_FREQ_30KHZ		(0 << 5)
#define LED_PUL_FREQ_40KHZ		(1 << 5)
#define LED_PUL_FREQ_50KHZ		(2 << 5)
#define LED_PUL_FREQ_60KHZ		(3 << 5)
#define LED_PUL_FREQ_70KHZ		(4 << 5)
#define LED_PUL_FREQ_80KHZ		(5 << 5)
#define LED_PUL_FREQ_90KHZ		(6 << 5)
#define LED_PUL_FREQ_100KHZ	(7 << 5)
#define LED_CURR_RDBCK			0
#define LED_CURR_DUTY_RDBCK	1
#define LED_PUL_FREQ_RDBCK		2
#define PS_LED_RDBCK			3

/* address 0x84 */
#define PS_MEAS_RPT_RATE_50MS		(0 << 0)
#define PS_MEAS_RPT_RATE_70MS		(1 << 0)
#define PS_MEAS_RPT_RATE_100MS	(2 << 0)
#define PS_MEAS_RPT_RATE_200MS	(3 << 0)
#define PS_MEAS_RPT_RATE_500MS	(4 << 0)
#define PS_MEAS_RPT_RATE_1000MS	(5 << 0)
#define PS_MEAS_RPT_RATE_2000MS	(6 << 0)
#define PS_MEAS_RPT_RATE_10MS		(8 << 0)

/* address 0x85 */
#define ALS_MEAS_RPT_RATE_50MS	(0 << 0)
#define ALS_MEAS_RPT_RATE_100MS	(1 << 0)
#define ALS_MEAS_RPT_RATE_200MS	(2 << 0)
#define ALS_MEAS_RPT_RATE_500MS	(3 << 0)
#define ALS_MEAS_RPT_RATE_1000MS	(4 << 0)
#define ALS_MEAS_RPT_RATE_2000MS	(5 << 0)
#define ALS_INTEG_TM_100MS		(0 << 3)
#define ALS_INTEG_TM_50MS			(1 << 3)
#define ALS_INTEG_TM_200MS		(2 << 3)
#define ALS_INTEG_TM_400MS		(3 << 3)
#define ALS_INTEG_TM_150MS		(4 << 3)
#define ALS_INTEG_TM_250MS		(5 << 3)
#define ALS_INTEG_TM_300MS		(6 << 3)
#define ALS_INTEG_TM_350MS		(7 << 3)
#define ALS_MEAS_RPT_RATE_RDBCK	0
#define ALS_INTEG_TM_RDBCK			1
#define ALS_MEAS_RATE_RDBCK		2

/* address 0x86 */
#define PART_NUM_ID_RDBCK		0
#define REVISION_ID_RDBCK		1
#define PART_ID_REG_RDBCK		2

/* address 0x8C */
#define PS_DATA_STATUS_RDBCK		0
#define PS_INTERR_STATUS_RDBCK	1
#define ALS_DATA_STATUS_RDBCK		2
#define ALS_INTERR_STATUS_RDBCK	3
#define ALS_GAIN_STATUS_RDBCK		4
#define ALS_VALID_STATUS_RDBCK	5
#define ALS_PS_STATUS_RDBCK		6

/* address 0x8F */
#define INT_MODE_00					(0 << 0)
#define INT_MODE_PS_TRIG			(1 << 0)
#define INT_MODE_ALS_TRIG			(2 << 0)
#define INT_MODE_ALSPS_TRIG		(3 << 0)
#define INT_POLAR_ACT_LO			(0 << 2)
#define INT_POLAR_ACT_HI			(1 << 2)
#define INT_MODE_RDBCK				0
#define INT_POLAR_RDBCK			1
#define INT_INTERRUPT_RDBCK		2

/* address 0x9E */
#define ALS_PERSIST_SHIFT	0
#define PS_PERSIST_SHIFT	4
#define ALS_PRST_RDBCK		0
#define PS_PRST_RDBCK		1
#define ALSPS_PRST_RDBCK	2

#define PON_DELAY		600

#define ALS_MIN_MEASURE_VAL	0
#define ALS_MAX_MEASURE_VAL	65535
#define ALS_VALID_MEASURE_MASK	ALS_MAX_MEASURE_VAL
#define PS_MIN_MEASURE_VAL	0
#define PS_MAX_MEASURE_VAL	2047
#define PS_VALID_MEASURE_MASK   PS_MAX_MEASURE_VAL
#define LO_LIMIT			0
#define HI_LIMIT			1
#define LO_N_HI_LIMIT	2
#define PS_OFFSET_MIN_VAL		0
#define PS_OFFSET_MAX_VAL		1023
#define ps_low_thres		800
#define ps_high_thres		300
#define		FAR_VAL		1
#define		NEAR_VAL		0

#define DRIVER_VERSION "1.13"
#define PARTID 0x92
#define MANUID 0x05

#define I2C_RETRY 5

#define DEVICE_NAME "LTR553ALSPS"

#define ACT_INTERRUPT 1

/*
 * Magic Number
 * ============
 * Refer to file ioctl-number.txt for allocation
 */
#define LTR553_IOCTL_MAGIC      'c'

/* IOCTLs for ltr553 device */
#define LTR553_IOCTL_PS_ENABLE		_IOR(LTR553_IOCTL_MAGIC, 1, int *)
#define LTR553_IOCTL_PS_GET_ENABLED	_IOW(LTR553_IOCTL_MAGIC, 2, int *)
#define LTR553_IOCTL_ALS_ENABLE		_IOR(LTR553_IOCTL_MAGIC, 3, int *)
#define LTR553_IOCTL_ALS_GET_ENABLED	_IOW(LTR553_IOCTL_MAGIC, 4, int *)


//LINE<JIRA_ID><DATE20131218><add PS Calibration>zenghaihui
#define ALSPS_IOCTL_PS_CALI_START			_IOW(LTR553_IOCTL_MAGIC, 0x05, int[2])
#define ALSPS_IOCTL_PS_SET_CALI			_IOW(LTR553_IOCTL_MAGIC, 0x06, int[2])
#define ALSPS_IOCTL_PS_GET_CALI			_IOW(LTR553_IOCTL_MAGIC, 0x07, int[2])
#define ALSPS_IOCTL_PS_CLR_CALI			_IO(LTR553_IOCTL_MAGIC, 0x08)
#define ALSPS_IOCTL_PS_CALI_RAW_DATA				_IOW(LTR553_IOCTL_MAGIC, 0x09, int)
#define ALSPS_IOCTL_PS_DATA				_IOW(LTR553_IOCTL_MAGIC, 0x0A, int)

//LINE<JIRA_ID><DATE20141211><show lux in ffbm>zenghaihui
#define ALSPS_IOCTL_ALS_RAW_DATA				_IOW(LTR553_IOCTL_MAGIC, 0x0B, int)

//add for low temperature 
#define LOW_TEMPERATURE
#define Debug_For_LOW_TEMPERATURE


static uint16_t previous_ls_data;

/*(Linux RTOS)>*/
struct ltr553_platform_data {
	/* ALS */
	uint16_t pfd_levels[5];
	uint16_t pfd_als_lowthresh;
	uint16_t pfd_als_highthresh;
	int pfd_disable_als_on_suspend;

	/* PS */
	uint16_t pfd_ps_lowthresh;
	uint16_t pfd_ps_highthresh;
	int pfd_disable_ps_on_suspend;

	/* Interrupt */
	int pfd_gpio_int_no;

	/* power contrl */
	int (*power)(unsigned char onoff);
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(bool);
};

struct ltr553_data {
	/* Device */
	struct i2c_client *i2c_client;
	struct input_dev *als_input_dev;
	struct input_dev *ps_input_dev;
	struct workqueue_struct *workqueue;
	struct wake_lock ps_wake_lock;
	struct mutex bus_lock;
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;

	/* regulator data */
	bool power_on;
	struct regulator *vdd;
	struct regulator *vio;

	struct ltr553_platform_data *platform_data;

	/* Device mode
	 * 0 = ALS
	 * 1 = PS
	 */
	uint8_t mode;

	/* ALS */
	uint8_t als_enable_flag;
	/* uint8_t als_suspend_enable_flag; */
	uint8_t als_irq_flag;
	uint8_t als_opened;
	uint16_t als_lowthresh;
	uint16_t als_highthresh;
	uint16_t default_als_lowthresh;
	uint16_t default_als_highthresh;
	uint16_t *adc_levels;
	/* Flag to suspend ALS on suspend or not */
	uint8_t disable_als_on_suspend;

	/* PS */
	uint8_t ps_enable_flag;
	/* uint8_t ps_suspend_enable_flag; */
	uint8_t ps_irq_flag;
	uint8_t ps_opened;
	uint16_t ps_lowthresh;
	uint16_t ps_highthresh;
	uint16_t default_ps_lowthresh;
	uint16_t default_ps_highthresh;
	/* Flag to suspend PS on suspend or not */
	uint8_t disable_ps_on_suspend;

	/* LED */
	int led_pulse_freq;
	int led_duty_cyc;
	int led_peak_curr;
	int led_pulse_count;

	/* Interrupt */
	int irq;
	int gpio_int_no;
	int is_suspend;
};

/*
 * Global data
 */
static struct ltr553_data *sensor_info;
static struct sensors_classdev sensors_light_cdev = {
	.name = "light",
	.vendor = "liteon",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "30000",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "proximity",
	.vendor = "liteon",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

#define	PS_MAX_INIT_KEPT_DATA_COUNTER		8
#define	PS_MAX_MOV_AVG_KEPT_DATA_CTR		7

static uint16_t winfac1 = 10;/*100;*/
static uint16_t winfac2 = 10;/*80;*/
static uint16_t winfac3 = 10;/*44;*/
static uint8_t eqn_prev;
static uint8_t ratio_old;
//static uint16_t ps_init_kept_data[PS_MAX_INIT_KEPT_DATA_COUNTER];
//static uint16_t ps_ct_avg;
static uint8_t ps_grabData_stage;
//static uint32_t ftn_init;
//static uint32_t ftn_final;
//static uint32_t ntf_final;
//static uint16_t lux_val_prev;
static uint8_t ps_kept_data_counter;
//static uint16_t ps_movavg_data[PS_MAX_MOV_AVG_KEPT_DATA_CTR];
static uint8_t ps_movavg_data_counter;
//static uint16_t ps_movct_avg;
/*uint16_t ps_thresh_hi, ps_thresh_lo;*/


//LINE<JIRA_ID><DATE20131219><add PS Calibration>zenghaihui
#define PROXIMITY_CALI_DEFAULT_DATA  (0x0A)
//#define PROXIMITY_CALI_DEFAULT_THRESHOLD_HIGH_OFFSET (0x60)
//#define PROXIMITY_CALI_DEFAULT_THRESHOLD_LOW_OFFSET (0x5A)
#define PROXIMITY_CALI_DEFAULT_THRESHOLD_HIGH_OFFSET (0x153)
#define PROXIMITY_CALI_DEFAULT_THRESHOLD_LOW_OFFSET (0x5C)
#define PROXIMITY_CALI_DATA_PATH  "/persist/ps_cali_data"

static int g_ps_cali_flag = 0;
static int g_ps_base_value = 0;
static int g_read_ps_cali_flag =0;

//LINE<JIRA_ID><DATE20141127><add ps cali data in dts>zenghaihui
static int g_ps_default_base_value = 0;
static int g_ps_default_threshold_high_offset = 0;
static int g_ps_default_threshold_low_offset =0;

//LINE<JIRA_ID><DATE20141203><detect ffbm in ltr553>zenghaihui
static int g_ffbm_flag =0xffff;
static void ltr553_read_ffbm_flag(void);

static void ltr553_write_ps_cali_data(int vl_flag, int vl_ps_data);
static void ltr553_read_ps_cali_data(void);
static int ltr553_read_lcd_brightness(void);

/* I2C Read */
/* take note ---------------------------------------
 for i2c read, need to send the register address follwed by buffer over
 to register.
 There should not be a stop in between register address and buffer.
 There should not be release of lock in between register address and buffer.
 take note ---------------------------------------*/
static int8_t I2C_Read(uint8_t *rxData, uint8_t length)
{
	int8_t index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter, data, 2) > 0)
			break;

		usleep(10000);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Read Fail !!!!\n", __func__);
		return -EIO;
	}

	return 0;
}


/* I2C Write */
static int8_t I2C_Write(uint8_t *txData, uint8_t length)
{
	int8_t index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter, data, 1) > 0)
			break;

		usleep(10000);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Write Fail !!!!\n", __func__);
		return -EIO;
	}

	return 0;
}


/* Set register bit */
static int8_t _ltr553_set_bit(struct i2c_client *client, uint8_t set,
						uint8_t cmd, uint8_t data)
{
	uint8_t buffer[2];
	uint8_t value;
	int8_t ret = 0;

	buffer[0] = cmd;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (set)
		value |= data;
	else
		value &= ~data;

	buffer[0] = cmd;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return -EIO;
	}

	return ret;
}

#if 0
//LINE<FFBAKK-119><DATE20141008><als lux map>zenghaihui
static uint16_t g_lux_sensor_map[8] = 
{ 5,  12,  1000, 4500,  8000, 65535,  65535,  65535};
static uint16_t g_lux_to_sys_map[8] = 
{ 1,  4,  2500,  3500,  5500,  20000,  20000,  20000};

#define SENSOR_MAP_LENGTH  (sizeof(g_lux_sensor_map) / sizeof(g_lux_sensor_map[0]))

static int8_t orient_index=0;
static int8_t anti_bounce(int8_t index)
{
    return index;
}

#else

// MDP set <DATE20141215>
static uint16_t g_lux_sensor_map[12] = 
{10,95,136,389 ,2469,  8548,  13642,  19943,  27031,  34534,  42233,  50675};
static uint16_t g_lux_to_sys_map[12] = 
{10,95,136,389 ,2469,  8548,  13642,  19943,  27031,  34534,  42233,  50675};
#define SENSOR_MAP_LENGTH  (sizeof(g_lux_sensor_map) / sizeof(g_lux_sensor_map[0]))

static int8_t orient_index=0;
static int8_t anti_bounce(int8_t index)
{

	pr_alert("anti_bounce:index= %d | orient_index=%d", index,orient_index);
	if(index==orient_index)          // no change
		return index;

	if(((index==0)&&(orient_index!=1))||(index==11))    //min & max
	{
		orient_index=index;
		return index;
	}
		
	if(orient_index>index)	//dark 
	{
		if((orient_index>3)&&((orient_index-index)>3))
		{
			orient_index=index;
			return index;
		}
		else
		{
			index=orient_index;
			return index;
		}
			
		if((orient_index-index)>2)
		{
			orient_index=index;
			return index;
		}
		else
		{
			index=orient_index;
			return index;
		}			
	}

	else				//bright
	{
		if(orient_index<3)
		{
			orient_index=index;
			return index;
		}
		if (orient_index<5)
		{
			if((index-orient_index)>1)
				orient_index=index;
			else
				index=orient_index;

			return index;
		}	
		if (orient_index<8)
		{
			if((index-orient_index)>2)
				orient_index=index;
			else
				index=orient_index;

			return index;
		}			
		if (orient_index<9)
		{
			if(index>9)
				orient_index=index;

			return index;
		}
		else 
			index=orient_index;
		return index;
			
	}		

}
#endif


static uint16_t lux_formula(uint16_t ch0_adc, uint16_t ch1_adc, uint8_t eqtn)
{
	uint32_t luxval = 0;
	uint32_t luxval_i = 0;
	uint32_t luxval_f = 0;
	uint16_t ch0_coeff_i = 0;
	uint16_t ch1_coeff_i = 0;
	uint16_t ch0_coeff_f = 0;
	uint16_t ch1_coeff_f = 0;
	int8_t ret;
	uint8_t gain = 1, als_int_fac;
	uint8_t buffer[2];
	uint16_t win_fac = 0;
	int8_t fac = 1;
        //LINE<FFBAKK-119><DATE20141008><als lux map>zenghaihui
	int8_t vl_index = 0;
	uint16_t luxval_temp = 0;

	buffer[0] = LTR553_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	gain = (buffer[0] & 0x70);
	gain >>= 4;

	if (gain == 0)			/*gain 1*/
		gain = 1;
	else if (gain == 1)		/*gain 2*/
		gain = 2;
	else if (gain == 2)	/*gain 4*/
		gain = 4;
	else if (gain == 3)	/*gain 8*/
		gain = 8;
	else if (gain == 6)	/*gain 48*/
		gain = 48;
	else if (gain == 7)	/*gain 96*/
		gain = 96;

	buffer[0] = LTR553_ALS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	als_int_fac = buffer[0] & 0x38;
	als_int_fac >>= 3;

	if (als_int_fac == 0)
		als_int_fac = 10;
	else if (als_int_fac == 1)
		als_int_fac = 5;
	else if (als_int_fac == 2)
		als_int_fac = 20;
	else if (als_int_fac == 3)
		als_int_fac = 40;
	else if (als_int_fac == 4)
		als_int_fac = 15;
	else if (als_int_fac == 5)
		als_int_fac = 25;
	else if (als_int_fac == 6)
		als_int_fac = 30;
	else if (als_int_fac == 7)
		als_int_fac = 35;

	if (eqtn == 1) {
		ch0_coeff_i = 1;
		ch1_coeff_i = 1;
		ch0_coeff_f = 7743;
		ch1_coeff_f = 1059;
		fac = 1;
		win_fac = winfac1;
		luxval_i = ((ch0_adc * ch0_coeff_i) +
			(ch1_adc * ch1_coeff_i)) * win_fac;
		luxval_f = (((ch0_adc * ch0_coeff_f) +
			(ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		/*luxval = ((17743 * ch0_calc) + (11059 * ch1_adc));*/
		/*luxval = ((1.7743 * ch0_calc) + (1.1059 * ch1_adc)) /
			(gain * (als_int_fac / 10));*/
	} else if (eqtn == 2) {
		ch0_coeff_i = 4;
		ch1_coeff_i = 1;
		ch0_coeff_f = 2785;
		ch1_coeff_f = 9548;/*696;*/
		win_fac = winfac2;
		if ((ch1_coeff_f * ch1_adc) < (ch0_adc * ch0_coeff_f)) {
			fac = 1;
			luxval_f = (((ch0_adc * ch0_coeff_f) -
			 (ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		} else {
			fac = -1;
			luxval_f = (((ch1_adc * ch1_coeff_f) -
			 (ch0_adc * ch0_coeff_f)) / 100) * win_fac;
		}
		luxval_i = ((ch0_adc * ch0_coeff_i) - (ch1_adc * ch1_coeff_i))
				 * win_fac;
		/*luxval = ((42785 * ch0_calc) - (10696 * ch1_adc));*/
		/*luxval = ((4.2785 * ch0_calc) - (1.9548 * ch1_adc)) /
					 (gain * (als_int_fac / 10));*/
	} else if (eqtn == 3) {
		ch0_coeff_i = 0;
		ch1_coeff_i = 0;
		ch0_coeff_f = 5926;
		ch1_coeff_f = 1185;/*1300;*/
		fac = 1;
		win_fac = winfac3;
		luxval_i = ((ch0_adc * ch0_coeff_i) +
				(ch1_adc * ch1_coeff_i)) * win_fac;
		luxval_f = (((ch0_adc * ch0_coeff_f) +
				(ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		/*luxval = ((5926 * ch0_calc) + (1185 * ch1_adc));*/
		/*luxval = ((0.5926 * ch0_calc) + (0.1185 * ch1_adc)) /
					 (gain * (als_int_fac / 10));*/
	} else if (eqtn == 4) {
		ch0_coeff_i = 0;
		ch1_coeff_i = 0;
		ch0_coeff_f = 0;
		ch1_coeff_f = 0;
		fac = 1;
		luxval_i = 0;
		luxval_f = 0;
		/*luxval = 0;*/
	}
	if (fac < 0)
		luxval = (luxval_i  -  (luxval_f / 100)) /
					(gain * als_int_fac);
	else if (fac == 1)
		luxval = (luxval_i  + ((fac) * luxval_f) / 100) /
					(gain * als_int_fac);


        //LINE<FFBAKK-119><DATE20141008><als lux map>zenghaihui
        luxval_temp = luxval;
        
        for(vl_index = 0; vl_index < SENSOR_MAP_LENGTH; vl_index++)
        {
            if(luxval_temp > g_lux_sensor_map[vl_index])
                continue;

            break;
        }

        if(vl_index >= SENSOR_MAP_LENGTH)
            vl_index = SENSOR_MAP_LENGTH -1;

		vl_index=anti_bounce(vl_index);
		
	pr_alert("after anti_bounce:vl_index= %d | orient_index=%d", vl_index,orient_index);
		
	return g_lux_to_sys_map[vl_index];
    
	//return luxval;
}


static uint16_t ratioHysterisis(uint16_t ch0_adc, uint16_t ch1_adc)
{
#define	RATIO_HYSVAL	10
	int ratio;
	uint8_t buffer[2], eqn_now;
	int8_t ret;
	uint16_t ch0_calc;
	uint32_t luxval = 0;
	int abs_ratio_now_old;

	buffer[0] = LTR553_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	ch0_calc = ch0_adc;
	if ((buffer[0] & 0x20) == 0x20)
		ch0_calc = ch0_adc - ch1_adc;

	if ((ch1_adc + ch0_calc) == 0)
		ratio = 100;
	else
		ratio = (ch1_adc*100) / (ch1_adc + ch0_calc);

	if (ratio < 45)
		eqn_now = 1;
	else if ((ratio >= 45) && (ratio < 68))
		eqn_now = 2;
	else if ((ratio >= 68) && (ratio < 99))
		eqn_now = 3;
	else if (ratio >= 99)
		eqn_now = 4;

	if (eqn_prev == 0) {
		luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
		ratio_old = ratio;
		eqn_prev = eqn_now;
	} else {
		if (eqn_now == eqn_prev) {
			luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
			ratio_old = ratio;
			eqn_prev = eqn_now;
		} else {
			abs_ratio_now_old = ratio - ratio_old;
			if (abs_ratio_now_old < 0)
				abs_ratio_now_old *= (-1);
			if (abs_ratio_now_old > RATIO_HYSVAL) {
				luxval = lux_formula(ch0_calc,
				ch1_adc, eqn_now);
				ratio_old = ratio;
				eqn_prev = eqn_now;
			} else {
				luxval = lux_formula(ch0_calc,
				ch1_adc, eqn_prev);
			}
		}
	}

	return luxval;
}

static uint16_t read_als_adc_value(struct ltr553_data *ltr553)
{
	int8_t ret = -99;
    	uint16_t value = -99;
        uint8_t buffer[4];
        int ratio;
        int ch0_val;
	int ch1_val;
        //LINE<FFBAKK-119><DATE20141008><als lux map>zenghaihui
	int8_t vl_index = 0;
	uint16_t luxval_temp = 0;
    
	/* ALS */
	buffer[0] = LTR553_ALS_DATA_CH1_0;

	/* read data bytes from data regs */
	ret = I2C_Read(buffer, 4);

	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	/* ALS Ch0 */
	ch0_val = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);
		dev_dbg(&ltr553->i2c_client->dev,
			"%s | als_ch0 value = 0x%04X\n", __func__,
				ch0_val);

	/* ALS Ch1 */
	ch1_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
		dev_dbg(&ltr553->i2c_client->dev,
			"%s | als_ch1 value = 0x%04X\n", __func__,
				ch1_val);

        if((ch0_val + ch1_val) == 0){
            ratio = 1000;
        } else{
            ratio = (ch1_val*1000)/(ch0_val + ch1_val);
        }

        if(ratio < 450){
            value =( (17743 * ch0_val) + (11059 * ch1_val))/10000;
        } else if((ratio >= 450) && (ratio < 640)){
            value = ((42785 * ch0_val) - (19548 * ch1_val))/10000;            
        }else if((ratio >= 640) && (ratio < 900)){
            value = ((5926 * ch0_val) + (1185 * ch1_val))/10000;  
        }else{
            value = 0;
        }

        if( value > 65530){
            value = 65535;
        }    


    
        luxval_temp = value;
        
        for(vl_index = 0; vl_index < SENSOR_MAP_LENGTH; vl_index++)
        {
            if(luxval_temp > g_lux_sensor_map[vl_index])
                continue;

            break;
        }

        if(vl_index >= SENSOR_MAP_LENGTH)
            vl_index = SENSOR_MAP_LENGTH-1;
            

		vl_index=anti_bounce(vl_index);
		
		
	pr_alert("after anti_bounce:vl_index= %d | orient_index=%d", vl_index,orient_index);
		
	return g_lux_to_sys_map[vl_index];
    
       //return value;     

}


//LINE<JIRA_ID><DATE20141211><show lux in ffbm>zenghaihui
static uint16_t read_als_adc_raw_value(struct ltr553_data *ltr553)
{
	int8_t ret = -99;
    	uint16_t value = -99;
        uint8_t buffer[4];
        int ratio;
        int ch0_val;
	int ch1_val;
        //LINE<FFBAKK-119><DATE20141008><als lux map>zenghaihui
	//int8_t vl_index = 0;
	//uint16_t luxval_temp = 0;
    
	/* ALS */
	buffer[0] = LTR553_ALS_DATA_CH1_0;

	/* read data bytes from data regs */
	ret = I2C_Read(buffer, 4);

	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	/* ALS Ch0 */
	ch0_val = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);
		dev_dbg(&ltr553->i2c_client->dev,
			"%s | als_ch0 value = 0x%04X\n", __func__,
				ch0_val);

	/* ALS Ch1 */
	ch1_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
		dev_dbg(&ltr553->i2c_client->dev,
			"%s | als_ch1 value = 0x%04X\n", __func__,
				ch1_val);

        if((ch0_val + ch1_val) == 0){
            ratio = 1000;
        } else{
            ratio = (ch1_val*1000)/(ch0_val + ch1_val);
        }

        if(ratio < 450){
            value =( (17743 * ch0_val) + (11059 * ch1_val))/10000;
        } else if((ratio >= 450) && (ratio < 640)){
            value = ((42785 * ch0_val) - (19548 * ch1_val))/10000;            
        }else if((ratio >= 640) && (ratio < 900)){
            value = ((5926 * ch0_val) + (1185 * ch1_val))/10000;  
        }else{
            value = 0;
        }

        if( value > 65530){
            value = 65535;
        }    
    
       return value;     

}



#if 0
static uint16_t read_als_adc_value_temp(struct ltr553_data *ltr553)
{

	int8_t ret = -99;
	uint16_t value = -99;
	int ch0_val;
	int ch1_val;
	uint8_t gain, value_temp, gain_chg_req = 0;
	uint8_t buffer[4], temp;

#define AGC_UP_THRESHOLD		40000
#define AGC_DOWN_THRESHOLD	5000
#define AGC_HYS					15
#define MAX_VAL					50000

	/* ALS */
	buffer[0] = LTR553_ALS_DATA_CH1_0;

	/* read data bytes from data regs */
	ret = I2C_Read(buffer, 4);

	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	/* ALS Ch0 */
	ch0_val = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);
		dev_dbg(&ltr553->i2c_client->dev,
			"%s | als_ch0 value = 0x%04X\n", __func__,
				ch0_val);

	if (ch0_val > ALS_MAX_MEASURE_VAL) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS Value Error: 0x%X\n", __func__,
					ch0_val);
	}
	ch0_val &= ALS_VALID_MEASURE_MASK;
	/*input_report_abs(ltr553->als_input_dev, ABS_MISC, ch0_val);*/
	/*input_sync(ltr553->als_input_dev);*/

	/* ALS Ch1 */
	ch1_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
		dev_dbg(&ltr553->i2c_client->dev,
			"%s | als_ch1 value = 0x%04X\n", __func__,
				ch1_val);

	if (ch1_val > ALS_MAX_MEASURE_VAL) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS Value Error: 0x%X\n", __func__,
					ch1_val);
	}
	ch1_val &= ALS_VALID_MEASURE_MASK;
	/*input_report_abs(ltr553->als_input_dev, ABS_MISC, ch1_val);*/
	/*input_sync(ltr553->als_input_dev);*/

	buffer[0] = LTR553_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	value_temp = buffer[0];
	temp = buffer[0];
	gain = (value_temp & 0x70);
	gain >>= 4;

	if (gain == 0) {			/*gain 1*/
		gain = 1;
	} else if (gain == 1) {		/*gain 2*/
		gain = 2;
	} else if (gain == 2) {		/*gain 4*/
		gain = 4;
	} else if (gain == 3) {		/*gain 8*/
		gain = 8;
	} else if (gain == 6) {		/*gain 48*/
		gain = 48;
	} else if (gain == 7) {		/*gain 96*/
		gain = 96;
	}

	buffer[0] = LTR553_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}
	value_temp = buffer[0];
	value_temp &= 0xE3;

	if ((ch0_val == 0) && (ch1_val > 50))
		value = lux_val_prev;
	else {
		if (gain == 1) {
			if ((ch0_val + ch1_val) <
				((AGC_DOWN_THRESHOLD * 10) / AGC_HYS)) {
				value = ratioHysterisis(ch0_val, ch1_val);
				value_temp |= ALS_GAIN_8x;
				gain_chg_req = 1;
			} else {
				value = ratioHysterisis(ch0_val, ch1_val);
			}
		} else if (gain == 8) {
			if ((ch0_val + ch1_val) > AGC_UP_THRESHOLD) {
				value = ratioHysterisis(ch0_val, ch1_val);
				value_temp |= ALS_GAIN_1x;
				gain_chg_req = 1;
			} else {
				value = ratioHysterisis(ch0_val, ch1_val);
			}
		} else {
			value = ratioHysterisis(ch0_val, ch1_val);
		}
		if (gain_chg_req) {
			buffer[0] = LTR553_ALS_CONTR;
			buffer[1] = value_temp;
			ret = I2C_Write(buffer, 2);
			if (ret < 0) {
				dev_err(&ltr553->i2c_client->dev,
					"%s | 0x%02X", __func__, buffer[0]);
				return ret;
			}
		}

	}

	if ((value > MAX_VAL) || (((ch0_val + ch1_val) >
			MAX_VAL) && (temp & 0x80)))
		value = MAX_VAL;

	lux_val_prev = value;

	return value;
}
#endif

#ifdef LOW_TEMPERATURE
//static uint16_t first_value_storage_highthreshold=0;
//static uint16_t first_value_storage_lowthreshold=0;
static uint16_t min_value_low_temperature=0x7ff;
static uint16_t set_first_value_flag_low_temperature=0;
static uint16_t reset_flag_low_temperature=0;

static int8_t set_ps_range(uint16_t lt, uint16_t ht, uint8_t lo_hi,struct ltr553_data *ltr553);


static uint16_t set_low_temperature_threshold_value(struct ltr553_data *ltr553,uint16_t ps)
{
	uint16_t temp_value_high;
	uint16_t temp_value_low;

#ifdef Debug_For_LOW_TEMPERATURE	
	pr_alert("[ltr553-MDP] reset_low_temperature_threshold_value:  ps=%d \n",ps);
	pr_alert("[ltr553-MDP] reset_low_temperature_threshold_value:reset_flag_low_temperature= %d  set_first_value_flag_low_temperature=%d \n", reset_flag_low_temperature,set_first_value_flag_low_temperature);
#endif


	if(reset_flag_low_temperature==1)
	{
		goto end_set_threshold;
	}
	
#ifdef Debug_For_LOW_TEMPERATURE	
	pr_alert("[ltr553-MDP] reset_low_temperature_threshold_value:g_ps_base_value= %d \n", g_ps_base_value);
	pr_alert("[ltr553-MDP] reset_low_temperature_threshold_value:ltr553->default_ps_highthresh= %d  ltr553->default_ps_lowthresh=%d \n", ltr553->default_ps_highthresh,ltr553->default_ps_lowthresh);
#endif

	if(min_value_low_temperature>ps)
	{
		min_value_low_temperature=ps;
	}

	if(g_ps_base_value+50>ps)
	{
		min_value_low_temperature=g_ps_base_value;
		reset_flag_low_temperature=1;
		
	}
	temp_value_high=min_value_low_temperature+g_ps_default_threshold_high_offset;
	temp_value_low=min_value_low_temperature+g_ps_default_threshold_low_offset;
	
	if(set_first_value_flag_low_temperature==0)
	{
		if(g_ps_base_value+50<ps)
		{
			#if 0
			first_value_storage_highthreshold=ltr553->default_ps_highthresh;
			first_value_storage_lowthreshold=ltr553->default_ps_lowthresh;
			ltr553->default_ps_highthresh =temp_value_high;
		    	ltr553->default_ps_lowthresh = temp_value_low;
			if( ltr553->default_ps_highthresh > PS_MAX_MEASURE_VAL)
			{
			        ltr553->default_ps_highthresh= PS_MAX_MEASURE_VAL -100;
			        ltr553->default_ps_lowthresh= PS_MAX_MEASURE_VAL -90;
			        pr_info("ltr553_ps_cali_set_threshold: set value_high=0x7f0,value_low=0x7e0, please check the phone \n");
			}
		   	 set_ps_range(ltr553->default_ps_highthresh,ltr553->default_ps_lowthresh, LO_N_HI_LIMIT, ltr553);
			#endif	
			
		#ifdef Debug_For_LOW_TEMPERATURE	
			pr_info("[ltr553-MDP]: set_low_temperature_threshold_value:PS is larger than g_ps_base_value, and threshold  need to be reset! \n");
			pr_info("[ltr553-MDP]: ltr553_ps_cali_set_threshold: ltr553  default_ps_highthresh=%x, default_ps_lowthresh=%x! \n", ltr553->default_ps_highthresh, ltr553->default_ps_lowthresh);
		#endif		


		}
		else
		{
			reset_flag_low_temperature=1;
			set_first_value_flag_low_temperature=1;
			
		#ifdef Debug_For_LOW_TEMPERATURE
			pr_info("[ltr553-MDP]: set_low_temperature_threshold_value:There is no need to set threshold! \n");
		#endif
		
			goto end_set_threshold;		
		}
				
		set_first_value_flag_low_temperature=1;
	}
	
	ltr553->default_ps_highthresh = temp_value_high;
	ltr553->default_ps_lowthresh = temp_value_low;
	if( ltr553->default_ps_highthresh > PS_MAX_MEASURE_VAL)
	{
		//ltr553->default_ps_highthresh= PS_MAX_MEASURE_VAL -100;
		//ltr553->default_ps_lowthresh= PS_MAX_MEASURE_VAL -90;
		ltr553->default_ps_highthresh= PS_MAX_MEASURE_VAL ;
		ltr553->default_ps_lowthresh= PS_MAX_MEASURE_VAL ;
		pr_info("ltr553_ps_cali_set_threshold: set value_high=0x7ff,value_low=0x7ff, please check the phone \n");
	}
	
#ifdef Debug_For_LOW_TEMPERATURE
	pr_info("[ltr553-MDP]: ltr553_ps_cali_set_threshold: ltr553  default_ps_highthresh=%x, default_ps_lowthresh=%x! \n", ltr553->default_ps_highthresh, ltr553->default_ps_lowthresh);
#endif

	set_ps_range(ltr553->default_ps_highthresh,ltr553->default_ps_lowthresh, LO_N_HI_LIMIT, ltr553);

	return ps;

	end_set_threshold: 
		return ps;
}

#endif

static uint16_t read_ps_adc_value(struct ltr553_data *ltr553)
{
	int8_t ret = -99;
	uint16_t value = -99;
	uint16_t ps_val;
	uint8_t buffer[4];

	buffer[0] = LTR553_PS_DATA_0;

	/* read data bytes from data regs */
	ret = I2C_Read(buffer, 2);

	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	ps_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
		dev_dbg(&ltr553->i2c_client->dev,
			"%s | ps value = 0x%04X\n", __func__,
			ps_val);

	if (ps_val > PS_MAX_MEASURE_VAL) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS Value Error: 0x%X\n", __func__,
					ps_val);
	}
	ps_val &= PS_VALID_MEASURE_MASK;

	value = ps_val;
	
#ifdef LOW_TEMPERATURE
	ps_val=set_low_temperature_threshold_value(ltr553,value);

#endif

	

	return value;
}


static int8_t als_mode_setup(uint8_t alsMode_set_reset,
					 struct ltr553_data *ltr553)
{
	int8_t ret = 0;

	ret = _ltr553_set_bit(ltr553->i2c_client, alsMode_set_reset,
				LTR553_ALS_CONTR, ALS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s ALS mode setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t als_sw_reset_setup(uint8_t alsSWReset_set_reset,
				 struct ltr553_data *ltr553)
{
	int8_t ret = 0;

	ret = _ltr553_set_bit(ltr553->i2c_client, alsSWReset_set_reset,
				LTR553_ALS_CONTR, ALS_SW_RESET);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s ALS sw reset setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t als_gain_setup(uint8_t alsgain_range, struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xE3;

	if (alsgain_range == 1)
		value |= ALS_GAIN_1x;
	else if (alsgain_range == 2)
		value |= ALS_GAIN_2x;
	else if (alsgain_range == 4)
		value |= ALS_GAIN_4x;
	else if (alsgain_range == 8)
		value |= ALS_GAIN_8x;
	else if (alsgain_range == 48)
		value |= ALS_GAIN_48x;
	else if (alsgain_range == 96)
		value |= ALS_GAIN_96x;

	buffer[0] = LTR553_ALS_CONTR;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s ALS gain setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t als_contr_setup(uint8_t als_contr_val, struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR553_ALS_CONTR;

	/* Default settings used for now. */
	buffer[1] = als_contr_val;
	buffer[1] &= 0x1F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | ALS_CONTR (0x%02X) setup fail...",
				__func__, buffer[0]);
	}

	return ret;
}


static int8_t als_contr_readback(uint8_t rdbck_type, uint8_t *retVal,
				struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == ALS_MODE_RDBCK)
		*retVal = value & 0x01;
	else if (rdbck_type == ALS_SWRT_RDBCK)
		*retVal = (value & 0x02) >> 1;
	else if (rdbck_type == ALS_GAIN_RDBCK)
		*retVal = (value & 0x1C) >> 2;
	else if (rdbck_type == ALS_CONTR_RDBCK)
		*retVal = value & 0x1F;

	return ret;
}


static int8_t ps_mode_setup(uint8_t psMode_set_reset,
				struct ltr553_data *ltr553)
{
	int8_t ret = 0;

	ret = _ltr553_set_bit(ltr553->i2c_client, psMode_set_reset,
					LTR553_PS_CONTR, PS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s PS mode setup fail...\n", __func__);
		return ret;
	}

	/* default state is far-away */
	input_report_abs(ltr553->ps_input_dev, ABS_DISTANCE, FAR_VAL);
	input_sync(ltr553->ps_input_dev);

	return ret;
}


static int8_t ps_gain_setup(uint8_t psgain_range, struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_PS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xF3;

	if (psgain_range == 16)
		value |= PS_GAIN_16x;
	else if (psgain_range == 32)
		value |= PS_GAIN_32x;
	else if (psgain_range == 64)
		value |= PS_GAIN_64x;

	buffer[0] = LTR553_PS_CONTR;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s PS gain setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_satu_indica_setup(uint8_t pssatuindica_enable,
				struct ltr553_data *ltr553)
{
	int8_t ret = 0;

	ret = _ltr553_set_bit(ltr553->i2c_client, pssatuindica_enable,
				LTR553_PS_CONTR, PS_SATUR_INDIC_EN);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s PS saturation indicator setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_contr_setup(uint8_t ps_contr_val,
				struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR553_PS_CONTR;

	/* Default settings used for now. */
	buffer[1] = ps_contr_val;
	buffer[1] &= 0x2F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | PS_CONTR (0x%02X) setup fail...",
			__func__, buffer[0]);
	}

	return ret;
}


static int8_t ps_contr_readback(uint8_t rdbck_type, uint8_t *retVal,
					struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_PS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == PS_MODE_RDBCK)
		*retVal = (value & 0x03);
	else if (rdbck_type == PS_GAIN_RDBCK)
		*retVal = (value & 0x0C) >> 2;
	else if (rdbck_type == PS_SATUR_RDBCK)
		*retVal = (value & 0x20) >> 5;
	else if (rdbck_type == PS_CONTR_RDBCK)
		*retVal = value & 0x2F;

	return ret;
}


static int8_t ps_ledCurrent_setup(uint8_t psledcurr_val,
					struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_PS_LED;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xF8;

	if (psledcurr_val == 5)
		value |= LED_CURR_5MA;
	else if (psledcurr_val == 10)
		value |= LED_CURR_10MA;
	else if (psledcurr_val == 20)
		value |= LED_CURR_20MA;
	else if (psledcurr_val == 50)
		value |= LED_CURR_50MA;
	else if (psledcurr_val == 100)
		value |= LED_CURR_100MA;

	buffer[0] = LTR553_PS_LED;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s PS LED current setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_ledCurrDuty_setup(uint8_t psleddutycycle_val,
					struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_PS_LED;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xE7;

	if (psleddutycycle_val == 25)
		value |= LED_CURR_DUTY_25PC;
	else if (psleddutycycle_val == 50)
		value |= LED_CURR_DUTY_50PC;
	else if (psleddutycycle_val == 75)
		value |= LED_CURR_DUTY_75PC;
	else if (psleddutycycle_val == 100)
		value |= LED_CURR_DUTY_100PC;

	buffer[0] = LTR553_PS_LED;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s PS LED current duty setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_ledPulseFreq_setup(uint8_t pspulreq_val,
				struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_PS_LED;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0x1F;

	if (pspulreq_val == 30)
		value |= LED_PUL_FREQ_30KHZ;
	else if (pspulreq_val == 40)
		value |= LED_PUL_FREQ_40KHZ;
	else if (pspulreq_val == 50)
		value |= LED_PUL_FREQ_50KHZ;
	else if (pspulreq_val == 60)
		value |= LED_PUL_FREQ_60KHZ;
	else if (pspulreq_val == 70)
		value |= LED_PUL_FREQ_70KHZ;
	else if (pspulreq_val == 80)
		value |= LED_PUL_FREQ_80KHZ;
	else if (pspulreq_val == 90)
		value |= LED_PUL_FREQ_90KHZ;
	else if (pspulreq_val == 100)
		value |= LED_PUL_FREQ_100KHZ;

	buffer[0] = LTR553_PS_LED;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s PS LED pulse frequency setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


/* LED Setup */
static int8_t ps_led_setup(uint8_t ps_led_val, struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR553_PS_LED;

	/* Default settings used for now. */
	buffer[1] = ps_led_val;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | PS_LED (0x%02X) setup fail...",
				__func__, buffer[0]);
	}

	return ret;
}


static int8_t ps_led_readback(uint8_t rdbck_type, uint8_t *retVal,
				struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_PS_LED;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == LED_CURR_RDBCK)
		*retVal = (value & 0x07);
	else if (rdbck_type == LED_CURR_DUTY_RDBCK)
		*retVal = (value & 0x18) >> 3;
	else if (rdbck_type == LED_PUL_FREQ_RDBCK)
		*retVal = (value & 0xE0) >> 5;
	else if (rdbck_type == PS_LED_RDBCK)
		*retVal = value;

	return ret;
}


static int8_t ps_ledPulseCount_setup(uint8_t pspulsecount_val,
				struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR553_PS_N_PULSES;

	/* Default settings used for now. */
	if (pspulsecount_val > 15)
		pspulsecount_val = 15;

	buffer[1] = pspulsecount_val;
	buffer[1] &= 0x0F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | PS_LED_COUNT (0x%02X) setup fail...",
				__func__, buffer[0]);
	}

	return ret;
}


static int8_t ps_ledPulseCount_readback(uint8_t *retVal,
				struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[3], value;

	buffer[0] = LTR553_PS_N_PULSES;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	*retVal = value;

	return ret;
}


static int8_t ps_meas_rate_setup(uint16_t meas_rate_val,
				struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_PS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	value = buffer[0];
	value &= 0xF0;

	if (meas_rate_val == 50)
		value |= PS_MEAS_RPT_RATE_50MS;
	else if (meas_rate_val == 70)
		value |= PS_MEAS_RPT_RATE_70MS;
	else if (meas_rate_val == 100)
		value |= PS_MEAS_RPT_RATE_100MS;
	else if (meas_rate_val == 200)
		value |= PS_MEAS_RPT_RATE_200MS;
	else if (meas_rate_val == 500)
		value |= PS_MEAS_RPT_RATE_500MS;
	else if (meas_rate_val == 1000)
		value |= PS_MEAS_RPT_RATE_1000MS;
	else if (meas_rate_val == 2000)
		value |= PS_MEAS_RPT_RATE_2000MS;
	else if (meas_rate_val == 10)
		value |= PS_MEAS_RPT_RATE_10MS;

	buffer[0] = LTR553_PS_MEAS_RATE;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s PS measurement rate setup fail...\n", __func__);

		return ret;
	}

	return ret;
}


static int8_t ps_meas_rate_readback(uint8_t *retVal,
				struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[3], value;

	buffer[0] = LTR553_PS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	*retVal = (value & 0x0F);

	return ret;
}


static int8_t als_meas_rate_setup(uint16_t meas_rate_val,
					struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_ALS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xF8;

	if (meas_rate_val == 50)
		value |= ALS_MEAS_RPT_RATE_50MS;
	else if (meas_rate_val == 100)
		value |= ALS_MEAS_RPT_RATE_100MS;
	else if (meas_rate_val == 200)
		value |= ALS_MEAS_RPT_RATE_200MS;
	else if (meas_rate_val == 500)
		value |= ALS_MEAS_RPT_RATE_500MS;
	else if (meas_rate_val == 1000)
		value |= ALS_MEAS_RPT_RATE_1000MS;
	else if (meas_rate_val == 2000)
		value |= ALS_MEAS_RPT_RATE_2000MS;

	buffer[0] = LTR553_ALS_MEAS_RATE;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s ALS measurement rate setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t als_integ_time_setup(uint16_t integ_time_val,
					struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_ALS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xC7;

	if (integ_time_val == 100)
		value |= ALS_INTEG_TM_100MS;
	else if (integ_time_val == 50)
		value |= ALS_INTEG_TM_50MS;
	else if (integ_time_val == 200)
		value |= ALS_INTEG_TM_200MS;
	else if (integ_time_val == 400)
		value |= ALS_INTEG_TM_400MS;
	else if (integ_time_val == 150)
		value |= ALS_INTEG_TM_150MS;
	else if (integ_time_val == 250)
		value |= ALS_INTEG_TM_250MS;
	else if (integ_time_val == 300)
		value |= ALS_INTEG_TM_300MS;
	else if (integ_time_val == 350)
		value |= ALS_INTEG_TM_350MS;

	buffer[0] = LTR553_ALS_MEAS_RATE;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s ALS integration time setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t als_meas_rate_reg_setup(uint8_t als_meas_rate_reg_val,
					struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR553_ALS_MEAS_RATE;

	buffer[1] = als_meas_rate_reg_val;
	buffer[1] &= 0x3F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | ALS_MEAS_RATE (0x%02X) setup fail...",
				__func__, buffer[0]);
	}

	return ret;
}


static int8_t als_meas_rate_readback(uint8_t rdbck_type, uint8_t *retVal,
						struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_ALS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == ALS_MEAS_RPT_RATE_RDBCK)
		*retVal = (value & 0x07);
	else if (rdbck_type == ALS_INTEG_TM_RDBCK)
		*retVal = (value & 0x38) >> 3;
	else if (rdbck_type == ALS_MEAS_RATE_RDBCK)
		*retVal = (value & 0x3F);

	return ret;
}


static int8_t part_ID_reg_readback(uint8_t rdbck_type, uint8_t *retVal,
					struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[1], value;

	buffer[0] = LTR553_PART_ID;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == PART_NUM_ID_RDBCK)
		*retVal = (value & 0xF0) >> 4;
	else if (rdbck_type == REVISION_ID_RDBCK)
		*retVal = value & 0x0F;
	else if (rdbck_type == PART_ID_REG_RDBCK)
		*retVal = value;

	return ret;
}


static int8_t manu_ID_reg_readback(uint8_t *retVal, struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[1], value;

	buffer[0] = LTR553_MANUFACTURER_ID;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	*retVal = value;

	return ret;
}


static int8_t als_ps_status_reg(uint8_t data_status_type, uint8_t *retVal,
						struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (data_status_type == PS_DATA_STATUS_RDBCK)
		*retVal = (value & 0x01);
	else if (data_status_type == PS_INTERR_STATUS_RDBCK)
		*retVal = (value & 0x02) >> 1;
	else if (data_status_type == ALS_DATA_STATUS_RDBCK)
		*retVal = (value & 0x04) >> 2;
	else if (data_status_type == ALS_INTERR_STATUS_RDBCK)
		*retVal = (value & 0x08) >> 3;
	else if (data_status_type == ALS_GAIN_STATUS_RDBCK)
		*retVal = (value & 0x70) >> 4;
	else if (data_status_type == ALS_VALID_STATUS_RDBCK)
		*retVal = (value & 0x80) >> 7;
	else if (data_status_type == ALS_PS_STATUS_RDBCK)
		*retVal = value;

	return ret;
}


static int8_t als_ch0ch1raw_calc_readback(uint16_t *retVal1, uint16_t *retVal2,
			uint16_t *retVal3, struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[11];
	uint16_t value1, value2, value3;

	buffer[0] = LTR553_ALS_DATA_CH1_0;
	ret = I2C_Read(buffer, 4);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value1 = ((int)buffer[2]) + ((int)buffer[3] << 8); /* CH0*/
	value2 = ((int)buffer[0]) + ((int)buffer[1] << 8); /* CH1*/

	value3 = ratioHysterisis(value1, value2);

	*retVal1 = value1;
	*retVal2 = value2;
	*retVal3 = value3;

	return ret;
}


static int8_t interrupt_mode_setup(uint8_t interr_mode_val,
					struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_INTERRUPT;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xFC;

	if (interr_mode_val == 0)
		value |= INT_MODE_00;
	else if (interr_mode_val == 1)
		value |= INT_MODE_PS_TRIG;
	else if (interr_mode_val == 2)
		value |= INT_MODE_ALS_TRIG;
	else if (interr_mode_val == 3)
		value |= INT_MODE_ALSPS_TRIG;

	buffer[0] = LTR553_INTERRUPT;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s Interrupt mode setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t interrupt_polarity_setup(uint8_t interr_polar_val,
						struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_INTERRUPT;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xFB;

	if (interr_polar_val == 0)
		value |= INT_POLAR_ACT_LO;
	else if (interr_polar_val == 1)
		value |= INT_POLAR_ACT_HI;

	buffer[0] = LTR553_INTERRUPT;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s Interrupt polarity setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t interrupt_setup(uint8_t interrupt_val,
			struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR553_INTERRUPT;

	/* Default settings used for now. */
	buffer[1] = interrupt_val;
	buffer[1] &= 0x07;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s |Interrupt (0x%02X) setup fail...",
				__func__, buffer[0]);
	}

	return ret;
}


static int8_t interrupt_readback(uint8_t rdbck_type, uint8_t *retVal,
					struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_INTERRUPT;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == INT_MODE_RDBCK)
		*retVal = (value & 0x03);
	else if (rdbck_type == INT_POLAR_RDBCK)
		*retVal = (value & 0x04) >> 2;
	else if (rdbck_type == INT_INTERRUPT_RDBCK)
		*retVal = (value & 0x07);

	return ret;
}


static int8_t ps_offset_setup(uint16_t ps_offset_val,
					struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR553_PS_OFFSET_1;
	buffer[1] = (ps_offset_val >> 8) & 0x03;
	buffer[2] = (ps_offset_val & 0xFF);

	ret = I2C_Write(buffer, 3);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s PS offset setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_offset_readback(uint16_t *offsetval,
				struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2];
	uint16_t value;

	buffer[0] = LTR553_PS_OFFSET_1;
	ret = I2C_Read(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value <<= 8;
	value += buffer[1];

	*offsetval = value;

	return ret;
}


static int8_t interrupt_persist_setup(uint8_t interr_persist_val,
					struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	value = interr_persist_val;

	buffer[0] = LTR553_INTERRUPT_PRST;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s Interrupt persist setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t interrupt_prst_readback(uint8_t *retVal,
						struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR553_INTERRUPT_PRST;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	*retVal = value;

	return ret;
}


/* Set ALS range */
static int8_t set_als_range(uint16_t lt, uint16_t ht, uint8_t lo_hi)
{
	int8_t ret;
	uint8_t buffer[5], num_data = 0;

	if (lo_hi == LO_LIMIT) {
		buffer[0] = LTR553_ALS_THRES_LOW_0;
		buffer[1] = lt & 0xFF;
		buffer[2] = (lt >> 8) & 0xFF;
		num_data = 3;
	} else if (lo_hi == HI_LIMIT) {
		buffer[0] = LTR553_ALS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0xFF;
		num_data = 3;
	} else if (lo_hi == LO_N_HI_LIMIT) {
		buffer[0] = LTR553_ALS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0xFF;
		buffer[3] = lt & 0xFF;
		buffer[4] = (lt >> 8) & 0xFF;
		num_data = 5;
	}

	ret = I2C_Write(buffer, num_data);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	dev_dbg(&sensor_info->i2c_client->dev,
		"%s Set als range:0x%04x - 0x%04x\n", __func__, lt, ht);

	return ret;
}


static int8_t als_range_readback(uint16_t *lt, uint16_t *ht,
					struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[5];
	uint16_t value_lo, value_hi;

	buffer[0] = LTR553_ALS_THRES_UP_0;
	ret = I2C_Read(buffer, 4);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value_lo = buffer[3];
	value_lo <<= 8;
	value_lo += buffer[2];
	*lt = value_lo;

	value_hi = buffer[1];
	value_hi <<= 8;
	value_hi += buffer[0];
	*ht = value_hi;

	return ret;
}


/* Set PS range */
static int8_t set_ps_range(uint16_t lt, uint16_t ht, uint8_t lo_hi,
					struct ltr553_data *ltr553)
{
	int8_t ret;
	uint8_t buffer[5], num_data = 0;

	if (lo_hi == LO_LIMIT) {
		buffer[0] = LTR553_PS_THRES_LOW_0;
		buffer[1] = lt & 0xFF;
		buffer[2] = (lt >> 8) & 0x07;
		num_data = 3;
	} else if (lo_hi == HI_LIMIT) {
		buffer[0] = LTR553_PS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0x07;
		num_data = 3;
	} else if (lo_hi == LO_N_HI_LIMIT) {
		buffer[0] = LTR553_PS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0x07;
		buffer[3] = lt & 0xFF;
		buffer[4] = (lt >> 8) & 0x07;
		num_data = 5;
	}

	ret = I2C_Write(buffer, num_data);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}
	dev_dbg(&ltr553->i2c_client->dev,
	"%s Set ps range:0x%04x - 0x%04x\n", __func__, lt, ht);

	return ret;
}


static int8_t ps_range_readback(uint16_t *lt, uint16_t *ht,
						struct ltr553_data *ltr553)
{
	int8_t ret = 0;
	uint8_t buffer[5];
	uint16_t value_lo, value_hi;

	buffer[0] = LTR553_PS_THRES_UP_0;
	ret = I2C_Read(buffer, 4);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	value_lo = buffer[3];
	value_lo <<= 8;
	value_lo += buffer[2];
	*lt = value_lo;

	value_hi = buffer[1];
	value_hi <<= 8;
	value_hi += buffer[0];
	*ht = value_hi;

	return ret;
}


//LINE<JIRA_ID><DATE20140930><ps modify>zenghaihui
#if 0
static uint16_t discardMinMax_findCTMov_Avg(uint16_t *ps_val)
{
#define MAX_NUM_PS_DATA1		PS_MAX_MOV_AVG_KEPT_DATA_CTR
#define STARTING_PS_INDEX1		0
#define ENDING_PS_INDEX1		5
#define NUM_AVG_DATA1			5

	uint8_t i_ctr, i_ctr2, maxIndex, minIndex;
	uint16_t maxVal, minVal, _ps_val[MAX_NUM_PS_DATA1];
	uint16_t temp = 0;

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++)
		_ps_val[i_ctr] = ps_val[i_ctr];

	maxVal = ps_val[STARTING_PS_INDEX1];
	maxIndex = STARTING_PS_INDEX1;
	minVal = ps_val[STARTING_PS_INDEX1];
	minIndex = STARTING_PS_INDEX1;

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
		if (ps_val[i_ctr] > maxVal) {
			maxVal = ps_val[i_ctr];
			maxIndex = i_ctr;
		}
	}

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
		if (ps_val[i_ctr] < minVal) {
			minVal = ps_val[i_ctr];
			minIndex = i_ctr;
		}
	}

	i_ctr2 = 0;

	if (minIndex != maxIndex) {
		for (i_ctr = STARTING_PS_INDEX1;
				i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
			if ((i_ctr != minIndex) && (i_ctr != maxIndex)) {
				ps_val[i_ctr2] = _ps_val[i_ctr];
				i_ctr2++;
			}
		}
	}
	ps_val[MAX_NUM_PS_DATA1 - 1] = 0;
	ps_val[MAX_NUM_PS_DATA1 - 2] = 0;

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < ENDING_PS_INDEX1; i_ctr++)
		temp += ps_val[i_ctr];

	temp = (temp / NUM_AVG_DATA1);

	return temp;
}
#endif

//LINE<JIRA_ID><DATE20141031><add ps calibration>zenghaihui
#if 0
static uint16_t findCT_Avg(uint16_t *ps_val)
{
#define MAX_NUM_PS_DATA2		PS_MAX_INIT_KEPT_DATA_COUNTER
#define STARTING_PS_INDEX2		3
#define NUM_AVG_DATA2			3

	uint8_t i_ctr, min_Index, max_Index;
	uint16_t max_val, min_val;
	uint16_t temp = 0;
	/*struct ltr553_data *ltr553 = sensor_info;*/

	max_val = ps_val[STARTING_PS_INDEX2];
	max_Index = STARTING_PS_INDEX2;
	min_val = ps_val[STARTING_PS_INDEX2];
	min_Index = STARTING_PS_INDEX2;

	for (i_ctr = STARTING_PS_INDEX2; i_ctr < MAX_NUM_PS_DATA2; i_ctr++) {
		if (ps_val[i_ctr] > max_val) {
			max_val = ps_val[i_ctr];
			max_Index = i_ctr;
		}
	}

	for (i_ctr = STARTING_PS_INDEX2; i_ctr < MAX_NUM_PS_DATA2; i_ctr++) {
		if (ps_val[i_ctr] < min_val) {
			min_val = ps_val[i_ctr];
			min_Index = i_ctr;
		}
	}

	if (min_val == max_val)
		/* all values are the same*/
		temp = ps_val[STARTING_PS_INDEX2];
	else {
		for (i_ctr = STARTING_PS_INDEX2;
				i_ctr < MAX_NUM_PS_DATA2; i_ctr++) {
			if ((i_ctr != min_Index) && (i_ctr != max_Index))
				temp += ps_val[i_ctr];
		}
		temp = (temp / NUM_AVG_DATA2);
	}

	/*temp = (temp / NUM_AVG_DATA2);*/

	return temp;
}
#endif


/* take note ------------------------------------------
 This function should be called in the function which is called
 when the CALL button is pressed.
 take note ------------------------------------------*/
static void setThrDuringCall(void)
{
	int8_t ret;
	struct ltr553_data *ltr553 = sensor_info;

	/* set ps measurement rate to 10ms*/
//LINE<JIRA_ID><DATE20141031><add ps calibration>zenghaihui
	ret = ps_meas_rate_setup(50/*10*/, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS MeasRate Setup Fail...\n", __func__);
	}

	ps_grabData_stage = 0;
	ps_kept_data_counter = 0;
	ps_movavg_data_counter = 0;

//LINE<JIRA_ID><DATE20141031><add ps calibration>zenghaihui
#if 0
	ret = set_ps_range(PS_MIN_MEASURE_VAL, PS_MIN_MEASURE_VAL,
						LO_N_HI_LIMIT, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s : PS thresholds setting Fail...\n", __func__);
	}

	ret = ps_contr_setup(0x03, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS Enable Fail...\n", __func__);
	}
#endif    
}

#ifdef LOW_TEMPERATURE



#endif




/* Report PS input event */
static void report_ps_input_event(struct ltr553_data *ltr553)
{
	int8_t ret;
	uint16_t adc_value;

	uint8_t buffer[2];
	int vl_brightness = -1;

	adc_value = read_ps_adc_value(ltr553);

//LINE<JIRA_ID><DATE20141031><add ps calibration>zenghaihui
#if 0
	if (ps_grabData_stage == 0) {
		if (ps_kept_data_counter < PS_MAX_INIT_KEPT_DATA_COUNTER) {
			if (adc_value != 0) {
				ps_init_kept_data[ps_kept_data_counter] =
					adc_value;
				ps_kept_data_counter++;
			}
		}

		if (ps_kept_data_counter >= PS_MAX_INIT_KEPT_DATA_COUNTER) {
			ps_ct_avg = findCT_Avg(ps_init_kept_data);
			ftn_init = ps_ct_avg * 15;
			ps_grabData_stage = 1;
		}
	}

	if (ps_grabData_stage == 1) {
		if ((ftn_init - (ps_ct_avg * 10)) < 1000)
			ftn_final = (ps_ct_avg * 10) + 1000;
		else {
			if ((ftn_init - (ps_ct_avg * 10)) > 1500)
				ftn_final = (ps_ct_avg * 10) + 1500;
			else
				ftn_final = ftn_init;
		}
		ntf_final = (ftn_final - (ps_ct_avg * 10));
		ntf_final *= 4;
		ntf_final /= 100;
		ntf_final += ps_ct_avg;
		ftn_final /= 10;
		if (ntf_final >= PS_MAX_MEASURE_VAL)
			ntf_final = PS_MAX_MEASURE_VAL;

		if (ftn_final >= PS_MAX_MEASURE_VAL)
			ftn_final = PS_MAX_MEASURE_VAL;

		ret = ps_meas_rate_setup(50, ltr553);
		if (ret < 0) {
			dev_err(&ltr553->i2c_client->dev,
				"%s: PS MeasRate Setup Fail...\n", __func__);
		}

		ps_grabData_stage = 2;
            
	}
#endif



        vl_brightness = ltr553_read_lcd_brightness();

        pr_info("%s: zenghh vl_brightness = %d \n", __func__, vl_brightness);

//LINE<JIRA_ID><DATE20141031><add ps calibration>zenghaihui
			if (adc_value > ltr553->default_ps_highthresh) {

                    //LINE<FFBAKK-247><DATE20141026><ps modify>zenghaihui
                            buffer[0] = 0x90;
                            buffer[1] = 0xff;
                            ret = I2C_Write(buffer, 2);
                            if(ret < 0)
                            {
                		dev_err(&ltr553->i2c_client->dev,
                			"%s write fail...\n", __func__);
                            }
                            
                            buffer[0] = 0x91;
                            buffer[1] = 0x07;
                            ret = I2C_Write(buffer, 2);
                            if(ret < 0)
                            {
                		dev_err(&ltr553->i2c_client->dev,
                			"%s write fail...\n", __func__);
                            }
                            
                            buffer[0] = 0x92;
                            buffer[1] = ltr553->default_ps_lowthresh & 0xff;
                            ret = I2C_Write(buffer, 2);
                            if(ret < 0)
                            {
                		dev_err(&ltr553->i2c_client->dev,
                			"%s write fail...\n", __func__);
                            }
                            
                            buffer[0] = 0x93;
                            buffer[1] = (ltr553->default_ps_lowthresh>>8) & 0xff;
                            ret = I2C_Write(buffer, 2);
                            if(ret < 0)
                            {
                		dev_err(&ltr553->i2c_client->dev,
                			"%s write fail...\n", __func__);
                            }
							
                        if(vl_brightness > 0)
                        {
                        pr_info("zenghh vl_brightness >0  report far before report near\n"); 
                        
    				input_report_abs(ltr553->ps_input_dev,
    					ABS_DISTANCE, FAR_VAL);
    				input_sync(ltr553->ps_input_dev);

                            msleep(50);
                        }
                           
                        pr_info("zenghh report_ps_input_event report near \n"); 
                            
				input_report_abs(ltr553->ps_input_dev,
					ABS_DISTANCE, NEAR_VAL);
				input_sync(ltr553->ps_input_dev);
			}

//LINE<JIRA_ID><DATE20141031><add ps calibration>zenghaihui
			if (adc_value < ltr553->default_ps_lowthresh) {

                    //LINE<FFBAKK-247><DATE20141026><ps modify>zenghaihui
                            buffer[0] = 0x90;
                            buffer[1] = ltr553->default_ps_highthresh&0xff;
                            ret = I2C_Write(buffer, 2);
                            if(ret < 0)
                            {
                		dev_err(&ltr553->i2c_client->dev,
                			"%s write fail...\n", __func__);
                            }
                            
                            buffer[0] = 0x91;
                            buffer[1] = (ltr553->default_ps_highthresh>>8)&0xff;
                            ret = I2C_Write(buffer, 2);
                            if(ret < 0)
                            {
                		dev_err(&ltr553->i2c_client->dev,
                			"%s write fail...\n", __func__);
                            }
                            
                            buffer[0] = 0x92;
                            buffer[1] = 0x00;
                            ret = I2C_Write(buffer, 2);
                            if(ret < 0)
                            {
                		dev_err(&ltr553->i2c_client->dev,
                			"%s write fail...\n", __func__);
                            }
                            
                            buffer[0] = 0x93;
                            buffer[1] = 0x00;
                            ret = I2C_Write(buffer, 2);
                            if(ret < 0)
                            {
                		dev_err(&ltr553->i2c_client->dev,
                			"%s write fail...\n", __func__);
                            }

			#ifdef LOW_TEMPERATURE
				#ifdef Debug_For_LOW_TEMPERATURE	
					pr_info("[ltr553-MDP]: report_ps_input_event: ltr553  default_ps_highthresh=%x, default_ps_lowthresh=%x! \n", ltr553->default_ps_highthresh, ltr553->default_ps_lowthresh);
				#endif		

				if(reset_flag_low_temperature==0)
				{
					
				 	buffer[0] = 0x92;
	                            buffer[1] = ltr553->default_ps_lowthresh & 0xff;
	                            ret = I2C_Write(buffer, 2);
	                            if(ret < 0)
	                            {
	                		dev_err(&ltr553->i2c_client->dev,
	                			"%s write fail...\n", __func__);
	                            }
	                            
	                            buffer[0] = 0x93;
	                            buffer[1] = (ltr553->default_ps_lowthresh>>8) & 0xff;
	                            ret = I2C_Write(buffer, 2);
	                            if(ret < 0)
	                            {
	                		dev_err(&ltr553->i2c_client->dev,
	                			"%s write fail...\n", __func__);
	                            }
				}
			#endif



                        if(vl_brightness == 0)
                        {
                        pr_info("zenghh vl_brightness ==0  report near before report far\n"); 
                        
				input_report_abs(ltr553->ps_input_dev,
					ABS_DISTANCE, NEAR_VAL);
				input_sync(ltr553->ps_input_dev);

                            msleep(50);
                        }
                           
                        pr_info("zenghh report_ps_input_event report far \n"); 

				input_report_abs(ltr553->ps_input_dev,
					ABS_DISTANCE, FAR_VAL);
				input_sync(ltr553->ps_input_dev);
			}


//LINE<JIRA_ID><DATE20140930><ps modify>zenghaihui
#if 0
		if (ps_movavg_data_counter < PS_MAX_MOV_AVG_KEPT_DATA_CTR) {
			if (adc_value != 0) {
				ps_movavg_data[ps_movavg_data_counter] =
					adc_value;
				ps_movavg_data_counter++;
			}
		}

		if (ps_movavg_data_counter >= PS_MAX_MOV_AVG_KEPT_DATA_CTR) {
			ps_movct_avg =
				discardMinMax_findCTMov_Avg(ps_movavg_data);

			if (ps_movct_avg < ps_ct_avg) {
				ps_ct_avg = ps_movct_avg;
				ftn_init = ps_ct_avg * 17;
				ps_grabData_stage = 1;
			}
			ps_movavg_data_counter = 5;
		}
#endif        


}

/* Report ALS input event */
static void report_als_input_event(struct ltr553_data *ltr553)
{
	/*int8_t ret;*/
	uint16_t adc_value;
	/*int thresh_hi, thresh_lo, thresh_delta;*/
	/*ltr553->mode = ALS;*/
	/*adc_value = read_adc_value (ltr553);*/
	adc_value = read_als_adc_value(ltr553);
//Ivan added	
	previous_ls_data = adc_value;
	input_report_abs(ltr553->als_input_dev, ABS_MISC, adc_value);
	input_sync(ltr553->als_input_dev);

}

/* Work when interrupt */
static void ltr553_schedwork(struct work_struct *work)
{
	int8_t ret;
	uint8_t status;
	uint8_t	interrupt_stat, newdata;
	struct ltr553_data *ltr553 = sensor_info;
	uint8_t buffer[2];

	buffer[0] = LTR553_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return;
	}
	status = buffer[0];
	interrupt_stat = status & 0x0A;
	newdata = status & 0x05;

	/* PS interrupt and PS with new data*/
	if ((interrupt_stat & 0x02) && (newdata & 0x01)) {

		ltr553->ps_irq_flag = 1;
		report_ps_input_event(ltr553);
		ltr553->ps_irq_flag = 0;
	}
	/* ALS interrupt and ALS with new data*/
	if ((interrupt_stat & 0x08) && (newdata & 0x04)) {

		ltr553->als_irq_flag = 1;
		report_als_input_event(ltr553);
		ltr553->als_irq_flag = 0;
	}
	enable_irq(ltr553->irq);
}

static DECLARE_WORK(irq_workqueue, ltr553_schedwork);


/* IRQ Handler */
static irqreturn_t ltr553_irq_handler(int irq, void *data)
{
	struct ltr553_data *ltr553 = data;

	/* disable an irq without waiting */
	disable_irq_nosync(ltr553->irq);

	schedule_work(&irq_workqueue);

	return IRQ_HANDLED;
}

static int ltr553_gpio_irq(struct ltr553_data *ltr553)
{
	int rc = 0;

	rc = gpio_request(ltr553->gpio_int_no, DEVICE_NAME);
	if (rc < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: GPIO %d Request Fail (%d)\n",
				__func__, ltr553->gpio_int_no, rc);
		return rc;
	}

	rc = gpio_direction_input(ltr553->gpio_int_no);
	if (rc < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: Set GPIO %d as Input Fail (%d)\n", __func__,
					ltr553->gpio_int_no, rc);
		goto out1;
	}

	/* Configure an active low trigger interrupt for the device */
	/*rc = request_irq(ltr553->irq, ltr553_irq_handler,
				IRQF_TRIGGER_FALLING, DEVICE_NAME, ltr553);*/
	rc = request_irq(ltr553->irq, ltr553_irq_handler, IRQF_TRIGGER_LOW,
				DEVICE_NAME, ltr553);
	if (rc < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: Request IRQ (%d) for GPIO %d Fail (%d)\n",
				__func__, ltr553->irq,
					ltr553->gpio_int_no, rc);
		goto out1;
	}

	return rc;

out1:
	gpio_free(ltr553->gpio_int_no);

	return rc;
}

/* PS Enable */
static int8_t ps_enable_init(struct ltr553_data *ltr553)
{
	int8_t rc = 0;
	//uint8_t buffer[1]; /* for dummy read*/


	setThrDuringCall();

	if (ltr553->ps_enable_flag) {
		dev_info(&ltr553->i2c_client->dev,
			"%s: already enabled\n", __func__);
		return 0;
	}

	/* Set thresholds where interrupt will *not* be generated */
#if ACT_INTERRUPT
	/*rc = set_ps_range(PS_MIN_MEASURE_VAL, PS_MIN_MEASURE_VAL,
				LO_N_HI_LIMIT);*/
	/*rc = set_ps_range(PS_MIN_MEASURE_VAL, 400, LO_N_HI_LIMIT);*/
	rc = set_ps_range(ltr553->default_ps_highthresh,
		ltr553->default_ps_lowthresh, LO_N_HI_LIMIT, ltr553);
#else
	rc = set_ps_range(PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL,
			LO_N_HI_LIMIT, ltr553);
#endif
	if (rc < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s : PS Thresholds Write Fail...\n", __func__);
		return rc;
	}

	rc = ps_led_setup(0x7F, ltr553);
	if (rc < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS LED Setup Fail...\n", __func__);
		return rc;
	}

//LINE<JIRA_ID><DATE20141031><add ps calibration>zenghaihui
	rc = ps_ledPulseCount_setup(0x0C/*0x08*/, ltr553);
	if (rc < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS LED pulse count setup Fail...\n", __func__);
	}

//LINE<JIRA_ID><DATE20141031><add ps calibration>zenghaihui
	rc = ps_meas_rate_setup(50/*10*/, ltr553);
	if (rc < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS MeasRate Setup Fail...\n", __func__);
		return rc;
	}
//LINE<FFBAKK-247><DATE20141026><ps modify>zenghaihui
#if 0
	rc = ps_contr_setup(0x00, ltr553);
	if (rc < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS Enable Fail...\n", __func__);
		return rc;
	}

	/* dummy read*/
	buffer[0] = LTR553_PS_CONTR;
	I2C_Read(buffer, 1);
	/* dummy read*/
#endif

	/* ltr553->ps_enable_flag = 0; */

	return rc;
}


/* PS Disable */
//LINE<JIRA_ID><DATE20141031><add ps calibration>zenghaihui
#if 0
static int8_t ps_disable(struct ltr553_data *ltr553)
{
	int8_t rc = 0;

	if (ltr553->ps_enable_flag == 0) {
		dev_info(&ltr553->i2c_client->dev,
			"%s: already disabled\n", __func__);
		return 0;
	}

	/*rc = _ltr553_set_bit(ltr553->i2c_client,
					CLR_BIT, LTR553_PS_CONTR, PS_MODE);*/
	rc = ps_mode_setup(CLR_BIT, ltr553);
	if (rc < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS Disable Fail...\n", __func__);
		return rc;
	}

	ltr553->ps_enable_flag = 0;

	return rc;
}
#endif

/* PS open fops */
static ssize_t ps_open(struct inode *inode, struct file *file)
{
	struct ltr553_data *ltr553 = sensor_info;

	if (ltr553->ps_opened)
		return -EBUSY;

	ltr553->ps_opened = 1;

	return 0;
}


/* PS release fops */
static ssize_t ps_release(struct inode *inode, struct file *file)
{
	struct ltr553_data *ltr553 = sensor_info;

	ltr553->ps_opened = 0;

	return 0; // ps_disable(ltr553);
}

//LINE<JIRA_ID><DATE20141031><add ps calibration>zenghaihui
static void ltr553_ps_cali_start(void)
{
    u16 			vl_read_ps = 0;
    u16 			vl_ps_count = 0;
    u16 			vl_ps_sun = 0;
    u16 			vl_index = 0;
	
    struct ltr553_data *ltr553 = sensor_info;
    
    

    pr_info("entry ltr553_ps_cali_start \n");
    
    if(NULL == ltr553->i2c_client)
    {
        pr_info("ltr501_obj->client == NULL\n"); 
        return;
    }


    if (ltr553->ps_enable_flag == 0)
    {
        ps_mode_setup(1, ltr553);
    }
    
    msleep(10);

    // read ps
    for(vl_index = 0; vl_index < 5; vl_index++)
    {
	vl_read_ps = read_ps_adc_value(ltr553);

        pr_info("vl_index=%d, vl_read_ps = %d \n",vl_index, vl_read_ps);

        //if(vl_index >=2)
        {
            vl_ps_sun += vl_read_ps;
            
            vl_ps_count ++;
        }
        
        vl_read_ps = 0;
        
        msleep(30);
    }

    g_ps_base_value = (vl_ps_sun/vl_ps_count);
    g_ps_cali_flag = 1;
    
    pr_info("ltr553_ps_cali_start:g_ps_base_value=%x \n",g_ps_base_value);
    
    
}


static void ltr553_ps_cali_set_threshold(void)
{
	//u8 data, data2, data3;
	u16 value_high,value_low;
	struct ltr553_data *ltr553 = sensor_info;

    //LINE<JIRA_ID><DATE20141127><add ps cali data in dts>zenghaihui
    value_high= g_ps_base_value + g_ps_default_threshold_high_offset;
    value_low= g_ps_base_value + g_ps_default_threshold_low_offset;

    if( value_high > PS_MAX_MEASURE_VAL)
    {
        value_high= PS_MAX_MEASURE_VAL -100;
        value_low= PS_MAX_MEASURE_VAL -90;
        pr_info("ltr553_ps_cali_set_threshold: set value_high=0x7f0,value_low=0x7e0, please check the phone \n");
    }

    ltr553->default_ps_highthresh = value_high;
    ltr553->default_ps_lowthresh = value_low;

    set_ps_range(ltr553->default_ps_highthresh,
    	ltr553->default_ps_lowthresh, LO_N_HI_LIMIT, ltr553);

	pr_info("ltr553_ps_cali_set_threshold:value_high=%x,value_low=%x! \n",value_high,value_low);
}





/* PS IOCTL */
static long ps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0, val = 0;
        //LINE<JIRA_ID><DATE20131219><add PS Calibration>zenghaihui        
	void __user *ptr = (void __user*) arg;
        int ps_cali_data[2] = {0x00};
	struct ltr553_data *ltr553 = sensor_info;
//LINE<JIRA_ID><DATE20141031><add ps calibration>zenghaihui
	static int factory_status = 0;

	pr_info("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case LTR553_IOCTL_PS_ENABLE:
			if (get_user(val, (unsigned long __user *)arg)) {
				rc = -EFAULT;
				break;
			}

//LINE<JIRA_ID><DATE20141031><add ps calibration>zenghaihui
			//rc = val ? ps_enable_init(ltr553) : ps_disable(ltr553);
			ps_mode_setup((uint8_t)val, ltr553);

			break;
	case LTR553_IOCTL_PS_GET_ENABLED:
			rc = put_user(ltr553->ps_enable_flag,
					(unsigned long __user *)arg);

			break;

    
            //LINE<JIRA_ID><DATE20131218><add PS Calibration>zenghaihui
            case ALSPS_IOCTL_PS_CALI_START:
                pr_info("case ALSPS_IOCTL_PS_CALI_START: \n");
                
                ltr553_ps_cali_start();
                
                if (ptr == NULL) {
                    pr_info("%s ptr == NULL", __FUNCTION__);
                    rc = -EINVAL;
                    break;
                }
                
                ps_cali_data[0] = g_ps_cali_flag;
                ps_cali_data[1] = g_ps_base_value;
                
                pr_info("g_ps_cali_flag = %x, g_ps_base_value = %x \n", g_ps_cali_flag, g_ps_base_value);

                ltr553_ps_cali_set_threshold();
                ltr553_write_ps_cali_data(g_ps_cali_flag, g_ps_base_value);

                if (copy_to_user(ptr, ps_cali_data, sizeof(ps_cali_data))) {
                    pr_info("%s copy_from_user error", __FUNCTION__);
                    rc = -EFAULT;
                    break;
                }
                break;

            case ALSPS_IOCTL_PS_SET_CALI:
                pr_info("case ALSPS_IOCTL_PS_SET_CALI: \n");
                
                if (ptr == NULL) {
                    pr_info("%s ptr == NULL", __FUNCTION__);
                    rc = -EINVAL;
                    break;
                }
                
                if (copy_from_user(&ps_cali_data, ptr, sizeof(ps_cali_data))) {
                    pr_info("%s copy_from_user error", __FUNCTION__);
                    rc = -EFAULT;
                    break;
                }

                g_ps_cali_flag = ps_cali_data[0];
                g_ps_base_value = ps_cali_data[1];

                if(!g_ps_cali_flag)
                {
                    g_ps_base_value = g_ps_default_base_value; // set default base value
                    pr_info("not calibration!!! set g_ps_base_value = %x \n", g_ps_default_base_value);
                }
                
                pr_info("g_ps_cali_flag = %x, g_ps_base_value = %x \n", g_ps_cali_flag, g_ps_base_value);

                ltr553_ps_cali_set_threshold();
                
                break;

            case ALSPS_IOCTL_PS_GET_CALI:
                pr_info("case ALSPS_IOCTL_PS_GET_CALI: \n");
                
                if (ptr == NULL) {
                    pr_info("%s ptr == NULL", __FUNCTION__);
                    rc = -EINVAL;
                    break;
                }
                
                ps_cali_data[0] = g_ps_cali_flag;
                ps_cali_data[1] = g_ps_base_value;
                
                pr_info("g_ps_cali_flag = %x, g_ps_base_value = %x \n", g_ps_cali_flag, g_ps_base_value);
                
                if (copy_to_user(ptr, ps_cali_data, sizeof(ps_cali_data))) {
                    pr_info("%s copy_to_user error", __FUNCTION__);
                    rc = -EFAULT;
                    break;
                }
                break;

            case ALSPS_IOCTL_PS_CLR_CALI:
                pr_info("case ALSPS_IOCTL_PS_CLR_CALI: \n");
                g_ps_cali_flag = 0;
                g_ps_base_value = 0;
                ltr553_ps_cali_set_threshold();
                break;
                
            case ALSPS_IOCTL_PS_CALI_RAW_DATA:    
                val = read_ps_adc_value(ltr553);
                            
                if(copy_to_user(ptr, &val, sizeof(val)))
                {
                    rc = -EFAULT;
                    //goto err_out;
                }  
                break;        
                
            case ALSPS_IOCTL_PS_DATA:  

                val = read_ps_adc_value(ltr553);

                if(val > ltr553->default_ps_highthresh)
                {
                    //dat = 0;  /*close*/
                    factory_status = 0;
                }
                else if(val < ltr553->default_ps_lowthresh)
                {
                    //dat = 1;  /*far*/
                    factory_status = 1;
                }

                 val = factory_status;
                    
                if(copy_to_user(ptr, &val, sizeof(val)))
                {
                    rc = -EFAULT;
                    //goto err_out;
                }  
                break;        

	default:
			pr_err("%s: INVALID COMMAND %d\n",
				__func__, _IOC_NR(cmd));
			rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations ps_fops = {
	.owner = THIS_MODULE,
	.open = ps_open,
	.release = ps_release,
	.unlocked_ioctl = ps_ioctl
};

static struct miscdevice ps_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ltr553_ps",
	.fops = &ps_fops
};


static int8_t als_enable_init(struct ltr553_data *ltr553)
{
	int8_t rc = 0;
	
 	uint8_t buffer[1]; /* for dummy read*/

	/* if device not enabled, enable it */
	/*if (ltr553->als_enable_flag) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: ALS already enabled...\n", __func__);
		return rc;
	}*/

	rc = als_meas_rate_reg_setup(0x03, ltr553);
	if (rc < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS_Meas_Rate register Setup Fail...\n", __func__);
		return rc;
	}

	/* Set minimummax thresholds where interrupt will *not* be generated */
#if ACT_INTERRUPT
	/*rc = set_als_range(ALS_MIN_MEASURE_VAL,
				ALS_MIN_MEASURE_VAL, LO_N_HI_LIMIT);*/
	rc = set_als_range(ALS_MAX_MEASURE_VAL,
			ALS_MIN_MEASURE_VAL, LO_N_HI_LIMIT);
#else
	rc = set_als_range(ALS_MIN_MEASURE_VAL,
			ALS_MAX_MEASURE_VAL, LO_N_HI_LIMIT);
#endif
	if (rc < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s : ALS Thresholds Write Fail...\n", __func__);
		return rc;
	}

	rc = als_contr_setup(0x18/*0x0C*/, ltr553);
	if (rc < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS Enable Fail...\n", __func__);
		return rc;
	}

	/* dummy read*/
	buffer[0] = LTR553_ALS_CONTR;
	I2C_Read(buffer, 1);
	/* dumy read*/

	/* ltr553->als_enable_flag = 0; */

	return rc;
}


static int8_t als_disable(struct ltr553_data *ltr553)
{
	int8_t rc = 0;

	if (ltr553->als_enable_flag == 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s : ALS already disabled...\n", __func__);
		return rc;
	}

	/*rc = _ltr553_set_bit(ltr553->i2c_client,
				CLR_BIT, LTR553_ALS_CONTR, ALS_MODE);*/
	rc = als_mode_setup(CLR_BIT, ltr553);
	if (rc < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS Disable Fail...\n", __func__);
		return rc;
	}
	ltr553->als_enable_flag = 0;

	return rc;
}


static ssize_t als_open(struct inode *inode, struct file *file)
{
	struct ltr553_data *ltr553 = sensor_info;
	int8_t rc = 0;

	if (ltr553->als_opened) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS already Opened...\n", __func__);
		rc = -EBUSY;
	}
	ltr553->als_opened = 1;

	return rc;
}


static ssize_t als_release(struct inode *inode, struct file *file)
{
	struct ltr553_data *ltr553 = sensor_info;

	ltr553->als_opened = 0;

	return als_disable(ltr553);
}

static long als_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0, val = 0;
	struct ltr553_data *ltr553 = sensor_info;
        //LINE<JIRA_ID><DATE20141211><show lux in ffbm>zenghaihui                
	void __user *ptr = (void __user*) arg;

	pr_debug("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case LTR553_IOCTL_ALS_ENABLE:
			if (get_user(val, (unsigned long __user *)arg)) {
				rc = -EFAULT;
				break;
			}
			/*pr_info("%s value = %d\n", __func__, val);*/
		rc = val ? als_enable_init(ltr553) : als_disable(ltr553);

				break;
	case LTR553_IOCTL_ALS_GET_ENABLED:
			val = ltr553->als_enable_flag;
			/*pr_info("%s enabled %d\n", __func__, val);*/
			rc = put_user(val, (unsigned long __user *)arg);

				break;

        //LINE<JIRA_ID><DATE20141211><show lux in ffbm>zenghaihui                
	case ALSPS_IOCTL_ALS_RAW_DATA:
                val = read_als_adc_raw_value(ltr553);
                if(copy_to_user(ptr, &val, sizeof(val)))
                {
                    rc = -EFAULT;
                    //goto err_out;
                }  
                break;        
                
	default:
			pr_err("%s: INVALID COMMAND %d\n",
				__func__, _IOC_NR(cmd));
			rc = -EINVAL;
	}

	return rc;
}


static const struct file_operations als_fops = {
	.owner = THIS_MODULE,
	.open = als_open,
	.release = als_release,
	.unlocked_ioctl = als_ioctl
};

static struct miscdevice als_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ltr553_ls",
	.fops = &als_fops
};


static ssize_t als_adc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint16_t value;
	int ret;
	struct ltr553_data *ltr553 = sensor_info;

	/*ltr553->mode = ALS;*/
	/*value = read_adc_value(ltr553);*/
	value = read_als_adc_value(ltr553);
	input_report_abs(ltr553->als_input_dev, ABS_MISC, value);
	input_sync(ltr553->als_input_dev);
	ret = snprintf(buf, sizeof(buf), "%d", value);

	return ret;
}

static DEVICE_ATTR(als_adc, 0444, als_adc_show, NULL);


static ssize_t ps_adc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint16_t value;
	int ret;
	struct ltr553_data *ltr553 = sensor_info;

	/*ltr553->mode = PS;*/
	/*value = read_adc_value(ltr553);*/
	value = read_ps_adc_value(ltr553);
	ret = snprintf(buf, sizeof(buf), "%d", value);

	return ret;
}

static DEVICE_ATTR(ps_adc, 0444, ps_adc_show, NULL);


static ssize_t psadcsaturationBit_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint16_t value;
	uint8_t saturation_bit;
	int ret;
	uint8_t buffer[3];
	struct ltr553_data *ltr553 = sensor_info;

	buffer[0] = LTR553_PS_DATA_0;
	ret = I2C_Read(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	value = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
	/*ltr553->mode = PS_W_SATURATION_BIT;*/
	/*value = read_adc_value(ltr553);*/
	saturation_bit = (value >> 15);
	value &= PS_VALID_MEASURE_MASK;
	ret = snprintf(buf, sizeof(buf), "%d %d\n", value, saturation_bit);

	return ret;
}

static DEVICE_ATTR(psadcsaturationBit, 0444, psadcsaturationBit_show, NULL);


static ssize_t ltr553help_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	pr_info("To show ALS value : cat als_adc\n");
	pr_info("To show PS value : cat ps_adc\n");
	pr_info("To show PS value with saturation bit : cat psadcsaturationBit\n\n");

	/* address 0x80*/
	pr_info("Address 0x80 (ALS_CONTR)\n");
	pr_info("ALS active mode : echo 1 > enable\n");
	pr_info("ALS standby mode : echo 0 > enable\n");
	pr_info("To read ALS mode : cat enable\n\n");

	pr_info("ALS SW reset : echo 1 > alsswresetsetup\n");
	pr_info("ALS SW not reset : echo 0 > alsswresetsetup\n");
	pr_info("To read ALS SW reset bit : cat alsswresetsetup\n\n");

	pr_info("ALS gain 1x : echo 1 > alsgainsetup\n");
	pr_info("ALS gain 2x : echo 2 > alsgainsetup\n");
	pr_info("ALS gain 4x : echo 4 > alsgainsetup\n");
	pr_info("ALS gain 8x : echo 8 > alsgainsetup\n");
	pr_info("ALS gain 48x : echo 48 > alsgainsetup\n");
	pr_info("ALS gain 96x : echo 96 > alsgainsetup\n");
	pr_info("To read ALS gain : cat alsgainsetup\n\n");

	pr_info("Write ALS_CONTR register: echo [hexcode value] > alscontrsetup\n");
	pr_info("Example...to write 0x13 : echo 13 > alscontrsetup\n");
	pr_info("To read register ALS_CONTR (0x80) : cat alscontrsetup\n\n");
	/* address 0x80*/

	/* address 0x81*/
	pr_info("Address 0x81 (PS_CONTR)\n");
	pr_info("PS active mode : echo 1 > enable\n");
	pr_info("PS standby mode : echo 0 > enable\n");
	pr_info("To read PS mode : cat enable\n\n");

	pr_info("PS gain x16 : echo 16 > psgainsetup\n");
	pr_info("PS gain x32 : echo 32 > psgainsetup\n");
	pr_info("PS gain x64 : echo 64 > psgainsetup\n");
	pr_info("To read PS gain : cat psgainsetup\n\n");

	pr_info("PS saturation indicator enable : echo 1 > pssatuindicasetup\n");
	pr_info("PS saturation indicator disable : echo 0 > pssatuindicasetup\n");
	pr_info("To read back PS saturation indicator : cat pssatuindicasetup\n\n");

	pr_info("Example...to write 0x13 : echo 13 > pscontrsetup\n");
	pr_info("To read register PS_CONTR (0x81) : cat pscontrsetup\n\n");
	/* address 0x81*/

	/* address 0x82*/
	pr_info("Address 0x82 (PS_LED)\n");
	pr_info("LED current 5mA : echo 5 > psledcurrsetup\n");
	pr_info("LED current 10mA : echo 10 > psledcurrsetup\n");
	pr_info("LED current 20mA : echo 20 > psledcurrsetup\n");
	pr_info("LED current 50mA : echo 50 > psledcurrsetup\n");
	pr_info("LED current 100mA : echo 100 > psledcurrsetup\n");
	pr_info("To read LED current : cat psledcurrsetup\n\n");

	pr_info("LED current duty 25%% : echo 25 > psledcurrduty\n");
	pr_info("LED current duty 50%% : echo 50 > psledcurrduty\n");
	pr_info("LED current duty 75%% : echo 75 > psledcurrduty\n");
	pr_info("LED current duty 100%% : echo 100 > psledcurrduty\n");
	pr_info("To read LED current duty : cat psledcurrduty\n\n");

	pr_info("LED pulse freq 30kHz : echo 30 > psledpulsefreqsetup\n");
	pr_info("LED pulse freq 40kHz : echo 40 > psledpulsefreqsetup\n");
	pr_info("LED pulse freq 50kHz : echo 50 > psledpulsefreqsetup\n");
	pr_info("LED pulse freq 60kHz : echo 60 > psledpulsefreqsetup\n");
	pr_info("LED pulse freq 70kHz : echo 70 > psledpulsefreqsetup\n");
	pr_info("LED pulse freq 80kHz : echo 80 > psledpulsefreqsetup\n");
	pr_info("LED pulse freq 90kHz : echo 90 > psledpulsefreqsetup\n");
	pr_info("LED pulse freq 100kHz : echo 100 > psledpulsefreqsetup\n");
	pr_info("To read LED pulse freq : cat psledpulsefreqsetup\n\n");

	pr_info("Example...to write 0x13 : echo 13 > psledsetup\n");
	pr_info("To read register PS_LED (0x82) : cat psledsetup\n\n");
	/* address 0x82*/

	/* address 0x83*/
	pr_info("Address 0x83 (PS_N_PULSES)\n");
	pr_info("[pulse count num] must be 0 to 15, inclusive\n");
	pr_info("Example...to set 0 count : echo 0 > psledpulsecountsetup\n");
	pr_info("Example...to set 13 counts : echo 13 > psledpulsecountsetup\n");
	/* address 0x83*/

	/* address 0x84*/
	pr_info("Address 0x84 (PS_MEAS_RATE)\n");
	pr_info("PS meas repeat rate 50ms : echo 50 > psmeasratesetup\n");
	pr_info("PS meas repeat rate 70ms : echo 70 > psmeasratesetup\n");
	pr_info("PS meas repeat rate 100ms : echo 100 > psmeasratesetup\n");
	pr_info("PS meas repeat rate 200ms : echo 200 > psmeasratesetup\n");
	pr_info("PS meas repeat rate 500ms : echo 500 > psmeasratesetup\n");
	pr_info("PS meas repeat rate 1000ms : echo 1000 > psmeasratesetup\n");
	pr_info("PS meas repeat rate 2000ms : echo 2000 > psmeasratesetup\n");
	pr_info("PS meas repeat rate 10ms : echo 10 > psmeasratesetup\n");
	pr_info("To read register PS_MEAS_RATE (0x84) : cat psmeasratesetup\n\n");
	/* address 0x84*/

	/* address 0x85*/
	pr_info("Address 0x85 (ALS_MEAS_RATE)\n");
	pr_info("ALS meas repeat rate 50ms : echo 50 > alsmeasratesetup\n");
	pr_info("ALS meas repeat rate 100ms : echo 100 > alsmeasratesetup\n");
	pr_info("ALS meas repeat rate 200ms : echo 200 > alsmeasratesetup\n");
	pr_info("ALS meas repeat rate 500ms : echo 500 > alsmeasratesetup\n");
	pr_info("ALS meas repeat rate 1000ms : echo 1000 > alsmeasratesetup\n");
	pr_info("ALS meas repeat rate 2000ms : echo 2000 > alsmeasratesetup\n");
	pr_info("To read ALS meas repeat rate : cat alsmeasratesetup\n\n");

	pr_info("ALS integration time 100ms : echo 100 > alsintegtimesetup\n");
	pr_info("ALS integration time 50ms : echo 50 > alsintegtimesetup\n");
	pr_info("ALS integration time 200ms : echo 200 > alsintegtimesetup\n");
	pr_info("ALS integration time 400ms : echo 400 > alsintegtimesetup\n");
	pr_info("ALS integration time 150ms : echo 150 > alsintegtimesetup\n");
	pr_info("ALS integration time 250ms : echo 250 > alsintegtimesetup\n");
	pr_info("ALS integration time 300ms : echo 300 > alsintegtimesetup\n");
	pr_info("ALS integration time 350ms : echo 350 > alsintegtimesetup\n");
	pr_info("To read ALS integration time : cat alsintegtimesetup\n\n");

	pr_info("Example...to write 0x13 : echo 13 > alsmeasrateregsetup\n");
	pr_info("To read register ALS_MEAS (0x85) : cat alsmeasrateregsetup\n\n");
	/* address 0x85*/

	/* address 0x86*/
	pr_info("To read part ID : cat partid\n");
	pr_info("To read revision ID : cat revid\n");
	pr_info("To read PART_ID register (0x86) : cat partidreg\n\n");
	/* address 0x86*/

	/* address 0x87*/
	pr_info("To read manufacturing ID : cat manuid\n\n");
	/* address 0x87*/

	/* address 0x8C*/
	pr_info("Address 0x8C (ALS_PS_STATUS)\n");
	pr_info("To read PS data status : cat psdatastatus\n");
	pr_info("To read PS interrupt status : cat psinterruptstatus\n");
	pr_info("To read ALS data status : cat alsdatastatus\n");
	pr_info("To read ALS interrupt status : cat alsinterruptstatus\n");
	pr_info("To read ALS gain status : cat alsgainstatus\n");
	pr_info("To read ALS validity status : cat alsdatavaliditystatus\n");
	pr_info("To read register ALS_PS_STATUS (0x8C) : cat alspsstatusreg\n\n");
	/* address 0x8C*/

	/* address 0x88, 0x89, 0x8A, 0x8B*/
	pr_info("ALS raw and calculated data, address 0x88, 0x89, 0x8A, 0x8B\n");
	pr_info("To read raw and calculated ALS data : cat alsch0ch1rawcalc\n\n");
	/* address 0x88, 0x89, 0x8A, 0x8B*/

	/* address 0x94, 0x95*/
	pr_info("Example...to write 55 : echo 55 > setpsoffset\n");
	pr_info("To read back the offset value : cat setpsoffset\n\n");
	/* address 0x94, 0x95*/

	/* address 0x8F*/
	pr_info("Address 0x8F (INTERRUPT)\n");
	pr_info("INT output pin inactive : echo 0 > interruptmodesetup\n");
	pr_info("Only PS triggers interrupt : echo 1 > interruptmodesetup\n");
	pr_info("Only ALS triggers interrupt : echo 2 > interruptmodesetup\n");
	pr_info("Both ALS PS trigger interrupt : echo 3 > interruptmodesetup\n");
	pr_info("To read interrupt mode : cat interruptmodesetup\n\n");

	pr_info("INT output pin active low : echo 0 > interruptpolarsetup\n");
	pr_info("INT output pin active high : echo 1 > interruptpolarsetup\n");
	pr_info("To read interrupt pin polarity : cat interruptpolarsetup\n\n");

	pr_info("Example...to write 0x13 : echo 13 > interruptsetup\n");
	pr_info("To read register INTERRUPT (0x8F) : cat interruptsetup\n\n");
	/* address 0x8F*/

	/* address 0x9E*/
	pr_info("Address 0x9E (INTERRUPT PERSIST)\n");
	pr_info("Example...to write 0x13 : echo 13 > interruptpersistsetup\n");
	/* address 0x9E*/

	/* ALS threshold setting*/
	pr_info("To read the threshold values : cat dispalsthrerange\n\n");
	/* ALS threshold setting*/

	/* PS threshold setting*/
	pr_info("PS threshold setting 0x90, 0x91, 0x92, 0x93\n");
	pr_info("To read the threshold values : cat disppsthrerange\n\n");
	/* PS threshold setting*/

	return 0;
}

static DEVICE_ATTR(ltr553help, 0444, ltr553help_show, NULL);


static ssize_t alsmodesetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = als_contr_readback(ALS_MODE_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS_MODE_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}

static ssize_t alsmodesetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;
	struct ltr553_data *ltr553 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltr553->i2c_client->dev,
				"%s: store value = %d\n", __func__, param);

	ret = als_mode_setup((uint8_t)param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: ALS mode setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(enable_als_sensor, 0644,
				alsmodesetup_show, alsmodesetup_store);


static ssize_t alsswresetsetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = als_contr_readback(ALS_SWRT_RDBCK, &rdback_val, ltr553);

	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS_SWRT_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}


static ssize_t alsswresetsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;

	struct ltr553_data *ltr553 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltr553->i2c_client->dev,
			"%s: store value = %d\n", __func__, param);

	ret = als_sw_reset_setup((uint8_t)param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS sw reset setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(alsswresetsetup, 0644,
				alsswresetsetup_show, alsswresetsetup_store);


static ssize_t alsgainsetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = als_contr_readback(ALS_GAIN_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS_GAIN_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}


static ssize_t alsgainsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[2];

	struct ltr553_data *ltr553 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
	} else if (count == 2) {
		param_temp[0] -= 48;
		param_temp[1] = 0;

		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
	}

	param = ((param_temp[0] * 10) + param_temp[1]);
	dev_dbg(&ltr553->i2c_client->dev,
				"%s: store value = %d\n", __func__, param);

	ret = als_gain_setup((uint8_t)param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: ALS gain setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static ssize_t alscontrsetup_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = als_contr_readback(ALS_CONTR_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS_CONTR_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(alsgainsetup, 0644, alsgainsetup_show, alsgainsetup_store);

static ssize_t alscontrsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[2];

	struct ltr553_data *ltr553 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}


	if (param_temp[0] >= 65 && param_temp[0] <= 70)
		param_temp[0] -= 55;
	else if (param_temp[0] >= 97 && param_temp[0] <= 102)
		param_temp[0] -= 87;
	else if (param_temp[0] >= 48 && param_temp[0] <= 57)
		param_temp[0] -= 48;
	else
		param_temp[0] = 0;

	if (param_temp[1] >= 65 && param_temp[1] <= 70)
		param_temp[1] -= 55;
	else if (param_temp[1] >= 97 && param_temp[1] <= 102)
		param_temp[1] -= 87;
	else if (param_temp[1] >= 48 && param_temp[1] <= 57)
		param_temp[1] -= 48;
	else
		param_temp[1] = 0;

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&ltr553->i2c_client->dev,
			"%s: store value = %d\n", __func__, param);

	ret = als_contr_setup(param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: ALS contr setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(alscontrsetup, 0644, alscontrsetup_show,
					alscontrsetup_store);

static ssize_t psmodesetup_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = ps_contr_readback(PS_MODE_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: PS_MODE_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}


static ssize_t psmodesetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;
	struct ltr553_data *ltr553 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltr553->i2c_client->dev,
		"%s: store value = %d\n", __func__, param);

	ret = ps_mode_setup((uint8_t)param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: PS mode setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;

}

static DEVICE_ATTR(enable_ps_sensor, 0644, psmodesetup_show, psmodesetup_store);


static ssize_t psgainsetup_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = ps_contr_readback(PS_GAIN_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: PS_GAIN_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}


static ssize_t psgainsetup_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	uint8_t param;
	int8_t ret;
	int param_temp[2];

	struct ltr553_data *ltr553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 2) {
		param_temp[0] = 0;
		param_temp[1] = 0;
	} else if (count >= 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
	}

	param = ((param_temp[0] * 10) + param_temp[1]);

	dev_dbg(&ltr553->i2c_client->dev,
	"%s: store value = %d\n", __func__, param);

	ret = ps_gain_setup(param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS gain setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(psgainsetup, 0644, psgainsetup_show, psgainsetup_store);


static ssize_t pssatuindicasetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = ps_contr_readback(PS_SATUR_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: PS_SATUR_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}


static ssize_t pssatuindicasetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;
	struct ltr553_data *ltr553 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltr553->i2c_client->dev,
		"%s: store value = %d\n", __func__, param);

	ret = ps_satu_indica_setup((uint8_t)param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: PS saturation indicator setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(pssatuindicasetup, 0644,
		pssatuindicasetup_show, pssatuindicasetup_store);


static ssize_t pscontrsetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = ps_contr_readback(PS_CONTR_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: PS_CONTR_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}


static ssize_t pscontrsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[2];

	struct ltr553_data *ltr553 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70)
		param_temp[0] -= 55;
	else if (param_temp[0] >= 97 && param_temp[0] <= 102)
		param_temp[0] -= 87;
	else if (param_temp[0] >= 48 && param_temp[0] <= 57)
		param_temp[0] -= 48;
	else
		param_temp[0] = 0;

	if (param_temp[1] >= 65 && param_temp[1] <= 70)
		param_temp[1] -= 55;
	else if (param_temp[1] >= 97 && param_temp[1] <= 102)
		param_temp[1] -= 87;
	else if (param_temp[1] >= 48 && param_temp[1] <= 57)
		param_temp[1] -= 48;
	else
		param_temp[1] = 0;

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&ltr553->i2c_client->dev,
		"%s: store value = %d\n", __func__, param);

	ret = ps_contr_setup(param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: PS contr setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(pscontrsetup, 0644, pscontrsetup_show, pscontrsetup_store);


static ssize_t psledcurrsetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = ps_led_readback(LED_CURR_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: LED_CURR_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;

}


static ssize_t psledcurrsetup_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[3];

	struct ltr553_data *ltr553 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
	} else if (count == 2) {
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;

		param_temp[2] = param_temp[0];
		param_temp[0] = 0;
		param_temp[1] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;

		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	}

	param = ((param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2]);
	dev_dbg(&ltr553->i2c_client->dev,
		"%s: store value = %d\n", __func__, param);

	ret = ps_ledCurrent_setup(param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS LED current setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;

}

static DEVICE_ATTR(psledcurrsetup, 0644,
			psledcurrsetup_show, psledcurrsetup_store);


static ssize_t psledcurrduty_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = ps_led_readback(LED_CURR_DUTY_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: LED_CURR_DUTY_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;

}


static ssize_t psledcurrduty_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[3];

	struct ltr553_data *ltr553 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];

	if (count < 3) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;

		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	}

	param = ((param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2]);
	dev_dbg(&ltr553->i2c_client->dev,
			"%s: store value = %d\n", __func__, param);

	ret = ps_ledCurrDuty_setup(param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS LED curent duty setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(psledcurrduty, 0644,
			psledcurrduty_show, psledcurrduty_store);


static ssize_t psledpulsefreqsetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = ps_led_readback(LED_PUL_FREQ_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: LED_PUL_FREQ_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;

}


static ssize_t psledpulsefreqsetup_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[3];

	struct ltr553_data *ltr553 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];

	if (count < 3) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;

		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	}

	param = ((param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2]);
	dev_dbg(&ltr553->i2c_client->dev,
		"%s: store value = %d\n", __func__, param);

	ret = ps_ledPulseFreq_setup(param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS LED pulse frequency setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(psledpulsefreqsetup, 0644,
		psledpulsefreqsetup_show, psledpulsefreqsetup_store);


static ssize_t psledsetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = ps_led_readback(PS_LED_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS_LED_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}


static ssize_t psledsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[2];

	struct ltr553_data *ltr553 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70)
		param_temp[0] -= 55;
	else if (param_temp[0] >= 97 && param_temp[0] <= 102)
		param_temp[0] -= 87;
	else if (param_temp[0] >= 48 && param_temp[0] <= 57)
		param_temp[0] -= 48;
	else
		param_temp[0] = 0;

	if (param_temp[1] >= 65 && param_temp[1] <= 70)
		param_temp[1] -= 55;
	else if (param_temp[1] >= 97 && param_temp[1] <= 102)
		param_temp[1] -= 87;
	else if (param_temp[1] >= 48 && param_temp[1] <= 57)
		param_temp[1] -= 48;
	else
		param_temp[1] = 0;

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&ltr553->i2c_client->dev,
		"%s: store value = %d\n", __func__, param);

	ret = ps_led_setup(param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: PS LED setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(psledsetup, 0644, psledsetup_show, psledsetup_store);


static ssize_t psledpulsecountsetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = ps_ledPulseCount_readback(&rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS LED pulse count readback Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}


static ssize_t psledpulsecountsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[2];

	struct ltr553_data *ltr553 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if ((count <= 1) || (count > 3)) {
		param_temp[0] = 0;
		param_temp[1] = 0;
	} else if (count == 2) {
		param_temp[0] -= 48;
		param_temp[1] = 0;

		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
	}

	param = ((param_temp[0] * 10) + param_temp[1]);
	if (param > 15)
		param = 15;

	dev_dbg(&ltr553->i2c_client->dev,
		"%s: store value = %d\n", __func__, param);

	ret = ps_ledPulseCount_setup(param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS LED pulse count setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(psledpulsecountsetup, 0644,
			psledpulsecountsetup_show, psledpulsecountsetup_store);


static ssize_t psmeasratesetup_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = ps_meas_rate_readback(&rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS meas rate readback Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;

}


static ssize_t psmeasratesetup_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t param;
	/*int *param_temp = buf;*/
	int param_temp[4];

	struct ltr553_data *ltr553 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 2) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 4) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
	}

	param = ((param_temp[0] * 1000) + (param_temp[1] * 100) +
				(param_temp[2] * 10) + param_temp[3]);
	dev_dbg(&ltr553->i2c_client->dev,
			"%s: store value = %d\n", __func__, param);

	ret = ps_meas_rate_setup(param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: PS measurement rate setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(psmeasratesetup, 0644,
		psmeasratesetup_show, psmeasratesetup_store);


static ssize_t alsmeasratesetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = als_meas_rate_readback(ALS_MEAS_RPT_RATE_RDBCK,
		&rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS_MEAS_RPT_RATE_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;

}


static ssize_t alsmeasratesetup_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t param;
	/*int *param_temp = buf;*/
	int param_temp[4];

	struct ltr553_data *ltr553 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 2) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 4) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
	}

	param = ((param_temp[0] * 1000) + (param_temp[1] * 100) +
				(param_temp[2] * 10) + param_temp[3]);
	dev_dbg(&ltr553->i2c_client->dev,
			"%s: store value = %d\n", __func__, param);

	ret = als_meas_rate_setup(param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS measurement rate setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(alsmeasratesetup, 0644,
				alsmeasratesetup_show, alsmeasratesetup_store);


static ssize_t alsintegtimesetup_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = als_meas_rate_readback(ALS_INTEG_TM_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: ALS_INTEG_TM_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;

}


static ssize_t alsintegtimesetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t param;
	/*int *param_temp = buf;*/

	int param_temp[3];

	struct ltr553_data *ltr553 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];

	if (count <= 2) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;

		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	}

	param = ((param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2]);
	dev_dbg(&ltr553->i2c_client->dev,
			"%s: store value = %d\n", __func__, param);

	ret = als_integ_time_setup(param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS integration time setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(alsintegtimesetup, 0644,
			alsintegtimesetup_show, alsintegtimesetup_store);


static ssize_t alsmeasrateregsetup_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = als_meas_rate_readback(ALS_MEAS_RATE_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS_MEAS_RATE_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;

}


static ssize_t alsmeasrateregsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[2];

	struct ltr553_data *ltr553 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70)
		param_temp[0] -= 55;
	else if (param_temp[0] >= 97 && param_temp[0] <= 102)
		param_temp[0] -= 87;
	else if (param_temp[0] >= 48 && param_temp[0] <= 57)
		param_temp[0] -= 48;
	else
		param_temp[0] = 0;

	if (param_temp[1] >= 65 && param_temp[1] <= 70)
		param_temp[1] -= 55;
	else if (param_temp[1] >= 97 && param_temp[1] <= 102)
		param_temp[1] -= 87;
	else if (param_temp[1] >= 48 && param_temp[1] <= 57)
		param_temp[1] -= 48;
	else
		param_temp[1] = 0;

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&ltr553->i2c_client->dev,
		"%s: store value = %d\n", __func__, param);

	ret = als_meas_rate_reg_setup(param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: ALS meas rate register setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(alsmeasrateregsetup, 0644,
			alsmeasrateregsetup_show, alsmeasrateregsetup_store);


static ssize_t partid_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = part_ID_reg_readback(PART_NUM_ID_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: PART_NUM_ID_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(partid, 0444, partid_show, NULL);


static ssize_t revid_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = part_ID_reg_readback(REVISION_ID_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: REVISION_ID_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(revid, 0444, revid_show, NULL);


static ssize_t partidreg_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = part_ID_reg_readback(PART_ID_REG_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: PART_ID_REG_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(partidreg, 0444, partidreg_show, NULL);


static ssize_t manuid_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = manu_ID_reg_readback(&rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: Manufacturing ID readback Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(manuid, 0444, manuid_show, NULL);


static ssize_t psdatastatus_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = als_ps_status_reg(PS_DATA_STATUS_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS_DATA_STATUS_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(psdatastatus, 0444, psdatastatus_show, NULL);


static ssize_t psinterruptstatus_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = als_ps_status_reg(PS_INTERR_STATUS_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS_INTERR_STATUS_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(psinterruptstatus, 0444, psinterruptstatus_show, NULL);


static ssize_t alsdatastatus_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = als_ps_status_reg(ALS_DATA_STATUS_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS_DATA_STATUS_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(alsdatastatus, 0444, alsdatastatus_show, NULL);


static ssize_t alsinterruptstatus_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = als_ps_status_reg(ALS_INTERR_STATUS_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS_INTERR_STATUS_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(alsinterruptstatus, 0444, alsinterruptstatus_show, NULL);


static ssize_t alsgainstatus_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = als_ps_status_reg(ALS_GAIN_STATUS_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS_GAIN_STATUS_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(alsgainstatus, 0444, alsgainstatus_show, NULL);


static ssize_t alsdatavaliditystatus_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = als_ps_status_reg(ALS_VALID_STATUS_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS_VALID_STATUS_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(alsdatavaliditystatus, 0444,
				alsdatavaliditystatus_show, NULL);


static ssize_t alspsstatusreg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = als_ps_status_reg(ALS_PS_STATUS_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS_PS_STATUS_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;

}

static DEVICE_ATTR(alspsstatusreg, 0444, alspsstatusreg_show, NULL);

static ssize_t alsch0ch1rawcalc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint16_t rdback_val1 = 0, rdback_val2 = 0, rdback_val3 = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = als_ch0ch1raw_calc_readback(&rdback_val1,
				&rdback_val2, &rdback_val3, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: ALS CH0 CH1 Calc reading readback Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d %d %d\n",
			rdback_val1, rdback_val2, rdback_val3);

	return ret;

}

static DEVICE_ATTR(alsch0ch1rawcalc, 0444, alsch0ch1rawcalc_show, NULL);


static ssize_t setpsoffset_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint16_t rdback_val;
	struct ltr553_data *ltr553 = sensor_info;

	ret = ps_offset_readback(&rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: PS offset readback Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}


static ssize_t setpsoffset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t ps_offset = 0;
	uint8_t param_temp[4];
	struct ltr553_data *ltr553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 2) { /* 1 digit*/
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { /* 2 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { /* 3 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 5) { /* 4 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
	}

	ps_offset = ((param_temp[0] * 1000) + (param_temp[1] * 100) +
					(param_temp[2] * 10) + param_temp[3]);
	if (ps_offset > 1023)
		ps_offset = 1023;

	dev_dbg(&ltr553->i2c_client->dev,
				"%s: store value = %d\n", __func__, ps_offset);

	ret = ps_offset_setup(ps_offset, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: set ps offset Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(setpsoffset, 0644, setpsoffset_show, setpsoffset_store);


static ssize_t interruptmodesetup_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = interrupt_readback(INT_MODE_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: INT_MODE_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}


static ssize_t interruptmodesetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;

	struct ltr553_data *ltr553 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltr553->i2c_client->dev,
				"%s: store value = %d\n", __func__, param);

	ret = interrupt_mode_setup((uint8_t)param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: interrupt mode setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(interruptmodesetup, 0644,
		interruptmodesetup_show, interruptmodesetup_store);


static ssize_t interruptpolarsetup_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = interrupt_readback(INT_POLAR_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: INT_POLAR_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}


static ssize_t interruptpolarsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;

	struct ltr553_data *ltr553 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltr553->i2c_client->dev,
			"%s: store value = %d\n", __func__, param);

	ret = interrupt_polarity_setup((uint8_t)param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: interrupt polarity setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(interruptpolarsetup, 0644,
			interruptpolarsetup_show, interruptpolarsetup_store);


static ssize_t interruptsetup_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = interrupt_readback(INT_INTERRUPT_RDBCK, &rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: INT_INTERRUPT_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}


static ssize_t interruptsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[2];

	struct ltr553_data *ltr553 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70)
		param_temp[0] -= 55;
	else if (param_temp[0] >= 97 && param_temp[0] <= 102)
		param_temp[0] -= 87;
	else if (param_temp[0] >= 48 && param_temp[0] <= 57)
		param_temp[0] -= 48;
	else
		param_temp[0] = 0;

	if (param_temp[1] >= 65 && param_temp[1] <= 70)
		param_temp[1] -= 55;
	else if (param_temp[1] >= 97 && param_temp[1] <= 102)
		param_temp[1] -= 87;
	else if (param_temp[1] >= 48 && param_temp[1] <= 57)
		param_temp[1] -= 48;
	else
		param_temp[1] = 0;

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&ltr553->i2c_client->dev,
				"%s: store value = %d\n", __func__, param);

	ret = interrupt_setup(param, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: interrupt setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(interruptsetup, 0644,
			interruptsetup_show, interruptsetup_store);


static ssize_t interruptpersistsetup_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr553_data *ltr553 = sensor_info;

	ret = interrupt_prst_readback(&rdback_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: Interrupt persist readback Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);

	return ret;
}


static ssize_t interruptpersistsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret = 0;
	/*uint8_t als_or_ps, prst_val;*/
	uint8_t prst_val;
	/*int *param_temp = buf;*/
	int param_temp[2];

	struct ltr553_data *ltr553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70)
		param_temp[0] -= 55;
	else if (param_temp[0] >= 97 && param_temp[0] <= 102)
		param_temp[0] -= 87;
	else if (param_temp[0] >= 48 && param_temp[0] <= 57)
		param_temp[0] -= 48;
	else
		param_temp[0] = 0;

	if (param_temp[1] >= 65 && param_temp[1] <= 70)
		param_temp[1] -= 55;
	else if (param_temp[1] >= 97 && param_temp[1] <= 102)
		param_temp[1] -= 87;
	else if (param_temp[1] >= 48 && param_temp[1] <= 57)
		param_temp[1] -= 48;
	else
		param_temp[1] = 0;

	prst_val = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&ltr553->i2c_client->dev,
			"%s: store value = %d\n", __func__, prst_val);

	/*ret = interrupt_persist_setup(als_or_ps, prst_val, ltr553);*/
	ret = interrupt_persist_setup(prst_val, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: Interrupt persist setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;

}

static DEVICE_ATTR(interruptpersistsetup, 0644,
	interruptpersistsetup_show, interruptpersistsetup_store);


static ssize_t setalslothrerange_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	int lo_thr = 0;
	uint8_t param_temp[5];
	struct ltr553_data *ltr553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];
	param_temp[4] = buf[4];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;
	} else if (count == 2) { /* 1 digit*/
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[0];
		param_temp[3] = 0;
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { /*2 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[1];
		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { /* 3 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[2];
		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 5) { /* 4 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
		param_temp[4] = 0;

		param_temp[4] = param_temp[3];
		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 6) { /* 5 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
		param_temp[4] -= 48;
	}

	lo_thr = ((param_temp[0] * 10000) + (param_temp[1] * 1000) +
	(param_temp[2] * 100) + (param_temp[3] * 10) + param_temp[4]);
	if (lo_thr > 65535)
		lo_thr = 65535;
	dev_dbg(&ltr553->i2c_client->dev,
			"%s: store value = %d\n", __func__, lo_thr);

	ret = set_als_range((uint16_t)lo_thr, 0, LO_LIMIT);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: set ALS lo threshold Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(setalslothrerange, 0644, NULL, setalslothrerange_store);


static ssize_t setalshithrerange_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	int hi_thr = 0;
	uint8_t param_temp[5];
	struct ltr553_data *ltr553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];
	param_temp[4] = buf[4];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;
	} else if (count == 2) { /* 1 digit*/
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[0];
		param_temp[3] = 0;
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { /* 2 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[1];
		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { /* 3 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[2];
		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 5) { /* 4 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
		param_temp[4] = 0;

		param_temp[4] = param_temp[3];
		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 6) { /* 5 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
		param_temp[4] -= 48;
	}

	hi_thr = ((param_temp[0] * 10000) + (param_temp[1] * 1000) +
	(param_temp[2] * 100) + (param_temp[3] * 10) + param_temp[4]);
	if (hi_thr > 65535)
		hi_thr = 65535;
	dev_dbg(&ltr553->i2c_client->dev,
			"%s: store value = %d\n", __func__, hi_thr);

	ret = set_als_range(0, (uint16_t)hi_thr, HI_LIMIT);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: set ALS hi threshold Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(setalshithrerange, 0644, NULL, setalshithrerange_store);

static ssize_t dispalsthrerange_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint16_t rdback_lo, rdback_hi;
	struct ltr553_data *ltr553 = sensor_info;

	ret = als_range_readback(&rdback_lo, &rdback_hi, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS threshold range readback Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d %d\n", rdback_lo, rdback_hi);

	return ret;
}

static DEVICE_ATTR(dispalsthrerange, 0444, dispalsthrerange_show, NULL);


static ssize_t setpslothrerange_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t lo_thr = 0;
	uint8_t param_temp[4];
	struct ltr553_data *ltr553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 2) { /* 1 digit*/
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { /* 2 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { /* 3 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 5) { /* 4 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
	}

	lo_thr = ((param_temp[0] * 1000) + (param_temp[1] * 100) +
				(param_temp[2] * 10) + param_temp[3]);
	if (lo_thr > 2047)
		lo_thr = 2047;
	dev_dbg(&ltr553->i2c_client->dev,
			"%s: store value = %d\n", __func__, lo_thr);

	ret = set_ps_range(lo_thr, 0, LO_LIMIT, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: set PS lo threshold Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(setpslothrerange, 0644, NULL, setpslothrerange_store);


static ssize_t setpshithrerange_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t hi_thr = 0;
	uint8_t param_temp[4];
	struct ltr553_data *ltr553 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 2) { /* 1 digit*/
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { /* 2 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { /* 3 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 5) { /* 4 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
	}

	hi_thr = ((param_temp[0] * 1000) + (param_temp[1] * 100) +
				(param_temp[2] * 10) + param_temp[3]);
	if (hi_thr > 2047)
			hi_thr = 2047;
	dev_dbg(&ltr553->i2c_client->dev,
			"%s: store value = %d\n", __func__, hi_thr);

	ret = set_ps_range(0, hi_thr, HI_LIMIT, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
				"%s: set PS hi threshold Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static DEVICE_ATTR(setpshithrerange, 0644, NULL, setpshithrerange_store);


static ssize_t disppsthrerange_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint16_t rdback_lo, rdback_hi;
	struct ltr553_data *ltr553 = sensor_info;

	ret = ps_range_readback(&rdback_lo, &rdback_hi, ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: PS threshold range readback Fail...\n", __func__);
		return -EPERM;
	}

	ret = snprintf(buf, sizeof(buf), "%d %d\n", rdback_lo, rdback_hi);

	return ret;
}

static DEVICE_ATTR(disppsthrerange, 0444, disppsthrerange_show, NULL);


static void sysfs_register_device(struct i2c_client *client)
{
	int rc = 0;

	rc += device_create_file(&client->dev, &dev_attr_als_adc);
	rc += device_create_file(&client->dev, &dev_attr_ps_adc);
	/*rc += device_create_file(&client->dev, &dev_attr_setwinfac1);
	rc += device_create_file(&client->dev, &dev_attr_setwinfac2);
	rc += device_create_file(&client->dev, &dev_attr_setwinfac3);*/
	rc += device_create_file(&client->dev, &dev_attr_psadcsaturationBit);
	rc += device_create_file(&client->dev, &dev_attr_ltr553help);
	rc += device_create_file(&client->dev, &dev_attr_enable_als_sensor);
	rc += device_create_file(&client->dev, &dev_attr_alsswresetsetup);
	rc += device_create_file(&client->dev, &dev_attr_alsgainsetup);
	rc += device_create_file(&client->dev, &dev_attr_alscontrsetup);
	rc += device_create_file(&client->dev, &dev_attr_enable_ps_sensor);
	rc += device_create_file(&client->dev, &dev_attr_psgainsetup);
	rc += device_create_file(&client->dev, &dev_attr_pssatuindicasetup);
	rc += device_create_file(&client->dev, &dev_attr_pscontrsetup);
	rc += device_create_file(&client->dev, &dev_attr_psledcurrsetup);
	rc += device_create_file(&client->dev, &dev_attr_psledcurrduty);
	rc += device_create_file(&client->dev, &dev_attr_psledpulsefreqsetup);
	rc += device_create_file(&client->dev, &dev_attr_psledsetup);
	rc += device_create_file(&client->dev, &dev_attr_psledpulsecountsetup);
	rc += device_create_file(&client->dev, &dev_attr_psmeasratesetup);
	rc += device_create_file(&client->dev, &dev_attr_alsmeasratesetup);
	rc += device_create_file(&client->dev, &dev_attr_alsintegtimesetup);
	rc += device_create_file(&client->dev, &dev_attr_alsmeasrateregsetup);
	rc += device_create_file(&client->dev, &dev_attr_partid);
	rc += device_create_file(&client->dev, &dev_attr_revid);
	rc += device_create_file(&client->dev, &dev_attr_partidreg);
	rc += device_create_file(&client->dev, &dev_attr_manuid);
	rc += device_create_file(&client->dev, &dev_attr_psdatastatus);
	rc += device_create_file(&client->dev, &dev_attr_psinterruptstatus);
	rc += device_create_file(&client->dev, &dev_attr_alsdatastatus);
	rc += device_create_file(&client->dev, &dev_attr_alsinterruptstatus);
	rc += device_create_file(&client->dev, &dev_attr_alsgainstatus);
	rc += device_create_file(&client->dev, &dev_attr_alsdatavaliditystatus);
	rc += device_create_file(&client->dev, &dev_attr_alspsstatusreg);
	rc += device_create_file(&client->dev, &dev_attr_alsch0ch1rawcalc);
	rc += device_create_file(&client->dev, &dev_attr_setpsoffset);
	rc += device_create_file(&client->dev, &dev_attr_interruptmodesetup);
	rc += device_create_file(&client->dev, &dev_attr_interruptpolarsetup);
	rc += device_create_file(&client->dev, &dev_attr_interruptsetup);
	rc += device_create_file(&client->dev, &dev_attr_interruptpersistsetup);
	rc += device_create_file(&client->dev, &dev_attr_setalslothrerange);
	rc += device_create_file(&client->dev, &dev_attr_setalshithrerange);
	rc += device_create_file(&client->dev, &dev_attr_dispalsthrerange);
	rc += device_create_file(&client->dev, &dev_attr_setpslothrerange);
	rc += device_create_file(&client->dev, &dev_attr_setpshithrerange);
	rc += device_create_file(&client->dev, &dev_attr_disppsthrerange);

	if (rc)
		dev_err(&client->dev,
			"%s Unable to create sysfs files\n", __func__);
	else
		dev_dbg(&client->dev,
			"%s Created sysfs files\n", __func__);
}


static int als_setup(struct ltr553_data *ltr553)
{
	int ret;

	ltr553->als_input_dev = input_allocate_device();
	if (!ltr553->als_input_dev) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: ALS Input Allocate Device Fail...\n", __func__);
		return -ENOMEM;
	}
	ltr553->als_input_dev->name = "light";
	set_bit(EV_ABS, ltr553->als_input_dev->evbit);
	input_set_abs_params(ltr553->als_input_dev, ABS_MISC,
		ALS_MIN_MEASURE_VAL, ALS_MAX_MEASURE_VAL, 0, 0);

	ret = input_register_device(ltr553->als_input_dev);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS Register Input Device Fail...\n", __func__);
		goto err_als_register_input_device;
	}

	ret = misc_register(&als_misc);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: ALS Register Misc Device Fail...\n", __func__);
		goto err_als_register_misc_device;
	}

	return ret;

err_als_register_misc_device:
	input_unregister_device(ltr553->als_input_dev);
err_als_register_input_device:
	input_free_device(ltr553->als_input_dev);

	return ret;
}


static int ps_setup(struct ltr553_data *ltr553)
{
	int ret;

	ltr553->ps_input_dev = input_allocate_device();
	if (!ltr553->ps_input_dev) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: PS Input Allocate Device Fail...\n", __func__);
		return -ENOMEM;
	}
	ltr553->ps_input_dev->name = "proximity";
	set_bit(EV_ABS, ltr553->ps_input_dev->evbit);
	input_set_abs_params(ltr553->ps_input_dev,
		ABS_DISTANCE, PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, 0, 0);

	ret = input_register_device(ltr553->ps_input_dev);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: PS Register Input Device Fail...\n", __func__);
		goto err_ps_register_input_device;
	}

	ret = misc_register(&ps_misc);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: PS Register Misc Device Fail...\n", __func__);
		goto err_ps_register_misc_device;
	}

	return ret;

err_ps_register_misc_device:
	input_unregister_device(ltr553->ps_input_dev);
err_ps_register_input_device:
	input_free_device(ltr553->ps_input_dev);

	return ret;
}


static int _check_part_id(struct ltr553_data *ltr553)
{
	uint8_t ret;
	uint8_t buffer[2];

	buffer[0] = LTR553_PART_ID;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev, "%s: Read failure :0x%02X",
		__func__, buffer[0]);
		return -EPERM;
	}

	if (buffer[0] != PARTID) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: Part failure miscompare act:0x%02x exp:0x%02x\n",
		__func__, buffer[0], PARTID);
		return -ENOENT;
	}

	return 0;
}


static int ltr553_setup(struct ltr553_data *ltr553)
{
	int ret = 0;

	/* Reset the devices */
	ret = _ltr553_set_bit(ltr553->i2c_client, SET_BIT,
						LTR553_ALS_CONTR, ALS_SW_RESET);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: ALS reset fail...\n", __func__);
		goto err_out1;
	}

	ret = _ltr553_set_bit(ltr553->i2c_client, CLR_BIT,
					LTR553_PS_CONTR, PS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS reset fail...\n", __func__);
		goto err_out1;
	}

	msleep(PON_DELAY);
	dev_dbg(&ltr553->i2c_client->dev,
		"%s: Reset ltr553 device\n", __func__);

	/* Do another part read to ensure we have exited reset */
	if (_check_part_id(ltr553) < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: Part ID Read Fail after reset...\n", __func__);
		goto err_out1;
	}

	ret = ltr553_gpio_irq(ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: GPIO Request Fail...\n", __func__);
		goto err_out1;
	}
	dev_dbg(&ltr553->i2c_client->dev,
		"%s Requested interrupt\n", __func__);

/* Set count of measurements outside data range before interrupt is generated */
	ret = _ltr553_set_bit(ltr553->i2c_client, SET_BIT,
						LTR553_INTERRUPT_PRST, 0x01);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS Set Persist Fail...\n", __func__);
		goto err_out2;
	}

	ret = _ltr553_set_bit(ltr553->i2c_client,
					SET_BIT, LTR553_INTERRUPT_PRST, 0x10);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: PS Set Persist Fail...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&ltr553->i2c_client->dev,
		"%s: Set ltr553 persists\n", __func__);

	/* Enable interrupts on the device and clear only when status is read */
#if ACT_INTERRUPT
	ret = _ltr553_set_bit(ltr553->i2c_client,
			SET_BIT, LTR553_INTERRUPT, INT_MODE_ALSPS_TRIG);
	/*ret = _ltr553_set_bit(ltr553->i2c_client, SET_BIT,
				LTR553_INTERRUPT, INT_MODE_PS_TRIG);*/
#else
	ret = _ltr553_set_bit(ltr553->i2c_client, SET_BIT,
						LTR553_INTERRUPT, INT_MODE_00);
#endif
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: Enabled interrupts failed...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&ltr553->i2c_client->dev,
			"%s Enabled interrupt to device\n", __func__);

	/* Turn on ALS and PS */
	ret = als_enable_init(ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s Unable to enable ALS", __func__);
		goto err_out2;
	}

	ret = ps_enable_init(ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s Unable to enable PS", __func__);
		goto err_out2;
	}

	return ret;

err_out2:
	free_irq(ltr553->irq, ltr553);
	gpio_free(ltr553->gpio_int_no);

err_out1:
	dev_err(&ltr553->i2c_client->dev,
		"%s Unable to setup device\n", __func__);

	return ret;
}

/******* device poweron init ***********/
static int ltr553_setup_poweron(struct ltr553_data *ltr553)
{
	int ret = 0;

	/* Reset the devices */
	ret = _ltr553_set_bit(ltr553->i2c_client, SET_BIT,
						LTR553_ALS_CONTR, ALS_SW_RESET);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: ALS reset fail...\n", __func__);
		goto err_out1;
	}

	ret = _ltr553_set_bit(ltr553->i2c_client, CLR_BIT,
					LTR553_PS_CONTR, PS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS reset fail...\n", __func__);
		goto err_out1;
	}

	msleep(20);
	dev_dbg(&ltr553->i2c_client->dev,
		"%s: Reset ltr553 device\n", __func__);

	/* Do another part read to ensure we have exited reset */
	if (_check_part_id(ltr553) < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: Part ID Read Fail after reset...\n", __func__);
		goto err_out1;
	}
/* Set count of measurements outside data range before interrupt is generated */
	ret = _ltr553_set_bit(ltr553->i2c_client, SET_BIT,
						LTR553_INTERRUPT_PRST, 0x01);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS Set Persist Fail...\n", __func__);
		goto err_out2;
	}

	ret = _ltr553_set_bit(ltr553->i2c_client,
					SET_BIT, LTR553_INTERRUPT_PRST, 0x10);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: PS Set Persist Fail...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&ltr553->i2c_client->dev,
		"%s: Set ltr553 persists\n", __func__);

	/* Enable interrupts on the device and clear only when status is read */
#if ACT_INTERRUPT
	ret = _ltr553_set_bit(ltr553->i2c_client,
			SET_BIT, LTR553_INTERRUPT, INT_MODE_ALSPS_TRIG);
	/*ret = _ltr553_set_bit(ltr553->i2c_client, SET_BIT,
				LTR553_INTERRUPT, INT_MODE_PS_TRIG);*/
#else
	ret = _ltr553_set_bit(ltr553->i2c_client, SET_BIT,
						LTR553_INTERRUPT, INT_MODE_00);
#endif
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: Enabled interrupts failed...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&ltr553->i2c_client->dev,
			"%s Enabled interrupt to device\n", __func__);

	/* Turn on ALS and PS */
	ret = als_enable_init(ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s Unable to enable ALS", __func__);
		goto err_out2;
	}

	ret = ps_enable_init(ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s Unable to enable PS", __func__);
		goto err_out2;
	}

	return ret;

err_out2:
	free_irq(ltr553->irq, ltr553);
	gpio_free(ltr553->gpio_int_no);

err_out1:
	dev_err(&ltr553->i2c_client->dev,
		"%s Unable to setup device\n", __func__);

	return ret;
}

static int ltr553_als_set_enable(struct sensors_classdev *sensors_cdev,
			unsigned int enable)
{
	int ret = 0;
	struct ltr553_data *data = container_of(sensors_cdev,
			struct ltr553_data, als_cdev);
	struct ltr553_platform_data *pdata = data->platform_data;

        //LINE<JIRA_ID><DATE20150106><als modify>zenghaihui
 	uint8_t buffer[1]; /* for dummy read*/
    
	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

        pr_info("ltr553.c %s: enable = %d \n", __func__, enable);

	/* reinit device after resume */
	if (enable == 1) {
		if ((data->als_enable_flag == 0)
			&& (data->ps_enable_flag == 0)) {
			/* Power on and initalize the device */
			if (pdata->power_on)
				pdata->power_on(true);

		ret = ltr553_setup_poweron(data);
			if (ret < 0) {
				dev_err(&data->i2c_client->dev,
				"%s: Setup Fail...\n", __func__);
				return ret;
			}
		}

		if (data->als_enable_flag == 0) {
			data->als_enable_flag = 1;

			ret = als_mode_setup((uint8_t)enable, data);
		}
//Ivan report initial data to HAL, otherwise need to wait for next int

         //LINE<JIRA_ID><DATE20141203><detect ffbm in ltr553>zenghaihui
        if(0xffff == g_ffbm_flag)
        {
            g_ffbm_flag = 0;
            ltr553_read_ffbm_flag();
        }
        
        if(1 == g_ffbm_flag)
        {
		msleep(50);
		//input_report_abs(data->als_input_dev, ABS_MISC, previous_ls_data);
		//input_sync(data->als_input_dev);
		report_als_input_event(data);
		msleep(50);
  		report_als_input_event(data);
      }

            //LINE<JIRA_ID><DATE20150106><als modify>zenghaihui
            msleep(50);

            /* dummy read*/
            buffer[0] = LTR553_ALS_CONTR;
            I2C_Read(buffer, 1);
            /* dumy read*/
    
	} else {
		data->als_enable_flag = 0;
		ret = als_mode_setup((uint8_t)enable, data);
	}


	return 0;
}



//LINE<JIRA_ID><DATE20141031><add ps calibration>zenghaihui
static void ltr553_write_ps_cali_data(int vl_flag, int vl_ps_data)
{
        mm_segment_t fs;   
        int ps_data[2] = {0x00};
        char str_ps_data[16] = {0x00};

        struct file *ps_filp = NULL;

        pr_info("%s: vl_flag = %d,  vl_ps_data = %d \n", __func__, vl_flag, vl_ps_data);

        ps_data[0] = vl_flag;
        ps_data[1] = vl_ps_data;
        
	snprintf(str_ps_data, sizeof(str_ps_data), "%d-%d-\n", vl_flag, vl_ps_data);


        fs=get_fs();   

        set_fs(KERNEL_DS);  
            
        ps_filp = filp_open(PROXIMITY_CALI_DATA_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0666);

        if (IS_ERR(ps_filp))
        {
            pr_info("%s    failed to open  %s\n", __func__, PROXIMITY_CALI_DATA_PATH);
        }
        else
        {

            ps_filp->f_op->llseek(ps_filp, 0, SEEK_SET);
            ps_filp->f_op->write(ps_filp, (char*)str_ps_data, strlen(str_ps_data), &ps_filp->f_pos);
            
        }

        if (ps_filp && !IS_ERR(ps_filp))
        {
            filp_close(ps_filp, NULL);
        }
        
        set_fs(fs);  

}


static void ltr553_read_ps_cali_data(void)
{
	int ret = 0;
	int vl_error_flag = 0;
        mm_segment_t fs;   
        int ps_data[2] = {0x00};
        char str_ps_data[16] = {0x00};

        struct file *ps_filp = NULL;

        pr_info("%s \n", __func__);
        
        fs=get_fs();   

        set_fs(KERNEL_DS);  
            
        ps_filp = filp_open(PROXIMITY_CALI_DATA_PATH, O_RDONLY, 0);

        if (IS_ERR(ps_filp))
        {
            pr_info("%s    failed to open  %s\n", __func__, PROXIMITY_CALI_DATA_PATH);

            vl_error_flag = 1;
        }
        else
        {

            ps_filp->f_op->llseek(ps_filp, 0, SEEK_SET);
            
            ret = ps_filp->f_op->read(ps_filp, (char*)str_ps_data, sizeof(str_ps_data), &ps_filp->f_pos);

            
            if (ret < 0)
            {
                pr_info("failed to read ps data from file");

                vl_error_flag = 1;
            }
            else
            {
                sscanf(str_ps_data, "%d-%d-", &ps_data[0], &ps_data[1]);
            }
        }
        
        if(vl_error_flag)
        {
            g_ps_cali_flag = 0;
        }
        else
        {
            pr_info("%s: ps_data[0] = %x, ps_data[1] = %x \n", __func__, ps_data[0], ps_data[1]);
            
            g_ps_cali_flag = ps_data[0];
            g_ps_base_value= ps_data[1];
        }
        
        if(!g_ps_cali_flag)
        {
            g_ps_base_value = g_ps_default_base_value; // set default base value
            
            pr_info("not calibration!!! set g_ps_base_value = %x \n", g_ps_default_base_value);
        }
        
        pr_info("%s: g_ps_cali_flag = %x, g_ps_base_value = %x \n", __func__, g_ps_cali_flag, g_ps_base_value);

        if (ps_filp && !IS_ERR(ps_filp))
        {
            filp_close(ps_filp, NULL);
        }
        
        set_fs(fs);  
    
}


#define DEFUALT_SYS_PATH_BL  "/sys/class/leds/lcd-backlight/brightness"
static int ltr553_read_lcd_brightness(void)
{
	int ret = 0;
	int vl_error_flag = 0;
        mm_segment_t fs;   
        int vl_brightness = -1;
        char str_bl_data[16] = {0x00};

        struct file *bl_filp = NULL;

        pr_info("%s \n", __func__);
        
        fs=get_fs();   

        set_fs(KERNEL_DS);  
            
        bl_filp = filp_open(DEFUALT_SYS_PATH_BL, O_RDONLY, 0);

        if (IS_ERR(bl_filp))
        {
            pr_info("%s    failed to open  %s\n", __func__, DEFUALT_SYS_PATH_BL);

            vl_error_flag = 1;
        }
        else
        {

            bl_filp->f_op->llseek(bl_filp, 0, SEEK_SET);
            
            ret = bl_filp->f_op->read(bl_filp, (char*)str_bl_data, sizeof(str_bl_data), &bl_filp->f_pos);

            
            if (ret < 0)
            {
                pr_info("failed to read brightness data from file");

                vl_error_flag = 1;
            }
            else
            {
                sscanf(str_bl_data, "%d", &vl_brightness);
            }
        }
        
        if(vl_error_flag)
        {
            vl_brightness = -1;
        }

        pr_info("%s: vl_brightness = %d \n", __func__, vl_brightness);

        if (bl_filp && !IS_ERR(bl_filp))
        {
            filp_close(bl_filp, NULL);
        }
        
        set_fs(fs);  

        return vl_brightness;
    
}



//LINE<JIRA_ID><DATE20141203><detect ffbm in ltr553>zenghaihui
static void ltr553_read_ffbm_flag(void)
{
	int ret = 0;
	int vl_error_flag = 0;
        mm_segment_t fs;   
        char cmdline[512] = {0x00};
        char *pffbm = NULL;

        struct file *ps_filp = NULL;

        pr_info("%s \n", __func__);
        
        fs=get_fs();   

        set_fs(KERNEL_DS);  
            
        ps_filp = filp_open("/proc/cmdline", O_RDONLY, 0);

        if (IS_ERR(ps_filp))
        {
            pr_info("%s    failed to open  %s\n", __func__, "/proc/cmdline");

            vl_error_flag = 1;
        }
        else
        {
            ps_filp->f_op->llseek(ps_filp, 0, SEEK_SET);
            
            ret = ps_filp->f_op->read(ps_filp, (char*)cmdline, 511, &ps_filp->f_pos);
            
            if (ret < 0)
            {
                pr_info("failed to read cmdline");

                vl_error_flag = 1;
            }
        }
        
        if(vl_error_flag)
        {
            // do nothing
        }
        else
        {
            cmdline[511] = 0x00;
            
            pr_info("%s: cmdline = %s \n", __func__, cmdline);
            
            pffbm = strstr(cmdline, "androidboot.mode=ffbm-01");

            if(pffbm)
            {
                g_ffbm_flag = 1; // boot ffbm mode
            }
            
        }
        
        pr_info("%s: g_ffbm_flag = %d \n", __func__, g_ffbm_flag);

        if (ps_filp && !IS_ERR(ps_filp))
        {
            filp_close(ps_filp, NULL);
        }
        
        set_fs(fs);  
    
}

#ifdef LOW_TEMPERATURE

static void low_temperature_value_all_reset(void)
{

	min_value_low_temperature=0x7ff;
	set_first_value_flag_low_temperature=0;
	reset_flag_low_temperature=0;
	 pr_info("low_temperature_value_all_reset: All values has been reset !");

}
#endif
static int ltr553_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	int ret = 0;
	struct ltr553_data *data = container_of(sensors_cdev,
			struct ltr553_data, ps_cdev);
	struct ltr553_platform_data *pdata = data->platform_data;

        //LINE<JIRA_ID><DATE20141021><ps modify for low temperature test>zenghaihui
	uint8_t buffer[1]; /* for dummy read*/

#ifdef Debug_For_LOW_TEMPERATURE	
	pr_alert("[ltr553-MDP] reset_low_temperature_threshold_value:  min_value_low_temperature=%d \n",min_value_low_temperature);
	pr_alert("[ltr553-MDP] reset_low_temperature_threshold_value:reset_flag_low_temperature= %d  set_first_value_flag_low_temperature=%d \n", reset_flag_low_temperature,set_first_value_flag_low_temperature);
#endif
		

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	/**** reinit sensor after resume power on ****/
	if (enable == 1) {
        
//LINE<JIRA_ID><DATE20141031><add ps calibration>zenghaihui
		if (data->ps_enable_flag == 0) {
            
		if ((data->als_enable_flag == 0) &&
			(data->ps_enable_flag == 0)) {
			/**** Power on and initalize the device ****/
			if (pdata->power_on)
				pdata->power_on(true);
		}

		ret = ltr553_setup_poweron(data);
			if (ret < 0) {
				dev_err(&data->i2c_client->dev,
				"%s: Setup Fail...\n", __func__);

				return ret;
		}

                    if(g_read_ps_cali_flag == 0)
                    {
                        ltr553_read_ps_cali_data();
                        g_read_ps_cali_flag = 1;
                    }
                    
                    ltr553_ps_cali_set_threshold();
            
			data->ps_enable_flag = 1;
			ret = ps_mode_setup((uint8_t)enable, data);

                    //LINE<JIRA_ID><DATE20141021><ps modify for low temperature test>zenghaihui
                    msleep(50);

                    /* dummy read*/
                    buffer[0] = LTR553_PS_CONTR;
                    I2C_Read(buffer, 1);
                    /* dummy read*/

		}

            //LINE<FFBAKK-247><DATE20141026><ps modify>zenghaihui
		if (!wake_lock_active(&(data->ps_wake_lock)))
			wake_lock(&(data->ps_wake_lock));
            
	} else {
	#ifdef LOW_TEMPERATURE
	low_temperature_value_all_reset();
	#endif
		data->ps_enable_flag = 0;
		ret = ps_mode_setup((uint8_t)enable, data);

            //LINE<FFBAKK-247><DATE20141026><ps modify>zenghaihui
		if (wake_lock_active(&(data->ps_wake_lock)))
			wake_unlock(&(data->ps_wake_lock));
                
	}

	/* when enable auto change backlight,
	* als_enable should be reinit when ps
	* disabled after use
	*/
	if ((enable == 0) && (data->als_enable_flag == 1))
		als_mode_setup(1, data);


	return 0;
}

/*****************regulator configuration start**************/
static int sensor_regulator_configure(struct ltr553_data *data, bool on)
{
	int rc;

	if (!on) {

		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				LTR553_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				LTR553_VIO_MAX_UV);

		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->i2c_client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->i2c_client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				LTR553_VDD_MIN_UV, LTR553_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->i2c_client->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->i2c_client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->i2c_client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				LTR553_VIO_MIN_UV, LTR553_VIO_MAX_UV);
			if (rc) {
				dev_err(&data->i2c_client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;
reg_vio_put:
	regulator_put(data->vio);

reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, LTR553_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

static int sensor_regulator_power_on(struct ltr553_data *data, bool on)
{
	int rc = 0;

	if (!on) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->i2c_client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->i2c_client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			rc = regulator_enable(data->vdd);
			dev_err(&data->i2c_client->dev,
					"Regulator vio re-enabled rc=%d\n", rc);
			/*
			 * Successfully re-enable regulator.
			 * Enter poweron delay and returns error.
			 */
			if (!rc) {
				rc = -EBUSY;
				goto enable_delay;
			}
		}
		return rc;
	} else {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->i2c_client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->i2c_client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			regulator_disable(data->vdd);
			return rc;
		}
	}

enable_delay:
	msleep(130);
	dev_dbg(&data->i2c_client->dev,
		"Sensor regulator power on =%d\n", on);
	return rc;
}

static int sensor_platform_hw_power_on(bool on)
{
	struct ltr553_data *data;
	int err = 0;

	/* sensor_info is global pointer to struct ltr553_data */
	if (sensor_info == NULL)
		return -ENODEV;

	data = sensor_info;

	if (data->power_on != on) {

		err = sensor_regulator_power_on(data, on);
		if (err)
			dev_err(&data->i2c_client->dev,
					"Can't configure regulator!\n");
		else
			data->power_on = on;
	}

	return err;
}

static int sensor_platform_hw_init(void)
{
	struct i2c_client *client;
	struct ltr553_data *data;
	int error;

	if (sensor_info == NULL)
		return -ENODEV;

	data = sensor_info;
	client = data->i2c_client;

	error = sensor_regulator_configure(data, true);
	if (error < 0) {
		dev_err(&client->dev, "unable to configure regulator\n");
		return error;
	}

	return 0;
}

static void sensor_platform_hw_exit(void)
{
	struct ltr553_data *data = sensor_info;

	if (data == NULL)
		return;

	sensor_regulator_configure(data, false);
}
/******************regulator ends***********************/

static int ltr_parse_dt(struct device *dev,
				struct ltr553_platform_data *ltr_pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	/* set functions of platform data */
	ltr_pdata->init = sensor_platform_hw_init;
	ltr_pdata->exit = sensor_platform_hw_exit;
	ltr_pdata->power_on = sensor_platform_hw_power_on;

	rc = of_get_named_gpio_flags(dev->of_node,
				"liteon,intr", 0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read intr\n");
		return rc;
	}
	ltr_pdata->pfd_gpio_int_no = rc;

	rc = of_property_read_u32(np, "liteon,highthr", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read high threshold\n");
		return rc;
	} else {
		ltr_pdata->pfd_ps_highthresh = temp_val;
	}

	rc = of_property_read_u32(np, "liteon,lowthr", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read low threshold\n");
		return rc;
	} else {
		ltr_pdata->pfd_ps_lowthresh = temp_val;
	}

        //LINE<JIRA_ID><DATE20141127><add ps cali data in dts>zenghaihui
	rc = of_property_read_u32(np, "tinno,base_value", &temp_val);
	if (rc && (rc != -EINVAL)) {
		pr_info("Unable to read tinno,base_value\n");
		g_ps_default_base_value = PROXIMITY_CALI_DEFAULT_DATA;
	} else {
		g_ps_default_base_value = temp_val;
		pr_info("read g_ps_default_base_value = %x \n", g_ps_default_base_value);
	}
    
	rc = of_property_read_u32(np, "tinno,threshold_high_offset", &temp_val);
	if (rc && (rc != -EINVAL)) {
		pr_info("Unable to read tinno,threshold_high_offset\n");
		g_ps_default_threshold_high_offset = PROXIMITY_CALI_DEFAULT_THRESHOLD_HIGH_OFFSET;
	} else {
		g_ps_default_threshold_high_offset = temp_val;
		pr_info("read g_ps_default_threshold_high_offset = %x \n", g_ps_default_threshold_high_offset);
	}
    
	rc = of_property_read_u32(np, "tinno,threshold_low_offset", &temp_val);
	if (rc && (rc != -EINVAL)) {
		pr_info("Unable to read tinno,threshold_low_offset\n");
		g_ps_default_threshold_low_offset = PROXIMITY_CALI_DEFAULT_THRESHOLD_LOW_OFFSET;
	} else {
		g_ps_default_threshold_low_offset = temp_val;
		pr_info("read g_ps_default_threshold_low_offset = %x \n", g_ps_default_threshold_low_offset);
	}

	return 0;
}
static int ltr553_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	int ret = 0;
	struct ltr553_data *ltr553;
	struct ltr553_platform_data *pdata;

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct ltr553_platform_data), GFP_KERNEL);

		if (!pdata) {
				dev_err(&client->dev,
				"failed to allocate memory for platform data\n");
				return -ENOMEM;
		}
		client->dev.platform_data = pdata;

		ret = ltr_parse_dt(&client->dev, pdata);
		if (ret) {
				dev_err(&client->dev,
		"Unable to parse platfrom data err=%d\n", ret);
				return ret;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			dev_err(&client->dev, "No platform data\n");
			return -ENODEV;
		}
	}


	ltr553 = kzalloc(sizeof(struct ltr553_data), GFP_KERNEL);
	if (!ltr553) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: Mem Alloc Fail...\n", __func__);

                //LINE<JIRA_ID><DATE20141027><ltr553 update>zenghaihui
                if (pdata && (client->dev.of_node))
                    devm_kfree(&client->dev, pdata);
    
		return -ENOMEM;
	}

	/* Set initial defaults */
	ltr553->als_enable_flag = 0;
	ltr553->ps_enable_flag = 0;

	ltr553->i2c_client = client;
	ltr553->irq = client->irq;
	ltr553->platform_data = pdata;

	/* Global pointer for this device */
	sensor_info = ltr553;
	/* h/w initialization */
	if (pdata->init)
		ret = pdata->init();
	i2c_set_clientdata(client, ltr553);

	/* Parse the platform data */
	ltr553->gpio_int_no = pdata->pfd_gpio_int_no;
	/* ltr553->adc_levels = platdata->pfd_levels; */
	ltr553->default_ps_lowthresh = pdata->pfd_ps_lowthresh;
	ltr553->default_ps_highthresh = pdata->pfd_ps_highthresh;

	if (_check_part_id(ltr553) < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: Part ID Read Fail...\n", __func__);
		goto err_out;
	}

	/* Setup the input subsystem for the ALS */
	ret = als_setup(ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: ALS Setup Fail...\n", __func__);
		goto err_out;
	}

	/* Setup the input subsystem for the PS */
	ret = ps_setup(ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
			"%s: PS Setup Fail...\n", __func__);
		goto err_als_ps_setup;
	}

	/* Create the workqueue for the interrup handler */
	ltr553->workqueue = create_singlethread_workqueue("ltr553_wq");
	if (!ltr553->workqueue) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: Create WorkQueue Fail...\n", __func__);
		ret = -ENOMEM;
		goto err_als_ps_setup;
	}

	/* Wake lock option for promity sensor */
	wake_lock_init(&(ltr553->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");

	/* Setup and configure both the ALS and PS on the ltr553 device */
	ret = ltr553_setup(ltr553);
	if (ret < 0) {
		dev_err(&ltr553->i2c_client->dev,
		"%s: Setup Fail...\n", __func__);
		goto err_ltr553_setup;
	}

	/* Register the sysfs files */
	sysfs_register_device(client);
	/*sysfs_register_als_device(client, &ltr553->als_input_dev->dev);*/
	/*sysfs_register_ps_device(client, &ltr553->ps_input_dev->dev);*/

	/* Register to sensors class */
	ltr553->als_cdev = sensors_light_cdev;
	ltr553->als_cdev.sensors_enable = ltr553_als_set_enable;
	ltr553->als_cdev.sensors_poll_delay = NULL;

	ltr553->ps_cdev = sensors_proximity_cdev;
	ltr553->ps_cdev.sensors_enable = ltr553_ps_set_enable;
	ltr553->ps_cdev.sensors_poll_delay = NULL;

	ret = sensors_classdev_register(&client->dev, &ltr553->als_cdev);
	if (ret) {
		pr_err("%s: Unable to register to sensors class: %d\n",
				__func__, ret);
		goto err_ltr553_setup;
	}

	ret = sensors_classdev_register(&client->dev, &ltr553->ps_cdev);
	if (ret) {
		pr_err("%s: Unable to register to sensors class: %d\n",
			       __func__, ret);
		goto err_ltr553_class_sysfs;
	}

	dev_dbg(&ltr553->i2c_client->dev, "%s: probe complete\n", __func__);

	return ret;
err_ltr553_class_sysfs:
	sensors_classdev_unregister(&ltr553->als_cdev);
err_ltr553_setup:
	destroy_workqueue(ltr553->workqueue);
err_als_ps_setup:
	if (pdata->power_on)
		pdata->power_on(false);
	if (pdata->exit)
		pdata->exit();
err_out:
	kfree(ltr553);

        //LINE<JIRA_ID><DATE20141027><ltr553 update>zenghaihui
	if (pdata && (client->dev.of_node))
		devm_kfree(&client->dev, pdata);
    
	pdata = NULL;

	return ret;
}

static const struct i2c_device_id ltr553_id[] = {
	{ DEVICE_NAME, 0 },
	{}
};

#ifdef CONFIG_OF
static struct of_device_id liteon_match_table[] = {
		{ .compatible = "liteon,ltr553",},
		{ },
};
#else
#define liteon_match_table NULL
#endif

static int ltr553_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr553_data *data = i2c_get_clientdata(client);
	//struct ltr553_platform_data *pdata = data->platform_data;

        
//LINE<JIRA_ID><DATE20141009><ltr553 update>zenghaihui
#if 0
	if (pdata->power_on)
		pdata->power_on(false);
#else
    if(data->als_enable_flag == 1)
    {
        als_mode_setup(0, data);
    }
#endif

	return 0;
}

static int ltr553_resume(struct device *dev)
{
	int ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr553_data *data = i2c_get_clientdata(client);
	//struct ltr553_platform_data *pdata = data->platform_data;

        
//LINE<JIRA_ID><DATE20141009><ltr553 update>zenghaihui
#if 0
	/* Power on and initalize the device */
	if (pdata->power_on)
		pdata->power_on(true);

	ret = ltr553_setup_poweron(data);
	if (ret < 0) {
		dev_err(&data->i2c_client->dev,
				"%s: Setup Fail...\n", __func__);
				return ret;
	}

	data->als_enable_flag = 1;
	ret = als_mode_setup(1, data);

	data->ps_enable_flag = 1;
	ret = ps_mode_setup(1, data);
#else
        if(data->als_enable_flag == 1)
        {
            ret = als_mode_setup(1, data);
        }
#endif

	return 0;
}

static SIMPLE_DEV_PM_OPS(ltr553_pm_ops, ltr553_suspend, ltr553_resume);
static struct i2c_driver ltr553_driver = {
	.probe = ltr553_probe,
	.id_table = ltr553_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
		.pm = &ltr553_pm_ops,
		.of_match_table = liteon_match_table,

	},
};


static int __init ltr553_init(void)
{
	return i2c_add_driver(&ltr553_driver);
}

static void __exit ltr553_exit(void)
{
	i2c_del_driver(&ltr553_driver);
}

module_init(ltr553_init)
module_exit(ltr553_exit)

MODULE_AUTHOR("Lite-On Technology Corp");
MODULE_DESCRIPTION("LTR-553ALSPS Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);
