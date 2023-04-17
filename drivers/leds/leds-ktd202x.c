#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <asm/errno.h>
#include <linux/cdev.h>
#include <linux/leds.h>

#define KTD_DEBUG
#ifdef KTD_DEBUG
#define LOG_DBG(fmt, args...)   printk(KERN_ERR "[%s]: " fmt, __func__, ## args);
#else
#define LOG_DBG(fmt, args...)
#endif

/* KTD2026 */
//#define KTD2027
#ifdef KTD2027
#define  LED_COUNT  4
#else
#define  LED_COUNT  3
#endif

#define  LED_CURRENT  0x4F
#define TOTAL_TIME_START_COUNT       0x01
#define TOTAL_TIME_START       348
#define TOTAL_TIME_STEP         128

#define TIME_PERCENT_START_COUNT   0X00
#define TIME_PERCENT_STEP (4)

void ktd202x_ch1_led_off(void);
void ktd202x_ch2_led_off(void);
void ktd202x_ch3_led_off(void);

#ifdef KTD2027
void ktd202x_ch4_led_off(void);
#endif

static DEFINE_MUTEX(leds_mutex);

struct i2c_client *ktd202x_i2c;

enum cust_led_id
{
	CUST_LED_CH1,
	CUST_LED_CH2,
	CUST_LED_CH3,
#ifdef KTD2027
	CUST_LED_CH4,
#endif
	CUST_LED_TOTAL
};

struct cust_led_data{
	char *name;
	enum cust_led_id led_id;
	int max_level;
};

struct ktd_led_data {
	struct led_classdev cdev;
	struct cust_led_data cust_data;
	struct work_struct work;

	unsigned int ktd_blink;
	int level;
	int delay_on;
	int delay_off;

	uint8_t ktd_scaling;
	uint8_t ktd_period;
	uint8_t ktd_percent;
	uint8_t ktd_ramp;
	uint8_t ktd_iout;
};

u8 led_status = 0;

struct cust_led_data cust_led_list[LED_COUNT] =
{
	{"ktd-ch1", CUST_LED_CH1, 0x9F},
	{"ktd-ch2", CUST_LED_CH2, 0x4F},
	{"ktd-ch3", CUST_LED_CH3, 0x4F},
#ifdef KTD2027
	{"ktd-ch4", CUST_LED_CH4, 0x4F},
#endif
};

struct ktd_led_data *g_ktd_data[LED_COUNT];


static int ktd202x_write(struct i2c_client *client, u8 reg, u8 val){
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if(ret<0)
		pr_err("[%s]:: i2c write failed, reg[0x%x] val[0x%x]\n", __func__, reg, val);
	return ret;
}
static int ktd202x_read(struct i2c_client *client, u8 reg){
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if(ret<0)
		pr_err("[%s]:: i2c read failed, reg[0x%x] val[0x%x]\n", __func__, reg, ret);
	return ret;
}

#ifdef KTD_DEBUG
void ktd202x_led_on(void){
	ktd202x_write(ktd202x_i2c, 0x06, 0x2f);//set current is 10mA
	ktd202x_write(ktd202x_i2c, 0x04, 0x15);//turn on all of leds
}

 void ktd202x_led_off(void){
	ktd202x_write(ktd202x_i2c, 0x06, 0x00);//set current is 0.125mA
	ktd202x_write(ktd202x_i2c, 0x04, 0x00);//turn off leds
}

void ktd202x_led_sleep_mode(void){
	ktd202x_write(ktd202x_i2c, 0x00, 0x08);
}

#define USING_8BIT(x)	((x < 0) ? 0 : ((x > 255) ? 255:(x)))
#define USING_4BIT(x)	((x < 0) ? 0 : ((x > 15) ? 15:(x)))
#define USING_7BIT(x) ((x < 0) ? 0 : ((x > 127) ? 127:(x)))

void ktd202x_breathing_leds_time(struct cust_led_data *cust, uint8_t scaling, uint8_t period,
									uint8_t percent, uint8_t rising, uint8_t falling, uint8_t iout_value )
{
	uint8_t iout_reg=0x6, ramp_time;
	LOG_DBG("channel[%x], scaling[%x],period[%x], percent[%x], rising[%x], falling[%x], current[%x]\n",
						cust->led_id,
						scaling,
						USING_7BIT(period),
						USING_8BIT(percent),
						USING_4BIT(rising), USING_4BIT(falling),
						USING_8BIT(iout_value));

	switch(cust->led_id){
		case CUST_LED_CH1:
			iout_reg = 0x06;
			led_status = led_status | 0x02;
			break;
		case CUST_LED_CH2:
			iout_reg = 0x07;
			led_status = led_status | 0x08;
			break;
		case CUST_LED_CH3:
			iout_reg = 0x08;
			led_status = led_status | 0x20;
			break;
#ifdef KTD2027
		case CUST_LED_CH4:
			iout_reg = 0x09;
			led_status = led_status | 0x80;
			break;
#endif
		default:
			break;
	}
	ktd202x_write(ktd202x_i2c, 0x00, 0x18 | scaling<<5);
	ktd202x_write(ktd202x_i2c, 0x01, USING_7BIT(period));
	ktd202x_write(ktd202x_i2c, 0x02, USING_8BIT(percent));
	ramp_time = (USING_4BIT(rising)<< 4) | USING_4BIT(falling);
	ktd202x_write(ktd202x_i2c, iout_reg, USING_8BIT(iout_value)/*0x4F*/);
	ktd202x_write(ktd202x_i2c, 0x05, ramp_time/*0x25*/);
	ktd202x_write(ktd202x_i2c, 0x04, led_status);
}

//{+ 2022/04/21
void ktd202x_alternated_breathing_leds_time(enum cust_led_id led1_id, enum cust_led_id led2_id, uint8_t scaling, uint8_t period,
									uint8_t percent1, uint8_t percent2, uint8_t rising, uint8_t falling, uint8_t iout1_value, uint8_t iout2_value )
{
	uint8_t iout1_reg=0x6, iout2_reg=0x7, ramp_time;
	LOG_DBG("channel_1[%x], channel_2[%x], scaling[%x], period[%x], percent_1[%x], percent_2[%x]"
			", rising[%x], falling[%x], current1[%x], current1[%x]\n",
			led1_id, led2_id,
			scaling,
			USING_7BIT(period),
			USING_8BIT(percent1), USING_8BIT(percent2),
			USING_4BIT(rising), USING_4BIT(falling),
			USING_8BIT(iout1_value), USING_8BIT(iout2_value));

	if (led1_id == led2_id) {
		LOG_DBG("channel 1 & 2 is the same ???");
		return;
	}

	switch(led1_id){
		case CUST_LED_CH1:
			iout1_reg = 0x06;
			led_status = led_status | 0x02;
			break;
		case CUST_LED_CH2:
			iout1_reg = 0x07;
			led_status = led_status | 0x08;
			break;
		case CUST_LED_CH3:
			iout1_reg = 0x08;
			led_status = led_status | 0x20;
			break;
#ifdef KTD2027
		case CUST_LED_CH4:
			iout1_reg = 0x09;
			led_status = led_status | 0x80;
			break;
#endif
		default:
			break;
	}

	switch(led2_id){
		case CUST_LED_CH1:
			iout2_reg = 0x06;
			led_status = led_status | 0x03;
			break;
		case CUST_LED_CH2:
			iout2_reg = 0x07;
			led_status = led_status | 0x0C;
			break;
		case CUST_LED_CH3:
			iout2_reg = 0x08;
			led_status = led_status | 0x30;
			break;
#ifdef KTD2027
		case CUST_LED_CH4:
			iout2_reg = 0x09;
			led_status = led_status | 0xC0;
			break;
#endif
		default:
			break;
	}

	ktd202x_write(ktd202x_i2c, 0x00, 0x1A | scaling<<5); //Used TSlot3
	ktd202x_write(ktd202x_i2c, 0x01, USING_7BIT(period));
	ktd202x_write(ktd202x_i2c, 0x02, USING_8BIT(percent1));
	ktd202x_write(ktd202x_i2c, 0x03, USING_8BIT(percent2));
	ramp_time = (USING_4BIT(rising)<< 4) | USING_4BIT(falling);
	ktd202x_write(ktd202x_i2c, iout1_reg, USING_8BIT(iout1_value)/*0x4F*/);
	ktd202x_write(ktd202x_i2c, iout2_reg, USING_8BIT(iout2_value)/*0x4F*/);
	ktd202x_write(ktd202x_i2c, 0x05, ramp_time/*0x25*/);
	ktd202x_write(ktd202x_i2c, 0x04, led_status);
}
//+ 2022/04/21}

#endif

int  ktd202x_translate_timer(unsigned long delay_on, unsigned long delay_off)
{
	int time_count = 0;
	int percent_count = 0;
	if(delay_on==0 && delay_off ==0)
		return -1;

	if((delay_on + delay_off)< 348)
	{
		time_count = 1;
	}
	else
	{
		time_count =
				(delay_on + delay_off - TOTAL_TIME_START)/TOTAL_TIME_STEP + TOTAL_TIME_START_COUNT + 1;
	}

	percent_count =
			(delay_on*100/(delay_on + delay_off))*10 /TIME_PERCENT_STEP + TIME_PERCENT_START_COUNT + 1;

	ktd202x_write(ktd202x_i2c, 0x01, time_count);
	ktd202x_write(ktd202x_i2c, 0x02, percent_count);

	LOG_DBG("time_count = %x  percent_count = %x   \n",  time_count, percent_count);

	return 0;
}

void ktd202x_ch1_led_blink(unsigned long delay_on, unsigned long delay_off)
{
	int ret=0;
	led_status = led_status | 0x02;
	ret = ktd202x_translate_timer(delay_on, delay_off);
	if(ret < 0)
	{
		ktd202x_ch1_led_off();
		return;
	}
	//ktd202x_write(ktd202x_i2c, 0x00, 0x18);
	ktd202x_write(ktd202x_i2c, 0x05, 0x00);
	ktd202x_write(ktd202x_i2c, 0x06, LED_CURRENT);
	ktd202x_write(ktd202x_i2c, 0x04, led_status);

}

void ktd202x_ch2_led_blink(unsigned long delay_on, unsigned long delay_off)
{
	int ret=0;
	led_status = led_status | 0x08;
	ret = ktd202x_translate_timer(delay_on, delay_off);
	if(ret < 0)
	{
		ktd202x_ch2_led_off();
		return;
	}
	//ktd202x_write(ktd202x_i2c, 0x00, 0x18);
	ktd202x_write(ktd202x_i2c, 0x05, 0x00);
	ktd202x_write(ktd202x_i2c, 0x07, LED_CURRENT);
	ktd202x_write(ktd202x_i2c, 0x04, led_status);
}

void ktd202x_ch3_led_blink(unsigned long delay_on, unsigned long delay_off)
{
	int ret=0;
	led_status = led_status | 0x20;
	ret = ktd202x_translate_timer(delay_on, delay_off);
	if(ret < 0)
	{
		ktd202x_ch3_led_off();
		return;
	}
	//ktd202x_write(ktd202x_i2c, 0x00, 0x18);
	ktd202x_write(ktd202x_i2c, 0x05, 0x00);
	ktd202x_write(ktd202x_i2c, 0x08, LED_CURRENT);
	ktd202x_write(ktd202x_i2c, 0x04, led_status);
}

#ifdef KTD2027
void ktd202x_ch4_led_blink(unsigned long delay_on, unsigned long delay_off)
{
	int ret = 0;
	led_status = led_status | 0x80;
	ret = ktd202x_translate_timer(delay_on, delay_off);
	if(ret < 0)
	{
		ktd202x_ch4_led_off();
		return;
	}
	//ktd202x_write(ktd202x_i2c, 0x00, 0x18);
	ktd202x_write(ktd202x_i2c, 0x05, 0x00);
	ktd202x_write(ktd202x_i2c, 0x09, LED_CURRENT);
	ktd202x_write(ktd202x_i2c, 0x04, led_status);

}
#endif

void ktd202x_ch1_led_off(void)
{
	led_status = led_status & (~0x03);
	//ktd202x_write(ktd202x_i2c, 0x00, 0x08);
	ktd202x_write(ktd202x_i2c, 0x06, 0x00);
	ktd202x_write(ktd202x_i2c, 0x04, led_status);
	LOG_DBG("led_status = 0x%x  \n",  led_status);
}

void ktd202x_ch2_led_off(void)
{
	led_status = led_status & (~0x0C);
	//ktd202x_write(ktd202x_i2c, 0x00, 0x08);
	ktd202x_write(ktd202x_i2c, 0x07, 0x00);
	ktd202x_write(ktd202x_i2c, 0x04, led_status);
	LOG_DBG("led_status = 0x%x  \n",  led_status);
}

void ktd202x_ch3_led_off(void)
{
	led_status = led_status & (~0x30);
	//ktd202x_write(ktd202x_i2c, 0x00, 0x08);
	ktd202x_write(ktd202x_i2c, 0x08, 0x00);
	ktd202x_write(ktd202x_i2c, 0x04, led_status);
	LOG_DBG("led_status = 0x%x  \n",  led_status);
}

#ifdef KTD2027
void ktd202x_ch4_led_off(void)
{
	led_status = led_status & (~0xC0);

	ktd202x_write(ktd202x_i2c, 0x09, 0x00);
	ktd202x_write(ktd202x_i2c, 0x04, led_status);
	LOG_DBG("led_status = 0x%x\n",  led_status);
}
#endif

void ktd202x_ch1_led_on(int level)
{
	led_status = led_status | 0x01;
	led_status = led_status & (~0x02) ;

	//ktd202x_write(ktd202x_i2c, 0x00, 0x18);
	//ktd202x_write(ktd202x_i2c, 0x02, 0XFF);
//	ktd202x_write(ktd202x_i2c, 0x06, LED_CURRENT);
	ktd202x_write(ktd202x_i2c, 0x06, level);
	ktd202x_write(ktd202x_i2c, 0x04, led_status);
	LOG_DBG("led_status = 0x%x, led_current=<0x%x>  \n",  led_status, level);
}

void ktd202x_ch2_led_on(int level)
{
	led_status = led_status|0x04;
	led_status = led_status & (~0x08);

	//ktd202x_write(ktd202x_i2c, 0x00, 0x18);
	//ktd202x_write(ktd202x_i2c, 0x02, 0XFF);
//	ktd202x_write(ktd202x_i2c, 0x07, LED_CURRENT);
	ktd202x_write(ktd202x_i2c, 0x07, level);
	ktd202x_write(ktd202x_i2c, 0x04, led_status);
	LOG_DBG("led_status = 0x%x, led_current=<0x%x>\n", led_status, level);
}

void ktd202x_ch3_led_on(int level)
{
	led_status = led_status|0x10;
	led_status = led_status & (~0x20);

	//ktd202x_write(ktd202x_i2c, 0x00, 0x18);
	//ktd202x_write(ktd202x_i2c, 0x02, 0XFF);
//	ktd202x_write(ktd202x_i2c, 0x07, LED_CURRENT);
	ktd202x_write(ktd202x_i2c, 0x08, level);
	ktd202x_write(ktd202x_i2c, 0x04, led_status);
	LOG_DBG("led_status  = %x , led_current =0x%x \n", led_status, level);
}

#ifdef KTD2027
void ktd202x_ch4_led_on(int level)
{
	led_status = led_status|0x40;
	led_status = led_status & (~0x80);

	//ktd202x_write(ktd202x_i2c, 0x00, 0x18);
	//ktd202x_write(ktd202x_i2c, 0x02, 0xFF);
	ktd202x_write(ktd202x_i2c, 0x09, level);
	ktd202x_write(ktd202x_i2c, 0x04, led_status);
	LOG_DBG("led_status  = %x , led_current =0x%x \n", led_status, level);
}
#endif

void ktd_cust_led_set_cust(struct cust_led_data *cust, int level)
{
	if (level > cust->max_level)
		level = cust->max_level;
#ifdef KTD_DEBUG
	pr_info("ktd_cust_led_set_cust: level = 0x%02x, cust->led_id = %d\n",level , cust->led_id);
#endif
	if(level > 0)
	{
		switch(cust->led_id){
			case  CUST_LED_CH1:
				ktd202x_ch1_led_on(level);
				break;
			case  CUST_LED_CH2:
				ktd202x_ch2_led_on(level);
				break;
			case  CUST_LED_CH3:
				ktd202x_ch3_led_on(level);
				break;
#ifdef KTD2027
			case CUST_LED_CH4:
				ktd202x_ch4_led_on(level);
				break;
#endif
			default:
				break;
			}
	}
	else
	{
		switch(cust->led_id){
			case  CUST_LED_CH1:
				ktd202x_ch1_led_off();
				break;
			case  CUST_LED_CH2:
				ktd202x_ch2_led_off();
				break;
			case  CUST_LED_CH3:
				ktd202x_ch3_led_off();
				break;
#ifdef KTD2027
			case  CUST_LED_CH4:
				ktd202x_ch4_led_off();
				break;
#endif
			default:
				break;
		}
	}
}

void ktd_led_blink_set(struct ktd_led_data *led_data)
{

	LOG_DBG("led_data->cust_data.led_id = %d \n" ,led_data->cust_data.led_id);
	LOG_DBG("led_data->delay_on = %d  led_data->delay_off = %d\n",
					led_data->delay_on, led_data->delay_off);

	switch(led_data->cust_data.led_id)
	{
		case CUST_LED_CH1:
			ktd202x_ch1_led_blink(100, 100);
			break;
		case CUST_LED_CH2:
			ktd202x_ch2_led_blink(100, 100);
			break;
		case CUST_LED_CH3:
			ktd202x_ch3_led_blink(100, 100);
			break;
#ifdef KTD2027
		case CUST_LED_CH4:
			ktd202x_ch4_led_blink(100, 100);
			break;
#endif
		default:
			break;
	}
}

static int ktd_blink_set(struct led_classdev *led_cdev, unsigned long *delay_on, unsigned long *delay_off)
{
	struct ktd_led_data *led_data =
		container_of(led_cdev, struct ktd_led_data, cdev);

	led_data->delay_on = *delay_on;
	led_data->delay_off = *delay_off;

	ktd_led_blink_set(led_data);

	return 0;
}

static void ktd_led_set(struct led_classdev *led_cdev, enum led_brightness level)
{
	struct ktd_led_data *led_data =
		container_of(led_cdev, struct ktd_led_data, cdev);

	LOG_DBG("level is -<0x%x>\n", level);
	if (level < LED_OFF) {
			return;
	}

	if (level > led_data->cdev.max_brightness)
		level = led_data->cdev.max_brightness;

	led_data->cdev.brightness = level;
	schedule_work(&led_data->work);
}

void ktd_led_work(struct work_struct *work)
{
	struct ktd_led_data *led_data =
		container_of(work, struct ktd_led_data, work);

	mutex_lock(&leds_mutex);
	ktd_cust_led_set_cust(&led_data->cust_data, led_data->cdev.brightness);
	mutex_unlock(&leds_mutex);

}

static ssize_t ktd202x_blink_led_show(struct device *dev,
											struct device_attribute *attr, char *buf)
{
	struct ktd_led_data *led_data = dev_get_drvdata(dev);

	switch(led_data->cust_data.led_id){
			case CUST_LED_CH1:
				ktd202x_ch1_led_blink(100, 100);
				break;
			case CUST_LED_CH2:
				ktd202x_ch2_led_blink(100, 100);
				break;
			case CUST_LED_CH3:
				ktd202x_ch3_led_blink(100, 100);
				break;
#ifdef KTD2027
			case CUST_LED_CH4:
				ktd202x_ch4_led_blink(100, 100);
				break;
#endif
			default:
				break;

	}
	return sprintf(buf, "%d\n", led_data->cust_data.led_id);
}

#define CURRENT_SETTING(x)  ((x > 24000) ? 255 :( (x <125) ? 0 : ((x/125)-1)))
static ssize_t ktd202x_breathing_show(struct device *dev,
											struct device_attribute *attr, char *buf)
{
	struct ktd_led_data *led_data = dev_get_drvdata(dev);

	//ktd202x_breathing_leds_time(&led_data->cust_data, 0x0A/*period*/, 200 /*percent*/,
	//								0x0A, 0x0A, /*rising time and falling time*/
	//								CURRENT_SETTING(10000));
	ktd202x_breathing_leds_time(&led_data->cust_data, led_data->ktd_scaling, led_data->ktd_period,
						led_data->ktd_percent, (led_data->ktd_ramp >> 4),
						(led_data->ktd_ramp & 0x0F),led_data->ktd_iout);


	return sprintf(buf, "%d\n", led_data->cust_data.led_id);
}

//{ +2022/04/22
static ssize_t ktd202x_alternated_show(struct device *dev,
											struct device_attribute *attr, char *buf)
{
	struct ktd_led_data *led_data = dev_get_drvdata(dev);

	ktd202x_alternated_breathing_leds_time(CUST_LED_CH1, CUST_LED_CH1, 0x1 /*scaling*/, 0x4D /*period*/
						, 0x3B/*percent2, timer1*/, 0x0B /*percent2, timer2*/
						, 0x9/*rising*/, 0x9/*falling*/, CURRENT_SETTING(10000), CURRENT_SETTING(10000));


	return sprintf(buf, "%d\n", led_data->cust_data.led_id);
}
//+ 2022/04/21 }

static ssize_t ktd202x_reg01_period_store(struct device *dev,
							struct device_attribute *attr, const char *buf, size_t count)
{
	struct ktd_led_data *led_data = dev_get_drvdata(dev);
	ssize_t ret;
	unsigned int value;

	ret = kstrtouint(buf, 10, &value);

	if(ret){
		dev_err(&ktd202x_i2c->dev, "%s: failed to store!\n", __func__);
		return ret;
	}

	led_data->ktd_period = USING_7BIT(value);
	ktd202x_write(ktd202x_i2c, 0x01, led_data->ktd_period);

	return count;
}

static ssize_t ktd202x_reg02_percent_store(struct device *dev,
							struct device_attribute *attr, const char *buf, size_t count)
{
	struct ktd_led_data *led_data = dev_get_drvdata(dev);
	ssize_t ret;
	unsigned int value;

	ret = kstrtouint(buf, 10, &value);

	if(ret){
		dev_err(&ktd202x_i2c->dev, "%s: failed to store!\n", __func__);
		return ret;
	}

	led_data->ktd_percent = USING_8BIT(value);
	ktd202x_write(ktd202x_i2c, 0x02, led_data->ktd_percent);

	return count;
}

static ssize_t ktd202x_reg03_percent_store(struct device *dev,
							struct device_attribute *attr, const char *buf, size_t count)
{
	struct ktd_led_data *led_data = dev_get_drvdata(dev);
	ssize_t ret;
	unsigned int value;

	ret = kstrtouint(buf, 10, &value);

	if(ret){
		dev_err(&ktd202x_i2c->dev, "%s: failed to store!\n", __func__);
		return ret;
	}
	led_data->ktd_percent = USING_8BIT(value);
	ktd202x_write(ktd202x_i2c, 0x02, led_data->ktd_percent);

	return count;
}

static ssize_t ktd202x_reg04_ramp_store(struct device *dev,
							struct device_attribute *attr, const char *buf, size_t count)
{
	struct ktd_led_data *led_data = dev_get_drvdata(dev);
	ssize_t ret;
	unsigned int value;

	ret = kstrtouint(buf, 10, &value);

	if(ret){
		dev_err(&ktd202x_i2c->dev, "%s: failed to store!\n", __func__);
		return ret;
	}
	led_data->ktd_ramp = USING_8BIT(value);
	ktd202x_write(ktd202x_i2c, 0x02, led_data->ktd_ramp);

	return count;
}

static ssize_t ktd202x_reg678_iout_store(struct device *dev,
							struct device_attribute *attr, const char *buf, size_t count)
{
	struct ktd_led_data *led_data = dev_get_drvdata(dev);
	ssize_t ret;
	unsigned int value;
	uint8_t update;

	ret = kstrtouint(buf, 10, &value);

	if(ret){
		dev_err(&ktd202x_i2c->dev, "%s: failed to store!\n", __func__);
		return ret;
	}

	update= USING_8BIT(value);
	if (update > led_data->cust_data.max_level)
		update = led_data->cust_data.max_level;

#ifdef KTD_DEBUG
	LOG_DBG("update = 0x%02x\n", update);
#endif

	switch(led_data->cust_data.led_id){
		case CUST_LED_CH1:
			ktd202x_write(ktd202x_i2c, 0x06, update);
			break;
		case CUST_LED_CH2:
			ktd202x_write(ktd202x_i2c, 0x07, update);
			break;
		case CUST_LED_CH3:
			ktd202x_write(ktd202x_i2c, 0x08, update);
			break;
#ifdef KTD2027
		case CUST_LED_CH4:
			ktd202x_write(ktd202x_i2c, 0x09, update);
			break;
#endif
		default:
			break;
	}

	led_data->ktd_iout = update;
	return count;
}

static ssize_t ktd202x_reg_dump_store(struct device *dev,
											struct device_attribute *attr, char *buf)
{
	//struct ktd_led_data *led_data = dev_get_drvdata(dev);

	char regdump[1024];
	unsigned int i;

	memset(regdump, 0, 1024);
	strcpy(regdump, "====REG DUMP=====\n");
	for(i=0; i<0x16; i++) {
		uint8_t data = 0;
		char buff[32];
		data = ktd202x_read(ktd202x_i2c, i);
		sprintf(buff, "REG(0x%2X): 0x%2X\n", i, data);
		strcat(regdump, buff);
	}

	return sprintf(buf, "%s\n", regdump);
}

static DEVICE_ATTR(ktd_blink, 0444, ktd202x_blink_led_show, NULL/*ktd202x_blink_led_store*/);
static DEVICE_ATTR(ktd_breathing, 0444, ktd202x_breathing_show, NULL);
static DEVICE_ATTR(ktd_alternated, 0444, ktd202x_alternated_show, NULL); //+2022/04/21
static DEVICE_ATTR(period, 0200, NULL, ktd202x_reg01_period_store);
static DEVICE_ATTR(timer1_p, 0200, NULL, ktd202x_reg02_percent_store);
static DEVICE_ATTR(timer2_p, 0200, NULL, ktd202x_reg03_percent_store);
static DEVICE_ATTR(ramp, 0200, NULL, ktd202x_reg04_ramp_store);
static DEVICE_ATTR(iout, 0200, NULL, ktd202x_reg678_iout_store);
//static DEVICE_ATTR(channel, 0200, NULL, ktd202x_reg04_channel_store);
static DEVICE_ATTR(regdump, 0200, ktd202x_reg_dump_store, NULL);

static struct attribute *ktd202x_led_attribute[] = {
	&dev_attr_ktd_blink.attr,
	&dev_attr_ktd_breathing.attr,
	&dev_attr_ktd_alternated.attr,  //+2022/04/21
	&dev_attr_period.attr,
	&dev_attr_timer1_p.attr,
	&dev_attr_timer2_p.attr,
	&dev_attr_ramp.attr,
	&dev_attr_iout.attr,
	&dev_attr_regdump.attr,
//	&dev_attr_channel.attr,
	NULL
};

static const struct attribute_group ktd202x_led_attr_group = {
	.attrs = ktd202x_led_attribute,
};

static int ktd202x_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int err = 0;
	int i = 0;

	LOG_DBG("start probe!\n");
	if (!i2c_check_functionality(client->adapter,I2C_FUNC_I2C)){
		dev_err(&client->dev,"[%s]: check_func failed.", __func__);
		err = -ENODEV;
		return err;
	}

	ktd202x_i2c = client;

	ktd202x_write(ktd202x_i2c, 0x00, 0x18);
	ktd202x_write(ktd202x_i2c, 0x04, 0x00);//turn off leds

	for(i = 0; i < LED_COUNT; i++)
	{
		g_ktd_data[i] = kzalloc(sizeof(struct ktd_led_data), GFP_KERNEL);
		if (!g_ktd_data[i]) {
			dev_err(&client->dev,"%s: memory allocation failed.", __func__);
			err = -ENOMEM;
			return err;
		}

		g_ktd_data[i]->cust_data.name = cust_led_list[i].name;
		g_ktd_data[i]->cust_data.led_id = cust_led_list[i].led_id;
		g_ktd_data[i]->cust_data.max_level = cust_led_list[i].max_level;
		g_ktd_data[i]->cdev.name = cust_led_list[i].name;
		g_ktd_data[i]->cdev.brightness_set = ktd_led_set;
		g_ktd_data[i]->cdev.blink_set =ktd_blink_set;

		/*for Breathing LED*/
		g_ktd_data[i]->ktd_scaling = 0x1;
		g_ktd_data[i]->ktd_period = 0x0A;
		g_ktd_data[i]->ktd_percent = 200;
		g_ktd_data[i]->ktd_ramp = 0x25;
		g_ktd_data[i]->ktd_iout = CURRENT_SETTING(10000);//10mA

		INIT_WORK(&g_ktd_data[i]->work, ktd_led_work);

		err = led_classdev_register((struct device *)&client->dev , &g_ktd_data[i]->cdev);

		err = sysfs_create_group(&g_ktd_data[i]->cdev.dev->kobj,
						&ktd202x_led_attr_group);
		if(err){
			dev_err(&client->dev,"failed to create sysfs group\n");
		}
	}

	return err;
}

static int ktd202x_remove(struct i2c_client *client)
{
	int i;

	ktd202x_led_off();
	for (i = 0; i < LED_COUNT; i++) {
		led_classdev_unregister(&g_ktd_data[i]->cdev);
		cancel_work_sync(&g_ktd_data[i]->work);
		kfree(g_ktd_data[i]);
		g_ktd_data[i] = NULL;
	}

	return 0;
}

static struct i2c_device_id ktd202x_id[] = {
	{ "ktd202x", 0 },
	{ }
};

static struct of_device_id ktd202x_match_table[] = {
	{ .compatible = "ktd, ktd202x"},
	{ },
};

MODULE_DEVICE_TABLE(i2c, ktd202x_id);

static struct i2c_driver ktd202x_driver = {
	.driver = {
		.name = "ktd202x",
		.owner = THIS_MODULE,
		.of_match_table = ktd202x_match_table,
	},
	.probe	  = ktd202x_probe,
	.remove   = ktd202x_remove,
	.id_table = ktd202x_id,
};

module_i2c_driver(ktd202x_driver);

MODULE_AUTHOR("Luke Jang");

