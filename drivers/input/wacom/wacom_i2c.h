
#ifndef _LINUX_WACOM_I2C_H
#define _LINUX_WACOM_I2C_H

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

/*
 * Wacom Interrupt Name
 */
#define WACOM_DEVICE_NAME		"wacom_i2c"
#define WACOM_INTERRUPT_NAME		"wacom_irq"
#define WACOM_PDCT_NAME		"wacom_pdct"
#define WACOM_PEN_INSERT_NAME	"wacom_pen_insert"

#define SW_PEN_INSERT 0x0e  /* set = pen out */

#define NAMEBUF 12

/*Wacom Command*/

/* Wacom coord information size in device tree */
#define WACOM_COORDS_ARR_SIZE	9

#define COM_COORD_NUM	7
#define COM_QUERY_NUM	9

#define COM_SAMPLERATE_STOP 0x30
#define COM_SAMPLERATE_40  0x33
#define COM_SAMPLERATE_80  0x32
#define COM_SAMPLERATE_133 0x31
#define COM_SURVEYSCAN     0x2B
#define COM_QUERY          0x2A
#define COM_FLASH          0xff
#define COM_CHECKSUM       0x63

/*I2C address for digitizer and its boot loader*/
#define WACOM_I2C_ADDR 0x56
#define WACOM_I2C_BOOT 0x57

/*Enable/disable irq*/
#define ENABLE_IRQ 1
#define DISABLE_IRQ 0

#define KEY_PEN_PDCT 0x230 /* E-PEN PDCT flag */

/*PDCT Signal*/
#define PDCT_NOSIGNAL 1
#define PDCT_DETECT_PEN 0

#if defined(CONFIG_QC_MODEM)
#define WACOM_HAVE_FWE_PIN
#endif

#define WACOM_POSX_OFFSET 170
#define WACOM_POSY_OFFSET 170

#define EPEN_RESUME_DELAY 180

#define WACOM_PIXELS_PER_MM	20

extern unsigned char *Binary;
extern bool ums_binary;

/*Parameters for wacom own features*/
struct wacom_features {
	int x_max;
	int y_max;
	int pressure_max;
	int x_resolution;
	int y_resolution;
	char comstat;
	u8 data[COM_QUERY_NUM];
	unsigned int fw_version;
	int firm_update_status;
};

/* Wacom data read from DEVICETREE */
struct wacom_devicetree_data {
/* using dts feature */
	int gpio_int;
	u32 irq_gpio_flags;
	int gpio_sda;
	u32 sda_gpio_flags;
	int gpio_scl;
	u32 scl_gpio_flags;
	int vdd_en;
	int gpio_pen_fwe1;
	u32 pen_fwe1_gpio_flags;
	int gpio_pen_pdct;
	u32 pen_pdct_gpio_flags;

	int x_invert;
	int y_invert;
	int xy_switch;
	int min_x;
	int max_x;
	int min_y;
	int max_y;
	int max_pressure;
	int min_pressure;
	int gpio_pendct;
	u32 ic_mpu_ver;
	u32 firmware_version_of_file;
	u32 firmware_checksum[5];
	u32 irq_flags;
	int irq_type;
	u32 wacom_firmup_flag;
	
	int gpio_pen_insert;
};

/*Parameters for i2c driver*/
struct wacom_i2c {
	struct i2c_client *client;
	struct i2c_client *client_boot;
	struct input_dev *input_dev;
	struct mutex lock;
	struct mutex irq_lock;
	struct wakeup_source *wakelock;
	struct completion resume_done;
	struct device	*dev;
	dev_t devepen;
	int irq;
	int irq_pdct;
	bool rdy_pdct;
	volatile bool reset_flag;
	int pen_pdct;
	int gpio;
	int irq_flag;
	int pen_prox;
	int pen_pressed;
	int side_pressed;
	int tool;
	s16 last_x;
	s16 last_y;
	u32 ic_mpu_ver;
	u32 firmware_version_of_file;
	u32 firmware_checksum[5];
	int irq_pen_insert;
	struct delayed_work pen_insert_dwork;
	bool pen_insert;
	int gpio_pen_insert;
#ifdef WACOM_HAVE_FWE_PIN
	int gpio_fwe;
#endif
	bool checksum_result;
	const char name[NAMEBUF];
	struct wacom_features *wac_feature;
	struct wacom_devicetree_data *wac_dt_data;
	struct delayed_work resume_work;
	struct delayed_work pendct_dwork;
	bool pm_suspend;
	bool pen_type;
	bool connection_check;
	bool battery_saving_mode;
    bool pwr_flag;
	volatile bool screen_on;
	bool power_enable;
	bool boot_mode;
	bool query_status;
	struct pinctrl *pinctrl_irq;
	struct pinctrl_state *pin_state_irq;
	
};

#endif /* _LINUX_WACOM_I2C_H */
