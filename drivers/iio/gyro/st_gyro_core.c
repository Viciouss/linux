// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics gyroscopes driver
 *
 * Copyright 2012-2013 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>

#include <linux/iio/common/st_sensors.h>
#include "st_gyro.h"

#define ST_GYRO_NUMBER_DATA_CHANNELS		3

/* DEFAULT VALUE FOR SENSORS */
#define ST_GYRO_DEFAULT_OUT_X_L_ADDR		0x28
#define ST_GYRO_DEFAULT_OUT_Y_L_ADDR		0x2a
#define ST_GYRO_DEFAULT_OUT_Z_L_ADDR		0x2c

/* FULLSCALE */
#define ST_GYRO_FS_AVL_245DPS			245
#define ST_GYRO_FS_AVL_250DPS			250
#define ST_GYRO_FS_AVL_500DPS			500
#define ST_GYRO_FS_AVL_2000DPS			2000

static const struct iio_chan_spec st_gyro_16bit_channels[] = {
	ST_SENSORS_LSM_CHANNELS(IIO_ANGL_VEL,
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
			ST_SENSORS_SCAN_X, 1, IIO_MOD_X, 's', IIO_LE, 16, 16,
			ST_GYRO_DEFAULT_OUT_X_L_ADDR),
	ST_SENSORS_LSM_CHANNELS(IIO_ANGL_VEL,
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
			ST_SENSORS_SCAN_Y, 1, IIO_MOD_Y, 's', IIO_LE, 16, 16,
			ST_GYRO_DEFAULT_OUT_Y_L_ADDR),
	ST_SENSORS_LSM_CHANNELS(IIO_ANGL_VEL,
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
			ST_SENSORS_SCAN_Z, 1, IIO_MOD_Z, 's', IIO_LE, 16, 16,
			ST_GYRO_DEFAULT_OUT_Z_L_ADDR),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static const struct st_sensor_settings st_gyro_sensors_settings[] = {
	{
		.wai = 0xd3,
		.wai_addr = ST_SENSORS_DEFAULT_WAI_ADDRESS,
		.sensors_supported = {
			[0] = L3G4200D_GYRO_DEV_NAME,
			[1] = LSM330DL_GYRO_DEV_NAME,
		},
		.ch = (struct iio_chan_spec *)st_gyro_16bit_channels,
		.odr = {
			.addr = 0x20,
			.mask = 0xc0,
			.odr_avl = {
				{ .hz = 100, .value = 0x00, },
				{ .hz = 200, .value = 0x01, },
				{ .hz = 400, .value = 0x02, },
				{ .hz = 800, .value = 0x03, },
			},
		},
		.pw = {
			.addr = 0x20,
			.mask = 0x08,
			.value_on = ST_SENSORS_DEFAULT_POWER_ON_VALUE,
			.value_off = ST_SENSORS_DEFAULT_POWER_OFF_VALUE,
		},
		.enable_axis = {
			.addr = ST_SENSORS_DEFAULT_AXIS_ADDR,
			.mask = ST_SENSORS_DEFAULT_AXIS_MASK,
		},
		.fs = {
			.addr = 0x23,
			.mask = 0x30,
			.fs_avl = {
				[0] = {
					.num = ST_GYRO_FS_AVL_250DPS,
					.value = 0x00,
					.gain = IIO_DEGREE_TO_RAD(8750),
				},
				[1] = {
					.num = ST_GYRO_FS_AVL_500DPS,
					.value = 0x01,
					.gain = IIO_DEGREE_TO_RAD(17500),
				},
				[2] = {
					.num = ST_GYRO_FS_AVL_2000DPS,
					.value = 0x02,
					.gain = IIO_DEGREE_TO_RAD(70000),
				},
			},
		},
		.bdu = {
			.addr = 0x23,
			.mask = 0x80,
		},
		.drdy_irq = {
			.int2 = {
				.addr = 0x22,
				.mask = 0x08,
			},
			/*
			 * The sensor has IHL (active low) and open
			 * drain settings, but only for INT1 and not
			 * for the DRDY line on INT2.
			 */
			.stat_drdy = {
				.addr = ST_SENSORS_DEFAULT_STAT_ADDR,
				.mask = 0x07,
			},
		},
		.sim = {
			.addr = 0x23,
			.value = BIT(0),
		},
		.multi_read_bit = true,
		.bootime = 2,
	},
	{
		.wai = 0xd4,
		.wai_addr = ST_SENSORS_DEFAULT_WAI_ADDRESS,
		.sensors_supported = {
			[0] = L3GD20_GYRO_DEV_NAME,
			[1] = LSM330D_GYRO_DEV_NAME,
			[2] = LSM330DLC_GYRO_DEV_NAME,
			[3] = L3G4IS_GYRO_DEV_NAME,
			[4] = LSM330_GYRO_DEV_NAME,
		},
		.ch = (struct iio_chan_spec *)st_gyro_16bit_channels,
		.odr = {
			.addr = 0x20,
			.mask = 0xc0,
			.odr_avl = {
				{ .hz = 95, .value = 0x00, },
				{ .hz = 190, .value = 0x01, },
				{ .hz = 380, .value = 0x02, },
				{ .hz = 760, .value = 0x03, },
			},
		},
		.pw = {
			.addr = 0x20,
			.mask = 0x08,
			.value_on = ST_SENSORS_DEFAULT_POWER_ON_VALUE,
			.value_off = ST_SENSORS_DEFAULT_POWER_OFF_VALUE,
		},
		.enable_axis = {
			.addr = ST_SENSORS_DEFAULT_AXIS_ADDR,
			.mask = ST_SENSORS_DEFAULT_AXIS_MASK,
		},
		.fs = {
			.addr = 0x23,
			.mask = 0x30,
			.fs_avl = {
				[0] = {
					.num = ST_GYRO_FS_AVL_250DPS,
					.value = 0x00,
					.gain = IIO_DEGREE_TO_RAD(8750),
				},
				[1] = {
					.num = ST_GYRO_FS_AVL_500DPS,
					.value = 0x01,
					.gain = IIO_DEGREE_TO_RAD(17500),
				},
				[2] = {
					.num = ST_GYRO_FS_AVL_2000DPS,
					.value = 0x02,
					.gain = IIO_DEGREE_TO_RAD(70000),
				},
			},
		},
		.bdu = {
			.addr = 0x23,
			.mask = 0x80,
		},
		.drdy_irq = {
			.int2 = {
				.addr = 0x22,
				.mask = 0x08,
			},
			/*
			 * The sensor has IHL (active low) and open
			 * drain settings, but only for INT1 and not
			 * for the DRDY line on INT2.
			 */
			.stat_drdy = {
				.addr = ST_SENSORS_DEFAULT_STAT_ADDR,
				.mask = 0x07,
			},
		},
		.sim = {
			.addr = 0x23,
			.value = BIT(0),
		},
		.multi_read_bit = true,
		.bootime = 2,
	},
	{
		.wai = 0xd4,
		.wai_addr = ST_SENSORS_DEFAULT_WAI_ADDRESS,
		.sensors_supported = {
			[0] = LSM9DS0_GYRO_DEV_NAME,
		},
		.ch = (struct iio_chan_spec *)st_gyro_16bit_channels,
		.odr = {
			.addr = 0x20,
			.mask = GENMASK(7, 6),
			.odr_avl = {
				{ .hz = 95, .value = 0x00, },
				{ .hz = 190, .value = 0x01, },
				{ .hz = 380, .value = 0x02, },
				{ .hz = 760, .value = 0x03, },
			},
		},
		.pw = {
			.addr = 0x20,
			.mask = BIT(3),
			.value_on = ST_SENSORS_DEFAULT_POWER_ON_VALUE,
			.value_off = ST_SENSORS_DEFAULT_POWER_OFF_VALUE,
		},
		.enable_axis = {
			.addr = ST_SENSORS_DEFAULT_AXIS_ADDR,
			.mask = ST_SENSORS_DEFAULT_AXIS_MASK,
		},
		.fs = {
			.addr = 0x23,
			.mask = GENMASK(5, 4),
			.fs_avl = {
				[0] = {
					.num = ST_GYRO_FS_AVL_245DPS,
					.value = 0x00,
					.gain = IIO_DEGREE_TO_RAD(8750),
				},
				[1] = {
					.num = ST_GYRO_FS_AVL_500DPS,
					.value = 0x01,
					.gain = IIO_DEGREE_TO_RAD(17500),
				},
				[2] = {
					.num = ST_GYRO_FS_AVL_2000DPS,
					.value = 0x02,
					.gain = IIO_DEGREE_TO_RAD(70000),
				},
			},
		},
		.bdu = {
			.addr = 0x23,
			.mask = BIT(7),
		},
		.drdy_irq = {
			.int2 = {
				.addr = 0x22,
				.mask = BIT(3),
			},
			/*
			 * The sensor has IHL (active low) and open
			 * drain settings, but only for INT1 and not
			 * for the DRDY line on INT2.
			 */
			.stat_drdy = {
				.addr = ST_SENSORS_DEFAULT_STAT_ADDR,
				.mask = GENMASK(2, 0),
			},
		},
		.sim = {
			.addr = 0x23,
			.value = BIT(0),
		},
		.multi_read_bit = true,
		.bootime = 2,
	},
	{
		.wai = 0xd7,
		.wai_addr = ST_SENSORS_DEFAULT_WAI_ADDRESS,
		.sensors_supported = {
			[0] = L3GD20H_GYRO_DEV_NAME,
		},
		.ch = (struct iio_chan_spec *)st_gyro_16bit_channels,
		.odr = {
			.addr = 0x20,
			.mask = 0xc0,
			.odr_avl = {
				{ .hz = 100, .value = 0x00, },
				{ .hz = 200, .value = 0x01, },
				{ .hz = 400, .value = 0x02, },
				{ .hz = 800, .value = 0x03, },
			},
		},
		.pw = {
			.addr = 0x20,
			.mask = 0x08,
			.value_on = ST_SENSORS_DEFAULT_POWER_ON_VALUE,
			.value_off = ST_SENSORS_DEFAULT_POWER_OFF_VALUE,
		},
		.enable_axis = {
			.addr = ST_SENSORS_DEFAULT_AXIS_ADDR,
			.mask = ST_SENSORS_DEFAULT_AXIS_MASK,
		},
		.fs = {
			.addr = 0x23,
			.mask = 0x30,
			.fs_avl = {
				[0] = {
					.num = ST_GYRO_FS_AVL_245DPS,
					.value = 0x00,
					.gain = IIO_DEGREE_TO_RAD(8750),
				},
				[1] = {
					.num = ST_GYRO_FS_AVL_500DPS,
					.value = 0x01,
					.gain = IIO_DEGREE_TO_RAD(17500),
				},
				[2] = {
					.num = ST_GYRO_FS_AVL_2000DPS,
					.value = 0x02,
					.gain = IIO_DEGREE_TO_RAD(70000),
				},
			},
		},
		.bdu = {
			.addr = 0x23,
			.mask = 0x80,
		},
		.drdy_irq = {
			.int2 = {
				.addr = 0x22,
				.mask = 0x08,
			},
			/*
			 * The sensor has IHL (active low) and open
			 * drain settings, but only for INT1 and not
			 * for the DRDY line on INT2.
			 */
			.stat_drdy = {
				.addr = ST_SENSORS_DEFAULT_STAT_ADDR,
				.mask = 0x07,
			},
		},
		.sim = {
			.addr = 0x23,
			.value = BIT(0),
		},
		.multi_read_bit = true,
		.bootime = 2,
	},
};

static int st_gyro_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *ch, int *val,
							int *val2, long mask)
{
	int err;
	struct st_sensor_data *gdata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		err = st_sensors_read_info_raw(indio_dev, ch, val);
		if (err < 0)
			goto read_error;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = gdata->current_fullscale->gain;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = gdata->odr;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}

read_error:
	return err;
}

static int st_gyro_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	int err;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		err = st_sensors_set_fullscale_by_gain(indio_dev, val2);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (val2)
			return -EINVAL;
		mutex_lock(&indio_dev->mlock);
		err = st_sensors_set_odr(indio_dev, val);
		mutex_unlock(&indio_dev->mlock);
		return err;
	default:
		err = -EINVAL;
	}

	return err;
}

static ST_SENSORS_DEV_ATTR_SAMP_FREQ_AVAIL();
static ST_SENSORS_DEV_ATTR_SCALE_AVAIL(in_anglvel_scale_available);

static struct attribute *st_gyro_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_gyro_attribute_group = {
	.attrs = st_gyro_attributes,
};

static const struct iio_info gyro_info = {
	.attrs = &st_gyro_attribute_group,
	.read_raw = &st_gyro_read_raw,
	.write_raw = &st_gyro_write_raw,
	.debugfs_reg_access = &st_sensors_debugfs_reg_access,
};

#ifdef CONFIG_IIO_TRIGGER
static const struct iio_trigger_ops st_gyro_trigger_ops = {
	.set_trigger_state = ST_GYRO_TRIGGER_SET_STATE,
	.validate_device = st_sensors_validate_device,
};
#define ST_GYRO_TRIGGER_OPS (&st_gyro_trigger_ops)
#else
#define ST_GYRO_TRIGGER_OPS NULL
#endif

#if defined(CONFIG_ACPI) || defined(CONFIG_OF)
static const struct iio_mount_matrix *
get_mount_matrix(const struct iio_dev *indio_dev,
		 const struct iio_chan_spec *chan)
{
	struct st_sensor_data *adata = iio_priv(indio_dev);

	return adata->mount_matrix;
}

static const struct iio_chan_spec_ext_info mount_matrix_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_ALL, get_mount_matrix),
	{ },
};
#endif

#ifdef CONFIG_ACPI
/* Read ST-specific _ONT orientation data from ACPI and generate an
 * appropriate mount matrix.
 */
static int apply_orientation(struct iio_dev *indio_dev,
			     struct iio_chan_spec *channels)
{
	struct st_sensor_data *adata = iio_priv(indio_dev);
	struct acpi_buffer buffer = {ACPI_ALLOCATE_BUFFER, NULL};
	struct acpi_device *adev;
	union acpi_object *ont;
	union acpi_object *elements;
	acpi_status status;
	int ret = -EINVAL;
	unsigned int val;
	int i, j;
	int final_ont[3][3] = { { 0 }, };

	/* For some reason, ST's _ONT translation does not apply directly
	 * to the data read from the sensor. Another translation must be
	 * performed first, as described by the matrix below. Perhaps
	 * ST required this specific translation for the first product
	 * where the device was mounted?
	 */
	const int default_ont[3][3] = {
		{  0,  1,  0 },
		{ -1,  0,  0 },
		{  0,  0, -1 },
	};


	adev = ACPI_COMPANION(adata->dev);
	if (!adev)
		return 0;

	/* Read _ONT data, which should be a package of 6 integers. */
	status = acpi_evaluate_object(adev->handle, "_ONT", NULL, &buffer);
	if (status == AE_NOT_FOUND) {
		return 0;
	} else if (ACPI_FAILURE(status)) {
		dev_warn(&indio_dev->dev, "failed to execute _ONT: %d\n",
			 status);
		return status;
	}

	ont = buffer.pointer;
	if (ont->type != ACPI_TYPE_PACKAGE || ont->package.count != 6)
		goto out;

	/* The first 3 integers provide axis order information.
	 * e.g. 0 1 2 would indicate normal X,Y,Z ordering.
	 * e.g. 1 0 2 indicates that data arrives in order Y,X,Z.
	 */
	elements = ont->package.elements;
	for (i = 0; i < 3; i++) {
		if (elements[i].type != ACPI_TYPE_INTEGER)
			goto out;

		val = elements[i].integer.value;
		if (val > 2)
			goto out;

		/* Avoiding full matrix multiplication, we simply reorder the
		 * columns in the default_ont matrix according to the
		 * ordering provided by _ONT.
		 */
		final_ont[0][i] = default_ont[0][val];
		final_ont[1][i] = default_ont[1][val];
		final_ont[2][i] = default_ont[2][val];
	}

	/* The final 3 integers provide sign flip information.
	 * 0 means no change, 1 means flip.
	 * e.g. 0 0 1 means that Z data should be sign-flipped.
	 * This is applied after the axis reordering from above.
	 */
	elements += 3;
	for (i = 0; i < 3; i++) {
		if (elements[i].type != ACPI_TYPE_INTEGER)
			goto out;

		val = elements[i].integer.value;
		if (val != 0 && val != 1)
			goto out;
		if (!val)
			continue;

		/* Flip the values in the indicated column */
		final_ont[0][i] *= -1;
		final_ont[1][i] *= -1;
		final_ont[2][i] *= -1;
	}

	/* Convert our integer matrix to a string-based iio_mount_matrix */
	adata->mount_matrix = devm_kmalloc(&indio_dev->dev,
					   sizeof(*adata->mount_matrix),
					   GFP_KERNEL);
	if (!adata->mount_matrix) {
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			int matrix_val = final_ont[i][j];
			char *str_value;

			switch (matrix_val) {
			case -1:
				str_value = "-1";
				break;
			case 0:
				str_value = "0";
				break;
			case 1:
				str_value = "1";
				break;
			default:
				goto out;
			}
			adata->mount_matrix->rotation[i * 3 + j] = str_value;
		}
	}

	/* Expose the mount matrix via ext_info */
	for (i = 0; i < indio_dev->num_channels; i++)
		channels[i].ext_info = mount_matrix_ext_info;

	ret = 0;
	dev_info(&indio_dev->dev, "computed mount matrix from ACPI\n");

out:
	kfree(buffer.pointer);
	return ret;
}
#elif defined(CONFIG_OF)
static int apply_orientation(struct iio_dev *indio_dev,
				  struct iio_chan_spec *channels)
{
	struct st_sensor_data *adata = iio_priv(indio_dev);
	struct device *dev = adata->dev;
	int ret = -EINVAL;
	int i;

	adata->mount_matrix = devm_kmalloc(dev,
					   sizeof(*adata->mount_matrix),
					   GFP_KERNEL);
	if (!adata->mount_matrix) {
		ret = -ENOMEM;
		goto out;
	}

	ret = iio_read_mount_matrix(dev, "mount-matrix", adata->mount_matrix);
	if (ret) {
		dev_err(dev, "error reading mount-matrix");
		goto out;
	}

	/* Expose the mount matrix via ext_info */
	for (i = 0; i < indio_dev->num_channels; i++)
		channels[i].ext_info = mount_matrix_ext_info;

out:
	return ret;
}
#else
static int apply_orientation(struct iio_dev *indio_dev,
				  struct iio_chan_spec *channels)
{
	return 0;
}
#endif

/*
 * st_gyro_get_settings() - get sensor settings from device name
 * @name: device name buffer reference.
 *
 * Return: valid reference on success, NULL otherwise.
 */
const struct st_sensor_settings *st_gyro_get_settings(const char *name)
{
	int index = st_sensors_get_settings_index(name,
					st_gyro_sensors_settings,
					ARRAY_SIZE(st_gyro_sensors_settings));
	if (index < 0)
		return NULL;

	return &st_gyro_sensors_settings[index];
}
EXPORT_SYMBOL(st_gyro_get_settings);

int st_gyro_common_probe(struct iio_dev *indio_dev)
{
	struct st_sensor_data *gdata = iio_priv(indio_dev);
	struct st_sensors_platform_data *pdata;
	int err;

	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &gyro_info;

	err = st_sensors_power_enable(indio_dev);
	if (err)
		return err;

	err = st_sensors_verify_id(indio_dev);
	if (err < 0)
		goto st_gyro_power_off;

	gdata->num_data_channels = ST_GYRO_NUMBER_DATA_CHANNELS;
	indio_dev->channels = gdata->sensor_settings->ch;
	indio_dev->num_channels = ST_SENSORS_NUMBER_ALL_CHANNELS;

	gdata->current_fullscale = &gdata->sensor_settings->fs.fs_avl[0];
	gdata->odr = gdata->sensor_settings->odr.odr_avl[0].hz;

	if (apply_orientation(indio_dev, gdata->sensor_settings->ch))
		dev_warn(&indio_dev->dev,
			 "failed to apply orientation data: %d\n", err);

	pdata = (struct st_sensors_platform_data *)&gyro_pdata;

	err = st_sensors_init_sensor(indio_dev, pdata);
	if (err < 0)
		goto st_gyro_power_off;

	err = st_gyro_allocate_ring(indio_dev);
	if (err < 0)
		goto st_gyro_power_off;

	if (gdata->irq > 0) {
		err = st_sensors_allocate_trigger(indio_dev,
						  ST_GYRO_TRIGGER_OPS);
		if (err < 0)
			goto st_gyro_probe_trigger_error;
	}

	err = iio_device_register(indio_dev);
	if (err)
		goto st_gyro_device_register_error;

	dev_info(&indio_dev->dev, "registered gyroscope %s\n",
		 indio_dev->name);

	return 0;

st_gyro_device_register_error:
	if (gdata->irq > 0)
		st_sensors_deallocate_trigger(indio_dev);
st_gyro_probe_trigger_error:
	st_gyro_deallocate_ring(indio_dev);
st_gyro_power_off:
	st_sensors_power_disable(indio_dev);

	return err;
}
EXPORT_SYMBOL(st_gyro_common_probe);

void st_gyro_common_remove(struct iio_dev *indio_dev)
{
	struct st_sensor_data *gdata = iio_priv(indio_dev);

	st_sensors_power_disable(indio_dev);

	iio_device_unregister(indio_dev);
	if (gdata->irq > 0)
		st_sensors_deallocate_trigger(indio_dev);

	st_gyro_deallocate_ring(indio_dev);
}
EXPORT_SYMBOL(st_gyro_common_remove);

MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics gyroscopes driver");
MODULE_LICENSE("GPL v2");
