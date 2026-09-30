// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics sensors buffer library driver
 *
 * Copyright 2012-2013 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 */

#include <linux/kernel.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/interrupt.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/irqreturn.h>
#include <linux/regmap.h>

#include <linux/iio/common/st_sensors.h>


/* Read @len bytes starting at register @addr into @dst; no-op if @len is 0 */
static int st_sensors_read_burst(struct st_sensor_data *sdata,
				 unsigned int addr, u8 *dst, unsigned int len)
{
	if (len && regmap_bulk_read(sdata->regmap, addr, dst, len) < 0)
		return -EIO;

	return 0;
}

static int st_sensors_get_buffer_element(struct iio_dev *indio_dev, u8 *buf)
{
	struct st_sensor_data *sdata = iio_priv(indio_dev);
	unsigned int num_data_channels = sdata->num_data_channels;
	/* Pending burst: burst_len bytes from burst_addr into burst_dst */
	unsigned int burst_addr = 0, burst_len = 0;
	u8 *burst_dst = NULL;
	int i, err;

	for_each_set_bit(i, indio_dev->active_scan_mask, num_data_channels) {
		const struct iio_chan_spec *channel = &indio_dev->channels[i];
		unsigned int bytes_to_read =
			DIV_ROUND_UP(channel->scan_type.realbits +
				     channel->scan_type.shift, 8);
		unsigned int storage_bytes =
			channel->scan_type.storagebits >> 3;
		bool extends_burst;

		buf = PTR_ALIGN(buf, storage_bytes);

		/*
		 * A channel joins the pending burst if its registers directly
		 * follow the burst's registers and its data directly follows
		 * the burst's data in the scan buffer. X/Y/Z then become one
		 * transfer instead of three.
		 */
		extends_burst = burst_len &&
				channel->address == burst_addr + burst_len &&
				buf == burst_dst + burst_len;

		if (!extends_burst) {
			err = st_sensors_read_burst(sdata, burst_addr,
						    burst_dst, burst_len);
			if (err)
				return err;

			burst_addr = channel->address;
			burst_dst = buf;
			burst_len = 0;
		}

		burst_len += bytes_to_read;

		/* Advance the buffer pointer */
		buf += storage_bytes;
	}

	return st_sensors_read_burst(sdata, burst_addr, burst_dst, burst_len);
}

irqreturn_t st_sensors_trigger_handler(int irq, void *p)
{
	int len;
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct st_sensor_data *sdata = iio_priv(indio_dev);
	s64 timestamp;

	/*
	 * If we do timestamping here, do it before reading the values, because
	 * once we've read the values, new interrupts can occur (when using
	 * the hardware trigger) and the hw_timestamp may get updated.
	 * By storing it in a local variable first, we are safe.
	 */
	if (iio_trigger_using_own(indio_dev))
		timestamp = sdata->hw_timestamp;
	else
		timestamp = iio_get_time_ns(indio_dev);

	len = st_sensors_get_buffer_element(indio_dev, sdata->buffer_data);
	if (len < 0)
		goto st_sensors_get_buffer_element_error;

	iio_push_to_buffers_with_timestamp(indio_dev, sdata->buffer_data,
					   timestamp);

st_sensors_get_buffer_element_error:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}
EXPORT_SYMBOL_NS(st_sensors_trigger_handler, IIO_ST_SENSORS);
