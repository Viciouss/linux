// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2014 Broadcom Corporation
 */
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_net.h>
#include <linux/mmc/sdio_func.h>

#include <defs.h>
#include "debug.h"
#include "core.h"
#include "common.h"
#include "of.h"

static int brcmf_of_get_country_codes(struct device *dev,
				      struct brcmf_mp_device *settings)
{
	struct device_node *np = dev->of_node;
	struct brcmfmac_pd_cc_entry *cce;
	struct brcmfmac_pd_cc *cc;
	int count;
	int i;

	count = of_property_count_strings(np, "brcm,ccode-map");
	if (count < 0) {
		/* If no explicit country code map is specified, check whether
		 * the trivial map should be used.
		 */
		settings->trivial_ccode_map =
			of_property_read_bool(np, "brcm,ccode-map-trivial");

		/* The property is optional, so return success if it doesn't
		 * exist. Otherwise propagate the error code.
		 */
		return (count == -EINVAL) ? 0 : count;
	}

	cc = devm_kzalloc(dev, struct_size(cc, table, count), GFP_KERNEL);
	if (!cc)
		return -ENOMEM;

	cc->table_size = count;

	for (i = 0; i < count; i++) {
		const char *map;

		cce = &cc->table[i];

		if (of_property_read_string_index(np, "brcm,ccode-map",
						  i, &map))
			continue;

		/* String format e.g. US-Q2-86 */
		if (sscanf(map, "%2c-%2c-%d", cce->iso3166, cce->cc,
			   &cce->rev) != 3)
			brcmf_err("failed to read country map %s\n", map);
		else
			brcmf_dbg(INFO, "%s-%s-%d\n", cce->iso3166, cce->cc,
				  cce->rev);
	}

	settings->country_codes = cc;

	return 0;
}

/* Vendor tuple holding the module vendor ID */
#define BRCMF_CIS_TUPLE_START	0x80
#define BRCMF_CIS_TAG_VENDOR	0x81

static void brcmf_of_get_module_board_type(struct device *dev,
					   struct brcmf_mp_device *settings)
{
	struct sdio_func *func = dev_to_sdio_func(dev);
	struct device_node *np = dev->of_node;
	struct sdio_func_tuple *tpl;
	const char *name;
	const u8 *ids;
	int count, len, id_len, i;

	count = of_property_count_strings(np, "brcm,module-names");
	ids = of_get_property(np, "brcm,module-ids", &len);
	if (count <= 0 || !ids || !settings->board_type)
		return;

	if (len % count) {
		brcmf_err("brcm,module-ids does not match brcm,module-names\n");
		return;
	}

	for (tpl = func->tuples; tpl; tpl = tpl->next)
		if (tpl->code == BRCMF_CIS_TUPLE_START && tpl->size > 1 &&
		    tpl->data[0] == BRCMF_CIS_TAG_VENDOR)
			break;
	if (!tpl) {
		brcmf_info("no module vendor ID in CIS\n");
		return;
	}

	id_len = tpl->size - 1;
	brcmf_info("module vendor ID %*ph\n", id_len, &tpl->data[1]);
	if (len / count != id_len)
		return;

	for (i = 0; i < count; i++)
		if (!memcmp(ids + i * id_len, &tpl->data[1], id_len))
			break;
	if (i == count ||
	    of_property_read_string_index(np, "brcm,module-names", i, &name))
		return;

	settings->module_board_type = devm_kasprintf(dev, GFP_KERNEL, "%s.%s",
						     settings->board_type,
						     name);
	brcmf_info("module %s, board type %s\n", name,
		   settings->module_board_type);
}

void brcmf_of_probe(struct device *dev, enum brcmf_bus_type bus_type,
		    struct brcmf_mp_device *settings)
{
	struct brcmfmac_sdio_pd *sdio = &settings->bus.sdio;
	struct device_node *root, *np = dev->of_node;
	const char *prop;
	int irq;
	int err;
	u32 irqf;
	u32 val;

	/* Apple ARM64 platforms have their own idea of board type, passed in
	 * via the device tree. They also have an antenna SKU parameter
	 */
	err = of_property_read_string(np, "brcm,board-type", &prop);
	if (!err)
		settings->board_type = prop;

	if (!of_property_read_string(np, "apple,antenna-sku", &prop))
		settings->antenna_sku = prop;

	/* The WLAN calibration blob is normally stored in SROM, but Apple
	 * ARM64 platforms pass it via the DT instead.
	 */
	prop = of_get_property(np, "brcm,cal-blob", &settings->cal_size);
	if (prop && settings->cal_size)
		settings->cal_blob = prop;

	/* Set board-type to the first string of the machine compatible prop */
	root = of_find_node_by_path("/");
	if (root && err) {
		char *board_type = NULL;
		const char *tmp;

		/* get rid of '/' in the compatible string to be able to find the FW */
		if (!of_property_read_string_index(root, "compatible", 0, &tmp))
			board_type = devm_kstrdup(dev, tmp, GFP_KERNEL);

		if (!board_type) {
			of_node_put(root);
			return;
		}
		strreplace(board_type, '/', '-');
		settings->board_type = board_type;
	}
	of_node_put(root);

	if (!np || !of_device_is_compatible(np, "brcm,bcm4329-fmac"))
		return;

	err = brcmf_of_get_country_codes(dev, settings);
	if (err)
		brcmf_err("failed to get OF country code map (err=%d)\n", err);

	of_get_mac_address(np, settings->mac);

	if (bus_type != BRCMF_BUSTYPE_SDIO)
		return;

	brcmf_of_get_module_board_type(dev, settings);

	if (of_property_read_u32(np, "brcm,drive-strength", &val) == 0)
		sdio->drive_strength = val;

	/* make sure there are interrupts defined in the node */
	if (!of_property_present(np, "interrupts"))
		return;

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		brcmf_err("interrupt could not be mapped\n");
		return;
	}
	irqf = irqd_get_trigger_type(irq_get_irq_data(irq));

	sdio->oob_irq_supported = true;
	sdio->oob_irq_nr = irq;
	sdio->oob_irq_flags = irqf;
}
