// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Copyright (C) 2020 Dimitar Gamishev <hehopmajieh@debian.bg>
 * Copyright (C) 2020 BayLibre, SAS
 * Author: Phong LE <ple@baylibre.com>
 * Copyright (C) 2018-2019, Artem Mygaiev
 * Copyright (C) 2017, Fresco Logic, Incorporated.
 *
 */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_modes.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#define IT66121_MASTER_SEL_REG			0x10
#define IT66121_MASTER_SEL_HOST			BIT(0)

#define IT66121_AFE_DRV_REG			0x61
#define IT66121_AFE_DRV_RST			BIT(4)
#define IT66121_AFE_DRV_PWD			BIT(5)

#define IT66121_INPUT_MODE_REG			0x70
#define IT66121_INPUT_MODE_RGB			(0 << 6)
#define IT66121_INPUT_MODE_YUV422		BIT(6)
#define IT66121_INPUT_MODE_YUV444		(2 << 6)
#define IT66121_INPUT_MODE_CCIR656		BIT(4)
#define IT66121_INPUT_MODE_SYNCEMB		BIT(3)
#define IT66121_INPUT_MODE_DDR			BIT(2)

#define IT66121_INPUT_CSC_REG			0x72
#define IT66121_INPUT_CSC_ENDITHER		BIT(7)
#define IT66121_INPUT_CSC_ENUDFILTER		BIT(6)
#define IT66121_INPUT_CSC_DNFREE_GO		BIT(5)
#define IT66121_INPUT_CSC_RGB_TO_YUV		0x02
#define IT66121_INPUT_CSC_YUV_TO_RGB		0x03
#define IT66121_INPUT_CSC_NO_CONV		0x00

#define IT66121_AFE_XP_REG			0x62
#define IT66121_AFE_XP_GAINBIT			BIT(7)
#define IT66121_AFE_XP_PWDPLL			BIT(6)
#define IT66121_AFE_XP_ENI			BIT(5)
#define IT66121_AFE_XP_ENO			BIT(4)
#define IT66121_AFE_XP_RESETB			BIT(3)
#define IT66121_AFE_XP_PWDI			BIT(2)

#define IT66121_AFE_IP_REG			0x64
#define IT66121_AFE_IP_GAINBIT			BIT(7)
#define IT66121_AFE_IP_PWDPLL			BIT(6)
#define IT66121_AFE_IP_CKSEL_05			(0 << 4)
#define IT66121_AFE_IP_CKSEL_1			BIT(4)
#define IT66121_AFE_IP_CKSEL_2			(2 << 4)
#define IT66121_AFE_IP_CKSEL_2OR4		(3 << 4)
#define IT66121_AFE_IP_ER0			BIT(3)
#define IT66121_AFE_IP_RESETB			BIT(2)
#define IT66121_AFE_IP_ENC			BIT(1)
#define IT66121_AFE_IP_EC1			BIT(0)

#define IT66121_AFE_XP_EC1_REG			0x68
#define IT66121_AFE_XP_EC1_LOWCLK		BIT(4)

#define IT66121_SW_RST_REG			0x04
#define IT66121_SW_RST_REF			BIT(5)
#define IT66121_SW_RST_AREF			BIT(4)
#define IT66121_SW_RST_VID			BIT(3)
#define IT66121_SW_RST_AUD			BIT(2)
#define IT66121_SW_RST_HDCP			BIT(0)

#define IT66121_DDC_COMMAND_REG			0x15
#define IT66121_DDC_COMMAND_BURST_READ		0x0
#define IT66121_DDC_COMMAND_EDID_READ		0x3
#define IT66121_DDC_COMMAND_FIFO_CLR		0x9
#define IT66121_DDC_COMMAND_SCL_PULSE		0xA
#define IT66121_DDC_COMMAND_ABORT		0xF

#define IT66121_HDCP_REG			0x20
#define IT66121_HDCP_CPDESIRED			BIT(0)
#define IT66121_HDCP_EN1P1FEAT			BIT(1)

#define IT66121_INT_STATUS1_REG			0x06
#define IT66121_INT_STATUS1_AUD_OVF		BIT(7)
#define IT66121_INT_STATUS1_DDC_NOACK		BIT(5)
#define IT66121_INT_STATUS1_DDC_FIFOERR		BIT(4)
#define IT66121_INT_STATUS1_DDC_BUSHANG		BIT(2)
#define IT66121_INT_STATUS1_RX_SENS_STATUS	BIT(1)
#define IT66121_INT_STATUS1_HPD_STATUS		BIT(0)

#define IT66121_DDC_HEADER_REG			0x11
#define IT66121_DDC_HEADER_HDCP			0x74
#define IT66121_DDC_HEADER_EDID			0xA0

#define IT66121_DDC_OFFSET_REG			0x12
#define IT66121_DDC_BYTE_REG			0x13
#define IT66121_DDC_SEGMENT_REG			0x14
#define IT66121_DDC_RD_FIFO_REG			0x17

#define IT66121_CLK_BANK_REG			0x0F
#define IT66121_CLK_BANK_PWROFF_RCLK		BIT(6)
#define IT66121_CLK_BANK_PWROFF_ACLK		BIT(5)
#define IT66121_CLK_BANK_PWROFF_TXCLK		BIT(4)
#define IT66121_CLK_BANK_PWROFF_CRCLK		BIT(3)
#define IT66121_CLK_BANK_0			0
#define IT66121_CLK_BANK_1			1

#define IT66121_INT_REG				0x05
#define IT66121_INT_ACTIVE_HIGH			BIT(7)
#define IT66121_INT_OPEN_DRAIN			BIT(6)
#define IT66121_INT_TX_CLK_OFF			BIT(0)

#define IT66121_INT_MASK1_REG			0x09
#define IT66121_INT_MASK1_AUD_OVF		BIT(7)
#define IT66121_INT_MASK1_DDC_NOACK		BIT(5)
#define IT66121_INT_MASK1_DDC_FIFOERR		BIT(4)
#define IT66121_INT_MASK1_DDC_BUSHANG		BIT(2)
#define IT66121_INT_MASK1_RX_SENS		BIT(1)
#define IT66121_INT_MASK1_HPD			BIT(0)

#define IT66121_INT_CLR1_REG			0x0C
#define IT66121_INT_CLR1_PKTACP			BIT(7)
#define IT66121_INT_CLR1_PKTNULL		BIT(6)
#define IT66121_INT_CLR1_PKTGEN			BIT(5)
#define IT66121_INT_CLR1_KSVLISTCHK		BIT(4)
#define IT66121_INT_CLR1_AUTHDONE		BIT(3)
#define IT66121_INT_CLR1_AUTHFAIL		BIT(2)
#define IT66121_INT_CLR1_RX_SENS		BIT(1)
#define IT66121_INT_CLR1_HPD			BIT(0)

#define IT66121_AV_MUTE_REG			0xC1
#define IT66121_AV_MUTE_ON			BIT(0)
#define IT66121_AV_MUTE_BLUESCR			BIT(1)

#define IT66121_PKT_GEN_CTRL_REG		0xC6
#define IT66121_PKT_GEN_CTRL_ON			BIT(0)
#define IT66121_PKT_GEN_CTRL_RPT		BIT(1)

#define IT66121_AVIINFO_DB1_REG			0x158
#define IT66121_AVIINFO_DB2_REG			0x159
#define IT66121_AVIINFO_DB3_REG			0x15A
#define IT66121_AVIINFO_DB4_REG			0x15B
#define IT66121_AVIINFO_DB5_REG			0x15C
#define IT66121_AVIINFO_CSUM_REG		0x15D
#define IT66121_AVIINFO_DB6_REG			0x15E
#define IT66121_AVIINFO_DB7_REG			0x15F
#define IT66121_AVIINFO_DB8_REG			0x160
#define IT66121_AVIINFO_DB9_REG			0x161
#define IT66121_AVIINFO_DB10_REG		0x162
#define IT66121_AVIINFO_DB11_REG		0x163
#define IT66121_AVIINFO_DB12_REG		0x164
#define IT66121_AVIINFO_DB13_REG		0x165

#define IT66121_AVI_INFO_PKT_REG		0xCD
#define IT66121_AVI_INFO_PKT_ON			BIT(0)
#define IT66121_AVI_INFO_PKT_RPT		BIT(1)

#define IT66121_HDMI_MODE_REG			0xC0
#define IT66121_HDMI_MODE_HDMI			BIT(0)

#define IT66121_SYS_STATUS_REG			0x0E
#define IT66121_SYS_STATUS_ACTIVE_IRQ		BIT(7)
#define IT66121_SYS_STATUS_HPDETECT		BIT(6)
#define IT66121_SYS_STATUS_SENDECTECT		BIT(5)
#define IT66121_SYS_STATUS_VID_STABLE		BIT(4)
#define IT66121_SYS_STATUS_AUD_CTS_CLR		BIT(1)
#define IT66121_SYS_STATUS_CLEAR_IRQ		BIT(0)

#define IT66121_DDC_STATUS_REG			0x16
#define IT66121_DDC_STATUS_TX_DONE		BIT(7)
#define IT66121_DDC_STATUS_ACTIVE		BIT(6)
#define IT66121_DDC_STATUS_NOACK		BIT(5)
#define IT66121_DDC_STATUS_WAIT_BUS		BIT(4)
#define IT66121_DDC_STATUS_ARBI_LOSE		BIT(3)
#define IT66121_DDC_STATUS_FIFO_FULL		BIT(2)
#define IT66121_DDC_STATUS_FIFO_EMPTY		BIT(1)
#define IT66121_DDC_STATUS_FIFO_VALID		BIT(0)

#define IT66121_VENDOR_ID0			0x54
#define IT66121_VENDOR_ID1			0x49
#define IT66121_DEVICE_ID0			0x12
#define IT66121_DEVICE_ID1			0x06
#define IT66121_DEVICE_MASK			0x0F
#define IT66121_EDID_SLEEP			20000
#define IT66121_EDID_TIMEOUT			200000
#define IT66121_EDID_FIFO_SIZE			32
#define IT66121_AFE_CLK_HIGH			80000

struct it66121_conf {
	unsigned int input_mode_reg;
	unsigned int input_conversion_reg;
};

struct it66121_ctx {
	struct regmap *regmap;
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct device *dev;
	struct gpio_desc *gpio_reset;
	struct i2c_client *client;
	struct regulator_bulk_data supplies[3];
	bool dual_edge;
	const struct it66121_conf *conf;
	struct mutex lock; /* Protects fields below and device registers */
	struct edid *edid;
	struct hdmi_avi_infoframe hdmi_avi_infoframe;
};

static const struct regmap_range_cfg it66121_regmap_banks[] = {
	{
		.name = "it66121",
		.range_min = 0x00,
		.range_max = 0x1FF,
		.selector_reg = IT66121_CLK_BANK_REG,
		.selector_mask = 0x1,
		.selector_shift = 0,
		.window_start = 0x00,
		.window_len = 0x130,
	},
};

static const struct regmap_config it66121_regmap_config = {
	.val_bits = 8,
	.reg_bits = 8,
	.max_register = 0x1FF,
	.ranges = it66121_regmap_banks,
	.num_ranges = ARRAY_SIZE(it66121_regmap_banks),
};

static const struct it66121_conf it66121_conf_simple = {
//	.input_mode_reg = IT66121_INPUT_MODE_RGB | IT66121_INPUT_MODE_DDR,
	.input_mode_reg = IT66121_INPUT_MODE_RGB,
	.input_conversion_reg = IT66121_INPUT_CSC_NO_CONV,
};

static void it66121_hw_reset(struct it66121_ctx *ctx)
{
	gpiod_set_value(ctx->gpio_reset, 1);
	msleep(20);
	gpiod_set_value(ctx->gpio_reset, 0);
}

static int ite66121_power_on(struct it66121_ctx *ctx)
{
	return regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
}

static int ite66121_power_off(struct it66121_ctx *ctx)
{
	return regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
}

static int it66121_preamble_ddc(struct it66121_ctx *ctx)
{
	return regmap_write(ctx->regmap, IT66121_MASTER_SEL_REG,
				IT66121_MASTER_SEL_HOST);
}

static int it66121_fire_afe(struct it66121_ctx *ctx)
{
	return regmap_write(ctx->regmap, IT66121_AFE_DRV_REG, 0);
}

static int it66121_configure_input(struct it66121_ctx *ctx)
{
	int ret;

	ret = regmap_write(ctx->regmap, IT66121_INPUT_MODE_REG,
			   ctx->conf->input_mode_reg);
	if (ret)
		return ret;

	return regmap_write(ctx->regmap, IT66121_INPUT_CSC_REG,
			    ctx->conf->input_conversion_reg);
}

/**
 * it66121_configure_afe() - Configure the analog front end
 * @ctx: it66121_ctx object
 *
 * RETURNS:
 * zero if success, a negative error code otherwise.
 */
static int it66121_configure_afe(struct it66121_ctx *ctx,
				 const struct drm_display_mode *mode)
{
	int ret;

	ret = regmap_write(ctx->regmap, IT66121_AFE_DRV_REG,
			   IT66121_AFE_DRV_RST);
	if (ret)
		return ret;

	if (mode->clock > IT66121_AFE_CLK_HIGH) {
		ret = regmap_write_bits(ctx->regmap, IT66121_AFE_XP_REG,
					IT66121_AFE_XP_GAINBIT |
					IT66121_AFE_XP_ENO,
					IT66121_AFE_XP_GAINBIT);
		if (ret)
			return ret;

		ret = regmap_write_bits(ctx->regmap, IT66121_AFE_IP_REG,
					IT66121_AFE_IP_GAINBIT |
					IT66121_AFE_IP_ER0 |
					IT66121_AFE_IP_EC1,
					IT66121_AFE_IP_GAINBIT);
		if (ret)
			return ret;

		ret = regmap_write_bits(ctx->regmap, IT66121_AFE_XP_EC1_REG,
					IT66121_AFE_XP_EC1_LOWCLK, 0x80);
		if (ret)
			return ret;
	} else {
		ret = regmap_write_bits(ctx->regmap, IT66121_AFE_XP_REG,
					IT66121_AFE_XP_GAINBIT |
					IT66121_AFE_XP_ENO,
					IT66121_AFE_XP_ENO);
		if (ret)
			return ret;

		ret = regmap_write_bits(ctx->regmap, IT66121_AFE_IP_REG,
					IT66121_AFE_IP_GAINBIT |
					IT66121_AFE_IP_ER0 |
					IT66121_AFE_IP_EC1, IT66121_AFE_IP_ER0 |
					IT66121_AFE_IP_EC1);
		if (ret)
			return ret;

		ret = regmap_write_bits(ctx->regmap, IT66121_AFE_XP_EC1_REG,
					IT66121_AFE_XP_EC1_LOWCLK,
					IT66121_AFE_XP_EC1_LOWCLK);
		if (ret)
			return ret;
	}

	/* Clear reset flags */
	ret = regmap_write_bits(ctx->regmap, IT66121_SW_RST_REG,
				IT66121_SW_RST_REF | IT66121_SW_RST_VID,
				~(IT66121_SW_RST_REF | IT66121_SW_RST_VID) &
				0xFF);
	if (ret)
		return ret;

	return it66121_fire_afe(ctx);
}

static inline int it66121_wait_ddc_ready(struct it66121_ctx *ctx)
{
	int ret, val;

	ret = regmap_read_poll_timeout(ctx->regmap, IT66121_DDC_STATUS_REG,
				       val, true,
				       IT66121_EDID_SLEEP,
				       IT66121_EDID_TIMEOUT);
	if (ret)
		return ret;

	if (val & (IT66121_DDC_STATUS_NOACK | IT66121_DDC_STATUS_WAIT_BUS |
	    IT66121_DDC_STATUS_ARBI_LOSE))
		return -EAGAIN;

	return 0;
}

static int it66121_clear_ddc_fifo(struct it66121_ctx *ctx)
{
	int ret;

	ret = it66121_preamble_ddc(ctx);
	if (ret)
		return ret;

	return regmap_write(ctx->regmap, IT66121_DDC_COMMAND_REG,
			    IT66121_DDC_COMMAND_FIFO_CLR);
}

static int it66121_abort_ddc_ops(struct it66121_ctx *ctx)
{
	int ret;
	unsigned int swreset, cpdesire;

	ret = regmap_read(ctx->regmap, IT66121_SW_RST_REG, &swreset);
	if (ret)
		return ret;

	ret = regmap_read(ctx->regmap, IT66121_HDCP_REG, &cpdesire);
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, IT66121_HDCP_REG,
			   cpdesire & (~IT66121_HDCP_CPDESIRED & 0xFF));
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, IT66121_SW_RST_REG,
			   swreset | IT66121_SW_RST_HDCP);
	if (ret)
		return ret;

	ret = it66121_preamble_ddc(ctx);
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, IT66121_DDC_COMMAND_REG,
			   IT66121_DDC_COMMAND_ABORT);
	if (ret)
		return ret;

	return it66121_wait_ddc_ready(ctx);
}

static int it66121_get_edid_block(void *context, u8 *buf,
				  unsigned int block, size_t len)
{
	struct it66121_ctx *ctx = context;
	unsigned int val;
	int remain = len;
	int offset = 0;
	int ret, cnt;

	offset = (block % 2) * len;
	block = block / 2;

	ret = regmap_read(ctx->regmap, IT66121_INT_STATUS1_REG, &val);
	if (ret)
		return ret;

	if (val & IT66121_INT_STATUS1_DDC_BUSHANG) {
		ret = it66121_abort_ddc_ops(ctx);
		if (ret)
			return ret;
	}

	ret = it66121_clear_ddc_fifo(ctx);
	if (ret)
		return ret;

	while (remain > 0) {
		cnt = (remain > IT66121_EDID_FIFO_SIZE) ?
				IT66121_EDID_FIFO_SIZE : remain;
		ret = it66121_preamble_ddc(ctx);
		if (ret)
			return ret;

		ret = regmap_write(ctx->regmap, IT66121_DDC_COMMAND_REG,
				   IT66121_DDC_COMMAND_FIFO_CLR);
		if (ret)
			return ret;

		ret = it66121_wait_ddc_ready(ctx);
		if (ret)
			return ret;

		ret = regmap_read(ctx->regmap, IT66121_INT_STATUS1_REG, &val);
		if (ret)
			return ret;

		if (val & IT66121_INT_STATUS1_DDC_BUSHANG) {
			ret = it66121_abort_ddc_ops(ctx);
			if (ret)
				return ret;
		}

		ret = it66121_preamble_ddc(ctx);
		if (ret)
			return ret;

		ret = regmap_write(ctx->regmap, IT66121_DDC_HEADER_REG,
				   IT66121_DDC_HEADER_EDID);
		if (ret)
			return ret;

		ret = regmap_write(ctx->regmap, IT66121_DDC_OFFSET_REG, offset);
		if (ret)
			return ret;

		ret = regmap_write(ctx->regmap, IT66121_DDC_BYTE_REG, cnt);
		if (ret)
			return ret;

		ret = regmap_write(ctx->regmap, IT66121_DDC_SEGMENT_REG, block);
		if (ret)
			return ret;

		ret = regmap_write(ctx->regmap, IT66121_DDC_COMMAND_REG,
				   IT66121_DDC_COMMAND_EDID_READ);
		if (ret)
			return ret;

		offset += cnt;
		remain -= cnt;
		msleep(20);

		ret = it66121_wait_ddc_ready(ctx);
		if (ret)
			return ret;

		do {
			ret = regmap_read(ctx->regmap,
					  IT66121_DDC_RD_FIFO_REG, &val);
			if (ret)
				return ret;
			*(buf++) = val;
			cnt--;
		} while (cnt > 0);
	}

	return 0;
}

static int it66121_connector_get_modes(struct drm_connector *connector)
{
	int ret, num_modes = 0;
	struct it66121_ctx *ctx = container_of(connector, struct it66121_ctx,
			connector);

	if (ctx->edid)
		return drm_add_edid_modes(connector, ctx->edid);

	mutex_lock(&ctx->lock);

	ctx->edid = drm_do_get_edid(connector, it66121_get_edid_block, ctx);
	if (!ctx->edid) {
		DRM_ERROR("Failed to read EDID\n");
		goto unlock;
	}

	ret = drm_connector_update_edid_property(connector,
						 ctx->edid);
	if (ret) {
		DRM_ERROR("Failed to update EDID property: %d\n", ret);
		goto unlock;
	}

	num_modes = drm_add_edid_modes(connector, ctx->edid);

unlock:
	mutex_unlock(&ctx->lock);

	return num_modes;
}

static bool it66121_is_hpd_detect(struct it66121_ctx *ctx)
{
	int val;

	if (regmap_read(ctx->regmap, IT66121_SYS_STATUS_REG, &val))
		return false;

	return (val & IT66121_SYS_STATUS_HPDETECT);
}

static int it66121_connector_detect_ctx(struct drm_connector *connector,
					struct drm_modeset_acquire_ctx *c,
					bool force)
{
	struct it66121_ctx *ctx = container_of(connector, struct it66121_ctx,
			connector);

	return (it66121_is_hpd_detect(ctx)) ?
		connector_status_connected : connector_status_disconnected;
}

static enum drm_mode_status
it66121_connector_mode_valid(struct drm_connector *connector,
			     struct drm_display_mode *mode)
{
	unsigned long max_clock;
	struct it66121_ctx *ctx = container_of(connector, struct it66121_ctx,
			connector);

	max_clock = ctx->dual_edge ? 74250 : 148500;

	if (mode->clock > max_clock)
		return MODE_CLOCK_HIGH;

	if (mode->clock < 25000)
		return MODE_CLOCK_LOW;

	return MODE_OK;
}

static struct drm_connector_helper_funcs it66121_connector_helper_funcs = {
	.get_modes = it66121_connector_get_modes,
	.detect_ctx = it66121_connector_detect_ctx,
	.mode_valid = it66121_connector_mode_valid,
};

static const struct drm_connector_funcs it66121_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int it66121_bridge_attach(struct drm_bridge *bridge)
{
	int ret;
	struct it66121_ctx *ctx = container_of(bridge, struct it66121_ctx,
			bridge);
#if 0
	if (flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR) {
		DRM_ERROR("Fix bridge driver to make connector optional!");
		return -EINVAL;
	}
#endif
	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found");
		return -ENODEV;
	}

	ret = regmap_write_bits(ctx->regmap, IT66121_CLK_BANK_REG,
				IT66121_CLK_BANK_PWROFF_RCLK, 0);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_INT_REG,
				IT66121_INT_TX_CLK_OFF, 0);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_AFE_DRV_REG,
				IT66121_AFE_DRV_PWD, 0);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_AFE_XP_REG,
				IT66121_AFE_XP_PWDI | IT66121_AFE_XP_PWDPLL, 0);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_AFE_IP_REG,
				IT66121_AFE_IP_PWDPLL, 0);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_AFE_DRV_REG,
				IT66121_AFE_DRV_RST, 0);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_AFE_XP_REG,
				IT66121_AFE_XP_RESETB, IT66121_AFE_XP_RESETB);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_AFE_IP_REG,
				IT66121_AFE_IP_RESETB, IT66121_AFE_IP_RESETB);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_SW_RST_REG,
				IT66121_SW_RST_REF,
				IT66121_SW_RST_REF);
	if (ret)
		return ret;

	msleep(50);

	ret = drm_connector_init(bridge->dev, &ctx->connector,
				 &it66121_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret)
		return ret;

	ctx->connector.polled = DRM_CONNECTOR_POLL_HPD;
	drm_connector_helper_add(&ctx->connector,
				 &it66121_connector_helper_funcs);

	ret = drm_connector_attach_encoder(&ctx->connector, bridge->encoder);
	if (ret)
		return ret;

	ret = drm_connector_register(&ctx->connector);
	if (ret)
		return ret;

	/* Start interrupts */
	return regmap_write_bits(ctx->regmap, IT66121_INT_MASK1_REG,
				 IT66121_INT_MASK1_DDC_NOACK |
				 IT66121_INT_MASK1_HPD |
				 IT66121_INT_MASK1_DDC_FIFOERR |
				 IT66121_INT_MASK1_DDC_BUSHANG,
				 ~(IT66121_INT_MASK1_DDC_NOACK |
				 IT66121_INT_MASK1_HPD |
				 IT66121_INT_MASK1_DDC_FIFOERR |
				 IT66121_INT_MASK1_DDC_BUSHANG) & 0xFF);
}

static int it66121_set_mute(struct it66121_ctx *ctx, bool mute)
{
	int ret;
	unsigned int val;

	val = mute ? IT66121_AV_MUTE_ON : (~IT66121_AV_MUTE_ON & 0xFF);
	ret = regmap_write_bits(ctx->regmap, IT66121_AV_MUTE_REG,
				IT66121_AV_MUTE_ON, val);
	if (ret)
		return ret;

	return regmap_write(ctx->regmap, IT66121_PKT_GEN_CTRL_REG,
			    IT66121_PKT_GEN_CTRL_ON |
			    IT66121_PKT_GEN_CTRL_RPT);
}

static void it66121_bridge_enable(struct drm_bridge *bridge)
{
	struct it66121_ctx *ctx = container_of(bridge, struct it66121_ctx,
			bridge);

	it66121_set_mute(ctx, false);
}

static void it66121_bridge_disable(struct drm_bridge *bridge)
{
	struct it66121_ctx *ctx = container_of(bridge, struct it66121_ctx,
			bridge);

	it66121_set_mute(ctx, true);
}

static
void it66121_bridge_mode_set(struct drm_bridge *bridge,
			     const struct drm_display_mode *mode,
			     const struct drm_display_mode *adjusted_mode)
{
	int ret, i;
	u8 buf[HDMI_INFOFRAME_SIZE(AVI)];
	struct it66121_ctx *ctx = container_of(bridge, struct it66121_ctx,
			bridge);
	const u16 aviinfo_reg[HDMI_AVI_INFOFRAME_SIZE] = {
		IT66121_AVIINFO_DB1_REG,
		IT66121_AVIINFO_DB2_REG,
		IT66121_AVIINFO_DB3_REG,
		IT66121_AVIINFO_DB4_REG,
		IT66121_AVIINFO_DB5_REG,
		IT66121_AVIINFO_DB6_REG,
		IT66121_AVIINFO_DB7_REG,
		IT66121_AVIINFO_DB8_REG,
		IT66121_AVIINFO_DB9_REG,
		IT66121_AVIINFO_DB10_REG,
		IT66121_AVIINFO_DB11_REG,
		IT66121_AVIINFO_DB12_REG,
		IT66121_AVIINFO_DB13_REG
	};

	mutex_lock(&ctx->lock);

	hdmi_avi_infoframe_init(&ctx->hdmi_avi_infoframe);

	ret = drm_hdmi_avi_infoframe_from_display_mode(&ctx->hdmi_avi_infoframe,
						       &ctx->connector,
						       adjusted_mode);
	if (ret) {
		DRM_ERROR("Failed to setup AVI infoframe: %d\n", ret);
		goto unlock;
	}

	ret = hdmi_avi_infoframe_pack(&ctx->hdmi_avi_infoframe, buf,
				      sizeof(buf));
	if (ret < 0) {
		DRM_ERROR("Failed to pack infoframe: %d\n", ret);
		goto unlock;
	}

	/* Write new AVI infoframe packet */
	for (i = 0; i < HDMI_AVI_INFOFRAME_SIZE; i++) {
		if (regmap_write(ctx->regmap, aviinfo_reg[i],
				 buf[i + HDMI_INFOFRAME_HEADER_SIZE]))
			goto unlock;
	}
	if (regmap_write(ctx->regmap, IT66121_AVIINFO_CSUM_REG, buf[3]))
		goto unlock;

	/* Enable AVI infoframe */
	if (regmap_write(ctx->regmap, IT66121_AVI_INFO_PKT_REG,
			 IT66121_AVI_INFO_PKT_ON |
			 IT66121_AVI_INFO_PKT_RPT))
		goto unlock;

	/* Set TX mode to HDMI */
	if (regmap_write(ctx->regmap, IT66121_HDMI_MODE_REG,
			 IT66121_HDMI_MODE_HDMI))
		goto unlock;

	if (regmap_write_bits(ctx->regmap, IT66121_CLK_BANK_REG,
			      IT66121_CLK_BANK_PWROFF_TXCLK,
			      IT66121_CLK_BANK_PWROFF_TXCLK))
		goto unlock;

	if (it66121_configure_input(ctx))
		goto unlock;

	if (it66121_configure_afe(ctx, adjusted_mode))
		goto unlock;

	regmap_write_bits(ctx->regmap, IT66121_CLK_BANK_REG,
			  IT66121_CLK_BANK_PWROFF_TXCLK,
			  ~IT66121_CLK_BANK_PWROFF_TXCLK & 0xFF);

unlock:
	mutex_unlock(&ctx->lock);
}

static const struct drm_bridge_funcs it66121_bridge_funcs = {
	.attach = it66121_bridge_attach,
	.enable = it66121_bridge_enable,
	.disable = it66121_bridge_disable,
	.mode_set = it66121_bridge_mode_set,
};

static irqreturn_t it66121_irq_threaded_handler(int irq, void *dev_id)
{
	int ret;
	unsigned int val;
	struct it66121_ctx *ctx = dev_id;
	struct device *dev = ctx->dev;
	bool event = false;

	mutex_lock(&ctx->lock);

	ret = regmap_read(ctx->regmap, IT66121_SYS_STATUS_REG, &val);
	if (ret)
		goto unlock;

	if (val & IT66121_SYS_STATUS_ACTIVE_IRQ) {
		ret = regmap_read(ctx->regmap, IT66121_INT_STATUS1_REG, &val);
		if (ret) {
			dev_err(dev, "Cannot read STATUS1_REG %d\n", ret);
		} else {
			if (val & IT66121_INT_STATUS1_DDC_FIFOERR)
				it66121_clear_ddc_fifo(ctx);
			if (val & (IT66121_INT_STATUS1_DDC_BUSHANG |
					IT66121_INT_STATUS1_DDC_NOACK))
				it66121_abort_ddc_ops(ctx);
			if (val & IT66121_INT_STATUS1_HPD_STATUS) {
				regmap_write_bits(ctx->regmap,
						  IT66121_INT_CLR1_REG,
						  IT66121_INT_CLR1_HPD,
						  IT66121_INT_CLR1_HPD);

				if (!it66121_is_hpd_detect(ctx)) {
					kfree(ctx->edid);
					ctx->edid = NULL;
				}
				event = true;
			}
		}

		regmap_write_bits(ctx->regmap, IT66121_SYS_STATUS_REG,
				  IT66121_SYS_STATUS_CLEAR_IRQ,
				  IT66121_SYS_STATUS_CLEAR_IRQ);
	}

unlock:
	mutex_unlock(&ctx->lock);

	if (event)
		drm_helper_hpd_irq_event(ctx->bridge.dev);

	return IRQ_HANDLED;
}

static int it66121_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	u8 ids[4];
	int i, ret;
	struct it66121_ctx *ctx;
	struct device *dev = &client->dev;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "I2C check functionality failed.\n");
		return -ENXIO;
	}

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	ctx->client = client;
	i2c_set_clientdata(client, ctx);
	mutex_init(&ctx->lock);
	ctx->conf = (struct it66121_conf *)of_device_get_match_data(dev);
	if (!ctx->conf)
		return -ENODEV;

	ctx->supplies[0].supply = "vcn33";
	ctx->supplies[1].supply = "vcn18";
	ctx->supplies[2].supply = "vrf12";
	ret = devm_regulator_bulk_get(ctx->dev, 3, ctx->supplies);
	if (ret) {
		dev_err(ctx->dev, "regulator_bulk failed\n");
		return ret;
	}

	ctx->dual_edge = of_property_read_bool(dev->of_node, "pclk-dual-edge");

	ret = ite66121_power_on(ctx);
	if (ret)
		return ret;

	it66121_hw_reset(ctx);

	ctx->regmap = devm_regmap_init_i2c(client, &it66121_regmap_config);
	if (IS_ERR(ctx->regmap)) {
		ite66121_power_off(ctx);
		return PTR_ERR(ctx);
	}

	for (i = 0; i < 4; i++) {
		regmap_read(ctx->regmap, i, &ret);
		ids[i] = ret;
	}

	if (ids[0] != IT66121_VENDOR_ID0 ||
	    ids[1] != IT66121_VENDOR_ID1 ||
	    ids[2] != IT66121_DEVICE_ID0 ||
	    ((ids[3] & IT66121_DEVICE_MASK) != IT66121_DEVICE_ID1)) {
		ite66121_power_off(ctx);
		return -ENODEV;
	}

	ctx->bridge.funcs = &it66121_bridge_funcs;
	ctx->bridge.of_node = dev->of_node;


	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					it66121_irq_threaded_handler,
					IRQF_SHARED | IRQF_TRIGGER_LOW |
					IRQF_ONESHOT,
					dev_name(dev),
					ctx);

	if (ret < 0) {
		dev_err(dev, "Failed to request irq %d:%d\n", client->irq, ret);
		ite66121_power_off(ctx);
		return ret;
	}

	drm_bridge_add(&ctx->bridge);

	return 0;
}

static int it66121_remove(struct i2c_client *client)
{
	struct it66121_ctx *ctx = i2c_get_clientdata(client);

	ite66121_power_off(ctx);
	drm_bridge_remove(&ctx->bridge);
	kfree(ctx->edid);
	mutex_destroy(&ctx->lock);

	return 0;
}

static const struct of_device_id it66121_dt_match[] = {
	{ .compatible = "ite,it66121",
	  .data = &it66121_conf_simple,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, it66121_dt_match);

static const struct i2c_device_id it66121_id[] = {
	{ "it66121", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, it66121_id);

static struct i2c_driver it66121_driver = {
	.driver = {
		.name	= "it66121",
		.of_match_table = it66121_dt_match,
	},
	.probe = it66121_probe,
	.remove = it66121_remove,
	.id_table = it66121_id,
};

module_i2c_driver(it66121_driver);

MODULE_AUTHOR("Phong LE");
MODULE_DESCRIPTION("IT66121 HDMI transmitter driver");
MODULE_LICENSE("GPL v2");
