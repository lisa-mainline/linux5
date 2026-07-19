// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Oleksii Onchul <oleksiionchul@gmail.com>
 *
 * Partially based on the vendor driver:
 *	Copyright (c) 2021 AWINIC Technology CO., LTD
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/fixp-arith.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/overflow.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>

#define AW8624_ID_REG				0x00
#define AW8624_ID_SOFTRST			0xaa
#define AW8624_CHIP_ID				0x24

#define AW8624_SYSINT_REG			0x02
#define AW8624_SYSINT_OVI			BIT(6)
#define AW8624_SYSINT_UVLI			BIT(5)
#define AW8624_SYSINT_FF_AEI			BIT(4)
#define AW8624_SYSINT_FF_AFI			BIT(3)
#define AW8624_SYSINT_OCDI			BIT(2)
#define AW8624_SYSINT_OTI			BIT(1)
#define AW8624_SYSINT_DONEI			BIT(0)

#define AW8624_SYSINTM_REG			0x03
#define AW8624_SYSINTM_OVM			BIT(6)
#define AW8624_SYSINTM_UVLOM			BIT(5)
#define AW8624_SYSINTM_FF_AEM			BIT(4)
#define AW8624_SYSINTM_FF_AFM			BIT(3)
#define AW8624_SYSINTM_DONEM			BIT(0)

#define AW8624_SYSCTRL_REG			0x04
#define AW8624_SYSCTRL_WAVDAT_MODE_MASK		GENMASK(7, 6)
#define AW8624_SYSCTRL_WAVDAT_MODE_1X		1
#define AW8624_SYSCTRL_RAMINIT			BIT(5)
#define AW8624_SYSCTRL_PLAY_MODE_MASK		GENMASK(3, 2)
#define AW8624_SYSCTRL_PLAY_MODE_RAM		0
#define AW8624_SYSCTRL_STANDBY			BIT(0)

#define AW8624_GO_REG				0x05
#define AW8624_GO_ENABLE			BIT(0)

#define AW8624_WAVSEQ1_REG			0x07
#define AW8624_WAVSEQ1_MASK			GENMASK(6, 0)
#define AW8624_WAVSEQ2_REG			0x08

#define AW8624_WAVLOOP1_REG			0x0f
#define AW8624_WAVLOOP1_SEQ1_MASK		GENMASK(7, 4)
#define AW8624_WAVLOOP1_SEQ1_INFINITE		0x0f

#define AW8624_DBGCTRL_REG			0x20
#define AW8624_DBGCTRL_INTN			BIT(5)
#define AW8624_DBGCTRL_INT_MODE_MASK		GENMASK(3, 2)
#define AW8624_DBGCTRL_INT_MODE_EDGE		1

#define AW8624_BASEADDRH_REG			0x21
#define AW8624_BASEADDRL_REG			0x22

#define AW8624_PWMDBG_REG			0x2e
#define AW8624_PWMDBG_PWM_MODE_MASK		GENMASK(6, 5)
#define AW8624_PWMDBG_PWM_24K			2

#define AW8624_GAIN_REG				0x3b

#define AW8624_RAMADDRH_REG			0x40
#define AW8624_RAMADDRL_REG			0x41
#define AW8624_RAMDATA_REG			0x42

#define AW8624_GLB_STATE_REG			0x47
#define AW8624_GLB_STATE_MASK			GENMASK(3, 0)
#define AW8624_GLB_STATE_STANDBY		0

#define AW8624_RAM_BASE_ADDR			0x0800
#define AW8624_WAVEFORM_RATE_HZ			24000
#define AW8624_WAVEFORM_AMPLITUDE		84
#define AW8624_MIN_LRA_FREQUENCY_HZ		100
#define AW8624_MAX_LRA_FREQUENCY_HZ		300

struct aw8624_sram_waveform {
	u8 version;
	__be16 start_address;
	__be16 end_address;
	u8 data[];
} __packed;

struct aw8624 {
	struct device *dev;
	struct input_dev *input;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct work_struct play_work;
	u32 lra_frequency_hz;
	u16 level;
};

static const struct regmap_config aw8624_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
	.max_register = 0x7f,
};

static void aw8624_hw_reset(struct aw8624 *haptics)
{
	gpiod_set_value_cansleep(haptics->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(haptics->reset_gpio, 0);
	usleep_range(8000, 8500);
}

static int aw8624_detect(struct aw8624 *haptics)
{
	unsigned int chip_id;
	int error;

	error = regmap_read(haptics->regmap, AW8624_ID_REG, &chip_id);
	if (error)
		return dev_err_probe(haptics->dev, error,
				     "Failed to read chip ID\n");

	if (chip_id != AW8624_CHIP_ID) {
		dev_err(haptics->dev, "Unexpected chip ID 0x%02x\n", chip_id);
		return -ENODEV;
	}

	return 0;
}

static int aw8624_stop(struct aw8624 *haptics)
{
	unsigned int state;
	int error;

	error = regmap_update_bits(haptics->regmap, AW8624_SYSINTM_REG,
				   AW8624_SYSINTM_UVLOM,
				   AW8624_SYSINTM_UVLOM);
	if (error)
		return error;

	error = regmap_write(haptics->regmap, AW8624_GO_REG, 0);
	if (error)
		return error;

	error = regmap_read_poll_timeout(haptics->regmap,
					 AW8624_GLB_STATE_REG, state,
					 FIELD_GET(AW8624_GLB_STATE_MASK,
						   state) ==
						AW8624_GLB_STATE_STANDBY,
					 2500, 250000);
	if (error && error != -ETIMEDOUT)
		return error;
	if (error)
		dev_warn(haptics->dev,
			 "Timed out waiting for standby; forcing it\n");

	return regmap_update_bits(haptics->regmap, AW8624_SYSCTRL_REG,
				  AW8624_SYSCTRL_STANDBY,
				  AW8624_SYSCTRL_STANDBY);
}

static int aw8624_set_active(struct aw8624 *haptics)
{
	unsigned int status;
	int error;

	error = regmap_update_bits(haptics->regmap, AW8624_SYSCTRL_REG,
				   AW8624_SYSCTRL_PLAY_MODE_MASK |
					AW8624_SYSCTRL_STANDBY,
				   FIELD_PREP(AW8624_SYSCTRL_PLAY_MODE_MASK,
					      AW8624_SYSCTRL_PLAY_MODE_RAM));
	if (error)
		return error;

	/* Reading SYSINT acknowledges any pending interrupts. */
	error = regmap_read(haptics->regmap, AW8624_SYSINT_REG, &status);
	if (error)
		return error;

	return regmap_update_bits(haptics->regmap, AW8624_SYSINTM_REG,
				  AW8624_SYSINTM_UVLOM, 0);
}

static int aw8624_play_sine(struct aw8624 *haptics)
{
	int error;

	error = aw8624_stop(haptics);
	if (error)
		return error;

	error = regmap_update_bits(haptics->regmap, AW8624_WAVSEQ1_REG,
				   AW8624_WAVSEQ1_MASK, 1);
	if (error)
		return error;

	error = regmap_write(haptics->regmap, AW8624_WAVSEQ2_REG, 0);
	if (error)
		return error;

	error = regmap_update_bits(haptics->regmap, AW8624_WAVLOOP1_REG,
				   AW8624_WAVLOOP1_SEQ1_MASK,
				   FIELD_PREP(AW8624_WAVLOOP1_SEQ1_MASK,
					      AW8624_WAVLOOP1_SEQ1_INFINITE));
	if (error)
		return error;

	error = regmap_write(haptics->regmap, AW8624_GAIN_REG,
			     haptics->level * 0x80 / 0xffff);
	if (error)
		return error;

	error = aw8624_set_active(haptics);
	if (error)
		return error;

	return regmap_write(haptics->regmap, AW8624_GO_REG,
			    AW8624_GO_ENABLE);
}

static void aw8624_play_work(struct work_struct *work)
{
	struct aw8624 *haptics = container_of(work, struct aw8624, play_work);
	int error;

	if (haptics->level)
		error = aw8624_play_sine(haptics);
	else
		error = aw8624_stop(haptics);

	if (error)
		dev_err(haptics->dev, "Failed to update playback: %d\n",
			error);
}

static int aw8624_play(struct input_dev *input, void *data,
		       struct ff_effect *effect)
{
	struct aw8624 *haptics = input_get_drvdata(input);
	u16 level = effect->u.rumble.strong_magnitude;

	if (!level)
		level = effect->u.rumble.weak_magnitude;

	if (haptics->level == level)
		return 0;

	haptics->level = level;
	schedule_work(&haptics->play_work);

	return 0;
}

static void aw8624_close(struct input_dev *input)
{
	struct aw8624 *haptics = input_get_drvdata(input);
	int error;

	cancel_work_sync(&haptics->play_work);
	haptics->level = 0;

	error = aw8624_stop(haptics);
	if (error)
		dev_err(haptics->dev, "Failed to stop playback: %d\n", error);
}

static int aw8624_haptic_init(struct aw8624 *haptics)
{
	unsigned int status;
	int error;

	error = regmap_update_bits(haptics->regmap, AW8624_SYSCTRL_REG,
				   AW8624_SYSCTRL_WAVDAT_MODE_MASK |
					AW8624_SYSCTRL_PLAY_MODE_MASK |
					AW8624_SYSCTRL_STANDBY,
				   FIELD_PREP(AW8624_SYSCTRL_WAVDAT_MODE_MASK,
					      AW8624_SYSCTRL_WAVDAT_MODE_1X) |
					FIELD_PREP(AW8624_SYSCTRL_PLAY_MODE_MASK,
						   AW8624_SYSCTRL_PLAY_MODE_RAM) |
					AW8624_SYSCTRL_STANDBY);
	if (error)
		return error;

	error = regmap_update_bits(haptics->regmap, AW8624_PWMDBG_REG,
				   AW8624_PWMDBG_PWM_MODE_MASK,
				   FIELD_PREP(AW8624_PWMDBG_PWM_MODE_MASK,
					      AW8624_PWMDBG_PWM_24K));
	if (error)
		return error;

	error = regmap_update_bits(haptics->regmap, AW8624_DBGCTRL_REG,
				   AW8624_DBGCTRL_INTN |
					AW8624_DBGCTRL_INT_MODE_MASK,
				   AW8624_DBGCTRL_INTN |
					FIELD_PREP(AW8624_DBGCTRL_INT_MODE_MASK,
						   AW8624_DBGCTRL_INT_MODE_EDGE));
	if (error)
		return error;

	error = regmap_read(haptics->regmap, AW8624_SYSINT_REG, &status);
	if (error)
		return error;

	/*
	 * Mask interrupts unused by RAM playback. UVLO is unmasked only while
	 * the actuator is active; over-current and over-temperature stay on.
	 */
	return regmap_write(haptics->regmap, AW8624_SYSINTM_REG,
			    AW8624_SYSINTM_OVM |
				AW8624_SYSINTM_UVLOM |
				AW8624_SYSINTM_FF_AEM |
				AW8624_SYSINTM_FF_AFM |
				AW8624_SYSINTM_DONEM);
}

static int aw8624_ram_init(struct aw8624 *haptics)
{
	struct aw8624_sram_waveform *waveform;
	unsigned int samples;
	size_t waveform_size;
	u16 start_address;
	int disable_error;
	int error;
	unsigned int i;

	samples = DIV_ROUND_CLOSEST(AW8624_WAVEFORM_RATE_HZ,
				    haptics->lra_frequency_hz);
	waveform_size = struct_size(waveform, data, samples);

	waveform = devm_kzalloc(haptics->dev, waveform_size, GFP_KERNEL);
	if (!waveform)
		return -ENOMEM;

	start_address = AW8624_RAM_BASE_ADDR + sizeof(*waveform);
	waveform->version = 1;
	waveform->start_address = cpu_to_be16(start_address);
	waveform->end_address = cpu_to_be16(start_address + samples - 1);

	for (i = 0; i < samples; i++) {
		s64 sample;

		sample = (s64)fixp_sin32(i * 360 / samples) *
			 AW8624_WAVEFORM_AMPLITUDE;
		waveform->data[i] = (u8)(s8)div_s64(sample, 0x7fffffff);
	}

	error = regmap_update_bits(haptics->regmap, AW8624_SYSCTRL_REG,
				   AW8624_SYSCTRL_RAMINIT,
				   AW8624_SYSCTRL_RAMINIT);
	if (error)
		return error;

	usleep_range(1000, 1500);

	error = regmap_write(haptics->regmap, AW8624_BASEADDRH_REG,
			     AW8624_RAM_BASE_ADDR >> 8);
	if (error)
		goto disable_raminit;

	error = regmap_write(haptics->regmap, AW8624_BASEADDRL_REG,
			     AW8624_RAM_BASE_ADDR & 0xff);
	if (error)
		goto disable_raminit;

	error = regmap_write(haptics->regmap, AW8624_RAMADDRH_REG,
			     AW8624_RAM_BASE_ADDR >> 8);
	if (error)
		goto disable_raminit;

	error = regmap_write(haptics->regmap, AW8624_RAMADDRL_REG,
			     AW8624_RAM_BASE_ADDR & 0xff);
	if (error)
		goto disable_raminit;

	error = regmap_noinc_write(haptics->regmap, AW8624_RAMDATA_REG,
				   waveform, waveform_size);

disable_raminit:
	disable_error = regmap_update_bits(haptics->regmap,
					   AW8624_SYSCTRL_REG,
					   AW8624_SYSCTRL_RAMINIT, 0);
	if (error)
		return error;

	return disable_error;
}

static irqreturn_t aw8624_irq(int irq, void *data)
{
	struct aw8624 *haptics = data;
	unsigned int status;
	int error;

	error = regmap_read(haptics->regmap, AW8624_SYSINT_REG, &status);
	if (error) {
		dev_err(haptics->dev, "Failed to read interrupt status: %d\n",
			error);
		return IRQ_NONE;
	}

	if (status & AW8624_SYSINT_OVI)
		dev_err(haptics->dev, "Over-voltage interrupt\n");
	if (status & AW8624_SYSINT_UVLI)
		dev_err(haptics->dev, "Under-voltage lockout interrupt\n");
	if (status & AW8624_SYSINT_OCDI)
		dev_err(haptics->dev, "Over-current interrupt\n");
	if (status & AW8624_SYSINT_OTI)
		dev_err(haptics->dev, "Over-temperature interrupt\n");
	if (status & AW8624_SYSINT_DONEI)
		dev_dbg(haptics->dev, "Playback finished\n");
	if (status & AW8624_SYSINT_FF_AFI)
		dev_dbg(haptics->dev, "RTP FIFO almost full\n");
	if (status & AW8624_SYSINT_FF_AEI)
		dev_dbg(haptics->dev, "RTP FIFO almost empty\n");

	return IRQ_HANDLED;
}

static int aw8624_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct aw8624 *haptics;
	int error;

	haptics = devm_kzalloc(dev, sizeof(*haptics), GFP_KERNEL);
	if (!haptics)
		return -ENOMEM;

	haptics->dev = dev;
	i2c_set_clientdata(client, haptics);

	error = device_property_read_u32(dev, "awinic,resonant-freq-hz",
					 &haptics->lra_frequency_hz);
	if (error)
		return dev_err_probe(dev, error,
				     "Missing LRA resonant frequency\n");

	if (haptics->lra_frequency_hz < AW8624_MIN_LRA_FREQUENCY_HZ ||
	    haptics->lra_frequency_hz > AW8624_MAX_LRA_FREQUENCY_HZ)
		return dev_err_probe(dev, -EINVAL,
				     "LRA resonant frequency is out of range\n");

	error = devm_regulator_get_enable_optional(dev, "vdd");
	if (error && error != -ENODEV)
		return dev_err_probe(dev, error,
				     "Failed to enable VDD supply\n");

	haptics->regmap = devm_regmap_init_i2c(client,
					       &aw8624_regmap_config);
	if (IS_ERR(haptics->regmap))
		return dev_err_probe(dev, PTR_ERR(haptics->regmap),
				     "Failed to allocate register map\n");

	haptics->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(haptics->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(haptics->reset_gpio),
				     "Failed to get reset GPIO\n");

	aw8624_hw_reset(haptics);

	error = aw8624_detect(haptics);
	if (error)
		return error;

	error = regmap_write(haptics->regmap, AW8624_ID_REG,
			     AW8624_ID_SOFTRST);
	if (error)
		return dev_err_probe(dev, error, "Failed to reset chip\n");

	usleep_range(2000, 3000);

	error = aw8624_haptic_init(haptics);
	if (error)
		return dev_err_probe(dev, error,
				     "Failed to initialize chip\n");

	if (client->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "Invalid interrupt\n");

	error = devm_request_threaded_irq(dev, client->irq, NULL, aw8624_irq,
					  IRQF_ONESHOT, NULL, haptics);
	if (error)
		return dev_err_probe(dev, error,
				     "Failed to request interrupt\n");

	error = aw8624_ram_init(haptics);
	if (error)
		return dev_err_probe(dev, error,
				     "Failed to initialize waveform RAM\n");

	INIT_WORK(&haptics->play_work, aw8624_play_work);

	haptics->input = devm_input_allocate_device(dev);
	if (!haptics->input)
		return -ENOMEM;

	haptics->input->name = "aw8624-haptics";
	haptics->input->close = aw8624_close;
	input_set_drvdata(haptics->input, haptics);
	input_set_capability(haptics->input, EV_FF, FF_RUMBLE);

	error = input_ff_create_memless(haptics->input, NULL, aw8624_play);
	if (error)
		return dev_err_probe(dev, error,
				     "Failed to create force-feedback device\n");

	error = input_register_device(haptics->input);
	if (error)
		return dev_err_probe(dev, error,
				     "Failed to register input device\n");

	return 0;
}

static const struct of_device_id aw8624_of_match[] = {
	{ .compatible = "awinic,aw8624" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw8624_of_match);

static struct i2c_driver aw8624_driver = {
	.driver = {
		.name = "aw8624-haptics",
		.of_match_table = aw8624_of_match,
	},
	.probe = aw8624_probe,
};
module_i2c_driver(aw8624_driver);

MODULE_AUTHOR("Oleksii Onchul <oleksiionchul@gmail.com>");
MODULE_DESCRIPTION("Awinic AW8624 LRA haptic driver");
MODULE_LICENSE("GPL");
