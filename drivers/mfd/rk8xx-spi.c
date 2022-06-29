// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip RK806 Core (SPI) driver
 *
 * Copyright (c) 2021 Rockchip Electronics Co., Ltd.
 * Copyright (c) 2023 Collabora Ltd.
 *
 * Author: Xu Shengfei <xsf@rock-chips.com>
 * Author: Sebastian Reichel <sebastian.reichel@collabora.com>
 */

#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/mfd/rk808.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#define RK806_CMD		0
#define RK806_REG_ADDR_L	1
#define RK806_REG_ADDR_H	2

static const struct regmap_range rk806_volatile_ranges[] = {
	regmap_reg_range(RK806_POWER_EN0, RK806_POWER_EN5),
	regmap_reg_range(RK806_DVS_START_CTRL, RK806_INT_MSK1),
};

static const struct regmap_access_table rk806_volatile_table = {
	.yes_ranges = rk806_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(rk806_volatile_ranges),
};

static const struct regmap_config rk806_regmap_config_spi = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.volatile_table = &rk806_volatile_table,
};

static int rk806_spi_bus_write(void *context, const void *vdata, size_t count)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	const char *data = vdata;
	char buffer[3] = { 0 };
	struct spi_transfer xfer[2] = { 0 };

	buffer[RK806_CMD]	 = RK806_CMD_WRITE | (count - 2);
	buffer[RK806_REG_ADDR_L] = data[0];
	buffer[RK806_REG_ADDR_H] = RK806_REG_H;

	xfer[0].tx_buf = buffer;
	xfer[0].len = sizeof(buffer);
	xfer[1].tx_buf = data+1;
	xfer[1].len = count-1;

	return spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));
}

static int rk806_spi_bus_read(void *context, const void *vreg, size_t reg_size,
			      void *val, size_t val_size)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	const char *reg = vreg;
	char txbuf[3] = { 0 };

	if (reg_size != sizeof(char) || val_size < 1)
		return -EINVAL;

	txbuf[RK806_CMD]	= RK806_CMD_READ | (val_size - 1);
	txbuf[RK806_REG_ADDR_L]	= *reg;
	txbuf[RK806_REG_ADDR_H]	= RK806_REG_H;

	return spi_write_then_read(spi, txbuf, sizeof(txbuf), val, val_size);
}

static const struct regmap_bus rk806_regmap_bus_spi = {
	.write = rk806_spi_bus_write,
	.read = rk806_spi_bus_read,
	.reg_format_endian_default = REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default = REGMAP_ENDIAN_NATIVE,
};

static int rk8xx_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init(&spi->dev, &rk806_regmap_bus_spi,
				  &spi->dev, &rk806_regmap_config_spi);
	if (IS_ERR(regmap))
		return dev_err_probe(&spi->dev, PTR_ERR(regmap),
				     "Failed to initialize register map\n");

	return rk8xx_probe(&spi->dev, RK806_ID, spi->irq, regmap);
}

static const struct of_device_id rk8xx_spi_of_match[] = {
	{ .compatible = "rockchip,rk806", },
	{ }
};
MODULE_DEVICE_TABLE(of, rk8xx_spi_of_match);

static const struct spi_device_id rk8xx_spi_id_table[] = {
	{ "rk806", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, rk8xx_spi_id_table);

static struct spi_driver rk8xx_spi_driver = {
	.driver		= {
		.name	= "rk8xx-spi",
		.of_match_table = rk8xx_spi_of_match,
	},
	.probe		= rk8xx_spi_probe,
	.id_table	= rk8xx_spi_id_table,
};
module_spi_driver(rk8xx_spi_driver);

MODULE_AUTHOR("Xu Shengfei <xsf@rock-chips.com>");
MODULE_DESCRIPTION("RK8xx SPI PMIC driver");
MODULE_LICENSE("GPL v2");
