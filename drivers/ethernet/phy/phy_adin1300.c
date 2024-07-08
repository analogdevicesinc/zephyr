#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT adi_adin1300
LOG_MODULE_REGISTER(DT_DRV_COMPAT, CONFIG_PHY_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
#include <zephyr/net/mdio.h>
#include <zephyr/drivers/mdio.h>

struct adin1300_config {
	const struct device *mdio;
	uint8_t phy_addr;
	bool led0_en;
	bool led1_en;
	bool tx_24v;
	bool mii;
};

struct phy_adin1300_data {
	const struct device *dev;
	struct phy_link_state state;
	struct k_sem sem;
	struct k_work_delayable monitor_work;
	phy_callback_t cb;
	void *cb_data;
};

static int adin1300_reg_read(const struct device *dev, uint16_t reg_addr,
			     uint32_t *data)
{
        const struct adin1300_config *cfg = dev->config;

        return mdio_read(cfg->mdio, 0x3, reg_addr, data);
}

static int adin1300_reg_write(const struct device *dev, uint16_t reg_addr,
			     uint32_t data)
{
        const struct adin1300_config *cfg = dev->config;

        return mdio_write(cfg->mdio, 0x3, reg_addr, data);
}

static int adin1300_get_id(const struct device *dev, uint32_t *phy_id)
{
       	uint16_t val;

	if (adin1300_reg_read(dev, MII_PHYID1R, &val) < 0) {
		return -EIO;
	}

	*phy_id = (val & UINT16_MAX) << 16;

	if (adin1300_reg_read(dev, MII_PHYID2R, &val) < 0) {
		return -EIO;
	}

	*phy_id |= (val & UINT16_MAX);

	return 0;
}

static int adin1300_init(const struct device *dev)
{
        struct phy_adin1300_config *cfg = dev->config;
        uint32_t id;

        adin1300_get_id(dev, &id);
        printf("ADIN1300 id 0x%X\n", id);

        return 0;
}

static const struct ethphy_driver_api adin1300_api = {
	// .get_link = phy_adin2111_get_link_state,
	// .cfg_link = phy_adin2111_cfg_link,
	// .link_cb_set = phy_adin2111_link_cb_set,
	.read = adin1300_reg_read,
	.write = adin1300_reg_write,
};

#define ADIN1300_PHY_INITIALIZE(n)						\
	static const struct adin1300_config adin1300_config_##n = {	\
		.mdio = DEVICE_DT_GET(DT_INST_BUS(n)),				\
		.phy_addr = DT_INST_REG_ADDR(n),				\
	};									\
	DEVICE_DT_INST_DEFINE(n, &adin1300_init, NULL,			\
			      NULL, &adin1300_config_##n, \
			      POST_KERNEL, CONFIG_PHY_INIT_PRIORITY,		\
			      &adin1300_api);

DT_INST_FOREACH_STATUS_OKAY(ADIN1300_PHY_INITIALIZE)