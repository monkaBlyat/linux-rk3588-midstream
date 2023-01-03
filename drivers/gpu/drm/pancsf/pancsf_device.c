// SPDX-License-Identifier: GPL-2.0
/* Copyright 2018 Marty E. Plummer <hanetzer@startmail.com> */
/* Copyright 2019 Linaro, Ltd, Rob Herring <robh@kernel.org> */
/* Copyright 2023 Collabora ltd. */

#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regulator/consumer.h>

#include "pancsf_sched.h"
#include "pancsf_device.h"
#include "pancsf_gpu.h"
#include "pancsf_mmu.h"

static int pancsf_reset_init(struct pancsf_device *pfdev)
{
	pfdev->rstc = devm_reset_control_array_get_optional_exclusive(pfdev->dev);
	if (IS_ERR(pfdev->rstc)) {
		dev_err(pfdev->dev, "get reset failed %ld\n", PTR_ERR(pfdev->rstc));
		return PTR_ERR(pfdev->rstc);
	}

	return reset_control_deassert(pfdev->rstc);
}

static void pancsf_reset_fini(struct pancsf_device *pfdev)
{
	reset_control_assert(pfdev->rstc);
}

static int pancsf_clk_init(struct pancsf_device *pfdev)
{
	int err, i;
	unsigned long rate;

	pfdev->clock = devm_clk_get(pfdev->dev, NULL);
	if (IS_ERR(pfdev->clock)) {
		dev_err(pfdev->dev, "get clock failed %ld\n", PTR_ERR(pfdev->clock));
		return PTR_ERR(pfdev->clock);
	}

	rate = clk_get_rate(pfdev->clock);
	dev_info(pfdev->dev, "clock rate = %lu\n", rate);

	err = clk_prepare_enable(pfdev->clock);
	if (err)
		return err;

	pfdev->bus_clock = devm_clk_get_optional(pfdev->dev, "bus");
	if (IS_ERR(pfdev->bus_clock)) {
		dev_err(pfdev->dev, "get bus_clock failed %ld\n",
			PTR_ERR(pfdev->bus_clock));
		return PTR_ERR(pfdev->bus_clock);
	}

	if (pfdev->bus_clock) {
		rate = clk_get_rate(pfdev->bus_clock);
		dev_info(pfdev->dev, "bus_clock rate = %lu\n", rate);

		err = clk_prepare_enable(pfdev->bus_clock);
		if (err)
			goto disable_main_clock;
	}

	if (pfdev->comp->num_clks) {
		pfdev->platform_clocks = devm_kcalloc(pfdev->dev, pfdev->comp->num_clks,
						      sizeof(*pfdev->platform_clocks),
						      GFP_KERNEL);
		if (!pfdev->platform_clocks) {
			err = -ENOMEM;
			goto disable_bus_clock;
		}

		for (i = 0; i < pfdev->comp->num_clks; i++)
			pfdev->platform_clocks[i].id = pfdev->comp->clk_names[i];

		err = devm_clk_bulk_get(pfdev->dev,
					pfdev->comp->num_clks,
					pfdev->platform_clocks);
		if (err < 0) {
			dev_err(pfdev->dev, "failed to get platform clocks: %d\n", err);
			goto disable_bus_clock;
		}

		err = clk_bulk_prepare_enable(pfdev->comp->num_clks,
					      pfdev->platform_clocks);
		if (err < 0) {
			dev_err(pfdev->dev, "failed to enable platform clocks: %d\n", err);
			goto disable_bus_clock;
		}
	}

	return 0;

disable_bus_clock:
	clk_disable_unprepare(pfdev->bus_clock);

disable_main_clock:
	clk_disable_unprepare(pfdev->clock);
	return err;
}

static void pancsf_clk_fini(struct pancsf_device *pfdev)
{
	if (pfdev->platform_clocks) {
		clk_bulk_disable_unprepare(pfdev->comp->num_clks,
					   pfdev->platform_clocks);
	}

	clk_disable_unprepare(pfdev->bus_clock);
	clk_disable_unprepare(pfdev->clock);
}

static int pancsf_regulator_init(struct pancsf_device *pfdev)
{
	int ret, i;

	pfdev->regulators = devm_kcalloc(pfdev->dev, pfdev->comp->num_supplies,
					 sizeof(*pfdev->regulators),
					 GFP_KERNEL);
	if (!pfdev->regulators)
		return -ENOMEM;

	for (i = 0; i < pfdev->comp->num_supplies; i++)
		pfdev->regulators[i].supply = pfdev->comp->supply_names[i];

	ret = devm_regulator_bulk_get(pfdev->dev,
				      pfdev->comp->num_supplies,
				      pfdev->regulators);
	if (ret < 0) {
		if (ret != -EPROBE_DEFER)
			dev_err(pfdev->dev, "failed to get regulators: %d\n",
				ret);
		return ret;
	}

	ret = regulator_bulk_enable(pfdev->comp->num_supplies,
				    pfdev->regulators);
	if (ret < 0) {
		dev_err(pfdev->dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	return 0;
}

static void pancsf_regulator_fini(struct pancsf_device *pfdev)
{
	if (!pfdev->regulators)
		return;

	regulator_bulk_disable(pfdev->comp->num_supplies, pfdev->regulators);
}

static void pancsf_pm_domain_fini(struct pancsf_device *pfdev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pfdev->pm_domain_devs); i++) {
		if (!pfdev->pm_domain_devs[i])
			break;

		if (pfdev->pm_domain_links[i])
			device_link_del(pfdev->pm_domain_links[i]);

		dev_pm_domain_detach(pfdev->pm_domain_devs[i], true);
	}
}

static int pancsf_pm_domain_init(struct pancsf_device *pfdev)
{
	int err;
	int i, num_domains;

	num_domains = of_count_phandle_with_args(pfdev->dev->of_node,
						 "power-domains",
						 "#power-domain-cells");

	/*
	 * Single domain is handled by the core, and, if only a single power
	 * the power domain is requested, the property is optional.
	 */
	if (num_domains < 2 && pfdev->comp->num_pm_domains < 2)
		return 0;

	if (num_domains != pfdev->comp->num_pm_domains) {
		dev_err(pfdev->dev,
			"Incorrect number of power domains: %d provided, %d needed\n",
			num_domains, pfdev->comp->num_pm_domains);
		return -EINVAL;
	}

	if (WARN(num_domains > ARRAY_SIZE(pfdev->pm_domain_devs),
		 "Too many supplies in compatible structure.\n"))
		return -EINVAL;

	for (i = 0; i < num_domains; i++) {
		pfdev->pm_domain_devs[i] =
			dev_pm_domain_attach_by_name(pfdev->dev,
						     pfdev->comp->pm_domain_names[i]);
		if (IS_ERR_OR_NULL(pfdev->pm_domain_devs[i])) {
			err = PTR_ERR(pfdev->pm_domain_devs[i]) ? : -ENODATA;
			pfdev->pm_domain_devs[i] = NULL;
			dev_err(pfdev->dev,
				"failed to get pm-domain %s(%d): %d\n",
				pfdev->comp->pm_domain_names[i], i, err);
			goto err;
		}

		pfdev->pm_domain_links[i] = device_link_add(pfdev->dev,
							    pfdev->pm_domain_devs[i],
							    DL_FLAG_PM_RUNTIME |
							    DL_FLAG_STATELESS |
							    DL_FLAG_RPM_ACTIVE);
		if (!pfdev->pm_domain_links[i]) {
			dev_err(pfdev->pm_domain_devs[i],
				"adding device link failed!\n");
			err = -ENODEV;
			goto err;
		}
	}

	return 0;

err:
	pancsf_pm_domain_fini(pfdev);
	return err;
}

int pancsf_device_init(struct pancsf_device *pfdev)
{
	struct resource *res;
	int err;

	err = pancsf_clk_init(pfdev);
	if (err) {
		dev_err(pfdev->dev, "clk init failed %d\n", err);
		return err;
	}

	err = pancsf_regulator_init(pfdev);
	if (err)
		goto err_clk_fini;

	err = pancsf_reset_init(pfdev);
	if (err) {
		dev_err(pfdev->dev, "reset init failed %d\n", err);
		goto err_regulator_fini;
	}

	err = pancsf_pm_domain_init(pfdev);
	if (err)
		goto err_reset_fini;

	pfdev->iomem = devm_platform_get_and_ioremap_resource(pfdev->pdev, 0, &res);
	if (IS_ERR(pfdev->iomem)) {
		err = PTR_ERR(pfdev->iomem);
		goto err_pm_domain_fini;
	}

	pfdev->phys_addr = res->start;

	err = pancsf_gpu_init(pfdev);
	if (err)
		goto err_pm_domain_fini;

	err = pancsf_mmu_init(pfdev);
	if (err)
		goto err_gpu_fini;

	err = pancsf_mcu_init(pfdev);
	if (err)
		goto err_mmu_fini;

	err = pancsf_sched_init(pfdev);
	if (err)
		goto err_mcu_fini;

	return 0;

err_mcu_fini:
	pancsf_mcu_fini(pfdev);
err_mmu_fini:
	pancsf_mmu_fini(pfdev);
err_gpu_fini:
	pancsf_gpu_fini(pfdev);
err_pm_domain_fini:
	pancsf_pm_domain_fini(pfdev);
err_reset_fini:
	pancsf_reset_fini(pfdev);
err_regulator_fini:
	pancsf_regulator_fini(pfdev);
err_clk_fini:
	pancsf_clk_fini(pfdev);
	return err;
}

void pancsf_device_fini(struct pancsf_device *pfdev)
{
	pancsf_sched_fini(pfdev);
	pancsf_mcu_fini(pfdev);
	pancsf_mmu_fini(pfdev);
	pancsf_gpu_fini(pfdev);
	pancsf_pm_domain_fini(pfdev);
	pancsf_reset_fini(pfdev);
	pancsf_regulator_fini(pfdev);
	pancsf_clk_fini(pfdev);
}

#define PANCSF_EXCEPTION(id) \
	[DRM_PANCSF_EXCEPTION_ ## id] = { \
		.name = #id, \
	}

struct pancsf_exception_info {
	const char *name;
};

static const struct pancsf_exception_info pancsf_exception_infos[] = {
	PANCSF_EXCEPTION(OK),
	PANCSF_EXCEPTION(TERMINATED),
	PANCSF_EXCEPTION(KABOOM),
	PANCSF_EXCEPTION(EUREKA),
	PANCSF_EXCEPTION(ACTIVE),
	PANCSF_EXCEPTION(CS_RES_TERM),
	PANCSF_EXCEPTION(CS_CONFIG_FAULT),
	PANCSF_EXCEPTION(CS_ENDPOINT_FAULT),
	PANCSF_EXCEPTION(CS_BUS_FAULT),
	PANCSF_EXCEPTION(CS_INSTR_INVALID),
	PANCSF_EXCEPTION(CS_CALL_STACK_OVERFLOW),
	PANCSF_EXCEPTION(CS_INHERIT_FAULT),
	PANCSF_EXCEPTION(INSTR_INVALID_PC),
	PANCSF_EXCEPTION(INSTR_INVALID_ENC),
	PANCSF_EXCEPTION(INSTR_BARRIER_FAULT),
	PANCSF_EXCEPTION(DATA_INVALID_FAULT),
	PANCSF_EXCEPTION(TILE_RANGE_FAULT),
	PANCSF_EXCEPTION(ADDR_RANGE_FAULT),
	PANCSF_EXCEPTION(IMPRECISE_FAULT),
	PANCSF_EXCEPTION(OOM),
	PANCSF_EXCEPTION(CSF_FW_INTERNAL_ERROR),
	PANCSF_EXCEPTION(CSF_RES_EVICTION_TIMEOUT),
	PANCSF_EXCEPTION(GPU_BUS_FAULT),
	PANCSF_EXCEPTION(GPU_SHAREABILITY_FAULT),
	PANCSF_EXCEPTION(SYS_SHAREABILITY_FAULT),
	PANCSF_EXCEPTION(GPU_CACHEABILITY_FAULT),
	PANCSF_EXCEPTION(TRANSLATION_FAULT_0),
	PANCSF_EXCEPTION(TRANSLATION_FAULT_1),
	PANCSF_EXCEPTION(TRANSLATION_FAULT_2),
	PANCSF_EXCEPTION(TRANSLATION_FAULT_3),
	PANCSF_EXCEPTION(TRANSLATION_FAULT_4),
	PANCSF_EXCEPTION(PERM_FAULT_0),
	PANCSF_EXCEPTION(PERM_FAULT_1),
	PANCSF_EXCEPTION(PERM_FAULT_2),
	PANCSF_EXCEPTION(PERM_FAULT_3),
	PANCSF_EXCEPTION(ACCESS_FLAG_1),
	PANCSF_EXCEPTION(ACCESS_FLAG_2),
	PANCSF_EXCEPTION(ACCESS_FLAG_3),
	PANCSF_EXCEPTION(ADDR_SIZE_FAULT_IN),
	PANCSF_EXCEPTION(ADDR_SIZE_FAULT_OUT0),
	PANCSF_EXCEPTION(ADDR_SIZE_FAULT_OUT1),
	PANCSF_EXCEPTION(ADDR_SIZE_FAULT_OUT2),
	PANCSF_EXCEPTION(ADDR_SIZE_FAULT_OUT3),
	PANCSF_EXCEPTION(MEM_ATTR_FAULT_0),
	PANCSF_EXCEPTION(MEM_ATTR_FAULT_1),
	PANCSF_EXCEPTION(MEM_ATTR_FAULT_2),
	PANCSF_EXCEPTION(MEM_ATTR_FAULT_3),
};

const char *pancsf_exception_name(u32 exception_code)
{
	if (WARN_ON(exception_code >= ARRAY_SIZE(pancsf_exception_infos) ||
		    !pancsf_exception_infos[exception_code].name))
		return "Unknown exception type";

	return pancsf_exception_infos[exception_code].name;
}

#ifdef CONFIG_PM
int pancsf_device_resume(struct device *dev)
{
	return 0;
}

int pancsf_device_suspend(struct device *dev)
{
	/* FIXME: PM support */
	return -EBUSY;
}
#endif
