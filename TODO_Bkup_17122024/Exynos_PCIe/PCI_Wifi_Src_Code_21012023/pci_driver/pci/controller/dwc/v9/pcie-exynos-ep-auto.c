// SPDX-License-Identifier: GPL-2.0
/**
 * pcie-exynos-ep-auto.c - Exynos PCIe EP driver
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Authors: Sanghoon Bae <sh86.bae@samsung.com>
 *	    Jiheon Oh <jiheon.oh@samsung.com>
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <dt-bindings/pci/pci.h>
#include <linux/random.h>
#include <linux/crc32.h>

#ifdef CONFIG_S2MPU
#include <vlx/s2mpu-protection.h>
#endif

#include "pcie-designware.h"
#include "pcie-exynos-ep-auto.h"
#include "pcie-exynos-ep-auto-cal.h"

struct exynos_ep_pcie g_pcie_ep[MAX_EP_NUM];

#define MEASURE_SPEED
#ifdef MEASURE_SPEED
u64 getCurrentTimestampInNSec(void)
{
	struct timespec64 timeStamp;

	ktime_get_real_ts64(&timeStamp);

	return (u64)timeStamp.tv_sec * 1000000000ul + (u64)timeStamp.tv_nsec;
}

void measureTransitionSpeed(struct exynos_ep_pcie *pcie_ep, u64 startTime,
		u64 endTime, u64 size)
{
	struct platform_device *pdev = pcie_ep->pdev;
	int unit = 1; //0: KB, 1: MB
	u64 convertedSize = (u64)(size >> 20); // MByte unit

	if (convertedSize == 0) {
		convertedSize = size >> 10; // KByte unit
		unit = 0;
	}

	if (unit == 1) {
		dev_info(&pdev->dev, "###### size: %lldMB , time: %lld us\n",
				convertedSize, endTime - startTime);
		dev_info(&pdev->dev, "###### speed: %lldMB/sec\n",
				(convertedSize*1000000000ul)/(endTime-startTime));
	} else {
		dev_info(&pdev->dev, "###### size: %lldKB , time: %lld us\n",
				convertedSize, endTime-startTime);
		dev_info(&pdev->dev, "###### speed: %lldMB/sec\n",
				((convertedSize * 1000000000ul) >> 10)/
				(endTime - startTime));
	}
}
#endif

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Dump register info"
 * @logic "Print ELBI region\n
 *	Print DBI region\n
 *	Print device/link status."
 * @params
 * @param{in, pcie_ep, struct ::exynos_ep_pcie, not NULL}
 * @endparam
 * @noret
 */
void exynos_pcie_ep_register_link_dump(struct exynos_ep_pcie *pcie_ep)
{
	struct platform_device *pdev = pcie_ep->pdev;
	struct dw_pcie *pci = pcie_ep->pci;
	u32 i, j, val;

	dev_info(&pdev->dev, "Print ELBI region...\n");
	for (i = 1; i < 45; i++) {
		for (j = 0; j < 4; j++) {
			if (((i * 0x10) + (j * 4)) < 0x2C4) {
				dev_info(&pdev->dev, "ELBI 0x%04x : 0x%08x\n",
					(i * 0x10) + (j * 4),
					readl(pcie_ep->elbi_base +
						(i * 0x10) + (j * 4)));
			}
		}
	}

	dev_info(&pdev->dev, "\n");

	/* RC Conf : 0x0 ~ 0x40 */
	dev_info(&pdev->dev, "Print DBI region...\n");
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			val = dw_pcie_readl_dbi(pci,
					(i * 0x10) + (j * 4));
			dev_info(&pdev->dev, "DBI 0x%04x : 0x%08x\n",
					(i * 0x10) + (j * 4), val);
		}
	}

	dev_info(&pdev->dev, "\n");

	val = dw_pcie_readl_dbi(pci, 0x78);
	dev_info(&pdev->dev, "EP Conf 0x0078(Device Status Register): 0x%08x\n",
			val);
	val = dw_pcie_readl_dbi(pci, 0x80);
	dev_info(&pdev->dev, "EP Conf 0x0080(Link Status Register): 0x%08x\n",
			val);
	val = dw_pcie_readl_dbi(pci, 0x104);
	dev_info(&pdev->dev, "EP Conf 0x0104(AER Register): 0x%08x\n", val);
	val = dw_pcie_readl_dbi(pci, 0x110);
	dev_info(&pdev->dev, "EP Conf 0x0110(AER Register): 0x%08x\n", val);
	val = dw_pcie_readl_dbi(pci, 0x130);
	dev_info(&pdev->dev, "EP Conf 0x0130(AER Register): 0x%08x\n", val);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_init}
 * @purpose "Enable/disable clock"
 * @logic "Call clk_prepare_enable or clk_disable_unprepare"
 * @params
 * @param{in, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @param{in, enable, int, 0~1}
 * @endparam
 * @retval{ret, int, 0, <=0, <0}
 */
static int exynos_pcie_ep_clk_enable(struct exynos_ep_pcie *pcie_ep, int enable)
{
	struct exynos_pcie_ep_clks *clks = &pcie_ep->clks;
	int i;
	int ret = 0;

	if (enable) {
		for (i = 0; i < pcie_ep->pcie_clk_num; i++)
			ret = clk_prepare_enable(clks->pcie_clks[i]);
#if !defined(CONFIG_SOC_EXYNOSAUTO9)
	} else {
		for (i = 0; i < pcie_ep->pcie_clk_num; i++)
			clk_disable_unprepare(clks->pcie_clks[i]);
#endif
	}

	return ret;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_init}
 * @purpose "Enable/disable phy clock"
 * @logic "Call clk_prepare_enable or clk_disable_unprepare"
 * @params
 * @param{in, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @param{in, enable, int, 0~1}
 * @endparam
 * @retval{ret, int, 0, <=0, <0}
 */
static int exynos_pcie_ep_phyclk_enable(struct exynos_ep_pcie *pcie_ep,
						int enable)
{
	struct exynos_pcie_ep_clks *clks = &pcie_ep->clks;
	int i;
	int ret = 0;

	if (enable) {
		for (i = 0; i < pcie_ep->phy_clk_num; i++)
			ret = clk_prepare_enable(clks->phy_clks[i]);
#if !defined(CONFIG_SOC_EXYNOSAUTO9)
	} else {
		for (i = 0; i < pcie_ep->phy_clk_num; i++)
			clk_disable_unprepare(clks->phy_clks[i]);
#endif
	}

	return ret;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_init}
 * @purpose "Initialize PCIe EP phy functions"
 * @logic "Allocate phy functions to ops."
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @noret
 */
void exynos_pcie_ep_phyinit(struct exynos_ep_pcie *pcie_ep)
{
	struct platform_device *pdev = pcie_ep->pdev;

	dev_info(&pdev->dev, "Initialize PCIE EP PHY functions.\n");

	pcie_ep->phy_ops.phy_all_pwrdn = exynos_pcie_ep_phy_all_pwrdn;
	pcie_ep->phy_ops.phy_all_pwrdn_clear =
					exynos_pcie_ep_phy_all_pwrdn_clear;
	pcie_ep->phy_ops.phy_config = exynos_pcie_ep_phy_config;
	pcie_ep->phy_ops.phy_other_set = exynos_pcie_ep_phy_others_set;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_init}
 * @purpose "Get resources from dt"
 * @logic "Get resources from dt\n
 *	Perform I/O remap for each bases."
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @retval{ret, int, 0, >=0, >0}
 */
static int exynos_pcie_ep_get_resource(struct exynos_ep_pcie *pcie_ep)
{
	struct platform_device *pdev = pcie_ep->pdev;
	struct resource *temp_rsc;
	int ret = 0;

	temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "elbi");
	pcie_ep->elbi_base = ioremap(temp_rsc->start, resource_size(temp_rsc));
	if (IS_ERR(pcie_ep->elbi_base)) {
		ret = PTR_ERR(pcie_ep->elbi_base);
		goto fail;
	}

	temp_rsc = platform_get_resource_byname(pdev,
						IORESOURCE_MEM, "elbi_cmn");
	pcie_ep->elbi_cmn_base = ioremap(temp_rsc->start,
						resource_size(temp_rsc));
	if (IS_ERR(pcie_ep->elbi_cmn_base)) {
		ret = PTR_ERR(pcie_ep->elbi_cmn_base);
		goto fail;
	}

	temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phy");
	pcie_ep->phy_base = ioremap(temp_rsc->start, resource_size(temp_rsc));
	if (IS_ERR(pcie_ep->phy_base)) {
		ret = PTR_ERR(pcie_ep->phy_base);
		goto fail;
	}

	temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sysreg");
	pcie_ep->sysreg_base = ioremap(temp_rsc->start,
					resource_size(temp_rsc));
	if (IS_ERR(pcie_ep->sysreg_base)) {
		ret = PTR_ERR(pcie_ep->sysreg_base);
		goto fail;
	}

	temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					"sysreg_ctrl");
	pcie_ep->sysreg_ctrl_base = devm_ioremap_resource(&pdev->dev, temp_rsc);
	if (IS_ERR(pcie_ep->sysreg_ctrl_base)) {
		ret = PTR_ERR(pcie_ep->sysreg_ctrl_base);
		goto fail;
	}

	temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	pcie_ep->dbi_base = devm_ioremap_resource(&pdev->dev, temp_rsc);
	if (IS_ERR(pcie_ep->dbi_base)) {
		ret = PTR_ERR(pcie_ep->dbi_base);
		goto fail;
	}

	temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "atu");
	pcie_ep->atu_base = devm_ioremap_resource(&pdev->dev, temp_rsc);
	if (IS_ERR(pcie_ep->pci->atu_base)) {
		ret = PTR_ERR(pcie_ep->pci->atu_base);
		goto fail;
	}

	temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dma");
	pcie_ep->dma_base = devm_ioremap_resource(&pdev->dev, temp_rsc);
	if (IS_ERR(pcie_ep->dma_base)) {
		ret = PTR_ERR(pcie_ep->dma_base);
		goto fail;
	}

	temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pcs");
	pcie_ep->phy_pcs_base = ioremap(temp_rsc->start,
					resource_size(temp_rsc));
	if (IS_ERR(pcie_ep->phy_pcs_base)) {
		ret = PTR_ERR(pcie_ep->phy_pcs_base);
		goto fail;
	}

	temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					"doorbell");
	pcie_ep->doorbell_base = devm_ioremap_resource(&pdev->dev, temp_rsc);
	if (IS_ERR(pcie_ep->doorbell_base)) {
		ret = PTR_ERR(pcie_ep->doorbell_base);
		goto fail;
	}

fail:
	return ret;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Raise irq"
 * @logic "Dummy function for ops"
 * @params
 * @param{in, ep, struct ::dw_pcie_ep *, not NULL}
 * @endparam
 * @retval{-, int, 0, 0, not 0}
 */
static int exynos_pcie_ep_raise_irq(struct dw_pcie_ep *ep,
	u8 func_no, enum pci_epc_irq_type type, u16 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	dev_info(pci->dev, "%s\n", __func__);

	return 0;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_init}
 * @purpose "EP init"
 * @logic "Dummy function for ops"
 * @params
 * @param{in, ep, struct ::dw_pcie_ep *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_pcie_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	dev_info(pci->dev, "%s\n", __func__);
}

static struct dw_pcie_ep_ops pcie_ep_ops = {
	.ep_init = exynos_pcie_ep_init,
	.raise_irq = exynos_pcie_ep_raise_irq,
};

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_init}
 * @purpose "Initialize designware endpoint"
 * @logic "Get resource from dt\n
 *	Call dw_pcie_ep_init"
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @retval{ret, int, 0, 0/-EINVAL, -EINVAL}
 */
static int exynos_add_pcie_ep(struct exynos_ep_pcie *pcie_ep)
{
	struct platform_device *pdev = pcie_ep->pdev;
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci = pcie_ep->pci;
	struct dw_pcie_ep *ep;
	struct resource *res;
	int ret = 0;

	ep = &pci->ep;
	ep->ops = &pcie_ep_ops;
	pci->dbi_base = pcie_ep->dbi_base;
	pci->atu_base = pcie_ep->atu_base;
	/* for designware mainline code */
	pci->dbi_base2 = pcie_ep->dbi_base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "addr_space");
	if (!res) {
		dev_err(dev, "can't get addr_space for PCIe EP from DT\n");
		return -EINVAL;
	}
	ep->phys_base = res->start;
	ep->addr_size = resource_size(res);

	ret = dw_pcie_ep_init(ep);
	if (ret) {
		dev_err(dev, "failed to initialize PCIe dw endpoint\n");
		return ret;
	}

	return 0;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_init}
 * @purpose "Reset ep"
 * @logic "Soft power reset\n
 *	Sticky reset\n
 *	Non-sticky reset\n
 *	Core reset."
 * @params
 * @param{in, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_pcie_reset_ep(struct exynos_ep_pcie *pcie_ep)
{
	struct platform_device *pdev = pcie_ep->pdev;
	u32 val;
	u32 linkA, linkB, phy_config_level = 1;

	dev_dbg(&pdev->dev, "START PCIe reset EP\n");

	/* Soft power reset */
	val = readl(pcie_ep->elbi_base + PCIE_SOFT_RESET);
	val |= SOFT_PWR_RESET;
	writel(val, pcie_ep->elbi_base + PCIE_SOFT_RESET);
	val = readl(pcie_ep->elbi_base + PCIE_SOFT_RESET);
	val &= ~SOFT_PWR_RESET;
	writel(val, pcie_ep->elbi_base + PCIE_SOFT_RESET);
	udelay(10);
	val = readl(pcie_ep->elbi_base + PCIE_SOFT_RESET);
	val |= SOFT_PWR_RESET;
	writel(val, pcie_ep->elbi_base + PCIE_SOFT_RESET);

	udelay(20);

	/* Sticky Reset */
	val = readl(pcie_ep->elbi_base + PCIE_SOFT_RESET);
	val |= SOFT_STICKY_RESET;
	writel(val, pcie_ep->elbi_base + PCIE_SOFT_RESET);
	val = readl(pcie_ep->elbi_base + PCIE_SOFT_RESET);
	val &= ~SOFT_STICKY_RESET;
	writel(val, pcie_ep->elbi_base + PCIE_SOFT_RESET);
	udelay(10);
	val = readl(pcie_ep->elbi_base + PCIE_SOFT_RESET);
	val |= SOFT_STICKY_RESET;
	writel(val, pcie_ep->elbi_base + PCIE_SOFT_RESET);

	/* Non-Sticky Reset */
	val = readl(pcie_ep->elbi_base + PCIE_SOFT_RESET);
	val |= SOFT_NON_STICKY_RESET;
	writel(val, pcie_ep->elbi_base + PCIE_SOFT_RESET);
	val = readl(pcie_ep->elbi_base + PCIE_SOFT_RESET);
	val &= ~SOFT_NON_STICKY_RESET;
	writel(val, pcie_ep->elbi_base + PCIE_SOFT_RESET);
	udelay(10);
	val = readl(pcie_ep->elbi_base + PCIE_SOFT_RESET);
	val |= SOFT_NON_STICKY_RESET;
	writel(val, pcie_ep->elbi_base + PCIE_SOFT_RESET);

	/* Core Reset */
	val = readl(pcie_ep->elbi_base + PCIE_SOFT_RESET);
	val |= SOFT_CORE_RESET;
	writel(val, pcie_ep->elbi_base + PCIE_SOFT_RESET);
	val = readl(pcie_ep->elbi_base + PCIE_SOFT_RESET);
	val &= ~SOFT_CORE_RESET;
	writel(val, pcie_ep->elbi_base + PCIE_SOFT_RESET);
	udelay(10);
	val = readl(pcie_ep->elbi_base + PCIE_SOFT_RESET);
	val |= SOFT_CORE_RESET;
	writel(val, pcie_ep->elbi_base + PCIE_SOFT_RESET);

	linkA = readl(pcie_ep->phy_pcs_base + 0x988);
	linkB = readl(pcie_ep->phy_pcs_base + 0x188);

	dev_dbg(&pdev->dev, "[%s] linkA:%d linkB:%d\n", __func__, linkA, linkB);

	if (((linkA >= 2 && linkA <= 3) && (linkB >= 2 && linkB <= 3))
		|| (!pcie_ep->use_bifurcation)) {
		phy_config_level = 0;
	}

	if (pcie_ep->use_sris && phy_config_level == 0) {
		writel(1, pcie_ep->elbi_base + PCIE_SRIS_MODE);
		dev_dbg(&pdev->dev, "[%s] SRIS enable\n", __func__);
	}
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_init}
 * @purpose "Setup ep phy"
 * @logic "Call exynos_pcie_reset_ep\n
 *	Setup ep phy config."
 * @params
 * @param{in, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_pcie_setup_ep(struct exynos_ep_pcie *pcie_ep)
{
	struct platform_device *pdev = pcie_ep->pdev;
	struct dw_pcie *pci = pcie_ep->pci;
	int ch_num = pcie_ep->ch_num;
	int lane = 0;
	u32 val;

	dev_dbg(&pdev->dev, "START PCIe setup EP\n");

	exynos_pcie_reset_ep(pcie_ep);

	if (pci->dbi_base == NULL)
		exynos_add_pcie_ep(pcie_ep);

	dw_pcie_dbi_ro_wr_en(pci);

	/* Port Link Control Register, set number of lanes to 1 */
	val = dw_pcie_readl_dbi(pci, PL_PT_LNK_CTRL_R);
	val &= 0xfff0ffff;
	dw_pcie_writel_dbi(pci, PL_PT_LNK_CTRL_R, val);

	val = dw_pcie_readl_dbi(pci, PL_PT_LNK_CTRL_R);

	/* use_bifurcation => 1: bifurcation, 0:aggregation */
	if (ch_num == CH0_4L || ch_num == CH1_4L) {
		if (!pcie_ep->use_bifurcation) {
			lane = 4;
			val |= (0x7 << 16);
			dw_pcie_writel_dbi(pci, PL_PT_LNK_CTRL_R, val);
		} else {
			lane = 2;
			val |= (0x3 << 16);
			dw_pcie_writel_dbi(pci, PL_PT_LNK_CTRL_R, val);
		}
	} else {
		if (!pcie_ep->use_bifurcation) {
			lane = 2;
			val |= (0x3 << 16);
			dw_pcie_writel_dbi(pci, PL_PT_LNK_CTRL_R, val);
		} else {
			lane = 1;
			val |= (0x1 << 16);
			dw_pcie_writel_dbi(pci, PL_PT_LNK_CTRL_R, val);
		}
	}

	dev_info(&pdev->dev, "[%s] lane : %d\n", __func__, lane);

	/* Link Width and speed Change control Register */
	val = dw_pcie_readl_dbi(pci, PL_GEN2_CTRL);
	val &= ~(0x1FF << 8);
	val |= (lane << 8);
	dw_pcie_writel_dbi(pci, PL_GEN2_CTRL, val);

	/* set max link width & speed : Gen1, lane1 */
	val = dw_pcie_readl_dbi(pci, LNK_CAP);
	val &= 0xfffffc00;
	val |= (lane << 4);
	val |= (3 << 0);
	dw_pcie_writel_dbi(pci, LNK_CAP, val);
	dw_pcie_writel_dbi(pci, LNK_STS_CTRL2, 0x3);

	dev_dbg(&pdev->dev,
		"[%s] PCIe EP lane number & speed selection\n", __func__);

	/* set N_FTS : 255 */
	val = dw_pcie_readl_dbi(pci, PL_ACK_FREQ_R);
	val &= 0xffff00ff;
	val |= (0xff << 8);
	dw_pcie_writel_dbi(pci, PL_ACK_FREQ_R, val);

	/* enable error reporting */
	dw_pcie_writel_dbi(pci, AER_RT_ERR_CMD, RC_REGMASK_CORRECTABLE_ERR_EN
				| RC_REGMASK_NONFATAL_ERR_EN
				| RC_REGMASK_FATAL_ERR_EN);
	val = dw_pcie_readl_dbi(pci, AER_CO_ERR_MS);
	dw_pcie_writel_dbi(pci, AER_CO_ERR_MS, val | (0x1 << 12));

	/* setup interrupt pins */
	dw_pcie_readl_dbi(pci, DBI_BRIDGE_CTRL_INT_PIN);
	val &= 0xffff00ff;
	val |= 0x00000100;
	dw_pcie_writel_dbi(pci, DBI_BRIDGE_CTRL_INT_PIN, val);

	val = dw_pcie_readl_dbi(pci, DBI_BRIDGE_CTRL_INT_PIN);

	/* setup bus number */
	/* Subordinate 0x01, Secondary Bus no 0x01, Primary bus no 0x00 */
	val = dw_pcie_readl_dbi(pci, DBI_PRIM_SEC_BUS);
	val &= 0xff000000;
	val |= 0x00ff0100;
	dw_pcie_writel_dbi(pci, DBI_PRIM_SEC_BUS, val);

	/* setup command register */
	val = dw_pcie_readl_dbi(pci, PCI_COMMAND);
	val &= 0xffff0000;
	val |= (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER |
			PCI_COMMAND_PARITY | PCI_COMMAND_SERR);
	dw_pcie_writel_dbi(pci, PCI_COMMAND, val);

	/* Turn off DUT error message */
	dw_pcie_writel_dbi(pci, DEV_STS_CTRL, 0);

	/* set max payload size to 256 bytes */
	val = dw_pcie_readl_dbi(pci, DEV_STS_CTRL);
	val |= ((1<<5) | (1<<12) | (1<<14));
	dw_pcie_writel_dbi(pci, DEV_STS_CTRL, val);
	val = dw_pcie_readl_dbi(pci, DEV_STS_CTRL);

	/* EQ off */
	dw_pcie_writel_dbi(pci, 0x890, 0x00012001);

	/* for dma transfer */
	dw_pcie_writel_dbi(pci, 0x8E8, 0x10101010);

	dw_pcie_dbi_ro_wr_dis(pci);

	dev_dbg(&pdev->dev, "[%s] set L23 ready bit\n", __func__);
	writel(0x1, pcie_ep->elbi_base + CSR_APP_READY_ENTR_L23_GEN3);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_link}
 * @purpose "Set atomic op"
 * @logic "Set atomic op"
 * @params
 * @param{in, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @param{in, enable, int, 0~1}
 * @endparam
 * @noret
 */
static void exynos_pcie_ep_atomic_op(struct exynos_ep_pcie *pcie_ep, int enable)
{
	struct platform_device *pdev = pcie_ep->pdev;
	u32 val = 0;

	dev_dbg(&pdev->dev, "START PCIe set AtomicOP\n");

	val = readl(pcie_ep->elbi_base + SUBCON_APPREQ_EXIT_L1_MODE);

	if (enable)
		val |= (0x1 << 0);
#if !defined(CONFIG_SOC_EXYNOSAUTO9)
	else
		val &= ~(0x1 << 0);
#endif

	writel(val, pcie_ep->elbi_base + SUBCON_APPREQ_EXIT_L1_MODE);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Set link traning"
 * @logic "Enable/disable link training"
 * @params
 * @param{in, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @param{in, enable, int, 0~1}
 * @endparam
 * @noret
 */
static void exynos_pcie_ep_set_ltssm(struct exynos_ep_pcie *pcie_ep, int enable)
{
	struct platform_device *pdev = pcie_ep->pdev;
	u32 val;

	if (enable) {
		dev_info(&pdev->dev, "PCIe EP Enable LTSSM\n");
		val = readl(pcie_ep->elbi_base + CSR_APP_LTSSM_EN_GEN3);
		val |= 0x1;
		writel(val, pcie_ep->elbi_base + CSR_APP_LTSSM_EN_GEN3);
	} else {
		dev_info(&pdev->dev, "PCIe EP Disable LTSSM\n");
		val = readl(pcie_ep->elbi_base + CSR_APP_LTSSM_EN_GEN3);
		val &= ~0x1;
		writel(val, pcie_ep->elbi_base + CSR_APP_LTSSM_EN_GEN3);
	}
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_link}
 * @purpose "Setup interrupt"
 * @logic "Clear pending bit\n
 *	Enable interrupt."
 * @params
 * @param{in, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @retval{-, int, 0, 0, not 0}
 */
static int exynos_pcie_ep_setup_interrupt(struct exynos_ep_pcie *pcie_ep)
{
	void __iomem *elbi_base = pcie_ep->elbi_base;
	u32 irq[4];

	irq[0] = readl(elbi_base + PCIE_IRQ0);
	irq[1] = readl(elbi_base + PCIE_IRQ1);
	irq[2] = readl(elbi_base + PCIE_IRQ2);
	irq[3] = readl(elbi_base + PCIE_IRQ3);

	/* clear pending bit */
	writel(irq[0], elbi_base + PCIE_IRQ0);
	writel(irq[1], elbi_base + PCIE_IRQ1);
	writel(irq[2], elbi_base + PCIE_IRQ2);
	writel(irq[3], elbi_base + PCIE_IRQ3);

	writel(PCIE_EP_IRQMASK, elbi_base + PCIE_IRQ0_EN);
	writel(PCIE_EP_IRQMASK1, elbi_base + PCIE_IRQ1_EN);
	writel(PCIE_EP_IRQMASK2, elbi_base + PCIE_IRQ2_EN);
	writel(PCIE_EP_IRQMASK3, elbi_base + PCIE_IRQ3_EN);

	return 0;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_link}
 * @purpose "Set inbound for doorbell"
 * @logic "Call pci_epc_set_bar to set inbound for doorbell"
 * @params
 * @param{in, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @param{in, bar_no, int, 0~7}
 * @endparam
 * @retval{ret, int, 0, <=0, not 0}
 */
static int exynos_pcie_ep_set_bar_inbound(struct exynos_ep_pcie *pcie_ep,
							int bar_no, int ch_no)
{
	struct dw_pcie *pci = pcie_ep->pci;
	struct dw_pcie_ep *ep = &pci->ep;
	struct pci_epc *epc = ep->epc;
	struct pci_epf_bar *epf_bar;

	int flags;
	int ret = 0;
	size_t size;
	dma_addr_t doorbell_addr;

	/* set phyinfo for PHY CAL */
	epf_bar = devm_kzalloc(pci->dev, sizeof(struct pci_epf_bar),
			GFP_KERNEL);
	if (!epf_bar)
		return -ENOMEM;

	size = 0; /* Dummy Parameter for Vendor Set Code */

	flags = PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_32;
	if (sizeof(dma_addr_t) == 0x8)
		flags |=  PCI_BASE_ADDRESS_MEM_TYPE_64;

	doorbell_addr = PCIE_DOORBELL_BASE_ADDRESS +
		(dma_addr_t)pcie_ep->ch_num * PCIE_DOORBELL_CH_OFFSET;

	epf_bar->barno = bar_no;
	epf_bar->size = size;
	epf_bar->flags = flags;
	epf_bar->phys_addr = doorbell_addr;

	pcie_ep->epf_bar = epf_bar;

	ret = pci_epc_set_bar(epc, 0, 0, pcie_ep->epf_bar);
	return ret;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Make EP driver ready for link"
 * @logic "Enable perst ineterrupt\n
 *	Phy power on\n
 *	Qch-sel disable."
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_ep_ready_establish_link(struct exynos_ep_pcie *pcie_ep)
{
	struct platform_device *pdev = pcie_ep->pdev;
	u32 val;

	/* interrupt enable getting the PREST irq from RC : SFR setting */
	exynos_pcie_ep_setup_interrupt(pcie_ep);

	pcie_ep->phy_ops.phy_all_pwrdn_clear(pcie_ep->ep_phyinfo);

	/* Qch-sel disable */
	val = readl(pcie_ep->elbi_base + PCIE_QCH_SEL);
	val &= ~(CLOCK_GATING_PMU_MASK | CLOCK_GATING_APB_MASK |
			CLOCK_GATING_AXI_MASK);
	writel(val, pcie_ep->elbi_base + PCIE_QCH_SEL);

	pcie_ep->linkup_ready = 1;
	dev_info(&pdev->dev, "[%s] Ready to establish link\n", __func__);

}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Check PCIe link status"
 * @logic "Read and print link status"
 * @params
 * @param{in, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @param{in, prests, unsigned int, 0x0~0x1F}
 * @endparam
 * @retval{linksts, int, 0, 0~255, <0}
 */
static unsigned int exynos_pcie_ep_check_linksts(struct exynos_ep_pcie *pcie_ep,
							unsigned int prests)
{
	struct platform_device *pdev = pcie_ep->pdev;
	unsigned int linksts =
		readl(pcie_ep->elbi_base + PCIE_ELBI_RDLH_LINKUP) & 0xff;

	if (linksts != prests)
		dev_info(&pdev->dev, "%s: (0x%x)\n", __func__, linksts);

	return linksts;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_link}
 * @purpose "Set SR-IOV"
 * @logic "Set SR-IOV "
 * @params
 * @param{in, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_pcie_ep_sriov_setting(struct exynos_ep_pcie *pcie_ep)
{
	struct dw_pcie *pci = pcie_ep->pci;
	u16 val;

	dev_info(pci->dev, "[%s]\n", __func__);

	dw_pcie_dbi_ro_wr_en(pci);
	dw_pcie_writew_dbi(pci, PCIE_SRIOV_CAP_POS + PCI_SRIOV_VF_OFFSET, 0x8);
	dw_pcie_writew_dbi(pci, PCIE_SRIOV_CAP_POS + PCI_SRIOV_VF_STRIDE, 0x1);
	dw_pcie_writew_dbi(pci, PCIE_SRIOV_CAP_POS + PCI_SRIOV_INITIAL_VF, 0x4);
	dw_pcie_writew_dbi(pci, PCIE_SRIOV_CAP_POS + PCI_SRIOV_TOTAL_VF, 0x4);
	val = dw_pcie_readw_dbi(pci, PCI_DEVICE_ID);
	dw_pcie_writew_dbi(pci, PCIE_SRIOV_CAP_POS + PCI_SRIOV_VF_DID,
								val+0x10);
	dw_pcie_dbi_ro_wr_dis(pci);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_link}
 * @purpose "Set PCIe BAR"
 * @logic "Set BAR information\n
 *	Set inbound ATU for BAR."
 * @params
 * @param{in, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_pcie_ep_bar_setting(struct exynos_ep_pcie *pcie_ep)
{
	struct dw_pcie *pci = pcie_ep->pci;
	u32 i;

	dev_info(pci->dev, "[%s]\n", __func__);

	dw_pcie_dbi_ro_wr_en(pci);
	/* to adjust PF, VF BAR0 size */
	dw_pcie_writel_dbi(pci, PCI_CLASS_REVISION,
					CLASS_ID<<0x8 | REVISION_ID);
	dw_pcie_writel_dbi(pci, PCIE_SHADOW_REG_POS + PCI_BASE_ADDRESS_0,
			0xffff);
	dw_pcie_writel_dbi(pci, PCIE_SHADOW_REG_POS + PCIE_SRIOV_CAP_POS
			+ PCI_SRIOV_BAR, 0xffff);

	for (i = 2; i < 2 + MAX_VF_NUM; i++) {
		dw_pcie_writel_dbi(pci, PCIE_SHADOW_REG_POS
				+ PCI_BASE_ADDRESS_0 + (i*4), 0);
		dw_pcie_writel_dbi(pci, PCIE_SHADOW_REG_POS + PCIE_SRIOV_CAP_POS
				+ PCI_SRIOV_BAR  + (i*4), 0);
	}
	dw_pcie_writel_dbi(pci, PCIE_SHADOW_REG_POS + PCI_ROM_ADDRESS, 0);
	dw_pcie_dbi_ro_wr_dis(pci);

	/* Set Inbound ATU for BAR0 */
	exynos_pcie_ep_set_bar_inbound(pcie_ep, 0, pcie_ep->ch_num);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_link}
 * @purpose "Initialize doorbell inerrupt"
 * @logic "Clear doorbell flag; enable doorbell"
 * @params
 * @param{in, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_pcie_ep_doorbell_init(struct exynos_ep_pcie *pcie_ep)
{
	struct platform_device *pdev = pcie_ep->pdev;
	int i;

	dev_dbg(&pdev->dev, "Initialize Doorbell IRQ\n");
	for (i = 0; i < MAX_VF_NUM; i++) {
		writel(0xffffffff, pcie_ep->doorbell_base +
			PCIE_DOORBELL0_CLEAR + i * PCIE_DOORBELL_OFFSET);
		writel(0xffffffff, pcie_ep->doorbell_base +
			PCIE_DOORBELL0_ENABLE + i * PCIE_DOORBELL_OFFSET);
	}
}

/* PCIe lane status checking function */
/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_link}
 * @purpose "Check lane status of PCIe ch"
 * @logic "Print lane status of PCIe ch"
 * @params
 * @param{in, ch_num, int, 0~5}
 * @endparam
 * @retval{-, int, 0, 0/1, not 0/1}
 */
int exynos_chk_ep_lane_status(int ch_num)
{
	struct exynos_ep_pcie *pcie_ep = &g_pcie_ep[ch_num];

	pr_info("[%s] PCIe ch%d lane status =%d\n", __func__, ch_num,
		pcie_ep->lane_status);

	return pcie_ep->lane_status;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_link}
 * @purpose "Update lane status in bifurcation"
 * @logic "Update lane status in bifurcation"
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @retval{-, int, 0, 0/1, not 0/1}
 * @noret
 */
static void exynos_update_ep_lane_status(struct exynos_ep_pcie *pcie_ep)
{
	struct platform_device *pdev = pcie_ep->pdev;
	struct device *dev = &pdev->dev;
	int other_lane_status;

	if (pcie_ep->ch_num % 2 == 0) {
		/* Current Link A */

		/* Current Link Status Update */
		pcie_ep->ep_phyinfo->linkA_status =
			3*(1 - pcie_ep->lane_status);

		other_lane_status = exynos_chk_ep_lane_status
							(pcie_ep->ch_num + 1);

		if (other_lane_status == -1) {
			/* Other link is RC */
			pcie_ep->ep_phyinfo->linkB_status =
					readl(pcie_ep->phy_pcs_base + 0x188);
		} else {
			/* Other link is EP */
			/* Other Link Status Update */
			pcie_ep->ep_phyinfo->linkB_status =
						3 * (1 - other_lane_status);
		}
	} else {
		/* Current Link B */

		other_lane_status =	exynos_chk_ep_lane_status
							(pcie_ep->ch_num - 1);

		if (other_lane_status == -1) {
			/* Other link is RC */
			pcie_ep->ep_phyinfo->linkA_status =
					readl(pcie_ep->phy_pcs_base + 0x988);
		} else {
			/* Other link is EP */
			/* Other Link Status Update */
			pcie_ep->ep_phyinfo->linkA_status =
						3 * (1 - other_lane_status);
		}

		/* Current Link Status Update */
		pcie_ep->ep_phyinfo->linkB_status =
					3 * (1 - pcie_ep->lane_status);
	}
	dev_info(dev, "#### linkA:%d linkB:%d\n",
					pcie_ep->ep_phyinfo->linkA_status,
					pcie_ep->ep_phyinfo->linkB_status);

}


/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_link}
 * @purpose "Link up PCIe link"
 * @logic "Check link in ready\n
 *	Initialize phy\n
 *	Setup EP\n
 *	Setup interrupt\n
 *	Doorbell initialize\n
 *	Enable LTSSM\n
 *	Check link status."
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @retval{linksts, int, 0, 0~0x91, not 0~0x91}
 */
static unsigned int exynos_pcie_ep_link_start(struct exynos_ep_pcie *pcie_ep)
{
	struct platform_device *pdev = pcie_ep->pdev;
	unsigned int linksts = 0;
	int cnt = 2000;

start_linkup:
	if (pcie_ep->linkup_ready == 1) {
		/* 1. phy initialize */
		pcie_ep->phy_ops.phy_config(pcie_ep->ep_phyinfo);
		/* 2. setup EP */
		exynos_pcie_setup_ep(pcie_ep);
		exynos_pcie_ep_bar_setting(pcie_ep);
		exynos_pcie_ep_sriov_setting(pcie_ep);
		/* 3. setup interrupt */
		exynos_pcie_ep_setup_interrupt(pcie_ep);
		/* 4. Doorbel irq init */
		exynos_pcie_ep_doorbell_init(pcie_ep);
		/* 5. pcie atomic op */
		exynos_pcie_ep_atomic_op(pcie_ep, 1);
		/* 6. pcie_setiATU */
		/* 7. enable LTSSM */
		exynos_pcie_ep_set_ltssm(pcie_ep, 1);

		while (cnt--) {
			linksts = exynos_pcie_ep_check_linksts(pcie_ep,
								linksts);
			mdelay(1);
			if ((linksts & 0x1f) == S_L0)
				break;
		}
	} else {
		dev_info(&pdev->dev,
			"PCIe EP didn't ready to start establish link\n");
		exynos_ep_ready_establish_link(pcie_ep);
		goto start_linkup;
	}

	return linksts;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_link}
 * @purpose "Start PCIe link power down"
 * @logic "Change ep state to STATE_LINK_DOWN_TRY\n
 *	Phy all power down\n
 *	Qch-sel disable."
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_pcie_ep_power_down(struct exynos_ep_pcie *pcie_ep)
{
	struct platform_device *pdev = pcie_ep->pdev;
	u32 val;

	dev_info(&pdev->dev, "[%s]\n", __func__);

	pcie_ep->state = STATE_LINK_DOWN_TRY;

	/* phy all power down */
	pcie_ep->phy_ops.phy_all_pwrdn(pcie_ep->ep_phyinfo);

	/* Qch-sel disable for receiving PERST INT*/
	val = readl(pcie_ep->elbi_base + PCIE_QCH_SEL);
	val &= ~(CLOCK_GATING_PMU_MASK | CLOCK_GATING_APB_MASK |
			CLOCK_GATING_AXI_MASK);
	writel(val, pcie_ep->elbi_base + PCIE_QCH_SEL);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_link}
 * @purpose "Stop PCIe link"
 * @logic "Change ep state to STATE_LINK_DOWN_TRY\n
 *	If using sris, chage perst_in_mux setting\n
 *	Disable LTSSM\n
 *	Call exynos_pcie_ep_power_down."
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @retval{-, int, 0, 0, not 0}
 */
static int exynos_pcie_ep_link_stop(struct exynos_ep_pcie *pcie_ep)
{
	struct platform_device *pdev = pcie_ep->pdev;
	u32 val;

	dev_info(&pdev->dev, "[%s]\n", __func__);

	pcie_ep->state = STATE_LINK_DOWN_TRY;

	if (pcie_ep->use_sris) {
		/* set perstn_in_mux from SYSREG to PAD(default)*/
		val = readl(pcie_ep->sysreg_ctrl_base);
		val |= (0x1 << 5);
		writel(val, pcie_ep->sysreg_ctrl_base);
	}

	/* LTSSM disable */
	exynos_pcie_ep_set_ltssm(pcie_ep, 0);

	pcie_ep->linkup_ready = 0;

	exynos_pcie_ep_power_down(pcie_ep);

	return 0;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_link}
 * @purpose "force clear BAR"
 * @logic "if cannot access DBI, Clear EP BAR"
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @retval{-, int, 0, 0, not 0}
 */
static int exynos_pcie_ep_clear_bar_force
							(struct exynos_ep_pcie *pcie_ep)
{
	struct platform_device *pdev = pcie_ep->pdev;
	struct dw_pcie *pci = pcie_ep->pci;
	struct dw_pcie_ep *ep = &pci->ep;
	u32 atu_index;

	if (pcie_ep->epf_bar->barno < 0) {
		dev_err(&pdev->dev, "pcie_ep->epf_bar->barno is neg index\n");
		return -EINVAL;
	}

	atu_index = ep->bar_to_atu[pcie_ep->epf_bar->barno];

	/* Clear ib window only if force dislink occur */
	/* (Cannot access DBI)                         */

	dev_info(&pdev->dev, "[%s]\n", __func__);

	clear_bit(atu_index, ep->ib_window_map);

	return 0;
}


/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Handle irq0 event"
 * @logic "In case of PM_TURNOFF, start link stop"
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @param{in, value, u32, 0~0xffffffff}
 * @param{in, mask, u32, 0~0xffffffff}
 * @endparam
 * @noret
 */
void exynos_pcie_ep_irq0_evt(struct exynos_ep_pcie *pcie_ep,
						u32 value, u32 mask)
{
	struct platform_device *pdev = pcie_ep->pdev;
	u32 val = 0x0;

	if ((value & mask) != mask)
		return;
	switch (mask) {
	case PM_TURNOFF:
		dev_info(&pdev->dev, "IRQ0: PM_TURNOFF\n");
		dev_info(&pdev->dev, "Check Link status\n");
		while (1) {
			val = readl(pcie_ep->elbi_base + PCIE_ELBI_RDLH_LINKUP);
			val &= 0x1f;
			dev_info(&pdev->dev, "Link sts: 0x%x\n", val);
			if (val == S_L2_IDLE) {
				exynos_pcie_ep_link_stop(pcie_ep);
				exynos_ep_ready_establish_link(pcie_ep);
				break;
			}
		}
		break;
	default:
		dev_dbg(&pdev->dev, "IRQ0: unknown(0x%x)\n", mask);
			break;
	}
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Handle irq1 event"
 * @logic "If value is PERST_N_1, start link up process\n
 *	If value is linkdown, start link stop process or handle exception."
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @param{in, value, u32, 0~0xffffffff}
 * @param{in, mask, u32, 0~0xffffffff}
 * @endparam
 * @noret
 */
void exynos_pcie_ep_irq1_evt(struct exynos_ep_pcie *pcie_ep,
						u32 value, u32 mask)
{
	struct platform_device *pdev = pcie_ep->pdev;

	switch (mask) {
	case PERST_N_0:
		dev_info(&pdev->dev, "IRQ1: PERST_N_0\n");
		break;
	case PERST_N_1:
		/* if PERST event is occurred, */
		/* PCIe EP starts several sequence to establish link */
		dev_info(&pdev->dev, "IRQ1: PERST_N_1\n");
		if (pcie_ep->state == STATE_LINK_DOWN) {
			pcie_ep->state = STATE_LINK_UP_TRY;
			queue_work(pcie_ep->pcie_ep_wq, &pcie_ep->irq1_evt_work.work);
		}
		break;
	case LINK_DOWN:
		dev_info(&pdev->dev, "IRQ1: LINK_DOWN !!!\n");

		if (pcie_ep->state == STATE_LINK_UP ||
				pcie_ep->state == STATE_LINK_UP_TRY) {

			exynos_pcie_ep_clear_bar_force(pcie_ep);

			complete_all(&pcie_ep->wdma0);
			complete_all(&pcie_ep->rdma0);

			reinit_completion(&pcie_ep->wdma0);
			reinit_completion(&pcie_ep->rdma0);

			complete_all(&pcie_ep->wdma1);
			complete_all(&pcie_ep->wdma2);
			complete_all(&pcie_ep->wdma3);

			complete_all(&pcie_ep->rdma1);
			complete_all(&pcie_ep->rdma2);
			complete_all(&pcie_ep->rdma3);

			reinit_completion(&pcie_ep->wdma1);
			reinit_completion(&pcie_ep->wdma2);
			reinit_completion(&pcie_ep->wdma3);

			reinit_completion(&pcie_ep->rdma1);
			reinit_completion(&pcie_ep->rdma2);
			reinit_completion(&pcie_ep->rdma3);

			pcie_ep->state = STATE_LINK_DOWN_ABNORMAL;
			dev_info(&pdev->dev,
				"IRQ1: STATE_LINK_DOWN_ABNORMAL\n");

			/* lane status off */
			pcie_ep->lane_status = 0;

			/* Register Link Reset Work Queue */
			pcie_ep->work_exception = WORK_LINK_RESET;
			queue_work(pcie_ep->pcie_ep_wq_exception,
				&pcie_ep->exception_work.work);
		} else {
			pcie_ep->state = STATE_LINK_DOWN;
		}
		break;
	default:
		dev_dbg(&pdev->dev, "IRQ1: unknown(0x%x)\n", mask);
		break;
	}
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Handle irq2 event"
 * @logic "If value is PM_LINKST_IN_L2, print link status"
 * @params
 * @param{in, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @param{in, value, u32, 0~0xffffffff}
 * @param{in, mask, u32, 0~0xffffffff}
 * @endparam
 * @noret
 */
void exynos_pcie_ep_irq2_evt(struct exynos_ep_pcie *pcie_ep,
						u32 value, u32 mask)
{
	struct platform_device *pdev = pcie_ep->pdev;
	u32 val = 0x0;

	if ((value & mask) != mask)
		return;
	switch (mask) {
	case PM_LINKST_IN_L2:
		dev_info(&pdev->dev, "IRQ2: PM_LINKST_IN_L2\n");
		val = readl(pcie_ep->elbi_base + PCIE_ELBI_RDLH_LINKUP) & 0x1f;
		dev_info(&pdev->dev, "Link sts: 0x%x\n", val);
		break;
	default:
		dev_dbg(&pdev->dev, "IRQ2: unknown(0x%x)\n", mask);
		break;
	}
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Handle irq3 event"
 * @logic "Check which dma(write ch0 or read ch0) is completed\n
 *	Make completion for dma."
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @param{in, value, u32, 0~0xffffffff}
 * @param{in, mask, u32, 0~0xffffffff}
 * @endparam
 * @noret
 */
void exynos_pcie_ep_irq3_evt(struct exynos_ep_pcie *pcie_ep,
						u32 value, u32 mask)
{
	struct platform_device *pdev = pcie_ep->pdev;

	if ((value & mask) != mask)
		return;
	switch (mask) {
	case WR_CH0:
		dev_dbg(&pdev->dev, "IRQ3: Write DMA ch0 DONE!\n");
		complete(&pcie_ep->wdma0);
		break;
	case RD_CH0:
		dev_dbg(&pdev->dev, "IRQ3: Read DMA ch0 DONE!\n");
		complete(&pcie_ep->rdma0);
		break;
	default:
		dev_dbg(&pdev->dev, "IRQ3: unknown(0x%x)\n", mask);
		break;
	}
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Handle doorbell event"
 * @logic "Start test work queue with WORK_DOORBELL command"
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @noret
 */
void exynos_pcie_ep_doorbell_evt(struct exynos_ep_pcie *pcie_ep)
{
	struct platform_device *pdev = pcie_ep->pdev;

	dev_dbg(&pdev->dev, "DOORBELL: 0x%x\n", pcie_ep->doorbell);

	/* Received Test Read Request */
	pcie_ep->size = pcie_ep->doorbell;

	if (pcie_ep->size > MAX_TEST_WR_SIZE) {
		dev_info(&pdev->dev, "Maximum read size exceeded, fix to %x\n",
							MAX_TEST_WR_SIZE);
		pcie_ep->size = MAX_TEST_WR_SIZE;
	}

	pcie_ep->work_cmd = WORK_DOORBELL;
	queue_work(pcie_ep->pcie_ep_wq, &pcie_ep->test_work.work);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Handler for PCIe interrupt1"
 * @logic "Start link up; check the link up result\n
 *	If result is success, change state STATE_LINK_UP and lane status \n
 *	If using sris, chage perst_in_mux setting."
 * @params
 * @param{in/out, work, struct ::work_struct *, not NULL}
 * @endparam
 * @noret
 */
void exynos_pcie_ep_irq1_handler(struct work_struct *work)
{
	struct exynos_ep_pcie *pcie_ep =
		container_of(work, struct exynos_ep_pcie, irq1_evt_work.work);
	struct platform_device *pdev = pcie_ep->pdev;
	u32 irq1_status = pcie_ep->irq1_status;
	unsigned int ret;
	u32 val;

	dev_dbg(&pdev->dev, "PCIe EP IRQ1 : 0x%x\n", irq1_status);

	ret = exynos_pcie_ep_link_start(pcie_ep);
	dev_info(&pdev->dev, "ret: 0x%x\n", ret);
	if ((ret & 0x1f) == S_L0) {
		dev_info(&pdev->dev, "IRQ1: LINK UP SUCCESS\n");
		pcie_ep->state = STATE_LINK_UP;

		pcie_ep->lane_status = 1;

		if (pcie_ep->use_sris) {
			/* set perstn_in_mux from PAD(default) to SYSREG */
			val = readl(pcie_ep->sysreg_ctrl_base);
			val &= ~(0x1 << 5);
			writel(val, pcie_ep->sysreg_ctrl_base);
		}
	} else {
		/* link up fail case */
		pcie_ep->state = STATE_LINK_DOWN;
		dev_info(&pdev->dev, "IRQ1: LINK UP FAIL\n");
	}
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Handler for PCIe interrupt"
 * @logic "If ep state is STATE_LINK_DOWN, power on all phy\n
 *	Read irq0~4 status and clear interrupt\n
 *	If irq1 status is intended value, call irq1 event\n
 *	If irq2 status is PM_LINKST_IN_L2, start link stop\n
 *	If irq3 status is intended value, call irq3 event\n
 *	If doorbell interrupt occurred, call doorbell event."
 * @params
 * @param{in, irq, int, >0}
 * @param{in/out, arg, void *, not NULL}
 * @endparam
 * @retval{-, irqreturn_t, 0, IRQ_HANDLED, not IRQ_HANDLED}
 */
static irqreturn_t exynos_pcie_ep_irq_handler(int irq, void *arg)
{
	struct exynos_ep_pcie *pcie_ep = arg;
	struct platform_device *pdev = pcie_ep->pdev;
	void __iomem	*elbi_base = pcie_ep->elbi_base;
	int loop;
	u32 mask;

	if (pcie_ep->state == STATE_LINK_DOWN)
		pcie_ep->phy_ops.phy_all_pwrdn_clear(pcie_ep->ep_phyinfo);

	/* read the status of each IRQ and do clear first */
	/* handle IRQ0 interrupt */
	pcie_ep->irq0_status = readl(elbi_base + PCIE_IRQ0);
	writel(pcie_ep->irq0_status, elbi_base + PCIE_IRQ0);

	/* handle IRQ1 interrupt */
	pcie_ep->irq1_status = readl(elbi_base + PCIE_IRQ1);
	writel(pcie_ep->irq1_status, elbi_base + PCIE_IRQ1);

	/* handle IRQ2 interrupt */
	pcie_ep->irq2_status = readl(elbi_base + PCIE_IRQ2);
	writel(pcie_ep->irq2_status, elbi_base + PCIE_IRQ2);

	/* handle IRQ3 interrupt */
	pcie_ep->irq3_status = readl(elbi_base + PCIE_IRQ3);
	writel(pcie_ep->irq3_status, elbi_base + PCIE_IRQ3);

	if (pcie_ep->irq0_status & PM_TURNOFF) {
		dev_info(&pdev->dev, "IRQ0: PM_TURNOFF\n");
	}

	if (pcie_ep->irq1_status & PCIE_EP_IRQMASK) {
		dev_dbg(&pdev->dev, "[%s] irq1_status : 0x%x\n",
			__func__, pcie_ep->irq1_status);
		for (loop = 0; loop < 32; loop++) {
			mask = (1 << loop);
			if ((pcie_ep->irq1_status & mask) == mask)
				exynos_pcie_ep_irq1_evt(pcie_ep,
						pcie_ep->irq1_status, mask);
		}
	}

	if (pcie_ep->irq2_status & PM_LINKST_IN_L2) {
		dev_info(&pdev->dev, "IRQ2: PM_LINKST_IN_L2\n");

		/* lane status off */
		pcie_ep->lane_status = 0;

		if (pcie_ep->use_bifurcation)
			exynos_update_ep_lane_status(pcie_ep);

		exynos_pcie_ep_link_stop(pcie_ep);
	}

	return IRQ_HANDLED;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Handle doorbell interrupt"
 * @logic "Find doorbell number from irq\n
 *	Read doorbell interrupt status and clear\n
 *	Call doorbell event."
 * @params
 * @param{in, irq, int, >0}
 * @param{in/out, arg, void *, not NULL}
 * @endparam
 * @retval{-, irqreturn_t, 0, IRQ_HANDLED/IRQ_NONE, IRQ_NONE}
 */
static irqreturn_t exynos_pcie_ep_doorbell_irq_handler(int irq, void *arg)
{
	struct exynos_ep_pcie *pcie_ep = arg;
	void __iomem	*doorbell_base = pcie_ep->doorbell_base;

	struct platform_device *pdev = pcie_ep->pdev;
	struct device *dev = &pdev->dev;

	int i, db_no = -1;

	for (i = 0; i < MAX_VF_NUM; i++) {
		if (irq == pcie_ep->db_irq[i]) {
			db_no = i;
			break;
		}
	}

	if (db_no == -1) {
		dev_err(dev, "Invalid Doorbell IRQ: %d\n", irq);
		return IRQ_NONE;
	}

	pcie_ep->doorbell = readl(doorbell_base + PCIE_DOORBELL0_STATUS +
		db_no * PCIE_DOORBELL_OFFSET);
	writel(0xffffffff, doorbell_base + PCIE_DOORBELL0_CLEAR +
		db_no * PCIE_DOORBELL_OFFSET);

	if (pcie_ep->doorbell & PCIE_EP_IRQMASK)
		exynos_pcie_ep_doorbell_evt(pcie_ep);

	return IRQ_HANDLED;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Handle dma0 interrupt"
 * @logic "Read and clear write dma0 status\n
 *	Check write is done or aborted and make completion\n
 *	Read and clear read dma0 status\n
 *	Check read is done or aborted and make completion."
 * @params
 * @param{in, irq, int, >0}
 * @param{in/out, arg, void *, not NULL}
 * @endparam
 * @retval{-, irqreturn_t, 0, IRQ_HANDLED/IRQ_NONE, IRQ_NONE}
 */
static irqreturn_t exynos_pcie_ep_dma0_irq_handler(int irq, void *arg)
{
	struct exynos_ep_pcie *pcie_ep = arg;
	struct platform_device *pdev = pcie_ep->pdev;
	struct device *dev = &pdev->dev;
	u32 val;
	u32 mask;
	int dma_offset;

	dma_offset = 0;

	val = readl(pcie_ep->dma_base + PCIE_DMA_WR_INT_STS);
	writel(val & PCIE_DMA_STS_IRQ0_MASK,
		pcie_ep->dma_base + PCIE_DMA_WR_INT_CLEAR);

	if (val != 0) {
		/* WR done Check */
		mask = (PCIE_DMA_STS_2CH_MASK << dma_offset);
		if (val & mask) {
			complete(&pcie_ep->wdma0);
			dev_info(dev, "Write done. DMA0 completion.\n");

			return IRQ_HANDLED;
		}

		/* WR abort Check */
		mask = (PCIE_DMA_STS_2CH_MASK << (dma_offset + 16));
		if (val & mask) {
			complete(&pcie_ep->wdma0);
			dev_info(dev, "Write Aborted. DMA0 completion.\n");

			return IRQ_HANDLED;
		}

		dev_info(dev, "Can not found write int flag\n");
		return IRQ_NONE;
	}

	val = readl(pcie_ep->dma_base + PCIE_DMA_RD_INT_STS);
	writel(val  & PCIE_DMA_STS_IRQ0_MASK,
		pcie_ep->dma_base + PCIE_DMA_RD_INT_CLEAR);

	if (val != 0) {
		/* RD done Check */
		mask = (PCIE_DMA_STS_2CH_MASK << dma_offset);
		if (val & mask) {
			complete(&pcie_ep->rdma0);
			dev_info(dev, "Read done. DMA0 completion.\n");

			return IRQ_HANDLED;
		}

		/* RD abort Check */
		mask = (PCIE_DMA_STS_2CH_MASK << (dma_offset+16));
		if (val & mask) {
			complete(&pcie_ep->rdma0);
			dev_info(dev, "Read Aborted. DMA0 completion.\n");

			return IRQ_HANDLED;
		}
		dev_err(dev, "Can not found read int flag\n");
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Handle dma1 interrupt"
 * @logic "Read and clear write dma1 status\n
 *	Check write is done or aborted and make completion\n
 *	Read and clear read dma1 status\n
 *	Check read is done or aborted and make completion."
 * @params
 * @param{in, irq, int, >0}
 * @param{in/out, arg, void *, not NULL}
 * @endparam
 * @retval{-, irqreturn_t, 0, IRQ_HANDLED/IRQ_NONE, IRQ_NONE}
 */
static irqreturn_t exynos_pcie_ep_dma1_irq_handler(int irq, void *arg)
{
	struct exynos_ep_pcie *pcie_ep = arg;
	struct platform_device *pdev = pcie_ep->pdev;
	struct device *dev = &pdev->dev;
	u32 val;
	u32 mask;
	int dma_offset;

	dma_offset = 2;


	val = readl(pcie_ep->dma_base + PCIE_DMA_WR_INT_STS);
	writel(val & PCIE_DMA_STS_IRQ1_MASK,
		pcie_ep->dma_base + PCIE_DMA_WR_INT_CLEAR);

	if (val != 0) {
		/* WR done Check */
		mask = (PCIE_DMA_STS_2CH_MASK << dma_offset);
		if (val & mask) {
			complete(&pcie_ep->wdma1);
			dev_info(dev, "Write done. DMA1 completion.\n");

			return IRQ_HANDLED;
		}

		/* WR abort Check */
		mask = (PCIE_DMA_STS_2CH_MASK << (dma_offset + 16));
		if (val & mask) {
			complete(&pcie_ep->wdma1);
			dev_info(dev, "Write Aborted. DMA1 completion.\n");

			return IRQ_HANDLED;
		}

		dev_err(dev, "Can not found write int flag\n");
		return IRQ_NONE;
	}

	val = readl(pcie_ep->dma_base + PCIE_DMA_RD_INT_STS);
	writel(val  & PCIE_DMA_STS_IRQ1_MASK,
		pcie_ep->dma_base + PCIE_DMA_RD_INT_CLEAR);

	if (val != 0) {
		/* RD done Check */
		mask = (PCIE_DMA_STS_2CH_MASK << dma_offset);
		if (val & mask) {
			complete(&pcie_ep->rdma1);
			dev_info(dev, "Read done. DMA1 completion.\n");

			return IRQ_HANDLED;
		}

		/* RD abort Check */
		mask = (PCIE_DMA_STS_2CH_MASK << (dma_offset+16));
		if (val & mask) {
			complete(&pcie_ep->rdma1);
			dev_info(dev, "Read Aborted. DMA1 completion.\n");

			return IRQ_HANDLED;
		}
		dev_err(dev, "Can not found read int flag\n");
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Handle dma2 interrupt"
 * @logic "Read and clear write dma2 status\n
 *	Check write is done or aborted and make completion\n
 *	Read and clear read dma2 status\n
 *	Check read is done or aborted and make completion."
 * @params
 * @param{in, irq, int, >0}
 * @param{in/out, arg, void *, not NULL}
 * @endparam
 * @retval{-, irqreturn_t, 0, IRQ_HANDLED/IRQ_NONE, IRQ_NONE}
 */
static irqreturn_t exynos_pcie_ep_dma2_irq_handler(int irq, void *arg)
{
	struct exynos_ep_pcie *pcie_ep = arg;
	struct platform_device *pdev = pcie_ep->pdev;
	struct device *dev = &pdev->dev;
	u32 val;
	u32 mask;
	int dma_offset;

	dma_offset = 4;

	val = readl(pcie_ep->dma_base + PCIE_DMA_WR_INT_STS);
	writel(val & PCIE_DMA_STS_IRQ2_MASK,
		pcie_ep->dma_base + PCIE_DMA_WR_INT_CLEAR);

	if (val != 0) {
		/* WR done Check */
		mask = (PCIE_DMA_STS_2CH_MASK << dma_offset);
		if (val & mask) {
			complete(&pcie_ep->wdma2);
			dev_info(dev, "Write done. DMA2 completion.\n");

			return IRQ_HANDLED;
		}

		/* WR abort Check */
		mask = (PCIE_DMA_STS_2CH_MASK << (dma_offset + 16));
		if (val & mask) {
			complete(&pcie_ep->wdma2);
			dev_info(dev, "Write Aborted. DMA2 completion.\n");

			return IRQ_HANDLED;
		}

		dev_err(dev, "Can not found write int flag\n");
		return IRQ_NONE;
	}

	val = readl(pcie_ep->dma_base + PCIE_DMA_RD_INT_STS);
	writel(val  & PCIE_DMA_STS_IRQ2_MASK,
		pcie_ep->dma_base + PCIE_DMA_RD_INT_CLEAR);

	if (val != 0) {
		/* RD done Check */
		mask = (PCIE_DMA_STS_2CH_MASK << dma_offset);
		if (val & mask) {
			complete(&pcie_ep->rdma2);
			dev_info(dev, "Read done. DMA2 completion.\n");

			return IRQ_HANDLED;
		}

		/* RD abort Check */
		mask = (PCIE_DMA_STS_2CH_MASK << (dma_offset+16));
		if (val & mask) {
			complete(&pcie_ep->rdma2);
			dev_info(dev, "Read Aborted. DMA2 completion.\n");

			return IRQ_HANDLED;
		}
		dev_err(dev, "Can not found read int flag\n");
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Handle dma3 interrupt"
 * @logic "Read and clear write dma3 status\n
 *	Check write is done or aborted and make completion\n
 *	Read and clear read dma3 status\n
 *	Check read is done or aborted and make completion."
 * @params
 * @param{in, irq, int, >0}
 * @param{in/out, arg, void *, not NULL}
 * @endparam
 * @retval{-, irqreturn_t, 0, IRQ_HANDLED/IRQ_NONE, IRQ_NONE}
 */
static irqreturn_t exynos_pcie_ep_dma3_irq_handler(int irq, void *arg)
{
	struct exynos_ep_pcie *pcie_ep = arg;
	struct platform_device *pdev = pcie_ep->pdev;
	struct device *dev = &pdev->dev;
	u32 val;
	u32 mask;
	int dma_offset;

	dma_offset = 6;

	val = readl(pcie_ep->dma_base + PCIE_DMA_WR_INT_STS);
	writel(val & PCIE_DMA_STS_IRQ3_MASK,
		pcie_ep->dma_base + PCIE_DMA_WR_INT_CLEAR);

	if (val != 0) {
		/* WR done Check */
		mask = (PCIE_DMA_STS_2CH_MASK << dma_offset);
		if (val & mask) {
			complete(&pcie_ep->wdma3);
			dev_info(dev, "Write done. DMA3 completion.\n");

			return IRQ_HANDLED;
		}

		/* WR abort Check */
		mask = (PCIE_DMA_STS_2CH_MASK << (dma_offset + 16));
		if (val & mask) {
			complete(&pcie_ep->wdma3);
			dev_info(dev, "Write Aborted. DMA3 completion.\n");

			return IRQ_HANDLED;
		}

		dev_err(dev, "Can not found write int flag\n");
		return IRQ_NONE;
	}

	val = readl(pcie_ep->dma_base + PCIE_DMA_RD_INT_STS);
	writel(val  & PCIE_DMA_STS_IRQ3_MASK,
		pcie_ep->dma_base + PCIE_DMA_RD_INT_CLEAR);

	if (val != 0) {
		/* RD done Check */
		mask = (PCIE_DMA_STS_2CH_MASK << dma_offset);
		if (val & mask) {
			complete(&pcie_ep->rdma3);
			dev_info(dev, "Read done. DMA3 completion.\n");

			return IRQ_HANDLED;
		}

		/* RD abort Check */
		mask = (PCIE_DMA_STS_2CH_MASK << (dma_offset+16));
		if (val & mask) {
			complete(&pcie_ep->rdma3);
			dev_info(dev, "Read Aborted. DMA3 completion.\n");

			return IRQ_HANDLED;
		}
		dev_err(dev, "Can not found read int flag\n");
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_init}
 * @purpose "Parse data from dt"
 * @logic "Parse data from dt and save to drive structure."
 * @params
 * @param{in, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @retval{-, int, 0, int, not 0}
 */
static int exynos_pcie_ep_parse_dt(struct exynos_ep_pcie *pcie_ep)
{
	struct platform_device *pdev = pcie_ep->pdev;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	const char *use_bifuraction;
	const char *use_sris;

	if (of_property_read_u32(np, "pmu-phy-offset",
					&pcie_ep->pmu_phy_offset)) {
		dev_err(dev, "Failed to parse the pmu phy offset\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "pcie-clk-num",
					&pcie_ep->pcie_clk_num)) {
		dev_err(dev, "Failed to parse the number of pcie clock\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "phy-clk-num",
					&pcie_ep->phy_clk_num)) {
		dev_err(dev, "Failed to parse the number of phy clock\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "max-link-speed",
				&pcie_ep->max_link_speed)) {
		dev_warn(dev, "MAX Link Speed is NOT defined...(GEN1)\n");
		/* Default Link Speet is GEN1 */
		pcie_ep->max_link_speed = LINK_SPEED_GEN1;
	}

	if (of_property_read_u32(np, "num-lanes",
				&pcie_ep->lanes_num)) {
		dev_err(dev, "failed to parse a number of lanes\n");
		return -EINVAL;
	}

	/* use_bifurcation => 1: bifurcation, 0:aggregation */
	if (!of_property_read_string(np, "use-bifurcation", &use_bifuraction)) {
		if (!strcmp(use_bifuraction, "true")) {
			pcie_ep->use_bifurcation = true;
			dev_info(dev, "Bifurcation mode is ENABLED.\n");
		} else if (!strcmp(use_bifuraction, "false")) {
			pcie_ep->use_bifurcation = false;
		} else {
			dev_warn(dev, "Invalid use-bifurcation value");
			dev_warn(dev, "Set to default -> false)\n");
			pcie_ep->use_bifurcation = false;
		}
	} else {
		pcie_ep->use_bifurcation = true;
	}

	if (!of_property_read_string(np, "use-sris", &use_sris)) {
		if (!strcmp(use_sris, "true")) {
			pcie_ep->use_sris = true;
			dev_info(dev, "SRIS is ENABLED.\n");
		} else if (!strcmp(use_sris, "false")) {
			pcie_ep->use_sris = false;
		} else {
			dev_warn(dev, "Invalid use-sris value");
			dev_warn(dev, "Set to default -> false)\n");
			pcie_ep->use_sris = false;
		}
	} else {
		pcie_ep->use_sris = true;
	}

	pcie_ep->pmureg = syscon_regmap_lookup_by_phandle(np,
					"samsung,syscon-phandle");
	if (IS_ERR(pcie_ep->pmureg)) {
		dev_err(dev, "syscon regmap lookup failed.\n");
		return PTR_ERR(pcie_ep->pmureg);
	}

	pcie_ep->sysreg = syscon_regmap_lookup_by_phandle(np,
			"samsung,sysreg-phandle");
	/* Check definitions to access SYSREG in DT*/
	if (IS_ERR(pcie_ep->sysreg) && IS_ERR(pcie_ep->sysreg_base)) {
		dev_err(dev, "SYSREG is not defined.\n");
		return PTR_ERR(pcie_ep->sysreg);
	}

	if (of_property_read_u32(np, "s2mpu-base", &pcie_ep->s2mpu_base))
		pcie_ep->s2mpu_base = 0;

	return 0;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_init}
 * @purpose "Get clock info from dt"
 * @logic "Get clock info from dt."
 * @params
 * @param{in, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @retval{-, int, 0, 0/-ENODEV, -ENODEV}
 */
static int exynos_pcie_ep_clk_get(struct exynos_ep_pcie *pcie_ep)
{
	struct exynos_pcie_ep_clks *clks = &pcie_ep->clks;
	struct platform_device *pdev = pcie_ep->pdev;
	struct device *dev = &pdev->dev;
	int pcie_clk_num = pcie_ep->pcie_clk_num;
	int pcie_phyclk_num = pcie_ep->phy_clk_num;
	int i, total_clk_num, phy_cnt;

	total_clk_num = pcie_clk_num + pcie_phyclk_num;

	/*
	 * CAUTION - PCIe and phy clock have to define in order.
	 * You must define related PCIe clock first in DT.
	 */
	for (i = 0; i < total_clk_num; i++) {
		if (i < pcie_clk_num) {
			clks->pcie_clks[i] = of_clk_get(dev->of_node, i);
			if (IS_ERR(clks->pcie_clks[i])) {
				dev_err(dev, "Failed to get pcie clock\n");
				return -ENODEV;
			}
		} else {
			phy_cnt = i - pcie_clk_num;
			if (phy_cnt < 0) {
				dev_err(dev, "phy_cnt value is negative\n");
				return -EINVAL;
			}
			clks->phy_clks[phy_cnt] =
				of_clk_get(dev->of_node, i);
			if (IS_ERR(clks->phy_clks[phy_cnt])) {
				dev_err(dev,
					"Failed to get PCIe EP PHY clock\n");
				return -ENODEV;
			}
		}
	}

	return 0;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Perform MSI test"
 * @logic "Allocate pci address\n
 *	Get msi address\n
 *	Map msi address to pci address\n
 *	Send msi."
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @param{in, msi_data, int, >=0}
 * @endparam
 * @retval{ret, int, 0, -ERANGE~0, <0}
 */
int exynos_pcie_ep_msi(struct exynos_ep_pcie *pcie_ep, int msi_data)
{
	struct platform_device *pdev = pcie_ep->pdev;
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci = pcie_ep->pci;
	struct dw_pcie_ep *ep = &pci->ep;
	struct pci_epc *epc = ep->epc;

	int ret = 0;
	void __iomem *dst_addr;

	phys_addr_t phys_addr;
	u64 umsi_pci_addr;
	u64 lmsi_pci_addr;
	u64 pci_addr;
	u32 size = 0x100;

	dst_addr = pci_epc_mem_alloc_addr(epc, &phys_addr, size);

	if (!dst_addr) {
		dev_err(dev, "failed to allocate address\n");
		ret = -ENOMEM;
		return ret;
	}

	lmsi_pci_addr = dw_pcie_readl_dbi(pci, 0x54);
	umsi_pci_addr = dw_pcie_readl_dbi(pci, 0x58);
	pci_addr = (umsi_pci_addr << 32) + lmsi_pci_addr;

	dev_info(dev, "umsi: 0x%llx , lmsi: 0x%llx\n",
			umsi_pci_addr, lmsi_pci_addr);
	dev_info(dev, "pci_addr: 0x%llx\n", pci_addr);

	ret = pci_epc_map_addr(epc, 0, 0, phys_addr, pci_addr, size);
	if (ret) {
		dev_err(dev, "failed to map address\n");
		ret = -ENOMEM;
		goto err;
	}

	dev_info(dev, "write MSI: %x\n", msi_data);
	writel(msi_data, dst_addr);

	pci_epc_unmap_addr(epc, 0, 0, phys_addr);
err:
	pci_epc_mem_free_addr(epc, phys_addr, dst_addr, size);

	return ret;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Perform dma write test"
 * @logic "Allocation pci address\n
 *	Set outbound\n
 *	Allocation dma write buffer\n
 *	Generate test data\n
 *	Set dma write and start dma\n
 *	Wait write dma complete."
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @param{in, dst_pci_addr, u64, >=0}
 * @param{in, dma_num, int, 0~7}
 * @endparam
 * @retval{ret, int, 0, -ERANGE~0, <0}
 */
int exynos_pcie_ep_write(struct exynos_ep_pcie *pcie_ep, u64 dst_pci_addr,
	int dma_num)
{
	struct platform_device *pdev = pcie_ep->pdev;
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci = pcie_ep->pci;
	struct dw_pcie_ep *ep = &pci->ep;
	struct pci_epc *epc = ep->epc;
	void __iomem *dst_addr;
	int ret = 0;
	u32 size = pcie_ep->size;
	u8 headbuf[2];
#ifdef MEASURE_SPEED
	u64 start, end;
#endif
	/* EP PCIe mem address */
	phys_addr_t phys_addr;

	pcie_ep->work_state = STATE_DMA_BUSY;

	/* For the test, add data length 2 bytes */
	size = size + 2;

	dst_addr = pci_epc_mem_alloc_addr(epc, &phys_addr, size);
	if (!dst_addr) {
		dev_err(dev, "failed to allocate address\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = pci_epc_map_addr(epc, 0, 0, phys_addr, dst_pci_addr, size);
	if (ret) {
		dev_err(dev, "failed to map address\n");
		ret = -ENOMEM;
		goto err_addr;
	}

	pcie_ep->dma_wr_buf = dma_alloc_coherent(dev,
			size, &pcie_ep->wr_dma, GFP_KERNEL);
	if (!pcie_ep->dma_wr_buf) {
		dev_err(dev, "failed to allocate buf\n");
		ret = -ENOMEM;
		goto err_map_addr;
	}

	headbuf[0] = (u8)(pcie_ep->size >> 8);
	headbuf[1] = (u8) pcie_ep->size;

	memcpy(pcie_ep->dma_wr_buf, headbuf, 2);

	get_random_bytes(pcie_ep->dma_wr_buf + 2, pcie_ep->size);

#ifdef MEASURE_SPEED
	start = getCurrentTimestampInNSec();
#endif

	/* DMA setting */
	writel(0x1, pcie_ep->dma_base + PCIE_DMA_WR_EN);
	writel(0x0, pcie_ep->dma_base + PCIE_DMA_WR_INT_MASK);
	writel(0x0, pcie_ep->dma_base + PCIE_DMA_WR_INT_CLEAR);
	writel(0x0 | CH_CTRL1_LIE | CH_CTRL1_DMA_RESERVED5, pcie_ep->dma_base +
		PCIE_DMA_CH_CTRL1 + dma_num * PCIE_DMA_CH_OFFSET);

	/* size ragne : 1byte ~ 4Gbyte */
	writel(size, pcie_ep->dma_base + PCIE_DMA_WR_XFER_SIZE +
		dma_num * PCIE_DMA_CH_OFFSET);

	/* source : dram */
	writel(lower_32_bits(pcie_ep->wr_dma),
			pcie_ep->dma_base + PCIE_DMA_WR_SARL +
				dma_num * PCIE_DMA_CH_OFFSET);
	writel(upper_32_bits(pcie_ep->wr_dma),
			pcie_ep->dma_base + PCIE_DMA_WR_SARH +
				dma_num * PCIE_DMA_CH_OFFSET);
	/* destination : PCIe mem */
	writel(lower_32_bits(phys_addr),
			pcie_ep->dma_base + PCIE_DMA_WR_DARL +
				dma_num * PCIE_DMA_CH_OFFSET);
	writel(upper_32_bits(phys_addr),
			pcie_ep->dma_base + PCIE_DMA_WR_DARH +
				dma_num * PCIE_DMA_CH_OFFSET);

	/* start dma ch0 */
	writel(dma_num, pcie_ep->dma_base + PCIE_DMA_WR_DBOFF);

	wait_for_completion(&pcie_ep->wdma0);

	/* DMA Stop */
	writel(PCIE_DMA_DBOFF_STOP, pcie_ep->dma_base + PCIE_DMA_WR_DBOFF);
	writel(0x0, pcie_ep->dma_base + PCIE_DMA_WR_EN);

	if (pcie_ep->state == STATE_LINK_DOWN ||
		pcie_ep->state == STATE_LINK_DOWN_ABNORMAL) {
		dev_info(&pdev->dev, "write work is closed forcedly\n");

		ret = -EPIPE;
		goto err_map_addr;
	}

#ifdef MEASURE_SPEED
	end = getCurrentTimestampInNSec();
	dev_info(&pdev->dev, "##### WRITE TEST #####\n");
	measureTransitionSpeed(pcie_ep, start, end, size);
#endif

	dev_info(&pdev->dev, "WR TEST SUCCESS\n");

err_map_addr:
	pci_epc_unmap_addr(epc, 0, 0, phys_addr);

err_addr:
	pci_epc_mem_free_addr(epc, phys_addr, dst_addr, size);

err:
	pcie_ep->work_state = STATE_DMA_IDLE;

	return ret;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Perform dma read test"
 * @logic "Alloc pci address\n
 *	Set outbound\n
 *	Alloc read dma\n
 *	Set and start read dma\n
 *	Wait read dma completion"
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @param{in, src_pci_addr, u64, >=0}
 * @param{in, size, u32, >=0}
 * @param{in, dma_num, int, 0~7}
 * @endparam
 * @retval{ret, int, 0, -ERANGE~0, <0}
 */
int exynos_pcie_ep_read(struct exynos_ep_pcie *pcie_ep, u64 src_pci_addr,
	u32 size, int dma_num)
{
	struct platform_device *pdev = pcie_ep->pdev;
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci = pcie_ep->pci;
	struct dw_pcie_ep *ep = &pci->ep;
	struct pci_epc *epc = ep->epc;

	int ret = 0;
	void __iomem *src_addr;
	u32 dma_xfer_size;
	/* EP PCIe mem address */
	phys_addr_t phys_addr;

	pcie_ep->work_state = STATE_DMA_BUSY;

	src_addr = pci_epc_mem_alloc_addr(epc, &phys_addr, size);
	if (!src_addr) {
		dev_err(dev, "failed to allocate address\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = pci_epc_map_addr(epc, 0, 0, phys_addr, src_pci_addr, size);
	if (ret) {
		dev_err(dev, "failed to map address\n");
		ret = -ENOMEM;
		goto err_addr;
	}

	if (!pcie_ep->dma_rd_buf) {
		pcie_ep->dma_rd_buf = dma_alloc_coherent(dev, size,
				&pcie_ep->rd_dma, GFP_KERNEL);
		if (!pcie_ep->dma_rd_buf) {
			dev_err(dev, "failed to allocate buf\n");
			ret = -ENOMEM;
			goto err_map_addr;
		}
	}

	/* DMA setting */
	writel(0x1, pcie_ep->dma_base + PCIE_DMA_RD_EN);
	writel(0x0, pcie_ep->dma_base + PCIE_DMA_RD_INT_MASK);
	writel(0x0, pcie_ep->dma_base + PCIE_DMA_RD_INT_CLEAR);
	writel(0x0 | CH_CTRL2_LIE | CH_CTRL2_DMA_RESERVED5, pcie_ep->dma_base +
		PCIE_DMA_CH_CTRL2 + dma_num*PCIE_DMA_CH_OFFSET);

	/* size ragne : 1byte ~ 4Gbyte */
	dma_xfer_size = size;
	writel(dma_xfer_size, pcie_ep->dma_base + PCIE_DMA_RD_XFER_SIZE +
		dma_num * PCIE_DMA_CH_OFFSET);

	/* source : PCIe mem */
	writel(lower_32_bits(phys_addr),
			pcie_ep->dma_base + PCIE_DMA_RD_SARL +
				dma_num * PCIE_DMA_CH_OFFSET);
	writel(upper_32_bits(phys_addr),
			pcie_ep->dma_base + PCIE_DMA_RD_SARH +
				dma_num * PCIE_DMA_CH_OFFSET);
	/* destination : Dram */
	writel(lower_32_bits(pcie_ep->rd_dma),
			pcie_ep->dma_base + PCIE_DMA_RD_DARL +
				dma_num * PCIE_DMA_CH_OFFSET);
	writel(upper_32_bits(pcie_ep->rd_dma),
			pcie_ep->dma_base + PCIE_DMA_RD_DARH +
				dma_num * PCIE_DMA_CH_OFFSET);

	/* start dma ch0 */
	writel(dma_num, pcie_ep->dma_base + PCIE_DMA_RD_DBOFF);

	wait_for_completion(&pcie_ep->rdma0);

	/* DMA Stop */
	writel(PCIE_DMA_DBOFF_STOP, pcie_ep->dma_base + PCIE_DMA_RD_DBOFF);
	writel(0x0, pcie_ep->dma_base + PCIE_DMA_RD_EN);

	if (pcie_ep->state == STATE_LINK_DOWN ||
		pcie_ep->state == STATE_LINK_DOWN_ABNORMAL) {
		dev_info(&pdev->dev, "read work is closed forcedly\n");

		ret = -EPIPE;
		goto err_map_addr;
	}

	dev_info(&pdev->dev, "RD TEST SUCCESS\n");

err_map_addr:
	pci_epc_unmap_addr(epc, 0, 0, phys_addr);

err_addr:
	pci_epc_mem_free_addr(epc, phys_addr, src_addr, size);

err:
	pcie_ep->work_state = STATE_DMA_IDLE;

	return ret;
}


/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Perform crc check test for write value"
 * @logic "Alloc read dma; set and start read dma\n
 *	Wait read dma completion\n
 *	Check crc"
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @param{in, pci_addr, u64, >=0}
 * @param{in, phys_addr, struct ::phys_addr_t, >=0}
 * @param{in, size, u32, >=0}
 * @param{in, dma_num, int, 0~7}
 * @endparam
 * @retval{ret, int, 0, -ERANGE~0, <0}
 */
int exynos_pcie_ep_test_write_check(struct exynos_ep_pcie *pcie_ep,
		u64 pci_addr, phys_addr_t phys_addr, u32 size, int dma_num)
{
	struct platform_device *pdev = pcie_ep->pdev;
	struct device *dev = &pdev->dev;
	int ret = 0;
	u32 crc32;
#ifdef MEASURE_SPEED
	u64 start, end;
#endif

	pcie_ep->work_state = STATE_DMA_BUSY;

	pcie_ep->dma_rd_buf = dma_alloc_coherent(dev,
			size, &pcie_ep->rd_dma, GFP_KERNEL);
	if (!pcie_ep->dma_rd_buf) {
		dev_err(dev, "failed to allocate buf\n");
		ret = -ENOMEM;
		goto err;
	}

#ifdef MEASURE_SPEED
	start = getCurrentTimestampInNSec();
#endif

	/* DMA setting */
	writel(0x1, pcie_ep->dma_base + PCIE_DMA_RD_EN);
	writel(0x0, pcie_ep->dma_base + PCIE_DMA_RD_INT_MASK);
	writel(0x0, pcie_ep->dma_base + PCIE_DMA_RD_INT_CLEAR);
	writel(0x0 | CH_CTRL2_LIE | CH_CTRL2_DMA_RESERVED5, pcie_ep->dma_base +
		PCIE_DMA_CH_CTRL2 + dma_num * PCIE_DMA_CH_OFFSET);

	/* size ragne : 1byte ~ 4Gbyte */
	writel(size, pcie_ep->dma_base + PCIE_DMA_RD_XFER_SIZE +
		dma_num * PCIE_DMA_CH_OFFSET);

	/* source : PCIe mem */
	writel(lower_32_bits(phys_addr),
			pcie_ep->dma_base + PCIE_DMA_RD_SARL +
				dma_num * PCIE_DMA_CH_OFFSET);
	writel(upper_32_bits(phys_addr),
			pcie_ep->dma_base + PCIE_DMA_RD_SARH +
				dma_num * PCIE_DMA_CH_OFFSET);
	/* destination : Dram */
	writel(lower_32_bits(pcie_ep->rd_dma),
			pcie_ep->dma_base + PCIE_DMA_RD_DARL +
				dma_num * PCIE_DMA_CH_OFFSET);
	writel(upper_32_bits(pcie_ep->rd_dma),
			pcie_ep->dma_base + PCIE_DMA_RD_DARH +
				dma_num * PCIE_DMA_CH_OFFSET);

	/* start dma ch0 */
	writel(dma_num, pcie_ep->dma_base + PCIE_DMA_RD_DBOFF);

	wait_for_completion(&pcie_ep->rdma0);

	/* DMA Stop */
	writel(PCIE_DMA_DBOFF_STOP, pcie_ep->dma_base + PCIE_DMA_RD_DBOFF);
	writel(0x0, pcie_ep->dma_base + PCIE_DMA_RD_EN);

#ifdef MEASURE_SPEED
	end = getCurrentTimestampInNSec();
	dev_info(&pdev->dev, "##### READ TEST #####\n");
	measureTransitionSpeed(pcie_ep, start, end, size);
#endif

	crc32 = crc32_le(~0, pcie_ep->dma_rd_buf, size);
	if (crc32 != pcie_ep->checksum) {
		ret = -EIO;
		goto err_dma;
	}
err_dma:
	dma_free_coherent(dev, size, pcie_ep->dma_rd_buf, pcie_ep->rd_dma);
err:
	pcie_ep->work_state = STATE_DMA_IDLE;

	return ret;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Perform dma write and read test"
 * @logic "Alloc pci address; set outbound\n
 *	Alloc write dma\n
 *	Generate write data\n
 *	Set and start write dma\n
 *	Wait write dma completion\n
 *	Alloc read dma\n
 *	Set and start read dma\n
 *	Wait read dma completion\n
 *	Check crc"
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @param{in, dst_pci_addr, u64, >=0}
 * @param{in, dma_num, int, >=0}
 * @endparam
 * @retval{ret, int, 0, -ERANGE~0, <0}
 */
int exynos_pcie_ep_write_read_check(struct exynos_ep_pcie *pcie_ep,
	u64 dst_pci_addr, int dma_num)
{
	struct platform_device *pdev = pcie_ep->pdev;
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci = pcie_ep->pci;
	struct dw_pcie_ep *ep = &pci->ep;
	struct pci_epc *epc = ep->epc;
	void __iomem *dst_addr;
	int ret = 0;
	u32 size = pcie_ep->size;
	u32 crc32_val;
#ifdef MEASURE_SPEED
	u64 start, end;
#endif
	/* EP PCIe mem address */
	phys_addr_t phys_addr;

	pcie_ep->work_state = STATE_DMA_BUSY;

	dst_addr = pci_epc_mem_alloc_addr(epc, &phys_addr, size);
	if (!dst_addr) {
		dev_err(dev, "failed to allocate address\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = pci_epc_map_addr(epc, 0, 0, phys_addr, dst_pci_addr, size);
	if (ret) {
		dev_err(dev, "failed to map address\n");
		ret = -ENOMEM;
		goto err_addr;
	}

	pcie_ep->dma_wr_buf = dma_alloc_coherent(dev,
			size, &pcie_ep->wr_dma, GFP_KERNEL);
	if (!pcie_ep->dma_wr_buf) {
		dev_err(dev, "failed to allocate buf\n");
		ret = -ENOMEM;
		goto err_map_addr;
	}

	get_random_bytes(pcie_ep->dma_wr_buf, pcie_ep->size);
	pcie_ep->checksum = crc32_le(~0, pcie_ep->dma_wr_buf, size);

#ifdef MEASURE_SPEED
	start = getCurrentTimestampInNSec();
#endif

	/* DMA setting */
	writel(0x1, pcie_ep->dma_base + PCIE_DMA_WR_EN);
	writel(0x0, pcie_ep->dma_base + PCIE_DMA_WR_INT_MASK);
	writel(0x0, pcie_ep->dma_base + PCIE_DMA_WR_INT_CLEAR);
	writel(0x0 | CH_CTRL1_LIE | CH_CTRL1_DMA_RESERVED5, pcie_ep->dma_base +
		PCIE_DMA_CH_CTRL1 + dma_num * PCIE_DMA_CH_OFFSET);

	/* size ragne : 1byte ~ 4Gbyte */
	writel(size, pcie_ep->dma_base + PCIE_DMA_WR_XFER_SIZE +
		dma_num * PCIE_DMA_CH_OFFSET);

	/* source : dram */
	writel(lower_32_bits(pcie_ep->wr_dma),
			pcie_ep->dma_base + PCIE_DMA_WR_SARL +
				dma_num * PCIE_DMA_CH_OFFSET);
	writel(upper_32_bits(pcie_ep->wr_dma),
			pcie_ep->dma_base + PCIE_DMA_WR_SARH +
				dma_num * PCIE_DMA_CH_OFFSET);
	/* destination : PCIe mem */
	writel(lower_32_bits(phys_addr),
			pcie_ep->dma_base + PCIE_DMA_WR_DARL +
				dma_num * PCIE_DMA_CH_OFFSET);
	writel(upper_32_bits(phys_addr),
			pcie_ep->dma_base + PCIE_DMA_WR_DARH +
				dma_num * PCIE_DMA_CH_OFFSET);

	/* start dma ch0 */
	writel(dma_num, pcie_ep->dma_base + PCIE_DMA_WR_DBOFF);

	wait_for_completion(&pcie_ep->wdma0);

	/* DMA Stop */
	writel(PCIE_DMA_DBOFF_STOP, pcie_ep->dma_base + PCIE_DMA_WR_DBOFF);
	writel(0x0, pcie_ep->dma_base + PCIE_DMA_WR_EN);

	if (pcie_ep->state == STATE_LINK_DOWN ||
		pcie_ep->state == STATE_LINK_DOWN_ABNORMAL) {
		dev_info(&pdev->dev, "write work is closed forcedly\n");

		ret = -EPIPE;
		goto err_dma;
	}

#ifdef MEASURE_SPEED
	end = getCurrentTimestampInNSec();
	dev_info(&pdev->dev, "##### WRITE TEST #####\n");
	measureTransitionSpeed(pcie_ep, start, end, size);
#endif

	/* Read operation */
	pcie_ep->dma_rd_buf = dma_alloc_coherent(dev,
			size, &pcie_ep->rd_dma, GFP_KERNEL);
	if (!pcie_ep->dma_rd_buf) {
		dev_err(dev, "failed to allocate buf\n");
		ret = -ENOMEM;
		goto err_dma;
	}

#ifdef MEASURE_SPEED
	start = getCurrentTimestampInNSec();
#endif

	/* DMA setting */
	writel(0x1, pcie_ep->dma_base + PCIE_DMA_RD_EN);
	writel(0x0, pcie_ep->dma_base + PCIE_DMA_RD_INT_MASK);
	writel(0x0, pcie_ep->dma_base + PCIE_DMA_RD_INT_CLEAR);
	writel(0x0 | CH_CTRL2_LIE | CH_CTRL2_DMA_RESERVED5, pcie_ep->dma_base +
		PCIE_DMA_CH_CTRL2 + dma_num * PCIE_DMA_CH_OFFSET);

	/* size ragne : 1byte ~ 4Gbyte */
	writel(size, pcie_ep->dma_base + PCIE_DMA_RD_XFER_SIZE +
		dma_num * PCIE_DMA_CH_OFFSET);

	/* source : PCIe mem */
	writel(lower_32_bits(phys_addr),
			pcie_ep->dma_base + PCIE_DMA_RD_SARL +
				dma_num * PCIE_DMA_CH_OFFSET);
	writel(upper_32_bits(phys_addr),
			pcie_ep->dma_base + PCIE_DMA_RD_SARH +
				dma_num * PCIE_DMA_CH_OFFSET);
	/* destination : Dram */
	writel(lower_32_bits(pcie_ep->rd_dma),
			pcie_ep->dma_base + PCIE_DMA_RD_DARL +
				dma_num * PCIE_DMA_CH_OFFSET);
	writel(upper_32_bits(pcie_ep->rd_dma),
			pcie_ep->dma_base + PCIE_DMA_RD_DARH +
				dma_num * PCIE_DMA_CH_OFFSET);

	/* start dma ch0 */
	writel(dma_num, pcie_ep->dma_base + PCIE_DMA_RD_DBOFF);

	wait_for_completion(&pcie_ep->rdma0);

	/* DMA Stop */
	writel(PCIE_DMA_DBOFF_STOP, pcie_ep->dma_base + PCIE_DMA_RD_DBOFF);
	writel(0x0, pcie_ep->dma_base + PCIE_DMA_RD_EN);

	if (pcie_ep->state == STATE_LINK_DOWN ||
		pcie_ep->state == STATE_LINK_DOWN_ABNORMAL) {
		dev_info(&pdev->dev, "read work is closed forcedly\n");

		ret = -EPIPE;
		goto err_dma;
	}

#ifdef MEASURE_SPEED
	end = getCurrentTimestampInNSec();
	dev_info(&pdev->dev, "##### READ TEST #####\n");
	measureTransitionSpeed(pcie_ep, start, end, size);
#endif

	crc32_val = crc32_le(~0, pcie_ep->dma_rd_buf, size);
	if (crc32_val != pcie_ep->checksum) {
		ret = -EIO;
		goto err_dma;
	}
	dev_info(&pdev->dev, "WR/RD TEST SUCCESS\n");

err_dma:
	dma_free_coherent(dev, size, pcie_ep->dma_wr_buf, pcie_ep->wr_dma);

err_map_addr:
	pci_epc_unmap_addr(epc, 0, 0, phys_addr);

err_addr:
	pci_epc_mem_free_addr(epc, phys_addr, dst_addr, size);

err:
	pcie_ep->work_state = STATE_DMA_IDLE;

	return ret;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Perform test functions"
 * @logic "Check test command\n
 *	Set testing address\n
 *	Call test function"
 * @params
 * @param{in/out, work, struct ::work_struct *, not NULL}
 * @endparam
 * @noret
 */
void exynos_pcie_ep_work_cmd(struct work_struct *work)
{
	struct exynos_ep_pcie *pcie_ep =
		container_of(work, struct exynos_ep_pcie, test_work.work);
	struct platform_device *pdev = pcie_ep->pdev;
	struct device *dev = &pdev->dev;
	u64 pci_addr;

	dev_info(dev, "Current work cmd: %d\n", pcie_ep->work_cmd);
	switch (pcie_ep->work_cmd) {
	case WORK_MEM_WR:
		/* Destination Address: RC Reserved Memory Address */
		pci_addr = WR_DMA_TEST_ADDR;
		if (exynos_pcie_ep_write(pcie_ep,
			pci_addr, pcie_ep->dma_num) == 0) {
			/* MSI Send to Notify End of Transaction */
			if (exynos_pcie_ep_msi(pcie_ep, 0) != 0)
				dev_warn(dev, "MSI TEST Fail\n");
		} else {
			dev_warn(dev, "WR TEST Fail\n");
		}
		break;

	case WORK_MEM_RD:
		/* Source Address: RC Reserved Memory Address */
		pci_addr = RD_DMA_TEST_ADDR;
		if (exynos_pcie_ep_read(pcie_ep, pci_addr, pcie_ep->size,
			pcie_ep->dma_num) != 0)
			dev_warn(dev, "RD TEST Fail\n");
		break;

	case WORK_MSI:
		if (exynos_pcie_ep_msi(pcie_ep, pcie_ep->msi_data) != 0)
			dev_warn(dev, "MSI TEST Fail\n");
		break;

	case WORK_DOORBELL:
		/* Received Test Read Request */
		pci_addr = RD_DMA_TEST_ADDR;
		if (exynos_pcie_ep_read(pcie_ep,
				pci_addr, pcie_ep->size, 0) == 0)
			dev_info(dev, "Read from RC RMEM Complete\n");
		else
			dev_warn(dev, "RD TEST Fail!\n");
		break;

	case WORK_MEM_WR_RD:
		pci_addr = WR_DMA_TEST_ADDR;
		if (exynos_pcie_ep_write_read_check(pcie_ep, pci_addr,
			pcie_ep->dma_num) != 0)
			dev_warn(dev, "MEM WR/RD TEST Fail\n");
		break;

	default:
		dev_warn(dev, "Unsupported work cmd: %d\n", pcie_ep->work_cmd);
		break;
	}
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_link}
 * @purpose "Perform exception handling"
 * @logic "If exception is WORK_LINK_RESET, start link stop"
 * @params
 * @param{in/out, work, struct ::work_struct *, not NULL}
 * @endparam
 * @noret
 */
void exynos_pcie_ep_work_exception(struct work_struct *work)
{
	struct exynos_ep_pcie *pcie_ep =
		container_of(work, struct exynos_ep_pcie, exception_work.work);
	struct platform_device *pdev = pcie_ep->pdev;
	struct device *dev = &pdev->dev;
	int count;

	dev_info(dev, "Current exception: %d\n", pcie_ep->work_exception);
	switch (pcie_ep->work_exception) {
	case WORK_LINK_RESET:

		count = 0;
		while (count < 10000) {
			if (pcie_ep->work_state != STATE_DMA_BUSY) {
				if (pcie_ep->use_bifurcation)
					exynos_update_ep_lane_status(pcie_ep);

				dev_info(&pdev->dev, "Start Link Stop\n");
				exynos_pcie_ep_link_stop(pcie_ep);

				pcie_ep->state = STATE_LINK_DOWN;

				break;
			}

			udelay(100);
			count++;
		}
		break;

	default:
		dev_info(dev, "Default exception: %d\n",
					pcie_ep->work_exception);
		break;
	}
}

static int exynos_pcie_ep_test_bar(struct device *dev,
					struct exynos_ep_pcie *pcie_ep)
{
	struct dw_pcie *pci = pcie_ep->pci;
	struct dw_pcie_ep *ep = &pci->ep;
	struct pci_epc *epc = ep->epc;
	struct pci_epf_bar epf_bar;
	int flags;
	int bar;
	int ret = 0;
	dma_addr_t phys_addr;
	size_t size;


	flags = PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_32;
	if (sizeof(dma_addr_t) == 0x8)
		flags |=  PCI_BASE_ADDRESS_MEM_TYPE_64;

	/* set bar 0 */
	bar = 0;
	phys_addr = 0x17000000;
	size = 0xffffff;

	epf_bar.barno = bar;
	epf_bar.size = size;
	epf_bar.flags = flags;
	epf_bar.phys_addr = phys_addr;

	ret = pci_epc_set_bar(epc, 0, 0, &epf_bar);
	if (ret) {
		dev_err(dev, "[%s] failed to set BAR\n", __func__);
		return ret;
	}

	return 0;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Show debug information"
 * @logic "Show debug information"
 * @params
 * @param{in, dev, struct ::device *, not NULL}
 * @param{in, attr, struct ::device_attribute *, not NULL}
 * @param{in, buf, char *, not NULL}
 * @endparam
 * @retval{-, ssize_t, 0, 0, not 0}
 */
static ssize_t exynos_pcie_ep_show_dbg(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	dev_info(dev, "[%s]\n", __func__);
	return 0;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Handler for sysfs test"
 * @logic "If command is 0, dump resistor values\n
 *	If command is 1, check link status\n
 *	If command is 2, call dma read test\n
 *	If command is 3, call dma write test\n
 *	If command is 4, call msi test\n
 *	If command is 5, call dma write/read test\n
 *	If command is 6, call bar allocation test"
 * @params
 * @param{in/out, dev, struct ::device *, not NULL}
 * @param{in, attr, struct ::device_attribute *, not NULL}
 * @param{in, buf, char *, not NULL}
 * @param{in, count, size_t, >=0}
 * @endparam
 * @retval{ret, ssize_t, 0, -ERANGE~0, <0}
 */
static ssize_t exynos_pcie_ep_store_dbg(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int op_num;
	struct exynos_ep_pcie *pcie_ep = dev_get_drvdata(dev);
	u32 val;
	unsigned int size;
	int cnt;
	int dma_num = 0;
	int msi_data;

	if (sscanf(buf, "%d", &op_num) == 0)
		return -EINVAL;
	switch (op_num) {
	case 0:
		exynos_pcie_ep_register_link_dump(pcie_ep);
		break;
	case 1:
		dev_info(dev, "Check Link status\n");
		val = readl(pcie_ep->elbi_base + PCIE_ELBI_RDLH_LINKUP) & 0x1f;
		dev_info(dev, "0x%x\n", val);
		break;
	case 2:
		if (sscanf(buf, "%d %d", &op_num, &dma_num) == 2) {
			if (pcie_ep->state != STATE_LINK_UP) {
				dev_info(dev, "current state is not link up\n");
				return -1;
			}
			dev_info(dev, "mem read\n");
			pcie_ep->work_cmd = WORK_MEM_RD;
			pcie_ep->dma_num = dma_num;

			queue_work(pcie_ep->pcie_ep_wq,
						&pcie_ep->test_work.work);
		} else {
			dev_warn(dev, "Command 2 argument invalid\n");
			return -EINVAL;
		}

		break;
	case 3:
		if (pcie_ep->state != STATE_LINK_UP) {
			dev_info(dev, "current state is not link up\n");
			return -1;
		}

		if (sscanf(buf, "%d %d %d %x", &op_num, &cnt,
						&dma_num, &size) == 4) {
			dev_info(dev, "count : %u , transfer size: 0x%x\n",
					cnt, size);

			if (size > MAX_TEST_WR_SIZE) {
				dev_info(dev,
				"Maximum write size exceeded, fix to %x\n",
					MAX_TEST_WR_SIZE);
				size = MAX_TEST_WR_SIZE;
			}

			pcie_ep->size = size;
			pcie_ep->cnt = cnt;
			pcie_ep->dma_num = dma_num;

			pcie_ep->work_cmd = WORK_MEM_WR;
			queue_work(pcie_ep->pcie_ep_wq,
					&pcie_ep->test_work.work);
		} else {
			dev_warn(dev, "Command 3 argument invalid\n");
			return -EINVAL;
		}
		break;
	case 4:
		if (sscanf(buf, "%d %x", &op_num, &msi_data) == 2) {
			if (pcie_ep->state != STATE_LINK_UP) {
				dev_info(dev, "current state is not link up\n");
				return -1;
			}

			dev_info(dev, "MSI test\n");
			pcie_ep->work_cmd = WORK_MSI;
			pcie_ep->msi_data = msi_data;
			queue_work(pcie_ep->pcie_ep_wq,
					&pcie_ep->test_work.work);
		} else {
			dev_warn(dev, "Command 4 argument invalid\n");
			return -EINVAL;
		}
		break;
	case 5:
		if (pcie_ep->state != STATE_LINK_UP) {
			dev_info(dev, "current state is not link up\n");
			return -1;
		}

		if (sscanf(buf, "%d %d %x", &op_num, &dma_num, &size) == 3) {
			dev_info(dev, "transfer size: 0x%x\n", size);

			if (size > MAX_TEST_WR_SIZE) {
				dev_info(dev,
				"Maximum write size exceeded, fix to %x\n",
					MAX_TEST_WR_SIZE);
					size = MAX_TEST_WR_SIZE;
			}

			pcie_ep->size = size;
			pcie_ep->dma_num = dma_num;

			dev_info(dev, "MEM WR/RD test\n");
			pcie_ep->work_cmd = WORK_MEM_WR_RD;
			queue_work(pcie_ep->pcie_ep_wq,
					&pcie_ep->test_work.work);
		} else {
			dev_warn(dev, "Command 5 argument invalid\n");
			return -EINVAL;
		}
		break;
	case 6:
		dev_info(dev, "Inbound test\n");
		exynos_pcie_ep_test_bar(dev, pcie_ep);
		break;

	default:
		dev_warn(dev, "Unsupported Test Number(%d)...\n", op_num);
		break;
	}
	return count;
}

static DEVICE_ATTR(ep_dbg, S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP,
			exynos_pcie_ep_show_dbg, exynos_pcie_ep_store_dbg);
/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Create sysfile for ep driver"
 * @logic "Create device file"
 * @params
 * @param{in/out, dev, struct ::device *, not NULL}
 * @endparam
 * @retval{-, int, 0, int, not 0}
 */
static inline int create_pcie_ep_sys_file(struct device *dev)
{
	return device_create_file(dev, &dev_attr_ep_dbg);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Remove sysfile for ep driver"
 * @logic "Remove device file"
 * @params
 * @param{in/out, dev, struct ::device *, not NULL}
 * @endparam
 * @noret
 */
static inline void remove_pcie_ep_sys_file(struct device *dev)
{
	device_remove_file(dev, &dev_attr_ep_dbg);
}

static const struct dw_pcie_ops dw_pcie_ep_ops = {
	.read_dbi = NULL,
	.write_dbi = NULL,
};

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Request irqs for PCIe EP"
 * @logic "Request irqs for PCIe EP."
 * @params
 * @param{in/out, pcie_ep, struct ::exynos_ep_pcie *, not NULL}
 * @endparam
 * @retval{ret, int, 0, -ERANGE~0, <0}
 */
static int exynos_pcie_ep_request_irq(struct exynos_ep_pcie *pcie_ep)
{
	struct platform_device *pdev = pcie_ep->pdev;
	int ret = 0;
	int i;

	/* request irq */
	pcie_ep->irq = platform_get_irq(pdev, 0);
	if (!pcie_ep->irq) {
		dev_err(&pdev->dev, "failed to get irq for PCIe EP\n");
		return -ENODEV;
	}

	ret = devm_request_irq(&pdev->dev, pcie_ep->irq,
			exynos_pcie_ep_irq_handler, IRQF_SHARED,
			"exynos-pcie-ep", pcie_ep);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq for PCIe EP\n");
		return -ENODEV;
	}

	for (i  = 0; i < MAX_VF_NUM; i++) {
		/* request Doorbell irq */
		pcie_ep->db_irq[i] = platform_get_irq(pdev, i + 1);
		if (!pcie_ep->db_irq[i]) {
			dev_err(&pdev->dev,
				"failed to get db_irq for PCIe EP\n");
			return -ENODEV;
		}

		ret = devm_request_irq(&pdev->dev, pcie_ep->db_irq[i],
			exynos_pcie_ep_doorbell_irq_handler, IRQF_SHARED,
			"exynos-pcie-ep", pcie_ep);
		if (ret) {
			dev_err(&pdev->dev,
				"failed to request db_irq for PCIe EP\n");
			return -ENODEV;
		}
	}

	if (pcie_ep->ch_num == 0 || pcie_ep->ch_num == 2) {
		for (i  = 0; i < MAX_VF_NUM; i++) {
			/* request DMA irq */
			pcie_ep->dma_irq[i] =
				platform_get_irq(pdev, i + 1 + MAX_VF_NUM);
			if (!pcie_ep->dma_irq[i]) {
				dev_err(&pdev->dev,
					"failed to get dma_irq for PCIe EP\n");
				return -ENODEV;
			}

			if (i == 0) {
				/* DMA Interrupt 0 */
				ret = devm_request_irq
					(&pdev->dev, pcie_ep->dma_irq[i],
					exynos_pcie_ep_dma0_irq_handler,
					IRQF_SHARED, "exynos-pcie-ep", pcie_ep);
				if (ret) {
					dev_err(&pdev->dev,
						"failed to request");
					dev_err(&pdev->dev,
						"dma0_irq for PCIe EP\n");
					return -ENODEV;
				}
			} else if (i == 1) {
				/* DMA Interrupt 1 */
				ret = devm_request_irq
					(&pdev->dev, pcie_ep->dma_irq[i],
						exynos_pcie_ep_dma1_irq_handler,
							IRQF_SHARED,
						"exynos-pcie-ep", pcie_ep);
				if (ret) {
					dev_err(&pdev->dev,
						"failed to request");
					dev_err(&pdev->dev,
						"dma1_irq for PCIe EP\n");
					return -ENODEV;
				}
			} else if (i == 2) {
				/* DMA Interrupt 2 */
				ret = devm_request_irq(&pdev->dev,
						pcie_ep->dma_irq[i],
						exynos_pcie_ep_dma2_irq_handler,
						IRQF_SHARED,
						"exynos-pcie-ep", pcie_ep);
				if (ret) {
					dev_err(&pdev->dev,
						"failed to request");
					dev_err(&pdev->dev,
						"dma2_irq for PCIe EP\n");
					return -ENODEV;
				}
			} else if (i == 3) {
				/* DMA Interrupt 3 */
				ret = devm_request_irq(&pdev->dev,
						pcie_ep->dma_irq[i],
						exynos_pcie_ep_dma3_irq_handler,
						IRQF_SHARED,
						"exynos-pcie-ep", pcie_ep);
				if (ret) {
					dev_err(&pdev->dev,
						"failed to request");
					dev_err(&pdev->dev,
						"dma3_irq for PCIe EP\n");
					return -ENODEV;
				}
			}
		}
	}

	return 0;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_init}
 * @purpose "Probe PCIe EP driver"
 * @logic "Get data from dt; alloc phyinfo\n
 *	Initialize work queue\n
 *	Initialize dma completion\n
 *	Get clock and enable\n
 *	Set pinctrl\n
 *	Request irqs\n
 *	Create sysfs file\n
 *	Initialize phy and ep device."
 * @params
 * @param{in/out, pdev, struct ::platform_device *, not NULL}
 * @endparam
 * @retval{ret, int, 0, <=0, not 0}
 */
static int exynos_pcie_ep_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci;
	struct device_node *np;
	struct exynos_ep_pcie *pcie_ep;
	struct exynos_ep_pcie_phyinfo *phyinfo;
	int ret = 0;
	int ch_num = 0;

	if (dev == NULL) {
		ret = -EINVAL;
		goto probe_fail;
	}

	np = pdev->dev.of_node;

	dev_info(&pdev->dev, "[%s]\n", __func__);

	/* get channel number from Device Tree */
	if (of_property_read_u32(np, "ch-num", &ch_num)) {
		dev_err(dev, "Failed to parse the channel number\n");
		return -EINVAL;
	}

	pcie_ep = &g_pcie_ep[ch_num];
	pcie_ep->ch_num = ch_num;
	pcie_ep->pdev = pdev;

	pcie_ep->work_state = STATE_DMA_IDLE;

	/* Set Onother lane status -1 (for RC EP Cross) */
	if (pcie_ep->ch_num % 2 == 0)
		g_pcie_ep[ch_num + 1].lane_status = -1;
	else
		g_pcie_ep[ch_num - 1].lane_status = -1;

	pcie_ep->lane_status = -1;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (pci == NULL) {
		ret = -ENOMEM;
		goto probe_fail;
	}
	pci->dev = dev;
	pcie_ep->pci = pci;
	pci->ops = &dw_pcie_ep_ops;

	ret = exynos_pcie_ep_get_resource(pcie_ep);
	if (ret) {
		dev_err(dev, "getting resource for PCIe EP is failed\n");
		ret = -EINVAL;
		goto probe_fail;
	}

	/* parsing pcie ep dts data for exynos */
	ret = exynos_pcie_ep_parse_dt(pcie_ep);
	if (ret)
		goto probe_fail;

	/* set phyinfo for PHY CAL */
	phyinfo = devm_kzalloc(dev, sizeof(struct exynos_ep_pcie_phyinfo),
			GFP_KERNEL);
	if (!phyinfo)
		return -ENOMEM;

	phyinfo->phy_base = pcie_ep->phy_base;
	phyinfo->phy_pcs_base = pcie_ep->phy_pcs_base;
	phyinfo->sysreg_base = pcie_ep->sysreg_base;
	phyinfo->elbi_base = pcie_ep->elbi_base;
	phyinfo->dbi_base = pcie_ep->dbi_base;
	phyinfo->ch_num = ch_num;
	phyinfo->use_bifurcation = pcie_ep->use_bifurcation;
	phyinfo->lanes_num = pcie_ep->lanes_num;
	phyinfo->use_sris = pcie_ep->use_sris;
	phyinfo->elbi_cmn_base = pcie_ep->elbi_cmn_base;

	pcie_ep->ep_phyinfo = phyinfo;

	/* create and initialize the workqueue for handling events*/
	pcie_ep->pcie_ep_wq = create_freezable_workqueue("pcie_ep_wq");
	if (IS_ERR(pcie_ep->pcie_ep_wq)) {
		dev_err(dev, "Creating workqueue ('pcie_ep_wq')");
		dev_err(dev, "is failed\n");
		ret = -EBUSY;
		goto probe_fail;
	}

	/* create and initialize the workqueue for exception handlers */
	pcie_ep->pcie_ep_wq_exception =
			create_freezable_workqueue("pcie_ep_wq_exception");
	if (IS_ERR(pcie_ep->pcie_ep_wq_exception)) {
		dev_err(dev, "Creating workqueue ('pcie_ep_wq_exception')");
		dev_err(dev, "is failed\n");
		ret = -EBUSY;
		goto probe_fail;
	}

	/* create and initialize the workqueue for dislink */
	pcie_ep->pcie_ep_wq_dislink =
			create_freezable_workqueue("pcie_ep_wq_dislink");
	if (IS_ERR(pcie_ep->pcie_ep_wq_dislink)) {
		dev_err(dev, "Creating workqueue ('pcie_ep_wq_dislink')");
		dev_err(dev, "is failed\n");
		ret = -EBUSY;
		goto probe_fail;
	}

	INIT_DELAYED_WORK(&pcie_ep->irq1_evt_work,
			exynos_pcie_ep_irq1_handler);

	INIT_DELAYED_WORK(&pcie_ep->test_work,
			exynos_pcie_ep_work_cmd);

	INIT_DELAYED_WORK(&pcie_ep->exception_work,
			exynos_pcie_ep_work_exception);

	/* for eDMA test */
	init_completion(&pcie_ep->wdma0);
	init_completion(&pcie_ep->wdma1);
	init_completion(&pcie_ep->wdma2);
	init_completion(&pcie_ep->wdma3);

	init_completion(&pcie_ep->rdma0);
	init_completion(&pcie_ep->rdma1);
	init_completion(&pcie_ep->rdma2);
	init_completion(&pcie_ep->rdma3);

	pcie_ep->dma_wr_buf = NULL;
	pcie_ep->dma_rd_buf = NULL;

	/* PMU isolation by pass */
	regmap_update_bits(pcie_ep->pmureg,
				pcie_ep->pmu_phy_offset,
				PCIE_EP_PHY_CONTROL_MASK, 1);

	/* Clock get & enable */
	ret = exynos_pcie_ep_clk_get(pcie_ep);
	if (ret) {
		dev_err(dev, "getting clocks for PCIe EP is failed\n");
		ret = -EINVAL;
		goto probe_fail;
	}
	exynos_pcie_ep_clk_enable(pcie_ep, 1);
	exynos_pcie_ep_phyclk_enable(pcie_ep, 1);

	/* PREST pin set to function mode -> should get pin info. from DT */
	pcie_ep->pcie_ep_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pcie_ep->pcie_ep_pinctrl)) {
		dev_err(&pdev->dev, "Can't get PCIe EP pinctrl\n");
		ret = -EINVAL;
		goto probe_fail;
	}
	pcie_ep->pin_perst =
		pinctrl_lookup_state(pcie_ep->pcie_ep_pinctrl, "perst_in");
	if (IS_ERR(pcie_ep->pin_perst)) {
		dev_err(&pdev->dev,
			"Can't set PERST pin to function mode for PCIe EP\n");
		ret = -EINVAL;
		goto probe_fail;
	}
	pinctrl_select_state(pcie_ep->pcie_ep_pinctrl,
			pcie_ep->pin_perst);

	pcie_ep->pin_clkreq =
		pinctrl_lookup_state(pcie_ep->pcie_ep_pinctrl, "clkreq");
	if (IS_ERR(pcie_ep->pin_clkreq)) {
		dev_err(&pdev->dev,
			"Can't set CLKREQ pin to function mode for PCIe EP\n");
		ret = -EINVAL;
		goto probe_fail;
	}

	pinctrl_select_state(pcie_ep->pcie_ep_pinctrl,
			pcie_ep->pin_clkreq);

	ret = exynos_pcie_ep_request_irq(pcie_ep);
	if (ret)
		goto probe_fail;

	/* create sysfs for debugging */
	if (create_pcie_ep_sys_file(&pdev->dev))
		dev_warn(&pdev->dev, "Failed to create pcie sys file\n");

	/* phy init : mapping phy function */
	exynos_pcie_ep_phyinit(pcie_ep);

	/* initialize designware PCIe EP => mainline i/f should be added */
	exynos_ep_ready_establish_link(pcie_ep);

	/* set drv data */
	platform_set_drvdata(pdev, pcie_ep);

#ifdef CONFIG_S2MPU
	if (pcie_ep->s2mpu_base) {
		if (hyp_call_s2mpu_protection(pcie_ep->s2mpu_base) != 0)
			dev_info(&pdev->dev, "%s Fail s2mpu=%08x\n",
				 __func__, pcie_ep->s2mpu_base);
		else
			dev_info(&pdev->dev, "%s Success s2mpu=%08x\n",
				__func__, pcie_ep->s2mpu_base);
	} else {
		dev_info(&pdev->dev, "%s s2mpu is disabled [s2mpu=%08x]\n",
			__func__, pcie_ep->s2mpu_base);
	}
#endif

probe_fail:
	if (ret)
		dev_err(&pdev->dev, "%s: pcie probe failed\n", __func__);
	else
		dev_info(&pdev->dev, "%s: pcie probe success\n", __func__);

	return ret;
}

/* PCIe link status checking function */
/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Check ep link status"
 * @logic "Check ep link status"
 * @params
 * @param{in, ch_num, int, 0~5}
 * @endparam
 * @retval{-, int, 0, 0, not 0}
 */
int exynos_check_pcie_ep_link_status(int ch_num)
{
	struct exynos_ep_pcie *pcie_ep = &g_pcie_ep[ch_num];
	struct platform_device *pdev = pcie_ep->pdev;

	dev_info(&pdev->dev, "%s\n", __func__);

	return 0;
}
EXPORT_SYMBOL(exynos_check_pcie_ep_link_status);

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Remove PCIe EP"
 * @logic "Remove PCIe EP"
 * @params
 * @param{in, pdev, struct ::platform_device *, not NULL}
 * @endparam
 * @retval{-, int, 0, 0, not 0}
 */
static int __exit exynos_pcie_ep_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "%s\n", __func__);

	return 0;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Shutdown PCIe EP"
 * @logic "Do nothing"
 * @params
 * @param{in, pdev, struct ::platform_device *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_pcie_ep_shutdown(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "%s\n", __func__);
}

static const struct of_device_id exynos_pcie_ep_of_match[] = {
	{ .compatible = "samsung,exynos-pcie-ep", },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_pcie_ep_of_match);

#ifdef CONFIG_PM
/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Suspend PCIe ep without irq"
 * @logic ""
 * @params
 * @param{in/out, dev, struct ::device *, not NULL}
 * @endparam
 * @retval{-, int, -, 0, not 0}
 */
static int exynos_pcie_ep_suspend_noirq(struct device *dev)
{
	struct exynos_ep_pcie *pcie_ep = dev_get_drvdata(dev);
	u32 val;

	dev_info(dev, "PCIe EP suspend no irq\n");

	if (pcie_ep->state != STATE_LINK_DOWN) {
		/* lane status off */
		pcie_ep->lane_status = 0;

		if (pcie_ep->use_bifurcation)
			exynos_update_ep_lane_status(pcie_ep);

		/* Link stop act */
		pcie_ep->state = STATE_LINK_DOWN_TRY;

		if (pcie_ep->use_sris) {
			/* set perstn_in_mux from SYSREG to PAD(default)*/
			val = readl(pcie_ep->sysreg_ctrl_base);
			val |= (0x1 << 5);
			writel(val, pcie_ep->sysreg_ctrl_base);
		}

		/* LTSSM disable */
		exynos_pcie_ep_set_ltssm(pcie_ep, 0);

		pcie_ep->linkup_ready = 0;

		pcie_ep->state = STATE_LINK_DOWN_TRY;

		/* phy all power down */
		pcie_ep->phy_ops.phy_all_pwrdn(pcie_ep->ep_phyinfo);

		pcie_ep->state = STATE_LINK_DOWN;
	}

	return 0;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_internal}
 * @purpose "Resume PCIe ep without irq"
 * @logic "call exynos_ep_ready_establish_link"
 * @params
 * @param{out, dev, struct ::device *, not NULL}
 * @endparam
 * @retval{Name, int, -, 0, not 0}
 */
static int exynos_pcie_ep_resume_noirq(struct device *dev)
{
	struct exynos_ep_pcie *pcie_ep = dev_get_drvdata(dev);

	dev_info(dev, "PCIe EP resume no irq\n");
	exynos_ep_ready_establish_link(pcie_ep);

	return 0;
}
#else
#define exynos_pcie_ep_suspend_noirq	NULL
#define exynos_pcie_ep_resume_noirq	NULL
#endif

static const struct dev_pm_ops exynos_pcie_ep_dev_pm_ops = {
	.suspend_noirq	= exynos_pcie_ep_suspend_noirq,
	.resume_noirq	= exynos_pcie_ep_resume_noirq,
};

static struct platform_driver exynos_pcie_ep_driver = {
	.probe = exynos_pcie_ep_probe,
	.remove = exynos_pcie_ep_remove,
	.shutdown = exynos_pcie_ep_shutdown,
	.driver = {
		.name	= "exynos-pcie-ep",
		.of_match_table = exynos_pcie_ep_of_match,
		.pm	= &exynos_pcie_ep_dev_pm_ops,
	},
};

module_platform_driver(exynos_pcie_ep_driver);

MODULE_AUTHOR("Kyounghye Yun <k-hye.yun@samsung.com>");
MODULE_AUTHOR("Jiheon Oh <jiheon.oh@samsung.com>");
MODULE_DESCRIPTION("Samsung PCIe Endpoint controller driver");
MODULE_LICENSE("GPL v2");
