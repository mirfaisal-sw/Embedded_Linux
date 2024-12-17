// SPDX-License-Identifier: GPL-2.0
/**
 * pcie-exynos-ep-auto-cal.c - Exynos PCIe EP cal
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Authors: Sanghoon Bae <sh86.bae@samsung.com>
 *	    Jiheon Oh <jiheon.oh@samsung.com>
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include "pcie-exynos-v920-ep-auto.h"
#include "pcie-exynos-v920-ep-auto-cal.h"

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_phypower}
 * @purpose "Power off the PCIe phy power"
 * @logic "Check other lane status\n
 *	If condition satisfied, phy power down."
 * @params
 * @param{in, phyinfo, struct ::exynos_ep_pcie_phyinfo *, not NULL}
 * @endparam
 * @noret
 */
void exynos_v920_pcie_ep_phy_all_pwrdn(struct exynos_ep_pcie_phyinfo *phyinfo)
{
	void __iomem *cmu_base_regs = phyinfo->cmu_base;

	writel(0x0, cmu_base_regs + 0x0);
}
EXPORT_SYMBOL(exynos_v920_pcie_ep_phy_all_pwrdn);

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_phypower}
 * @purpose "Power on all phy"
 * @logic "Power on all phy."
 * @params
 * @param{in, phyinfo, struct ::exynos_ep_pcie_phyinfo *, not NULL}
 * @endparam
 * @noret
 */
void exynos_v920_pcie_ep_phy_all_pwrdn_clear(struct exynos_ep_pcie_phyinfo *phyinfo)
{
	void __iomem *cmu_base_regs = phyinfo->cmu_base;

	writel(0x6, cmu_base_regs + 0x0);
}
EXPORT_SYMBOL(exynos_v920_pcie_ep_phy_all_pwrdn_clear);

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_physet}
 * @purpose "Config pcie phy at link start or reset sequence"
 * @logic "Phy reset\n
 *	Set to ep selection\n
 *	SRIS enable if use SRIS feature\n
 *	Perform phy setting"
 * @params
 * @param{in/out, phyinfo, struct ::exynos_ep_pcie_phyinfo *, not NULL}
 * @param{in/out, result, int, not NULL}
 * @endparam
 * @noret
 */
void exynos_v920_pcie_ep_phy_config(struct exynos_ep_pcie_phyinfo *phyinfo, int *result)
{
	void __iomem *elbi_base_regs = phyinfo->elbi_base;
	void __iomem *sysreg_base_regs = phyinfo->sysreg_base;
	void __iomem *e32_phy_regs = phyinfo->e32_phy_base;

	u32 linkA, linkB, phy_config_level = 1;
	u32 val;
	int timeout, pcie_bifur_sel_reg;

	*result = 1;
	pcie_bifur_sel_reg = phyinfo->ch_num % 2;

	if (pcie_bifur_sel_reg) {
		linkA = readl(phyinfo->elbi_base_other) & 0x1f;
		linkB = readl(elbi_base_regs + 0x304) & 0x1f;
	} else {
		linkA = readl(elbi_base_regs + 0x304) & 0x1f;
		linkB = readl(phyinfo->elbi_base_other) & 0x1f;
	}

	dev_info(NULL, "linkA: %x and linkB: %x\n", linkA, linkB);
	if ((linkA != 0x11 && linkB != 0x11) || (!phyinfo->use_bifurcation)) {
		phy_config_level = 0;
	}

	phyinfo->cold_rst = 1;
	exynos_v920_pcie_ep_phy_g_rst(phyinfo);

	if (phy_config_level == 0) {
		/*	PLL Setting */
		val = readl(sysreg_base_regs + 0x600);
		writel(val | (0x1<<8), sysreg_base_regs + 0x600);

		val = readl(sysreg_base_regs + 0x604);
		writel(val | (0x1), sysreg_base_regs + 0x604);

		val = readl(sysreg_base_regs + 0x608);
		writel(val | (0x1<<24), sysreg_base_regs + 0x608);

		/*	Clock buffer TX enable	*/
		val = readl(sysreg_base_regs + 0x620);
		writel(val | (0x1<<0), sysreg_base_regs + 0x620);
		val = readl(sysreg_base_regs + 0x630);
		writel(val | (0x1<<0), sysreg_base_regs + 0x630);
		val = readl(sysreg_base_regs + 0x640);
		writel(val | (0x1<<0), sysreg_base_regs + 0x640);
		val = readl(sysreg_base_regs + 0x650);
		writel(val | (0x1<<0), sysreg_base_regs + 0x650);

		//Input REFCLK setting
		//PHY0_REFA_CLK_SEL
		val = readl(e32_phy_regs + 0x10);
		writel(val & ~(0x3<<16), e32_phy_regs + 0x10);
		val = readl(e32_phy_regs + 0x10);
		writel(val & (0x1<<16), e32_phy_regs + 0x10);

		//PHY0_REFB_CLK_SEL
		val = readl(e32_phy_regs + 0x10);
		writel(val & ~(0x3<<18), e32_phy_regs + 0x10);
		val = readl(e32_phy_regs + 0x10);
		writel(val & (0x1<<18), e32_phy_regs + 0x10);

		//PHY0_REFB_CLK_SEL
		udelay(100);

		/*	PHY resistor tune request	*/
		writel(0x10000001, e32_phy_regs);
		/*	PHY External Tx RoPLL PostDiv Control */
		writel(0x1249, e32_phy_regs + 0x1A8);
		/*	PHY External TX RoPLL PostDiv Override Enable	*/
		writel(0xF, e32_phy_regs + 0x1C4);
		/*	Lanepll Bypass Mode Control	*/
		writel(0x00, e32_phy_regs + 0x384);
		writel(0x000, e32_phy_regs + 0x384);

		/*Enable Enable Reference Clock Detection for refA/B	*/
		writel(0x06100060, e32_phy_regs + 0x10);
	}

	phyinfo->cold_rst = 0;
	exynos_v920_pcie_ep_phy_g_rst(phyinfo);

	if (phy_config_level == 0) {
		/*	SRAM Bypass Start	*/
		val = readl(e32_phy_regs + 0x10);
		writel(val | (0x1<<10), e32_phy_regs + 0x10);

		timeout = 0;
		do {
			udelay(1);

			timeout++;
			if (timeout == 2000) {
				dev_info(NULL, "SRAM bypass FAIL\n");
				*result = 0;
				break;
			}
		} while (!(readl(e32_phy_regs + 0x10) >> 31));
		timeout = 0;

		val = readl(e32_phy_regs + 0x10);
		writel(val | (0x1<<11), e32_phy_regs + 0x10);

		/*	WAIT CDM in reset	*/
		do {
			udelay(1);
			timeout++;
			if (timeout == 2000) {
				dev_info(NULL, "CDM init FAIL\n");
				*result = 0;
				break;
			}
		} while (!(readl(elbi_base_regs + 0x304) >> 24));
		timeout = 0;
	}

	if (phyinfo->use_bifurcation && (phyinfo->ch_num == 0 || phyinfo->ch_num == 2)) {
		if (phyinfo->ch_num == 0) {
			val = readl(sysreg_base_regs + 0x828);
			val = val | (1 << 28);
			writel(val, (sysreg_base_regs + 0x828));
		} else if (phyinfo->ch_num == 2) {
			val = readl(sysreg_base_regs + 0x868);
			val = val | (1 << 28);
			writel(val, (sysreg_base_regs + 0x868));
		}
	}

}
EXPORT_SYMBOL(exynos_v920_pcie_ep_phy_config);

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_physet}
 * @purpose "Additional phy setting"
 * @logic ""
 * @params
 * @param{in, phyinfo, struct ::exynos_ep_pcie_phyinfo *, not NULL}
 * @endparam
 * @noret
 */
void exynos_v920_pcie_ep_phy_pcs_rst_set(struct exynos_ep_pcie_phyinfo *phyinfo)
{
	/* For SRIS  PHY settings */
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_physet}
 * @purpose "Reset pcie global phy"
 * @logic "Reset pcie global phy"
 * @params
 * @param{in, phyinfo, struct ::exynos_ep_pcie_phyinfo *, not NULL}
 * @endparam
 * @noret
 */
void exynos_v920_pcie_ep_phy_g_rst(struct exynos_ep_pcie_phyinfo *phyinfo)
{
	void __iomem *gen_subsys_regs = phyinfo->gen_subsys_base;
	u32 val;

	if (phyinfo->cold_rst) {
		/*	Perform Cold Reset */
		if (phyinfo->ch_num == 0 || phyinfo->ch_num == 2) {
			/*	PE0 COLD reset */
			val = readl(gen_subsys_regs + 0x48);
			writel(val | 0x1, gen_subsys_regs + 0x48);
			udelay(10);
			writel(val & ~(0x1), gen_subsys_regs + 0x48);
		} else if (phyinfo->ch_num == 1 || phyinfo->ch_num == 3) {
			/*	PE0 COLD reset */
			val = readl(gen_subsys_regs + 0x48);
			writel(val | (0x1<<8), gen_subsys_regs + 0x48);
			udelay(10);
			writel(val & ~(0x1<<8), gen_subsys_regs + 0x48);
		}
	} else {
		/*	Perform Warm Reset/PHY Reset */
		if (phyinfo->ch_num == 0 || phyinfo->ch_num == 2) {
		/*	PE0 SW reset */
			val = readl(gen_subsys_regs + 0x48);
			writel(val | (0x3<<1), gen_subsys_regs + 0x48);
			udelay(10);
			writel(val & ~(0x3<<1), gen_subsys_regs + 0x48);
		} else if (phyinfo->ch_num == 1 || phyinfo->ch_num == 3) {
		/*	PE0 SW reset */
			val = readl(gen_subsys_regs + 0x48);
			writel(val | (0x3<<9), gen_subsys_regs + 0x48);
			udelay(10);
			writel(val & ~(0x3<<9), gen_subsys_regs + 0x48);
		}
	}

}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_physet}
 * @purpose "Misc setting for pcie phy"
 * @logic "REFCLK setting; Lane power enable"
 * @params
 * @param{in, phyinfo, struct ::exynos_ep_pcie_phyinfo *, not NULL}
 * @endparam
 * @noret
 */
void exynos_v920_pcie_ep_phy_others_set(struct exynos_ep_pcie_phyinfo *phyinfo)
{
	/* For MSIC PHY settings */
}
EXPORT_SYMBOL(exynos_v920_pcie_ep_phy_others_set);

