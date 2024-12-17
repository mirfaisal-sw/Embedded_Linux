// SPDX-License-Identifier: GPL-2.0
/**
 * pcie-exynos-rc-auto-cal.c - Exynos PCIe RC cal
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
#include <linux/pci.h>
#include <linux/delay.h>
#include "../pcie-designware.h"
#include "pcie-exynos-v920-rc-auto.h"

void exynos_v920_host_pcie_others_set(struct pcie_port *pp);

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_rc_phypower}
 * @purpose "Power down phy"
 * @logic "Check state of link A/B\n
 *	Power down phy."
 * @params
 * @param{in, pp, struct ::pcie_port *, not NULL}
 * @endparam
 * @noret
 */
void exynos_v920_phy_all_pwrdn(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	void __iomem *cmu_base_regs = exynos_pcie->cmu_base;

	writel(0x0, cmu_base_regs + 0x0);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_rc_phypower}
 * @purpose "Power on phy"
 * @logic "Power on all phy"
 * @params
 * @param{in, pp, struct ::pcie_port *, not NULL}
 * @endparam
 * @noret
 */
void exynos_v920_phy_all_pwrdn_clear(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	void __iomem *cmu_base_regs = exynos_pcie->cmu_base;

	writel(0x6, cmu_base_regs + 0x0);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_rc_physet}
 * @purpose "Config PCIe RC phy"
 * @logic "Reset global pcs\n
 *	Select RC\n
 *	Enable SRIS (if enabled)\n
 *	Config phy for each lane."
 * @params
 * @param{in, pp, struct ::pcie_port *, not NULL}
 * @param{in/out, result, int, not NULL}
 * @endparam
 * @noret
 */
void exynos_v920_pcie_phy_config(struct pcie_port *pp, int *result)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);

	u32 val;
	int pcie_bifur_sel_reg;
	u32 linkA, linkB, phy_config_level = 1;
	int timeout;

	void __iomem *sysreg_base_regs = exynos_pcie->sysreg_base;
	void __iomem *elbi_base_regs = exynos_pcie->elbi_base;
	void __iomem *e32_phy_regs = exynos_pcie->e32_phy_base;
	void __iomem *gen_subsys_regs = exynos_pcie->gen_subsys_base;

	*result = 1;

	pcie_bifur_sel_reg = exynos_pcie->ch_num % 2;
	if (pcie_bifur_sel_reg) {
		linkA = readl(exynos_pcie->elbi_base_other) & 0x1f;
		linkB = readl(elbi_base_regs + 0x304) & 0x1f;
	} else {
		linkA = readl(elbi_base_regs + 0x304) & 0x1f;
		linkB = readl(exynos_pcie->elbi_base_other) & 0x1f;
	}

	dev_info(pci->dev, "linkA: %x and linkB: %x\n", linkA, linkB);
	if ((linkA != 0x11 && linkB != 0x11) || (!exynos_pcie->use_bifurcation)) {
		phy_config_level = 0;
	}

	if (exynos_pcie->ch_num == 0 || exynos_pcie->ch_num == 2) {
		/*	PE0 COLD reset */
		val = readl(gen_subsys_regs + 0x48);
		writel(val | 0x1, gen_subsys_regs + 0x48);
		udelay(10);
		writel(val & ~(0x1), gen_subsys_regs + 0x48);
	} else if (exynos_pcie->ch_num == 1 || exynos_pcie->ch_num == 3) {
		/*	PE0 COLD reset */
		val = readl(gen_subsys_regs + 0x48);
		writel(val | (0x1<<8), gen_subsys_regs + 0x48);
		udelay(10);
		writel(val & ~(0x1<<8), gen_subsys_regs + 0x48);
	}

	if (exynos_pcie->ch_num == 0) {
		val = readl(sysreg_base_regs + 0x828);
		writel(val | (0x4<<24), sysreg_base_regs + 0x828);
	} else if (exynos_pcie->ch_num == 1) {
		val = readl(sysreg_base_regs + 0x848);
		writel(val | (0x4<<25), sysreg_base_regs + 0x848);
	} else if (exynos_pcie->ch_num == 2) {
		val = readl(sysreg_base_regs + 0x868);
		writel(val | (0x4<<24), sysreg_base_regs + 0x868);
	} else if (exynos_pcie->ch_num == 3) {
		val = readl(sysreg_base_regs + 0x888);
		writel(val | (0x4<<25), sysreg_base_regs + 0x888);
	}

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
		if (exynos_pcie->use_bifurcation)
			writel(val & (0x2<<16), e32_phy_regs + 0x10);

		//PHY0_REFB_CLK_SEL
		val = readl(e32_phy_regs + 0x10);
		writel(val & ~(0x3<<18), e32_phy_regs + 0x10);
		if (exynos_pcie->use_bifurcation)
			writel(val & (0x2<<18), e32_phy_regs + 0x10);

		val = readl(e32_phy_regs + 0x10);

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

		if (exynos_pcie->use_bifurcation) {
			/*	Enable Enable Reference Clock Detection for refA/B	*/
			writel(0x061A0060, e32_phy_regs + 0x10);
		} else {
			/*	Enable Enable Reference Clock Detection for refA/B	*/
			writel(0x06100060, e32_phy_regs + 0x10);
		}
	}

	writel(0x10, elbi_base_regs + 0x0);
	udelay(10);

	writel(0x4, elbi_base_regs + 0x0);

	if (exynos_pcie->ch_num == 0 || exynos_pcie->ch_num == 2) {
		/*	PE0 SW reset */
		val = readl(gen_subsys_regs + 0x48);
		writel(val | (0x3<<1), gen_subsys_regs + 0x48);
		udelay(10);
		writel(val & ~(0x3<<1), gen_subsys_regs + 0x48);
	} else if (exynos_pcie->ch_num == 1 || exynos_pcie->ch_num == 3) {
		/*	PE0 SW reset */
		val = readl(gen_subsys_regs + 0x48);
		writel(val | (0x3<<9), gen_subsys_regs + 0x48);
		udelay(10);
		writel(val & ~(0x3<<9), gen_subsys_regs + 0x48);
	}

	/*	PE1 SW reset */
	val = readl(gen_subsys_regs + 0x48);
	val = 0x808;
	writel(val, gen_subsys_regs + 0x48);
	udelay(10);

	if (phy_config_level == 0) {

		/*	Set SRAM bypass	*/
		val = readl(e32_phy_regs + 0x10);
		writel(val | (0x1<<10), e32_phy_regs + 0x10);

		timeout = 0;
		do {
			udelay(1);
			timeout++;
			if (timeout == 2000) {
				dev_info(pci->dev, "SRAM bypass FAIL\n");
				*result = 0;
				break;
			}
		} while (!(readl(e32_phy_regs + 0x10) >> 31));

		timeout = 0;
		/*	Set PHY*_SRAM_EXT_LD_DONE */
		val = readl(e32_phy_regs + 0x10);
		writel(val | (0x1<<11), e32_phy_regs + 0x10);

		do {
			udelay(1);
			timeout++;
			if (timeout == 2000) {
				val = readl(e32_phy_regs + 0x10);
				dev_info(pci->dev, "CDM init FAIL\n");
				*result = 0;
				break;
			}
		} while ((readl(elbi_base_regs + 0x304) >> 24) & 0x1);
		timeout = 0;
	}

	if (exynos_pcie->use_bifurcation && (exynos_pcie->ch_num == 0 || exynos_pcie->ch_num == 2)) {
		if (exynos_pcie->ch_num == 0) {
			val = readl(sysreg_base_regs + 0x828);
			val = val | (1 << 28);
			writel(val, (sysreg_base_regs + 0x828));
		} else if (exynos_pcie->ch_num == 2) {
			val = readl(sysreg_base_regs + 0x868);
			val = val | (1 << 28);
			writel(val, (sysreg_base_regs + 0x868));
		}
	}
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_rc_physet}
 * @purpose "Misc setting for phy"
 * @logic "Select REFCLK\n
 *	Enable lane power."
 * @params
 * @param{in, pp, struct ::pcie_port *, not NULL}
 * @endparam
 * @noret
 */
void exynos_v920_host_pcie_others_set(struct pcie_port *pp)
{
	/* For SRIS  PHY settings */
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_rc_physet}
 * @purpose "Initialize phy ops"
 * @logic "Initialize phy ops"
 * @params
 * @param{in/out, pp, struct ::pcie_port *, not NULL}
 * @endparam
 * @noret
 */
void exynos_v920_pcie_phy_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);

	dev_info(pci->dev, "Initialize v920 PHY functions.\n");

	exynos_pcie->phy_ops.phy_all_pwrdn = exynos_v920_phy_all_pwrdn;
	exynos_pcie->phy_ops.phy_all_pwrdn_clear = exynos_v920_phy_all_pwrdn_clear;
	exynos_pcie->phy_ops.phy_config = exynos_v920_pcie_phy_config;
}
EXPORT_SYMBOL(exynos_v920_pcie_phy_init);
