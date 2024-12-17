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
#include "pcie-designware.h"
#include "pcie-exynos-rc-auto.h"

void exynos_host_1_pcie_others_set(struct pcie_port *pp);

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
void exynos_phy_all_pwrdn(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);

	u32 val;
	int pcie_channel_sel_reg;
	int pcie_bifur_sel_reg;
	u32 linkA, linkB, selection = 0;

	pcie_channel_sel_reg = exynos_pcie->ch_num >> 1;
	pcie_bifur_sel_reg = exynos_pcie->ch_num % 2;

	linkA = readl(exynos_pcie->phy_pcs_base + 0x988);
	linkB = readl(exynos_pcie->phy_pcs_base + 0x188);
	dev_info(pci->dev, "[%s] linkA:%d linkB:%d\n", __func__, linkA, linkB);

	if (((linkA >= 2 && linkA <= 3) && (linkB >= 2 && linkB <= 3))
		|| (!exynos_pcie->use_bifurcation)) {
		selection = 1;
	} else if ((linkA >= 2 && linkA <= 3) && linkB == 0) {
		if (pcie_bifur_sel_reg == 1)
			selection = 1;
	} else if (linkA == 0 && (linkB >= 2 && linkB <= 3)) {
		if (pcie_bifur_sel_reg == 0)
			selection = 1;
	}

	if (selection) {
		/* phy all power down */
		val = readl(exynos_pcie->phy_base + 0x400);
		val |= 0x3;
		writel(val, exynos_pcie->phy_base + 0x400);

		if (pcie_channel_sel_reg == 0) {
			dev_info(pci->dev, "[%s] phy power down 4L - CH[%d]\n",
					__func__, pcie_channel_sel_reg);
		} else {
			dev_info(pci->dev, "[%s] phy power down 2L - CH[%d]\n",
					__func__, pcie_channel_sel_reg);
		}
	}
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
void exynos_phy_all_pwrdn_clear(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);
	u32 val;

	dev_info(pci->dev, "[%s]\n", __func__);
	val = readl(exynos_pcie->phy_base + 0x400);
	val &= ~(0x3);
	writel(val, exynos_pcie->phy_base + 0x400);
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
 * @endparam
 * @noret
 */
void exynos_pcie_phy_config(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);

	u32 val;
	int pcie_channel_sel_reg;
	int pcie_bifur_sel_reg;
	int i, j;
	int common_reg_count = 8, arr_size;
	int lane_total_num = 2;
	u32 linkA, linkB, phy_config_level = 1;

	const u32 phy_offset_reg[] = {
		0x030, 0x044, 0x078, 0x07C, 0x18C, 0x214, 0x228, 0x450,
		0x830, 0x838, 0x83C, 0x848, 0x8EC, 0x8F0, 0x8F8, 0x8FC,
		0x914, 0x990, 0x994, 0x99C, 0x9A0, 0x9A4, 0xA18, 0xA1C,
		0xB4C, 0xB58, 0xBA0, 0xBA4, 0xBA8};

	const u8 phy_setting_value[] = {
		0xB9, 0x03, 0x2F, 0xF8, 0x4D, 0x1C, 0x56, 0x8A,
		0xFE, 0x12, 0xD8, 0x00, 0x3A, 0x1E, 0x1E, 0xF0,
		0x70, 0xE4, 0x0E, 0x44, 0x23, 0x5E, 0x3E, 0x00,
		0x25, 0x24, 0x05, 0x55, 0x6A};

	void __iomem *elbi_base_regs = exynos_pcie->elbi_base;
	void __iomem *phy_base_regs = exynos_pcie->phy_base;
	void __iomem *phy_pcs_base_regs = exynos_pcie->phy_pcs_base;
	void __iomem *elbi_cmn_base_regs = exynos_pcie->elbi_cmn_base;

	pcie_channel_sel_reg = exynos_pcie->ch_num >> 1;
	pcie_bifur_sel_reg = exynos_pcie->ch_num % 2;

	linkA = readl(phy_pcs_base_regs + 0x988);
	linkB = readl(phy_pcs_base_regs + 0x188);
	dev_info(pci->dev, "[%s] linkA:%d linkB:%d\n", __func__, linkA, linkB);

	if (((linkA >= 2 && linkA <= 3) && (linkB >= 2 && linkB <= 3))
		|| (!exynos_pcie->use_bifurcation))
		phy_config_level = 0;

	dev_info(pci->dev, "[%s] phy_config_level:%d\n", __func__,
			phy_config_level);

	if (phy_config_level == 0) {
		exynos_host_1_pcie_others_set(pp);
		udelay(10);

		/* pcs_g_rst */
		val = readl(elbi_cmn_base_regs);
		val |= (0x1 << 0);
		writel(val, elbi_cmn_base_regs);
		val = readl(elbi_cmn_base_regs);
		val &= ~(0x1 << 0);
		writel(val, elbi_cmn_base_regs);
		udelay(10);
		val = readl(elbi_cmn_base_regs);
		val |= (0x1 << 0);
		writel(val, elbi_cmn_base_regs);
	}

	/* RC selection */
	writel(0x4, elbi_base_regs + 0x80);
	udelay(10);

	if (exynos_pcie->use_sris && phy_config_level == 0) {
		writel(1, elbi_base_regs + PCIE_SRIS_MODE);
		dev_info(pci->dev, "[%s] SRIS enable\n", __func__);
	}

	if (phy_config_level == 0) {

		/* PHY Setting: 26MHz or 100MHz */
		if (pcie_channel_sel_reg == 0)
			lane_total_num = 4;

		for (i = 0; i < common_reg_count; i++) {
			writel(phy_setting_value[i], phy_base_regs
					+ phy_offset_reg[i]);
		}

		arr_size = ARRAY_SIZE(phy_offset_reg);
		for (j = 0; j < lane_total_num; j++) {
			for (i = common_reg_count; i < arr_size; i++) {
				writel(phy_setting_value[i],
						phy_base_regs
						+ phy_offset_reg[i]
						+ (0x800 * j));
			}
		}

		/* Additional PHY Setting */
		/* Additional PCS Setting */
		/* Lane Map & Bifurcation setting */
		if (!exynos_pcie->use_bifurcation) {
			if (pcie_channel_sel_reg == 0)
				val = 0x0F;
			else
				val = 0x03;

			writel(val, phy_pcs_base_regs + 0x004);
			writel(val, phy_pcs_base_regs + 0x804);

			/* Bifurcation disable: ~(0x01 << 6) */
			val = readl(phy_base_regs + 0x400);
			val &= ~(0x01 << 6);
			writel(val, phy_base_regs + 0x400);

			/* REFCLK OUT setting --> REFCLK0 output only */
			writel(0x01, phy_base_regs + 0x438);
			writel(0x5D, phy_base_regs + 0x478);
		} else {
			/*	Set bifurcation reset */
			if (pcie_channel_sel_reg == 0)
				writel(0x50, phy_pcs_base_regs + 0x4DC);
			else
				writel(0x40, phy_pcs_base_regs + 0x4DC);

			/* Bifurcation On */
			if (pcie_channel_sel_reg == 0)
				val = 0x03;
			else
				val = 0x01;

			writel(val, phy_pcs_base_regs + 0x004);
			writel(val, phy_pcs_base_regs + 0x804);

			/* Bifurcation Enable: (0x01 << 6) */
			val = readl(phy_base_regs + 0x400);
			val |= (0x01 << 6);
			writel(val, phy_base_regs + 0x400);

			/* REFCLK OUT setting --> REFCLK0 & REFCLK1 output */
			writel(0x03, phy_base_regs + 0x438);
			writel(0x4D, phy_base_regs + 0x478);

			/* REFCLK OUT setting --> REFCLK0 & REFCLK1 output */
			writel(0x2C, phy_base_regs + 0x224);
			writel(0xCB, phy_base_regs + 0x238);
		}

		/* gen3 */
		if (pcie_channel_sel_reg == 0) {
			writel(0x1F, phy_base_regs + 0x0818);
			writel(0x77, phy_base_regs + 0x0820);
			writel(0x0F, phy_base_regs + 0x0A40);
			writel(0x0F, phy_base_regs + 0x0A44);
			writel(0x0F, phy_base_regs + 0x0A48);
			writel(0x1C, phy_base_regs + 0x0B98);

			writel(0x1F, phy_base_regs + 0x1018);
			writel(0x77, phy_base_regs + 0x1020);
			writel(0x0F, phy_base_regs + 0x1240);
			writel(0x0F, phy_base_regs + 0x1244);
			writel(0x0F, phy_base_regs + 0x1248);
			writel(0x1C, phy_base_regs + 0x1398);

			writel(0x1F, phy_base_regs + 0x1818);
			writel(0x77, phy_base_regs + 0x1820);
			writel(0x0F, phy_base_regs + 0x1A40);
			writel(0x0F, phy_base_regs + 0x1A44);
			writel(0x0F, phy_base_regs + 0x1A48);
			writel(0x1C, phy_base_regs + 0x1B98);

			writel(0x1F, phy_base_regs + 0x2018);
			writel(0x77, phy_base_regs + 0x2020);
			writel(0x0F, phy_base_regs + 0x2240);
			writel(0x0F, phy_base_regs + 0x2244);
			writel(0x0F, phy_base_regs + 0x2248);
			writel(0x1C, phy_base_regs + 0x2398);

			dev_info(pci->dev, "lane0(0xD24)=0x%x\n",
					readl(phy_base_regs + 0xD24));
			dev_info(pci->dev, "lane1(0x1524)=0x%x\n",
					readl(phy_base_regs + 0x1524));
			dev_info(pci->dev, "lane2(0x1D24)=0x%x\n",
					readl(phy_base_regs + 0x1D24));
			dev_info(pci->dev, "lane3(0x2524)=0x%x\n",
					readl(phy_base_regs + 0x2524));
		} else {
			writel(0x1F, phy_base_regs + 0x0818);
			writel(0x77, phy_base_regs + 0x0820);
			writel(0x0F, phy_base_regs + 0x0A40);
			writel(0x0F, phy_base_regs + 0x0A44);
			writel(0x0F, phy_base_regs + 0x0A48);
			writel(0x1C, phy_base_regs + 0x0B98);

			writel(0x1F, phy_base_regs + 0x1018);
			writel(0x77, phy_base_regs + 0x1020);
			writel(0x0F, phy_base_regs + 0x1240);
			writel(0x0F, phy_base_regs + 0x1244);
			writel(0x0F, phy_base_regs + 0x1248);
			writel(0x1C, phy_base_regs + 0x1398);

			dev_info(pci->dev, "lane0(0xD24)=0x%x\n",
					readl(phy_base_regs + 0xD24));
			dev_info(pci->dev, "lane1(0x1524)=0x%x\n",
					readl(phy_base_regs + 0x1524));
		}

		dev_info(pci->dev, "[%s] bifurcation?(to be 0x0, 0x40) 0x%x\n",
			__func__, readl(phy_base_regs + 0x400));

		udelay(10);

		dev_info(pci->dev, "[%s] PCIE_MAC RST\n", __func__);
		/* PCIE_MAC RST */
		val = readl(elbi_base_regs + 0x1400);
		val |= (0x1 << 0);
		writel(val, elbi_base_regs + 0x1400);

		val = readl(elbi_base_regs + 0x1400);
		val &= ~(0x1 << 0);
		writel(val, elbi_base_regs + 0x1400);
		udelay(10);

		val = readl(elbi_base_regs + 0x1400);
		val |= (0x1 << 0);
		writel(val, elbi_base_regs + 0x1400);
		udelay(10);

		/* PCIE_PHY PCS&PMA(CMN)_RST */
		val = readl(elbi_cmn_base_regs + 0x4);
		val |= (0x1 << 0);
		writel(val, elbi_cmn_base_regs + 0x4);

		val = readl(elbi_cmn_base_regs + 0x4);
		val &= ~(0x1 << 0);
		writel(val, elbi_cmn_base_regs + 0x4);
		udelay(10);

		val = readl(elbi_cmn_base_regs + 0x4);
		val |= (0x1 << 0);
		writel(val, elbi_cmn_base_regs + 0x4);
		udelay(10);
	}

	if (exynos_pcie->use_sris && phy_config_level == 0) {
		writel(1, exynos_pcie->elbi_base + PCIE_SRIS_MODE);
		dev_info(pci->dev, "[%s] SRIS enable\n", __func__);
	}
	udelay(10);

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
void exynos_host_1_pcie_others_set(struct pcie_port *pp)
{
	u32 val;
	int pcie_channel_sel_reg;
	int pcie_bifur_sel_reg;

	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);

	pcie_channel_sel_reg = exynos_pcie->ch_num >> 1;
	pcie_bifur_sel_reg = exynos_pcie->ch_num % 2;

	/* sysreg setting */
	/* REFCLK selection for PHY input (26Mhz) */
	val = readl(exynos_pcie->sysreg_base);
	val &= ~((0x1 << 1) | (0x1 << 5) | (0x1 << 4));
	writel(val, exynos_pcie->sysreg_base);

	/* Lane power enable */
	val = readl(exynos_pcie->sysreg_base);
	if (pcie_channel_sel_reg == 0)
		val &= ~(0xF00);
	else
		val &= ~(0x300);
	writel(val, exynos_pcie->sysreg_base);

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
void exynos_pcie_phy_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);

	dev_info(pci->dev, "Initialize PHY functions.\n");

	exynos_pcie->phy_ops.phy_all_pwrdn = exynos_phy_all_pwrdn;
	exynos_pcie->phy_ops.phy_all_pwrdn_clear = exynos_phy_all_pwrdn_clear;
	exynos_pcie->phy_ops.phy_config = exynos_pcie_phy_config;
}
EXPORT_SYMBOL(exynos_pcie_phy_init);
