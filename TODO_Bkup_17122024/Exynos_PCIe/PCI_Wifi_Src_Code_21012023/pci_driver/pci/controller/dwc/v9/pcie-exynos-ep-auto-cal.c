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

#include "pcie-exynos-ep-auto.h"
#include "pcie-exynos-ep-auto-cal.h"

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
void exynos_pcie_ep_phy_all_pwrdn(struct exynos_ep_pcie_phyinfo *phyinfo)
{
	u32 val;
	int pcie_channel_sel_reg;
	int pcie_bifur_sel_reg;
	u32 linkA = 0, linkB = 0, selection = 0;

	pcie_channel_sel_reg = phyinfo->ch_num >> 1;
	pcie_bifur_sel_reg = phyinfo->ch_num % 2;

	linkA = phyinfo->linkA_status;
	linkB = phyinfo->linkB_status;
	dev_info(NULL, "[%s] linkA:%d linkB:%d\n", __func__, linkA, linkB);

	if (((linkA >= 2 && linkA <= 3) && (linkB >= 2 && linkB <= 3))
		|| (!phyinfo->use_bifurcation)) {
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
		val = readl(phyinfo->phy_base + 0x400);
		val |= 0x3;
		writel(val, phyinfo->phy_base + 0x400);

		if (pcie_channel_sel_reg == 0) {
			dev_info(NULL, "[%s] phy power down 4L - CH[%d]\n",
					__func__, pcie_channel_sel_reg);
		} else {
			dev_info(NULL, "[%s] phy power down 2L - CH[%d]\n",
					__func__, pcie_channel_sel_reg);
		}
	}
}
EXPORT_SYMBOL(exynos_pcie_ep_phy_all_pwrdn);

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
void exynos_pcie_ep_phy_all_pwrdn_clear(struct exynos_ep_pcie_phyinfo *phyinfo)
{
	void __iomem *phy_base_regs = phyinfo->phy_base;
	u32 val;

	val = readl(phy_base_regs + 0x400);
	val &= ~(0x3);
	writel(val, phy_base_regs + 0x400);

	dev_info(NULL, "[%s] val: 0x%x , done!\n", __func__, val);
}
EXPORT_SYMBOL(exynos_pcie_ep_phy_all_pwrdn_clear);

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
 * @endparam
 * @noret
 */
void exynos_pcie_ep_phy_config(struct exynos_ep_pcie_phyinfo *phyinfo)
{
	void __iomem *phy_base_regs = phyinfo->phy_base;
	void __iomem *elbi_base_regs = phyinfo->elbi_base;
	void __iomem *phy_pcs_base_regs = phyinfo->phy_pcs_base;
	int i, j, lane_total_num, total_reg_size;
	int pcie_channel_sel_reg = phyinfo->ch_num >> 1;
	int common_reg_count = 8;
	int ep_only_reg_count = 10;
	u32 linkA, linkB, phy_config_level = 1;

	const u32 phy_offset_reg[] = {
		0x030, 0x044, 0x078, 0x07C, 0x18C, 0x214, 0x228, 0x450,
		0x830, 0x838, 0x83C, 0x848, 0x8EC, 0x8F0, 0x8F8, 0x8FC,
		0x914, 0x990, 0x994, 0x99C, 0x9A0, 0x9A4, 0xA18, 0xA1C,
		0xB4C, 0xB58, 0xBA0, 0xBA4, 0xBA8};

	const u8 phy_setting_value[] = {
		0xB9, 0x03, 0x2F, 0xF8, 0x4D, 0x1C, 0x56, 0x8A, /* common */
		0xFE, 0x12, 0xD8, 0x00, 0x3A, 0x1E, 0x1E, 0xF0,
		0x70, 0xE4, 0x0E, 0x44, 0x23, 0x5E, 0x3E, 0x00,
		0x25, 0x24, 0x05, 0x55, 0x6A};

	const u32 additional_phy_offset_reg[] = {
		0x094, 0x0B0, 0x0B4, 0x190, 0x194,
		0x1B0, 0x224, 0x400, 0x414, 0x438, /* EP only */
		0xACC, 0xAD4, 0xADC, 0xAE4, 0xC50};

	const u8 additional_phy_setting_value[] = {
		0x14, 0x5B, 0x25, 0x64, 0x14,
		0x11, 0x2C, 0x20, 0x05, 0x00, /* EP only */
		0xC8, 0xC8, 0xA0, 0x40, 0x00};

	linkA = readl(phy_pcs_base_regs + 0x988);
	phyinfo->linkA_status = linkA;
	linkB = readl(phy_pcs_base_regs + 0x188);
	phyinfo->linkB_status = linkB;

	if (((linkA >= 2 && linkA <= 3) && (linkB >= 2 && linkB <= 3))
		|| (!phyinfo->use_bifurcation)) {
		phy_config_level = 0;
	}

	if (phy_config_level == 0) {
		exynos_pcie_ep_phy_others_set(phyinfo);
		udelay(10);
		exynos_pcie_ep_phy_g_rst(phyinfo);
	}

	/* EP selection */
	writel(0x0, elbi_base_regs + 0x80);

	if (phyinfo->use_sris && phy_config_level == 0) {
		writel(1, elbi_base_regs + PCIE_SRIS_MODE);
		dev_info(NULL, "[%s] SRIS enable\n", __func__);
	}

	if (phy_config_level == 0) {

		/* PHY Setting: 26MHz or 100MHz */
		if (pcie_channel_sel_reg == 0)
			lane_total_num = 4;
		else
			lane_total_num = 2;

		for (i = 0; i < common_reg_count; i++) {
			writel(phy_setting_value[i],
				phy_base_regs + phy_offset_reg[i]);
		}

		total_reg_size = ARRAY_SIZE(phy_offset_reg);
		for (j = 0; j < lane_total_num; j++) {
			for (i = common_reg_count; i < total_reg_size; i++) {
				writel(phy_setting_value[i],
						phy_base_regs
						+ phy_offset_reg[i]
						+ (0x800 * j));
			}
		}

		if (!phyinfo->use_sris) {
			for (i = 0; i < ep_only_reg_count; i++) {
				writel(additional_phy_setting_value[i],
						phy_base_regs
						+ additional_phy_offset_reg[i]);
			}

			total_reg_size = ARRAY_SIZE(additional_phy_offset_reg);
			for (j = 0; j < lane_total_num; j++) {
				for (i = ep_only_reg_count;
					i < total_reg_size; i++) {
					writel(additional_phy_setting_value[i],
						phy_base_regs
						+ additional_phy_offset_reg[i]
						+ (0x800 * j));
				}
			}

			/* mask_refclk_out_en */
			writel(0x0A, phy_base_regs + 0x450);
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

			dev_info(NULL, "lane0(0xD24)=0x%x\n",
				readl(phy_base_regs + 0xD24));
			dev_info(NULL, "lane1(0x1524)=0x%x\n",
				readl(phy_base_regs + 0x1524));
			dev_info(NULL, "lane2(0x1D24)=0x%x\n",
				readl(phy_base_regs + 0x1D24));
			dev_info(NULL, "lane3(0x2524)=0x%x\n",
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

			dev_info(NULL, "lane0(0xD24)=0x%x\n",
				readl(phy_base_regs + 0xD24));
			dev_info(NULL, "lane1(0x1524)=0x%x\n",
				readl(phy_base_regs + 0x1524));
		}
		udelay(10);

		exynos_pcie_ep_phy_pcs_rst_set(phyinfo);
	}
}
EXPORT_SYMBOL(exynos_pcie_ep_phy_config);

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_ep_physet}
 * @purpose "Additional phy setting"
 * @logic "Set lane mapping and bifurcation\n
 *	PCIE_MAC PCIE_PHY PCS&PMA(CMN) reset"
 * @params
 * @param{in, phyinfo, struct ::exynos_ep_pcie_phyinfo *, not NULL}
 * @endparam
 * @noret
 */
void exynos_pcie_ep_phy_pcs_rst_set(struct exynos_ep_pcie_phyinfo *phyinfo)
{
	void __iomem *phy_base_regs = phyinfo->phy_base;
	void __iomem *phy_pcs_base_regs = phyinfo->phy_pcs_base;
	void __iomem *elbi_base_regs = phyinfo->elbi_base;
	void __iomem *elbi_cmn_base_regs = phyinfo->elbi_cmn_base;

	int ch_num = phyinfo->ch_num;
	int use_bifur = phyinfo->use_bifurcation;
	int val;

	/* Additional PHY Setting */
	/* Additional PCS Setting */
	//Lane Map & Bifurcation setting
	dev_info(NULL, "[%s] CMN_RST, ch_num: %d, use_bifur: %d\n",
			__func__, ch_num, use_bifur);

	/* use_bifur => 1: bifurcation, 0:aggregation */
	if (ch_num == CH0_4L || ch_num == CH1_4L) {
		if (!use_bifur) {
			writel(0xF, phy_pcs_base_regs + 0x004);
			writel(0xF, phy_pcs_base_regs + 0x804);

			/* Bifurcation disable */
			val = readl(phy_base_regs + 0x400);
			val |= 0x20;
			writel(val, phy_base_regs + 0x400);
		} else {
			/*	Set bifurcation reset */
			writel(0x50, phy_pcs_base_regs + 0x4DC);

			writel(0x3, phy_pcs_base_regs + 0x004);
			writel(0x3, phy_pcs_base_regs + 0x804);

			/* Bifurcation enable */
			val = readl(phy_base_regs + 0x400);
			val |= 0x60;
			writel(val, phy_base_regs + 0x400);
		}
	} else {
		if (!use_bifur) {
			writel(0x3, phy_pcs_base_regs + 0x004);
			writel(0x3, phy_pcs_base_regs + 0x804);

			/* Bifurcation disable */
			val = readl(phy_base_regs + 0x400);
			val |= 0x20;
			writel(val, phy_base_regs + 0x400);
		} else {
			/*	Set bifurcation reset */
			writel(0x40, phy_pcs_base_regs + 0x4DC);

			writel(0x1, phy_pcs_base_regs + 0x004);
			writel(0x1, phy_pcs_base_regs + 0x804);

			/* Bifurcation enable */
			val = readl(phy_base_regs + 0x400);
			val |= 0x60;
			writel(val, phy_base_regs + 0x400);
		}
	}

	udelay(10);

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
void exynos_pcie_ep_phy_g_rst(struct exynos_ep_pcie_phyinfo *phyinfo)
{
	void __iomem *elbi_cmn_base_regs = phyinfo->elbi_cmn_base;
	u32 val;

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
void exynos_pcie_ep_phy_others_set(struct exynos_ep_pcie_phyinfo *phyinfo)
{
	void __iomem *sysreg_base = phyinfo->sysreg_base;
	int pcie_channel_sel_reg = phyinfo->ch_num >> 1;
	u32 val;

	/* sysreg setting */
	/* REFCLK selection for PHY input (26Mhz) */
	val = readl(sysreg_base);
	val &= ~SYSREG_HIGH_SPEED;		//PCS_HIGH_SPEED = "0"
	val &= ~SYSREG_LCPLL_REFCLK_SEL1;	//PHY input = 100MHz
	if (phyinfo->use_sris)
		val &= ~SYSREG_LCPLL_REFCLK_SEL0; //PHY input = 26MHz
	else
		val |= SYSREG_LCPLL_REFCLK_SEL0; //PHY input = 100MHz
	writel(val, sysreg_base);

	/* Lane power enable */
	val = readl(sysreg_base);
	if (pcie_channel_sel_reg == 0)
		val &= ~(0xF00);
	else
		val &= ~(0x300);
	writel(val, sysreg_base);
}
EXPORT_SYMBOL(exynos_pcie_ep_phy_others_set);
