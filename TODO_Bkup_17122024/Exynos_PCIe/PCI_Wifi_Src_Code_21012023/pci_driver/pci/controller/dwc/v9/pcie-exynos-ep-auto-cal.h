/* SPDX-License-Identifier: GPL-2.0 */
/**
 * pcie-exynos-ep-auto-cal.h - Exynos PCIe EP cal
 *
 * Copyright (c) 2013-2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Authors: Sanghoon Bae <sh86.bae@samsung.com>
 *	    Jiheon Oh <jiheon.oh@samsung.com>
 *
 */

void exynos_pcie_ep_phy_all_pwrdn(struct exynos_ep_pcie_phyinfo *phyinfo);
void exynos_pcie_ep_phy_all_pwrdn_clear(struct exynos_ep_pcie_phyinfo *phyinfo);
void exynos_pcie_ep_phy_config(struct exynos_ep_pcie_phyinfo *phyinfo);
void exynos_pcie_ep_phy_others_set(struct exynos_ep_pcie_phyinfo *phyinfo);
void exynos_pcie_ep_phy_pcs_rst_set(struct exynos_ep_pcie_phyinfo *phyinfo);
void exynos_pcie_ep_phy_g_rst(struct exynos_ep_pcie_phyinfo *phyinfo);

/* define the register offset map */
