/* SPDX-License-Identifier: GPL-2.0 */
/**
 * pcie-exynos-ep-auto.h - Exynos PCIe EP driver
 *
 * Copyright (c) 2013-2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Authors: Sanghoon Bae <sh86.bae@samsung.com>
 *	    Jiheon Oh <jiheon.oh@samsung.com>
 *
 */


#define to_exynos_pcie_ep(x)	dev_get_drvdata((x)->dev)

/* Exynos9 Auto */
#define MAX_EP_NUM		(6)
#define CLASS_ID			(PCI_CLASS_SYSTEM_OTHER)
#define REVISION_ID			(0x01)
#define MAX_VF_NUM			4

#define MAX_TEST_WR_SIZE		0x1000000
#define WR_DMA_TEST_ADDR		0xDD000000
#define RD_DMA_TEST_ADDR		0xDE000000

#define PCIE_EP_PHY_CONTROL		0x071C
#define PCIE_EP_PHY_CONTROL_MASK	0x1
#define PCIE_EP_GEN3_4L_PHY_CONTROL	0x0724
#define PCIE_EP_GEN3_2L_PHY0_CONTROL	0x071C
#define PCIE_EP_GEN3_2L_PHY1_CONTROL	0x0720

/* BAR */
#define BAR0 0x10
#define BAR1 0x14
#define BAR2 0x18
#define BAR3 0x1C
#define BAR4 0x20
#define BAR5 0x24

/* SUB Controller */
#define PCIE_IRQ0			0x0
#define PCIE_IRQ1			0x4
#define PCIE_IRQ2			0x8
#define PCIE_IRQ3			0xC
#define PCIE_IRQ0_EN			0x10
#define PCIE_IRQ1_EN			0x14
#define PCIE_IRQ2_EN			0x18
#define PCIE_IRQ3_EN			0x1C
#define PCIE_ELBI_RDLH_LINKUP		0x2C8
#define PCIE_INIT_RSTN			0x1404
#define PCIE_SRIS_MODE			0x1818


#define PCIE_PM_STS		0x034

#define PCIE_INT_STS		0x05C

#define PCIE_RX_MSG_INT_STS		0x080
#define PCIE_RX_MSG_INT_EN		0x090

#define PCIE_MISC_INT_STS		0x088
#define PCIE_MISC_INT_EN		0x098

#define CSR_APP_CLK_PM_EN_GEN3		0x20
#define CSR_APP_CLK_REQ_N_GEN3		0x24
#define CSR_APP_ERR_ADVISORY_GEN3	0x28
#define CSR_APP_ERR_BUS_GEN3		0x2C
#define CSR_APP_ERR_FUNC_NUM_GEN3	0x30
#define CSR_APP_HDR_LOG_W0_GEN3		0x34
#define CSR_APP_HDR_LOG_W1_GEN3		0x38
#define CSR_APP_HDR_LOG_W2_GEN3		0x3C
#define CSR_APP_HDR_LOG_W3_GEN3		0x40
#define CSR_APP_HDR_VALID_GEN3		0x44
#define CSR_APP_LTR_MSG_FUNC_NUM_GEN3	0x48
#define CSR_APP_LTR_MSG_LATENCY_GEN3    0x4C
#define CSR_APP_LTR_MSG_REQ_GEN3	0x50
#define CSR_APP_LTSSM_EN_GEN3		0x8
#define CSR_APP_READY_ENTR_L23_GEN3	0x68
#define CSR_APP_REQ_EXIT_L1_GEN3	0x6C
#define CSR_APP_REQ_RETRY_EN_GEN3	0x70
#define CSR_APP_XFER_PENDING_GEN3	0x74

#define PCIE_SOFT_RESET			0x3A4
#define SOFT_CORE_RESET			(0x1 << 0)
#define SOFT_PWR_RESET			(0x1 << 1)
#define SOFT_STICKY_RESET		(0x1 << 2)
#define SOFT_NON_STICKY_RESET		(0x1 << 3)
#define SOFT_PHY_RESET			(0x1 << 4)

#define PCIE_QCH_SEL			0x3A8
#define CLOCK_GATING_PMU_MASK		(0xF << 8)
#define CLOCK_GATING_APB_MASK		(0xF << 4)
#define CLOCK_GATING_AXI_MASK		(0xF << 0)

#define SUBCON_APPREQ_EXIT_L1_MODE	0x3BC

/* channel define */
#define CH0_4L 0
#define CH1_4L 1
#define CH2_2L0 2
#define CH3_2L0 3
#define CH4_2L1 4
#define CH5_2L1 5

/* sysreg */
#define FSYS0_PCIE_GEN3_4LA_PHY_CTRL	0x828
#define FSYS0_PCIE_GEN3_4LB_PHY_CTRL	0x848
#define FSYS0_PCIE_GEN3_2LA_PHY_CTRL	0x868
#define FSYS0_PCIE_GEN3_2LB_PHY_CTRL	0x888

#define SYSREG_HIGH_SPEED		(0x1 << 1)
#define SYSREG_LCPLL_REFCLK_SEL0	(0x1 << 4)
#define SYSREG_LCPLL_REFCLK_SEL1	(0x1 << 5)
#define SYSREG_LN0_POWER_OFF		(0x1 << 8)
#define SYSREG_LN1_POWER_OFF		(0x1 << 9)
#define SYSREG_LN2_POWER_OFF		(0x1 << 10)
#define SYSREG_LN3_POWER_OFF		(0x1 << 11)

#define FSYS0_PCIE_GEN3A_4L_IP_CTRL	0x828
#define FSYS0_PCIE_GEN3B_4L_IP_CTRL	0x848
#define FSYS0_PCIE_GEN3A_2L0_IP_CTRL	0x868
#define FSYS0_PCIE_GEN3B_2L0_IP_CTRL	0x888
#define FSYS0_PCIE_GEN3A_2L1_IP_CTRL	0x8a8
#define FSYS0_PCIE_GEN3B_2L1_IP_CTRL	0x8c8

/* irq 0 */
#define VENDOR_MSG		(0x1 << 31)
#define PM_TURNOFF		(0x1 << 19)
#define PM_TO_ACK		(0x1 << 29)
#define PM_PME			(0x1 << 28)
#define NONFATAL_ERR		(0x1 << 27)
#define MSG_UNLOCK		(0x1 << 26)
#define RSVD0			(0x1 << 25)
#define MSG_LTR			(0x1 << 24)
#define RSVD1			(0x1 << 23)
#define RSVD2			(0x1 << 22)
#define INTD_DEASSERTED		(0x1 << 21)
#define INTD_ASSERTED		(0x1 << 20)
#define INTC_DEASSERTED		(0x1 << 19)
#define INTC_ASSERTED		(0x1 << 18)
#define INTB_DEASSERTED		(0x1 << 17)
#define INTB_ASSERTED		(0x1 << 16)
#define INTA_DEASSERTED		(0x1 << 15)
#define INTA_ASSERTED		(0x1 << 14)
#define FATAL_ERR		(0x1 << 13)
#define CORRECTABLE_ERR		(0x1 << 12)
#define HP_PME			(0x1 << 11)
#define HP_MSI			(0x1 << 10)
#define FLUSH_DONE		(0x1 << 9)
#define RSVD3			(0x1 << 8)
#define CFG_SYS_ERR_RC		(0x1 << 7)
#define CFG_PME_MSI		(0x1 << 6)
#define CFG_LINK_AUTO_BW_INT	(0x1 << 5)
#define CFG_BW_MGT_INT		(0x1 << 4)
#define CFG_AER_RC_ERR_MSI	(0x1 << 3)
#define APP_PARITY_ERRS_2	(0x1 << 2)
#define APP_PARITY_ERRS_1	(0x1 << 1)
#define APP_PARITY_ERRS_0	(0x1 << 0)

/* irq 1 */
#define SMLH_REQ_RST_NOT_1	(0x1 << 11)
#define LINK_DOWN		(0x1 << 10)
#define SMLH_LINK_DISABLE	(0x1 << 9)
#define PERST_N_1		(0x1 << 11)
#define PERST_N_0		(0x1 << 7)
#define RATE_CHANGED_0		(0x1 << 6)
#define RATE_CHANGED_1		(0x1 << 5)
#define CFG_ACCESSED		(0x1 << 4)
#define SMLH_TRAINING_RST_N	(0x1 << 3)
#define VEN_MSI_GRANT		(0x1 << 2)
#define VEN_MSG_GRANT		(0x1 << 1)
#define TRGT_CPL_TIMEOUT	(0x1 << 0)

/* irq 2 */
#define COMPARE_END_CHECKER	(0x1 << 31)
#define WAKE			(0x1 << 30)
#define RTLH_RFC_UPD_0_1	(0x1 << 29)
#define RTLH_RFC_UPD_0_0	(0x1 << 28)
#define RDLH_LINK_UP_1		(0x1 << 27)
#define RDLH_LINK_UP_0		(0x1 << 26)
#define Q_NOT_EMPTY		(0x1 << 25)
#define RADM_CPL_TIMEOUT	(0x1 << 24)
#define PM_XLTH_BLOCK_TLP	(0x1 << 23)
#define PM_LINKST_L2_EXIT	(0x1 << 22)
#define PM_LINKST_IN_L2		(0x1 << 14)
#define MSI_CTRL_INT		(0x1 << 20)
#define PM_LINKST_IN_L1		(0x1 << 19)
#define PM_LINKST_IN_L0S	(0x1 << 18)
#define MSI_CTRL_INT_1		(0x1 << 17)
#define MSI_CTRL_INT_0		(0x1 << 16)
#define LINK_REQ_RST_NOT_1	(0x1 << 15)
#define LINK_REQ_RST_NOT_0	(0x1 << 14)
#define SMLH_LINKUP_1		(0x1 << 13)
#define SMLH_LINKUP_0		(0x1 << 12)
#define PM_LINKST_IN_L1SUB_1	(0x1 << 11)
#define PM_LINKST_IN_L1SUB_0	(0x1 << 10)
#define MSI_STATUS_SEC_EN	(0x1 << 9)
#define HP_INT			(0x1 << 8)
#define RTLH_RFC_UPD_1_1	(0x1 << 7)
#define RTLH_RFC_UPD_1_0	(0x1 << 6)
#define CFG_PME_INT		(0x1 << 5)
#define CFG_LINK_EQ_REQ_INT	(0x1 << 4)
#define CFG_BUS_MASTER_EN	(0x1 << 3)
#define CFG_AER_RC_ERR_INT	(0x1 << 2)
#define QOVERFLOW		(0x1 << 1)
#define APP_LTR_MSG_GRANT	(0x1 << 0)

/* irq3 */
#define WR_CH0 (0x1 << 16)
#define WR_CH1 (0x1 << 17)
#define WR_CH2 (0x1 << 18)
#define WR_CH3 (0x1 << 19)
#define WR_CH4 (0x1 << 20)
#define WR_CH5 (0x1 << 21)
#define WR_CH6 (0x1 << 22)
#define WR_CH7 (0x1 << 23)
#define RD_CH0 (0x1 << 24)
#define RD_CH1 (0x1 << 25)
#define RD_CH2 (0x1 << 26)
#define RD_CH3 (0x1 << 27)
#define RD_CH4 (0x1 << 28)
#define RD_CH5 (0x1 << 29)
#define RD_CH6 (0x1 << 30)
#define RD_CH7 (0x1 << 31)


// PCIE Capability Register fields
#define CAP_REGSHIFT_VERSION		0
#define CAP_REGMASK_VERSION		(0xF << CAP_REGSHIFT_VERSION)
#define CAP_REGSHIFT_PORTTYPE		4
#define CAP_REGMASK_PORTTYPE		(0xF << CAP_REGSHIFT_PORTTYPE)
#define CAP_REGSHIFT_SLOTIMP		8
#define CAP_REGMASK_SLOTIMP		(1 << CAP_REGSHIFT_SLOTIMP)
#define CAP_REGSHIFT_INTRNUM		9
#define CAP_REGMASK_INTRNUM		(0xF << CAP_REGSHIFT_INTRNUM)
#define CAP_MC_REGSHIFT_64BIT		7
#define CAP_MC_REGMASK_64BIT		(1 << CAP_MC_REGSHIFT_64BIT)
#define CAP_MC_REGSHIFT_MSG		7

#define RC_REGSHIFT_VENDOR_ID		0
#define RC_REGMASK_VENDOR_ID		(0xFFFF << RC_REGSHIFT_VENDOR_ID)
#define RC_REGSHIFT_DEVICE_ID		16
#define RC_REGMASK_DEVICE_ID		(0xFFFF << RC_REGSHIFT_DEVICE_ID)
#define RC_REGSHIFT_SCRAMBLE		1
#define RC_REGMASK_SCRAMBLE		(1 << RC_REGSHIFT_SCRAMBLE)
#define RC_REGSHIFT_LINK_MODE		16
#define RC_REGMASK_LINK_MODE		(0x3F << RC_REGSHIFT_LINK_MODE)
#define RC_REGSHIFT_LINK_WIDTH		8
#define RC_REGMASK_LINK_WIDTH		(0x1FF << RC_REGSHIFT_LINK_WIDTH)
#define RC_REGSHIFT_DBI_ACCESS		21
#define RC_REGSHIFT_TLP_TYPE		0
#define RC_REGSHIFT_LINK_NUM		0
#define RC_REGMASK_LINK_NUM		(0xF << RC_REGSHIFT_LINK_NUM)
#define RC_REGSHIFT_LINK_COMMAND	16
#define RC_REGMASK_LINK_COMMAND		(0x1F << RC_REGSHIFT_LINK_COMMAND)
#define RC_REGSHIFT_FORCE_LINK		15
#define RC_REGMASK_FORCE_LINK		(0x1 << RC_REGSHIFT_LINK_COMMAND)
#define RC_REGSHIFT_IO_EN		0
#define RC_REGMASK_IO_EN		(0x1 << RC_REGSHIFT_IO_EN)
#define RC_REGSHIFT_MEM_EN		1
#define RC_REGMASK_MEM_EN		(0x1 << RC_REGSHIFT_MEM_EN)
#define RC_REGSHIFT_BUS_MASTER_EN	2
#define RC_REGMASK_BUS_MASTER_EN	(0x1 << RC_REGSHIFT_BUS_MASTER_EN)
#define RC_REGSHIFT_SERR_EN		8
#define RC_REGMASK_SERR_EN		(0x1 << RC_REGSHIFT_SERR_EN)
#define RC_REGSHIFT_PARR_ERR_EN		6
#define RC_REGMASK_PARR_ERR_EN		(0x1 << RC_REGSHIFT_PARR_ERR_EN)
#define RC_REGSHIFT_DIRECTION		31
#define RC_REGSHIFT_SPEED_CHANGE	17
#define RC_REGMASK_SPEED_CHANGE		(0x1 << RC_REGSHIFT_SPEED_CHANGE)
#define RC_REGSHIFT_CORRECTABLE_ERR_EN	0
#define RC_REGMASK_CORRECTABLE_ERR_EN	(0x1 << RC_REGSHIFT_CORRECTABLE_ERR_EN)
#define RC_REGSHIFT_NONFATAL_ERR_EN	1
#define RC_REGMASK_NONFATAL_ERR_EN	(0x1 << RC_REGSHIFT_NONFATAL_ERR_EN)
#define RC_REGSHIFT_FATAL_ERR_EN	2
#define RC_REGMASK_FATAL_ERR_EN		(0x1 << RC_REGSHIFT_FATAL_ERR_EN)
#define RC_REGSHIFT_ERR_COR_RCV		0
#define RC_REGMASK_ERR_COR_RCV		(0x1 << RC_REGSHIFT_ERR_COR_RCV)
#define RC_REGSHIFT_ERR_MCOR_RCV	1
#define RC_REGMASK_ERR_MCOR_RCV		(0x1 << RC_REGSHIFT_ERR_MCOR_RCV)
#define RC_REGSHIFT_ERR_FAT_RCV		2
#define RC_REGMASK_ERR_FAT_RCV		(0x1 << RC_REGSHIFT_ERR_FAT_RCV)
#define RC_REGSHIFT_ERR_MFAT_RCV	3
#define RC_REGMASK_ERR_MFAT_RCV		(0x1 << RC_REGSHIFT_ERR_MFAT_RCV)
#define RC_REGSHIFT_FIRST_FAT_RCV	4
#define RC_REGMASK_FIRST_FAT_RCV	(0x1 << RC_REGSHIFT_FIRST_FAT_RCV)
#define RC_REGSHIFT_NON_FAT_RCV		5
#define RC_REGMASK_NON_FAT_RCV		(0x1 << RC_REGSHIFT_NON_FAT_RCV)
#define RC_REGSHIFT_MSG_FAT_RCV		6
#define RC_REGMASK_MSG_FAT_RCV		(0x1 << RC_REGSHIFT_MSG_FAT_RCV)
#define RC_REGSHIFT_RCV_ERR_STATUS	0
#define RC_REGMASK_RCV_ERR_STATUS	(0x1 << RC_REGSHIFT_RCV_ERR_STATUS)
#define RC_REGSHIFT_BAD_TLP		6
#define RC_REGMASK_BAD_TLP		(0x1 << RC_REGSHIFT_BAD_TLP)
#define RC_REGSHIFT_BAD_DLLP		7
#define RC_REGMASK_BAD_DLLP		(0x1 << RC_REGSHIFT_BAD_DLLP)
#define RC_REGSHIFT_RN_ROLLOVER		8
#define RC_REGMASK_RN_ROLLOVER		(0x1 << RC_REGSHIFT_RN_ROLLOVER)
#define RC_REGSHIFT_TIMEOUT		12
#define RC_REGMASK_TIMEOUT		(0x1 << RC_REGSHIFT_TIMEOUT)
#define RC_REGSHIFT_NONFATAL		13
#define RC_REGMASK_NONFATAL		(0x1 << RC_REGSHIFT_NONFATAL)
#define RC_REGSHIFT_CORINT		14
#define RC_REGMASK_CORINT		(0x1 << RC_REGSHIFT_CORINT)
#define RC_REGSHIFT_LINKERR		4
#define RC_REGMASK_LINKERR		(0x1 << RC_REGSHIFT_LINKERR)
#define RC_REGSHIFT_POISONED		12
#define RC_REGMASK_POISONED		(0x1 << RC_REGSHIFT_POISONED)
#define RC_REGSHIFT_FLOWCONTROL		13
#define RC_REGMASK_FLOWCONTROL		(0x1 << RC_REGSHIFT_FLOWCONTROL)
#define RC_REGSHIFT_CTIMEOUT		14
#define RC_REGMASK_CTIMEOUT		(0x1 << RC_REGSHIFT_CTIMEOUT)
#define RC_REGSHIFT_CABORT		15
#define RC_REGMASK_CABORT		(0x1 << RC_REGSHIFT_CABORT)
#define RC_REGSHIFT_UNEXPECTEDC		16
#define RC_REGMASK_UNEXPECTEDC		(0x1 << RC_REGSHIFT_UNEXPECTEDC)
#define RC_REGSHIFT_ROVERFLOW		17
#define RC_REGMASK_ROVERFLOW		(0x1 << RC_REGSHIFT_ROVERFLOW)
#define RC_REGSHIFT_MALFORMEDTLP	18
#define RC_REGMASK_MALFORMEDTLP		(0x1 << RC_REGSHIFT_MALFORMEDTLP)
#define RC_REGSHIFT_ECRC_ERR		19
#define RC_REGMASK_ECRC_ERR		(0x1 << RC_REGSHIFT_ECRC_ERR)
#define RC_REGSHIFT_UR_ERR		20
#define RC_REGMASK_UR_ERR		(0x1 << RC_REGSHIFT_UR_ERR)
#define RC_REGSHIFT_UCI_ERR		22
#define RC_REGMASK_UCI_ERR		(0x1 << RC_REGSHIFT_UCI_ERR)
#define RC_REGSHIFT_ATOMIC_BLOCKED	24
#define RC_REGMASK_ATOMIC_BLOCKED	(0x1 << RC_REGSHIFT_ATOMIC_BLOCKED)
#define RC_REGSHIFT_TLPBLOCKED_ERR	25
#define RC_REGMASK_TLPBLOCKED_ERR	(0x1 << RC_REGSHIFT_TLPBLOCKED_ERR)
#define RC_REGSHIFT_CAP_LIST_EXIST	20
#define RC_REGMASK_CAP_LIST_EXIST	(0x1 << RC_REGSHIFT_CAP_LIST_EXIST)
#define RC_REGSHIFT_DMA_H_ERR_UR	0
#define RC_REGMASK_DMA_H_ERR_UR		(0xFF << RC_REGSHIFT_DMA_H_ERR_UR)
#define RC_REGSHIFT_DMA_H_ERR_CA	8
#define RC_REGMASK_DMA_H_ERR_CA		(0xFF << RC_REGSHIFT_DMA_H_ERR_CA)
#define RC_REGSHIFT_DMA_H_ERR_CTO	16
#define RC_REGMASK_DMA_H_ERR_CTO	 (0xFF << RC_REGSHIFT_DMA_H_ERR_CTO)
#define RC_REGSHIFT_DMA_H_ERR_DP	24
#define RC_REGMASK_DMA_H_ERR_DP		(0xFF << RC_REGSHIFT_DMA_H_ERR_DP)
#define RC_REGSHIFT_DMA_L_ERR_AWED	0
#define RC_REGMASK_DMA_L_ERR_AWED	(0xFF << RC_REGSHIFT_DMA_L_ERR_AWED)
#define RC_REGSHIFT_DMA_L_ERR_LLEFED	16
#define RC_REGMASK_DMA_L_ERR_LLEFED	(0xFF << RC_REGSHIFT_DMA_L_ERR_LLEFED)
#define RC_REGSHIFT_DMA_W_ERR_AWED	0
#define RC_REGMASK_DMA_W_ERR_AWED	(0xFF << RC_REGSHIFT_DMA_W_ERR_AWED)
#define RC_REGSHIFT_DMA_W_ERR_LLEFED	16
#define RC_REGMASK_DMA_W_ERR_LLEFED	(0xFF << RC_REGSHIFT_DMA_W_ERR_LLEFED)
#define RC_REGSHIFT_TARGET_LINK_SPEED	0
#define RC_REGMASK_TARGET_LINK_SPEED	(0xF << RC_REGSHIFT_TARGET_LINK_SPEED)
#define RC_REGSHIFT_TRANSMIT_MARGIN	0
#define RC_REGMASK_TRANSMIT_MARGIN	(0x7 << RC_REGSHIFT_TRANSMIT_MARGIN)
#define RC_REGSHIFT_COMPLIANCE		4
#define RC_REGMASK_COMPLIANCE		(0x1 << RC_REGSHIFT_COMPLIANCE)
#define RC_REGSHIFT_ASPM		0
#define RC_REGMASK_ASPM			(0x3 << RC_REGSHIFT_ASPM)
#define RC_REGSHIFT_ASPM_ENTER_L1	30
#define RC_REGMASK_ASPM_ENTER_L1	(0x1 << RC_REGSHIFT_ASPM_ENTER_L1)
#define RC_REGSHIFT_RETRAIN		5
#define RC_REGMASK_RETRAIN		(0x1 << RC_REGSHIFT_RETRAIN)
#define RC_REGSHIFT_MAX_PAYLOAD		5
#define RC_REGMASK_MAX_PAYLOAD		(0x7 << RC_REGSHIFT_MAX_PAYLOAD)
#define RC_REGSHIFT_MAX_READREQ		12
#define RC_REGMASK_MAX_READREQ		(0x7 << RC_REGSHIFT_MAX_READREQ)
#define RC_REGSHIFT_SECONDARY_BUS_RESET	22
#define RC_REGMASK_SECONDARY_BUS_RESET	(1 << RC_REGSHIFT_SECONDARY_BUS_RESET)
#define RC_REGSHIFT_CB			0
#define RC_REGMASK_CB			(1 << RC_REGSHIFT_CB)
#define RC_REGSHIFT_TCB			1
#define RC_REGMASK_TCB			(1 << RC_REGSHIFT_TCB)
#define RC_REGSHIFT_LLP			2
#define RC_REGMASK_LLP			(1 << RC_REGSHIFT_LLP)
#define RC_REGSHIFT_LIE			3
#define RC_REGMASK_LIE			(1 << RC_REGSHIFT_LIE)
#define RC_REGSHIFT_RIE			4
#define RC_REGMASK_RIE			(1 << RC_REGSHIFT_RIE)
#define RC_REGSHIFT_CS			5
#define RC_REGMASK_CS			(3 << RC_REGSHIFT_CS)
#define RC_REGSHIFT_DMA_STOP		31
#define RC_REGMASK_DMA_STOP		(1 << RC_REGSHIFT_DMA_STOP)
#define RC_REGSHIFT_DBI_RO_WR_EN	0
#define RC_REGMASK_DBI_RO_WR_EN		(1 << RC_REGSHIFT_DBI_RO_WR_EN)

#define XMU_PCIE_POWER_DOMAIN_NO	6
#define CSR_REGSHIFT_CORE_RST_STATUS	1
#define CSR_REGMASK_CORE_RST_STATUS	(1 << CSR_REGSHIFT_CORE_RST_STATUS)

#define CAP_REGSHIFT_D1_SUPPORT		9
#define CAP_REGMASK_D1_SUPPORT		(1 << CAP_REGSHIFT_D1_SUPPORT)
#define CAP_REGSHIFT_POWERSTATE		0
#define CAP_REGMASK_POWERSTATE		(3 << CAP_REGSHIFT_POWERSTATE)
// PCIE SPACE REGISTERS
///////////////////////////////////////////////////////////////////////////////
//  PCIe Basic Design Maps
///////////////////////////////////////////////////////////////////////////////
#define CFG_TYPE1_HDR	        0x00	//`defineCFG_TYPE1_HDR 8'h00
#define CFG_PM_CAP		0x40	//`define CFG_PM_CAP 8'h40
#define CFG_MSI_CAP		0x50	//`define CFG_PM_CAP 8'h40
#define CFG_PCIE_CAP	        0x70	//`define CFG_PCIE_CAP 8'h70
#define CFG_AER_CAP		0x100	//`define CFG_AER_CAP 8'h100
#define CFG_L1SUB_CAP		0x150	//`define CFG_L1SUB_CAP 8'h150
#define	PORTLOG_OFFSET		0x0700
#define	BARMASK_OFFSET		0x1000
#define	ATU_OFFSET		0x4000
#define PORTLOG_OFFSET_GEN3	0x600000 //Gen3 @KITT
//---------------------------------------------------------------------------
// PCI_CONFIG_HEADER
//---------------------------------------------------------------------------
#define	DBI_DEV_VENDOR_ID			(CFG_TYPE1_HDR+0x00)
#define	DBI_STATUS_COMMAND			(CFG_TYPE1_HDR+0x04)
#define	DBI_CLASS_CODE_REV_ID			(CFG_TYPE1_HDR+0x08)
#define	DBI_BIST_HDRTYPE_LAT_CACHELINESIZE	(CFG_TYPE1_HDR+0x0C)
#define	DBI_BAR0	                        (CFG_TYPE1_HDR+0x10)
#define	DBI_BAR1	                        (CFG_TYPE1_HDR+0x14)
#define	DBI_PRIM_SEC_BUS			(CFG_TYPE1_HDR+0x18)
#define	DBI_BAR_IO_LIMIT_BASE			(CFG_TYPE1_HDR+0x1C)
#define	DBI_BAR_MEM_LIMIT_BASE			(CFG_TYPE1_HDR+0x20)
#define	DBI_BAR_PREF_MEM_LIMIT_BASE		(CFG_TYPE1_HDR+0x24)
#define	DBI_BAR_PREF_BASE_UPPER			(CFG_TYPE1_HDR+0x28)
#define	DBI_BAR_PREF_LIMIT_UPPER		(CFG_TYPE1_HDR+0x2C)
#define	DBI_BAR_IO_LIMIT_BASE_UPPER		(CFG_TYPE1_HDR+0x30)
#define DBI_CAP_PTR				(CFG_TYPE1_HDR+0x34)
#define DBI_EXP_ROM_BASE			(CFG_TYPE1_HDR+0x38)
#define DBI_BRIDGE_CTRL_INT_PIN			(CFG_TYPE1_HDR+0x3C)
//---------------------------------------------------------------------------
// PCIE CAPABILITY STRUCTURE
//---------------------------------------------------------------------------
#define	PCIE_CAP_ID		(CFG_PCIE_CAP+0x0)
#define	DEV_CAP			(CFG_PCIE_CAP+0x4)
#define	DEV_STS_CTRL		(CFG_PCIE_CAP+0x8)
#define	DEV_STS			(CFG_PCIE_CAP+0xA)
#define	LNK_CAP			(CFG_PCIE_CAP+0xC)
#define	LNK_STS_CTRL		(CFG_PCIE_CAP+0x10)
#define	ROOT_CTRL_CAP		(CFG_PCIE_CAP+0x1C)
#define	ROOT_STATUS		(CFG_PCIE_CAP+0x20)
#define	DEV_CAP2		(CFG_PCIE_CAP+0x24)
#define	DEV_STS_CTRL2		(CFG_PCIE_CAP+0x28)
#define	LINK_CAP2		(CFG_PCIE_CAP+0x2C)
#define	LNK_STS_CTRL2		(CFG_PCIE_CAP+0x30)
//----------------------------------------------------------------------------
//	Advanced Error Reporting CAPABILITY STRUCTURE
//----------------------------------------------------------------------------
#define	AER_PCIE_EN_CAP	    (CFG_AER_CAP+0x0)
#define	AER_UN_ERR_ST       (CFG_AER_CAP+0x4)
#define	AER_UN_ERR_MS       (CFG_AER_CAP+0x8)
#define	AER_UN_ERR_SV       (CFG_AER_CAP+0xC)
#define	AER_CO_ERR_ST       (CFG_AER_CAP+0x10)
#define	AER_CO_ERR_MS       (CFG_AER_CAP+0x14)
#define	AER_CAP_CR          (CFG_AER_CAP+0x18)
#define	AER_HD_L_0          (CFG_AER_CAP+0x1C)
#define	AER_HD_L_4          (CFG_AER_CAP+0x20)
#define	AER_HD_L_8          (CFG_AER_CAP+0x24)
#define	AER_HD_L_12         (CFG_AER_CAP+0x28)
#define	AER_RT_ERR_CMD      (CFG_AER_CAP+0x2C)
#define	AER_RT_ERR_STS      (CFG_AER_CAP+0x30)
#define	AER_ERR_SRC_ID      (CFG_AER_CAP+0x34)
#define AER_TLP_PRE_LOG     (CFG_AER_CAP+0x38)
//---------------------------------------------------------------------------
//PORT_LOGIC
//---------------------------------------------------------------------------
#define	PL_LAT_REL_TIM			(PORTLOG_OFFSET+0x0)
#define	PL_OT_MSG_R			(PORTLOG_OFFSET+0x4)
#define	PL_PT_LNK_R			(PORTLOG_OFFSET+0x8)
#define	PL_ACK_FREQ_R			(PORTLOG_OFFSET+0xC)
#define	PL_PT_LNK_CTRL_R		(PORTLOG_OFFSET+0x10)
#define	PL_LN_SKW_R			(PORTLOG_OFFSET+0x14)
#define	PL_SYMB_N_R			(PORTLOG_OFFSET+0x18)
#define	PL_SYMB_T_R			(PORTLOG_OFFSET+0x1C)
#define	PL_FL_MSK_R2			(PORTLOG_OFFSET+0x20)
#define	PL_DB_R0			(PORTLOG_OFFSET+0x28)
#define	PL_DB_R1			(PORTLOG_OFFSET+0x2C)
#define	PL_TR_P_STS_R			(PORTLOG_OFFSET+0x30)
#define	PL_TR_NP_STS_R			(PORTLOG_OFFSET+0x34)
#define	PL_TR_C_STS_R			(PORTLOG_OFFSET+0x38)
#define	PL_Q_STS_R			(PORTLOG_OFFSET+0x3C)
#define	PL_VC_TR_A_R1			(PORTLOG_OFFSET+0x40)
#define	PL_VC_TR_A_R2			(PORTLOG_OFFSET+0x44)
#define	PL_VC0_PR_Q_C			(PORTLOG_OFFSET+0x48)
#define	PL_VC0_NPR_Q_C			(PORTLOG_OFFSET+0x4C)
#define	PL_VC0_CR_Q_C			(PORTLOG_OFFSET+0x50)
#define	PL_GEN2_CTRL			(PORTLOG_OFFSET+0x10C)
#define	PL_PHY_STS_R			(PORTLOG_OFFSET+0x110)
#define	PL_PHY_CTRL_R			(PORTLOG_OFFSET+0x114)
#define	PL_MSI_CNT_ADDR			(PORTLOG_OFFSET+0x120)
#define	PL_MSI_CNT_ADDR_UPPER		(PORTLOG_OFFSET+0x124)
#define	PL_MSI_CNT_INT_EN(num)		(PORTLOG_OFFSET+0x128+(num*0xc))
#define	PL_MSI_CNT_INT_MSK(num)		(PORTLOG_OFFSET+0x12C+(num*0xc))
#define	PL_MSI_CNT_INT_STS(num)		(PORTLOG_OFFSET+0x130+(num*0xc))
#define	PL_MSI_CNT_GPIO			(PORTLOG_OFFSET+0x188)
#define	PL_PIPE_LOOPBACK_CTRL		(PORTLOG_OFFSET+0x1B8)
#define	PL_MISC_CTRL			0x8BC
#define	PL_AMBA_ERR_RESP		(PORTLOG_OFFSET+0x1D0)
#define	PL_AMBA_LINK_TOUT		(PORTLOG_OFFSET+0x1D4)
#define	PL_AMBA_ORDRMGR			(PORTLOG_OFFSET+0x1DC)
#define	PL_COHERENCY_CTRL1		(PORTLOG_OFFSET+0x1E0)
#define	PL_COHERENCY_CTRL2		(PORTLOG_OFFSET+0x1E4)
#define	PL_COHERENCY_CTRL3		(PORTLOG_OFFSET+0x1E8)
#define	PL_LTR_LATENCY			(PORTLOG_OFFSET+0x430)
#define	PL_AUX_CLK_FREQ			(PORTLOG_OFFSET+0x440)
#define	PL_L1_SUBSTATES			(PORTLOG_OFFSET+0x444)

// ATU
#define PCIE_ATU_BASE			0x600000
// for outbound
#define PCIE_ATU_CR1_OUT0		(PCIE_ATU_BASE + 0x0)
#define PCIE_ATU_CR2_OUT0		(PCIE_ATU_BASE + 0x4)
#define PCIE_ATU_LOWER_BASE_OUT0	(PCIE_ATU_BASE + 0x8)
#define PCIE_ATU_UPPER_BASE_OUT0	(PCIE_ATU_BASE + 0xC)
#define PCIE_ATU_LIMIT_OUT0		(PCIE_ATU_BASE + 0x10)
#define PCIE_ATU_LOWER_TARGET_OUT0	(PCIE_ATU_BASE + 0x14)
#define PCIE_ATU_UPPER_TARGET_OUT0	(PCIE_ATU_BASE + 0x18)
// for inbound
#define PCIE_ATU_CR1_IN0		(PCIE_ATU_BASE + 0x100)
#define PCIE_ATU_CR2_IN0		(PCIE_ATU_BASE + 0x104)
#define PCIE_ATU_LOWER_BASE_IN0		(PCIE_ATU_BASE + 0x108)
#define PCIE_ATU_UPPER_BASE_IN0		(PCIE_ATU_BASE + 0x10C)
#define PCIE_ATU_LIMIT_IN0		(PCIE_ATU_BASE + 0x110)
#define PCIE_ATU_LOWER_TARGET_IN0	(PCIE_ATU_BASE + 0x114)
#define PCIE_ATU_UPPER_TARGET_IN0	(PCIE_ATU_BASE + 0x118)
//DMA
#define PCIE_DMA_BASE		(0x0)

#define PCIE_DMA_DBOFF_STOP	(0x80000000)

#define PCIE_DMA_WR_INT_STS	(PCIE_DMA_BASE + 0x2000)
#define PCIE_DMA_WR_INT_MASK	(PCIE_DMA_BASE + 0x2004)
#define PCIE_DMA_WR_INT_CLEAR	(PCIE_DMA_BASE + 0x2008)
#define PCIE_DMA_WR_ERR_STS	(PCIE_DMA_BASE + 0x200C)
#define PCIE_DMA_RD_INT_STS	(PCIE_DMA_BASE + 0x2010)
#define PCIE_DMA_RD_INT_MASK	(PCIE_DMA_BASE + 0x2014)
#define PCIE_DMA_RD_INT_CLEAR	(PCIE_DMA_BASE + 0x2018)

#define PCIE_DMA_CH_CTRL1	(PCIE_DMA_BASE + 0x3000)
#define PCIE_DMA_CH_CTRL2	(PCIE_DMA_BASE + 0x3100)

#define PCIE_DMA_CH_OFFSET	(0x1000)

#define PCIE_DMA_STS_2CH_MASK	(0x3)

#define PCIE_DMA_STS_IRQ0_MASK	(0x30003)
#define PCIE_DMA_STS_IRQ1_MASK	(0xC000C)
#define PCIE_DMA_STS_IRQ2_MASK	(0x300030)
#define PCIE_DMA_STS_IRQ3_MASK	(0xC000C0)

#define PCIE_DMA_CAP_CTRL	(PCIE_DMA_BASE + 0x8)
#define PCIE_DMA_WR_EN		(PCIE_DMA_BASE + 0xC)
#define PCIE_DMA_WR_DBOFF	(PCIE_DMA_BASE + 0x10)
#define DBOFF_WR_OFF		(0x1 << 31)
#define DBOFF_DMA_CH_MASK	(0x3 << 0)

#define PCIE_DMA_RD_EN		(PCIE_DMA_BASE + 0x2C)
#define PCIE_DMA_RD_DBOFF	(PCIE_DMA_BASE + 0x30)


#define CH_CTRL1_CB		(0x1 << 0)
#define CH_CTRL1_TCB		(0x1 << 1)
#define CH_CTRL1_LLP		(0x1 << 2)
#define CH_CTRL1_LIE		(0x1 << 3)
#define CH_CTRL1_RIE		(0x1 << 4)
#define CH_CTRL1_CS		(0x3 << 5)
#define CH_CTRL1_DMA_RESERVED0	(0x1 << 7)
#define CH_CTRL1_CCS		(0x1 << 8)
#define CH_CTRL1_LLE		(0x1 << 9)
#define CH_CTRL1_DMA_RESERVED1	(0x3 << 10)
#define CH_CTRL1_DMA_FUNC_NUM	(0x1F << 12)
#define CH_CTRL1_DMA_RESERVED2	(0x3F << 17)
#define CH_CTRL1_DMA_NS_DST	(0x1 << 23)
#define CH_CTRL1_DMA_NS_SRC	(0x1 << 24)
#define CH_CTRL1_DMA_RO		(0x1 << 25)
#define CH_CTRL1_DMA_RESERVED5	(0x1 << 26)
#define CH_CTRL1_DMA_TC		(0x7 << 27)
#define CH_CTRL1_DMA_AT		(0x3 << 30)

#define PCIE_DMA_WR_XFER_SIZE	(PCIE_DMA_CH_CTRL1 + 0x08)
#define PCIE_DMA_WR_SARL	(PCIE_DMA_CH_CTRL1 + 0x0C)
#define PCIE_DMA_WR_SARH	(PCIE_DMA_CH_CTRL1 + 0x10)
#define PCIE_DMA_WR_DARL	(PCIE_DMA_CH_CTRL1 + 0x14)
#define PCIE_DMA_WR_DARH	(PCIE_DMA_CH_CTRL1 + 0x18)

#define CH_CTRL2_CB		(0x1 << 0)
#define CH_CTRL2_TCB		(0x1 << 1)
#define CH_CTRL2_LLP		(0x1 << 2)
#define CH_CTRL2_LIE		(0x1 << 3)
#define CH_CTRL2_RIE		(0x1 << 4)
#define CH_CTRL2_CS		(0x3 << 5)
#define CH_CTRL2_DMA_RESERVED0	(0x1 << 7)
#define CH_CTRL2_CCS		(0x1 << 8)
#define CH_CTRL2_LLE		(0x1 << 9)
#define CH_CTRL2_DMA_RESERVED1	(0x3 << 10)
#define CH_CTRL2_DMA_FUNC_NUM	(0x1F << 12)
#define CH_CTRL2_DMA_RESERVED2	(0x3F << 17)
#define CH_CTRL2_DMA_NS_DST	(0x1 << 23)
#define CH_CTRL2_DMA_NS_SRC	(0x1 << 24)
#define CH_CTRL2_DMA_RO		(0x1 << 25)
#define CH_CTRL2_DMA_RESERVED5	(0x1 << 26)
#define CH_CTRL2_DMA_TC		(0x7 << 27)
#define CH_CTRL2_DMA_AT		(0x3 << 30)

#define PCIE_DMA_RD_XFER_SIZE	(PCIE_DMA_CH_CTRL2 + 0x08)
#define PCIE_DMA_RD_SARL	(PCIE_DMA_CH_CTRL2 + 0x0C)
#define PCIE_DMA_RD_SARH	(PCIE_DMA_CH_CTRL2 + 0x10)
#define PCIE_DMA_RD_DARL	(PCIE_DMA_CH_CTRL2 + 0x14)
#define PCIE_DMA_RD_DARH	(PCIE_DMA_CH_CTRL2 + 0x18)

//Doorbell
#define PCIE_DOORBELL_CH_OFFSET		(0x10000)
#define PCIE_DOORBELL_BASE_ADDRESS	(0x17B30000)
#define PCIE_DOORBELL0_STATUS		(0x0)
#define PCIE_DOORBELL0_ENABLE		(0x4)
#define PCIE_DOORBELL0_CLEAR		(0x8)

#define PCIE_DOORBELL_OFFSET		(0x1000)

#define PCIE_EP_INTADMASK	0xF
#define PCIE_EP_IRQMASK		0xFFFFFFFF
#define PCIE_EP_IRQMASK1	PCIE_EP_IRQMASK
#define PCIE_EP_IRQMASK2	(PCIE_EP_IRQMASK & \
					(~(RTLH_RFC_UPD_0_1 | \
						RTLH_RFC_UPD_0_0 | \
						Q_NOT_EMPTY) \
					) \
				)
#define PCIE_EP_IRQMASK3	0xFFFF0000

//SR-IOV
#define PCIE_SRIOV_CAP_POS	(0x200)

#define PCIE_SHADOW_REG_POS	(0x200000)

enum exynos_pcie_ep_state {
	STATE_LINK_DOWN = 0,
	STATE_LINK_UP_TRY,
	STATE_LINK_DOWN_TRY,
	STATE_LINK_UP,
	STATE_LINK_DOWN_ABNORMAL,
};

enum exynos_pcie_ep_work_state {
	STATE_DMA_IDLE = 0,
	STATE_DMA_BUSY,
};

enum __ltssm_states {
	S_DETECT_QUIET		= 0x00,
	S_DETECT_ACT		= 0x01,
	S_POLL_ACTIVE		= 0x02,
	S_POLL_COMPLIANCE	= 0x03,
	S_POLL_CONFIG		= 0x04,
	S_PRE_DETECT_QUIET	= 0x05,
	S_DETECT_WAIT		= 0x06,
	S_CFG_LINKWD_START	= 0x07,
	S_CFG_LINKWD_ACEPT	= 0x08,
	S_CFG_LANENUM_WAIT	= 0x09,
	S_CFG_LANENUM_ACEPT	= 0x0A,
	S_CFG_COMPLETE		= 0x0B,
	S_CFG_IDLE		= 0x0C,
	S_RCVRY_LOCK		= 0x0D,
	S_RCVRY_SPEED		= 0x0E,
	S_RCVRY_RCVRCFG		= 0x0F,
	S_RCVRY_IDLE		= 0x10,
	S_L0			= 0x11,
	S_L0S			= 0x12,
	S_L123_SEND_EIDLE	= 0x13,
	S_L1_IDLE		= 0x14,
	S_L2_IDLE		= 0x15,
	S_L2_WAKE		= 0x16,
	S_DISABLED_ENTRY	= 0x17,
	S_DISABLED_IDLE		= 0x18,
	S_DISABLED		= 0x19,
	S_LPBK_ENTRY		= 0x1A,
	S_LPBK_ACTIVE		= 0x1B,
	S_LPBK_EXIT		= 0x1C,
	S_LPBK_EXIT_TIMEOUT	= 0x1D,
	S_HOT_RESET_ENTRY	= 0x1E,
	S_HOT_RESET		= 0x1F,
	GEN3_LINKUP		= 0x91,
};

enum exynos_pcie_ep_work_cmd {
	WORK_MEM_WR = 0,
	WORK_MEM_RD,
	WORK_MSI,
	WORK_DOORBELL,
	WORK_MEM_WR_RD,
};

enum exynos_pcie_ep_work_exception {
	WORK_DEFAULT = 0,
	WORK_LINK_RESET,
};

struct exynos_ep_pcie_phyinfo {
	void __iomem		*phy_base;
	void __iomem		*sysreg_base;
	void __iomem		*elbi_base;
	void __iomem		*gen_subsys_base;
	void __iomem		*e32_phy_base;
	void __iomem		*elbi_base_other;
	void __iomem		*dbi_base;
	void __iomem		*elbi_cmn_base;
	void __iomem		*cmu_base;
	void __iomem		*doorbell_base;
	int ch_num;
	/* 0: bifurcation, 1:aggregation */
	bool use_bifurcation;
	int lanes_num;
	bool use_sris;
	bool cold_rst;

	u32 linkA_status;
	u32 linkB_status;
};

struct pcie_ep_phyops {
	void (*phy_all_pwrdn)(struct exynos_ep_pcie_phyinfo *phyinfo);
	void (*phy_all_pwrdn_clear)(struct exynos_ep_pcie_phyinfo *phyinfo);
	void (*phy_config)(struct exynos_ep_pcie_phyinfo *phyinfo, int *result);
	void (*phy_other_set)(struct exynos_ep_pcie_phyinfo *phyinfo);
};

struct exynos_pcie_ep_clks {
	struct clk	*pcie_clks[10];
	struct clk	*phy_clks[3];
};

struct exynos_ep_pcie {
	struct dw_pcie			*pci;
	struct exynos_ep_pcie_phyinfo	*ep_phyinfo;
	void __iomem			*elbi_base;
	void __iomem			*gen_subsys_base;
	void __iomem			*e32_phy_base;
	void __iomem			*elbi_base_other;
	void __iomem			*phy_base;
	void __iomem			*sysreg_base;
	void __iomem			*dbi_base;
	void __iomem			*ia_base;
	void __iomem			*atu_base;
	void __iomem			*elbi_cmn_base;
	void __iomem			*sysreg_ctrl_base;
	void __iomem			*doorbell_base;
	void __iomem			*cmu_base;
	void __iomem			*dma_base;

	struct pcie_ep_phyops		phy_ops;
	struct platform_device	*pdev;
	struct workqueue_struct		*pcie_ep_wq;
	struct workqueue_struct		*pcie_ep_wq_exception;
	struct workqueue_struct		*pcie_ep_wq_dislink;
	struct exynos_pcie_ep_clks	clks;

	struct delayed_work		irq1_evt_work;
	struct delayed_work		test_work;
	struct delayed_work		exception_work;
	struct delayed_work		dislink_work;
	enum exynos_pcie_ep_work_cmd	work_cmd;
	enum exynos_pcie_ep_work_exception	work_exception;

	enum exynos_pcie_ep_state	state;
	enum exynos_pcie_ep_work_state	work_state;

	spinlock_t			dma_lock;
	int				dma_ch_num;
	void				*dma_wr_buf;
	void				*dma_rd_buf;
	dma_addr_t			rd_dma;
	dma_addr_t			wr_dma;
	u32			checksum;
	unsigned int			size;
	int				cnt;

	int				dma_num;
	int				msi_data;

	int				sysreg_offset;

	struct completion		wdma0;
	struct completion		rdma0;

	struct completion		wdma1;
	struct completion		wdma2;
	struct completion		wdma3;

	struct completion		rdma1;
	struct completion		rdma2;
	struct completion		rdma3;

	u32				irq0_status;
	u32				irq1_status;
	u32				irq2_status;
	u32				irq3_status;
	u32				doorbell;

	int			irq;
	int			db_irq[MAX_VF_NUM];
	int			dma_irq[MAX_VF_NUM];

	u32				pmu_phy_offset;
	/* 1: bifurcation, 0:aggregation */
	bool				use_bifurcation;
	bool				use_sris;
	int				lanes_num;

	int				ch_num;
	int				pcie_clk_num;
	int				phy_clk_num;
	int				max_link_speed;
	int				linkup_ready;
	struct regmap			*pmureg;
	struct regmap			*sysreg;

	struct pinctrl			*pcie_ep_pinctrl;
	struct pinctrl_state		*pin_perst;
	struct pinctrl_state		*pin_clkreq;

	int				perst_gpio;

	int             lane_status;

	struct pci_epf_bar *epf_bar;

	unsigned int			s2mpu_base;
};
