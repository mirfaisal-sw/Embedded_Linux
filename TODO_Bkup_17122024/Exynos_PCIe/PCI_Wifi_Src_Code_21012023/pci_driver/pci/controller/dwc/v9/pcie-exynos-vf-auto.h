/* SPDX-License-Identifier: GPL-2.0 */
/**
 * pcie-exynos-vf-auto.h - Exynos PCIe VF driver
 *
 * Copyright (c) 2013-2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Authors: Sanghoon Bae <sh86.bae@samsung.com>
 *	    Jiheon Oh <jiheon.oh@samsung.com>
 *
 */


#include <nk/nkern.h>

#define USE_RESERVED_MEMORY	0

#define VF_DB_OFFSET	0x1000
#define TOTAL_VF_NUM	4
#define PCIE_VF_IRQMASK		0xFFFFFFFF

#define VF_PMEM_DEF_SIZE 1024
#define VF_SHARED_MEM_SIZE (VF_PMEM_DEF_SIZE)

#define VF_PMEM_ID_BASE	0
#define VF_PMEM_ID_SHARED	1

#define VF_MSI_CTRL_INTR0_ENABLE	0x0
#define VF_MSI_CTRL_INTR0_MASK	0x4
#define VF_MSI_CTRL_INTR0_STATUS	0x8
#define VF_MSI_CTRL_INT_OFFSET	0x2000

typedef unsigned int vpci_size_t;

typedef struct vpci_pmem_t {
	NkPhAddr shared_paddr;
} vpci_pmem_t;

typedef struct pmem_share {
	NkPhAddr paddr;
	unsigned long vaddr;
} pmem_share_t;

typedef struct vpci_shared {
	struct pmem_share pmem;
} vpci_shared_t;

typedef struct vpci_end_t {
	NkOsId id;
	NkXIrq xirq;
	volatile int *state;
} vpci_end_t;

typedef struct vpci_link_t {
	NkPhAddr plink;
	NkDevVlink *vlink;
	vpci_pmem_t *pmem;
	vpci_end_t my;
	vpci_end_t peer;

	NkXIrqId xid;
	bool server;
	int	id;

	wait_queue_head_t wait;

	int status;

	bool	running;

	vpci_shared_t shared;

	spinlock_t lock;
	unsigned long flags;
} vpci_link_t;

struct exynos_pcie_vf {
	struct dw_pcie			*pci;
	struct platform_device *pdev;

	void __iomem			*outbound_base;
	void __iomem			*dbi_base;
	void __iomem			*rmem_base;

	int irq;
	int msi_data;

	bool backend;

	u32 vf_num;
	u32 doorbell;
	u32 msi_status;

	struct resource	*ob;
	struct resource	*dbi;

	bool use_msi;

	NkXIrqId scid;
	bool inited;
	u32 vlink_cnt;
	u32 vlink_num;
	vpci_link_t *vpci[TOTAL_VF_NUM];
};

struct exynos_pcie_vf_rmem {
	const char	*name;
	phys_addr_t	base;
	phys_addr_t	size;
};

enum exynos_pcie_vf_status {
	PF_LINKDOWN = 0,
	PF_LINKUP,
	PF_ABNORMALDOWN,
	PF_RESET
};

void exynos_vf_request_vlink_send(int value);


