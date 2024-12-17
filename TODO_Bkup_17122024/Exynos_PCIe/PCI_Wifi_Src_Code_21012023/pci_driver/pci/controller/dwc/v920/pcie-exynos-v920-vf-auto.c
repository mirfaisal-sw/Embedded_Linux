// SPDX-License-Identifier: GPL-2.0
/**
 * pcie-exynos-vf-auto.c - Exynos PCIe VF driver
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

#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>

#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/freezer.h>
#include <linux/sysfs.h>
#include <linux/device.h>

#include <linux/spinlock.h>

#include "pcie-designware.h"
#include "pcie-exynos-v920-vf-auto.h"

struct exynos_pcie_vf g_pcie_vf;
#if USE_RESERVED_MEMORY
struct exynos_pcie_vf_rmem g_pcie_vf_rmem;
#endif

////////////////////////
//   PCIe PV driver   //
////////////////////////
static DEFINE_MUTEX(_mutex);
static inline bool exynos_v920_vpci_is_server(vpci_link_t *);
static int exynos_v920_vpci_link_init(struct exynos_pcie_vf *pcie_vf,
	NkDevVlink *vlink, NkPhAddr plink, bool server);
static int exynos_v920_vpci_init(struct exynos_pcie_vf *pcie_vf);
static void exynos_v920_vpci_pmem_show(struct exynos_pcie_vf *pcie_vf);
static void exynos_v920_pcie_vf_msi_clear_irq(struct exynos_pcie_vf *pcie_vf);
static void exynos_v920_pcie_vf_msi_set_irq(struct exynos_pcie_vf *pcie_vf);

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Init spinlock"
 * @logic "Init spin lock"
 * @params
 * @param{in/out, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @noret
 */
static inline void exynos_v920_vpci_lock_init(vpci_link_t *link)
{
	spin_lock_init(&link->lock);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Lock spinlock"
 * @logic "Lock spinlock"
 * @params
 * @param{in/out, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @noret
 */
static inline void exynos_v920_vpci_lock(vpci_link_t *link)
{
	unsigned long flags;

	spin_lock_irqsave(&link->lock, flags);
	link->flags = flags;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Unlock spinlock"
 * @logic "Unlock spinlock"
 * @params
 * @param{in/out, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @noret
 */
static inline void exynos_v920_vpci_unlock(vpci_link_t *link)
{
	unsigned long flags = link->flags;

	spin_unlock_irqrestore(&link->lock, flags);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Post xirq"
 * @logic "Post xirq"
 * @params
 * @param{in/out, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_v920_vpci_xirq_post(vpci_link_t *link)
{
	exynos_v920_vpci_lock(link);
	dev_info(NULL, "[%s] [peer.xirq]:%d [peer.id]:%d\n",
		__func__, link->peer.xirq, link->peer.id);
	nkops.nk_xirq_trigger(link->peer.xirq, link->peer.id);
	exynos_v920_vpci_unlock(link);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Post sysconf irq"
 * @logic "Post sysconf irq"
 * @params
 * @param{in, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_v920_vpci_sysconf_post(vpci_link_t *link)
{
	dev_info(NULL, "[%s] [peer.id]:%d\n", __func__,  link->peer.id);
	nkops.nk_xirq_trigger(NK_XIRQ_SYSCONF, link->peer.id);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Reset pmem"
 * @logic "Do nothing"
 * @params
 * @param{in/out, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_v920_vpci_pmem_reset(vpci_link_t *link)
{
	(void)link;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Init pmem"
 * @logic "Do nothing"
 * @params
 * @param{in, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_v920_vpci_pmem_init(vpci_link_t *link)
{

}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Check vlink is online"
 * @logic "Get my state and peer state"
 * @params
 * @param{in, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @retval{-, bool, 0, 0/1, not 0/1}
 */
bool exynos_v920_vpci_link_is_online(vpci_link_t *link)
{
	volatile int *my_state;
	int peer_state;

	my_state = link->my.state;
	peer_state = *link->peer.state;

	return (*my_state == NK_DEV_VLINK_ON) &&
		(peer_state == NK_DEV_VLINK_ON);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Perform handshake for vlink"
 * @logic "Check my state and peer state\n
 *	Change my state and post sysconf interrupt"
 * @params
 * @param{in/out, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @retval{-, bool, 0, 0/1, not 0/1}
 */
static bool exynos_v920_vpci_handshake(vpci_link_t *link)
{
	volatile int *my_state;
	int peer_state;

	my_state = link->my.state;
	peer_state = *link->peer.state;

	switch (*my_state) {
	case NK_DEV_VLINK_OFF:
		if (peer_state != NK_DEV_VLINK_ON) {
			exynos_v920_vpci_pmem_reset(link);
			*my_state = NK_DEV_VLINK_RESET;
			exynos_v920_vpci_sysconf_post(link);
		}
		break;
	case NK_DEV_VLINK_RESET:
		if (peer_state != NK_DEV_VLINK_OFF) {
			*my_state = NK_DEV_VLINK_ON;
			exynos_v920_vpci_pmem_init(link);
			exynos_v920_vpci_sysconf_post(link);
		}
		break;
	case NK_DEV_VLINK_ON:
		if (peer_state == NK_DEV_VLINK_OFF) {
			*my_state = NK_DEV_VLINK_RESET;
			exynos_v920_vpci_sysconf_post(link);
		}
		break;
	default:
		break;
	}
	return (*my_state == NK_DEV_VLINK_ON &&
		(peer_state == NK_DEV_VLINK_ON));
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Handle vlink interrupt"
 * @logic "If VF is in pf-domain, set link status to each VF's pmem\n
 *	If VF is not in pf-domain, read pmem"
 * @params
 * @param{in/out, cookie, void *, not NULL}
 * @param{in, xirq, NkXIrq, >0}
 * @endparam
 * @noret
 */
static void exynos_v920_vpci_xirq_handler(void *cookie, NkXIrq xirq)
{
	struct exynos_pcie_vf *pcie_vf = cookie;
	struct device *dev = pcie_vf->pci->dev;

	exynos_v920_vpci_lock(pcie_vf->vpci[pcie_vf->vlink_num]);
	dev_info(dev, "[%s] XIRQ[%d] is received\n", __func__, xirq);
	if (pcie_vf->vpci[0]->server) {
		if (pcie_vf->vpci[0]->my.xirq == xirq)
			memcpy(&pcie_vf->vpci[0]->status,
				(int *)pcie_vf->vpci[0]->shared.pmem.vaddr,
				sizeof(int));
		else if (pcie_vf->vpci[1]->my.xirq == xirq)
			memcpy(&pcie_vf->vpci[1]->status,
				(int *)pcie_vf->vpci[1]->shared.pmem.vaddr,
				sizeof(int));
		else
			dev_warn(dev, "[%s] Undefined xirq[%d]\n",
					__func__, xirq);
	} else {
		// READ Pmem IF XIRQ received in VF
		exynos_v920_vpci_pmem_show(pcie_vf);
	}
	exynos_v920_vpci_unlock(pcie_vf->vpci[pcie_vf->vlink_num]);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Start vlink handshake"
 * @logic "Perform vlink handshake"
 * @params
 * @param{in/out, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @retval{ret, int, 0, 0/1, not 0/1}
 */
static int exynos_v920_vpci_link_ready(vpci_link_t *link)
{
	int ret = 0;

	exynos_v920_vpci_lock(link);
	ret = exynos_v920_vpci_handshake(link);
	exynos_v920_vpci_unlock(link);
	return ret;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Handle sysconf interrupt"
 * @logic "Check vm location and link status\n
 *	Start vlink handshake"
 * @params
 * @param{in/out, cookie, void *, not NULL}
 * @param{in, xirq, NkXIrq, >0}
 * @endparam
 * @noret
 */
static void exynos_v920_vpci_sysconf_handler(void *cookie, NkXIrq xirq)
{
	struct exynos_pcie_vf *pcie_vf = cookie;
	int i;

	if (pcie_vf->vpci[0]->running) {
		if (*pcie_vf->vpci[0]->peer.state != NK_DEV_VLINK_ON ||
			*pcie_vf->vpci[0]->my.state != NK_DEV_VLINK_ON)
			exynos_v920_vpci_link_ready(pcie_vf->vpci[0]);
	}

	if (pcie_vf->backend) {
		for (i = 1; i < pcie_vf->vlink_cnt; i++) {
			if (pcie_vf->vpci[i]->running) {
				if (*pcie_vf->vpci[i]->peer.state !=
				NK_DEV_VLINK_ON || *pcie_vf->vpci[i]->my.state
				!= NK_DEV_VLINK_ON)
					exynos_v920_vpci_link_ready
						(pcie_vf->vpci[i]);
			}
		}
	}
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "To notify vlink is server"
 * @logic "Notify vlink is server"
 * @params
 * @param{out, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @retval{-, bool, 0, 0/1, not 0/1}
 */
static inline bool exynos_v920_vpci_is_server(vpci_link_t *link)
{
	return link->server;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "To notify vlink is client"
 * @logic "Notify vlink is client"
 * @params
 * @param{out, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @retval{-, bool, 0, 0/1, not 0/1}
 */
static inline bool exynos_v920_vpci_is_client(vpci_link_t *link)
{
	return !link->server;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Get my xirq id"
 * @logic "Get my xirq id"
 * @params
 * @param{in, pcie_vf, struct ::exynos_pcie_vf *, not NULL}
 * @param{in, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @retval{ret, int, 0, 0~4, not 0~4}
 */
static int exynos_v920_vpci_my_xirq_id(struct exynos_pcie_vf *pcie_vf,
					vpci_link_t *link)
{
	int id = 0;
	int ret = 0;

	if (pcie_vf->backend)
		id = pcie_vf->vlink_num + 1;

	ret = (exynos_v920_vpci_is_server(link) ? id : 0);
	return ret;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Get peer xirq id"
 * @logic "Get peer xirq id"
 * @params
 * @param{in, pcie_vf, struct ::exynos_pcie_vf *, not NULL}
 * @param{in, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @retval{ret, int, 0, 0~4, not 0~4}
 */
static int exynos_v920_vpci_peer_xirq_id(struct exynos_pcie_vf *pcie_vf,
					vpci_link_t *link)
{
	int id = 0;
	int ret = 0;

	if (pcie_vf->backend)
		id = pcie_vf->vlink_num + 1;
	else
		id = pcie_vf->vf_num;

	ret = (exynos_v920_vpci_is_server(link) ? 0 : id);
	return  ret;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Allocate xirq"
 * @logic "Allocate xirq"
 * @params
 * @param{in, pcie_vf, struct ::exynos_pcie_vf *, not NULL}
 * @param{in/out, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @retval{-, bool, 0, 0/1, not 0/1}
 */
static bool exynos_v920_vpci_pxirq_alloc(struct exynos_pcie_vf *pcie_vf,
					vpci_link_t *link)
{
	NkPhAddr plink = link->plink;
	int mid = exynos_v920_vpci_my_xirq_id(pcie_vf, link);
	int pid = exynos_v920_vpci_peer_xirq_id(pcie_vf, link);

	link->my.xirq = nkops.nk_pxirq_alloc(plink, mid, link->my.id, 1);
	link->peer.xirq = nkops.nk_pxirq_alloc(plink, pid, link->peer.id, 1);

	return (link->my.xirq && link->peer.xirq);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Get pmem info size"
 * @logic "Get pmem info size"
 * @noparam
 * @retval{size, vpci_size_t, 1024, 1024, not 1024}
 */
static vpci_size_t pmem_info_size_v920(void)
{
	vpci_size_t size = VF_PMEM_DEF_SIZE;
	return size;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Get pmem size"
 * @logic "Get pmem size"
 * @noparam
 * @retval{-, vpci_size_t, 1024, 1024, not 1024}
 */
static vpci_size_t exynos_v920_vpci_pmem_size(void)
{
	vpci_size_t msize = pmem_info_size_v920();
	vpci_size_t psize = pmem_info_size_v920();

	return (msize > psize ? msize : psize);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Allocate pmem"
 * @logic "Get pmem size\n
 *	Allocate pmem"
 * @params
 * @param{in/out, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @retval{-, bool, 0, 0/1, not 0/1}
 */
static bool exynos_v920_vpci_pmem_alloc(vpci_link_t *link)
{
	vpci_size_t size = exynos_v920_vpci_pmem_size();
	NkPhAddr paddr = nkops.nk_pmem_alloc(link->plink, 0, size);

	if (!paddr) {
		dev_warn(NULL, "cannot alloc %d bytes of pmem\n", size);
		return 0;
	}
	dev_info(NULL, "allocate link->pmem:%p\n", (void *)paddr);
	link->pmem = (vpci_pmem_t *) nkops.nk_mem_map(paddr, size);
	return 1;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Allocate shared memory"
 * @logic "Allocate shared memory"
 * @params
 * @param{in/out, link, struct ::vpci_link_t *, not NULL}
 * @endparam
 * @retval{-, bool, 0, 0/1, not 0/1}
 */
static bool exynos_v920_vpci_shared_mem_alloc(vpci_link_t *link)
{
	NkPhAddr paddr = nkops.nk_pmem_alloc(link->plink, VF_PMEM_ID_SHARED,
			VF_SHARED_MEM_SIZE);
	if (!paddr) {
		dev_warn(NULL, "cannot alloc %d bytes of pmem\n",
				VF_SHARED_MEM_SIZE);
		return 0;
	}
	link->shared.pmem.paddr = paddr;
	link->shared.pmem.vaddr = (unsigned long)nkops.nk_mem_map(
							link->shared.pmem.paddr,
							VF_SHARED_MEM_SIZE);
	return 1;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Initialize vlink"
 * @logic "Update vlink data\n
 *	Allocate pmem\n
 *	Attach sysconf handler\n
 *	Allocate xirq\n
 *	Allocate shared memory\n
 *	Attach xirq\n"
 * @params
 * @param{in/out, pcie_vf, struct ::exynos_pcie_vf *, not NULL}
 * @param{in, vlink, NkDevVlink *, not NULL}
 * @param{in, plink, NkPhAddr, 0~0xffffffff}
 * @param{in, server, bool, 0~1}
 * @endparam
 * @retval{ret, int, 0, -ERANGE~0, <0}
 */
static int exynos_v920_vpci_link_init(struct exynos_pcie_vf *pcie_vf,
	NkDevVlink *vlink, NkPhAddr plink, bool server)
{
	struct device *dev = pcie_vf->pci->dev;
	vpci_link_t *vpci_link;

	vpci_link = (vpci_link_t *)kzalloc(sizeof(vpci_link_t), GFP_KERNEL);
	pcie_vf->vpci[pcie_vf->vlink_cnt] = vpci_link;
	if (!vpci_link) {
		dev_err(dev, "vpci_link ERROR");
		return -ENOMEM;
	}

	exynos_vpci_lock_init(vpci_link);

	vpci_link->vlink = vlink;
	vpci_link->plink = plink;
	vpci_link->my.id = nkops.nk_id_get();
	vpci_link->server = server;

	vpci_link->status = PF_LINKDOWN;

	if (!server) {
		pcie_vf->vpci[0]->id = 0;
		pcie_vf->vlink_num = 0;
	} else {
		pcie_vf->vpci[pcie_vf->vlink_cnt]->id = pcie_vf->vlink_cnt + 1;
		pcie_vf->vlink_num = pcie_vf->vlink_cnt;
	}

	dev_info(dev, "my.id: %d\n", vpci_link->my.id);

	init_waitqueue_head(&vpci_link->wait);

	if (exynos_v920_vpci_is_server(vpci_link)) {
		vpci_link->my.state = &vlink->s_state;
		vpci_link->peer.id = vlink->c_id;
		vpci_link->peer.state = &vlink->c_state;
	} else {
		vpci_link->my.state = &vlink->c_state;
		vpci_link->peer.id = vlink->s_id;
		vpci_link->peer.state = &vlink->s_state;
	}

	if (!exynos_v920_vpci_pmem_alloc(vpci_link)) {
		dev_err(dev, "exynos_v920_vpci_pmem_alloc ERROR\n");
		return -EINVAL;
	}


	pcie_vf->scid = nkops.nk_xirq_attach(NK_XIRQ_SYSCONF,
			exynos_v920_vpci_sysconf_handler, pcie_vf);
	if (!pcie_vf->scid) {
		mutex_unlock(&_mutex);
		dev_err(dev, "unable to attach a sysconf handler\n");
		return -ENOMEM;
	}

	if (!exynos_v920_vpci_pxirq_alloc(pcie_vf, vpci_link)) {
		dev_err(dev, "exynos_v920_vpci_pxirq_alloc ERROR\n");
		return -EINVAL;
	}

	if (!exynos_v920_vpci_shared_mem_alloc(vpci_link)) {
		dev_err(dev, "exynos_v920_vpci_shared_mem_alloc ERROR\n");
		return -EINVAL;
	}

	vpci_link->xid = nkops.nk_xirq_attach(vpci_link->my.xirq,
			exynos_v920_vpci_xirq_handler, pcie_vf);
	dev_info(dev, "para-link interrupt number is : %d", vpci_link->my.xirq);
	if (!vpci_link->xid) {
		dev_err(dev, "exynos_vpci_link_attach ERROR\n");
		return -EINVAL;
	}

	/* ### Start Handshake ### */
	if (!pcie_vf->backend)
		vpci_link->running = true;

	return 0;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Print link status in pmem"
 * @logic "Print link status in pmem for each vlinks\n
 *	If vf is not on pf-domain, clear/set msi"
 * @params
 * @param{in, pcie_vf, struct ::exynos_pcie_vf *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_v920_vpci_pmem_show(struct exynos_pcie_vf *pcie_vf)
{
	struct device *dev = pcie_vf->pci->dev;
	u32 curVM = nkops.nk_id_get();
	int i;

	memcpy(&pcie_vf->vpci[0]->status,
		(int *)pcie_vf->vpci[0]->shared.pmem.vaddr, sizeof(int));
	dev_info(dev, "[%s] curVM :%d pcie_vf->vpci[0]->status: %d\n",
			__func__, curVM, pcie_vf->vpci[0]->status);

	if (pcie_vf->backend) {
		for (i = 1; i < pcie_vf->vlink_cnt; i++) {
			memcpy(&pcie_vf->vpci[i]->status,
				(int *)pcie_vf->vpci[i]->shared.pmem.vaddr,
				sizeof(int));
			dev_info(dev, "[%s] pcie_vf->vpci[%d]->status: %d\n",
				__func__, i, pcie_vf->vpci[i]->status);
		}
	}

	if (pcie_vf->vpci[0]->status == PF_LINKUP) {
		exynos_v920_pcie_vf_msi_clear_irq(pcie_vf);
		exynos_v920_pcie_vf_msi_set_irq(pcie_vf);
	}
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Set link status in pmem"
 * @logic "Set link status for each vlinks in pmem"
 * @params
 * @param{in/out, pcie_vf, struct ::exynos_pcie_vf *, not NULL}
 * @param{in, status, int, 0~3}
 * @endparam
 * @noret
 */
static void exynos_v920_vpci_pmem_set(struct exynos_pcie_vf *pcie_vf, int status)
{
	struct device *dev = pcie_vf->pci->dev;
	int i;

	dev_info(dev, "pmem_set: status: %d\n", status);

	pcie_vf->vpci[0]->status = status;

	if (exynos_v920_vpci_link_is_online(pcie_vf->vpci[0])) {
		memcpy((int *)pcie_vf->vpci[0]->shared.pmem.vaddr,
			&status, sizeof(int));
	} else {
		dev_info(dev, "V Link 0 is not online\n");
	}

	if (pcie_vf->backend) {
		for (i = 1; i < pcie_vf->vlink_cnt; i++) {
			pcie_vf->vpci[i]->status = status;
			if (exynos_v920_vpci_link_is_online(pcie_vf->vpci[i])) {
				memcpy(
				(int *)pcie_vf->vpci[i]->shared.pmem.vaddr,
						&status, sizeof(int));
			} else {
				dev_info(dev, "V Link %d is not online\n", i);
			}
		}
	}

	if (status == PF_LINKUP) {
		exynos_v920_pcie_vf_msi_clear_irq(pcie_vf);
		exynos_v920_pcie_vf_msi_set_irq(pcie_vf);
	}
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Trigger xirq"
 * @logic "Trigger xirq"
 * @params
 * @param{in/out, pcie_vf, struct ::exynos_pcie_vf *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_v920_vpci_intr_set(struct exynos_pcie_vf *pcie_vf)
{
	struct device *dev = pcie_vf->pci->dev;
	int i = 0;

	dev_info(dev, "trigger xirq\n");

	if (exynos_v920_vpci_link_is_online(pcie_vf->vpci[0]))
		exynos_v920_vpci_xirq_post(pcie_vf->vpci[0]);
	else
		dev_info(dev, "V Link 0 is not online\n");

	if (pcie_vf->vpci[0]->server) {
		for (i = 1; i < pcie_vf->vlink_cnt; i++) {
			if (exynos_v920_vpci_link_is_online(pcie_vf->vpci[i]))
				exynos_v920_vpci_xirq_post(pcie_vf->vpci[i]);
			else
				dev_info(dev, "V Link %d is not online\n", i);
		}
	}
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Set link status and transfer using xirq"
 * @logic "If vlink is not established, try vlink handshake\n
 *	Set link status info in pmem\n
 *	Trigger xirq"
 * @params
 * @param{in, value, int, 0~3}
 * @endparam
 * @noret
 */
void exynos_v920_vf_request_vlink_send(int value)
{
	struct exynos_pcie_vf *pcie_vf;
	int i, timeout;

	pcie_vf = &g_pcie_vf;

	if (!exynos_v920_vpci_link_is_online(pcie_vf->vpci[0])) {
		pcie_vf->vpci[0]->running = true;

		exynos_v920_vpci_link_ready(pcie_vf->vpci[0]);

		timeout = 0;

		while (!exynos_v920_vpci_link_is_online(pcie_vf->vpci[0]) &&
			timeout < 20000) {
			udelay(10);
			timeout++;
		}
	}

	if (pcie_vf->backend) {
		for (i = 1; i < pcie_vf->vlink_cnt; i++) {
			if (!exynos_v920_vpci_link_is_online(pcie_vf->vpci[i])) {
				pcie_vf->vpci[i]->running = true;

				exynos_v920_vpci_link_ready(pcie_vf->vpci[i]);

				timeout = 0;

				while (!exynos_v920_vpci_link_is_online
					(pcie_vf->vpci[i]) && timeout < 20000) {
					udelay(10);
					timeout++;
				}
			}
		}
	}

	exynos_v920_vpci_pmem_set(&g_pcie_vf, value);
	exynos_v920_vpci_intr_set(&g_pcie_vf);
}
EXPORT_SYMBOL(exynos_v920_vf_request_vlink_send);

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_pv}
 * @purpose "Initialize pci pv driver"
 * @logic "Get pci link info from dt\n
 *	Call vlink init for all vlinks"
 * @params
 * @param{in/out, pcie_vf, struct ::exynos_pcie_vf *, not NULL}
 * @endparam
 * @retval{-, int, 0, -ERANGE~0, <0}
 */
static int exynos_v920_vpci_init(struct exynos_pcie_vf *pcie_vf)
{
	struct device *dev = pcie_vf->pci->dev;
	int		 res;
	NkOsId	 myid  = nkops.nk_id_get();
	NkPhAddr plink = 0;

	mutex_lock(&_mutex);

	if (pcie_vf->inited) {
		mutex_unlock(&_mutex);
		return 0;
	}
	pcie_vf->inited = 1;
	pcie_vf->vlink_cnt = 0;

	while ((plink = nkops.nk_vlink_lookup("vpci", plink)) != 0) {
		NkDevVlink *vlink = (NkDevVlink *) nkops.nk_ptov(plink);

		if (!vlink) {
			dev_err(dev, "Getting Vlink Fail\n");
			return -EINVAL;
		}

		// Server
		if (vlink->s_id == myid) {
			pcie_vf->backend = true;
			res = exynos_v920_vpci_link_init(pcie_vf, vlink,
							plink, true);
			if (res != 0) {
				mutex_unlock(&_mutex);
				dev_err(dev, "Server: VLINK %d (%d -> %d) initialization failed\n",
					vlink->link, vlink->s_id, vlink->c_id);
				return res;
			}
		}

		// Client
		if (vlink->c_id == myid) {
			pcie_vf->backend = false;
			res = exynos_v920_vpci_link_init
					(pcie_vf, vlink, plink, false);
			if (res != 0) {
				mutex_unlock(&_mutex);
				dev_err(dev, "Client: VLINK %d (%d -> %d) initialization failed\n",
					vlink->link, vlink->c_id, vlink->s_id);
				return res;
			}
		}
		pcie_vf->vlink_cnt++;
	}

	mutex_unlock(&_mutex);

	if (pcie_vf->vlink_cnt == 0) {
		dev_err(dev, "can't find vpci vlink\n");
		return -ENOMEM;
	}

	return 0;
}

////////////////////////
//   PCIe VF driver   //
////////////////////////
/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_vf}
 * @purpose "Parse vf driver data from dt"
 * @logic "Parse vf driver data from dt"
 * @params
 * @param{in/out, pcie_vf, struct ::exynos_pcie_vf *, not NULL}
 * @param{in, np, struct ::device_node *, not NULL}
 * @endparam
 * @retval{-, int, 0, -ERANGE~0, <0}
 */
static int exynos_v920_pcie_vf_parse_dt(struct exynos_pcie_vf *pcie_vf,
					struct device_node *np)
{
	const char *use_msi;

	if (of_property_read_u32(np, "vf-num", &pcie_vf->vf_num)) {
		dev_err(NULL, "Failed to parse the number of VFs\n");
		return -EINVAL;
	} else {
		dev_info(NULL, "vf-num: %d\n", pcie_vf->vf_num);
	}

	if (of_property_read_u32(np, "msi-data", &pcie_vf->msi_data)) {
		dev_err(NULL, "Failed to parse MSI Data\n");
		return -EINVAL;
	} else {
		dev_info(NULL, "msi-data: %d\n", pcie_vf->msi_data);
	}

	if (!of_property_read_string(np, "use-msi", &use_msi)) {
		if (!strcmp(use_msi, "true")) {
			pcie_vf->use_msi = true;
			dev_info(NULL, "MSI is ENABLED.\n");
		} else if (!strcmp(use_msi, "false")) {
			pcie_vf->use_msi = false;
		} else {
			dev_info(NULL,
			"Invalid use-msi value (Set to default -> true)\n");
			pcie_vf->use_msi = true;
		}
	} else {
		pcie_vf->use_msi = false;
	}

	return 0;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_vf}
 * @purpose "Get vf resource from dt"
 * @logic "Get vf resource from dt"
 * @params
 * @param{in/out, pcie_vf, struct ::exynos_pcie_vf *, not NULL}
 * @endparam
 * @retval{ret, int, 0, >=0, >0}
 */
static int exynos_v920_pcie_vf_get_resource(struct exynos_pcie_vf *pcie_vf)
{
	struct platform_device *pdev = pcie_vf->pdev;
	struct resource *temp_rsc;
	int ret = 0;

	temp_rsc = platform_get_resource_byname
				(pdev, IORESOURCE_MEM, "outbound_mem");
	pcie_vf->outbound_base = ioremap
				(temp_rsc->start, resource_size(temp_rsc));
	if (IS_ERR(pcie_vf->outbound_base)) {
		ret = PTR_ERR(pcie_vf->outbound_base);
		goto fail;
	}

	pcie_vf->ob = temp_rsc;
	pcie_vf->ob->name = "OB";

	temp_rsc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	pcie_vf->dbi_base = ioremap(temp_rsc->start, resource_size(temp_rsc));
	if (IS_ERR(pcie_vf->dbi_base)) {
		ret = PTR_ERR(pcie_vf->dbi_base);
		goto fail;
	}

	pcie_vf->dbi = temp_rsc;
	pcie_vf->dbi->name = "DBI";

fail:
	return ret;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_vf}
 * @purpose "Clear msi irq"
 * @logic "Clear msi irq"
 * @params
 * @param{in, pcie_vf, struct ::exynos_pcie_vf *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_v920_pcie_vf_msi_clear_irq(struct exynos_pcie_vf *pcie_vf)
{
	struct device *dev = pcie_vf->pci->dev;
	int offset, irq;
	u32 val;

	offset = (pcie_vf->msi_data / 32) % 2;
	irq = pcie_vf->msi_data;

	dw_pcie_read(pcie_vf->dbi_base + VF_MSI_CTRL_INTR0_ENABLE
		+ offset * VF_MSI_CTRL_INT_OFFSET, 0x4, &val);
	val &= ~(1 << irq);
	dw_pcie_write(pcie_vf->dbi_base + VF_MSI_CTRL_INTR0_ENABLE
		+ offset * VF_MSI_CTRL_INT_OFFSET, 0x4, val);

	dev_info(dev, "[%s] msi_data:%d, val:0x%x\n",
				__func__, pcie_vf->msi_data, val);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_vf}
 * @purpose "Set msi irq"
 * @logic "Set msi irq"
 * @params
 * @param{in, pcie_vf, struct ::exynos_pcie_vf *, not NULL}
 * @endparam
 * @noret
 */
static void exynos_v920_pcie_vf_msi_set_irq(struct exynos_pcie_vf *pcie_vf)
{
	struct device *dev = pcie_vf->pci->dev;
	int offset, irq;
	u32 val;

	offset = (pcie_vf->msi_data / 32) % 2;
	irq = pcie_vf->msi_data;

	dw_pcie_read(pcie_vf->dbi_base + VF_MSI_CTRL_INTR0_ENABLE
		+ offset * VF_MSI_CTRL_INT_OFFSET, 0x4, &val);
	val |= 1 << irq;
	dw_pcie_write(pcie_vf->dbi_base + VF_MSI_CTRL_INTR0_ENABLE
		+ offset * VF_MSI_CTRL_INT_OFFSET, 0x4, val);

	dev_info(dev, "[%s] msi_data:%d, val:0x%x\n", __func__,
		pcie_vf->msi_data, val);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_vf}
 * @purpose "Handle msi interrupt"
 * @logic "Handle msi interrupt"
 * @params
 * @param{in, irq, int, >0}
 * @param{in, arg, void *, not NULL}
 * @endparam
 * @retval{-, irqreturn_t, 0, IRQ_HANDLED, not IRQ_HANDLED}
 */
static irqreturn_t exynos_v920_pcie_vf_irq_handler(int irq, void *arg)
{
	struct exynos_pcie_vf *pcie_vf = arg;
	int offset;
	u32 val;

	offset = (pcie_vf->msi_data / 32) % 2;

	dw_pcie_read(pcie_vf->dbi_base + VF_MSI_CTRL_INTR0_STATUS
		+ offset * VF_MSI_CTRL_INT_OFFSET, 0x4, &val);

	dw_pcie_write(pcie_vf->dbi_base + VF_MSI_CTRL_INTR0_STATUS
		+ offset * VF_MSI_CTRL_INT_OFFSET, 0x4, val);

	dev_info(pcie_vf->pci->dev, "[%s] msi_data:%d, val:0x%x\n",
		__func__, pcie_vf->msi_data, val);

	return IRQ_HANDLED;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_vf}
 * @purpose "Show pcie vf driver info"
 * @logic "Show pcie vf driver info"
 * @params
 * @param{in, dev, struct ::device *, not NULL}
 * @param{int, attr, struct ::device_attribute *, not NULL}
 * @param{in/out, buf, char *, not NULL}
 * @endparam
 * @retval{ret, int, 0, 0, not 0}
 */
static ssize_t show_pcie_vf_v920(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret = 0;

	dev_info(dev, "[%s]\n", __func__);
	return ret;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_vf}
 * @purpose "Handler for sysfs tests"
 * @logic "Parse test command from sysfs\n
 *	Perform test function"
 * @params
 * @param{in/out, dev, struct ::device *, not NULL}
 * @param{in, attr, struct ::device_attribute *, not NULL}
 * @param{in, buf, char *, not NULL}
 * @param{in, count, size_t, >=0}
 * @endparam
 * @retval{count, ssize_t, >=-ERANGE, int, -EINVAL}
 */
static ssize_t store_pcie_vf_v920(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int op_num;
	struct exynos_pcie_vf *pcie_vf = dev_get_drvdata(dev);
	u64 iommu_addr, iommu_size;

	u64 pf_addr, vf_addr;
	void __iomem *pf_rsc_start, *vf_rsc_start;

	int i;

	pf_addr = pcie_vf->ob->start - (pcie_vf->vf_num+1) *
					(resource_size(pcie_vf->ob)+1);


	if (sscanf(buf, "%10d%llx%llx", &op_num, &iommu_addr, &iommu_size) == 0)
		return -EINVAL;

	switch (op_num) {
	case 1:
		if (pcie_vf->vf_num == 0) {
			pf_rsc_start = ioremap(pf_addr, 0xffffff);
			dev_info(dev, "[%s] send PF doorbell\n", __func__);
			writel(0x5, pf_rsc_start + 0x0000);
			iounmap(pf_rsc_start);
		} else {
			dev_info(dev,
				"[%s] Try to send doorbell interrupt in VM%d\n",
				__func__, pcie_vf->vf_num);
		}
		break;

	case 2:
		vf_addr = pcie_vf->ob->start;
		vf_rsc_start = ioremap(vf_addr, 0xffffff);
		vf_rsc_start += (pcie_vf->vf_num + 1) * VF_DB_OFFSET;

		dev_info(dev, "[%s] send VF%d doorbell\n",
				__func__, pcie_vf->vf_num);
		writel((pcie_vf->vf_num+1), vf_rsc_start);
		iounmap(vf_rsc_start);
		break;

	case 3:
#if USE_RESERVED_MEMORY
		writel(0x1111, pcie_vf->rmem_base);
#endif
		break;

	case 4:
		exynos_v920_vpci_pmem_show(pcie_vf);
		break;

	case 5:
		if (pcie_vf->vf_num == 0)
			exynos_v920_vpci_pmem_set(pcie_vf, 1);
		else
			exynos_v920_vpci_pmem_set(pcie_vf, 2);

		break;

	case 6:
		exynos_v920_vpci_intr_set(pcie_vf);
		break;

	case 7:
		if (exynos_v920_vpci_link_is_online(pcie_vf->vpci[0]))
			dev_info(dev, "V Link 0 is online\n");
		else
			dev_info(dev, "V Link 0 is not online\n");

		if (pcie_vf->backend) {
			for (i = 1; i < pcie_vf->vlink_cnt; i++) {
				if (exynos_v920_vpci_link_is_online
					(pcie_vf->vpci[i])) {
					dev_info(dev,
						"V Link %d is online\n", i);
				} else {
					dev_info(dev,
						"V Link %d is not online\n", i);
				}
		}
	}
	break;

	case 8:
		/* V Link Start Test (for PF) */
		pcie_vf->vpci[0]->running = true;

		exynos_v920_vpci_link_ready(pcie_vf->vpci[0]);

		if (pcie_vf->backend) {
			for (i = 1; i < pcie_vf->vlink_cnt; i++) {
				pcie_vf->vpci[i]->running = true;

				exynos_v920_vpci_link_ready(pcie_vf->vpci[i]);
			}
		}
	break;

	default:
		dev_warn(dev, "Unsupported Test Number(%d)...\n", op_num);
	}

	return count;
}

static DEVICE_ATTR(vf_sysfs, S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP,
			show_pcie_vf_v920, store_pcie_vf_v920);

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_vf}
 * @purpose "Create PCIe vf sys file"
 * @logic "Create PCIe vf sys file"
 * @params
 * @param{in/out, dev, struct ::device *, not NULL}
 * @endparam
 * @retval{-, int, 0, >=-EINVAL, not 0}
 */
static inline int create_pcie_sys_file_v920(struct device *dev)
{
	return device_create_file(dev, &dev_attr_vf_sysfs);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_vf}
 * @purpose "Remove PCIe vf sys file"
 * @logic "Remove PCIe vf sys file"
 * @params
 * @param{in/out, dev, struct ::device *, not NULL}
 * @endparam
 * @noret
 */
static inline void remove_pcie_sys_file_v920(struct device *dev)
{
	device_remove_file(dev, &dev_attr_vf_sysfs);
}

#if USE_RESERVED_MEMORY
/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_vf}
 * @purpose "Initialize reserve memory for pcie vf"
 * @logic "Initialize reserve memory for pcie vf"
 * @params
 * @param{in, rmem, struct ::reserved_mem *, not NULL}
 * @endparam
 * @retval{-, int, 0, 0, not 0}
 */
static int exynos_v920_pcie_vf_rmem_init(struct reserved_mem *rmem)
{
	dev_info(NULL, "[%s] Shared memory for VF\n", __func__);

	dev_info(NULL, "[%s] VF mem name : 0x%s\n", __func__, rmem->name);
	dev_info(NULL, "[%s] VF mem base : 0x%llx\n", __func__, rmem->base);
	dev_info(NULL, "[%s] VF mem size : 0x%llx\n", __func__, rmem->size);

	g_pcie_vf_rmem.base = rmem->base;
	g_pcie_vf_rmem.name = rmem->name;
	g_pcie_vf_rmem.size = rmem->size;

	return 0;
}
RESERVEDMEM_OF_DECLARE(pcie_vf_rmem, "exynosautov920,pcie-vf-rmem",
				exynos_v920_pcie_vf_rmem_init);
#endif

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_vf}
 * @purpose "Probe pcie vf"
 * @logic "Create sys file\n
 *	Parse dt\n
 *	Get resource\n
 *	Request irq\n
 *	Set reserve memory\n
 *	Set driver data\n
 *	Initialize pv driver"
 * @params
 * @param{in/out, pdev, struct ::platform_device *, not NULL}
 * @endparam
 * @retval{ret, int, 0, not NULL, not 0}
 */
static int exynos_v920_pcie_vf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci;
	struct exynos_pcie_vf *pcie_vf;
	int ret = 0;
	int vf_num;

	if (create_pcie_sys_file_v920(&pdev->dev))
		dev_warn(&pdev->dev, "Failed to create pcie vf sys file\n");

	if (of_property_read_u32(pdev->dev.of_node, "vf-num", &vf_num)) {
		dev_err(&pdev->dev, "Failed to parse the VF channel number\n");
		return -EINVAL;
	}

	pcie_vf = &g_pcie_vf;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pcie_vf->pci = pci;
	pcie_vf->pdev = pdev;

	/* parsing pcie dts data for exynos */
	ret = exynos_v920_pcie_vf_parse_dt(pcie_vf, pdev->dev.of_node);
	if (ret)
		goto probe_fail;

	ret = exynos_v920_pcie_vf_get_resource(pcie_vf);
	if (ret) {
		dev_err(dev, "getting resource for PCIe EP is failed\n");
		ret = -EINVAL;
		goto probe_fail;
	}

	/* request irq */
	pcie_vf->irq = platform_get_irq(pdev, 0);
	if (!pcie_vf->irq) {
		dev_err(&pdev->dev, "failed to get irq for PCIe VF\n");
		ret = -ENODEV;
		goto probe_fail;
	}
	dev_info(&pdev->dev, "[%s] pcie_vf->irq: %d\n", __func__, pcie_vf->irq);

	dev_info(&pdev->dev, "[%s] pp->msi_irq: %d msi data:%llx\n",
		__func__, pcie_vf->pci->pp.msi_irq,
		virt_to_phys((void *)pcie_vf->pci->pp.msi_data));

	ret = request_irq(pcie_vf->irq, exynos_v920_pcie_vf_irq_handler,
			IRQF_SHARED, "exynos-pcie-vf", pcie_vf);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq\n");
		ret = -EINVAL;
		goto probe_fail;
	}
#if USE_RESERVED_MEMORY
	dev_info(&pdev->dev, "PCIe VF: allocate reserved mem\n");
	dev_info(&pdev->dev, "base : 0x%llx, size: 0x%llx\n",
		g_pcie_vf_rmem.base, g_pcie_vf_rmem.size);
	pcie_vf->rmem_base = ioremap(g_pcie_vf_rmem.base, g_pcie_vf_rmem.size);
	if (IS_ERR(pcie_vf->rmem_base)) {
		ret = PTR_ERR(pcie_vf->rmem_base);
		goto probe_fail;
	}
	dev_info(&pdev->dev, "pcie_vf->rmem_base : 0x%p\n", pcie_vf->rmem_base);
#endif
	/* set drv data */
	platform_set_drvdata(pdev, pcie_vf);

	ret = exynos_v920_vpci_init(pcie_vf);
	if (ret) {
		dev_err(&pdev->dev, "failed to vpci init\n");
		ret = -EPERM;
		goto probe_fail;
	}

probe_fail:
	if (ret)
		dev_err(&pdev->dev, "%s: pcie probe failed\n", __func__);
	else
		dev_info(&pdev->dev, "%s: pcie probe success\n", __func__);

	return ret;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{pcie_vf}
 * @purpose "Remove pcie vf"
 * @logic "Do nothing"
 * @params
 * @param{in, pdev, struct ::platform_device *, not NULL}
 * @endparam
 * @retval{-, int, 0, 0, not 0}
 */
static int exynos_v920_pcie_vf_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "Remove EXYNOS PCIe VF driver\n");
	return 0;
}

static const struct of_device_id exynos_v920_pcie_vf_of_match[] = {
	{ .compatible = "samsung,exynos-v920-pcie-vf", },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_v920_pcie_vf_of_match);

static struct platform_driver exynos_pcie_vf_driver = {
	.remove		= exynos_v920_pcie_vf_remove,
	.driver = {
		.name	= "exynos-v920-pcie-vf",
		.owner	= THIS_MODULE,
		.of_match_table = exynos_v920_pcie_vf_of_match,
	},
};

module_platform_driver(exynos_pcie_vf_driver);

MODULE_AUTHOR("Jiheon Oh <jiheon.oh@samsung.com>");
MODULE_DESCRIPTION("Samsung PCIe VF driver");
MODULE_LICENSE("GPL v2");

