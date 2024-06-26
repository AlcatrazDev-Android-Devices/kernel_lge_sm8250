/* SPDX-License-Identifier: GPL-2.0 */
/*
 * iommu trace points
 *
 * Copyright (C) 2013 Shuah Khan <shuah.kh@samsung.com>
 *
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM iommu

#if !defined(_TRACE_IOMMU_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_IOMMU_H

#include <linux/tracepoint.h>
#include <linux/iommu.h>

struct device;
struct iommu_domain;

DECLARE_EVENT_CLASS(iommu_group_event,

	TP_PROTO(int group_id, struct device *dev),

	TP_ARGS(group_id, dev),

	TP_STRUCT__entry(
		__field(int, gid)
		__string(device, dev_name(dev))
	),

	TP_fast_assign(
		__entry->gid = group_id;
		__assign_str(device, dev_name(dev));
	),

	TP_printk("IOMMU: groupID=%d device=%s",
			__entry->gid, __get_str(device)
	)
);

DEFINE_EVENT(iommu_group_event, add_device_to_group,

	TP_PROTO(int group_id, struct device *dev),

	TP_ARGS(group_id, dev)

);

DEFINE_EVENT(iommu_group_event, remove_device_from_group,

	TP_PROTO(int group_id, struct device *dev),

	TP_ARGS(group_id, dev)
);

DECLARE_EVENT_CLASS(iommu_device_event,

	TP_PROTO(struct device *dev),

	TP_ARGS(dev),

	TP_STRUCT__entry(
		__string(device, dev_name(dev))
	),

	TP_fast_assign(
		__assign_str(device, dev_name(dev));
	),

	TP_printk("IOMMU: device=%s", __get_str(device)
	)
);

DEFINE_EVENT(iommu_device_event, attach_device_to_domain,

	TP_PROTO(struct device *dev),

	TP_ARGS(dev)
);

DEFINE_EVENT(iommu_device_event, detach_device_from_domain,

	TP_PROTO(struct device *dev),

	TP_ARGS(dev)
);

TRACE_EVENT(map,

	TP_PROTO(struct iommu_domain *domain, unsigned long iova,
		 phys_addr_t paddr, size_t size, int prot),

	TP_ARGS(domain, iova, paddr, size, prot),

	TP_STRUCT__entry(
		__string(name, domain->name)
		__field(u64, iova)
		__field(u64, paddr)
		__field(size_t, size)
		__field(int, prot)
	),

	TP_fast_assign(
		__assign_str(name, domain->name);
		__entry->iova = iova;
		__entry->paddr = paddr;
		__entry->size = size;
		__entry->prot = prot;
	),

	TP_printk("IOMMU:%s iova=0x%016llx paddr=0x%016llx size=0x%zx prot=0x%x",
			__get_str(name), __entry->iova, __entry->paddr,
			__entry->size, __entry->prot
	)
);

TRACE_EVENT(unmap,

	TP_PROTO(struct iommu_domain *domain, unsigned long iova, size_t size,
			size_t unmapped_size),

	TP_ARGS(domain, iova, size, unmapped_size),

	TP_STRUCT__entry(
		__string(name, domain->name)
		__field(u64, iova)
		__field(size_t, size)
		__field(size_t, unmapped_size)
	),

	TP_fast_assign(
		__assign_str(name, domain->name);
		__entry->iova = iova;
		__entry->size = size;
		__entry->unmapped_size = unmapped_size;
	),

	TP_printk("IOMMU:%s iova=0x%016llx size=0x%zx unmapped_size=0x%zx",
			__get_str(name), __entry->iova, __entry->size,
			__entry->unmapped_size
	)
);

TRACE_EVENT(map_sg,

	TP_PROTO(struct iommu_domain *domain, unsigned long iova, size_t size,
		int prot),

	TP_ARGS(domain, iova, size, prot),

	TP_STRUCT__entry(
		__string(name, domain->name)
		__field(u64, iova)
		__field(size_t, size)
		__field(int, prot)
	),

	TP_fast_assign(
		__assign_str(name, domain->name);
		__entry->iova = iova;
		__entry->size = size;
		__entry->prot = prot;
	),

	TP_printk("IOMMU:%s iova=0x%016llx size=0x%zx prot=0x%x",
			__get_str(name), __entry->iova, __entry->size,
			__entry->prot
	)
);

#ifdef CONFIG_LGE_MORE_IOMMU_TRACE
TRACE_EVENT(io_pgtable_free,

	TP_PROTO(u64 *table_base, u64 *pudp, u64 pud, unsigned long iova, int block),

	TP_ARGS(table_base, pudp, pud, iova, block),

	TP_STRUCT__entry(
		__field(u64*, table_base)
		__field(u64*, pudp)
		__field(u64, pud)
		__field(u64, iova)
		__field(int, block)
	),

	TP_fast_assign(
		__entry->table_base = table_base;
		__entry->pudp = pudp;
		__entry->pud = pud;
		__entry->iova = iova;
		__entry->block = block;
	),

	TP_printk("IOMMU: Freeing %s io-pgtable at 0x%lx pud=0x%lx *pud=0x%lx iova=0x%lx blk=%d",
			__entry->block ? "block" : "", __entry->table_base, __entry->pudp,
			__entry->pud, __entry->iova, __entry->block
	)
);

TRACE_EVENT(io_pgtable_install,

	TP_PROTO(u64 *table_base, u64 *pudp, u64 pud, int block),

	TP_ARGS(table_base, pudp, pud, block),

	TP_STRUCT__entry(
		__field(u64*, table_base)
		__field(u64*, pudp)
		__field(u64, pud)
		__field(int, block)
	),

	TP_fast_assign(
		__entry->table_base = table_base;
		__entry->pudp = pudp;
		__entry->pud = pud;
		__entry->block = block;
	),

	TP_printk("IOMMU: Installed %s io-pgtable at 0x%lx pud=0x%lx *pud=0x%lx blk=%d",
			__entry->block ? "block" : "", __entry->table_base, __entry->pudp,
			__entry->pud, __entry->block
	)
);
#endif /* CONFIG_LGE_MORE_IOMMU_TRACE */

DECLARE_EVENT_CLASS(iommu_error,

	TP_PROTO(struct device *dev, unsigned long iova, int flags),

	TP_ARGS(dev, iova, flags),

	TP_STRUCT__entry(
		__string(device, dev_name(dev))
		__string(driver, dev_driver_string(dev))
		__field(u64, iova)
		__field(int, flags)
	),

	TP_fast_assign(
		__assign_str(device, dev_name(dev));
		__assign_str(driver, dev_driver_string(dev));
		__entry->iova = iova;
		__entry->flags = flags;
	),

	TP_printk("IOMMU:%s %s iova=0x%016llx flags=0x%04x",
			__get_str(driver), __get_str(device),
			__entry->iova, __entry->flags
	)
);

DEFINE_EVENT(iommu_error, io_page_fault,

	TP_PROTO(struct device *dev, unsigned long iova, int flags),

	TP_ARGS(dev, iova, flags)
);

DECLARE_EVENT_CLASS(iommu_tlbi,

	TP_PROTO(struct device *dev, u64 time),

	TP_ARGS(dev, time),

	TP_STRUCT__entry(
		__string(device, dev_name(dev))
		__field(u64, time)
	),

	TP_fast_assign(
		__assign_str(device, dev_name(dev));
		__entry->time = time;
	),

	TP_printk("IOMMU:%s %lld us",
			__get_str(device), __entry->time
	)
);

DEFINE_EVENT(iommu_tlbi, tlbi_start,

	TP_PROTO(struct device *dev, u64 time),

	TP_ARGS(dev, time)
);

DEFINE_EVENT(iommu_tlbi, tlbi_end,

	TP_PROTO(struct device *dev, u64 time),

	TP_ARGS(dev, time)
);

DEFINE_EVENT(iommu_tlbi, tlbsync_timeout,

	TP_PROTO(struct device *dev, u64 time),

	TP_ARGS(dev, time)
);

TRACE_EVENT(smmu_init,

	TP_PROTO(u64 time),

	TP_ARGS(time),

	TP_STRUCT__entry(
		__field(u64, time)
	),

	TP_fast_assign(
		__entry->time = time;
	),

	TP_printk("ARM SMMU init latency: %lld us", __entry->time)
);

#endif /* _TRACE_IOMMU_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
