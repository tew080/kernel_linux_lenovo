/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Macros for manipulating and testing flags related to a
 * pageblock_nr_pages number of pages.
 *
 * Copyright (C) IBM Corporation, 2006
 *
 * Original author, Mel Gorman
 * Major cleanups and reduction of bit operations, Andy Whitcroft
 */
#ifndef PAGEBLOCK_FLAGS_H
#define PAGEBLOCK_FLAGS_H

#include <linux/types.h>

#define PB_migratetype_bits 3
/* Bit indices that affect a whole block of pages */
enum pageblock_bits {
	PB_migrate,
	PB_migrate_end = PB_migrate + PB_migratetype_bits - 1,
			/* 3 bits required for migrate types */
	PB_migrate_skip,/* If set the block is skipped by compaction */

	/*
	 * Assume the bits will always align on a word. If this assumption
	 * changes then get/set pageblock needs updating.
	 */
	NR_PAGEBLOCK_BITS
};

#if defined(CONFIG_HUGETLB_PAGE)

#ifdef CONFIG_HUGETLB_PAGE_SIZE_VARIABLE

/* Huge page sizes are variable */
extern unsigned int pageblock_order;

#else /* CONFIG_HUGETLB_PAGE_SIZE_VARIABLE */

/*
 * Huge pages are a constant size, but don't exceed the maximum allocation
 * granularity.
 */
#define pageblock_order		MIN_T(unsigned int, HUGETLB_PAGE_ORDER, MAX_PAGE_ORDER)

#endif /* CONFIG_HUGETLB_PAGE_SIZE_VARIABLE */

#elif defined(CONFIG_TRANSPARENT_HUGEPAGE)

#define pageblock_order		MIN_T(unsigned int, HPAGE_PMD_ORDER, MAX_PAGE_ORDER)

#else /* CONFIG_TRANSPARENT_HUGEPAGE */

/* If huge pages are not used, group by MAX_ORDER_NR_PAGES */
#ifdef CONFIG_TWEAKS
#define pageblock_order		PAGE_ALLOC_COSTLY_ORDER
#else
#define pageblock_order		MAX_PAGE_ORDER
#endif

#endif /* CONFIG_HUGETLB_PAGE */

#define pageblock_nr_pages	(1UL << pageblock_order)
#define pageblock_align(pfn)	ALIGN((pfn), pageblock_nr_pages)
#define pageblock_aligned(pfn)	IS_ALIGNED((pfn), pageblock_nr_pages)
#define pageblock_start_pfn(pfn)	ALIGN_DOWN((pfn), pageblock_nr_pages)
#define pageblock_end_pfn(pfn)		ALIGN((pfn) + 1, pageblock_nr_pages)

/* Forward declaration */
struct page;

unsigned long get_pfnblock_flags_mask(const struct page *page,
				unsigned long pfn,
				unsigned long mask);

void set_pfnblock_flags_mask(struct page *page,
				unsigned long flags,
				unsigned long pfn,
				unsigned long mask);

/* Declarations for getting and setting flags. See mm/page_alloc.c */
#ifdef CONFIG_COMPACTION
#define get_pageblock_skip(page) \
	get_pfnblock_flags_mask(page, page_to_pfn(page),	\
			(1 << (PB_migrate_skip)))
#define clear_pageblock_skip(page) \
	set_pfnblock_flags_mask(page, 0, page_to_pfn(page),	\
			(1 << PB_migrate_skip))
#define set_pageblock_skip(page) \
	set_pfnblock_flags_mask(page, (1 << PB_migrate_skip),	\
			page_to_pfn(page),			\
			(1 << PB_migrate_skip))
#else
static inline bool get_pageblock_skip(struct page *page)
{
	return false;
}
static inline void clear_pageblock_skip(struct page *page)
{
}
static inline void set_pageblock_skip(struct page *page)
{
}
#endif /* CONFIG_COMPACTION */

#endif	/* PAGEBLOCK_FLAGS_H */
