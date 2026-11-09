/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * CPU and GPU voltages for this very chip, as the stock kernel picks them
 * (pwrcal, S5E7870-asv.c). Every Exynos 7870 is binned at the factory: the
 * ASV fuses give its group and the table version, and the ECT, which S-BOOT
 * leaves in RAM, holds the voltage of every level for every group, plus the
 * sub-group corrections (SSA). The DTB carries the voltages of one unit; here
 * they are replaced, OPP by OPP, with the ones of the unit we run on.
 *
 * The GPU rail, BUCK2, also supplies MIF (the memory controller), INT and
 * DISP: the stock devfreq drivers of those domains ask BUCK2 for their own
 * voltages (exynos7870.dtsi, reg_name "BUCK2"), and the regulator keeps the
 * highest request. Here the GPU is the only consumer, while MIF and INT run
 * at the clocks S-BOOT left (MIF at 902 MHz), so no GPU OPP may ask for less
 * than the highest level of those three domains on this chip. Below that,
 * the memory controller and the bus run undervolted as soon as the GPU
 * driver probes: random corruption, or a hard hang, on chips with less
 * margin.
 *
 * Everything is computed first and written only if every OPP found its
 * voltage: anything missing or out of range leaves the DTB as it is.
 */

#include <libfdt.h>
#include "loader.h"

#define ECT_ADDR	0x90000000UL	/* reserved as ect@90000000 in the DTB */
#define ECT_SIZE	0x14000UL
#define ASV_FUSES	0x100D9000UL	/* secure only: read through the monitor */
#define SMC_CMD_REG	0xFFFFFF9BUL
#define SMC_SFR_READ	0xC0000000UL

#define VOLT_STEP	6250		/* compact ASV tables: u8 x 6.25 mV */
#define VOLT_MARGIN	25000		/* as the DTB: COLD_VOLT_OFFSET, Mali */
#define VOLT_MIN	500000
#define VOLT_MAX	1300000
#define MAX_OPPS	20

static const struct {
	const char *ect;	/* ASV domain */
	const char *ssa;	/* GEN table with its SSA parameters */
	unsigned int fuse;	/* 16-bit field in the fuse words */
	const char *path;	/* OPP table in the DTB */
} domains[] = {
	{ "dvfs_cpucl0", "SSA_CPUCL0", 0, "/opp-table-cpucl0" },
	{ "dvfs_cpucl1", "SSA_CPUCL1", 1, "/opp-table-cpucl1" },
	{ "dvfs_g3d", "SSA_G3D", 2, "/soc@0/gpu@11400000/opp-table" },
};

#define NDOM (sizeof(domains) / sizeof(domains[0]))
#define DOM_G3D 2

/* domains sharing BUCK2 with the GPU, with no DVFS of their own here */
static const struct {
	const char *ect, *ssa;
	unsigned int fuse;
} rail[] = {
	{ "dvfs_mif", "SSA_MIF", 3 },
	{ "dvfs_int", "SSA_INT", 4 },
	{ "dvfs_disp", "SSA_DISP", 5 },
};

#define NRAIL (sizeof(rail) / sizeof(rail[0]))

static const unsigned char *ect = (const unsigned char *)ECT_ADDR;

static int smc_read(unsigned long addr, uint32_t *val)
{
	register unsigned long x0 asm("x0") = SMC_CMD_REG;
	register unsigned long x1 asm("x1") = SMC_SFR_READ | (addr >> 2);
	register unsigned long x2 asm("x2") = 0;
	register unsigned long x3 asm("x3") = 0;

	asm volatile("smc #0"
		     : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3)
		     :
		     : "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11",
		       "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	if (x0)
		return -1;
	*val = x2;
	return 0;
}

/* bounds-checked little-endian reads inside the ECT */
static int u32_at(unsigned long off, uint32_t *v)
{
	if (off + 4 > ECT_SIZE)
		return -1;
	*v = ect[off] | ect[off + 1] << 8 | ect[off + 2] << 16 | (uint32_t)ect[off + 3] << 24;
	return 0;
}

/* an ECT string: u32 length, the bytes and a NUL, padded to 4 */
static int str_at(unsigned long *off, const char **s)
{
	uint32_t len;

	if (u32_at(*off, &len) || len > 64 || *off + 4 + len + 1 > ECT_SIZE)
		return -1;
	*s = (const char *)ect + *off + 4;
	len += 1;
	*off += 4 + ((len + 3) & ~3u);
	return 0;
}

/*
 * Offset of entry name in a directory: the ECT header (from offset 12) or
 * a block (from offset 8, after parser version and version string).
 * Returns the offset, relative to rel, or 0.
 */
static unsigned long find(unsigned long dir, unsigned long rel, const char *name)
{
	uint32_t n, o, i;
	const char *s;

	if (u32_at(dir, &n) || n > 64)
		return 0;
	dir += 4;
	for (i = 0; i < n; i++) {
		if (str_at(&dir, &s) || u32_at(dir, &o))
			return 0;
		dir += 4;
		if (!strncmp(s, name, strlen(name) + 1))
			return o && rel + o < ECT_SIZE ? rel + o : 0;
	}
	return 0;
}

struct asv_domain {
	uint32_t ngrp, nlev;
	unsigned long levels;	/* u32[nlev], MHz */
	unsigned long volts;	/* u8[nlev][ngrp] of the table version */
	uint32_t subgrp, base, step, ssa1[8];
};

static int load_domain(unsigned long asv, unsigned long gen, const char *dom,
		       const char *ssa, uint32_t ver, struct asv_domain *a)
{
	unsigned long off, p;
	uint32_t ntab, ncol, nrow, t, i;

	off = find(asv + 8, asv, dom);
	if (!off || u32_at(off, &a->ngrp) || u32_at(off + 4, &a->nlev) ||
	    u32_at(off + 8, &ntab) || !a->ngrp || a->ngrp > 32 ||
	    !a->nlev || a->nlev > 64 || !ntab || ntab > 16 ||
	    (a->nlev * a->ngrp) % 4)	/* no padding between compact tables */
		return -1;
	a->levels = off + 12;
	off = a->levels + 4 * a->nlev;
	if (ver >= ntab)
		ver = ntab - 1;	/* as asv_set_freq_limit() */
	for (t = 0; ; t++) {
		off += 4 + 8 + 4 * a->nlev;	/* version, boot/resume, level_en */
		if (t == ver)
			break;
		off += a->nlev * a->ngrp;
	}
	a->volts = off;
	if (off + a->nlev * a->ngrp > ECT_SIZE)
		return -1;

	p = find(gen + 8, gen, ssa);
	if (!p || u32_at(p, &ncol) || u32_at(p + 4, &nrow) || ncol < 12 || ver >= nrow)
		return -1;
	p += 8 + 4 * ver * ncol;
	if (u32_at(p + 4, &a->subgrp) || u32_at(p + 8, &a->base) || u32_at(p + 12, &a->step))
		return -1;
	for (i = 0; i < 8; i++)
		if (u32_at(p + 16 + 4 * i, &a->ssa1[i]))
			return -1;
	return 0;
}

/* get_asv_voltage() of the stock kernel, for the level at mhz */
static int voltage(const struct asv_domain *a, uint32_t field, uint32_t mhz, uint32_t *uv)
{
	int grp = field & 0xf, mod = (field >> 4) & 0xf;
	unsigned int ssa10 = (field >> 8) & 3, ssa11 = (field >> 10) & 3;
	unsigned int ssa0 = (field >> 12) & 0xf;
	uint32_t lv, f, v;

	if (mod >= 8)
		mod -= 16;
	grp += mod;
	if (grp < 0 || grp >= (int)a->ngrp)
		return -1;
	for (lv = 0; lv < a->nlev; lv++)
		if (!u32_at(a->levels + 4 * lv, &f) && f == mhz)
			break;
	if (lv == a->nlev)
		return -1;
	v = ect[a->volts + lv * a->ngrp + grp] * VOLT_STEP;
	v += lv < a->subgrp ? a->ssa1[ssa10] : a->ssa1[ssa11 + 4];
	if (a->base && v < a->base + ssa0 * a->step)
		v = a->base + ssa0 * a->step;
	*uv = v;
	return 0;
}

/* the highest voltage of any level of the domain */
static int max_voltage(const struct asv_domain *a, uint32_t field, uint32_t *uv)
{
	uint32_t lv, f, v;

	*uv = 0;
	for (lv = 0; lv < a->nlev; lv++) {
		if (u32_at(a->levels + 4 * lv, &f) || voltage(a, field, f, &v))
			return -1;
		if (v > *uv)
			*uv = v;
	}
	return 0;
}

/* the 16-bit fuse field of a domain; 0 on unfused parts */
static uint32_t fuse_field(const uint32_t *w, unsigned int idx)
{
	if (!((w[3] >> 7) & 1))
		return 0;
	return (w[idx / 2] >> (16 * (idx % 2))) & 0xffff;
}

static uint32_t be32(const void *p)
{
	const unsigned char *b = p;

	return (uint32_t)b[0] << 24 | b[1] << 16 | b[2] << 8 | b[3];
}

static void put_be32(void *p, uint32_t v)
{
	unsigned char *b = p;

	b[0] = v >> 24;
	b[1] = v >> 16;
	b[2] = v >> 8;
	b[3] = v;
}

void set_asv(void *fdt)
{
	static struct {
		int node, cells;
		uint32_t uv;
	} opp[NDOM * MAX_OPPS];
	struct asv_domain a;
	uint32_t w[4], ver, pv, field, floor = 0, uv;
	unsigned long asv, gen;
	unsigned int d, n = 0, i;
	int table, node, chosen;

	for (i = 0; i < 4; i++)
		if (smc_read(ASV_FUSES + 4 * i, &w[i])) {
			loader_log("asv-smc", i);
			return;
		}
	chosen = fdt_path_offset(fdt, "/chosen");
	if (chosen >= 0) {
		uint32_t cells[4];

		for (i = 0; i < 4; i++)
			put_be32(&cells[i], w[i]);
		fdt_setprop(fdt, chosen, "gtaxl,asv-fuses", cells, sizeof(cells));
	}

	/* unfused parts: table 0, group 0, no SSA, as asv_get_asvinfo() */
	ver = (w[3] >> 7) & 1 ? w[3] & 0x7f : 0;
	if (memcmp(ect, "PARA", 4)) {
		loader_log("asv-ect", 0);
		return;
	}
	asv = find(12, 0, "ASV");
	gen = find(12, 0, "GEN");
	if (!asv || !gen || u32_at(asv, &pv) || pv < 3) {
		loader_log("asv-ect", 1);
		return;
	}

	for (i = 0; i < NRAIL; i++) {
		if (load_domain(asv, gen, rail[i].ect, rail[i].ssa, ver, &a) ||
		    max_voltage(&a, fuse_field(w, rail[i].fuse), &uv)) {
			loader_log("asv-rail", i);
			return;
		}
		if (uv > floor)
			floor = uv;
	}
	floor += VOLT_MARGIN;
	if (floor < VOLT_MIN || floor > VOLT_MAX) {
		loader_log("asv-rail-uv", floor);
		return;
	}

	for (d = 0; d < NDOM; d++) {
		field = fuse_field(w, domains[d].fuse);
		table = fdt_path_offset(fdt, domains[d].path);
		if (table < 0 || load_domain(asv, gen, domains[d].ect, domains[d].ssa, ver, &a)) {
			loader_log("asv-dom", d);
			return;
		}
		fdt_for_each_subnode(node, fdt, table) {
			const void *hz = fdt_getprop(fdt, node, "opp-hz", NULL);
			const uint32_t *mv;
			int len;
			uint32_t uv, mhz;

			mv = fdt_getprop(fdt, node, "opp-microvolt", &len);
			if (!hz || !mv)
				continue;
			mhz = (uint32_t)((((uint64_t)be32(hz) << 32 | be32((const char *)hz + 4)) + 500000) / 1000000);
			if (n == NDOM * MAX_OPPS || voltage(&a, field, mhz, &uv) ||
			    (len != 4 && len != 12)) {
				loader_log("asv-opp", mhz);
				return;
			}
			uv += VOLT_MARGIN;
			if (d == DOM_G3D && uv < floor)
				uv = floor;
			if (uv < VOLT_MIN || uv > VOLT_MAX ||
			    (len == 12 && uv > be32(&mv[2]))) {
				loader_log("asv-uv", uv);
				return;
			}
			opp[n].node = node;
			opp[n].cells = len / 4;
			opp[n].uv = uv;
			n++;
		}
	}

	/* every OPP has its voltage: write them, in place */
	for (i = 0; i < n; i++) {
		uint32_t cells[3];
		const uint32_t *mv = fdt_getprop(fdt, opp[i].node, "opp-microvolt", NULL);

		memcpy(cells, mv, opp[i].cells * 4);
		put_be32(&cells[0], opp[i].uv);
		if (opp[i].cells == 3)
			put_be32(&cells[1], opp[i].uv);
		fdt_setprop_inplace(fdt, opp[i].node, "opp-microvolt", cells, opp[i].cells * 4);
	}
	if (chosen >= 0) {
		uint32_t cell;

		put_be32(&cell, floor);
		fdt_setprop(fdt, chosen, "gtaxl,buck2-floor", &cell, sizeof(cell));
	}
}
