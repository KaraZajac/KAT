/*
 * tuner_trace: run librtlsdr's E4000, FC0012, FC0013 and FC2580 drivers against a model of the
 * tuner's registers and print every I2C transaction they make, so that the Rust port in
 * ../src/tuners can be checked against the C transaction by transaction (the tests in
 * ../src/tuners/trace.rs replay the same scenarios and compare with the traces made here).
 *
 * Build, with the librtlsdr sources (https://gitea.osmocom.org/sdr/rtl-sdr, release 2.0.3) in
 * $RTLSDR:
 *
 *   gcc -O0 -I$RTLSDR/include -o tuner_trace tuner_trace.c $RTLSDR/src/tuner_e4k.c \
 *       $RTLSDR/src/tuner_fc0012.c $RTLSDR/src/tuner_fc0013.c $RTLSDR/src/tuner_fc2580.c
 *
 * and for each tuner:
 *
 *   ./tuner_trace ../src/tuners/testdata/fc0013.scn > ../src/tuners/testdata/fc0013.trace
 *
 * Scenario lines (hex for addresses, registers and values; '#' starts a comment):
 *   tuner e4000|fc0012|fc0013|fc2580   start over with that tuner and an all-zero register model
 *   xtal <Hz>                          the tuner's clock (default 28.8 MHz)
 *   reg <addr> <reg> <val>             set a register of the model
 *   ovr <addr> <reg> <and> <or>        reads of that register return (stored & and) | or
 *   init | exit | freq <Hz> | bw <Hz> | gain <tenths of dB> | gainmode <0|1>
 * Output: "> <line>" for each scenario line, then "W <addr>: <bytes>" for each I2C write,
 * "R <addr>: <byte>" for each I2C read, "G <pin> <0|1>" for each GPIO bit set, and "= ok" or
 * "= err" for what the operation returned.
 *
 * The glue functions below (e4000_*, fc0012_*, fc0013_*, fc2580_*) are librtlsdr.c's, with its
 * device structure replaced by this model.
 *
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de> (the librtlsdr.c glue)
 * Copyright (C) 2026 KAT contributors
 *
 * This program is free software; you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rtlsdr_i2c.h"
#include "tuner_e4k.h"
#include "tuner_fc0012.h"
#include "tuner_fc0013.h"
#include "tuner_fc2580.h"

#define min(a, b) (((a) < (b)) ? (a) : (b))

/* The register model: 256 registers per I2C address, the register pointer the last write set,
 * and read overrides for status registers (PLL lock, VCO voltage, filter calibration). */
static uint8_t regs[256][256];
static uint8_t ptr[256];
struct ovr { uint8_t addr, reg, and_mask, or_mask; };
static struct ovr ovrs[32];
static int novrs;

static uint32_t xtal = 28800000;
static struct e4k_state e4k;
static void *dev = &e4k; /* any non-NULL pointer: the drivers only pass it back to the I2C glue */

static void reset_model(void)
{
	memset(regs, 0, sizeof(regs));
	memset(ptr, 0, sizeof(ptr));
	memset(&e4k, 0, sizeof(e4k));
	novrs = 0;
	xtal = 28800000;
}

static uint8_t model_read(uint8_t addr, uint8_t reg)
{
	uint8_t v = regs[addr][reg];
	int i;

	for (i = 0; i < novrs; i++)
		if (ovrs[i].addr == addr && ovrs[i].reg == reg)
			v = (v & ovrs[i].and_mask) | ovrs[i].or_mask;
	return v;
}

int rtlsdr_i2c_write_fn(void *d, uint8_t addr, uint8_t *buf, int len)
{
	int i;

	if (!d)
		return -1;
	printf("W %02x:", addr);
	for (i = 0; i < len; i++)
		printf(" %02x", buf[i]);
	printf("\n");
	ptr[addr] = buf[0];
	for (i = 1; i < len; i++)
		regs[addr][(uint8_t)(buf[0] + i - 1)] = buf[i];
	return len;
}

int rtlsdr_i2c_read_fn(void *d, uint8_t addr, uint8_t *buf, int len)
{
	int i;

	if (!d)
		return -1;
	printf("R %02x:", addr);
	for (i = 0; i < len; i++) {
		buf[i] = model_read(addr, (uint8_t)(ptr[addr] + i));
		printf(" %02x", buf[i]);
	}
	printf("\n");
	return len;
}

uint32_t rtlsdr_get_tuner_clock(void *d)
{
	return d ? xtal : 0;
}

static void set_gpio_bit(uint8_t gpio, int val)
{
	printf("G %u %d\n", gpio, val ? 1 : 0);
}

/* librtlsdr.c's tuner glue */

static int e4000_init(void)
{
	e4k.i2c_addr = E4K_I2C_ADDR;
	e4k.vco.fosc = xtal;
	e4k.rtl_dev = dev;
	return e4k_init(&e4k);
}
static int e4000_exit(void) { return e4k_standby(&e4k, 1); }
static int e4000_set_freq(uint32_t freq) { return e4k_tune_freq(&e4k, freq); }
static int e4000_set_bw(int bw)
{
	int r = 0;

	r |= e4k_if_filter_bw_set(&e4k, E4K_IF_FILTER_MIX, bw);
	r |= e4k_if_filter_bw_set(&e4k, E4K_IF_FILTER_RC, bw);
	r |= e4k_if_filter_bw_set(&e4k, E4K_IF_FILTER_CHAN, bw);
	return r;
}
static int e4000_set_gain(int gain)
{
	int mixgain = (gain > 340) ? 12 : 4;

	if (e4k_set_lna_gain(&e4k, min(300, gain - mixgain * 10)) == -EINVAL)
		return -1;
	if (e4k_mixer_gain_set(&e4k, mixgain) == -EINVAL)
		return -1;
	return 0;
}
static int e4000_set_gain_mode(int manual) { return e4k_enable_manual_gain(&e4k, manual); }

static int fc0012_set_freq(uint32_t freq)
{
	/* select V-band/U-band filter */
	set_gpio_bit(6, (freq > 300000000) ? 1 : 0);
	return fc0012_set_params(dev, freq, 6000000);
}
static int fc0013_set_freq(uint32_t freq) { return fc0013_set_params(dev, freq, 6000000); }
static int fc2580_set_bw(int bw) { (void)bw; return fc2580_SetBandwidthMode(dev, 1); }

enum tuner { NONE, E4000, FC0012, FC0013, FC2580 };

static int run(enum tuner t, const char *op, long arg)
{
	if (!strcmp(op, "init")) {
		switch (t) {
		case E4000: return e4000_init();
		case FC0012: return fc0012_init(dev);
		case FC0013: return fc0013_init(dev);
		case FC2580: return fc2580_Initialize(dev);
		default: return -1;
		}
	}
	if (!strcmp(op, "exit"))
		return t == E4000 ? e4000_exit() : 0;
	if (!strcmp(op, "freq")) {
		switch (t) {
		case E4000: return e4000_set_freq((uint32_t)arg);
		case FC0012: return fc0012_set_freq((uint32_t)arg);
		case FC0013: return fc0013_set_freq((uint32_t)arg);
		case FC2580: return fc2580_SetRfFreqHz(dev, (unsigned long)arg);
		default: return -1;
		}
	}
	if (!strcmp(op, "bw")) {
		switch (t) {
		case E4000: return e4000_set_bw((int)arg);
		case FC2580: return fc2580_set_bw((int)arg);
		default: return 0;
		}
	}
	if (!strcmp(op, "gain")) {
		switch (t) {
		case E4000: return e4000_set_gain((int)arg);
		case FC0012: return fc0012_set_gain(dev, (int)arg);
		case FC0013: return fc0013_set_lna_gain(dev, (int)arg);
		default: return 0;
		}
	}
	if (!strcmp(op, "gainmode")) {
		switch (t) {
		case E4000: return e4000_set_gain_mode((int)arg);
		case FC0013: return fc0013_set_gain_mode(dev, (int)arg);
		default: return 0;
		}
	}
	fprintf(stderr, "unknown operation %s\n", op);
	exit(2);
}

int main(int argc, char **argv)
{
	char line[256], op[32], name[32];
	unsigned a, r, v, w;
	long arg;
	enum tuner t = NONE;
	FILE *f;

	if (argc != 2 || !(f = fopen(argv[1], "r"))) {
		fprintf(stderr, "usage: %s <scenario>\n", argv[0]);
		return 2;
	}
	while (fgets(line, sizeof(line), f)) {
		char *hash = strchr(line, '#');

		if (hash)
			*hash = 0;
		line[strcspn(line, "\r\n")] = 0;
		while (*line && line[strlen(line) - 1] == ' ')
			line[strlen(line) - 1] = 0;
		if (sscanf(line, "%31s", op) != 1)
			continue;
		printf("> %s\n", line);
		if (!strcmp(op, "tuner") && sscanf(line, "%*s %31s", name) == 1) {
			reset_model();
			t = !strcmp(name, "e4000") ? E4000 : !strcmp(name, "fc0012") ? FC0012 :
			    !strcmp(name, "fc0013") ? FC0013 : !strcmp(name, "fc2580") ? FC2580 : NONE;
			if (t == NONE) {
				fprintf(stderr, "unknown tuner %s\n", name);
				return 2;
			}
		} else if (!strcmp(op, "xtal") && sscanf(line, "%*s %ld", &arg) == 1) {
			xtal = (uint32_t)arg;
		} else if (!strcmp(op, "reg") && sscanf(line, "%*s %x %x %x", &a, &r, &v) == 3) {
			regs[a & 0xff][r & 0xff] = (uint8_t)v;
		} else if (!strcmp(op, "ovr") && sscanf(line, "%*s %x %x %x %x", &a, &r, &v, &w) == 4) {
			struct ovr o = { (uint8_t)a, (uint8_t)r, (uint8_t)v, (uint8_t)w };
			int i;

			for (i = 0; i < novrs; i++)
				if (ovrs[i].addr == o.addr && ovrs[i].reg == o.reg)
					break;
			if (i == novrs)
				novrs++;
			ovrs[i] = o;
		} else {
			arg = 0;
			sscanf(line, "%*s %ld", &arg);
			printf("= %s\n", run(t, op, arg) ? "err" : "ok");
		}
	}
	return 0;
}
