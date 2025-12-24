# Copyright 2023 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51
#
# Paul Scheffler <paulsc@iis.ee.ethz.ch>

GIT ?= git
BENDER ?= bender
VSIM ?= vsim

all: build run

clean: sim_clean

# Ensure half-built targets are purged
.DELETE_ON_ERROR:

ifdef gui
VSIM_ARGS := -do
else
VSIM_ARGS := -c -do
endif

# --------------
# RTL SIMULATION
# --------------

VLOG_ARGS += -suppress vlog-2583 -suppress vlog-13314 -suppress vlog-13233 -timescale \"1 ns / 1 ps\"
XVLOG_ARGS += -64bit -compile -vtimescale 1ns/1ns -quiet

define generate_vsim
	echo 'set ROOT [file normalize [file dirname [info script]]/$3]' > $1
	bender script $(VSIM) --vlog-arg="$(VLOG_ARGS)" $2 | grep -v "set ROOT" >> $1
	echo >> $1
endef

sim_all: scripts/compile.tcl

sim_clean:
	rm -rf scripts/compile.tcl
	rm -rf work

# Download (partially non-free) simulation models from publically available sources;
# by running these targets or targets depending on them, you accept this (see README.md).
S27KS_ZIP ?= $(CURDIR)/model.zip

models/s27ks0641:
	mkdir -p $@
	rm -rf model_tmp && mkdir -p model_tmp
	@if [ ! -f "$(S27KS_ZIP)" ]; then \
		echo "ERROR: Missing zip: $(S27KS_ZIP)"; \
		echo "Place it at ./model.zip or run: make $@ S27KS_ZIP=/path/to/zip"; \
		exit 1; \
	fi
	cp "$(S27KS_ZIP)" model_tmp/model.zip
	cd model_tmp; unzip -q model.zip

	# Your zip already contains model/ directly
	@if [ ! -f model_tmp/model/s27ks0641.v ]; then \
		echo "ERROR: model_tmp/model/s27ks0641.v not found. Contents:"; \
		find model_tmp -maxdepth 3 -type f | head; \
		exit 1; \
	fi
	cp model_tmp/model/s27ks0641.v $@
	@if [ -f model_tmp/model/s27ks0641_verilog.sdf ]; then \
		cp model_tmp/model/s27ks0641_verilog.sdf $@/s27ks0641.sdf; \
	elif [ -f model_tmp/model/s27ks0641.sdf ]; then \
		cp model_tmp/model/s27ks0641.sdf $@/s27ks0641.sdf; \
	else \
		echo "WARNING: no SDF found under model_tmp/model/"; \
	fi
	rm -rf model_tmp

scripts/compile.tcl: Bender.yml models/s27ks0641
	$(call generate_vsim, $@, -t rtl -t test -t hyper_test,..)

build: scripts/compile.tcl
	$(VSIM) -c -do "source scripts/compile.tcl; exit"

run: clean build
	$(VSIM) $(VSIM_ARGS) "source scripts/start.tcl"
