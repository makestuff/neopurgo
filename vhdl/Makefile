TOP_LEVEL = TopLevel
PLATFORM = s3board
VHDLS = \
	SevenSeg.vhdl

ifeq ($(PLATFORM),s3board)
	FPGA = xc3s200-ft256-4
endif

ifeq ($(OS),Windows_NT)
	XSVF2CSVF = ../xsvf2csvf/Release/xsvf2csvf.exe
else
	XSVF2CSVF = ../xsvf2csvf/xsvf2csvf
endif

all: $(TOP_LEVEL).csvf

xsvf: $(TOP_LEVEL).xsvf

csvf: $(TOP_LEVEL).csvf

report: $(TOP_LEVEL).twr

$(TOP_LEVEL).csvf: $(TOP_LEVEL).xsvf $(XSVF2CSVF)
	$(XSVF2CSVF) $< $@

# This assumes that the "XILINX" environment variable is set 
$(TOP_LEVEL).xsvf: $(PLATFORM).batch $(TOP_LEVEL).bit
	cat $< | sed s#\$${XILINX}#$(subst \,/,$(XILINX))#g > temp.batch
	impact -batch temp.batch
	rm -f temp.batch

$(TOP_LEVEL).bit: $(TOP_LEVEL).ut $(TOP_LEVEL).ncd
	bitgen -intstyle ise -f $+

$(TOP_LEVEL).twr: $(TOP_LEVEL).ncd $(PLATFORM).ucf
	trce -intstyle ise -v 3 -s 4 -n 3 -fastpaths -xml ${TOP_LEVEL}.twx $< -o $@ ${TOP_LEVEL}.pcf -ucf $(PLATFORM).ucf

$(TOP_LEVEL).ncd: $(TOP_LEVEL)_map.ncd
	par -w -intstyle ise -ol high -t 1 $< $@ ${TOP_LEVEL}.pcf

$(TOP_LEVEL)_map.ncd: $(TOP_LEVEL).ngd
	map -intstyle ise -p xc3s200-ft256-4 -cm area -ir off -pr off -c 100 -o $@ $< ${TOP_LEVEL}.pcf

$(TOP_LEVEL).ngd: $(TOP_LEVEL).ngc $(PLATFORM).ucf
	ngdbuild -intstyle ise -dd _ngo -nt timestamp -uc $(PLATFORM).ucf -p $(FPGA) $< $@

$(TOP_LEVEL).ngc: $(PLATFORM).xst $(TOP_LEVEL).prj
	mkdir -p xst/projnav.tmp
	xst -intstyle ise -ifn $< -ofn $(TOP_LEVEL).syr

$(TOP_LEVEL).prj: $(VHDLS)
	for i in $(TOP_LEVEL).vhdl $+; do echo "vhdl work \"$$i\""; done > $@

clean: FORCE
	rm -rf *.xsvf *.csvf _ngo *.bgn *.drc *.ncd *.ntrc_log *.prj *.twr *.csv *.html fx2fpga_xdb _xmsgs *.bit *.gise *.ngc *.pad *.ptwx *.twx *.ngm *.txt *.xml *.xrpt *.bld *.ise *.ngd *.par *.stx *.map *.twr auto_project_xdb *.cmd_log *.lso *.ngr *.pcf *.syr *.unroutes *.xpi *.mrp xst *.log *.cmd *.xwbt iseconfig xlnx_auto_0_xdb

FORCE:
