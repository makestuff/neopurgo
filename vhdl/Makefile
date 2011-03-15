PROJ=fx2fpga

all: $(PROJ).bit

report: $(PROJ).twr

$(PROJ).bit: $(PROJ).ut $(PROJ).ncd
	/opt/Xilinx/12.3/ISE_DS/ISE/bin/lin/bitgen -intstyle ise -f $+

$(PROJ).twr: $(PROJ).ncd $(PROJ).ucf
	/opt/Xilinx/12.3/ISE_DS/ISE/bin/lin/trce -intstyle ise -v 3 -s 4 -n 3 -fastpaths -xml ${PROJ}.twx $< -o $@ ${PROJ}.pcf -ucf $(PROJ).ucf

$(PROJ).ncd: $(PROJ)_map.ncd
	/opt/Xilinx/12.3/ISE_DS/ISE/bin/lin/par -w -intstyle ise -ol high -t 1 $< $@ ${PROJ}.pcf

$(PROJ)_map.ncd: $(PROJ).ngd
	/opt/Xilinx/12.3/ISE_DS/ISE/bin/lin/map -intstyle ise -p xc3s200-ft256-4 -cm area -ir off -pr off -c 100 -o $@ $< ${PROJ}.pcf

$(PROJ).ngd: $(PROJ).ngc $(PROJ).ucf
	/opt/Xilinx/12.3/ISE_DS/ISE/bin/lin/ngdbuild -intstyle ise -dd _ngo -nt timestamp -uc $(PROJ).ucf -p xc3s200-ft256-4 $< $@

$(PROJ).ngc: $(PROJ).xst $(PROJ).prj
	mkdir -p xst/projnav.tmp
	/opt/Xilinx/12.3/ISE_DS/ISE/bin/lin/xst -intstyle ise -ifn ${PROJ}.xst -ofn ${PROJ}.syr

$(PROJ).prj: \
		top_level.vhdl \
		seven_seg.vhdl
	for i in $+; do echo "vhdl work \"$$i\""; done > $@

clean: FORCE
	rm -rf _ngo *.bgn *.drc *.ncd *.ntrc_log *.prj *.twr *.csv *.html fx2fpga_xdb _xmsgs *.bit *.gise *.ngc *.pad *.ptwx *.twx *.ngm *.txt *.xml *.xrpt *.bld *.ise *.ngd *.par *.stx *.map *.twr auto_project_xdb *.cmd_log *.lso *.ngr *.pcf *.syr *.unroutes *.xpi *.mrp xst *.log *.cmd *.xwbt iseconfig xlnx_auto_0_xdb

FORCE:
