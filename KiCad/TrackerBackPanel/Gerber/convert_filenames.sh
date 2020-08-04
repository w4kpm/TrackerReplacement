#! /bin/bash
mv BackPanel-B_Cu.gbr BackPanel.GBL
mv BackPanel-F_Cu.gbr BackPanel.GTL
mv BackPanel-F_Mask.gbr BackPanel.GTS
mv BackPanel-B_Mask.gbr BackPanel.GBS
mv BackPanel-F_SilkS.gbr BackPanel.GTO
mv BackPanel-B_SilkS.gbr BackPanel.GBO
mv BackPanel-Edge_Cuts.gbr BackPanel.GML
mv BackPanel.drl BackPanel.TXT
zip BackPanelCompiled.zip BackPanel.*
