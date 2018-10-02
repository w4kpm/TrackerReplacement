#! /bin/bash
mv FrontPanel-B.Cu.gbr FrontPanel.GBL
mv FrontPanel-F.Cu.gbr FrontPanel.GTL
mv FrontPanel-F.Mask.gbr FrontPanel.GTS
mv FrontPanel-B.Mask.gbr FrontPanel.GBS
mv FrontPanel-F.SilkS.gbr FrontPanel.GTO
mv FrontPanel-B.SilkS.gbr FrontPanel.GBO
mv FrontPanel-Edge.Cuts.gbr FrontPanel.GML
mv FrontPanel.drl FrontPanel.TXT
zip FrontPanelCompiled.zip FrontPanel.*
