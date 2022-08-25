# Tracker replacement #

ChibiOS - A linked repo for the embedded OS
Firmware - Firmware code for the back panel
FrontPanelFirmware - Firmware code for the front panel
KiCad - all board design files using OSS KiCad
Testing - python files used for original testing of the board
TrackerControlCode - code use in production for updating tracker position and for resetting errors remotely. 


Included in this repo:
   * Board design files for user interface panel and tracker controller
   * Firmware for both interface panel and tracker controller
   * Examples of software used to control, reset and poll the tracker.
   * In KiCad/TrackerBackPanel and KiCad/TrackerFrontPanel are directories for gerber for board production
   * in KiCad/TrackerBackPanel/BackPanel.csv and KiCad/TrackerFrontPanel/FrontPanel.csv are parts lists
   * pos files are also there for SMT parts
   * back panel used polou VNH5019 motor driver board Item #1451 to control the motor