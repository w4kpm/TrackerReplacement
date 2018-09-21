EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:stm32
LIBS:MyParts
LIBS:FrontPanel-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L STM32F303V(B-C)Tx_u U?
U 3 1 5B9C0C94
P 8300 1600
F 0 "U?" H 8300 1700 50  0000 C CNN
F 1 "STM32F303V(B-C)Tx_u" H 8300 1500 50  0000 C CNN
F 2 "LQFP100" H 8300 1400 50  0000 C CIN
F 3 "" H 8300 1600 50  0000 C CNN
	3    8300 1600
	-1   0    0    1   
$EndComp
$Comp
L STM32F303V(B-C)Tx_u U?
U 4 1 5B9C0D45
P 8300 3450
F 0 "U?" H 8300 3550 50  0000 C CNN
F 1 "STM32F303V(B-C)Tx_u" H 8300 3350 50  0000 C CNN
F 2 "LQFP100" H 8300 3250 50  0000 C CIN
F 3 "" H 8300 3450 50  0000 C CNN
	4    8300 3450
	-1   0    0    1   
$EndComp
Wire Wire Line
	4450 3350 5800 3350
Wire Wire Line
	4200 1300 5800 1300
Wire Wire Line
	4300 1400 5800 1400
$Sheet
S 2100 1500 1650 1350
U 5B9C174F
F0 "serial" 60
F1 "serial.sch" 60
F2 "TX_en" I R 3750 2350 60 
F3 "TX" I R 3750 2750 60 
F4 "RX" I R 3750 2550 60 
F5 "RS485B(+)" O L 2100 1850 60 
F6 "RS485A(-)" O L 2100 2000 60 
F7 "AngleTX_en" I R 3750 1600 60 
F8 "AngleTX" I R 3750 1900 60 
F9 "AngleRX" I R 3750 1750 60 
F10 "AngleRS485B(+)" O L 2100 2400 60 
F11 "AngleRS485A(-)" O L 2100 2550 60 
$EndSheet
Wire Wire Line
	5800 3450 4350 3450
Wire Wire Line
	4050 1200 5800 1200
Wire Wire Line
	4350 3450 4350 2750
Wire Wire Line
	4350 2750 3750 2750
Wire Wire Line
	4450 3350 4450 2550
Wire Wire Line
	4450 2550 3750 2550
Wire Wire Line
	5800 3250 4550 3250
Wire Wire Line
	4550 3250 4550 2350
Wire Wire Line
	4550 2350 3750 2350
Wire Wire Line
	4050 1200 4050 1600
Wire Wire Line
	4050 1600 3750 1600
Wire Wire Line
	4200 1300 4200 1750
Wire Wire Line
	4200 1750 3750 1750
Wire Wire Line
	4300 1400 4300 1900
Wire Wire Line
	4300 1900 3750 1900
$Comp
L Conn_01x04 J?
U 1 1 5B9C45D1
P 750 2650
F 0 "J?" H 750 2850 50  0000 C CNN
F 1 "Conn_01x04" H 750 2350 50  0000 C CNN
F 2 "" H 750 2650 50  0001 C CNN
F 3 "" H 750 2650 50  0001 C CNN
	1    750  2650
	-1   0    0    1   
$EndComp
$Comp
L +24V #PWR?
U 1 1 5B9C47A5
P 1500 2750
F 0 "#PWR?" H 1500 2600 50  0001 C CNN
F 1 "+24V" H 1500 2890 50  0000 C CNN
F 2 "" H 1500 2750 50  0001 C CNN
F 3 "" H 1500 2750 50  0001 C CNN
	1    1500 2750
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5B9C48A0
P 1200 1400
F 0 "#PWR?" H 1200 1150 50  0001 C CNN
F 1 "GND" H 1200 1250 50  0000 C CNN
F 2 "" H 1200 1400 50  0001 C CNN
F 3 "" H 1200 1400 50  0001 C CNN
	1    1200 1400
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5B9C48FA
P 1200 2450
F 0 "#PWR?" H 1200 2200 50  0001 C CNN
F 1 "GND" H 1200 2300 50  0000 C CNN
F 2 "" H 1200 2450 50  0001 C CNN
F 3 "" H 1200 2450 50  0001 C CNN
	1    1200 2450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2100 1850 1900 1850
Wire Wire Line
	1900 1850 1900 1500
Wire Wire Line
	1900 1500 1000 1500
Wire Wire Line
	2100 2000 1850 2000
Wire Wire Line
	1850 2000 1850 1600
Wire Wire Line
	1850 1600 1000 1600
Wire Wire Line
	2100 2550 1900 2550
Wire Wire Line
	1900 2550 1900 2650
Wire Wire Line
	1900 2650 950  2650
Wire Wire Line
	2100 2400 1800 2400
Wire Wire Line
	1800 2400 1800 2550
Wire Wire Line
	1800 2550 950  2550
Wire Wire Line
	1200 1400 1000 1400
Wire Wire Line
	1200 2450 950  2450
Wire Wire Line
	950  2750 1500 2750
Text Notes 650  950  0    60   ~ 0
NB!!!!  - These may be totally wrong compared to actual pinouts of devices\nPlease check . . .
$Comp
L Conn_01x04 J?
U 1 1 5B9C4CC0
P 750 3450
F 0 "J?" H 750 3650 50  0000 C CNN
F 1 "Conn_01x04" H 750 3150 50  0000 C CNN
F 2 "" H 750 3450 50  0001 C CNN
F 3 "" H 750 3450 50  0001 C CNN
	1    750  3450
	-1   0    0    1   
$EndComp
Wire Wire Line
	1000 2450 1000 3250
Wire Wire Line
	1000 3250 950  3250
Connection ~ 1000 2450
Wire Wire Line
	1100 2550 1100 3350
Wire Wire Line
	1100 3350 950  3350
Connection ~ 1100 2550
Wire Wire Line
	1150 2650 1150 3450
Wire Wire Line
	1150 3450 950  3450
Connection ~ 1150 2650
Wire Wire Line
	1200 2750 1200 3550
Wire Wire Line
	1200 3550 950  3550
$Comp
L Conn_01x03 J?
U 1 1 5B9C5204
P 800 1500
F 0 "J?" H 800 1700 50  0000 C CNN
F 1 "Conn_01x03" H 800 1300 50  0000 C CNN
F 2 "" H 800 1500 50  0001 C CNN
F 3 "" H 800 1500 50  0001 C CNN
	1    800  1500
	-1   0    0    1   
$EndComp
Connection ~ 1200 2750
$EndSCHEMATC
