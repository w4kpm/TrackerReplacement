EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
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
L BackPanel-rescue:Conn_01x04 J8
U 1 1 5B9C45D1
P 750 2650
F 0 "J8" H 750 2850 50  0000 C CNN
F 1 "Conn_01x04" H 750 2350 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-4_P5.00mm" H 750 2650 50  0001 C CNN
F 3 "" H 750 2650 50  0001 C CNN
	1    750  2650
	-1   0    0    1   
$EndComp
Wire Wire Line
	2100 2550 1900 2550
Wire Wire Line
	2100 2400 1800 2400
$Comp
L BackPanel-rescue:Conn_01x04 J9
U 1 1 5B9C4CC0
P 750 3450
F 0 "J9" H 750 3650 50  0000 C CNN
F 1 "Conn_01x04" H 750 3150 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-4_P5.00mm" H 750 3450 50  0001 C CNN
F 3 "" H 750 3450 50  0001 C CNN
	1    750  3450
	-1   0    0    1   
$EndComp
Wire Wire Line
	1000 2450 1000 3000
Wire Wire Line
	1000 3250 950  3250
Wire Wire Line
	1000 2450 950  2450
$Comp
L Connector_Generic:Conn_01x02 J4
U 1 1 5BA2AFF3
P 750 850
F 0 "J4" H 670 525 50  0000 C CNN
F 1 "Conn_01x02" H 670 616 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-2_P5.00mm" H 750 850 50  0001 C CNN
F 3 "~" H 750 850 50  0001 C CNN
	1    750  850 
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J6
U 1 1 5BA2B06E
P 750 1650
F 0 "J6" H 670 1325 50  0000 C CNN
F 1 "Conn_01x02" H 670 1416 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-2_P5.00mm" H 750 1650 50  0001 C CNN
F 3 "~" H 750 1650 50  0001 C CNN
	1    750  1650
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J7
U 1 1 5BA2B0AE
P 750 2000
F 0 "J7" H 670 1675 50  0000 C CNN
F 1 "Conn_01x02" H 670 1766 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-2_P5.00mm" H 750 2000 50  0001 C CNN
F 3 "~" H 750 2000 50  0001 C CNN
	1    750  2000
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J5
U 1 1 5BA2BE12
P 750 1300
F 0 "J5" H 670 975 50  0000 C CNN
F 1 "Conn_01x02" H 670 1066 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-2_P5.00mm" H 750 1300 50  0001 C CNN
F 3 "~" H 750 1300 50  0001 C CNN
	1    750  1300
	-1   0    0    1   
$EndComp
Wire Wire Line
	950  1550 950  1600
Wire Wire Line
	950  1900 950  2000
Wire Wire Line
	1450 2000 2100 2000
Wire Wire Line
	1700 1850 2100 1850
$Comp
L power:GND #PWR013
U 1 1 5BA30707
P 1100 1600
F 0 "#PWR013" H 1100 1350 50  0001 C CNN
F 1 "GND" V 1105 1472 50  0000 R CNN
F 2 "" H 1100 1600 50  0001 C CNN
F 3 "" H 1100 1600 50  0001 C CNN
	1    1100 1600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1100 1600 950  1600
Connection ~ 950  1600
Wire Wire Line
	950  1600 950  1650
Text Label 1050 1600 0    50   ~ 0
RS485C
Text Notes 1000 1950 0    50   ~ 0
Shield - NC\n
Wire Wire Line
	950  2550 1100 2550
Wire Wire Line
	1100 2550 1100 3100
Wire Wire Line
	1100 3350 950  3350
Wire Wire Line
	950  3450 1200 3450
Wire Wire Line
	1200 3450 1200 3200
Wire Wire Line
	1200 2650 950  2650
Wire Wire Line
	950  2750 1300 2750
Wire Wire Line
	1300 2750 1300 3300
Wire Wire Line
	1300 3550 950  3550
Wire Wire Line
	1900 2550 1900 3000
Wire Wire Line
	1900 3000 1000 3000
Connection ~ 1000 3000
Wire Wire Line
	1000 3000 1000 3250
Wire Wire Line
	1800 2400 1800 3100
Wire Wire Line
	1800 3100 1100 3100
Connection ~ 1100 3100
Wire Wire Line
	1100 3100 1100 3350
$Comp
L power:+24V #PWR014
U 1 1 5BA36855
P 1800 3200
F 0 "#PWR014" H 1800 3050 50  0001 C CNN
F 1 "+24V" V 1815 3328 50  0000 L CNN
F 2 "" H 1800 3200 50  0001 C CNN
F 3 "" H 1800 3200 50  0001 C CNN
	1    1800 3200
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5BA36901
P 1800 3300
F 0 "#PWR015" H 1800 3050 50  0001 C CNN
F 1 "GND" V 1805 3172 50  0000 R CNN
F 2 "" H 1800 3300 50  0001 C CNN
F 3 "" H 1800 3300 50  0001 C CNN
	1    1800 3300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1800 3200 1200 3200
Connection ~ 1200 3200
Wire Wire Line
	1200 3200 1200 2650
Wire Wire Line
	1800 3300 1300 3300
Connection ~ 1300 3300
Wire Wire Line
	1300 3300 1300 3550
Text Notes 800  4050 0    50   ~ 0
Gnd should be to the left on these - match with wire on angle sensor\n
NoConn ~ 5800 900 
NoConn ~ 5800 1000
NoConn ~ 5800 1100
NoConn ~ 5800 1500
NoConn ~ 5800 1600
NoConn ~ 5800 1700
NoConn ~ 5800 1800
NoConn ~ 5800 1900
NoConn ~ 5800 2000
NoConn ~ 5800 2100
NoConn ~ 5800 2200
NoConn ~ 5800 2300
NoConn ~ 5800 2400
NoConn ~ 5800 2750
NoConn ~ 5800 2850
NoConn ~ 5800 2950
NoConn ~ 5800 3050
NoConn ~ 5800 3150
$Comp
L stm32:STM32F303V(B-C)Tx_u U1
U 3 1 5BA554D3
P 8300 1600
F 0 "U1" H 5921 1559 50  0000 R CNN
F 1 "STM32F303V(B-C)Tx_u" H 5921 1650 50  0000 R CNN
F 2 "Package_QFP:LQFP-100_14x14mm_P0.5mm" H 5921 1741 50  0000 R CIN
F 3 "" H 8300 1600 50  0000 C CNN
	3    8300 1600
	-1   0    0    1   
$EndComp
$Comp
L stm32:STM32F303V(B-C)Tx_u U1
U 4 1 5BA555EF
P 8300 3450
F 0 "U1" H 5671 3409 50  0000 R CNN
F 1 "STM32F303V(B-C)Tx_u" H 5671 3500 50  0000 R CNN
F 2 "Package_QFP:LQFP-100_14x14mm_P0.5mm" H 5671 3591 50  0000 R CIN
F 3 "" H 8300 3450 50  0000 C CNN
	4    8300 3450
	-1   0    0    1   
$EndComp
$Comp
L MyParts:SW-DIP-8 SW1
U 1 1 5BA44F0E
P 4600 4550
F 0 "SW1" H 4600 3813 60  0000 C CNN
F 1 "SW-DIP-8" H 4600 3919 60  0000 C CNN
F 2 "Button_Switch_SMD:SW_DIP_SPSTx08_Slide_6.7x21.88mm_W8.61mm_P2.54mm_LowProfile" H 4600 4550 60  0001 C CNN
F 3 "" H 4600 4550 60  0000 C CNN
	1    4600 4550
	-1   0    0    1   
$EndComp
Wire Wire Line
	5800 3550 5200 3550
Wire Wire Line
	5200 3550 5200 4100
Wire Wire Line
	5200 4100 5100 4100
Wire Wire Line
	5100 4200 5250 4200
Wire Wire Line
	5250 4200 5250 3650
Wire Wire Line
	5250 3650 5800 3650
Wire Wire Line
	5800 3750 5300 3750
Wire Wire Line
	5300 3750 5300 4300
Wire Wire Line
	5300 4300 5100 4300
Wire Wire Line
	5800 3850 5350 3850
Wire Wire Line
	5350 3850 5350 4400
Wire Wire Line
	5350 4400 5100 4400
Wire Wire Line
	5800 3950 5400 3950
Wire Wire Line
	5400 3950 5400 4700
Wire Wire Line
	5400 4700 5100 4700
Wire Wire Line
	5800 4050 5450 4050
Wire Wire Line
	5450 4050 5450 4800
Wire Wire Line
	5450 4800 5100 4800
Wire Wire Line
	5800 4150 5500 4150
Wire Wire Line
	5500 4150 5500 4900
Wire Wire Line
	5500 4900 5100 4900
Wire Wire Line
	5800 4250 5550 4250
Wire Wire Line
	5550 4250 5550 5000
Wire Wire Line
	5550 5000 5100 5000
Wire Wire Line
	4100 4100 3900 4100
Wire Wire Line
	3900 4100 3900 4200
Wire Wire Line
	3900 5000 4100 5000
Wire Wire Line
	4100 4200 3900 4200
Connection ~ 3900 4200
Wire Wire Line
	3900 4200 3900 4300
Wire Wire Line
	4100 4300 3900 4300
Connection ~ 3900 4300
Wire Wire Line
	3900 4300 3900 4400
Wire Wire Line
	4100 4400 3900 4400
Connection ~ 3900 4400
Wire Wire Line
	3900 4400 3900 4550
Wire Wire Line
	4100 4700 3900 4700
Connection ~ 3900 4700
Wire Wire Line
	3900 4700 3900 4800
Wire Wire Line
	4100 4800 3900 4800
Connection ~ 3900 4800
Wire Wire Line
	3900 4800 3900 4900
Wire Wire Line
	4100 4900 3900 4900
Connection ~ 3900 4900
Wire Wire Line
	3900 4900 3900 5000
$Comp
L power:GND #PWR0102
U 1 1 5BA5FC8E
P 3600 4550
F 0 "#PWR0102" H 3600 4300 50  0001 C CNN
F 1 "GND" V 3605 4422 50  0000 R CNN
F 2 "" H 3600 4550 50  0001 C CNN
F 3 "" H 3600 4550 50  0001 C CNN
	1    3600 4550
	0    1    1    0   
$EndComp
Wire Wire Line
	3600 4550 3900 4550
Connection ~ 3900 4550
Wire Wire Line
	3900 4550 3900 4700
Wire Wire Line
	950  1200 950  1250
Wire Wire Line
	950  750  950  800 
Wire Wire Line
	1450 2000 1450 1250
Wire Wire Line
	1450 1250 950  1250
Connection ~ 950  1250
Wire Wire Line
	950  1250 950  1300
Wire Wire Line
	1700 800  950  800 
Wire Wire Line
	1700 800  1700 1850
Connection ~ 950  800 
Wire Wire Line
	950  800  950  850 
Text Label 1200 800  0    50   ~ 0
RS485A(-)
Text Label 1150 1250 0    50   ~ 0
RS485B(+)
Text Notes 2100 850  0    50   ~ 0
NB - These have been switched on purpose -\nindustry is labelled opposite of what the chip says.\n\n
$EndSCHEMATC
