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
Sheet 1 4
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
L Conn_01x06 J?
U 1 1 5B9BD506
P 5200 7300
F 0 "J?" H 5200 7600 50  0000 C CNN
F 1 "Conn_01x06" H 5200 6900 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 5200 7300 50  0001 C CNN
F 3 "" H 5200 7300 50  0001 C CNN
	1    5200 7300
	0    1    1    0   
$EndComp
$Comp
L Conn_02x04_Odd_Even J?
U 1 1 5B9BD507
P 3700 1000
F 0 "J?" H 3750 1200 50  0000 C CNN
F 1 "Conn_02x04_Odd_Even" H 3750 700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_2x04_Pitch2.54mm" H 3700 1000 50  0001 C CNN
F 3 "" H 3700 1000 50  0001 C CNN
	1    3700 1000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5B9BD508
P 4200 900
F 0 "#PWR?" H 4200 650 50  0001 C CNN
F 1 "GND" H 4200 750 50  0000 C CNN
F 2 "" H 4200 900 50  0001 C CNN
F 3 "" H 4200 900 50  0001 C CNN
	1    4200 900 
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5B9BD509
P 4200 1000
F 0 "#PWR?" H 4200 850 50  0001 C CNN
F 1 "+3.3V" H 4200 1140 50  0000 C CNN
F 2 "" H 4200 1000 50  0001 C CNN
F 3 "" H 4200 1000 50  0001 C CNN
	1    4200 1000
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5B9BD50A
P 5400 6900
F 0 "#PWR?" H 5400 6650 50  0001 C CNN
F 1 "GND" H 5400 6750 50  0000 C CNN
F 2 "" H 5400 6900 50  0001 C CNN
F 3 "" H 5400 6900 50  0001 C CNN
	1    5400 6900
	-1   0    0    1   
$EndComp
NoConn ~ 5300 7100
NoConn ~ 5200 7100
NoConn ~ 4900 7100
Wire Wire Line
	4000 900  4200 900 
Wire Wire Line
	4000 1000 4200 1000
Wire Wire Line
	3500 1100 3250 1100
Wire Wire Line
	3250 1100 3250 1400
Wire Wire Line
	3250 1400 4800 1400
Wire Wire Line
	4800 1400 4800 1100
Wire Wire Line
	4800 1100 6100 1100
Wire Wire Line
	3500 900  3500 750 
Wire Wire Line
	3500 750  4850 750 
Wire Wire Line
	4850 750  4850 1000
Wire Wire Line
	4850 1000 6100 1000
Wire Wire Line
	5400 7100 5400 6900
Wire Wire Line
	5100 7100 5100 5000
Wire Wire Line
	5100 5000 6200 5000
Wire Wire Line
	5000 7100 5000 5100
Wire Wire Line
	5000 5100 6200 5100
Text Label 5450 2100 0    60   ~ 0
RX
Text Label 5750 2200 0    60   ~ 0
TX
$Comp
L Conn_01x08 J?
U 1 1 5B9BD50E
P 1200 2200
F 0 "J?" H 1200 2600 50  0000 C CNN
F 1 "Conn_01x08" H 1200 1700 50  0000 C CNN
F 2 "footprints:RJ45" H 1200 2200 50  0001 C CNN
F 3 "" H 1200 2200 50  0001 C CNN
	1    1200 2200
	-1   0    0    1   
$EndComp
Text Notes 600  1550 0    60   ~ 0
NB: use only Straight through Ethernet cable-
$Comp
L GND #PWR?
U 1 1 5B9BD50F
P 1600 1850
F 0 "#PWR?" H 1600 1600 50  0001 C CNN
F 1 "GND" H 1600 1700 50  0000 C CNN
F 2 "" H 1600 1850 50  0001 C CNN
F 3 "" H 1600 1850 50  0001 C CNN
	1    1600 1850
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5B9BD510
P 1600 2450
F 0 "#PWR?" H 1600 2300 50  0001 C CNN
F 1 "+3.3V" H 1600 2590 50  0000 C CNN
F 2 "" H 1600 2450 50  0001 C CNN
F 3 "" H 1600 2450 50  0001 C CNN
	1    1600 2450
	0    1    1    0   
$EndComp
Wire Wire Line
	1400 1800 1600 1800
Wire Wire Line
	1600 1800 1600 1900
Wire Wire Line
	1600 1900 1400 1900
Connection ~ 1600 1850
Wire Wire Line
	1400 2400 1600 2400
Wire Wire Line
	1600 2400 1600 2500
Wire Wire Line
	1600 2500 1400 2500
Connection ~ 1600 2450
$Sheet
S 900  3300 1100 950 
U 5B9BD512
F0 "pwr" 60
F1 "file5B9BD512.sch" 60
$EndSheet
$Comp
L STM32F303V(B-C)Tx_u U?
U 1 1 5B9BD513
P 11400 1600
F 0 "U?" H 11400 1700 50  0000 C CNN
F 1 "STM32F303V(B-C)Tx_u" H 11400 1500 50  0000 C CNN
F 2 "Housings_QFP:LQFP-100_14x14mm_Pitch0.5mm" H 11400 1400 50  0001 C CIN
F 3 "" H 11400 1600 50  0000 C CNN
	1    11400 1600
	-1   0    0    1   
$EndComp
$Comp
L STM32F303V(B-C)Tx_u U?
U 2 1 5B9BD514
P 9500 4900
F 0 "U?" H 9500 5000 50  0000 C CNN
F 1 "STM32F303V(B-C)Tx_u" H 9500 4800 50  0000 C CNN
F 2 "Housings_QFP:LQFP-100_14x14mm_Pitch0.5mm" H 9500 4700 50  0001 C CIN
F 3 "" H 9500 4900 50  0000 C CNN
	2    9500 4900
	-1   0    0    1   
$EndComp
NoConn ~ 6200 5700
NoConn ~ 6200 5600
NoConn ~ 6200 5500
NoConn ~ 6200 5400
NoConn ~ 6200 5300
NoConn ~ 6200 5200
NoConn ~ 6200 4900
NoConn ~ 6200 4300
NoConn ~ 6100 2400
NoConn ~ 6100 2300
NoConn ~ 6100 2000
NoConn ~ 6100 1900
NoConn ~ 6100 1800
NoConn ~ 6100 1700
NoConn ~ 6100 1600
NoConn ~ 6100 1500
NoConn ~ 6100 1400
NoConn ~ 6100 1300
NoConn ~ 6100 1200
NoConn ~ 6100 900 
NoConn ~ 5450 3300
NoConn ~ 1400 2300
NoConn ~ 1400 2000
NoConn ~ 3500 1000
NoConn ~ 3500 1200
NoConn ~ 4000 1100
NoConn ~ 4000 1200
Wire Wire Line
	1400 2100 4600 2100
Wire Wire Line
	4600 2100 4600 2200
Wire Wire Line
	4600 2200 6100 2200
Wire Wire Line
	1400 2200 3200 2200
Wire Wire Line
	3200 2200 3200 1950
Wire Wire Line
	3200 1950 5100 1950
Wire Wire Line
	5100 1950 5100 2100
Wire Wire Line
	5100 2100 6100 2100
$Sheet
S 950  6100 1050 850 
U 5B9C0C81
F0 "C-D" 60
F1 "C-D.sch" 60
$EndSheet
$EndSCHEMATC
