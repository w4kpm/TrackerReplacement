EESchema Schematic File Version 4
LIBS:FrontPanel-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 3
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
L stm32:STM32F303V(B-C)Tx_u U1
U 8 1 5B96F745
P 3050 3600
F 0 "U1" H 2150 3800 50  0000 C CNN
F 1 "STM32F303V(B-C)Tx_u" H 2150 3650 50  0000 C CNN
F 2 "Housings_QFP:LQFP-100_14x14mm_Pitch0.5mm" H 2050 3500 50  0001 C CIN
F 3 "" H 3050 3600 50  0000 C CNN
	8    3050 3600
	1    0    0    -1  
$EndComp
$Comp
L stm32:STM32F303V(B-C)Tx_u U1
U 9 1 5B96F746
P 3050 2100
F 0 "U1" H 2250 2100 50  0000 C CNN
F 1 "STM32F303V(B-C)Tx_u" H 2350 2000 50  0000 C CNN
F 2 "Housings_QFP:LQFP-100_14x14mm_Pitch0.5mm" H 2250 1850 50  0001 C CIN
F 3 "" H 3050 2100 50  0000 C CNN
	9    3050 2100
	1    0    0    -1  
$EndComp
$Comp
L stm32:STM32F303V(B-C)Tx_u U1
U 7 1 5B96F747
P 3400 1050
F 0 "U1" H 2750 1150 50  0000 C CNN
F 1 "STM32F303V(B-C)Tx_u" H 2650 1000 50  0000 C CNN
F 2 "Housings_QFP:LQFP-100_14x14mm_Pitch0.5mm" H 2700 850 50  0001 C CIN
F 3 "" H 3400 1050 50  0000 C CNN
	7    3400 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5B96F748
P 3900 850
F 0 "#PWR011" H 3900 600 50  0001 C CNN
F 1 "GND" H 3900 700 50  0000 C CNN
F 2 "" H 3900 850 50  0001 C CNN
F 3 "" H 3900 850 50  0001 C CNN
	1    3900 850 
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR012
U 1 1 5B96F749
P 6550 3500
F 0 "#PWR012" H 6550 3250 50  0001 C CNN
F 1 "GND" H 6550 3350 50  0000 C CNN
F 2 "" H 6550 3500 50  0001 C CNN
F 3 "" H 6550 3500 50  0001 C CNN
	1    6550 3500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3650 3400 3750 3400
Wire Wire Line
	6550 3400 6550 3500
Wire Wire Line
	3650 3500 4300 3500
Wire Wire Line
	3650 3600 4900 3600
Connection ~ 6550 3500
Wire Wire Line
	6550 3700 5750 3700
Connection ~ 6550 3600
$Comp
L power:+3.3V #PWR013
U 1 1 5B96F74A
P 6500 2050
F 0 "#PWR013" H 6500 1900 50  0001 C CNN
F 1 "+3.3V" H 6500 2190 50  0000 C CNN
F 2 "" H 6500 2050 50  0001 C CNN
F 3 "" H 6500 2050 50  0001 C CNN
	1    6500 2050
	0    1    1    0   
$EndComp
Wire Wire Line
	3550 1900 6500 1900
Wire Wire Line
	6500 1900 6500 2000
Wire Wire Line
	3550 2000 3750 2000
Connection ~ 6500 2000
Wire Wire Line
	3550 2100 4300 2100
Connection ~ 6500 2050
Wire Wire Line
	3550 2200 4900 2200
Connection ~ 6500 2100
Wire Wire Line
	6500 2300 5750 2300
Connection ~ 6500 2200
NoConn ~ 3900 950 
$Comp
L power:+3.3V #PWR014
U 1 1 5B96F74B
P 4050 1100
F 0 "#PWR014" H 4050 950 50  0001 C CNN
F 1 "+3.3V" H 4050 1240 50  0000 C CNN
F 2 "" H 4050 1100 50  0001 C CNN
F 3 "" H 4050 1100 50  0001 C CNN
	1    4050 1100
	0    1    1    0   
$EndComp
Wire Wire Line
	3900 1050 4050 1050
Wire Wire Line
	4050 1050 4050 1100
Wire Wire Line
	4050 1150 3900 1150
Connection ~ 4050 1100
$Comp
L power:+3.3V #PWR015
U 1 1 5B96F74E
P 2950 4600
F 0 "#PWR015" H 2950 4450 50  0001 C CNN
F 1 "+3.3V" H 2950 4740 50  0000 C CNN
F 2 "" H 2950 4600 50  0001 C CNN
F 3 "" H 2950 4600 50  0001 C CNN
	1    2950 4600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5B96F74F
P 2950 5200
F 0 "#PWR016" H 2950 4950 50  0001 C CNN
F 1 "GND" H 2950 5050 50  0000 C CNN
F 2 "" H 2950 5200 50  0001 C CNN
F 3 "" H 2950 5200 50  0001 C CNN
	1    2950 5200
	1    0    0    -1  
$EndComp
$Comp
L FrontPanel-rescue:CP1 C1
U 1 1 5B96F750
P 2950 4900
AR Path="/5B96F750" Ref="C1"  Part="1" 
AR Path="/5B96CF57/5B96F750" Ref="C1"  Part="1" 
F 0 "C1" H 2975 5000 50  0000 L CNN
F 1 "100uF" H 2975 4800 50  0000 L CNN
F 2 "Capacitors_SMD:CP_Elec_5x5.3" H 2950 4900 50  0001 C CNN
F 3 "" H 2950 4900 50  0001 C CNN
	1    2950 4900
	1    0    0    -1  
$EndComp
$Comp
L FrontPanel-rescue:C C2
U 1 1 5B96F751
P 3750 2800
AR Path="/5B96F751" Ref="C2"  Part="1" 
AR Path="/5B96CF57/5B96F751" Ref="C2"  Part="1" 
F 0 "C2" H 3775 2900 50  0000 L CNN
F 1 "1uF" H 3775 2700 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 3788 2650 50  0001 C CNN
F 3 "" H 3750 2800 50  0001 C CNN
	1    3750 2800
	1    0    0    -1  
$EndComp
$Comp
L FrontPanel-rescue:C C3
U 1 1 5B96F752
P 4000 2800
AR Path="/5B96F752" Ref="C3"  Part="1" 
AR Path="/5B96CF57/5B96F752" Ref="C3"  Part="1" 
F 0 "C3" H 4025 2900 50  0000 L CNN
F 1 ".1uF" H 4025 2700 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 4038 2650 50  0001 C CNN
F 3 "" H 4000 2800 50  0001 C CNN
	1    4000 2800
	1    0    0    -1  
$EndComp
$Comp
L FrontPanel-rescue:C C4
U 1 1 5B96F753
P 4300 2800
AR Path="/5B96F753" Ref="C4"  Part="1" 
AR Path="/5B96CF57/5B96F753" Ref="C4"  Part="1" 
F 0 "C4" H 4325 2900 50  0000 L CNN
F 1 "1uF" H 4325 2700 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 4338 2650 50  0001 C CNN
F 3 "" H 4300 2800 50  0001 C CNN
	1    4300 2800
	1    0    0    -1  
$EndComp
$Comp
L FrontPanel-rescue:C C5
U 1 1 5B96F754
P 4550 2800
AR Path="/5B96F754" Ref="C5"  Part="1" 
AR Path="/5B96CF57/5B96F754" Ref="C5"  Part="1" 
F 0 "C5" H 4575 2900 50  0000 L CNN
F 1 ".1uF" H 4575 2700 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 4588 2650 50  0001 C CNN
F 3 "" H 4550 2800 50  0001 C CNN
	1    4550 2800
	1    0    0    -1  
$EndComp
$Comp
L FrontPanel-rescue:C C6
U 1 1 5B96F755
P 4900 2800
AR Path="/5B96F755" Ref="C6"  Part="1" 
AR Path="/5B96CF57/5B96F755" Ref="C6"  Part="1" 
F 0 "C6" H 4925 2900 50  0000 L CNN
F 1 "1uF" H 4925 2700 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 4938 2650 50  0001 C CNN
F 3 "" H 4900 2800 50  0001 C CNN
	1    4900 2800
	1    0    0    -1  
$EndComp
$Comp
L FrontPanel-rescue:C C7
U 1 1 5B96F756
P 5200 2800
AR Path="/5B96F756" Ref="C7"  Part="1" 
AR Path="/5B96CF57/5B96F756" Ref="C7"  Part="1" 
F 0 "C7" H 5225 2900 50  0000 L CNN
F 1 ".1uF" H 5225 2700 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 5238 2650 50  0001 C CNN
F 3 "" H 5200 2800 50  0001 C CNN
	1    5200 2800
	1    0    0    -1  
$EndComp
$Comp
L FrontPanel-rescue:C C8
U 1 1 5B96F757
P 5450 2800
AR Path="/5B96F757" Ref="C8"  Part="1" 
AR Path="/5B96CF57/5B96F757" Ref="C8"  Part="1" 
F 0 "C8" H 5475 2900 50  0000 L CNN
F 1 "1uF" H 5475 2700 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 5488 2650 50  0001 C CNN
F 3 "" H 5450 2800 50  0001 C CNN
	1    5450 2800
	1    0    0    -1  
$EndComp
$Comp
L FrontPanel-rescue:C C9
U 1 1 5B96F758
P 5750 2800
AR Path="/5B96F758" Ref="C9"  Part="1" 
AR Path="/5B96CF57/5B96F758" Ref="C9"  Part="1" 
F 0 "C9" H 5775 2900 50  0000 L CNN
F 1 ".1uF" H 5775 2700 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 5788 2650 50  0001 C CNN
F 3 "" H 5750 2800 50  0001 C CNN
	1    5750 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 2650 3750 2000
Connection ~ 3750 2000
Wire Wire Line
	3750 2950 3750 3400
Connection ~ 3750 3400
Wire Wire Line
	4000 2650 4000 2000
Connection ~ 4000 2000
Wire Wire Line
	4000 2950 4000 3400
Connection ~ 4000 3400
Wire Wire Line
	4300 2650 4300 2100
Connection ~ 4300 2100
Wire Wire Line
	4300 2950 4300 3500
Connection ~ 4300 3500
Wire Wire Line
	4550 2650 4550 2100
Connection ~ 4550 2100
Wire Wire Line
	4550 2950 4550 3500
Connection ~ 4550 3500
Wire Wire Line
	4900 2650 4900 2200
Connection ~ 4900 2200
Wire Wire Line
	4900 2950 4900 3600
Connection ~ 4900 3600
Wire Wire Line
	5200 2650 5200 2200
Connection ~ 5200 2200
Wire Wire Line
	5200 2950 5200 3600
Connection ~ 5200 3600
Wire Wire Line
	5450 2650 5450 2300
Connection ~ 5450 2300
Wire Wire Line
	5450 2950 5450 3700
Connection ~ 5450 3700
Wire Wire Line
	5750 2650 5750 2300
Connection ~ 5750 2300
Wire Wire Line
	5750 2950 5750 3700
Connection ~ 5750 3700
$Comp
L power:PWR_FLAG #FLG017
U 1 1 5B96F75A
P 2550 5100
F 0 "#FLG017" H 2550 5175 50  0001 C CNN
F 1 "PWR_FLAG" H 2550 5250 50  0000 C CNN
F 2 "" H 2550 5100 50  0001 C CNN
F 3 "" H 2550 5100 50  0001 C CNN
	1    2550 5100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2950 4600 2950 4650
Wire Wire Line
	2950 5050 2950 5100
Wire Wire Line
	2550 5100 2950 5100
Connection ~ 2950 5100
$Comp
L power:PWR_FLAG #FLG018
U 1 1 5B96FD49
P 2550 4650
F 0 "#FLG018" H 2550 4725 50  0001 C CNN
F 1 "PWR_FLAG" H 2550 4800 50  0000 C CNN
F 2 "" H 2550 4650 50  0001 C CNN
F 3 "" H 2550 4650 50  0001 C CNN
	1    2550 4650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2550 4650 2950 4650
Connection ~ 2950 4650
Wire Wire Line
	6550 3500 6550 3600
Wire Wire Line
	6550 3600 6550 3700
Wire Wire Line
	6500 2000 6500 2050
Wire Wire Line
	6500 2050 6500 2100
Wire Wire Line
	6500 2100 6500 2200
Wire Wire Line
	6500 2200 6500 2300
Wire Wire Line
	4050 1100 4050 1150
Wire Wire Line
	3750 2000 4000 2000
Wire Wire Line
	3750 3400 4000 3400
Wire Wire Line
	4000 2000 6500 2000
Wire Wire Line
	4000 3400 6550 3400
Wire Wire Line
	4300 2100 4550 2100
Wire Wire Line
	4300 3500 4550 3500
Wire Wire Line
	4550 2100 6500 2100
Wire Wire Line
	4550 3500 6550 3500
Wire Wire Line
	4900 2200 5200 2200
Wire Wire Line
	4900 3600 5200 3600
Wire Wire Line
	5200 2200 6500 2200
Wire Wire Line
	5200 3600 6550 3600
Wire Wire Line
	5450 2300 3550 2300
Wire Wire Line
	5450 3700 3650 3700
Wire Wire Line
	5750 2300 5450 2300
Wire Wire Line
	5750 3700 5450 3700
Wire Wire Line
	2950 5100 2950 5200
Wire Wire Line
	2950 4650 2950 4750
$EndSCHEMATC
