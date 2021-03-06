EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "Motor Driver"
Date ""
Rev "2"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	11200 5250 12650 5250
Wire Wire Line
	8700 1450 8700 2150
Wire Wire Line
	9100 3550 8500 3550
Wire Wire Line
	11200 5250 11200 2150
$Comp
L power:+5V #PWR057
U 1 1 5E5656C4
P 12500 1150
F 0 "#PWR057" H 12500 1000 50  0001 C CNN
F 1 "+5V" V 12515 1278 50  0000 L CNN
F 2 "" H 12500 1150 50  0001 C CNN
F 3 "" H 12500 1150 50  0001 C CNN
	1    12500 1150
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR05
U 1 1 5E566ACC
P 2200 1900
F 0 "#PWR05" H 2200 1750 50  0001 C CNN
F 1 "+5V" V 2215 2028 50  0000 L CNN
F 2 "" H 2200 1900 50  0001 C CNN
F 3 "" H 2200 1900 50  0001 C CNN
	1    2200 1900
	0    1    1    0   
$EndComp
Wire Wire Line
	12150 2000 12150 1050
Wire Wire Line
	12150 1050 12500 1050
Wire Wire Line
	12350 4700 12350 5450
Text Label 12400 4700 0    50   ~ 0
M4+
Wire Wire Line
	12350 5450 12650 5450
Wire Wire Line
	2500 2550 2500 1800
Wire Wire Line
	1750 2550 2500 2550
Wire Wire Line
	2500 1800 2200 1800
Wire Wire Line
	4750 2450 6300 2450
Wire Wire Line
	2950 2450 3950 2450
Wire Wire Line
	2850 2350 3950 2350
Wire Wire Line
	3600 2550 3950 2550
Wire Wire Line
	3700 2650 3950 2650
Wire Wire Line
	1850 6250 1850 4350
Wire Wire Line
	1750 3750 1750 2550
Wire Wire Line
	1650 2450 1650 3950
Wire Wire Line
	2400 2450 1650 2450
Wire Wire Line
	2400 2300 2400 2450
Wire Wire Line
	6300 2550 4750 2550
Wire Wire Line
	4750 2650 6300 2650
$Comp
L rover:encoder-motor M1
U 1 1 5E6F7FC8
P 1950 1700
F 0 "M1" H 1867 1825 50  0000 C CNN
F 1 "encoder-motor" H 1867 1734 50  0000 C CNN
F 2 "Rover:Motor_Header_Pitch2.54mm" H 1950 1700 50  0001 C CNN
F 3 "" H 1950 1700 50  0001 C CNN
	1    1950 1700
	-1   0    0    -1  
$EndComp
$Comp
L rover:encoder-motor M4
U 1 1 5E6F7712
P 12900 4850
F 0 "M4" H 12850 4850 50  0000 L CNN
F 1 "encoder-motor" H 12600 4150 50  0000 L CNN
F 2 "Rover:Motor_Header_Pitch2.54mm" H 12900 4850 50  0001 C CNN
F 3 "" H 12900 4850 50  0001 C CNN
	1    12900 4850
	1    0    0    -1  
$EndComp
$Comp
L rover:encoder-motor M3
U 1 1 5E6F394C
P 12750 950
F 0 "M3" H 12650 250 50  0000 L CNN
F 1 "encoder-motor" H 12450 950 50  0000 L CNN
F 2 "Rover:Motor_Header_Pitch2.54mm" H 12750 950 50  0001 C CNN
F 3 "" H 12750 950 50  0001 C CNN
	1    12750 950 
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Female J2
U 1 1 5E6CAD27
P 5650 3950
F 0 "J2" H 5700 3900 50  0000 C CNN
F 1 "SPI" H 5550 4200 50  0000 C CNN
F 2 "Rover:SPI_3.3V" H 5650 3950 50  0001 C CNN
F 3 "~" H 5650 3950 50  0001 C CNN
	1    5650 3950
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x04_Female J7
U 1 1 5E6C7B21
P 9800 10050
F 0 "J7" H 9900 10000 50  0000 C CNN
F 1 "UART/Ain" H 9700 10250 50  0000 C CNN
F 2 "Rover:UART_Ain_3.3V" H 9800 10050 50  0001 C CNN
F 3 "~" H 9800 10050 50  0001 C CNN
	1    9800 10050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female J10
U 1 1 5E6C5BDB
P 12150 7650
F 0 "J10" H 12250 7600 50  0000 C CNN
F 1 "I2C/Ain/PWM 5V" H 12050 7850 50  0000 C CNN
F 2 "Rover:I2C_Ain_PWM_5V" H 12150 7650 50  0001 C CNN
F 3 "~" H 12150 7650 50  0001 C CNN
	1    12150 7650
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR09
U 1 1 5E8D9084
P 2450 5650
F 0 "#PWR09" H 2450 5500 50  0001 C CNN
F 1 "+3.3V" V 2465 5778 50  0000 L CNN
F 2 "" H 2450 5650 50  0001 C CNN
F 3 "" H 2450 5650 50  0001 C CNN
	1    2450 5650
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR011
U 1 1 5E8D7308
P 2850 3150
F 0 "#PWR011" H 2850 3000 50  0001 C CNN
F 1 "+3.3V" V 2865 3278 50  0000 L CNN
F 2 "" H 2850 3150 50  0001 C CNN
F 3 "" H 2850 3150 50  0001 C CNN
	1    2850 3150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR047
U 1 1 5E8D5C39
P 11750 2200
F 0 "#PWR047" H 11750 2050 50  0001 C CNN
F 1 "+3.3V" V 11765 2328 50  0000 L CNN
F 2 "" H 11750 2200 50  0001 C CNN
F 3 "" H 11750 2200 50  0001 C CNN
	1    11750 2200
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR050
U 1 1 5E85F69A
P 11950 6900
F 0 "#PWR050" H 11950 6750 50  0001 C CNN
F 1 "+5V" V 11965 7028 50  0000 L CNN
F 2 "" H 11950 6900 50  0001 C CNN
F 3 "" H 11950 6900 50  0001 C CNN
	1    11950 6900
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR049
U 1 1 5E85F690
P 11950 6800
F 0 "#PWR049" H 11950 6550 50  0001 C CNN
F 1 "GND" V 11955 6672 50  0000 R CNN
F 2 "" H 11950 6800 50  0001 C CNN
F 3 "" H 11950 6800 50  0001 C CNN
	1    11950 6800
	0    1    -1   0   
$EndComp
Wire Wire Line
	12900 1900 12900 3000
Wire Wire Line
	12500 1900 12900 1900
Wire Wire Line
	12800 2800 12800 2000
Wire Wire Line
	12750 3300 12750 3400
Wire Wire Line
	12650 4800 12650 4950
Wire Wire Line
	9350 3650 8500 3650
Wire Wire Line
	9350 3600 9350 3650
Wire Wire Line
	11450 3600 9350 3600
Wire Wire Line
	9450 3750 8500 3750
Wire Wire Line
	9450 3500 9450 3750
Wire Wire Line
	11450 3500 9450 3500
Wire Wire Line
	9550 3400 11450 3400
Wire Wire Line
	9550 3850 9550 3400
Wire Wire Line
	8500 3850 9550 3850
Wire Wire Line
	10700 3950 8500 3950
Wire Wire Line
	10700 3300 10700 3950
Wire Wire Line
	11450 3300 10700 3300
Wire Wire Line
	11000 4850 8500 4850
Wire Wire Line
	11000 3100 11000 4850
Wire Wire Line
	11450 3100 11000 3100
Wire Wire Line
	10900 4950 8500 4950
Wire Wire Line
	10900 3000 10900 4950
Wire Wire Line
	11450 3000 10900 3000
$Comp
L power:+3.3V #PWR038
U 1 1 5E7BA2E4
P 9600 10050
F 0 "#PWR038" H 9600 9900 50  0001 C CNN
F 1 "+3.3V" V 9615 10178 50  0000 L CNN
F 2 "" H 9600 10050 50  0001 C CNN
F 3 "" H 9600 10050 50  0001 C CNN
	1    9600 10050
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR037
U 1 1 5E7BA2D0
P 9600 9950
F 0 "#PWR037" H 9600 9700 50  0001 C CNN
F 1 "GND" V 9605 9822 50  0000 R CNN
F 2 "" H 9600 9950 50  0001 C CNN
F 3 "" H 9600 9950 50  0001 C CNN
	1    9600 9950
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR051
U 1 1 5E7B43F6
P 11950 7550
F 0 "#PWR051" H 11950 7300 50  0001 C CNN
F 1 "GND" V 11955 7422 50  0000 R CNN
F 2 "" H 11950 7550 50  0001 C CNN
F 3 "" H 11950 7550 50  0001 C CNN
	1    11950 7550
	0    1    1    0   
$EndComp
Wire Wire Line
	3150 4550 6300 4550
Wire Wire Line
	3150 4350 6300 4350
Text GLabel 11450 2800 0    50   Input ~ 0
STBY
Text GLabel 8500 3450 2    50   Input ~ 0
STBY
$Comp
L power:+3.3V #PWR022
U 1 1 5E725C67
P 5850 4050
F 0 "#PWR022" H 5850 3900 50  0001 C CNN
F 1 "+3.3V" V 5865 4178 50  0000 L CNN
F 2 "" H 5850 4050 50  0001 C CNN
F 3 "" H 5850 4050 50  0001 C CNN
	1    5850 4050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR023
U 1 1 5E723FEE
P 5850 4150
F 0 "#PWR023" H 5850 3900 50  0001 C CNN
F 1 "GND" V 5855 4022 50  0000 R CNN
F 2 "" H 5850 4150 50  0001 C CNN
F 3 "" H 5850 4150 50  0001 C CNN
	1    5850 4150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6200 3950 5850 3950
Wire Wire Line
	6200 4150 6200 3950
Wire Wire Line
	6300 4150 6200 4150
$Comp
L power:+3.3V #PWR029
U 1 1 5E711E32
P 8500 4750
F 0 "#PWR029" H 8500 4600 50  0001 C CNN
F 1 "+3.3V" V 8515 4878 50  0000 L CNN
F 2 "" H 8500 4750 50  0001 C CNN
F 3 "" H 8500 4750 50  0001 C CNN
	1    8500 4750
	0    1    1    0   
$EndComp
Wire Wire Line
	2850 2350 2850 2100
$Comp
L Teensy:Teensy4.0 U5
U 1 1 5E69C7B1
P 7400 3600
F 0 "U5" H 7400 5215 50  0000 C CNN
F 1 "Teensy4.0" H 7400 5124 50  0000 C CNN
F 2 "Teensy:Teensy40" H 7000 3800 50  0001 C CNN
F 3 "" H 7000 3800 50  0001 C CNN
	1    7400 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5E34B9B2
P 2750 5650
F 0 "#PWR010" H 2750 5400 50  0001 C CNN
F 1 "GND" V 2755 5522 50  0000 R CNN
F 2 "" H 2750 5650 50  0001 C CNN
F 3 "" H 2750 5650 50  0001 C CNN
	1    2750 5650
	0    -1   -1   0   
$EndComp
Text Label 12750 4800 0    50   ~ 0
M4-
Text Label 12500 1900 0    50   ~ 0
M3+
Text Label 12200 1050 0    50   ~ 0
M3-
Text Label 2300 2300 0    50   ~ 0
M1+
Text Label 2350 1800 0    50   ~ 0
M1-
$Comp
L power:-BATT #PWR014
U 1 1 5E2C61BF
P 3200 6350
F 0 "#PWR014" H 3200 6200 50  0001 C CNN
F 1 "-BATT" H 3215 6523 50  0000 C CNN
F 2 "" H 3200 6350 50  0001 C CNN
F 3 "" H 3200 6350 50  0001 C CNN
	1    3200 6350
	-1   0    0    1   
$EndComp
$Comp
L power:+BATT #PWR013
U 1 1 5E2C3B2E
P 2900 6350
F 0 "#PWR013" H 2900 6200 50  0001 C CNN
F 1 "+BATT" H 2915 6523 50  0000 C CNN
F 2 "" H 2900 6350 50  0001 C CNN
F 3 "" H 2900 6350 50  0001 C CNN
	1    2900 6350
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR027
U 1 1 5E56360A
P 8500 4550
F 0 "#PWR027" H 8500 4400 50  0001 C CNN
F 1 "+5V" V 8515 4678 50  0000 L CNN
F 2 "" H 8500 4550 50  0001 C CNN
F 3 "" H 8500 4550 50  0001 C CNN
	1    8500 4550
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR058
U 1 1 5E2CF0C2
P 12500 1450
F 0 "#PWR058" H 12500 1200 50  0001 C CNN
F 1 "GND" V 12505 1322 50  0000 R CNN
F 2 "" H 12500 1450 50  0001 C CNN
F 3 "" H 12500 1450 50  0001 C CNN
	1    12500 1450
	0    1    1    0   
$EndComp
Wire Wire Line
	12500 1900 12500 1550
Wire Wire Line
	3400 1350 3400 2750
$Comp
L power:GND #PWR060
U 1 1 5E2CD954
P 12650 5350
F 0 "#PWR060" H 12650 5100 50  0001 C CNN
F 1 "GND" V 12655 5222 50  0000 R CNN
F 2 "" H 12650 5350 50  0001 C CNN
F 3 "" H 12650 5350 50  0001 C CNN
	1    12650 5350
	0    1    1    0   
$EndComp
Wire Wire Line
	12900 3100 12650 3100
Wire Wire Line
	3700 2650 3700 6750
Wire Wire Line
	3600 2550 3600 6850
$Comp
L power:-BATT #PWR055
U 1 1 5E2C92CF
P 12350 4250
F 0 "#PWR055" H 12350 4100 50  0001 C CNN
F 1 "-BATT" H 12365 4423 50  0000 C CNN
F 2 "" H 12350 4250 50  0001 C CNN
F 3 "" H 12350 4250 50  0001 C CNN
	1    12350 4250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR024
U 1 1 5E2BD080
P 6300 2250
F 0 "#PWR024" H 6300 2000 50  0001 C CNN
F 1 "GND" V 6305 2122 50  0000 R CNN
F 2 "" H 6300 2250 50  0001 C CNN
F 3 "" H 6300 2250 50  0001 C CNN
	1    6300 2250
	0    1    1    0   
$EndComp
Wire Wire Line
	2950 2000 2950 2450
Wire Wire Line
	2200 2000 2950 2000
Wire Wire Line
	2850 2100 2200 2100
Text GLabel 3150 3750 2    50   Input ~ 0
STBY
Wire Wire Line
	2200 2300 2400 2300
$Comp
L power:GND #PWR06
U 1 1 5E2E09D1
P 2200 2200
F 0 "#PWR06" H 2200 1950 50  0001 C CNN
F 1 "GND" V 2205 2072 50  0000 R CNN
F 2 "" H 2200 2200 50  0001 C CNN
F 3 "" H 2200 2200 50  0001 C CNN
	1    2200 2200
	0    -1   -1   0   
$EndComp
$Comp
L power:-BATT #PWR07
U 1 1 5E2E098F
P 2250 5200
F 0 "#PWR07" H 2250 5050 50  0001 C CNN
F 1 "-BATT" H 2265 5373 50  0000 C CNN
F 2 "" H 2250 5200 50  0001 C CNN
F 3 "" H 2250 5200 50  0001 C CNN
	1    2250 5200
	-1   0    0    1   
$EndComp
$Comp
L power:+BATT #PWR02
U 1 1 5E2E0985
P 2150 3150
F 0 "#PWR02" H 2150 3000 50  0001 C CNN
F 1 "+BATT" V 2165 3277 50  0000 L CNN
F 2 "" H 2150 3150 50  0001 C CNN
F 3 "" H 2150 3150 50  0001 C CNN
	1    2150 3150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR012
U 1 1 5E2E0961
P 2850 5150
F 0 "#PWR012" H 2850 4900 50  0001 C CNN
F 1 "GND" V 2855 5022 50  0000 R CNN
F 2 "" H 2850 5150 50  0001 C CNN
F 3 "" H 2850 5150 50  0001 C CNN
	1    2850 5150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR053
U 1 1 5E7112B0
P 11950 8250
F 0 "#PWR053" H 11950 8000 50  0001 C CNN
F 1 "GND" V 11955 8122 50  0000 R CNN
F 2 "" H 11950 8250 50  0001 C CNN
F 3 "" H 11950 8250 50  0001 C CNN
	1    11950 8250
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR054
U 1 1 5E7107CE
P 11950 8350
F 0 "#PWR054" H 11950 8200 50  0001 C CNN
F 1 "+5V" V 11965 8478 50  0000 L CNN
F 2 "" H 11950 8350 50  0001 C CNN
F 3 "" H 11950 8350 50  0001 C CNN
	1    11950 8350
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x03_Female J11
U 1 1 5E6EE9DA
P 12150 8350
F 0 "J11" H 12178 8376 50  0000 L CNN
F 1 "PWM/PPM 5V" H 11850 8150 50  0000 L CNN
F 2 "Rover:PPM_PWM_5V" H 12150 8350 50  0001 C CNN
F 3 "~" H 12150 8350 50  0001 C CNN
	1    12150 8350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5E2E09BD
P 2150 6950
F 0 "#PWR04" H 2150 6700 50  0001 C CNN
F 1 "GND" V 2155 6822 50  0000 R CNN
F 2 "" H 2150 6950 50  0001 C CNN
F 3 "" H 2150 6950 50  0001 C CNN
	1    2150 6950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3600 6850 2150 6850
Wire Wire Line
	2150 6750 3700 6750
Text Label 2350 7050 0    50   ~ 0
M2+
Text Label 2300 6550 0    50   ~ 0
M2-
$Comp
L rover:encoder-motor M2
U 1 1 5E6F8B45
P 1900 6450
F 0 "M2" H 1817 6575 50  0000 C CNN
F 1 "encoder-motor" H 1817 6484 50  0000 C CNN
F 2 "Rover:Motor_Header_Pitch2.54mm" H 1900 6450 50  0001 C CNN
F 3 "" H 1900 6450 50  0001 C CNN
	1    1900 6450
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2500 6250 1850 6250
Wire Wire Line
	2150 6550 2500 6550
Wire Wire Line
	2500 6550 2500 6250
Wire Wire Line
	2600 6150 1950 6150
Wire Wire Line
	2150 7050 2600 7050
Wire Wire Line
	2600 7050 2600 6150
$Comp
L power:+5V #PWR03
U 1 1 5E5671D4
P 2150 6650
F 0 "#PWR03" H 2150 6500 50  0001 C CNN
F 1 "+5V" V 2165 6778 50  0000 L CNN
F 2 "" H 2150 6650 50  0001 C CNN
F 3 "" H 2150 6650 50  0001 C CNN
	1    2150 6650
	0    1    1    0   
$EndComp
Wire Wire Line
	2250 5200 2250 5150
Wire Wire Line
	1750 3750 1950 3750
Wire Wire Line
	1850 4350 1950 4350
Wire Wire Line
	1950 4250 1850 4250
Wire Wire Line
	1850 4250 1850 4350
Connection ~ 1850 4350
Wire Wire Line
	1650 3950 1950 3950
Wire Wire Line
	1950 3850 1950 3750
Wire Wire Line
	1950 4050 1950 3950
Wire Wire Line
	3150 4250 6300 4250
Wire Wire Line
	3500 2850 3500 1250
Wire Wire Line
	5850 3850 6200 3850
Wire Wire Line
	6200 3850 6200 3550
Wire Wire Line
	6200 3550 6300 3550
Wire Wire Line
	5850 3750 6100 3750
Wire Wire Line
	6100 3750 6100 3450
Wire Wire Line
	6100 3450 6300 3450
Wire Wire Line
	5850 3650 6000 3650
Wire Wire Line
	6000 3650 6000 3350
Wire Wire Line
	6000 3350 6300 3350
Wire Wire Line
	5900 3550 5900 3250
Wire Wire Line
	5900 3250 6300 3250
Wire Wire Line
	5800 3450 5800 3150
Wire Wire Line
	5800 3150 6300 3150
Wire Wire Line
	3500 1250 12500 1250
Wire Wire Line
	3400 1350 12500 1350
Wire Wire Line
	8700 2150 11200 2150
Wire Wire Line
	12150 2000 12800 2000
Wire Wire Line
	5450 3550 5450 4150
Wire Wire Line
	5450 3550 5900 3550
Wire Wire Line
	5350 3450 5350 4050
Wire Wire Line
	5350 3450 5800 3450
Wire Wire Line
	6300 2950 5550 2950
Wire Wire Line
	6300 2850 4750 2850
Wire Wire Line
	4750 2750 6300 2750
Wire Wire Line
	4750 2350 6300 2350
Wire Wire Line
	3500 2850 3950 2850
Wire Wire Line
	3400 2750 3950 2750
$Comp
L power:GND #PWR017
U 1 1 5ED05D17
P 4350 2050
F 0 "#PWR017" H 4350 1800 50  0001 C CNN
F 1 "GND" V 4355 1922 50  0000 R CNN
F 2 "" H 4350 2050 50  0001 C CNN
F 3 "" H 4350 2050 50  0001 C CNN
	1    4350 2050
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR016
U 1 1 5ED06847
P 4250 3650
F 0 "#PWR016" H 4250 3500 50  0001 C CNN
F 1 "+5V" V 4265 3778 50  0000 L CNN
F 2 "" H 4250 3650 50  0001 C CNN
F 3 "" H 4250 3650 50  0001 C CNN
	1    4250 3650
	-1   0    0    1   
$EndComp
Wire Wire Line
	4200 3600 4250 3600
Wire Wire Line
	4250 3600 4250 3650
$Comp
L power:+3.3V #PWR018
U 1 1 5ED12190
P 4450 3650
F 0 "#PWR018" H 4450 3500 50  0001 C CNN
F 1 "+3.3V" V 4465 3778 50  0000 L CNN
F 2 "" H 4450 3650 50  0001 C CNN
F 3 "" H 4450 3650 50  0001 C CNN
	1    4450 3650
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR021
U 1 1 5ED5C54A
P 5100 3200
F 0 "#PWR021" H 5100 3050 50  0001 C CNN
F 1 "+3.3V" V 5115 3328 50  0000 L CNN
F 2 "" H 5100 3200 50  0001 C CNN
F 3 "" H 5100 3200 50  0001 C CNN
	1    5100 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	12350 4250 12350 4200
$Comp
L power:GND #PWR048
U 1 1 5EE41D28
P 11750 4200
F 0 "#PWR048" H 11750 3950 50  0001 C CNN
F 1 "GND" V 11755 4072 50  0000 R CNN
F 2 "" H 11750 4200 50  0001 C CNN
F 3 "" H 11750 4200 50  0001 C CNN
	1    11750 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	12350 4200 12250 4200
Connection ~ 12250 4200
Wire Wire Line
	12150 4200 12050 4200
Wire Wire Line
	12250 4200 12150 4200
Connection ~ 12150 4200
Connection ~ 12350 4200
$Comp
L Driver_Motor:TB6612FNG U2
U 1 1 5E6E87D2
P 12050 3200
F 0 "U2" H 12050 4381 50  0000 C CNN
F 1 "TB6612FNG" H 12050 4290 50  0000 C CNN
F 2 "Housings_SSOP:SSOP-24_5.3x8.2mm_Pitch0.65mm" H 12500 3800 50  0001 C CNN
F 3 "https://toshiba.semicon-storage.com/us/product/linear/motordriver/detail.TB6612FNG.html" H 12500 3800 50  0001 C CNN
	1    12050 3200
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR056
U 1 1 5EE43780
P 12400 2200
F 0 "#PWR056" H 12400 2050 50  0001 C CNN
F 1 "+BATT" V 12415 2327 50  0000 L CNN
F 2 "" H 12400 2200 50  0001 C CNN
F 3 "" H 12400 2200 50  0001 C CNN
	1    12400 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	12400 2200 12350 2200
Wire Wire Line
	12350 2200 12250 2200
Connection ~ 12350 2200
Wire Wire Line
	12250 2200 12150 2200
Connection ~ 12250 2200
Wire Wire Line
	12650 4700 12350 4700
Wire Wire Line
	12750 4800 12650 4800
Wire Wire Line
	12650 3600 12650 3500
Wire Wire Line
	12750 3300 12650 3300
Wire Wire Line
	12650 3400 12750 3400
Wire Wire Line
	12750 4800 12750 3400
Connection ~ 12750 3400
Wire Wire Line
	12650 3600 12650 4700
Connection ~ 12650 3600
Wire Wire Line
	12800 2800 12650 2800
Wire Wire Line
	12800 2800 12800 2900
Wire Wire Line
	12800 2900 12650 2900
Connection ~ 12800 2800
Wire Wire Line
	12650 3000 12900 3000
Connection ~ 12900 3000
Wire Wire Line
	12900 3000 12900 3100
Wire Wire Line
	3300 1450 8700 1450
$Comp
L Device:C C3
U 1 1 5F0D4105
P 2600 5650
F 0 "C3" V 2348 5650 50  0000 C CNN
F 1 "0.1uF" V 2439 5650 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 2638 5500 50  0001 C CNN
F 3 "~" H 2600 5650 50  0001 C CNN
	1    2600 5650
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR045
U 1 1 5F0D6769
P 11050 1900
F 0 "#PWR045" H 11050 1750 50  0001 C CNN
F 1 "+3.3V" V 11065 2028 50  0000 L CNN
F 2 "" H 11050 1900 50  0001 C CNN
F 3 "" H 11050 1900 50  0001 C CNN
	1    11050 1900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR046
U 1 1 5F0D6773
P 11350 1900
F 0 "#PWR046" H 11350 1650 50  0001 C CNN
F 1 "GND" V 11355 1772 50  0000 R CNN
F 2 "" H 11350 1900 50  0001 C CNN
F 3 "" H 11350 1900 50  0001 C CNN
	1    11350 1900
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C9
U 1 1 5F0D677D
P 11200 1900
F 0 "C9" V 10948 1900 50  0000 C CNN
F 1 "0.1uF" V 11039 1900 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 11238 1750 50  0001 C CNN
F 3 "~" H 11200 1900 50  0001 C CNN
	1    11200 1900
	0    1    1    0   
$EndComp
$Comp
L Device:C C5
U 1 1 5F0F12E2
P 4050 3600
F 0 "C5" V 4200 3600 50  0000 C CNN
F 1 "0.1uF" V 4300 3600 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4088 3450 50  0001 C CNN
F 3 "~" H 4050 3600 50  0001 C CNN
	1    4050 3600
	0    1    1    0   
$EndComp
Wire Wire Line
	5450 4150 3150 4150
Wire Wire Line
	3150 4150 3150 4050
Wire Wire Line
	3250 4050 3250 3950
Wire Wire Line
	3250 3950 3150 3950
$Comp
L Device:C C6
U 1 1 5F117322
P 4650 3600
F 0 "C6" V 4800 3600 50  0000 C CNN
F 1 "0.1uF" V 4900 3600 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4688 3450 50  0001 C CNN
F 3 "~" H 4650 3600 50  0001 C CNN
	1    4650 3600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5F117DCA
P 3900 3600
F 0 "#PWR015" H 3900 3350 50  0001 C CNN
F 1 "GND" V 4000 3600 50  0000 R CNN
F 2 "" H 3900 3600 50  0001 C CNN
F 3 "" H 3900 3600 50  0001 C CNN
	1    3900 3600
	0    1    1    0   
$EndComp
$Comp
L Device:C C1
U 1 1 5F13C88D
P 3050 6000
F 0 "C1" V 3000 6250 50  0000 C CNN
F 1 "10uF" V 3100 6300 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 3088 5850 50  0001 C CNN
F 3 "~" H 3050 6000 50  0001 C CNN
	1    3050 6000
	0    1    1    0   
$EndComp
Connection ~ 3200 6250
Wire Wire Line
	3200 6250 3200 6350
Connection ~ 2900 6250
Wire Wire Line
	2900 6250 2900 6350
Wire Wire Line
	3200 6000 3200 6250
Wire Wire Line
	2900 6000 2900 6250
$Comp
L Device:C C4
U 1 1 5F13DF1D
P 3050 6250
F 0 "C4" V 3000 6500 50  0000 C CNN
F 1 "0.1uF" V 3100 6550 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 3088 6100 50  0001 C CNN
F 3 "~" H 3050 6250 50  0001 C CNN
	1    3050 6250
	0    1    1    0   
$EndComp
$Comp
L power:-BATT #PWR062
U 1 1 5F15EF3E
P 13100 6200
F 0 "#PWR062" H 13100 6050 50  0001 C CNN
F 1 "-BATT" H 13115 6373 50  0000 C CNN
F 2 "" H 13100 6200 50  0001 C CNN
F 3 "" H 13100 6200 50  0001 C CNN
	1    13100 6200
	-1   0    0    1   
$EndComp
$Comp
L power:+BATT #PWR061
U 1 1 5F15EF48
P 12800 6200
F 0 "#PWR061" H 12800 6050 50  0001 C CNN
F 1 "+BATT" H 12815 6373 50  0000 C CNN
F 2 "" H 12800 6200 50  0001 C CNN
F 3 "" H 12800 6200 50  0001 C CNN
	1    12800 6200
	-1   0    0    1   
$EndComp
$Comp
L Device:C C2
U 1 1 5F15EF60
P 12950 5850
F 0 "C2" V 12900 6100 50  0000 C CNN
F 1 "10uF" V 13000 6150 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 12988 5700 50  0001 C CNN
F 3 "~" H 12950 5850 50  0001 C CNN
	1    12950 5850
	0    1    1    0   
$EndComp
Connection ~ 13100 6100
Wire Wire Line
	13100 6100 13100 6200
Connection ~ 12800 6100
Wire Wire Line
	12800 6100 12800 6200
Wire Wire Line
	13100 5850 13100 6100
Wire Wire Line
	12800 5850 12800 6100
$Comp
L Device:C C10
U 1 1 5F15EF72
P 12950 6100
F 0 "C10" V 12900 6350 50  0000 C CNN
F 1 "0.1uF" V 13000 6400 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 12988 5950 50  0001 C CNN
F 3 "~" H 12950 6100 50  0001 C CNN
	1    12950 6100
	0    1    1    0   
$EndComp
$Comp
L power:-BATT #PWR025
U 1 1 5F16E107
P 7200 5850
F 0 "#PWR025" H 7200 5700 50  0001 C CNN
F 1 "-BATT" H 7215 6023 50  0000 C CNN
F 2 "" H 7200 5850 50  0001 C CNN
F 3 "" H 7200 5850 50  0001 C CNN
	1    7200 5850
	0    -1   -1   0   
$EndComp
$Comp
L power:+BATT #PWR026
U 1 1 5F16E862
P 7200 6250
F 0 "#PWR026" H 7200 6100 50  0001 C CNN
F 1 "+BATT" H 7215 6423 50  0000 C CNN
F 2 "" H 7200 6250 50  0001 C CNN
F 3 "" H 7200 6250 50  0001 C CNN
	1    7200 6250
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR039
U 1 1 5F1FE144
P 9850 5850
F 0 "#PWR039" H 9850 5700 50  0001 C CNN
F 1 "+3.3V" V 9865 5978 50  0000 L CNN
F 2 "" H 9850 5850 50  0001 C CNN
F 3 "" H 9850 5850 50  0001 C CNN
	1    9850 5850
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR044
U 1 1 5F1FE16E
P 11000 5800
F 0 "#PWR044" H 11000 5550 50  0001 C CNN
F 1 "GND" V 11100 5800 50  0000 R CNN
F 2 "" H 11000 5800 50  0001 C CNN
F 3 "" H 11000 5800 50  0001 C CNN
	1    11000 5800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9100 6550 10150 6550
Wire Wire Line
	9100 3550 9100 6550
Wire Wire Line
	9200 3350 9200 6450
Wire Wire Line
	9200 6450 9300 6450
Wire Wire Line
	8500 3350 9200 3350
Wire Wire Line
	9300 3250 9300 6350
Wire Wire Line
	9300 6350 9400 6350
Wire Wire Line
	8500 3250 9300 3250
Wire Wire Line
	11700 6550 11700 5150
Wire Wire Line
	11700 5150 12650 5150
Wire Wire Line
	10950 6550 11700 6550
Wire Wire Line
	11950 6450 10950 6450
Wire Wire Line
	10950 6350 11950 6350
$Comp
L power:GND #PWR040
U 1 1 5F1FE164
P 10100 5800
F 0 "#PWR040" H 10100 5550 50  0001 C CNN
F 1 "GND" V 10000 5800 50  0000 R CNN
F 2 "" H 10100 5800 50  0001 C CNN
F 3 "" H 10100 5800 50  0001 C CNN
	1    10100 5800
	0    1    1    0   
$EndComp
$Comp
L Device:C C7
U 1 1 5F1FE15A
P 10250 5800
F 0 "C7" V 10400 5800 50  0000 C CNN
F 1 "0.1uF" V 10500 5800 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 10288 5650 50  0001 C CNN
F 3 "~" H 10250 5800 50  0001 C CNN
	1    10250 5800
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C8
U 1 1 5F1FE150
P 10850 5800
F 0 "C8" V 11000 5800 50  0000 C CNN
F 1 "0.1uF" V 11100 5800 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 10888 5650 50  0001 C CNN
F 3 "~" H 10850 5800 50  0001 C CNN
	1    10850 5800
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR041
U 1 1 5F1FE138
P 10450 5700
F 0 "#PWR041" H 10450 5550 50  0001 C CNN
F 1 "+3.3V" V 10465 5828 50  0000 L CNN
F 2 "" H 10450 5700 50  0001 C CNN
F 3 "" H 10450 5700 50  0001 C CNN
	1    10450 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	10700 5800 10650 5800
$Comp
L power:+5V #PWR043
U 1 1 5F1FE12C
P 10650 5700
F 0 "#PWR043" H 10650 5550 50  0001 C CNN
F 1 "+5V" V 10665 5828 50  0000 L CNN
F 2 "" H 10650 5700 50  0001 C CNN
F 3 "" H 10650 5700 50  0001 C CNN
	1    10650 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 6650 9150 6650
Wire Wire Line
	9000 4150 9000 6650
Wire Wire Line
	8900 6750 9050 6750
Wire Wire Line
	8900 4050 8900 6750
Wire Wire Line
	5950 4950 6300 4950
Wire Wire Line
	6050 4850 6300 4850
Wire Wire Line
	6150 6950 8800 6950
Wire Wire Line
	10950 6850 11450 6850
Wire Wire Line
	11450 6850 11450 7750
Wire Wire Line
	11450 7750 11950 7750
Wire Wire Line
	10950 6950 11350 6950
Wire Wire Line
	11350 6950 11350 7850
Wire Wire Line
	11350 7850 11950 7850
$Comp
L Connector:Conn_01x04_Female J9
U 1 1 5F4684E8
P 12150 6900
F 0 "J9" H 12250 6850 50  0000 C CNN
F 1 "UART/I2C/PWM 5V" H 12050 7100 50  0000 C CNN
F 2 "Rover:UART_I2C_PWM_5V" H 12150 6900 50  0001 C CNN
F 3 "~" H 12150 6900 50  0001 C CNN
	1    12150 6900
	1    0    0    -1  
$EndComp
Wire Wire Line
	10950 6750 11550 6750
Wire Wire Line
	11550 6750 11550 7100
Wire Wire Line
	11550 7100 11950 7100
Wire Wire Line
	11950 7000 11650 7000
Wire Wire Line
	11650 7000 11650 6650
Wire Wire Line
	11650 6650 10950 6650
$Comp
L Connector:Conn_01x04_Female J5
U 1 1 5F4B36F5
P 9800 8750
F 0 "J5" H 9900 8700 50  0000 C CNN
F 1 "I2C/Ain/PWM 3.3V" H 9700 8950 50  0000 C CNN
F 2 "Rover:I2C_Ain_PWM_3.3V" H 9800 8750 50  0001 C CNN
F 3 "~" H 9800 8750 50  0001 C CNN
	1    9800 8750
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR034
U 1 1 5F4B36FF
P 9600 8750
F 0 "#PWR034" H 9600 8600 50  0001 C CNN
F 1 "+3.3V" V 9615 8878 50  0000 L CNN
F 2 "" H 9600 8750 50  0001 C CNN
F 3 "" H 9600 8750 50  0001 C CNN
	1    9600 8750
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR033
U 1 1 5F4B3709
P 9600 8650
F 0 "#PWR033" H 9600 8400 50  0001 C CNN
F 1 "GND" V 9605 8522 50  0000 R CNN
F 2 "" H 9600 8650 50  0001 C CNN
F 3 "" H 9600 8650 50  0001 C CNN
	1    9600 8650
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR052
U 1 1 5F4C26FF
P 11950 7650
F 0 "#PWR052" H 11950 7500 50  0001 C CNN
F 1 "+5V" V 11965 7778 50  0000 L CNN
F 2 "" H 11950 7650 50  0001 C CNN
F 3 "" H 11950 7650 50  0001 C CNN
	1    11950 7650
	0    -1   1    0   
$EndComp
Wire Wire Line
	8900 6850 8900 8850
Wire Wire Line
	8900 8850 9600 8850
Connection ~ 8900 6850
Wire Wire Line
	8900 6850 10150 6850
Wire Wire Line
	8800 6950 8800 8950
Wire Wire Line
	8800 8950 9600 8950
Connection ~ 8800 6950
Wire Wire Line
	8800 6950 10150 6950
$Comp
L power:GND #PWR031
U 1 1 5F4F6192
P 9600 8000
F 0 "#PWR031" H 9600 7750 50  0001 C CNN
F 1 "GND" V 9605 7872 50  0000 R CNN
F 2 "" H 9600 8000 50  0001 C CNN
F 3 "" H 9600 8000 50  0001 C CNN
	1    9600 8000
	0    1    -1   0   
$EndComp
$Comp
L Connector:Conn_01x04_Female J4
U 1 1 5F4F619C
P 9800 8100
F 0 "J4" H 9900 8050 50  0000 C CNN
F 1 "UART/I2C/PWM 3.3V" H 9700 8300 50  0000 C CNN
F 2 "Rover:UART_I2C_PWM_3.3V" H 9800 8100 50  0001 C CNN
F 3 "~" H 9800 8100 50  0001 C CNN
	1    9800 8100
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 8300 9050 8300
Wire Wire Line
	9050 8300 9050 6750
Connection ~ 9050 6750
Wire Wire Line
	9050 6750 10150 6750
Wire Wire Line
	9600 8200 9150 8200
Wire Wire Line
	9150 8200 9150 6650
Connection ~ 9150 6650
Wire Wire Line
	9150 6650 10150 6650
$Comp
L Connector:Conn_01x02_Female J8
U 1 1 5E98A624
P 12150 6350
F 0 "J8" H 12178 6326 50  0000 L CNN
F 1 "32-33 5V" H 11950 6450 50  0000 L CNN
F 2 "Rover:32-33_5V" H 12150 6350 50  0001 C CNN
F 3 "~" H 12150 6350 50  0001 C CNN
	1    12150 6350
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J3
U 1 1 5F5609E9
P 9800 7600
F 0 "J3" H 9828 7576 50  0000 L CNN
F 1 "32-33 3.3V" H 9600 7700 50  0000 L CNN
F 2 "Rover:32-33_3.3V" H 9800 7600 50  0001 C CNN
F 3 "~" H 9800 7600 50  0001 C CNN
	1    9800 7600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 7700 9300 7700
Wire Wire Line
	9300 7700 9300 6450
Connection ~ 9300 6450
Wire Wire Line
	9300 6450 10150 6450
Wire Wire Line
	9400 6350 9400 7600
Wire Wire Line
	9400 7600 9600 7600
Connection ~ 9400 6350
Wire Wire Line
	9400 6350 10150 6350
Wire Wire Line
	6250 6850 8900 6850
Wire Wire Line
	6250 4650 6300 4650
Wire Wire Line
	6250 4650 6250 6850
Wire Wire Line
	6300 4750 6150 4750
Wire Wire Line
	6150 4750 6150 6950
Wire Wire Line
	3950 2950 3300 2950
Wire Wire Line
	3300 2950 3300 1450
Wire Wire Line
	5050 3050 5050 2950
Wire Wire Line
	5050 2950 4750 2950
Wire Wire Line
	5050 3050 6300 3050
Wire Wire Line
	10950 7050 11250 7050
Wire Wire Line
	11250 7050 11250 8450
Wire Wire Line
	11250 8450 11950 8450
$Comp
L power:GND #PWR035
U 1 1 5F64E937
P 9600 9250
F 0 "#PWR035" H 9600 9000 50  0001 C CNN
F 1 "GND" V 9605 9122 50  0000 R CNN
F 2 "" H 9600 9250 50  0001 C CNN
F 3 "" H 9600 9250 50  0001 C CNN
	1    9600 9250
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x03_Female J6
U 1 1 5F64E943
P 9800 9350
F 0 "J6" H 9828 9376 50  0000 L CNN
F 1 "PWM/PPM 3.3V" H 9500 9150 50  0000 L CNN
F 2 "Rover:PPM_PWM_3.3V" H 9800 9350 50  0001 C CNN
F 3 "~" H 9800 9350 50  0001 C CNN
	1    9800 9350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 9450 9600 9450
Wire Wire Line
	8650 9450 8650 7050
Wire Wire Line
	5550 7050 8650 7050
Connection ~ 8650 7050
Wire Wire Line
	8650 7050 10150 7050
Wire Wire Line
	6050 4850 6050 7150
Wire Wire Line
	6050 7150 8500 7150
Wire Wire Line
	8500 7150 8500 10150
Wire Wire Line
	8500 10150 9600 10150
Wire Wire Line
	5950 4950 5950 7250
Wire Wire Line
	5950 7250 8400 7250
Wire Wire Line
	8400 7250 8400 10250
Wire Wire Line
	8400 10250 9600 10250
$Comp
L power:GND #PWR028
U 1 1 5F7A6113
P 8500 4650
F 0 "#PWR028" H 8500 4400 50  0001 C CNN
F 1 "GND" V 8505 4522 50  0000 R CNN
F 2 "" H 8500 4650 50  0001 C CNN
F 3 "" H 8500 4650 50  0001 C CNN
	1    8500 4650
	0    -1   -1   0   
$EndComp
NoConn ~ 6300 4050
NoConn ~ 6300 3950
NoConn ~ 6300 3850
NoConn ~ 6300 3750
NoConn ~ 6300 3650
NoConn ~ 8500 4450
NoConn ~ 8500 3150
NoConn ~ 8500 3050
NoConn ~ 8500 2950
NoConn ~ 8500 2850
NoConn ~ 8500 2750
NoConn ~ 8500 2650
NoConn ~ 8500 2550
NoConn ~ 8500 2450
NoConn ~ 8500 2350
NoConn ~ 8500 2250
NoConn ~ 4750 3050
NoConn ~ 3950 3050
$Comp
L rover:XT-60 J1
U 1 1 5F8E713E
P 7200 6050
F 0 "J1" H 7703 6096 50  0000 L CNN
F 1 "XT-60" H 7703 6005 50  0000 L CNN
F 2 "Rover:AMASS_XT60-M_1x02_P7.20mm_Vertical" H 7200 6050 50  0001 C CNN
F 3 "" H 7200 6050 50  0001 C CNN
	1    7200 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 5950 7300 5850
Wire Wire Line
	7300 5850 7200 5850
Wire Wire Line
	7300 6150 7300 6250
Wire Wire Line
	7300 6250 7200 6250
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5F92CC1C
P 1850 10750
F 0 "#FLG01" H 1850 10825 50  0001 C CNN
F 1 "PWR_FLAG" H 1850 10923 50  0000 C CNN
F 2 "" H 1850 10750 50  0001 C CNN
F 3 "~" H 1850 10750 50  0001 C CNN
	1    1850 10750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5F92EB1C
P 1850 10750
F 0 "#PWR01" H 1850 10500 50  0001 C CNN
F 1 "GND" V 1855 10622 50  0000 R CNN
F 2 "" H 1850 10750 50  0001 C CNN
F 3 "" H 1850 10750 50  0001 C CNN
	1    1850 10750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 2950 5550 7050
$Comp
L Device:R R1
U 1 1 5F92FDC6
P 5050 3450
F 0 "R1" H 5120 3496 50  0000 L CNN
F 1 "47k" H 5120 3405 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4980 3450 50  0001 C CNN
F 3 "~" H 5050 3450 50  0001 C CNN
	1    5050 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 5F931609
P 5050 3600
F 0 "#PWR020" H 5050 3350 50  0001 C CNN
F 1 "GND" V 5050 3450 50  0000 R CNN
F 2 "" H 5050 3600 50  0001 C CNN
F 3 "" H 5050 3600 50  0001 C CNN
	1    5050 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 5F11781D
P 4800 3600
F 0 "#PWR019" H 4800 3350 50  0001 C CNN
F 1 "GND" V 4700 3600 50  0000 R CNN
F 2 "" H 4800 3600 50  0001 C CNN
F 3 "" H 4800 3600 50  0001 C CNN
	1    4800 3600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5100 3200 5050 3200
Wire Wire Line
	5050 3300 5050 3200
Connection ~ 5050 3200
Wire Wire Line
	5050 3200 4900 3200
$Comp
L Device:R R2
U 1 1 5F9B9CFF
P 9700 6050
F 0 "R2" H 9770 6096 50  0000 L CNN
F 1 "47k" H 9770 6005 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9630 6050 50  0001 C CNN
F 3 "~" H 9700 6050 50  0001 C CNN
	1    9700 6050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR030
U 1 1 5F9CAE17
P 9550 6050
F 0 "#PWR030" H 9550 5800 50  0001 C CNN
F 1 "GND" V 9450 6050 50  0000 R CNN
F 2 "" H 9550 6050 50  0001 C CNN
F 3 "" H 9550 6050 50  0001 C CNN
	1    9550 6050
	0    1    1    0   
$EndComp
Wire Wire Line
	9850 5850 9850 6050
Connection ~ 9850 6050
$Comp
L power:+5V #PWR059
U 1 1 5FA78404
P 12650 5050
F 0 "#PWR059" H 12650 4900 50  0001 C CNN
F 1 "+5V" V 12665 5178 50  0000 L CNN
F 2 "" H 12650 5050 50  0001 C CNN
F 3 "" H 12650 5050 50  0001 C CNN
	1    12650 5050
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR08
U 1 1 5FA79E2E
P 2300 10750
F 0 "#PWR08" H 2300 10600 50  0001 C CNN
F 1 "+5V" V 2315 10878 50  0000 L CNN
F 2 "" H 2300 10750 50  0001 C CNN
F 3 "" H 2300 10750 50  0001 C CNN
	1    2300 10750
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5FA7C0EA
P 2300 10750
F 0 "#FLG02" H 2300 10825 50  0001 C CNN
F 1 "PWR_FLAG" H 2300 10923 50  0000 C CNN
F 2 "" H 2300 10750 50  0001 C CNN
F 3 "~" H 2300 10750 50  0001 C CNN
	1    2300 10750
	1    0    0    -1  
$EndComp
Connection ~ 1950 3950
Connection ~ 1950 3750
Connection ~ 1950 4550
Wire Wire Line
	1950 4550 1950 6150
Wire Wire Line
	1950 4450 1950 4550
Connection ~ 2450 5150
Wire Wire Line
	2450 5150 2550 5150
Connection ~ 2350 5150
Connection ~ 2250 5150
Wire Wire Line
	2450 5150 2350 5150
Wire Wire Line
	2250 5150 2350 5150
Connection ~ 2250 3150
Connection ~ 2350 3150
Wire Wire Line
	2350 3150 2450 3150
Wire Wire Line
	2350 3150 2250 3150
Wire Wire Line
	2150 3150 2250 3150
Wire Wire Line
	5350 4050 3250 4050
Wire Wire Line
	3150 4450 6300 4450
$Comp
L Driver_Motor:TB6612FNG U1
U 1 1 5E6DA167
P 2550 4150
F 0 "U1" H 2550 5331 50  0000 C CNN
F 1 "TB6612FNG" H 2550 5240 50  0000 C CNN
F 2 "Housings_SSOP:SSOP-24_5.3x8.2mm_Pitch0.65mm" H 3000 4750 50  0001 C CNN
F 3 "https://toshiba.semicon-storage.com/us/product/linear/motordriver/detail.TB6612FNG.html" H 3000 4750 50  0001 C CNN
	1    2550 4150
	-1   0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR036
U 1 1 5FB60539
P 9600 9350
F 0 "#PWR036" H 9600 9200 50  0001 C CNN
F 1 "+3.3V" V 9615 9478 50  0000 L CNN
F 2 "" H 9600 9350 50  0001 C CNN
F 3 "" H 9600 9350 50  0001 C CNN
	1    9600 9350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10650 5800 10650 5950
Connection ~ 10450 5800
Wire Wire Line
	10450 5700 10450 5800
Wire Wire Line
	10400 5800 10450 5800
Wire Wire Line
	10450 5800 10450 5950
Wire Wire Line
	10650 5700 10650 5800
Connection ~ 10650 5800
Wire Wire Line
	10150 6250 10050 6250
Wire Wire Line
	10050 6250 10050 6050
Wire Wire Line
	9850 6050 10050 6050
Wire Wire Line
	4450 3650 4450 3600
Wire Wire Line
	4450 3600 4500 3600
Wire Wire Line
	4450 3600 4450 3450
Connection ~ 4450 3600
Wire Wire Line
	4250 3450 4250 3600
Connection ~ 4250 3600
Wire Wire Line
	4900 3200 4900 3150
Wire Wire Line
	4900 3150 4750 3150
$Comp
L Logic_LevelTranslator:TXS0108EPW U3
U 1 1 5FCD18DD
P 4350 2750
F 0 "U3" H 4050 1950 50  0000 C CNN
F 1 "TXS0108EPW" H 4000 2050 50  0000 C CNN
F 2 "Housings_SSOP:TSSOP-20_4.4x6.5mm_Pitch0.65mm" H 4350 2000 50  0001 C CNN
F 3 "www.ti.com/lit/ds/symlink/txs0108e.pdf" H 4350 2650 50  0001 C CNN
	1    4350 2750
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR042
U 1 1 5F1FE122
P 10550 7350
F 0 "#PWR042" H 10550 7100 50  0001 C CNN
F 1 "GND" V 10555 7222 50  0000 R CNN
F 2 "" H 10550 7350 50  0001 C CNN
F 3 "" H 10550 7350 50  0001 C CNN
	1    10550 7350
	1    0    0    -1  
$EndComp
$Comp
L Logic_LevelTranslator:TXS0108EPW U4
U 1 1 5FC06432
P 10550 6650
F 0 "U4" H 10900 5800 50  0000 C CNN
F 1 "TXS0108EPW" H 10900 5900 50  0000 C CNN
F 2 "Housings_SSOP:TSSOP-20_4.4x6.5mm_Pitch0.65mm" H 10550 5900 50  0001 C CNN
F 3 "www.ti.com/lit/ds/symlink/txs0108e.pdf" H 10550 6550 50  0001 C CNN
	1    10550 6650
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR032
U 1 1 5FE34F50
P 9600 8100
F 0 "#PWR032" H 9600 7950 50  0001 C CNN
F 1 "+3.3V" V 9615 8228 50  0000 L CNN
F 2 "" H 9600 8100 50  0001 C CNN
F 3 "" H 9600 8100 50  0001 C CNN
	1    9600 8100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9000 4150 8500 4150
Wire Wire Line
	8900 4050 8500 4050
$EndSCHEMATC
