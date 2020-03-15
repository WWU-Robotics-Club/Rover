EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
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
	11200 5250 12650 5250
Wire Wire Line
	8700 1450 8700 2150
Wire Wire Line
	9100 3550 8500 3550
Wire Wire Line
	11200 5250 11200 2150
$Comp
L power:+5V #PWR0104
U 1 1 5E5656C4
P 12500 1150
F 0 "#PWR0104" H 12500 1000 50  0001 C CNN
F 1 "+5V" V 12515 1278 50  0000 L CNN
F 2 "" H 12500 1150 50  0001 C CNN
F 3 "" H 12500 1150 50  0001 C CNN
	1    12500 1150
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR0106
U 1 1 5E566ACC
P 2200 1900
F 0 "#PWR0106" H 2200 1750 50  0001 C CNN
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
	4950 2450 6300 2450
Wire Wire Line
	2950 2450 3750 2450
Wire Wire Line
	2850 2350 3750 2350
Wire Wire Line
	3600 2550 3750 2550
Wire Wire Line
	3700 2650 3750 2650
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
	6300 2550 4950 2550
Wire Wire Line
	4950 2650 6300 2650
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
L power:+3.3V #PWR0120
U 1 1 5E8D9084
P 2450 5650
F 0 "#PWR0120" H 2450 5500 50  0001 C CNN
F 1 "+3.3V" V 2465 5778 50  0000 L CNN
F 2 "" H 2450 5650 50  0001 C CNN
F 3 "" H 2450 5650 50  0001 C CNN
	1    2450 5650
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0119
U 1 1 5E8D7308
P 2850 3150
F 0 "#PWR0119" H 2850 3000 50  0001 C CNN
F 1 "+3.3V" V 2865 3278 50  0000 L CNN
F 2 "" H 2850 3150 50  0001 C CNN
F 3 "" H 2850 3150 50  0001 C CNN
	1    2850 3150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0118
U 1 1 5E8D5C39
P 11750 2200
F 0 "#PWR0118" H 11750 2050 50  0001 C CNN
F 1 "+3.3V" V 11765 2328 50  0000 L CNN
F 2 "" H 11750 2200 50  0001 C CNN
F 3 "" H 11750 2200 50  0001 C CNN
	1    11750 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 4150 8900 4150
$Comp
L power:+5V #PWR0116
U 1 1 5E85F69A
P 11900 7050
F 0 "#PWR0116" H 11900 6900 50  0001 C CNN
F 1 "+5V" V 11915 7178 50  0000 L CNN
F 2 "" H 11900 7050 50  0001 C CNN
F 3 "" H 11900 7050 50  0001 C CNN
	1    11900 7050
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5E85F690
P 11900 7150
F 0 "#PWR0115" H 11900 6900 50  0001 C CNN
F 1 "GND" V 11905 7022 50  0000 R CNN
F 2 "" H 11900 7150 50  0001 C CNN
F 3 "" H 11900 7150 50  0001 C CNN
	1    11900 7150
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
L power:+3.3V #PWR0114
U 1 1 5E7BA2E4
P 9600 10050
F 0 "#PWR0114" H 9600 9900 50  0001 C CNN
F 1 "+3.3V" V 9615 10178 50  0000 L CNN
F 2 "" H 9600 10050 50  0001 C CNN
F 3 "" H 9600 10050 50  0001 C CNN
	1    9600 10050
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 5E7BA2D0
P 9600 9950
F 0 "#PWR0113" H 9600 9700 50  0001 C CNN
F 1 "GND" V 9605 9822 50  0000 R CNN
F 2 "" H 9600 9950 50  0001 C CNN
F 3 "" H 9600 9950 50  0001 C CNN
	1    9600 9950
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 5E7B43F6
P 11950 7550
F 0 "#PWR0111" H 11950 7300 50  0001 C CNN
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
L power:+3.3V #PWR0110
U 1 1 5E725C67
P 5850 4050
F 0 "#PWR0110" H 5850 3900 50  0001 C CNN
F 1 "+3.3V" V 5865 4178 50  0000 L CNN
F 2 "" H 5850 4050 50  0001 C CNN
F 3 "" H 5850 4050 50  0001 C CNN
	1    5850 4050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 5E723FEE
P 5850 4150
F 0 "#PWR0109" H 5850 3900 50  0001 C CNN
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
L power:+3.3V #PWR0108
U 1 1 5E711E32
P 8500 4750
F 0 "#PWR0108" H 8500 4600 50  0001 C CNN
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
L power:GND #PWR03
U 1 1 5E34B9B2
P 2750 5650
F 0 "#PWR03" H 2750 5400 50  0001 C CNN
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
L power:-BATT #PWR020
U 1 1 5E2C61BF
P 3200 6350
F 0 "#PWR020" H 3200 6200 50  0001 C CNN
F 1 "-BATT" H 3215 6523 50  0000 C CNN
F 2 "" H 3200 6350 50  0001 C CNN
F 3 "" H 3200 6350 50  0001 C CNN
	1    3200 6350
	-1   0    0    1   
$EndComp
$Comp
L power:+BATT #PWR019
U 1 1 5E2C3B2E
P 2900 6350
F 0 "#PWR019" H 2900 6200 50  0001 C CNN
F 1 "+BATT" H 2915 6523 50  0000 C CNN
F 2 "" H 2900 6350 50  0001 C CNN
F 3 "" H 2900 6350 50  0001 C CNN
	1    2900 6350
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR0101
U 1 1 5E56360A
P 8500 4550
F 0 "#PWR0101" H 8500 4400 50  0001 C CNN
F 1 "+5V" V 8515 4678 50  0000 L CNN
F 2 "" H 8500 4550 50  0001 C CNN
F 3 "" H 8500 4550 50  0001 C CNN
	1    8500 4550
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5E2CF0C2
P 12500 1450
F 0 "#PWR015" H 12500 1200 50  0001 C CNN
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
L power:GND #PWR017
U 1 1 5E2CD954
P 12650 5350
F 0 "#PWR017" H 12650 5100 50  0001 C CNN
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
L power:-BATT #PWR018
U 1 1 5E2C92CF
P 12350 4250
F 0 "#PWR018" H 12350 4100 50  0001 C CNN
F 1 "-BATT" H 12365 4423 50  0000 C CNN
F 2 "" H 12350 4250 50  0001 C CNN
F 3 "" H 12350 4250 50  0001 C CNN
	1    12350 4250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5E2BD080
P 6300 2250
F 0 "#PWR021" H 6300 2000 50  0001 C CNN
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
L power:GND #PWR04
U 1 1 5E2E09D1
P 2200 2200
F 0 "#PWR04" H 2200 1950 50  0001 C CNN
F 1 "GND" V 2205 2072 50  0000 R CNN
F 2 "" H 2200 2200 50  0001 C CNN
F 3 "" H 2200 2200 50  0001 C CNN
	1    2200 2200
	0    -1   -1   0   
$EndComp
$Comp
L power:-BATT #PWR08
U 1 1 5E2E098F
P 2250 5200
F 0 "#PWR08" H 2250 5050 50  0001 C CNN
F 1 "-BATT" H 2265 5373 50  0000 C CNN
F 2 "" H 2250 5200 50  0001 C CNN
F 3 "" H 2250 5200 50  0001 C CNN
	1    2250 5200
	-1   0    0    1   
$EndComp
$Comp
L power:+BATT #PWR05
U 1 1 5E2E0985
P 2150 3150
F 0 "#PWR05" H 2150 3000 50  0001 C CNN
F 1 "+BATT" V 2165 3277 50  0000 L CNN
F 2 "" H 2150 3150 50  0001 C CNN
F 3 "" H 2150 3150 50  0001 C CNN
	1    2150 3150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5E2E0961
P 2850 5150
F 0 "#PWR07" H 2850 4900 50  0001 C CNN
F 1 "GND" V 2855 5022 50  0000 R CNN
F 2 "" H 2850 5150 50  0001 C CNN
F 3 "" H 2850 5150 50  0001 C CNN
	1    2850 5150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5E7112B0
P 11950 8250
F 0 "#PWR0105" H 11950 8000 50  0001 C CNN
F 1 "GND" V 11955 8122 50  0000 R CNN
F 2 "" H 11950 8250 50  0001 C CNN
F 3 "" H 11950 8250 50  0001 C CNN
	1    11950 8250
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0102
U 1 1 5E7107CE
P 11950 8350
F 0 "#PWR0102" H 11950 8200 50  0001 C CNN
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
Wire Wire Line
	8500 4050 9000 4050
$Comp
L TXB0108PWR:TXB0108PWR U3
U 1 1 5E6E4ECA
P 4350 2850
F 0 "U3" H 4650 2000 50  0000 C CNN
F 1 "TXB0108PWR" H 4350 3200 50  0000 C CNN
F 2 "TXB0108PWR:TSSOP20" H 4350 2850 50  0001 L BNN
F 3 "" H 4350 2850 50  0001 C CNN
	1    4350 2850
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5E2E09BD
P 2150 6950
F 0 "#PWR02" H 2150 6700 50  0001 C CNN
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
L power:+5V #PWR0107
U 1 1 5E5671D4
P 2150 6650
F 0 "#PWR0107" H 2150 6500 50  0001 C CNN
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
	6300 2850 4950 2850
Wire Wire Line
	4950 2750 6300 2750
Wire Wire Line
	4950 2350 6300 2350
Wire Wire Line
	3500 2850 3750 2850
Wire Wire Line
	3400 2750 3750 2750
$Comp
L power:GND #PWR0112
U 1 1 5ED05D17
P 4350 1950
F 0 "#PWR0112" H 4350 1700 50  0001 C CNN
F 1 "GND" V 4355 1822 50  0000 R CNN
F 2 "" H 4350 1950 50  0001 C CNN
F 3 "" H 4350 1950 50  0001 C CNN
	1    4350 1950
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR0117
U 1 1 5ED06847
P 4250 3700
F 0 "#PWR0117" H 4250 3550 50  0001 C CNN
F 1 "+5V" V 4265 3828 50  0000 L CNN
F 2 "" H 4250 3700 50  0001 C CNN
F 3 "" H 4250 3700 50  0001 C CNN
	1    4250 3700
	-1   0    0    1   
$EndComp
Wire Wire Line
	4200 3700 4250 3700
Wire Wire Line
	4250 3700 4250 3650
$Comp
L power:+3.3V #PWR0121
U 1 1 5ED12190
P 4450 3700
F 0 "#PWR0121" H 4450 3550 50  0001 C CNN
F 1 "+3.3V" V 4465 3828 50  0000 L CNN
F 2 "" H 4450 3700 50  0001 C CNN
F 3 "" H 4450 3700 50  0001 C CNN
	1    4450 3700
	-1   0    0    1   
$EndComp
Wire Wire Line
	4500 3700 4450 3700
Wire Wire Line
	4450 3700 4450 3650
$Comp
L power:+3.3V #PWR0122
U 1 1 5ED5C54A
P 5150 3350
F 0 "#PWR0122" H 5150 3200 50  0001 C CNN
F 1 "+3.3V" V 5165 3478 50  0000 L CNN
F 2 "" H 5150 3350 50  0001 C CNN
F 3 "" H 5150 3350 50  0001 C CNN
	1    5150 3350
	0    1    1    0   
$EndComp
Wire Wire Line
	12350 4250 12350 4200
$Comp
L power:GND #PWR0123
U 1 1 5EE41D28
P 11750 4200
F 0 "#PWR0123" H 11750 3950 50  0001 C CNN
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
L power:+BATT #PWR0124
U 1 1 5EE43780
P 12400 2200
F 0 "#PWR0124" H 12400 2050 50  0001 C CNN
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
L power:+3.3V #PWR0125
U 1 1 5F0D6769
P 11050 1900
F 0 "#PWR0125" H 11050 1750 50  0001 C CNN
F 1 "+3.3V" V 11065 2028 50  0000 L CNN
F 2 "" H 11050 1900 50  0001 C CNN
F 3 "" H 11050 1900 50  0001 C CNN
	1    11050 1900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0126
U 1 1 5F0D6773
P 11350 1900
F 0 "#PWR0126" H 11350 1650 50  0001 C CNN
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
Connection ~ 4450 3700
Connection ~ 4250 3700
$Comp
L Device:C C5
U 1 1 5F0F12E2
P 4050 3700
F 0 "C5" V 4200 3700 50  0000 C CNN
F 1 "0.1uF" V 4300 3700 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4088 3550 50  0001 C CNN
F 3 "~" H 4050 3700 50  0001 C CNN
	1    4050 3700
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
P 4650 3700
F 0 "C6" V 4800 3700 50  0000 C CNN
F 1 "0.1uF" V 4900 3700 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4688 3550 50  0001 C CNN
F 3 "~" H 4650 3700 50  0001 C CNN
	1    4650 3700
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0128
U 1 1 5F117DCA
P 3900 3700
F 0 "#PWR0128" H 3900 3450 50  0001 C CNN
F 1 "GND" V 4000 3700 50  0000 R CNN
F 2 "" H 3900 3700 50  0001 C CNN
F 3 "" H 3900 3700 50  0001 C CNN
	1    3900 3700
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
L power:-BATT #PWR0129
U 1 1 5F15EF3E
P 12700 6200
F 0 "#PWR0129" H 12700 6050 50  0001 C CNN
F 1 "-BATT" H 12715 6373 50  0000 C CNN
F 2 "" H 12700 6200 50  0001 C CNN
F 3 "" H 12700 6200 50  0001 C CNN
	1    12700 6200
	-1   0    0    1   
$EndComp
$Comp
L power:+BATT #PWR0130
U 1 1 5F15EF48
P 12400 6200
F 0 "#PWR0130" H 12400 6050 50  0001 C CNN
F 1 "+BATT" H 12415 6373 50  0000 C CNN
F 2 "" H 12400 6200 50  0001 C CNN
F 3 "" H 12400 6200 50  0001 C CNN
	1    12400 6200
	-1   0    0    1   
$EndComp
$Comp
L Device:C C2
U 1 1 5F15EF60
P 12550 5850
F 0 "C2" V 12500 6100 50  0000 C CNN
F 1 "10uF" V 12600 6150 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 12588 5700 50  0001 C CNN
F 3 "~" H 12550 5850 50  0001 C CNN
	1    12550 5850
	0    1    1    0   
$EndComp
Connection ~ 12700 6100
Wire Wire Line
	12700 6100 12700 6200
Connection ~ 12400 6100
Wire Wire Line
	12400 6100 12400 6200
Wire Wire Line
	12700 5850 12700 6100
Wire Wire Line
	12400 5850 12400 6100
$Comp
L Device:C C10
U 1 1 5F15EF72
P 12550 6100
F 0 "C10" V 12500 6350 50  0000 C CNN
F 1 "0.1uF" V 12600 6400 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 12588 5950 50  0001 C CNN
F 3 "~" H 12550 6100 50  0001 C CNN
	1    12550 6100
	0    1    1    0   
$EndComp
$Comp
L power:-BATT #PWR0131
U 1 1 5F16E107
P 7200 5850
F 0 "#PWR0131" H 7200 5700 50  0001 C CNN
F 1 "-BATT" H 7215 6023 50  0000 C CNN
F 2 "" H 7200 5850 50  0001 C CNN
F 3 "" H 7200 5850 50  0001 C CNN
	1    7200 5850
	0    -1   -1   0   
$EndComp
$Comp
L power:+BATT #PWR0132
U 1 1 5F16E862
P 7200 6250
F 0 "#PWR0132" H 7200 6100 50  0001 C CNN
F 1 "+BATT" H 7215 6423 50  0000 C CNN
F 2 "" H 7200 6250 50  0001 C CNN
F 3 "" H 7200 6250 50  0001 C CNN
	1    7200 6250
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0133
U 1 1 5F1FE144
P 9850 5850
F 0 "#PWR0133" H 9850 5700 50  0001 C CNN
F 1 "+3.3V" V 9865 5978 50  0000 L CNN
F 2 "" H 9850 5850 50  0001 C CNN
F 3 "" H 9850 5850 50  0001 C CNN
	1    9850 5850
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0134
U 1 1 5F1FE16E
P 11000 5700
F 0 "#PWR0134" H 11000 5450 50  0001 C CNN
F 1 "GND" V 11100 5700 50  0000 R CNN
F 2 "" H 11000 5700 50  0001 C CNN
F 3 "" H 11000 5700 50  0001 C CNN
	1    11000 5700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9100 6550 9950 6550
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
	11150 6550 11700 6550
Wire Wire Line
	11900 6450 11150 6450
Wire Wire Line
	11150 6350 11900 6350
$Comp
L power:GND #PWR0135
U 1 1 5F1FE164
P 10100 5700
F 0 "#PWR0135" H 10100 5450 50  0001 C CNN
F 1 "GND" V 10000 5700 50  0000 R CNN
F 2 "" H 10100 5700 50  0001 C CNN
F 3 "" H 10100 5700 50  0001 C CNN
	1    10100 5700
	0    1    1    0   
$EndComp
$Comp
L Device:C C7
U 1 1 5F1FE15A
P 10250 5700
F 0 "C7" V 10400 5700 50  0000 C CNN
F 1 "0.1uF" V 10500 5700 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 10288 5550 50  0001 C CNN
F 3 "~" H 10250 5700 50  0001 C CNN
	1    10250 5700
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C8
U 1 1 5F1FE150
P 10850 5700
F 0 "C8" V 11000 5700 50  0000 C CNN
F 1 "0.1uF" V 11100 5700 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 10888 5550 50  0001 C CNN
F 3 "~" H 10850 5700 50  0001 C CNN
	1    10850 5700
	0    -1   -1   0   
$EndComp
Connection ~ 10650 5700
Connection ~ 10450 5700
Wire Wire Line
	10450 5700 10450 5750
Wire Wire Line
	10400 5700 10450 5700
$Comp
L power:+3.3V #PWR0136
U 1 1 5F1FE138
P 10450 5700
F 0 "#PWR0136" H 10450 5550 50  0001 C CNN
F 1 "+3.3V" V 10465 5828 50  0000 L CNN
F 2 "" H 10450 5700 50  0001 C CNN
F 3 "" H 10450 5700 50  0001 C CNN
	1    10450 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	10650 5700 10650 5750
Wire Wire Line
	10700 5700 10650 5700
$Comp
L power:+5V #PWR0137
U 1 1 5F1FE12C
P 10650 5700
F 0 "#PWR0137" H 10650 5550 50  0001 C CNN
F 1 "+5V" V 10665 5828 50  0000 L CNN
F 2 "" H 10650 5700 50  0001 C CNN
F 3 "" H 10650 5700 50  0001 C CNN
	1    10650 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0138
U 1 1 5F1FE122
P 10550 7450
F 0 "#PWR0138" H 10550 7200 50  0001 C CNN
F 1 "GND" V 10555 7322 50  0000 R CNN
F 2 "" H 10550 7450 50  0001 C CNN
F 3 "" H 10550 7450 50  0001 C CNN
	1    10550 7450
	1    0    0    -1  
$EndComp
$Comp
L TXB0108PWR:TXB0108PWR U4
U 1 1 5F1FE113
P 10550 6550
F 0 "U4" H 10850 5700 50  0000 C CNN
F 1 "TXB0108PWR" H 10550 6900 50  0000 C CNN
F 2 "TXB0108PWR:TSSOP20" H 10550 6550 50  0001 L BNN
F 3 "" H 10550 6550 50  0001 C CNN
	1    10550 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 6650 9150 6650
Wire Wire Line
	9000 4050 9000 6650
Wire Wire Line
	8900 6750 9050 6750
Wire Wire Line
	8900 4150 8900 6750
Wire Wire Line
	5950 4950 6300 4950
Wire Wire Line
	6050 4850 6300 4850
Wire Wire Line
	6150 6950 8800 6950
Wire Wire Line
	11150 6850 11450 6850
Wire Wire Line
	11450 6850 11450 7750
Wire Wire Line
	11450 7750 11950 7750
Wire Wire Line
	11150 6950 11350 6950
Wire Wire Line
	11350 6950 11350 7850
Wire Wire Line
	11350 7850 11950 7850
$Comp
L Connector:Conn_01x04_Female J9
U 1 1 5F4684E8
P 12100 6950
F 0 "J9" H 12200 6900 50  0000 C CNN
F 1 "UART/I2C/PWM 5V" H 12000 7150 50  0000 C CNN
F 2 "Rover:I2C_Ain_PWM_5V" H 12100 6950 50  0001 C CNN
F 3 "~" H 12100 6950 50  0001 C CNN
	1    12100 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	11150 6750 11550 6750
Wire Wire Line
	11550 6750 11550 6950
Wire Wire Line
	11550 6950 11900 6950
Wire Wire Line
	11900 6850 11650 6850
Wire Wire Line
	11650 6850 11650 6650
Wire Wire Line
	11650 6650 11150 6650
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
L power:+3.3V #PWR0139
U 1 1 5F4B36FF
P 9600 8750
F 0 "#PWR0139" H 9600 8600 50  0001 C CNN
F 1 "+3.3V" V 9615 8878 50  0000 L CNN
F 2 "" H 9600 8750 50  0001 C CNN
F 3 "" H 9600 8750 50  0001 C CNN
	1    9600 8750
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0140
U 1 1 5F4B3709
P 9600 8650
F 0 "#PWR0140" H 9600 8400 50  0001 C CNN
F 1 "GND" V 9605 8522 50  0000 R CNN
F 2 "" H 9600 8650 50  0001 C CNN
F 3 "" H 9600 8650 50  0001 C CNN
	1    9600 8650
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0141
U 1 1 5F4C26FF
P 11950 7650
F 0 "#PWR0141" H 11950 7500 50  0001 C CNN
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
	8900 6850 9950 6850
Wire Wire Line
	8800 6950 8800 8950
Wire Wire Line
	8800 8950 9600 8950
Connection ~ 8800 6950
Wire Wire Line
	8800 6950 9950 6950
$Comp
L power:+5V #PWR0142
U 1 1 5F4F6188
P 9600 8200
F 0 "#PWR0142" H 9600 8050 50  0001 C CNN
F 1 "+5V" V 9615 8328 50  0000 L CNN
F 2 "" H 9600 8200 50  0001 C CNN
F 3 "" H 9600 8200 50  0001 C CNN
	1    9600 8200
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR0143
U 1 1 5F4F6192
P 9600 8300
F 0 "#PWR0143" H 9600 8050 50  0001 C CNN
F 1 "GND" V 9605 8172 50  0000 R CNN
F 2 "" H 9600 8300 50  0001 C CNN
F 3 "" H 9600 8300 50  0001 C CNN
	1    9600 8300
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
	9600 8100 9050 8100
Wire Wire Line
	9050 8100 9050 6750
Connection ~ 9050 6750
Wire Wire Line
	9050 6750 9950 6750
Wire Wire Line
	9600 8000 9150 8000
Wire Wire Line
	9150 8000 9150 6650
Connection ~ 9150 6650
Wire Wire Line
	9150 6650 9950 6650
$Comp
L Connector:Conn_01x02_Female J8
U 1 1 5E98A624
P 12100 6350
F 0 "J8" H 12128 6326 50  0000 L CNN
F 1 "32-33" H 11900 6450 50  0000 L CNN
F 2 "Rover:32-33_5V" H 12100 6350 50  0001 C CNN
F 3 "~" H 12100 6350 50  0001 C CNN
	1    12100 6350
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J3
U 1 1 5F5609E9
P 9800 7600
F 0 "J3" H 9828 7576 50  0000 L CNN
F 1 "32-33" H 9600 7700 50  0000 L CNN
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
	9300 6450 9950 6450
Wire Wire Line
	9400 6350 9400 7600
Wire Wire Line
	9400 7600 9600 7600
Connection ~ 9400 6350
Wire Wire Line
	9400 6350 9950 6350
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
	3750 2950 3300 2950
Wire Wire Line
	3300 2950 3300 1450
Wire Wire Line
	5050 3050 5050 2950
Wire Wire Line
	5050 2950 4950 2950
Wire Wire Line
	5050 3050 6300 3050
Wire Wire Line
	11150 7050 11250 7050
Wire Wire Line
	11250 7050 11250 8450
Wire Wire Line
	11250 8450 11950 8450
$Comp
L power:GND #PWR0144
U 1 1 5F64E937
P 9600 9250
F 0 "#PWR0144" H 9600 9000 50  0001 C CNN
F 1 "GND" V 9605 9122 50  0000 R CNN
F 2 "" H 9600 9250 50  0001 C CNN
F 3 "" H 9600 9250 50  0001 C CNN
	1    9600 9250
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0145
U 1 1 5F64E93D
P 9600 9350
F 0 "#PWR0145" H 9600 9200 50  0001 C CNN
F 1 "+5V" V 9615 9478 50  0000 L CNN
F 2 "" H 9600 9350 50  0001 C CNN
F 3 "" H 9600 9350 50  0001 C CNN
	1    9600 9350
	0    -1   -1   0   
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
	8650 7050 9950 7050
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
L power:GND #PWR0146
U 1 1 5F7A6113
P 8500 4650
F 0 "#PWR0146" H 8500 4400 50  0001 C CNN
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
NoConn ~ 4950 3050
NoConn ~ 3750 3050
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
L power:PWR_FLAG #FLG0101
U 1 1 5F92CC1C
P 1850 10750
F 0 "#FLG0101" H 1850 10825 50  0001 C CNN
F 1 "PWR_FLAG" H 1850 10923 50  0000 C CNN
F 2 "" H 1850 10750 50  0001 C CNN
F 3 "~" H 1850 10750 50  0001 C CNN
	1    1850 10750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0147
U 1 1 5F92EB1C
P 1850 10750
F 0 "#PWR0147" H 1850 10500 50  0001 C CNN
F 1 "GND" V 1855 10622 50  0000 R CNN
F 2 "" H 1850 10750 50  0001 C CNN
F 3 "" H 1850 10750 50  0001 C CNN
	1    1850 10750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 2950 5550 7050
$Comp
L Device:R 47k1
U 1 1 5F92FDC6
P 5100 3600
F 0 "47k1" H 5170 3646 50  0000 L CNN
F 1 "R" H 5170 3555 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5030 3600 50  0001 C CNN
F 3 "~" H 5100 3600 50  0001 C CNN
	1    5100 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0148
U 1 1 5F931609
P 5100 3750
F 0 "#PWR0148" H 5100 3500 50  0001 C CNN
F 1 "GND" V 5000 3750 50  0000 R CNN
F 2 "" H 5100 3750 50  0001 C CNN
F 3 "" H 5100 3750 50  0001 C CNN
	1    5100 3750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0127
U 1 1 5F11781D
P 4800 3700
F 0 "#PWR0127" H 4800 3450 50  0001 C CNN
F 1 "GND" V 4700 3700 50  0000 R CNN
F 2 "" H 4800 3700 50  0001 C CNN
F 3 "" H 4800 3700 50  0001 C CNN
	1    4800 3700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5150 3350 5100 3350
Wire Wire Line
	5100 3450 5100 3350
Connection ~ 5100 3350
Wire Wire Line
	5100 3350 4950 3350
$Comp
L Device:R R1
U 1 1 5F9B9CFF
P 9700 6050
F 0 "R1" H 9770 6096 50  0000 L CNN
F 1 "47k" H 9770 6005 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9630 6050 50  0001 C CNN
F 3 "~" H 9700 6050 50  0001 C CNN
	1    9700 6050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0149
U 1 1 5F9CAE17
P 9550 6050
F 0 "#PWR0149" H 9550 5800 50  0001 C CNN
F 1 "GND" V 9450 6050 50  0000 R CNN
F 2 "" H 9550 6050 50  0001 C CNN
F 3 "" H 9550 6050 50  0001 C CNN
	1    9550 6050
	0    1    1    0   
$EndComp
Wire Wire Line
	9950 6050 9850 6050
Wire Wire Line
	9850 5850 9850 6050
Connection ~ 9850 6050
$Comp
L power:+5V #PWR0103
U 1 1 5FA78404
P 12650 5050
F 0 "#PWR0103" H 12650 4900 50  0001 C CNN
F 1 "+5V" V 12665 5178 50  0000 L CNN
F 2 "" H 12650 5050 50  0001 C CNN
F 3 "" H 12650 5050 50  0001 C CNN
	1    12650 5050
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR0150
U 1 1 5FA79E2E
P 2300 10750
F 0 "#PWR0150" H 2300 10600 50  0001 C CNN
F 1 "+5V" V 2315 10878 50  0000 L CNN
F 2 "" H 2300 10750 50  0001 C CNN
F 3 "" H 2300 10750 50  0001 C CNN
	1    2300 10750
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5FA7C0EA
P 2300 10750
F 0 "#FLG0102" H 2300 10825 50  0001 C CNN
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
$EndSCHEMATC
