EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Amplifier"
Date "2021-07-24"
Rev "2"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Amplifier_Audio:TDA2003 U1
U 1 1 60F80348
P 4600 2250
F 0 "U1" H 4944 2296 50  0000 L CNN
F 1 "TDA2003" H 4944 2205 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-5_P3.4x3.7mm_StaggerOdd_Lead3.8mm_Vertical" H 4600 2250 50  0001 C CIN
F 3 "http://www.st.com/resource/en/datasheet/cd00000123.pdf" H 4600 2250 50  0001 C CNN
	1    4600 2250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 60F81B2E
P 3950 2150
F 0 "C1" V 3698 2150 50  0000 C CNN
F 1 "10uF" V 3789 2150 50  0000 C CNN
F 2 "" H 3988 2000 50  0001 C CNN
F 3 "~" H 3950 2150 50  0001 C CNN
	1    3950 2150
	0    1    1    0   
$EndComp
Wire Wire Line
	4100 2150 4300 2150
$Comp
L Device:C C2
U 1 1 60F87707
P 4500 1700
F 0 "C2" H 4615 1746 50  0000 L CNN
F 1 "100nF" H 4615 1655 50  0000 L CNN
F 2 "" H 4538 1550 50  0001 C CNN
F 3 "~" H 4500 1700 50  0001 C CNN
	1    4500 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C3
U 1 1 60F8C00E
P 4950 1700
F 0 "C3" H 4832 1654 50  0000 R CNN
F 1 "1000uF" H 4832 1745 50  0000 R CNN
F 2 "" H 4988 1550 50  0001 C CNN
F 3 "~" H 4950 1700 50  0001 C CNN
	1    4950 1700
	-1   0    0    1   
$EndComp
Wire Wire Line
	4500 1950 4500 1850
Wire Wire Line
	4500 1850 4950 1850
Connection ~ 4500 1850
Wire Wire Line
	4950 1550 4500 1550
$Comp
L Connector_Generic:Conn_01x03 J1
U 1 1 60F8ED9F
P 3100 2250
F 0 "J1" H 3180 2292 50  0000 L CNN
F 1 "Audio In" H 3180 2201 50  0000 L CNN
F 2 "" H 3100 2250 50  0001 C CNN
F 3 "~" H 3100 2250 50  0001 C CNN
	1    3100 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 2150 3800 2150
$Comp
L Device:R R1
U 1 1 60F8FF44
P 3250 2650
F 0 "R1" H 3320 2696 50  0000 L CNN
F 1 "470" H 3320 2605 50  0000 L CNN
F 2 "" V 3180 2650 50  0001 C CNN
F 3 "~" H 3250 2650 50  0001 C CNN
	1    3250 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 60F90CA4
P 3550 2650
F 0 "R2" H 3620 2696 50  0000 L CNN
F 1 "470" H 3620 2605 50  0000 L CNN
F 2 "" V 3480 2650 50  0001 C CNN
F 3 "~" H 3550 2650 50  0001 C CNN
	1    3550 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 2250 3400 2250
Wire Wire Line
	3400 2250 3400 2500
Wire Wire Line
	3050 2350 3700 2350
Wire Wire Line
	3700 2350 3700 2500
Connection ~ 3800 2150
Wire Wire Line
	3800 2150 3800 2800
Connection ~ 3550 2800
Wire Wire Line
	3250 2500 3400 2500
Wire Wire Line
	3550 2800 3800 2800
Wire Wire Line
	3700 2500 3550 2500
Wire Wire Line
	3250 2800 3550 2800
$Comp
L Device:R R3
U 1 1 60F970DD
P 4300 3100
F 0 "R3" H 4370 3146 50  0000 L CNN
F 1 "100" H 4370 3055 50  0000 L CNN
F 2 "" V 4230 3100 50  0001 C CNN
F 3 "~" H 4300 3100 50  0001 C CNN
	1    4300 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 60F9801D
P 4650 3250
F 0 "C4" V 4398 3250 50  0000 C CNN
F 1 "10nF" V 4489 3250 50  0000 C CNN
F 2 "" H 4688 3100 50  0001 C CNN
F 3 "~" H 4650 3250 50  0001 C CNN
	1    4650 3250
	0    1    1    0   
$EndComp
$Comp
L Device:C C5
U 1 1 60F98CF3
P 4650 3650
F 0 "C5" V 4398 3650 50  0000 C CNN
F 1 "22nF" V 4489 3650 50  0000 C CNN
F 2 "" H 4688 3500 50  0001 C CNN
F 3 "~" H 4650 3650 50  0001 C CNN
	1    4650 3650
	0    1    1    0   
$EndComp
Wire Wire Line
	4300 3250 4500 3250
Wire Wire Line
	4300 3250 4300 3650
Wire Wire Line
	4300 3650 4500 3650
Connection ~ 4300 3250
Wire Wire Line
	4800 3250 4900 3250
Wire Wire Line
	4800 3650 4900 3650
Connection ~ 4900 3250
Wire Wire Line
	4900 3250 4900 3650
$Comp
L Device:R R4
U 1 1 60F9A791
P 5350 2400
F 0 "R4" V 5143 2400 50  0000 C CNN
F 1 "220" V 5234 2400 50  0000 C CNN
F 2 "" V 5280 2400 50  0001 C CNN
F 3 "~" H 5350 2400 50  0001 C CNN
	1    5350 2400
	-1   0    0    1   
$EndComp
Wire Wire Line
	4900 2250 5350 2250
$Comp
L Device:R R5
U 1 1 60F9C869
P 5350 3050
F 0 "R5" V 5143 3050 50  0000 C CNN
F 1 "4.7" V 5234 3050 50  0000 C CNN
F 2 "" V 5280 3050 50  0001 C CNN
F 3 "~" H 5350 3050 50  0001 C CNN
	1    5350 3050
	-1   0    0    1   
$EndComp
$Comp
L Device:R R6
U 1 1 60F9D3FB
P 6250 2400
F 0 "R6" H 6180 2354 50  0000 R CNN
F 1 "1" H 6180 2445 50  0000 R CNN
F 2 "" V 6180 2400 50  0001 C CNN
F 3 "~" H 6250 2400 50  0001 C CNN
	1    6250 2400
	-1   0    0    1   
$EndComp
$Comp
L Device:CP C6
U 1 1 60F9DAA6
P 5200 2800
F 0 "C6" H 5082 2754 50  0000 R CNN
F 1 "680uF" H 5082 2845 50  0000 R CNN
F 2 "" H 5238 2650 50  0001 C CNN
F 3 "~" H 5200 2800 50  0001 C CNN
	1    5200 2800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4900 2250 4900 3250
Connection ~ 4900 2250
$Comp
L power:GND1 #PWR?
U 1 1 60FA3773
P 4500 2500
F 0 "#PWR?" H 4500 2250 50  0001 C CNN
F 1 "GND1" H 4505 2327 50  0000 C CNN
F 2 "" H 4500 2500 50  0001 C CNN
F 3 "" H 4500 2500 50  0001 C CNN
	1    4500 2500
	1    0    0    -1  
$EndComp
$Comp
L power:GND1 #PWR?
U 1 1 60FACCDA
P 5350 3300
F 0 "#PWR?" H 5350 3050 50  0001 C CNN
F 1 "GND1" H 5355 3127 50  0000 C CNN
F 2 "" H 5350 3300 50  0001 C CNN
F 3 "" H 5350 3300 50  0001 C CNN
	1    5350 3300
	1    0    0    -1  
$EndComp
$Comp
L power:GND1 #PWR?
U 1 1 60FADC2D
P 4250 1600
F 0 "#PWR?" H 4250 1350 50  0001 C CNN
F 1 "GND1" H 4255 1427 50  0000 C CNN
F 2 "" H 4250 1600 50  0001 C CNN
F 3 "" H 4250 1600 50  0001 C CNN
	1    4250 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 1550 4250 1550
Wire Wire Line
	4250 1550 4250 1600
Connection ~ 4500 1550
$Comp
L Device:CP C7
U 1 1 60FB3965
P 5700 2250
F 0 "C7" V 5955 2250 50  0000 C CNN
F 1 "2700uF" V 5864 2250 50  0000 C CNN
F 2 "" H 5738 2100 50  0001 C CNN
F 3 "~" H 5700 2250 50  0001 C CNN
	1    5700 2250
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C8
U 1 1 60FB9655
P 6250 2700
F 0 "C8" H 6365 2746 50  0000 L CNN
F 1 "100nF" H 6365 2655 50  0000 L CNN
F 2 "" H 6288 2550 50  0001 C CNN
F 3 "~" H 6250 2700 50  0001 C CNN
	1    6250 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 2250 6700 2500
Wire Wire Line
	4500 2550 4500 2500
$Comp
L Connector:Jack-DC J3
U 1 1 60FC900B
P 9100 4250
F 0 "J3" H 8870 4208 50  0000 R CNN
F 1 "Jack-DC 15v" H 8870 4299 50  0000 R CNN
F 2 "" H 9150 4210 50  0001 C CNN
F 3 "~" H 9150 4210 50  0001 C CNN
	1    9100 4250
	-1   0    0    1   
$EndComp
$Comp
L power:GND1 #PWR?
U 1 1 60FCA912
P 8800 4500
F 0 "#PWR?" H 8800 4250 50  0001 C CNN
F 1 "GND1" H 8805 4327 50  0000 C CNN
F 2 "" H 8800 4500 50  0001 C CNN
F 3 "" H 8800 4500 50  0001 C CNN
	1    8800 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 4350 8800 4500
$Comp
L Switch:SW_SPST SW1
U 1 1 60FCE744
P 8500 4150
F 0 "SW1" H 8500 4385 50  0000 C CNN
F 1 "On/Off" H 8500 4294 50  0000 C CNN
F 2 "" H 8500 4150 50  0001 C CNN
F 3 "~" H 8500 4150 50  0001 C CNN
	1    8500 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 4150 8800 4150
$Comp
L Regulator_Linear:L7812 U2
U 1 1 60FD0A53
P 8000 4450
F 0 "U2" V 7954 4555 50  0000 L CNN
F 1 "L7812" V 8045 4555 50  0000 L CNN
F 2 "" H 8025 4300 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 8000 4400 50  0001 C CNN
	1    8000 4450
	0    1    1    0   
$EndComp
Wire Wire Line
	8000 4150 8300 4150
$Comp
L power:GND1 #PWR?
U 1 1 60FD8779
P 7650 4500
F 0 "#PWR?" H 7650 4250 50  0001 C CNN
F 1 "GND1" H 7655 4327 50  0000 C CNN
F 2 "" H 7650 4500 50  0001 C CNN
F 3 "" H 7650 4500 50  0001 C CNN
	1    7650 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 4500 7650 4450
Wire Wire Line
	7650 4450 7700 4450
$Comp
L power:+12V #PWR?
U 1 1 60FDBC8D
P 8000 4900
F 0 "#PWR?" H 8000 4750 50  0001 C CNN
F 1 "+12V" H 8015 5073 50  0000 C CNN
F 2 "" H 8000 4900 50  0001 C CNN
F 3 "" H 8000 4900 50  0001 C CNN
	1    8000 4900
	-1   0    0    1   
$EndComp
Wire Wire Line
	8000 4900 8000 4850
$Comp
L power:+12V #PWR?
U 1 1 60FDF540
P 4250 1850
F 0 "#PWR?" H 4250 1700 50  0001 C CNN
F 1 "+12V" H 4265 2023 50  0000 C CNN
F 2 "" H 4250 1850 50  0001 C CNN
F 3 "" H 4250 1850 50  0001 C CNN
	1    4250 1850
	-1   0    0    1   
$EndComp
Wire Wire Line
	4250 1850 4500 1850
$Comp
L Device:C C9
U 1 1 60FE0E42
P 7250 4700
F 0 "C9" H 7365 4746 50  0000 L CNN
F 1 "100nF" H 7365 4655 50  0000 L CNN
F 2 "" H 7288 4550 50  0001 C CNN
F 3 "~" H 7250 4700 50  0001 C CNN
	1    7250 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 4550 7250 4450
Wire Wire Line
	7250 4450 7650 4450
Connection ~ 7650 4450
Wire Wire Line
	7250 4850 8000 4850
Connection ~ 8000 4850
Wire Wire Line
	8000 4850 8000 4750
$Comp
L Device:C C10
U 1 1 60FE417F
P 7250 4300
F 0 "C10" H 7365 4346 50  0000 L CNN
F 1 "220nF" H 7365 4255 50  0000 L CNN
F 2 "" H 7288 4150 50  0001 C CNN
F 3 "~" H 7250 4300 50  0001 C CNN
	1    7250 4300
	1    0    0    -1  
$EndComp
Connection ~ 7250 4450
Wire Wire Line
	7250 4150 8000 4150
Connection ~ 8000 4150
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 60FF545A
P 6900 2500
F 0 "J2" H 6980 2492 50  0000 L CNN
F 1 "Speaker" H 6980 2401 50  0000 L CNN
F 2 "" H 6900 2500 50  0001 C CNN
F 3 "~" H 6900 2500 50  0001 C CNN
	1    6900 2500
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J4
U 1 1 60FF76A7
P 3100 3700
F 0 "J4" H 3180 3742 50  0000 L CNN
F 1 "+12v" H 3180 3651 50  0000 L CNN
F 2 "" H 3100 3700 50  0001 C CNN
F 3 "~" H 3100 3700 50  0001 C CNN
	1    3100 3700
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J5
U 1 1 60FF8689
P 3100 4300
F 0 "J5" H 3180 4342 50  0000 L CNN
F 1 "GND1" H 3180 4251 50  0000 L CNN
F 2 "" H 3100 4300 50  0001 C CNN
F 3 "~" H 3100 4300 50  0001 C CNN
	1    3100 4300
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 60FF94CA
P 3450 3800
F 0 "#PWR?" H 3450 3650 50  0001 C CNN
F 1 "+12V" H 3465 3973 50  0000 C CNN
F 2 "" H 3450 3800 50  0001 C CNN
F 3 "" H 3450 3800 50  0001 C CNN
	1    3450 3800
	-1   0    0    1   
$EndComp
Wire Wire Line
	3450 3600 3450 3700
Wire Wire Line
	3100 3600 3450 3600
Wire Wire Line
	3100 3700 3450 3700
Connection ~ 3450 3700
Wire Wire Line
	3450 3800 3450 3700
Wire Wire Line
	3100 3800 3450 3800
Connection ~ 3450 3800
$Comp
L power:GND1 #PWR?
U 1 1 60FFECB6
P 3450 4450
F 0 "#PWR?" H 3450 4200 50  0001 C CNN
F 1 "GND1" H 3455 4277 50  0000 C CNN
F 2 "" H 3450 4450 50  0001 C CNN
F 3 "" H 3450 4450 50  0001 C CNN
	1    3450 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 4400 3450 4400
Wire Wire Line
	3450 4400 3450 4450
Wire Wire Line
	3100 4300 3450 4300
Wire Wire Line
	3450 4300 3450 4400
Connection ~ 3450 4400
Wire Wire Line
	3100 4200 3450 4200
Wire Wire Line
	3450 4200 3450 4300
Connection ~ 3450 4300
Connection ~ 6250 2250
Wire Wire Line
	6250 2250 6700 2250
$Comp
L power:GND1 #PWR?
U 1 1 6100C55B
P 6250 2950
F 0 "#PWR?" H 6250 2700 50  0001 C CNN
F 1 "GND1" H 6255 2777 50  0000 C CNN
F 2 "" H 6250 2950 50  0001 C CNN
F 3 "" H 6250 2950 50  0001 C CNN
	1    6250 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 2950 6250 2850
Wire Wire Line
	5850 2250 6250 2250
$Comp
L power:GND1 #PWR?
U 1 1 6100DD86
P 6700 2950
F 0 "#PWR?" H 6700 2700 50  0001 C CNN
F 1 "GND1" H 6705 2777 50  0000 C CNN
F 2 "" H 6700 2950 50  0001 C CNN
F 3 "" H 6700 2950 50  0001 C CNN
	1    6700 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 2950 6700 2600
Wire Wire Line
	4300 2350 4300 2800
Wire Wire Line
	5350 2800 5350 2550
Wire Wire Line
	5050 2800 4300 2800
Connection ~ 4300 2800
Wire Wire Line
	4300 2800 4300 2950
Wire Wire Line
	5350 2800 5350 2900
Connection ~ 5350 2800
Wire Wire Line
	5350 3200 5350 3300
Wire Wire Line
	5550 2250 5350 2250
Connection ~ 5350 2250
$Comp
L Device:Speaker LS1
U 1 1 60FC916D
P 7700 2500
F 0 "LS1" H 7870 2496 50  0000 L CNN
F 1 "Speaker" H 7870 2405 50  0000 L CNN
F 2 "" H 7700 2300 50  0001 C CNN
F 3 "~" H 7690 2450 50  0001 C CNN
	1    7700 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 2600 7500 2750
Wire Wire Line
	7500 2750 6900 2750
Wire Wire Line
	6900 2750 6900 2600
Wire Wire Line
	6700 2600 6900 2600
Connection ~ 6700 2600
Wire Wire Line
	7500 2500 7500 2300
Wire Wire Line
	7500 2300 6900 2300
Wire Wire Line
	6900 2300 6900 2500
Wire Wire Line
	6900 2500 6700 2500
Connection ~ 6700 2500
$Comp
L Regulator_Linear:L7805 U4
U 1 1 60FD18A9
P 5000 5000
F 0 "U4" V 4954 5105 50  0000 L CNN
F 1 "L7805" V 5045 5105 50  0000 L CNN
F 2 "" H 5025 4850 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 5000 4950 50  0001 C CNN
	1    5000 5000
	0    1    1    0   
$EndComp
$Comp
L power:GND1 #PWR?
U 1 1 60FD32E8
P 6450 4900
F 0 "#PWR?" H 6450 4650 50  0001 C CNN
F 1 "GND1" H 6455 4727 50  0000 C CNN
F 2 "" H 6450 4900 50  0001 C CNN
F 3 "" H 6450 4900 50  0001 C CNN
	1    6450 4900
	1    0    0    -1  
$EndComp
$Comp
L CostumParts:MP2307_Breakout U3
U 1 1 60FCFA67
P 6050 4800
F 0 "U3" H 6050 4435 50  0000 C CNN
F 1 "MP2307_Breakout" H 6050 4526 50  0000 C CNN
F 2 "" H 6050 4800 50  0001 C CNN
F 3 "" H 6050 4800 50  0001 C CNN
	1    6050 4800
	-1   0    0    1   
$EndComp
$Comp
L power:GND1 #PWR?
U 1 1 60FDF864
P 4600 5000
F 0 "#PWR?" H 4600 4750 50  0001 C CNN
F 1 "GND1" H 4605 4827 50  0000 C CNN
F 2 "" H 4600 5000 50  0001 C CNN
F 3 "" H 4600 5000 50  0001 C CNN
	1    4600 5000
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 60FE07D7
P 5050 6650
F 0 "#PWR?" H 5050 6500 50  0001 C CNN
F 1 "+5V" H 5065 6823 50  0000 C CNN
F 2 "" H 5050 6650 50  0001 C CNN
F 3 "" H 5050 6650 50  0001 C CNN
	1    5050 6650
	-1   0    0    1   
$EndComp
$Comp
L CostumParts:PQS07-S5-S5 U5
U 1 1 60FF454B
P 5150 6100
F 0 "U5" V 5154 6328 50  0000 L CNN
F 1 "PQS07-S5-S5" V 5245 6328 50  0000 L CNN
F 2 "" H 5150 6050 50  0001 C CNN
F 3 "" H 5150 6050 50  0001 C CNN
	1    5150 6100
	0    1    1    0   
$EndComp
$Comp
L power:GND2 #PWR?
U 1 1 60FFA93B
P 5250 6650
F 0 "#PWR?" H 5250 6400 50  0001 C CNN
F 1 "GND2" H 5255 6477 50  0000 C CNN
F 2 "" H 5250 6650 50  0001 C CNN
F 3 "" H 5250 6650 50  0001 C CNN
	1    5250 6650
	1    0    0    -1  
$EndComp
$Comp
L power:GND1 #PWR?
U 1 1 610065FE
P 5250 5600
F 0 "#PWR?" H 5250 5350 50  0001 C CNN
F 1 "GND1" H 5255 5427 50  0000 C CNN
F 2 "" H 5250 5600 50  0001 C CNN
F 3 "" H 5250 5600 50  0001 C CNN
	1    5250 5600
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J6
U 1 1 610070D5
P 3100 5450
F 0 "J6" H 3180 5492 50  0000 L CNN
F 1 "+5v" H 3180 5401 50  0000 L CNN
F 2 "" H 3100 5450 50  0001 C CNN
F 3 "~" H 3100 5450 50  0001 C CNN
	1    3100 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 5450 3450 5350
Connection ~ 3450 5450
Wire Wire Line
	3450 5450 3450 5550
Wire Wire Line
	3100 5350 3450 5350
Wire Wire Line
	3100 5550 3450 5550
Wire Wire Line
	3100 5450 3450 5450
Wire Wire Line
	5650 4700 5000 4700
Text HLabel 5650 4700 3    50   Input ~ 0
+8v
$Comp
L Connector_Generic:Conn_01x02 J7
U 1 1 6102C596
P 3100 6500
F 0 "J7" H 3180 6492 50  0000 L CNN
F 1 "5v/GND2" H 3180 6401 50  0000 L CNN
F 2 "" H 3100 6500 50  0001 C CNN
F 3 "~" H 3100 6500 50  0001 C CNN
	1    3100 6500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 6102D698
P 3100 6700
F 0 "#PWR?" H 3100 6550 50  0001 C CNN
F 1 "+5V" H 3115 6873 50  0000 C CNN
F 2 "" H 3100 6700 50  0001 C CNN
F 3 "" H 3100 6700 50  0001 C CNN
	1    3100 6700
	-1   0    0    1   
$EndComp
$Comp
L power:GND2 #PWR?
U 1 1 6102E340
P 3100 6350
F 0 "#PWR?" H 3100 6100 50  0001 C CNN
F 1 "GND2" H 3105 6177 50  0000 C CNN
F 2 "" H 3100 6350 50  0001 C CNN
F 3 "" H 3100 6350 50  0001 C CNN
	1    3100 6350
	-1   0    0    1   
$EndComp
Wire Wire Line
	3100 6500 3100 6350
Wire Wire Line
	3100 6600 3100 6700
Wire Wire Line
	6700 4700 6700 4850
Wire Wire Line
	6700 4850 7250 4850
Wire Wire Line
	6450 4700 6700 4700
Connection ~ 7250 4850
Wire Wire Line
	3450 5450 5000 5450
Wire Wire Line
	5000 5300 5000 5450
Connection ~ 5000 5450
Wire Wire Line
	5000 5450 5000 5600
Wire Wire Line
	4700 5000 4600 5000
$Comp
L Device:C C12
U 1 1 6106B892
P 4150 5150
F 0 "C12" H 4265 5196 50  0000 L CNN
F 1 "100nF" H 4265 5105 50  0000 L CNN
F 2 "" H 4188 5000 50  0001 C CNN
F 3 "~" H 4150 5150 50  0001 C CNN
	1    4150 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 6106BFBA
P 4150 4850
F 0 "C11" H 4265 4896 50  0000 L CNN
F 1 "220nF" H 4265 4805 50  0000 L CNN
F 2 "" H 4188 4700 50  0001 C CNN
F 3 "~" H 4150 4850 50  0001 C CNN
	1    4150 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 5000 4150 5000
Connection ~ 4600 5000
Connection ~ 4150 5000
Wire Wire Line
	4150 4700 5000 4700
Connection ~ 5000 4700
Wire Wire Line
	4150 5300 5000 5300
Connection ~ 5000 5300
$EndSCHEMATC
