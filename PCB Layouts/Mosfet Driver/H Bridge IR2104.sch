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
LIBS:ir2104
LIBS:H Bridge IR2104-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
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
$Comp
L IRF3205 Q1
U 1 1 59DB9089
P 8100 1700
F 0 "Q1" H 8350 1775 50  0000 L CNN
F 1 "IRF3205" H 8350 1700 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-220-3_Vertical" H 8350 1625 50  0001 L CIN
F 3 "" H 8100 1700 50  0001 L CNN
	1    8100 1700
	1    0    0    -1  
$EndComp
$Comp
L IRF3205 Q2
U 1 1 59DB930B
P 8100 2350
F 0 "Q2" H 8350 2425 50  0000 L CNN
F 1 "IRF3205" H 8350 2350 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-220-3_Vertical" H 8350 2275 50  0001 L CIN
F 3 "" H 8100 2350 50  0001 L CNN
	1    8100 2350
	1    0    0    -1  
$EndComp
$Comp
L D D1
U 1 1 59DBA271
P 6500 1650
F 0 "D1" H 6500 1750 50  0000 C CNN
F 1 "D" H 6500 1550 50  0000 C CNN
F 2 "Diodes_THT:D_A-405_P7.62mm_Horizontal" H 6500 1650 50  0001 C CNN
F 3 "" H 6500 1650 50  0001 C CNN
	1    6500 1650
	-1   0    0    1   
$EndComp
$Comp
L R R8
U 1 1 59DBA479
P 7550 2350
F 0 "R8" V 7630 2350 50  0000 C CNN
F 1 "R" V 7550 2350 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7480 2350 50  0001 C CNN
F 3 "" H 7550 2350 50  0001 C CNN
	1    7550 2350
	0    -1   -1   0   
$EndComp
$Comp
L VCC #PWR9
U 1 1 59DBA577
P 5900 1600
F 0 "#PWR9" H 5900 1450 50  0001 C CNN
F 1 "VCC" H 5900 1750 50  0000 C CNN
F 2 "" H 5900 1600 50  0001 C CNN
F 3 "" H 5900 1600 50  0001 C CNN
	1    5900 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR11
U 1 1 59DBA5F9
P 6000 2450
F 0 "#PWR11" H 6000 2200 50  0001 C CNN
F 1 "GND" H 6000 2300 50  0000 C CNN
F 2 "" H 6000 2450 50  0001 C CNN
F 3 "" H 6000 2450 50  0001 C CNN
	1    6000 2450
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 59DBA41B
P 7550 1700
F 0 "R7" V 7630 1700 50  0000 C CNN
F 1 "R" V 7550 1700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7480 1700 50  0001 C CNN
F 3 "" H 7550 1700 50  0001 C CNN
	1    7550 1700
	0    -1   -1   0   
$EndComp
$Comp
L VCC #PWR13
U 1 1 59DBAD5D
P 8200 1450
F 0 "#PWR13" H 8200 1300 50  0001 C CNN
F 1 "VCC" H 8200 1600 50  0000 C CNN
F 2 "" H 8200 1450 50  0001 C CNN
F 3 "" H 8200 1450 50  0001 C CNN
	1    8200 1450
	1    0    0    -1  
$EndComp
Text Label 5600 2050 2    60   ~ 0
IN1_H
$Comp
L 4N25 U1
U 1 1 59DBBF11
P 4800 1850
F 0 "U1" H 4600 2050 50  0000 L CNN
F 1 "4N25" H 4800 2050 50  0000 L CNN
F 2 "Housings_DIP:DIP-6_W7.62mm" H 4600 1650 50  0001 L CIN
F 3 "" H 4800 1850 50  0001 L CNN
	1    4800 1850
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 59DBC099
P 4350 1950
F 0 "R1" V 4430 1950 50  0000 C CNN
F 1 "R" V 4350 1950 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4280 1950 50  0001 C CNN
F 3 "" H 4350 1950 50  0001 C CNN
	1    4350 1950
	0    -1   -1   0   
$EndComp
$Comp
L R R4
U 1 1 59DBC254
P 5200 1650
F 0 "R4" V 5280 1650 50  0000 C CNN
F 1 "R" V 5200 1650 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5130 1650 50  0001 C CNN
F 3 "" H 5200 1650 50  0001 C CNN
	1    5200 1650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR6
U 1 1 59DBCBA5
P 5200 2000
F 0 "#PWR6" H 5200 1750 50  0001 C CNN
F 1 "GND" H 5200 1850 50  0000 C CNN
F 2 "" H 5200 2000 50  0001 C CNN
F 3 "" H 5200 2000 50  0001 C CNN
	1    5200 2000
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR5
U 1 1 59DBCD61
P 5200 1500
F 0 "#PWR5" H 5200 1350 50  0001 C CNN
F 1 "VCC" H 5200 1650 50  0000 C CNN
F 2 "" H 5200 1500 50  0001 C CNN
F 3 "" H 5200 1500 50  0001 C CNN
	1    5200 1500
	1    0    0    -1  
$EndComp
$Comp
L 4N25 U2
U 1 1 59DBD273
P 4800 2800
F 0 "U2" H 4600 3000 50  0000 L CNN
F 1 "4N25" H 4800 3000 50  0000 L CNN
F 2 "Housings_DIP:DIP-6_W7.62mm" H 4600 2600 50  0001 L CIN
F 3 "" H 4800 2800 50  0001 L CNN
	1    4800 2800
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 59DBD279
P 4350 2900
F 0 "R2" V 4430 2900 50  0000 C CNN
F 1 "R" V 4350 2900 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4280 2900 50  0001 C CNN
F 3 "" H 4350 2900 50  0001 C CNN
	1    4350 2900
	0    -1   -1   0   
$EndComp
$Comp
L R R5
U 1 1 59DBD27F
P 5200 2600
F 0 "R5" V 5280 2600 50  0000 C CNN
F 1 "R" V 5200 2600 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5130 2600 50  0001 C CNN
F 3 "" H 5200 2600 50  0001 C CNN
	1    5200 2600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR8
U 1 1 59DBD285
P 5200 2950
F 0 "#PWR8" H 5200 2700 50  0001 C CNN
F 1 "GND" H 5200 2800 50  0000 C CNN
F 2 "" H 5200 2950 50  0001 C CNN
F 3 "" H 5200 2950 50  0001 C CNN
	1    5200 2950
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR7
U 1 1 59DBD28F
P 5200 2450
F 0 "#PWR7" H 5200 2300 50  0001 C CNN
F 1 "VCC" H 5200 2600 50  0000 C CNN
F 2 "" H 5200 2450 50  0001 C CNN
F 3 "" H 5200 2450 50  0001 C CNN
	1    5200 2450
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x04 J1
U 1 1 59DBE71C
P 3300 2750
F 0 "J1" H 3300 2950 50  0000 C CNN
F 1 "Conn_01x04" H 3300 2450 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 3300 2750 50  0001 C CNN
F 3 "" H 3300 2750 50  0001 C CNN
	1    3300 2750
	-1   0    0    1   
$EndComp
NoConn ~ 5100 2700
NoConn ~ 5100 1750
$Comp
L GNDD #PWR1
U 1 1 59DBEFC2
P 3550 2900
F 0 "#PWR1" H 3550 2650 50  0001 C CNN
F 1 "GNDD" H 3550 2775 50  0000 C CNN
F 2 "" H 3550 2900 50  0001 C CNN
F 3 "" H 3550 2900 50  0001 C CNN
	1    3550 2900
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR19
U 1 1 59DC053D
P 9750 950
F 0 "#PWR19" H 9750 800 50  0001 C CNN
F 1 "VCC" H 9750 1100 50  0000 C CNN
F 2 "" H 9750 950 50  0001 C CNN
F 3 "" H 9750 950 50  0001 C CNN
	1    9750 950 
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR21
U 1 1 59DC05A2
P 9950 950
F 0 "#PWR21" H 9950 800 50  0001 C CNN
F 1 "VCC" H 9950 1100 50  0000 C CNN
F 2 "" H 9950 950 50  0001 C CNN
F 3 "" H 9950 950 50  0001 C CNN
	1    9950 950 
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR25
U 1 1 59DC0C56
P 10400 750
F 0 "#PWR25" H 10400 600 50  0001 C CNN
F 1 "VCC" H 10400 900 50  0000 C CNN
F 2 "" H 10400 750 50  0001 C CNN
F 3 "" H 10400 750 50  0001 C CNN
	1    10400 750 
	-1   0    0    1   
$EndComp
$Comp
L PWR_FLAG #FLG1
U 1 1 59DC0EF4
P 10400 750
F 0 "#FLG1" H 10400 825 50  0001 C CNN
F 1 "PWR_FLAG" H 10400 900 50  0000 C CNN
F 2 "" H 10400 750 50  0001 C CNN
F 3 "" H 10400 750 50  0001 C CNN
	1    10400 750 
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR26
U 1 1 59DC1DD9
P 10550 1050
F 0 "#PWR26" H 10550 900 50  0001 C CNN
F 1 "VCC" H 10550 1200 50  0000 C CNN
F 2 "" H 10550 1050 50  0001 C CNN
F 3 "" H 10550 1050 50  0001 C CNN
	1    10550 1050
	1    0    0    -1  
$EndComp
$Comp
L Screw_Terminal_01x02 J3
U 1 1 59DC1E93
P 10800 1100
F 0 "J3" H 10800 1200 50  0000 C CNN
F 1 "Screw_Terminal_01x02" H 10800 900 50  0000 C CNN
F 2 "Connectors_WAGO:WAGO_734_2pin_Straight" H 10800 1100 50  0001 C CNN
F 3 "" H 10800 1100 50  0001 C CNN
	1    10800 1100
	1    0    0    -1  
$EndComp
$Comp
L Screw_Terminal_01x02 J2
U 1 1 5A4720EB
P 9100 3000
F 0 "J2" H 9100 3100 50  0000 C CNN
F 1 "Screw_Terminal_01x02" H 9100 2800 50  0000 C CNN
F 2 "Connectors_WAGO:WAGO_734_2pin_Straight" H 9100 3000 50  0001 C CNN
F 3 "" H 9100 3000 50  0001 C CNN
	1    9100 3000
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR17
U 1 1 5A47276D
P 9100 1000
F 0 "#PWR17" H 9100 850 50  0001 C CNN
F 1 "VCC" H 9100 1150 50  0000 C CNN
F 2 "" H 9100 1000 50  0001 C CNN
F 3 "" H 9100 1000 50  0001 C CNN
	1    9100 1000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR18
U 1 1 5A472773
P 9100 1200
F 0 "#PWR18" H 9100 950 50  0001 C CNN
F 1 "GND" H 9100 1050 50  0000 C CNN
F 2 "" H 9100 1200 50  0001 C CNN
F 3 "" H 9100 1200 50  0001 C CNN
	1    9100 1200
	1    0    0    -1  
$EndComp
$Comp
L Screw_Terminal_01x02 J4
U 1 1 5A472779
P 9350 1050
F 0 "J4" H 9350 1150 50  0000 C CNN
F 1 "Screw_Terminal_01x02" H 9350 850 50  0000 C CNN
F 2 "Connectors_WAGO:WAGO_734_2pin_Straight" H 9350 1050 50  0001 C CNN
F 3 "" H 9350 1050 50  0001 C CNN
	1    9350 1050
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR2
U 1 1 5A47355A
P 3650 2750
F 0 "#PWR2" H 3650 2600 50  0001 C CNN
F 1 "+5V" H 3650 2890 50  0000 C CNN
F 2 "" H 3650 2750 50  0001 C CNN
F 3 "" H 3650 2750 50  0001 C CNN
	1    3650 2750
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR3
U 1 1 5A473D36
P 4450 1700
F 0 "#PWR3" H 4450 1550 50  0001 C CNN
F 1 "+5V" H 4450 1840 50  0000 C CNN
F 2 "" H 4450 1700 50  0001 C CNN
F 3 "" H 4450 1700 50  0001 C CNN
	1    4450 1700
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR4
U 1 1 5A474101
P 4450 2650
F 0 "#PWR4" H 4450 2500 50  0001 C CNN
F 1 "+5V" H 4450 2790 50  0000 C CNN
F 2 "" H 4450 2650 50  0001 C CNN
F 3 "" H 4450 2650 50  0001 C CNN
	1    4450 2650
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR23
U 1 1 5A474AC2
P 10150 950
F 0 "#PWR23" H 10150 800 50  0001 C CNN
F 1 "VCC" H 10150 1100 50  0000 C CNN
F 2 "" H 10150 950 50  0001 C CNN
F 3 "" H 10150 950 50  0001 C CNN
	1    10150 950 
	1    0    0    -1  
$EndComp
$Comp
L CP C3
U 1 1 5A474B43
P 9750 1100
F 0 "C3" H 9775 1200 50  0000 L CNN
F 1 "CP" H 9775 1000 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D10.0mm_P2.50mm_P5.00mm" H 9788 950 50  0001 C CNN
F 3 "" H 9750 1100 50  0001 C CNN
	1    9750 1100
	1    0    0    -1  
$EndComp
$Comp
L CP C4
U 1 1 5A474C7E
P 9950 1100
F 0 "C4" H 9975 1200 50  0000 L CNN
F 1 "CP" H 9975 1000 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D10.0mm_P2.50mm_P5.00mm" H 9988 950 50  0001 C CNN
F 3 "" H 9950 1100 50  0001 C CNN
	1    9950 1100
	1    0    0    -1  
$EndComp
$Comp
L CP C5
U 1 1 5A474CEA
P 10150 1100
F 0 "C5" H 10175 1200 50  0000 L CNN
F 1 "CP" H 10175 1000 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D10.0mm_P2.50mm_P5.00mm" H 10188 950 50  0001 C CNN
F 3 "" H 10150 1100 50  0001 C CNN
	1    10150 1100
	1    0    0    -1  
$EndComp
$Comp
L IR2104 U4
U 1 1 59DB9F5E
P 6450 2100
F 0 "U4" H 6200 1750 60  0000 C CNN
F 1 "IR2104" H 6450 2150 60  0000 C CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_LongPads" H 6650 2650 60  0001 C CNN
F 3 "" H 6650 2650 60  0001 C CNN
	1    6450 2100
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5A475AA2
P 5900 2000
F 0 "R3" V 5980 2000 50  0000 C CNN
F 1 "R" V 5900 2000 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5830 2000 50  0001 C CNN
F 3 "" H 5900 2000 50  0001 C CNN
	1    5900 2000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR20
U 1 1 5A477296
P 9750 1250
F 0 "#PWR20" H 9750 1000 50  0001 C CNN
F 1 "GND" H 9750 1100 50  0000 C CNN
F 2 "" H 9750 1250 50  0001 C CNN
F 3 "" H 9750 1250 50  0001 C CNN
	1    9750 1250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR22
U 1 1 5A4772E6
P 9950 1250
F 0 "#PWR22" H 9950 1000 50  0001 C CNN
F 1 "GND" H 9950 1100 50  0000 C CNN
F 2 "" H 9950 1250 50  0001 C CNN
F 3 "" H 9950 1250 50  0001 C CNN
	1    9950 1250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR24
U 1 1 5A477336
P 10150 1250
F 0 "#PWR24" H 10150 1000 50  0001 C CNN
F 1 "GND" H 10150 1100 50  0000 C CNN
F 2 "" H 10150 1250 50  0001 C CNN
F 3 "" H 10150 1250 50  0001 C CNN
	1    10150 1250
	1    0    0    -1  
$EndComp
$Comp
L D D3
U 1 1 5A478067
P 7550 1550
F 0 "D3" H 7550 1650 50  0000 C CNN
F 1 "D" H 7550 1450 50  0000 C CNN
F 2 "Diodes_THT:D_A-405_P7.62mm_Horizontal" H 7550 1550 50  0001 C CNN
F 3 "" H 7550 1550 50  0001 C CNN
	1    7550 1550
	1    0    0    -1  
$EndComp
$Comp
L R_Small R11
U 1 1 5A478322
P 7750 1850
F 0 "R11" H 7780 1870 50  0000 L CNN
F 1 "R_Small" H 7780 1810 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" H 7750 1850 50  0001 C CNN
F 3 "" H 7750 1850 50  0001 C CNN
	1    7750 1850
	1    0    0    -1  
$EndComp
$Comp
L R_Small R12
U 1 1 5A4784F2
P 7750 2500
F 0 "R12" H 7780 2520 50  0000 L CNN
F 1 "R_Small" H 7780 2460 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" H 7750 2500 50  0001 C CNN
F 3 "" H 7750 2500 50  0001 C CNN
	1    7750 2500
	1    0    0    -1  
$EndComp
$Comp
L D D4
U 1 1 5A478E96
P 7550 2200
F 0 "D4" H 7550 2300 50  0000 C CNN
F 1 "D" H 7550 2100 50  0000 C CNN
F 2 "Diodes_THT:D_A-405_P7.62mm_Horizontal" H 7550 2200 50  0001 C CNN
F 3 "" H 7550 2200 50  0001 C CNN
	1    7550 2200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR14
U 1 1 5A4795FF
P 8200 2700
F 0 "#PWR14" H 8200 2450 50  0001 C CNN
F 1 "GND" H 8200 2550 50  0000 C CNN
F 2 "" H 8200 2700 50  0001 C CNN
F 3 "" H 8200 2700 50  0001 C CNN
	1    8200 2700
	1    0    0    -1  
$EndComp
$Comp
L CP_Small C1
U 1 1 5A479AA6
P 7000 1900
F 0 "C1" H 7010 1970 50  0000 L CNN
F 1 "CP_Small" H 7010 1820 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.50mm" H 7000 1900 50  0001 C CNN
F 3 "" H 7000 1900 50  0001 C CNN
	1    7000 1900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR27
U 1 1 5A47AD7D
P 10550 1250
F 0 "#PWR27" H 10550 1000 50  0001 C CNN
F 1 "GND" H 10550 1100 50  0000 C CNN
F 2 "" H 10550 1250 50  0001 C CNN
F 3 "" H 10550 1250 50  0001 C CNN
	1    10550 1250
	1    0    0    -1  
$EndComp
$Comp
L IRF3205 Q3
U 1 1 5A47B88E
P 8100 3700
F 0 "Q3" H 8350 3775 50  0000 L CNN
F 1 "IRF3205" H 8350 3700 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-220-3_Vertical" H 8350 3625 50  0001 L CIN
F 3 "" H 8100 3700 50  0001 L CNN
	1    8100 3700
	1    0    0    -1  
$EndComp
$Comp
L IRF3205 Q4
U 1 1 5A47B894
P 8100 4350
F 0 "Q4" H 8350 4425 50  0000 L CNN
F 1 "IRF3205" H 8350 4350 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-220-3_Vertical" H 8350 4275 50  0001 L CIN
F 3 "" H 8100 4350 50  0001 L CNN
	1    8100 4350
	1    0    0    -1  
$EndComp
$Comp
L D D2
U 1 1 5A47B89A
P 6500 3650
F 0 "D2" H 6500 3750 50  0000 C CNN
F 1 "D" H 6500 3550 50  0000 C CNN
F 2 "Diodes_THT:D_A-405_P7.62mm_Horizontal" H 6500 3650 50  0001 C CNN
F 3 "" H 6500 3650 50  0001 C CNN
	1    6500 3650
	-1   0    0    1   
$EndComp
$Comp
L R R10
U 1 1 5A47B8A0
P 7550 4350
F 0 "R10" V 7630 4350 50  0000 C CNN
F 1 "R" V 7550 4350 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7480 4350 50  0001 C CNN
F 3 "" H 7550 4350 50  0001 C CNN
	1    7550 4350
	0    -1   -1   0   
$EndComp
$Comp
L VCC #PWR10
U 1 1 5A47B8A6
P 5900 3600
F 0 "#PWR10" H 5900 3450 50  0001 C CNN
F 1 "VCC" H 5900 3750 50  0000 C CNN
F 2 "" H 5900 3600 50  0001 C CNN
F 3 "" H 5900 3600 50  0001 C CNN
	1    5900 3600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR12
U 1 1 5A47B8AC
P 6000 4450
F 0 "#PWR12" H 6000 4200 50  0001 C CNN
F 1 "GND" H 6000 4300 50  0000 C CNN
F 2 "" H 6000 4450 50  0001 C CNN
F 3 "" H 6000 4450 50  0001 C CNN
	1    6000 4450
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 5A47B8B2
P 7550 3700
F 0 "R9" V 7630 3700 50  0000 C CNN
F 1 "R" V 7550 3700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7480 3700 50  0001 C CNN
F 3 "" H 7550 3700 50  0001 C CNN
	1    7550 3700
	0    -1   -1   0   
$EndComp
$Comp
L VCC #PWR15
U 1 1 5A47B8B8
P 8200 3450
F 0 "#PWR15" H 8200 3300 50  0001 C CNN
F 1 "VCC" H 8200 3600 50  0000 C CNN
F 2 "" H 8200 3450 50  0001 C CNN
F 3 "" H 8200 3450 50  0001 C CNN
	1    8200 3450
	1    0    0    -1  
$EndComp
$Comp
L IR2104 U3
U 1 1 5A47B8BE
P 6450 4100
F 0 "U3" H 6200 3750 60  0000 C CNN
F 1 "IR2104" H 6450 4150 60  0000 C CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_LongPads" H 6650 4650 60  0001 C CNN
F 3 "" H 6650 4650 60  0001 C CNN
	1    6450 4100
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 5A47B8C4
P 5900 4000
F 0 "R6" V 5980 4000 50  0000 C CNN
F 1 "R" V 5900 4000 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5830 4000 50  0001 C CNN
F 3 "" H 5900 4000 50  0001 C CNN
	1    5900 4000
	1    0    0    -1  
$EndComp
$Comp
L D D5
U 1 1 5A47B8CA
P 7550 3550
F 0 "D5" H 7550 3650 50  0000 C CNN
F 1 "D" H 7550 3450 50  0000 C CNN
F 2 "Diodes_THT:D_A-405_P7.62mm_Horizontal" H 7550 3550 50  0001 C CNN
F 3 "" H 7550 3550 50  0001 C CNN
	1    7550 3550
	1    0    0    -1  
$EndComp
$Comp
L R_Small R13
U 1 1 5A47B8D0
P 7750 3850
F 0 "R13" H 7780 3870 50  0000 L CNN
F 1 "R_Small" H 7780 3810 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" H 7750 3850 50  0001 C CNN
F 3 "" H 7750 3850 50  0001 C CNN
	1    7750 3850
	1    0    0    -1  
$EndComp
$Comp
L R_Small R14
U 1 1 5A47B8D6
P 7750 4500
F 0 "R14" H 7780 4520 50  0000 L CNN
F 1 "R_Small" H 7780 4460 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" H 7750 4500 50  0001 C CNN
F 3 "" H 7750 4500 50  0001 C CNN
	1    7750 4500
	1    0    0    -1  
$EndComp
$Comp
L D D6
U 1 1 5A47B8DC
P 7550 4200
F 0 "D6" H 7550 4300 50  0000 C CNN
F 1 "D" H 7550 4100 50  0000 C CNN
F 2 "Diodes_THT:D_A-405_P7.62mm_Horizontal" H 7550 4200 50  0001 C CNN
F 3 "" H 7550 4200 50  0001 C CNN
	1    7550 4200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR16
U 1 1 5A47B8E2
P 8200 4700
F 0 "#PWR16" H 8200 4450 50  0001 C CNN
F 1 "GND" H 8200 4550 50  0000 C CNN
F 2 "" H 8200 4700 50  0001 C CNN
F 3 "" H 8200 4700 50  0001 C CNN
	1    8200 4700
	1    0    0    -1  
$EndComp
$Comp
L CP_Small C2
U 1 1 5A47B8E8
P 7000 3900
F 0 "C2" H 7010 3970 50  0000 L CNN
F 1 "CP_Small" H 7010 3820 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.50mm" H 7000 3900 50  0001 C CNN
F 3 "" H 7000 3900 50  0001 C CNN
	1    7000 3900
	1    0    0    -1  
$EndComp
$Comp
L D_Zener_Small D7
U 1 1 5A47DB33
P 7850 1850
F 0 "D7" H 7850 1940 50  0000 C CNN
F 1 "D_Zener_Small" H 7850 1760 50  0000 C CNN
F 2 "Diodes_THT:D_A-405_P7.62mm_Horizontal" V 7850 1850 50  0001 C CNN
F 3 "" V 7850 1850 50  0001 C CNN
	1    7850 1850
	0    1    1    0   
$EndComp
$Comp
L D_Zener_Small D8
U 1 1 5A47DBB2
P 7850 2500
F 0 "D8" H 7850 2590 50  0000 C CNN
F 1 "D_Zener_Small" H 7850 2410 50  0000 C CNN
F 2 "Diodes_THT:D_A-405_P7.62mm_Horizontal" V 7850 2500 50  0001 C CNN
F 3 "" V 7850 2500 50  0001 C CNN
	1    7850 2500
	0    1    1    0   
$EndComp
$Comp
L D_Zener_Small D9
U 1 1 5A47EE55
P 7850 3850
F 0 "D9" H 7850 3940 50  0000 C CNN
F 1 "D_Zener_Small" H 7850 3760 50  0000 C CNN
F 2 "Diodes_THT:D_A-405_P7.62mm_Horizontal" V 7850 3850 50  0001 C CNN
F 3 "" V 7850 3850 50  0001 C CNN
	1    7850 3850
	0    1    1    0   
$EndComp
$Comp
L D_Zener_Small D10
U 1 1 5A47EEC2
P 7850 4500
F 0 "D10" H 7850 4590 50  0000 C CNN
F 1 "D_Zener_Small" H 7850 4410 50  0000 C CNN
F 2 "Diodes_THT:D_A-405_P7.62mm_Horizontal" V 7850 4500 50  0001 C CNN
F 3 "" V 7850 4500 50  0001 C CNN
	1    7850 4500
	0    1    1    0   
$EndComp
Wire Wire Line
	6050 2350 6000 2350
Wire Wire Line
	6000 2350 6000 2450
Wire Wire Line
	8200 1450 8200 1500
Wire Wire Line
	5600 2050 6050 2050
Wire Wire Line
	5100 1850 5600 1850
Wire Wire Line
	5200 1850 5200 1800
Wire Wire Line
	5100 1950 5200 1950
Wire Wire Line
	5200 1950 5200 2000
Wire Wire Line
	5200 2800 5200 2750
Wire Wire Line
	5100 2900 5200 2900
Wire Wire Line
	5200 2900 5200 2950
Connection ~ 5200 1850
Connection ~ 5200 2800
Wire Wire Line
	5600 1850 5600 2050
Wire Wire Line
	5100 2800 5550 2800
Wire Wire Line
	3500 2850 3550 2850
Wire Wire Line
	3550 2850 3550 2900
Wire Wire Line
	10600 1100 10550 1100
Wire Wire Line
	10550 1100 10550 1050
Wire Wire Line
	10600 1200 10550 1200
Wire Wire Line
	10550 1200 10550 1250
Wire Wire Line
	9150 1050 9100 1050
Wire Wire Line
	9100 1050 9100 1000
Wire Wire Line
	9150 1150 9100 1150
Wire Wire Line
	9100 1150 9100 1200
Wire Wire Line
	3650 2750 3500 2750
Wire Wire Line
	3500 2550 4000 2550
Wire Wire Line
	4000 2550 4000 1950
Wire Wire Line
	4000 1950 4200 1950
Wire Wire Line
	3500 2650 4100 2650
Wire Wire Line
	4100 2650 4100 2900
Wire Wire Line
	4100 2900 4200 2900
Wire Wire Line
	4500 2700 4450 2700
Wire Wire Line
	4450 2700 4450 2650
Wire Wire Line
	5900 2150 5900 2200
Wire Wire Line
	5900 2200 6050 2200
Wire Wire Line
	7700 1700 7900 1700
Connection ~ 7750 1700
Wire Wire Line
	7750 1950 7750 2050
Wire Wire Line
	8200 1900 8200 2150
Wire Wire Line
	7700 2350 7900 2350
Wire Wire Line
	7750 2200 7750 2400
Connection ~ 7750 2350
Wire Wire Line
	7750 2600 7750 2650
Wire Wire Line
	7750 2650 8200 2650
Wire Wire Line
	8200 2550 8200 2700
Connection ~ 8200 2000
Wire Wire Line
	7700 2200 7750 2200
Wire Wire Line
	7700 1550 7750 1550
Wire Wire Line
	7750 1550 7750 1750
Wire Wire Line
	7400 1550 7350 1550
Wire Wire Line
	7350 1550 7350 1700
Wire Wire Line
	7250 1700 7400 1700
Wire Wire Line
	7400 2200 7350 2200
Wire Wire Line
	7350 2200 7350 2350
Wire Wire Line
	6900 2350 7400 2350
Wire Wire Line
	6900 2200 7250 2200
Wire Wire Line
	7250 2200 7250 1700
Connection ~ 7350 1700
Connection ~ 7350 2350
Connection ~ 8200 2650
Wire Wire Line
	6650 1650 7000 1650
Wire Wire Line
	7000 1650 7000 1800
Wire Wire Line
	7000 1800 6900 1800
Connection ~ 7000 1800
Wire Wire Line
	7000 2050 7000 2000
Connection ~ 8200 2050
Connection ~ 7000 2050
Wire Wire Line
	5900 1600 5900 1850
Wire Wire Line
	6050 1800 5900 1800
Connection ~ 5900 1800
Wire Wire Line
	6350 1650 5900 1650
Connection ~ 5900 1650
Wire Wire Line
	6050 4350 6000 4350
Wire Wire Line
	6000 4350 6000 4450
Wire Wire Line
	8200 3450 8200 3500
Wire Wire Line
	5550 4050 6050 4050
Wire Wire Line
	5900 4150 5900 4200
Wire Wire Line
	5900 4200 6050 4200
Wire Wire Line
	7700 3700 7900 3700
Connection ~ 7750 3700
Wire Wire Line
	7750 3950 7750 4050
Wire Wire Line
	8200 3900 8200 4150
Wire Wire Line
	7700 4350 7900 4350
Wire Wire Line
	7750 4200 7750 4400
Connection ~ 7750 4350
Wire Wire Line
	7750 4600 7750 4650
Wire Wire Line
	8200 4550 8200 4700
Wire Wire Line
	7700 4200 7750 4200
Wire Wire Line
	7700 3550 7750 3550
Wire Wire Line
	7750 3550 7750 3750
Wire Wire Line
	7400 3550 7350 3550
Wire Wire Line
	7350 3550 7350 3700
Wire Wire Line
	7250 3700 7400 3700
Wire Wire Line
	7400 4200 7350 4200
Wire Wire Line
	7350 4200 7350 4350
Wire Wire Line
	6900 4350 7400 4350
Wire Wire Line
	6900 4200 7250 4200
Wire Wire Line
	7250 4200 7250 3700
Connection ~ 7350 3700
Connection ~ 7350 4350
Connection ~ 8200 4650
Wire Wire Line
	6650 3650 7000 3650
Wire Wire Line
	7000 3650 7000 3800
Wire Wire Line
	7000 3800 6900 3800
Connection ~ 7000 3800
Wire Wire Line
	7000 4050 7000 4000
Connection ~ 8200 4050
Connection ~ 7000 4050
Wire Wire Line
	5900 3600 5900 3850
Wire Wire Line
	6050 3800 5900 3800
Connection ~ 5900 3800
Wire Wire Line
	6350 3650 5900 3650
Connection ~ 5900 3650
Wire Wire Line
	5550 2800 5550 4050
Wire Wire Line
	8650 2050 8650 3000
Wire Wire Line
	8650 3000 8900 3000
Wire Wire Line
	8650 4050 8650 3100
Wire Wire Line
	8650 3100 8900 3100
Wire Wire Line
	7850 1750 7850 1700
Connection ~ 7850 1700
Wire Wire Line
	7850 1950 7850 2000
Wire Wire Line
	7850 2000 8200 2000
Wire Wire Line
	6900 2050 8650 2050
Connection ~ 7750 2050
Wire Wire Line
	7850 2400 7850 2350
Connection ~ 7850 2350
Wire Wire Line
	7850 2600 7850 2650
Connection ~ 7850 2650
Wire Wire Line
	6900 4050 8650 4050
Connection ~ 7750 4050
Wire Wire Line
	7850 3750 7850 3700
Connection ~ 7850 3700
Wire Wire Line
	7850 3950 7850 4050
Connection ~ 7850 4050
Wire Wire Line
	7850 4400 7850 4350
Connection ~ 7850 4350
Wire Wire Line
	7750 4650 8200 4650
Wire Wire Line
	7850 4600 7850 4650
Connection ~ 7850 4650
Wire Wire Line
	4450 1700 4450 1750
Wire Wire Line
	4450 1750 4500 1750
$EndSCHEMATC