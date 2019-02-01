EESchema Schematic File Version 4
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 7650 1000 1700 1700
U 596138FA
F0 "motor_part" 60
F1 "motor_part.sch" 60
F2 "BRAKE_1" I L 7650 1150 60 
F3 "TACHO_1" I L 7650 1300 60 
F4 "DIAG_1" I L 7650 1450 60 
F5 "DIR_1" I L 7650 1600 60 
F6 "EN_1" I L 7650 1750 60 
F7 "BRAKE_2" I L 7650 2000 60 
F8 "TACHO_2" I L 7650 2150 60 
F9 "DIAG_2" I L 7650 2300 60 
F10 "DIR_2" I L 7650 2450 60 
F11 "EN_2" I L 7650 2600 60 
$EndSheet
$Sheet
S 5050 900  1600 2450
U 59613C4C
F0 "uC" 60
F1 "uC.sch" 60
F2 "Ch_A1" I L 5050 3250 60 
F3 "Ch_B1" I L 5050 3100 60 
F4 "Ch_A2" I R 6650 3250 60 
F5 "Ch_B2" I R 6650 3100 60 
F6 "BRAKE_1" I R 6650 1150 60 
F7 "TACHO_1" I R 6650 1300 60 
F8 "DIAG_1" I R 6650 1450 60 
F9 "DIR_1" I R 6650 1600 60 
F10 "EN_1" I R 6650 1750 60 
F11 "BRAKE_2" I R 6650 2000 60 
F12 "DIR_2" I R 6650 2450 60 
F13 "DIAG_2" I R 6650 2300 60 
F14 "TACHO_2" I R 6650 2150 60 
F15 "EN_2" I R 6650 2600 60 
$EndSheet
$Sheet
S 2750 1100 1500 1800
U 59613F6F
F0 "power_part" 60
F1 "power_part.sch" 60
$EndSheet
$Sheet
S 5050 3950 2350 800 
U 5962A1C1
F0 "encodeurs" 60
F1 "encodeurs.sch" 60
F2 "Ch_A1" I L 5050 4100 60 
F3 "Ch_B1" I L 5050 4300 60 
F4 "Ch_A2" I R 7400 4100 60 
F5 "Ch_B2" I R 7400 4300 60 
$EndSheet
Wire Wire Line
	5050 3250 4950 3250
Wire Wire Line
	4950 3250 4950 4100
Wire Wire Line
	4950 4100 5050 4100
Wire Wire Line
	5050 3100 4850 3100
Wire Wire Line
	4850 3100 4850 4300
Wire Wire Line
	4850 4300 5050 4300
Wire Wire Line
	6650 3250 7500 3250
Wire Wire Line
	7500 3250 7500 4100
Wire Wire Line
	7500 4100 7400 4100
Wire Wire Line
	6650 3100 7600 3100
Wire Wire Line
	7600 3100 7600 4300
Wire Wire Line
	7600 4300 7400 4300
Wire Wire Line
	6650 1150 7650 1150
Wire Wire Line
	6650 1300 7650 1300
Wire Wire Line
	6650 1450 7650 1450
Wire Wire Line
	6650 1600 7650 1600
Wire Wire Line
	6650 1750 7650 1750
Wire Wire Line
	7650 2000 6650 2000
Wire Wire Line
	6650 2150 7650 2150
Wire Wire Line
	6650 2300 7650 2300
Wire Wire Line
	6650 2450 7650 2450
Wire Wire Line
	7650 2600 6650 2600
$Comp
L Propulsion_control:Trou-3 U1
U 1 1 5A340DE3
P 2000 4150
F 0 "U1" H 2000 4150 60  0000 C CNN
F 1 "Trou-3" H 2000 4300 60  0000 C CNN
F 2 "maxon:Trou-3" H 2000 4150 60  0001 C CNN
F 3 "" H 2000 4150 60  0000 C CNN
	1    2000 4150
	1    0    0    -1  
$EndComp
$Comp
L Propulsion_control:Trou-3 U2
U 1 1 5A340F1E
P 2000 4400
F 0 "U2" H 2000 4400 60  0000 C CNN
F 1 "Trou-3" H 2000 4550 60  0000 C CNN
F 2 "maxon:Trou-3" H 2000 4400 60  0001 C CNN
F 3 "" H 2000 4400 60  0000 C CNN
	1    2000 4400
	1    0    0    -1  
$EndComp
$Comp
L Propulsion_control:Trou-3 U3
U 1 1 5A340FE1
P 2000 4700
F 0 "U3" H 2000 4700 60  0000 C CNN
F 1 "Trou-3" H 2000 4850 60  0000 C CNN
F 2 "maxon:Trou-3" H 2000 4700 60  0001 C CNN
F 3 "" H 2000 4700 60  0000 C CNN
	1    2000 4700
	1    0    0    -1  
$EndComp
$Comp
L Propulsion_control:Trou-3 U4
U 1 1 5A3410AC
P 2000 5000
F 0 "U4" H 2000 5000 60  0000 C CNN
F 1 "Trou-3" H 2000 5150 60  0000 C CNN
F 2 "maxon:Trou-3" H 2000 5000 60  0001 C CNN
F 3 "" H 2000 5000 60  0000 C CNN
	1    2000 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5A34117D
P 1300 5200
F 0 "#PWR01" H 1300 4950 50  0001 C CNN
F 1 "GND" H 1300 5050 50  0000 C CNN
F 2 "" H 1300 5200 50  0000 C CNN
F 3 "" H 1300 5200 50  0000 C CNN
	1    1300 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 4150 1300 4150
Wire Wire Line
	1300 4150 1300 4400
Wire Wire Line
	1700 4400 1300 4400
Connection ~ 1300 4400
Wire Wire Line
	1700 4700 1300 4700
Connection ~ 1300 4700
Wire Wire Line
	1700 5000 1300 5000
Connection ~ 1300 5000
Wire Wire Line
	1300 4400 1300 4700
Wire Wire Line
	1300 4700 1300 5000
Wire Wire Line
	1300 5000 1300 5200
$EndSCHEMATC
