0x807F
Noise_Threshold
Threshold of noise level
0x8080
NC Reserved
0x8081
NC Reserved
0x8082
Hopping_seg1_BitFreqL
Frequency hopping segment band 1 central frequency (for driver A/B)
0x8083
Hopping_seg1_BitFreqH
0x8084
Hopping_seg1_Factor
Frequency hopping segment 1 central frequency coefficient
0x8085
Hopping_seg2_BitFreqL
Frequency hopping segment band 2 central frequency (for driver A/B)
0x8086
Hopping_seg2_BitFreqH
0x8087
Hopping_seg2_Factor
Frequency hopping segment 2 central frequency coefficient
0x8088
Hopping_seg3_BitFreqL
Frequency hopping segment band 3 central frequency (for driver A/B)
0x8089
Hopping_seg3_BitFreqH
0x808A
Hopping_seg3_Factor
Frequency hopping segment 3 central frequency coefficient
0x808B
Hopping_seg4_BitFreqL
Frequency hopping segment band 4 central frequency (for driver A/B)
Single Layer Touch Screen Chip GT968
13
Copyright reserved by GOODIX
0x808C
Hopping_seg4_BitFreqH
0x808D
Hopping_seg4_Factor
Frequency hopping segment 4 central frequency coefficient
0x808E
Hopping_seg5_BitFreqL
Frequency hopping segment band 5 central frequency (for driver A/B)
0x808F
Hopping_seg5_BitFreqH
0x8090
Hopping_seg5_Factor
Frequency hopping segment 5 central frequency coefficient
0x8091
NC Reserved
0x8092
NC Reserved
0x8093
Key 1 Key 1 Position: 0-255 valid (0 means no touch, it means independent touch key when 4 of the keys are 8 multiples
0x8094
Key 2 Key 2 position
0x8095
Key 3 Key 3 position
0x8096
Key 4 Key 4 position
0x8097
Key_Area Time limit for long press(1~16 s) Touch valid interval setting: 0-15 valid
0x8098
Key_Touch_Level Key threshold of touch key
0x8099
Key_Leave_Level Key threshold of leave key
0x809A
Key_Sens KeySens_1(sensitivity coefficient of key 1, same below) KeySens_2
0x809B
Key_Sens KeySens_3 KeySens_4
0x809C
Key_Restrain The suppression time of keys when finger leave the screen
The independent button adjacent key suppression parameter
0x809D
NC Reserved
0x809E
NC Reserved
0x809F
NC Reserved
0x80A0
NC Reserved
0x80A1
NC Reserved
0x80A2
Proximity_Drv_Select Drv_Start_Ch (start channel of driving direction) Drv_End_Ch (End channel)
0x80A3
Proximity_Sens_Select Sens_Start_Ch (start channel of sensing direction) Sens_End_Ch (End channel)
0x80A4
Proximity_Touch_Level Proximity effective threshold value
0x80A5 Proximity_Leave_Level Proximity ineffective threshold value
0x80A6 Proximity_Sample_Add_TimeSampled values accumulated times.
Single Layer Touch Screen Chip GT968
14
Copyright reserved by GOODIX
s
0x80A7 Proximity_Sample_Dec_ValL Sampled values minus this parameter before accumulate.
0x80A8 Proximity_Sample_Dec_ValH
0x80A9 Proximity_Leave_Shake_Count Stop proximity after this time
0x80AA Self_Cap_Tx_gain Self-capacitance transmit gain
0x80AB Self_Cap_Rx_gain Self-capacitance receive gain
0x80AC Self_Cap_Dump_Shift Self-capacitance amplification factor
0x80AD
SCap_Diff_Up_Level_Drv The self-capacitance suppression value of suspended increase threshold(driving direction)
0x80AE
Scap_Merge_Touch_Level_Drv Self-capacitance touch level(driving direction)
0x80AF
SCap_Pulse_TimeL Self-capacitance pulse time(low byte)
0x80B0
SCap_Pulse_TimeH Self-capacitance pulse time(high byte)
0x80B1
SCap_Diff_Up_Level_Sen The self-capacitance suppression value of suspended increase threshold(sensing direction)
0x80B2
Scap_Merge_Touch_Level_Sen Self-capacitance touch level(sensing direction)
0x80B3 NC Reserved
0x80B4 NC Reserved
0x80B5 NC Reserved
0x80B6 NC Reserved
0x80B7~
0x80C0 Sensor_CH0~ Sensor_CH9 Corresponding channel no. of ITO Sensor
0x80C1~
0x80D4 NC Reserved
0x80D5~
0x80E6 Driver_CH0~ Driver_CH16 Corresponding channel no. of ITO Driver0
0x80E7~
0x80FE NC Reserved
0x80FF~
0x810F Drv0_Gain~ Drv16_Gain