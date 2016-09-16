# Balancer
A (semi)self-driving (fully)self-balancing robot

ToDo:
*figure out a way (electronical preffered, mechanically different connections otherwise) to connect UNO and motor driver as a shield while still being able to use pin 2)

Gyro calibration offsets (MPU6050): acelX acelY acelZ giroX giroY giroZ -88	-290	2279	-599	39	15


====================================================== :)
PIN LAYOUT:

Gyro uses:
*3.3V
*GND
*A4 (I2c)
*A5 (I2c)

Bluetooth uses:
*3.3V
*GND
*D0(RX)
*D1(TX)

Motor shield uses:
*D2 - Motor 1 direction input A
*D4 - Motor 1 direction input B
*D6 - Motor 1 enable input/fault output
*D7 - Motor 2 direction input A
*D8 - Motor 2 direction input B
*D9 - Motor 1 speed input
*D10 - Motor 2 speed input
*D12 - Motor 2 enable input/fault output
*A0 - Motor 1 current sense output
*A1 Motor 2 current sense output

Comments:
Most pins on motor driver are not required, and all but PWM pins can be remapped (and even those, if deemed necessary). This has to be taken into consideration when choosing distance sensors. If currentSensing is not used, it is possible to use up to 4 distance traditional, analogRead sensors. If more are deemed necessary, other type of connection will be required for sensors.
======================================================= :)
