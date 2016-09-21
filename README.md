# Balancer
A (semi)self-driving (fully)self-balancing robot

ToDo:

	*figure out a way (electronical preffered, mechanically different connections otherwise) to connect UNO and motor driver as a shield while still being able to use pin 2)
	EDIT: found a neat mechanical and aesthetical way to choose which pins to use
	*Gyro calibration offsets (MPU6050): acelX acelY acelZ giroX giroY giroZ -88	-290	2279	-599	39	15
	*Print out a good prototype chassis
	*Add encoder pin layout to PIN LAYOUT
	*Write a motor library (since default library doesn't work)
	*Write simple code that keeps the robot upright
	*Write simple code that enables for the robot to drive forward/backward, turn right and left
	*Write a code for controlling the robot via bluetooth with Android
	*Write a code for using encoders to keep speed/balance better; make balancing and driving code more ergonomical&better
	*Write code for mapping and then reproducing path
	*Write code for autonomous drive on a table withouth falling off nor bumping into obstacles
	*Write code for line following
	
	*Put a glass of water on robot and drive so
	
	
Done for now! Next step?
	

PIN LAYOUT:
	
	Gyro uses:
	
		*3.3V
		*GND
		*A4 (I2c) SDA
		*A5 (I2c) SCL
	
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
		*A0 - Motor 1 current sense output - NOT USED
		*A1 Motor 2 current sense output - NOT USED
		*MUST share a common ground with Arduino
	
	Encoders use:
	
		*TBA!
		

Sources:
	High frequency PWM:
		http://forum.arduino.cc/index.php/topic,117425.0.html
	
