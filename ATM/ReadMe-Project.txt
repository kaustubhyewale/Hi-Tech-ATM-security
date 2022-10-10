
Project Name:- ATM Security System
Group Name:- Rutherford

Project Information:
			This Project is implemented for providing security to the ATM Machine.  If anyone is trying to steal the ATM machine at that time the message will be send to the police and security office of the respective bank.  If any bank employee wants to do any changes in the machine then he/she can scan the ID which is given by the bank.If the ID is authorised and matched then our system will allow him to do his/her work without any interrupt but in case if the ID is not matched, our system will send the message to the police and security office of the bank and the shutter will close. 



Components Used:
Software: 
	1.Raspbian Operating System
	2.Python Complier
	3.Libraries
		> mfrc522
		> therading
		> Rpi.GPIO	
		> os
		> smbus
		> time
	
HardWare:
	1.Raspberry 3 b+ board
	2.Gyroscope sensor(MPU6050)
	3.GSM module
	4.Servo motor
	5.vibration Sensor
	6.RFID sensor



 
Sensor Information :

1.Gyroscope seinsor(MPU6050):
		
		The MPU-6050 devices combine a 3-axis gyroscope and a 3-axis accelerometer on the same silicon die, together with an onboard Digital Motion Processor(DMP), which processes complex 6-axis MotionFusion algorithms.  The device can access external magnetometers or other sensors through an auxiliary master I²C bus.

	Protocol used: Inter-integrated circuit(I2C)

	Pins and connection: 
				
		PIN		raspberry Pi(Board)
		Vcc		Pin 1(3.3V)
		GND		Pin 6(GND)
		SDA		Pin 3	
		SCL		Pin 5
		XDA		NC
		xCL		NC
		AD0		NC
		INT		NC

	https://www.electronicwings.com/sensors-modules/mpu6050-gyroscope-accelerometer-temperature-sensor-module


2.GSM Module:

			A GSM module or a GPRS module is a chip or circuit that will be used to establish communication between a mobile device or a computing machine and a GSM or GPRS system. The modem (modulator-demodulator) is a critical part here.These modules consist of a GSM module or GPRS modem powered by a power supply circuit and communication interfaces (like RS-232,USB 2.0, and others) for computer. A GSM modem can be a dedicated modem device with a serial, USB or Bluetooth connection, or it can be a mobile phone that provides GSM modem capabilities.  The MODEM needs AT commands, for interacting with processor or controller, which are communicated through serial communication. These commands are sent by the controller/processor. The MODEM sends back a result after it receives a command. Different AT commands supported by the MODEM can be sent by the processor/controller/computer to interact with the GSM and GPRS cellular network.
	
	Uses of GSM Module:
		> Read, write and delete SMS messages.		
  		> Send SMS messages.
		> Monitor the signal strength.
		> Monitor the charging status and charge level of the battery.	
		> Read, write and search phone book entries.

	https://www.factoryforward.com/sim800l-gsm-module-arduino-commands-library/



3.Servo motor
		Servo motor is a DC motor which operates on 5 v.  It has three pins brown ,red and orange.  In our project, two servo moters are used to control the shutter lock 
		
		Servo motor 1:	
			pin  		connection
			Red		5 V
			Brown 		GND 
			orange		Pin 13 (GPIO 27)
		
		Servo motor 2:
			pin		connection
			Red		5 V
			Brown		GND
			orange		Pin 15(GPIO 22)

		https://circuitdigest.com/article/servo-motor-basics


4.Vibration sensor:
		
		The Grove - Vibration Sensor (SW-420) is a high sensitivity non-directional vibration sensor. When the module is stable, the circuit is turned on and the output is high. When the movement or vibration occurs, the circuit will be briefly disconnected and output low.  At the same time, you can also adjust the sensitivity according to your own needs. This sensor is used to detect the vibrations or any type of impact on ATM machine
		
		connections: 
			
			pin		connections
			Vcc	-	3.3 V
			GND 	-	GND(pin 6)
			D0	-	Pin 11(GPIO 17)
		
		http://www.piddlerintheroot.com/vibration-sensor/
			

5.RFID sensor
		RFID Sensor is mfrc522.  It is used to scan the tag. 
		
		Protocol Used:- I2C,SPI
		
		connections:
			Sensor pin 	Raspberry(Board)
			SDA	-	Pin 24(SDA) 
			SCL 	-	Pin 23(SCL)
			MOSI 	-	Pin 19(MOSI)
			MISO	-	Pin 21(MISO)
			RSI	-	Pin 22
			GND 	-	Pin 6(GND)
			Vcc	-	PIn 1(3.3 V)			
			
		https://www.raspberrypi-spy.co.uk/wp-content/uploads/2018/02/rc522_rfid_raspberry_pi_wiring.png
		https://www.raspberrypi-spy.co.uk/2018/02/rc522-rfid-tag-read-raspberry-pi/

		
		








 
