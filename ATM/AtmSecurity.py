'''
		Project Name:- ATM Security System
		Group Name:-   RutherFord 
		
'''

#import Libraries
import RPi.GPIO as GPIO
import serial
from mfrc522 import SimpleMFRC522
import os, time
import threading		# for thread
import smbus			#import SMBus module of I2C
from time import sleep


GPIO.setwarnings(False)


#Varible declaration
c2w_rutherford_channel = 17    #SW 420
c2w_rutherford_authTag = 868592376076  #authincated Tag value
c2w_rutherford_servoPIN = 22  #BOARD 15
c2w_rutherford_servoPIN2 = 27    #BOARD 13


#Set the pin mode
GPIO.setmode(GPIO.BCM)
GPIO.setup(c2w_rutherford_servoPIN, GPIO.OUT)
GPIO.setup(c2w_rutherford_servoPIN2, GPIO.OUT)
GPIO.setup(c2w_rutherford_channel, GPIO.IN)

c2w_rutherford_port = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=1)
c2w_rutherford_reader = SimpleMFRC522()

c2w_rutherford_Servo1 = GPIO.PWM(c2w_rutherford_servoPIN, 50)# GPIO 17 for PWM with 50Hz
c2w_rutherford_Servo2 = GPIO.PWM(c2w_rutherford_servoPIN2,50)

c2w_rutherford_Servo1.start(7.5)# Initialization
c2w_rutherford_Servo2.start(7.5)# Initialization

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

#initilize the MPU registers
def MPU_Init():
        #write to sample rate register
        c2w_rutherford_bus.write_byte_data(c2w_rutherford_Device_Address, SMPLRT_DIV, 7)
        #Write to power management register
        c2w_rutherford_bus.write_byte_data(c2w_rutherford_Device_Address, PWR_MGMT_1, 1)
        #Write to Configuration register
        c2w_rutherford_bus.write_byte_data(c2w_rutherford_Device_Address, CONFIG, 0)
        #Write to Gyro configuration register
        c2w_rutherford_bus.write_byte_data(c2w_rutherford_Device_Address, GYRO_CONFIG, 24)
        #Write to interrupt enable register
        c2w_rutherford_bus.write_byte_data(c2w_rutherford_Device_Address, INT_ENABLE, 1)

#function to read the data from the registers
def read_raw_data(addr):
        #Accelero and Gyro value are 16-bit
        high = c2w_rutherford_bus.read_byte_data(c2w_rutherford_Device_Address, addr)
        low = c2w_rutherford_bus.read_byte_data(c2w_rutherford_Device_Address, addr+1)
        #concatenate higher and lower value
        value = ((high << 8) | low)
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


#function to get the value from the sensor
def sensor(var):
    while True:
        if var == 1:
                sleep(15)
        else :
	#read the raw data
                gyro_x = read_raw_data(GYRO_XOUT_H)
                gyro_y = read_raw_data(GYRO_YOUT_H)
                gyro_z = read_raw_data(GYRO_ZOUT_H)
	
	#convert it into readable format
                Gx = gyro_x/131.0
                Gy = gyro_y/131.0
                Gz = gyro_z/131.0

	 #print the reading
                print("Gx=%.2f" %Gx)
                print("Gy=%.2f" %Gy)
                print("Gz=%.2f" %Gz)
                sleep(3)

	#check for movement
                if Gx>=6 or Gy>=6 or Gz>=6 or GPIO.input(c2w_rutherford_channel):
                        print("movement detected")
                        c2w_rutherford_Servo1.ChangeDutyCycle(7.5)
                        c2w_rutherford_Servo2.ChangeDutyCycle(7.5)
                        time.sleep(0.5)
                        
                        c2w_rutherford_Servo1.ChangeDutyCycle(2)
                        c2w_rutherford_Servo2.ChangeDutyCycle(12)
                        time.sleep(3)
                        
                        c2w_rutherford_Servo1.ChangeDutyCycle(7.5)
                        c2w_rutherford_Servo2.ChangeDutyCycle(7.5)
                        time.sleep(0.5)
                        sendSMS()
                        
#Function to scan the RFID tag
def rfid():
    while True:
       id, text = c2w_rutherford_reader.read()
       if id == c2w_rutherford_authTag :
           print("wait for ...10.. minutes")
           

#if unauthorised movement is detected the send a msg
def sendSMS():
        c2w_rutherford_port.write('AT\r'.encode())
        rcv = c2w_rutherford_port.read(10)
        time.sleep(5)
        print (rcv)
        time.sleep(1)

        c2w_rutherford_port.write('ATE0'+'\r')      # Disable the Echo
        rcv = c2w_rutherford_port.read(10)
        time.sleep(5)
        print (rcv)
        time.sleep(1)

        c2w_rutherford_port.write('AT+CMGF=1'+'\r')  # Select Message format as Text mode 
        rcv = c2w_rutherford_port.read(10)
        time.sleep(1)
        print (rcv)
        time.sleep(1)

        	
        pc2w_rutherford_port.write('AT+CMGS="+918097924037"'+'\r')	#send a msg to specified number
        rcv = c2w_rutherford_port.read(10)
        time.sleep(5)
        print (rcv)
        time.sleep(1)

        c2w_rutherford_port.write('Sir tumch ATM choral.....'+'\r')  # Message 
        rcv = c2w_rutherford_port.read(10)
        print (rcv)

        c2w_rutherford_port.write("\x1A") # Enable to send SMS
        for i in range(10):
                rcv = c2w_rutherford_port.read(10)
                print (rcv)

    
c2w_rutherford_bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards

#MPU 6050 address i.e 1101000
c2w_rutherford_Device_Address = 0x68   # MPU6050 device address
MPU_Init()


if __name__ == "__main__":

    #threads to excute the rfid and sensor function    
    thread1 = threading.Thread(target = rfid)
    thread2 = threading.Thread(target = sensor,args=(0,))

    #start the thread
    thread2.start()
    thread1.start()
        
                       

