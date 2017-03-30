import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.UART as UART
import serial
from vnpy import *

#Set pin numbers
motAPWM = "P8_01"
motBPWM = "P8_01"
motAConf1 = "P8_01"
motAConf2 = "P8_01"
motStdby = "P8_01"
motBConf1 = "P8_01"
motBConf2 = "P8_01"

pumpOn = "P8_01"
pumpDir = "P8_01"

tankLevelPin = "P8_01"
linPosPin = "P8_01"

pressureSensorPin = "P8_01"

rotServoPin = "P8_01"

PWM.start(motAPWM,50)
PWM.start(motBPWM,50)
PWM.set_duty_cycle(motAPWM,0)
PWM.set_duty_cycle(motBPWM,0)

GPIO.setup(motAConf1,GPIO.OUT)
GPIO.setup(motAConf2,GPIO.OUT)
GPIO.setup(motBConf1,GPIO.OUT)
GPIO.setup(motBConf2,GPIO.OUT)
GPIO.setup(pumpOn,GPIO.OUT)
GPIO.setup(pumpDir,GPIO.OUT)

ADC.setup()

UART.setup("UART1")

imu = VnSensor()
imu.connect('/dev/tty01',baudrate=115200)

ypr
gps

gliderLinPos
gliderTankPos
gliderPressure

def main():
	updateIMU()
	updateCompass()
	updateGlider()
	logData()


def updateIMU():
	ypr = imu.read_yaw_pitch_roll()
	gps = imu.read_gps_solution_lla()
	gpsEst=imu.read_ins_solution_lla()


def actuate():
	#Low level motion controller

	if mode==POSITION:

def updateGlider():
	gliderLinPos=ADC.read(linPosPin)
	gliderTankPos=ADC.read(tankLevelPin)
	gliderPressure=ADC.read(pressureSensorPin)
	
