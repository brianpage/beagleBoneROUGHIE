import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.UART as UART
import serial


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

tankLevel = "P8_01"
linPos = "P8_01"

pressureSensor = "P8_01"

rotServo = "P8_01"

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




def main():
	updateIMU()
	updateCompass()
	updateGlider()
	logData()


def updateIMU():



def actuate():
	#Low level motion controller

	if mode==POSITION:

def updateIMU():
	