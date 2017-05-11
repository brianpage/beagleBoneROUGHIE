import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.UART as UART
import serial
from vnpy import *
#from Adafruit_I2C import Adafruit_I2C
import timeit


#Set pin numbers
motAPWM = 22
motBPWM = "P8_01"
motAConf1 = "P8_24"
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

#define glide states
GLIDE = 0
RESET = 1
START = 2
STOP = 3
ROLLTEST = 4
ROLLSTART = 5
FLOAT = 6

POSITION = 1
PWM = 2

DOWNGLIDE = 1
NEUTRAL = 2
UPGLIDE = 3

#i2c = Adafruit_I2C(0x1E)

PWM.start("P9_14", 50)
PWM.start(motBPWM, 50)
PWM.start(rotServoPin,50)
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

running=1

start = timeit.timeit()

while(running==1):
	updateIMU()
	updateCompass()
	updateGlider()
	if(SDgo):
		logData()

	newCommand=readSerial()
	if newCommand != GLIDE:
		command=newCommand

	if command==START:
		if checkPump(paramTankMid)&checkMass(paramLinMid):
			completedGlides=0
			t0=millis()
			tstart=t0
			rollI=0
			rotOutput=0
			lastUpAngle=0
			lastDownAngle=0
			downLoops=0
			upLoops=0
			currentState=DOWNGLIDE
			flag=false
			command=GLIDE
		else:
			actuate(paramLinMid,paramRotMid,paramTankMid,POSITION)

	if command==STOP:
		turnOff()

	if command==GLIDE:
		completedGlides=completedGlides+sawtooth(lin,rot,pump,mode)
		actuate(lin,rot,pump,mode)
		if completedGlides>=paramNumberofGlides:
			command=FLOAT

	if command==ROLLSTART:
		tstart=millis()
		rollI=0
		rotStor=0
		rollOutput=0
		gliderRunTime=0
		command=ROLLTEST

	if command==ROLLTEST:
		if gliderRunTime<30000:
			rollAngle=0
		elif gliderRunTime<45000:
			rollAngle=paramRollover
		elif gliderRunTime<60000:
			rollAngle=-paramRollover
		elif gliderRunTime<75000:
			rollAngle=paramRollover
		elif gliderRunTime<90000:
			rollAngle=-paramRollover
		elif gliderRunTime<105000:
			rollAngle=paramRollover
		elif gliderRunTime<120000:
			rollAngle=-paramRollover
		elif gliderRunTime<135000:
			rollAngle=paramRollover
		elif gliderRunTime<150000:
			rollAngle=-paramRollover
		else:
			command=RESET

		if turnFeedback:
			rollAngle=rotPID(rollAngle)

		actute(paramLinMid,rollAngle,paramTankMid,POSITION)


def sawtooth():
	pumpDone
	glideCompleted=0
	if currentState==DOWNGLIDE:
		pump=paramTankBackLimit
		checkFF(pump,paramLinNoseDownTarget,flag)

		if feedforward&flag==0:
			linMode=POSTITION
			lin=paramDownFeedforward
		elif linPID==0:
			linMode=POSTITION
			lin=paramDownFeedforward
		else:
			linMode=PWM
			lin=paramLinNoseDownTarget
		if delayRoll==0:
			if circle|(dubin&(millis()-t0<paramDubinTime)):
				rot=paramRollover
			else:
				rot=0
		else:
			if (flag&(circle|(dubin&(millis()-t0>paramDubinTime)))):
				rot=paramRollover
			else:
				rot=0
		if headingControl:
			rot=headingFeedback(currentState,rotStorage)
		if millis()-t0>paramDesTime-1000:
			downLoops=downLoops+1
			lastDownangle=lastDownangle+imuPitch
		if millis()-t0>paramDesTime:
			currentState=NEUTRAL
			nextState=UPGLIDE
			rotStorage=rot
			t0=millis()

	if currentState==NEUTRAL:
		pump=paramTankMid
		if feedforward&millis()-t0>pumpTime:
			if nextState==UPGLIDE:
				lin=paramUpFeedforward
			if nextState==DOWNGLIDE:
				lin=paramDownFeedforward
			mode=POSITION
		else:
			lin=0
			mode=PWM
		rot=rotStorage
		if millis()-t0>paramNeutralTime:
			currentState=nextState
			t0=millis()
		flag=0

	if currentState==UPGLIDE:
		if millis()-t0>paramRiseTime-1000:
			upLoops=upLoops+1
			lastUpAngle=lastUpAngle+imuPitch
		pump=paramTankFrontLimit
		checkFF(pump,paramLinNoseUpTarget,flag)
		if(feedforward&flag==0)|(linPID==0):
			linMode=POSITION
			lin=paramUpFeedforward
		else:
			linMode=PWM
			lin=paramLinNoseUpTarget

		if circle:
			rot=-paramRollover
		elif dubin&(millis()-t0<paramDubinTime):
			rot=paramRollover
		else:
			rot=0

		if delayRoll==0:
			if circle:
				rot=-paramRollover
			elif dubin&(millis()-t0<paramDubinTime):
				rot=paramRollover
			else:
				rot=0
		else:
			if flag:
				if circle:
					rot=-paramRollover
				elif dubin&(millis()-t0<paramDubinTime):
					rot=paramRollover
				else:
					rot=0
			else:
				rot=0
		if headingControl:
			rot=headingFeedback(currentState,rotStorage)
		if millis()-t0>paramRiseTime:
			currentState=NEUTRAL
			nextState=DOWNGLIDE
			glideCompleted=1
			rotStorage=rot
			t0=millis()
	rotStorage=rot
	if linMode==PWM:
		if linPID:
			lin=linPIDrate(lin)
	if turnFeedback:
		rot=rotPID(rot)
	if glideCompleted & (completedGlides+1<paramNumberofGlides):
		lastUpAngle=0
		lastDownAngle=0
		upLoops=0
		downLoops=0
	return (glideCompleted,lin,rot,pump,linMode)



def millis():
	return timer.timeit()

def updateIMU():
	ypr = imu.read_yaw_pitch_roll()
	gps = imu.read_gps_solution_lla()
	gpsEst=imu.read_ins_solution_lla()

def updateGlider():
	gliderLinPos=ADC.read(linPosPin)
	gliderTankPos=ADC.read(tankLevelPin)
	gliderPressure=ADC.read(pressureSensorPin)



def checkPump(tank):
	if(abs(gliderTankPos-tank)<10):
		return 1
	else:
		return 0

def checkMass(mass):
	if(abs(gliderLinPos-mass)<10):
		return 1
	else:
		return 0

def checkFF(tank,angle,flag):
	if abs(gliderTankPos-tank)<10 & abs(imuPitch-angle)<paramFFerror:
		flag=1
	if millis()-t0>paramFFtime:
		flag=1

	return flag


def actuate(lin,rot,tank,mode):
	#Low level motion controller

	if mode==POSITION:#Bang-Bang control
		if lin<paramLinFrontLimit:
			print("Too far forward, going to limit")
			lin=paramLinFrontLimit
		if lin>paramLinBackLimit:
			print("Too far back, going to limit")
			lin=paramLinBackLimit
		if abs(gliderLinPos-lin)<10:
			updateMotors(0)
		else:
			if gliderLinPos>lin:
				updateMotors(speed)
			else:
				updateMotors(-speed)

	if mode==PWM:
		lin=constrain(l,-100,100)
		if lin>0 & gliderLinPos<=paramLinFrontLimit:
			lin=0
		if lin<0 & gliderLinPos>=paramLinBackLimit:
			lin=0
		if abs(lin)<10:
			lin=0
		updateMotors(lin)
	updateServo(rot)
	updateTank(tank)


def turnOff():
	updateMotors(0)
	updateTank(0)



def constrain(number,lower,upper):
	return max(min(upper,number),lower)

def updateServo(angle):
	angle=constrain(angle,0,180)
	duty_min=3#need to be experimentally set
	duty_max=14.5#need to be experimentally set
	duty_span=duty_max-duty_min
	duty=100-((angle/180)*duty_span+duty_min)
	PWM.set_duty_cycle(rotServoPin,duty)

def updateTank(tank):
	if tank<paramTankFrontLimit:
		tank=paramTankFrontLimit
	if tank>paramTankBackLimit:
		tank=paramTankBackLimit
	if abs(gliderTankPos-tank)<10:
		tank=0
	if gliderTankPos>tank:
		GPIO.output(pumpDir,LOW)
	else:
		GPIO.output(pumpDir,HIGH)
	if tank==0:
		GPIO.output(pumpOn,LOW)
	else:
		GPIO.output(pumpOn,HIGH)

def updateMotors(speed):
	if speed>0:
		GPIO.output(motAConf1,HIGH)
		GPIO.output(motAConf2,LOW)
		GPIO.output(motBConf1,HIGH)
		GPIO.output(motBConf2,LOW)
		GPIO.output(motStdby,HIGH)
	elif speed==0:
		GPIO.output(motStdby,LOW)
	else:
		GPIO.output(motAConf1,LOW)
		GPIO.output(motAConf2,HIGH)
		GPIO.output(motBConf1,LOW)
		GPIO.output(motBConf2,HIGH)
		GPIO.output(motStdby,HIGH)
	PWM.set_duty_cycle(motAPWM,abs(speed))
	PWM.set_duty_cycle(motBPWM,abs(speed))

def rotPID(rot):
	error=-(rot-imuRoll)
	rollI=rollI+error/100
	rotOutput=paramRollKp*error+paramRollKi*rollI+paramRollKd*imuRollD+rotOutput
	rotOutput=constrain(rotOutput,-90,90)
	return rotOutput

def linPIDRate(lin):
	P=-(lin-imuPitch)
	linI=linI+P/100
	return paramLinKp*P+paramLinKi*linI+paramLinKd*imuPitchD

def logData():
	file=open("testfile.txt","w")
	file.write(gliderLinPos)
	file.write(",")
	file.write(gliderTankPos)
	file.write(",")
	file.write(gliderPressure)
	file.write(",")
	file.write(ypr)
	file.write(",")