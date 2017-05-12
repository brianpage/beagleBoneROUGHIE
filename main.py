import Adafruit_BBIO.GPIO# as GPIO
import Adafruit_BBIO.PWM# as PWM
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.UART as UART
import serial
from vnpy import *
#from Adafruit_I2C import Adafruit_I2C
import time
import signal
import os



#Set pin numbers
motAPWM = "P9_21"
motBPWM = "P9_22"
motAConf1 = "P9_14"
motAConf2 = "P9_15"
motStdby = "P9_13"
motBConf1 = "P9_16"
motBConf2 = "P9_17"

pumpOn = "P9_16"
pumpDir = "P9_17"

tankLevelPin = "P9_37"
linPosPin = "P9_37"

pressureSensorPin = "P9_37"

rotServoPin = "P8_19"

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

currentState=DOWNGLIDE

#i2c = Adafruit_I2C(0x1E)

Adafruit_BBIO.PWM.start(str(motAPWM), 50)
Adafruit_BBIO.PWM.start(str(motBPWM),50)
Adafruit_BBIO.PWM.start(str(rotServoPin),50)
Adafruit_BBIO.PWM.set_duty_cycle(str(motAPWM),0)
Adafruit_BBIO.PWM.set_duty_cycle(str(motBPWM),0)


Adafruit_BBIO.GPIO.setup("P9_13",Adafruit_BBIO.GPIO.OUT)#motconfa1
Adafruit_BBIO.GPIO.setup("P9_14",Adafruit_BBIO.GPIO.OUT)#motconfa2
Adafruit_BBIO.GPIO.setup("P9_15",Adafruit_BBIO.GPIO.OUT)#motconfb1
Adafruit_BBIO.GPIO.setup("P9_16",Adafruit_BBIO.GPIO.OUT)#motconfb2
Adafruit_BBIO.GPIO.setup("P9_17",Adafruit_BBIO.GPIO.OUT)#motstby
Adafruit_BBIO.GPIO.setup(str(pumpOn),Adafruit_BBIO.GPIO.OUT)#pumpon

ADC.setup()

UART.setup("UART1")


imu = VnSensor()
imu.connect('/dev/ttyUSB0',115200)

paramTankBackLimit=1
paramTankFrontLimit=0
paramLinNoseDownTarget=-30
paramLinNoseUpTarget=30
paramFFerror=3
paramFFtime=6000
paramUpFeedforward=2
paramDownFeedforward=1
paramDesTime=3000
paramRiseTime=3000
paramNeutralTime=3000
paramLinFrontLimit=0
paramLinBackLimit=1
paramLinMid=0.5
paramNumberofGlides=3
linPIDrate=3
pressure_m=3
pressure_b=3
paramTankBackLimit=0
paramTankFrontLimit=1
paramTankMid=0.5


flag=0
gliderTankPos=0
gliderLinPos=0
imuPitch=0
imuPitchD=0
imuPitchoffset=0
imuRoll=0
imuRollD=0
imuRolloffset=0
imuYaw=0
t0=0


#Config area. Config must be changed before start.
feedforward=1
linPID=0
dubin=0
circle=0
headingControl=0
delayRoll=0
headingFeedback=0
turnFeedback=0
SDgo=1
command = START#default=START
mode=POSITION


def sawtooth():
	global t0
	global currentState,flag
	global lin,rot,pump,mode
	pumpDone=0
	glideCompleted=0
	if currentState==DOWNGLIDE:
		pump=paramTankBackLimit
		checkFF(pump,paramLinNoseDownTarget,flag)

		if feedforward and flag==0:
			linMode=POSITION
			lin=paramDownFeedforward
		elif linPID==0:
			linMode=POSITION
			lin=paramDownFeedforward
		else:
			linMode=PWM
			lin=paramLinNoseDownTarget
		if delayRoll==0:
			if circle or (dubin and (millis()-t0<paramDubinTime)):
				rot=paramRollover
			else:
				rot=0
		else:
			if (flag and (circle or (dubin and (millis()-t0>paramDubinTime)))):
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
			# t0=millis()

	if currentState==NEUTRAL:
		pump=paramTankMid
		if feedforward and millis()-t0>pumpTime:
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
			# t0=millis()
		flag=0

	if currentState==UPGLIDE:
		if millis()-t0>paramRiseTime-1000:
			upLoops=upLoops+1
			lastUpAngle=lastUpAngle+imuPitch
		pump=paramTankFrontLimit
		checkFF(pump,paramLinNoseUpTarget,flag)
		if(feedforward and flag==0) or (linPID==0):
			linMode=POSITION
			lin=paramUpFeedforward
		else:
			linMode=PWM
			lin=paramLinNoseUpTarget

		if circle:
			rot=-paramRollover
		elif dubin and (millis()-t0<paramDubinTime):
			rot=paramRollover
		else:
			rot=0

		if delayRoll==0:
			if circle:
				rot=-paramRollover
			elif dubin and (millis()-t0<paramDubinTime):
				rot=paramRollover
			else:
				rot=0
		else:
			if flag:
				if circle:
					rot=-paramRollover
				elif dubin and (millis()-t0<paramDubinTime):
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
			# t0=millis()
	rotStorage=rot
	if linMode==PWM:
		if linPID:
			lin=linPIDrate(lin)
	if turnFeedback:
		rot=rotPID(rot)
	if glideCompleted and (completedGlides+1<paramNumberofGlides):
		lastUpAngle=0
		lastDownAngle=0
		upLoops=0
		downLoops=0
	return glideCompleted



def millis():
	return time.time()

def updateIMU():
	global imuYaw,imuPitch,imuRoll
	ypr = imu.read_yaw_pitch_roll()
	imuYaw=ypr.x
	imuPitch=ypr.y+imuPitchoffset
	imuRoll=ypr.z+imuRolloffset
	# gps = imu.read_gps_solution_lla()
	# gpsEst=imu.read_ins_solution_lla()

def updateGlider():
	global gliderLinPos,gliderTankPos,gliderPressure
	gliderLinPos=ADC.read(linPosPin)
	gliderTankPos=ADC.read(tankLevelPin)
	gliderPressure=ADC.read(pressureSensorPin)


def updateCompass():
	dummyVar=0

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
	if abs(gliderTankPos-tank)<10 and abs(imuPitch-angle)<paramFFerror:
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
		if lin>0 and gliderLinPos<=paramLinFrontLimit:
			lin=0
		if lin<0 and gliderLinPos>=paramLinBackLimit:
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
	Adafruit_BBIO.PWM.set_duty_cycle(str(rotServoPin),duty)

def updateTank(tank):
	if tank<paramTankFrontLimit:
		tank=paramTankFrontLimit
	if tank>paramTankBackLimit:
		tank=paramTankBackLimit
	if abs(gliderTankPos-tank)<10:
		tank=0
	if gliderTankPos>tank:
		Adafruit_BBIO.GPIO.output("P9_17",Adafruit_BBIO.GPIO.LOW)
	else:
		Adafruit_BBIO.GPIO.output("P9_17",Adafruit_BBIO.GPIO.HIGH)
	if tank==0:
		Adafruit_BBIO.GPIO.output(pumpOn,Adafruit_BBIO.GPIO.LOW)
	else:
		Adafruit_BBIO.GPIO.output(pumpOn,Adafruit_BBIO.GPIO.HIGH)

def updateMotors(speed):
	if speed>0:
		Adafruit_BBIO.GPIO.output("P9_14",Adafruit_BBIO.GPIO.HIGH)
		Adafruit_BBIO.GPIO.output("P9_15",Adafruit_BBIO.GPIO.LOW)
		Adafruit_BBIO.GPIO.output("P9_16",Adafruit_BBIO.GPIO.HIGH)
		Adafruit_BBIO.GPIO.output("P9_17",Adafruit_BBIO.GPIO.LOW)
		Adafruit_BBIO.GPIO.output("P9_13",Adafruit_BBIO.GPIO.HIGH)
	elif speed==0:
		Adafruit_BBIO.GPIO.output("P9_13",Adafruit_BBIO.GPIO.LOW)
	else:
		Adafruit_BBIO.GPIO.output("P9_14",Adafruit_BBIO.GPIO.LOW)
		Adafruit_BBIO.GPIO.output("P9_15",Adafruit_BBIO.GPIO.HIGH)
		Adafruit_BBIO.GPIO.output("P9_16",Adafruit_BBIO.GPIO.LOW)
		Adafruit_BBIO.GPIO.output("P9_17",Adafruit_BBIO.GPIO.HIGH)
		Adafruit_BBIO.GPIO.output("P9_13",Adafruit_BBIO.GPIO.HIGH)
	Adafruit_BBIO.PWM.set_duty_cycle("P9_21",abs(speed))
	Adafruit_BBIO.PWM.set_duty_cycle(motBPWM,abs(speed))

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
	global f
	f.write(str(millis()-t0))
	f.write(",")
	f.write(str(imuPitch))
	f.write(",")
	f.write(str(imuRoll))
	f.write(",")
	f.write(str(imuYaw))
	f.write("\n")

def createSDfile():
	global f
	i=0
	while os.path.exists("/var/lib/cloud9/data/data%s.csv"%i):
		i+=1
	f=open("/var/lib/cloud9/data/data%s.csv"%i,'w+')
	f.write("Time,Pitch,Roll,Yaw\n")
	
def closeFile():
	global f
	f.close()

	
# TIMEOUT = 1 # number of seconds your want for timeout

# def interrupted(signum, frame):
#     "called when read times out"
#     print 'interrupted!'
# signal.signal(signal.SIGALRM, interrupted)

# def input():
#     try:
#             foo = raw_input()
#             return foo
#     except:
#             # timeout
#             return

	
# def readSerial():
# 	global linPID,circle,dubin,headingControl,feedforward,turnFeedback,pressureControl,delayRoll,pressure_b
# 	global command,SDgo
# 	print(value)
# 	# signal.alarm(TIMEOUT)
# 	# value=input().split()
# 	# signal.alarm(0)
# 	print value
# 	if value[0]=="exit":
# 		print("shutting down the script")
# 		exit()
		
# 	if value[0]=="reset":
# 		print("Glider Resetting")
# 		command=RESET
	
# 	if value[0]=="start":
# 		pressure_b=pressure_m*gliderPressure
# 		print("Pressure Calibrated")
# 		print("Starting")
# 		command=START
	
# 	if value[0]=="toDefault":
# 		paramInit()
# 		print("Parameters set to default")
	
# 	if value[0]=="rollTest":
# 		pressure_b=pressure_m*gliderPressure
# 		print("Roll test starting in 30 seconds")
# 		command=ROLLSTART
		
# 	if value[0]=="stop":
# 		print("Stopping")
# 		command=STOP
		
# 	if value[0]=="sdstart":
# 		if SDgo==0:
# 			createSDfile()
# 			SDgo=1
# 			print("SD start")
# 		else:
# 			print("Can't start a second file until current one is closed")
			
# 	if value[0]=="sdstop":
# 		if SDgo:
# 			closeFile()
# 			SDgo=0
# 			print("File closed")
# 		else:
# 			print("No file to close")
			
# 	if value[0]=="turnto":
# 		dummy=0
	
# 	if value[0]=="circle":
# 		if circle:
# 			circle=0
# 		else:
# 			circle=1
# 			dubin=0
# 			headingControl=0
# 		printController()
	
# 	if value[0]=="headingControl":
# 		if headingControl:
# 			headingControl=0
# 		else:
# 			headingControl=1
# 			dubin=0
# 			circle=0
# 		printController()

# 	if value[0]=="updateHeading":
# 		desHeading=compassHeading
# 		print("Desired heading updated to: "+desHeading)
	
# 	if value[0]=="feedforward":
# 		if feedforward:
# 			feedforward=0
# 		else:
# 			feedforward=1
			
# 	if value[0]=="dubin":
# 		if dubin:
# 			dubin=0
# 		else:
# 			dubin=1
# 			circle=0
# 			headingControl=0
# 		printController()
		
# 	if value[0]=="linear":
# 		if linPID:
# 			linPID=0
# 		else:
# 			linPID=1
			
# 		printController()
		
# 	if value[0]=="turnFeedback":
# 		if turnFeedback:
# 			turnFeedback=0
# 		else:
# 			turnFeedback=1
	
# 	if value[0]=="float":
# 		print("floating")
# 		command=FLOAT
		
# 	if value[0]=="pressurecontrol":
# 		if pressureControl:
# 			pressureControl=0
# 		else:
# 			pressureControl=1
# 		printController()
	
# 	if value[0]=="delayRoll":
# 		if delayRoll:
# 			delayRoll=0
# 		else:
# 			delayRoll=1
# 		printController()
	
# 	if value[0]=="pressurecal":
# 		pressure_b=pressure_m*gliderPressure
		
# 	if value[0]=="gimme":
# 		if value[1]=="glideAngles":
# 			print("Final seond IMU averages")
# 			print("Last downglide angle: "+lastDownAngle/downLoops)
# 			print("Last upglide angle: "+lastUpAngle/upLoops)
		
# 		if value[1]=="power":
# 			print("print power stuff (TODO)")
		
# 		if value[1]=="status":
# 			print("Roll: "+imuRoll)
# 			print("Pitch: "+imuPitch)
# 			print("Yaw: "+imuYaw)
# 			print("Tank Level: "+gliderTankPos)
# 			print("Linear Position: "+gliderLinPos)
# 			print("Pressure: "+gliderPressure)
# 			print("Latitude: "+gpsLat)
# 			print("Longitude: "+gpsLong)
	
# 	if value[0]=="update":
# 		arg=float(value[2])
# 		if value[1]=="-rollover":
# 			paramRollover=arg
# 			print("Updated Rollover to: "+paramRollover)
# 		if value[1]=="-dubinTime":
# 			paramDubinTime=arg
# 			print("Updated Dubintime to: "+paramDubinTime)
# 		if value[1]=="-upFeedforward":
# 			paramUpFeedforward=arg
# 			print("Updated Upfeedforward to: "+paramUpFeedforward)
# 		if value[1]=="-downFeedforward":
# 			paramDownFeedforward=arg
# 			print("Updated downFeedforward to: "+paramDownFeedforward)
# 		if value[1]=="-glidebottom":
# 			paramGlideCycleBottom=arg
# 			print("Updated glidebottom to: "+paramGlideCycleBottom)
# 		if value[1]=="-glidetop":
# 			paramGlideCycleTop=arg
# 			print("Updated glidetop to: "+paramGlideCycleTop)
# 		if value[1]=="-linrate":
# 			paramLinRate=arg
# 			print("Updated linrate to: "+paramLinRate)
# 		if value[1]=="-destime":
# 			paramDesTime=arg
# 			print("Updated destime to: "+paramDesTime)
# 		if value[1]=="-FFerror":
# 			paramFFerror=arg
# 			print("Updated FFerror to: "+paramFFerror)
# 		if value[1]=="-FFtime":
# 			paramFFtime=arg
# 			print("Updated FFtime to: "+paramFFtime)
# 		if value[1]=="-neutralTime":
# 			paramNeutralTime=arg
# 			print("Updated neutraltime to: "+paramNeutralTime)
# 		if value[1]=="-risetime":
# 			paramRiseTime=arg
# 			print("Updated risetime to: "+paramRiseTime)
# 		if value[1]=="-linmid":
# 			paramLinMid=arg
# 			print("Updated linmid to: "+paramLinMid)
# 		if value[1]=="-rotmid":
# 			paramRotMid=arg
# 			print("Updated rotmid to: "+paramRotMid)
# 		if value[1]=="-tankmid":
# 			paramTankMid=arg
# 			print("Updated tankmid to: "+paramTankMid)
# 		if value[1]=="-linfrontlimit":
# 			paramLinFrontLimit=arg
# 			print("Updated linfrontlimit to: "+paramLinFrontLimit)
# 		if value[1]=="-linbacklimit":
# 			paramLinBackLimit=arg
# 			print("Updated linbacklimit to: "+paramLinBackLimit)
# 		if value[1]=="-tankbacklimit":
# 			paramTankBackLimit=arg
# 			print("Updated tankbacklimit to: "+paramTankBackLimit)
# 		if value[1]=="-tankfrontlimit":
# 			paramTankFrontLimit=arg
# 			print("Updated tankfrontlimit to: "+paramTankFrontLimit)
# 		if value[1]=="-rotlowlimit":
# 			paramRotLowLimit=arg
# 			print("Updated rotlowlimit to: "+paramRotLowLimit)
# 		if value[1]=="-rothighlimit":
# 			paramRotHighLimit=arg
# 			print("Updated rothighlimit to: "+paramRotHighLimit)
# 		if value[1]=="-linkp":
# 			paramLinKp=arg
# 			print("Updated linkp to: "+paramLinKp)
# 		if value[1]=="-linki":
# 			paramLinKi=arg
# 			print("Updated linki to: "+paramLinKi)
# 		if value[1]=="-linkd":
# 			paramLinKd=arg
# 			print("Updated linkd to: "+paramLinKd)
# 		if value[1]=="-rollkp":
# 			paramRollKp=arg
# 			print("Updated rollkp to: "+paramRollKp)
# 		if value[1]=="-rollki":
# 			paramRollKi=arg
# 			print("Updated rollki to: "+paramRollKi)
# 		if value[1]=="-rollkd":
# 			paramRollKd=arg
# 			print("Updated rollkd to: "+paramRollKd)
# 		if value[1]=="-headingkp":
# 			paramHeadingKp=arg
# 			print("Updated headingkp to: "+paramHeadingKp)
# 		if value[1]=="-desiredheading":
# 			desHeading=arg
# 			print("Updated desiredheading to: "+desHeading)
# 		if value[1]=="-linnoseuptarget":
# 			paramLinNoseUpTarget=arg
# 			print("Updated linnoseuptarget to: "+paramLinNoseUpTarget)
# 		if value[1]=="-linnosedowntarget":
# 			paramLinNoseDownTarget=arg
# 			print("Updated linnosedowntarget to: "+paramLinNoseDownTarget)
# 		if value[1]=="-numberofglides":
# 			paramNumberofGlides=arg
# 			print("Updated numbersofglides to: "+paramNumberofGlides)
# 	if value[0]=="imucal":
# 		imuRolloffset=0
# 		imuPitchoffset=0
# 		updateIMU()
# 		imuRolloffset=-imuRoll
# 		imuPitchoffset=-imuPitch
# 		print("IMU Calibrated")
# 		print("Roll offset: "+imuRolloffset)
# 		print("Pitch offset: "+imuPitchoffset)
# 	if value[0]=="imureset":
# 		imuRolloffset=0
# 		imuPitchoffset=0
# 		print("IMU forgotten")
# 	if value[0]=="params":
# 		print("Desired rotational mass position: "+paramDesiredRotaryPosition+" Acceptable range: "+paramRotLowLimit+" to "+paramRotHighLimit)
# 		print("Linear mass acceptable range: "+paramLinFrontLimit+" to "+paramLinBackLimit)
# 		print("Ballast acceptable range: "+paramTankFrontLimit+"(full) to "+paramTankFrontLimit+"(empty)")
# 		print("Descent time: "+paramDesTime)
# 		print("Rise time: "+paramRiseTime)
# 		print("Neutral time: "+paramNeutralTime)
# 		print("Dubin time: "+paramDubinTime)
# 		print("Feedforward timeout: "+paramFFtime)
# 		print("Feedforward error bound: "+paramFFerror)
# 		print("Middle settings")
# 		print("Tank: "+paramTankMid+"  Default: "+tankMid)
# 		print("Linear: "+paramLinMid+"  Default: "+linMid)
# 		print("Roll: "+paramRotMid+"  Default: "+rotMid)
# 		print("Upfeedforward target: "+paramUpFeedforward)
# 		print("Downfeedforward target: "+paramDownFeedforward)
# 		print("Down glide angle: "+paramLinNoseDownTarget)
# 		print("Up glide angle: "+paramLinNoseUpTarget)
# 		print("PID settings:")
# 		print("Linear kp: "+paramLinKp+" ki: "+paramLinKi+" kd: "+paramLinKd)
# 		print("Roll kp: "+paramRollKp+" ki: "+paramRollKi+" kd: "+paramRollKd)
# 		print("Glide cycle bottom: "+paramGlideCycleBottom)
# 		print("Glide cycle top: "+paramGlideCycleTop)
# 	if value[0]=="help":
# 		printHelp()
	
# 	return command
		
# def printController():
# 	print("controller")
			
		
		


running=1

# start = timeit.timeit()

completedGlides=0
lin = 0
rot=0
pump=0
updateIMU()
imuRolloffset=-imuRoll
imuPitchoffset=-imuPitch
# imu.tare()

createSDfile()
t0=millis()
while(running==1):
	# print("Loop START")
	# t3=millis()
	updateIMU()
	# t2=millis()
	# print("IMU updated%f",t2-t3)
	updateCompass()
	# t3=millis()
	# print("Compass Updated%f",t3-t2)
	updateGlider()
	# t2=millis()
	# print("glider updated%f",t2-t3)
	if(SDgo):
		logData()
	# t3=millis()
	# print("Data logged%f",t3-t2)
	# print("data logged%f",millis()-t0)
	#newCommand=readSerial()
	# if newCommand != GLIDE:
	# 	command=newCommand

	if command==START:
		if checkPump(paramTankMid) and checkMass(paramLinMid):
			completedGlides=0
			# t0=millis()
			tstart=t0
			rollI=0
			rotOutput=0
			lastUpAngle=0
			lastDownAngle=0
			downLoops=0
			upLoops=0
			currentState=DOWNGLIDE
			flag=False
			command=GLIDE
		else:
			actuate(paramLinMid,paramRotMid,paramTankMid,POSITION)

	if command==STOP:
		turnOff()

	if command==GLIDE:
		completedGlides=completedGlides+sawtooth()
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

		actuate(paramLinMid,rollAngle,paramTankMid,POSITION)
	# t2=millis()
	# print("the rest of the stuff took%f",t2-t3)

