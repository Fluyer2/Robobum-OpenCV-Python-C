from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import threading
import Queue
import RPi.GPIO as GPIO
import serial
import time
import getch
import traceback
import sys


ser = serial.Serial(
	port= '/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
	baudrate = 9600
)

i = 0	

# coding= UTF-8


#####VERSION 5:
#####+Robot sledi normalni crti
#####+Robot Zaznava Zeleno Crto
#####+Robot Se izmika oviram
#####-
#####-
#####-Zaviraj na ovinku
#####-PID za sledenje????
#####-

class Settings:
	def __init__(self):

		self.PozicijaX = 0
		self.PozicijaY = 0
		
		self.LabelCameraSettingsSet()
	
	#### Ustvari in nastavi Label za vrednosti kamere #### 
	def LabelCameraSettingsSet(self):
		self.ValueList = []
		
		##### Zapise si vrednosti iz file ####	
		with open("/home/pi/TriPike/KameraSettings.txt") as CameraSettings: # 45.100.4118.200.1.10.4
			self.CameraSettingsValues = CameraSettings.read().split('.')
			CameraSettings.close()
			
		with open("/home/pi/TriPike/Vzmet1.txt") as Vzmet1File: # 0.640.0.480.10000.0
			self.Vzmet1Settings = Vzmet1File.read().split('.')
			Vzmet1File.close()
			
		with open("/home/pi/TriPike/MaxSettingsValue.txt") as MaxSettings: # 100.100.10000.500.10.10.10.640.640.480.480.10000.10000
			self.MaxSettingsValue = MaxSettings.read().split('.')
			MaxSettings.close()
			
		with open("/home/pi/TriPike/PozicijaXY.txt") as Pozicija:
			self.PozicijaXY = Pozicija.read().split('.')
			Pozicija.close()
		

lbegin = 0
n1 = 0

p = GreenAvg = GreenSum = 0
		
def Following():
	global Speed
	global Zelena
	global CelaSlika1
	global imageFirst
	
	CrtaNaLevi = False
	CrtaNaDesni = False
	
	Izmik = False
	Check = False

	Crta = [0] * 80	
	CrtaLevi = [0] * 40
	CrtaDesni= [0] * 40
	
	GreenLine = [0] * 80	
	GreenLeft = [0] * 40
	GreenRight = [0] * 40
	
	DesnaPosOld = 0
	LevaPosOld = 0
	
	begin = 0
	n = 0
	
	CelaSlika =0
	GreenPos =0
	
	CameraValue = my_gui.CameraSettingsValues
	PozicijaXY1 = my_gui.PozicijaXY
	VzmetValue = my_gui.Vzmet1Settings						 ######

	HUE_MIN = int(VzmetValue[1])							 ######
	SAT_MIN = int(VzmetValue[2])							 ######
	VAL_MIN = int(VzmetValue[3])							 ######

	HUE_MAX = int(VzmetValue[4])							 ######
	SAT_MAX = int(VzmetValue[5])							 ######
	VAL_MAX = int(VzmetValue[6])							 ######

	if HUE_MAX < HUE_MIN:									 ######
		HUE_MIN = HUE_MIN - ((HUE_MIN - HUE_MAX) + 1)		 ######

	if SAT_MAX < SAT_MIN:									 ######
		SAT_MIN = SAT_MIN - ((SAT_MIN - SAT_MAX) + 1)		 ######

	if VAL_MAX < VAL_MIN:									 ######
		VAL_MIN = VAL_MIN - ((VAL_MIN - VAL_MAX) + 1)		 ######

	LowerBound = np.array([HUE_MIN, SAT_MIN, VAL_MIN])		 ######
	UpperBound = np.array([HUE_MAX, SAT_MAX, VAL_MAX])	
	
	def SpeedSerial(cx, GrSerial):
		global Speed
		global Smer
		global lbegin
		global n1
		
		# cx = cx - 320	
		
		if lbegin == 0:
			lbegin = time.time()
		elif (time.time() - lbegin) > 1:
			#print('FPS_Serial: ' + str(n1) + '------->' +	str(time.time() - lbegin))
			lbegin = 0
			n1 = 0
		else:
				n1 = n1 + 1
		
		Speed = int(round((cx*cx*(20.0/1521.0)), 0)) # y=k*x*x
		#Speed = int(round(cx*(20.0/7.0)))
		#print('Speed:	' + str(Speed) + 'Smer:	 ' + str(Smer))
		
		if cx < 0: #Pri uporabi kvadratne funkcije
			Speed *= (-1)
		
		if -2 <= Speed < 2:
			Smer = 3
			Speed = abs(Speed)
		
		elif 2 < Speed <= 20: # NAPAKA NA DESNI STRANI
			Smer = 2 # Zavij Na levo
			
		elif -20 <= Speed < -2: # NAPAKA NA LEVI STRANI
			Smer = 1 # Zavij Na desno
			Speed *= (-1)
			
		elif Speed > 20:
			Speed = 20
		
		
		cx = abs(cx)
		if GrSerial == 'Levo': # ZAVIJ LEVO
			Smer = 5
		elif GrSerial == 'Desno': # ZAVIJ DESNO
			Smer = 4
		elif GrSerial =='ShTurn':
			Smer = 6
			Speed = 0;
		SendPack = Smer*100 + Speed 
		#print(SendPack)	
		ser.write("<" + str(SendPack) + ">")
	####
	def GreenCheck(GreenValue):
		global p
		global GreenAvg
		global GreenSum
		
		ser.write("<" + str(000) + ">")
		if p < 11:
			GreenSum = GreenSum + GreenValue
			GreenAvg = GreenSum/10
			p = p + 1
			GreenAvgOut = None
		else:
			p = 0
			Set = 0
			GreenSum = 0
			GreenAvgOut = GreenAvg
			GreenAvg = 0
		return GreenAvgOut
	
	#######################################################
	Step = 2
	BaseReso = 30 * Step
	BaseResoG = 30 * Step
	Zelena = False
	HSV_MASK = 0
	BL_TOC = BL_TOC_OLD = 0
	BL_ROC = BL_C_OLD = BL_C = 0
	while True:
		try:
			
			CelaSlika = FollowQueue.get(True, None)
			if GreenQueue.full():
				HSV_MASK = GreenQueue.get(False)
			else:
				HSV_MASK = 0
			# if begin == 0:
				# begin = time.time()
			# elif (time.time() - begin) > 1:
				# print('FPS_FOLLOW: ' + str(n) + '------->' +	str(time.time() - begin))
				# begin = 0
				# n = 0
			# else:
				# n = n + 1
			
			if CelaSlika is not 0:
				
				######################## Najdi crto
				
				
				if HSV_MASK is not 0:
					
					Zelena = True
					
					for i in range(80):	 #POZICIJA ZELENE GLEDE NA SREDINO VIDNEGA POLJA
						imageG = HSV_MASK[0:30, (Step * i):((Step * i) + Step)] ## Narezi sliko
						BeliG = cv2.countNonZero(imageG)
						if BaseResoG*0.25 < BeliG: ### Naredi list z pozicijami kjer je polno
							GreenLine[i] = 1
						else:
							GreenLine[i] = 0
						
						for i in range(80): ###Razpolovi listo
							if i <= 39:
								GreenLeft[i] = GreenLine[i]
							elif i >= 40:
								GreenRight[i - 40] = GreenLine[i]		
					
						try:	
							GreenLeftPos = GreenLeft.index(1)
							GreenLeftPos = 39 - GreenLeftPos #Pozicija od Srednjice
							#print('Levi:	' + str(CrtaLevi) + str(LevaPos))
						except ValueError:
							GreenLeftPos = 0
							#print('GreenLeftPosERROR')
							
						try:
							GreenRight.reverse()
							GreenRightPos = GreenRight.index(1)
							GreenRightPos = 39 - GreenRightPos #Pozicija od Srednjice
							#print('Desni:	' + str(CrtaDesni) + str(DesnaPos))
						except ValueError:
							GreenRightPos = 0
							#print('GreenRightPosERROR')
				
				thresh = CelaSlika
				
				for i in range(80):
					image = thresh[0:40, (Step * i):((Step * i) + Step)] ## Narezi sliko
					Beli = cv2.countNonZero(image)
					if BaseReso*0.25 < Beli: ### Naredi list z pozicijami kjer je polno
						Crta[i] = 1
					else:
						Crta[i] = 0
				
				for i in range(80): ###Razpolovi listo
					if i <= 39:
						CrtaLevi[i] = Crta[i]
					elif i >= 40:
						CrtaDesni[i - 40] = Crta[i]		
				
				try:	
					LevaPos = CrtaLevi.index(1)
					LevaPos = 39 - LevaPos #Pozicija od Srednjice
					#print('Levi:	' + str(CrtaLevi) + str(LevaPos))
				except ValueError:
					LevaPos = 0
					
				try:
					CrtaDesni.reverse()
					DesnaPos = CrtaDesni.index(1)
					DesnaPos = 39 - DesnaPos #Pozicija od Srednjice
					#print('Desni:	' + str(CrtaDesni) + str(DesnaPos))
				except ValueError:
					DesnaPos = 0
						
				if GPIO.input(18) == 1:
					Izmik = True
					Check = True
				
				###########################################################3Izmikanje oviri
				if Izmik == True: #### Izmikanje oviri
					SpeedSerial(0)
					if Check == True:
						if DesnaPos < LevaPos: #Crta je na Levi strani
							CrtaNaLevi = True
							Check = False
						elif LevaPos < DesnaPos: #Crta je na desni strani
							CrtaNaDesni = True
							Check = False
					elif Check == False:
						if CrtaNaLevi == True:
							if LevaPos < DesnaPos: #Crta je zdaj na desni strani
								Izmik = False
								CrtaNaLevi = False
								
						elif CrtaNaDesni == True:
							if DesnaPos < LevaPos: #Crta je zdaj na Levi strani
								Izmik = False
								CrtaNaLevi = False
				############################################################3 Voznja				
				elif DesnaPos < LevaPos: # Crta je na levi strani
					if Zelena == True:
						GreenPos = GreenCheck((LevaPos - GreenLeftPos))
						print ('GreenPos: ' + str(GreenPos))
						if GreenPos == None:
							Zelena = False
							#SpeedSerial(0)
						elif GreenPos > 0:
							GreenOnRight = True
							SpeedSerial(GreenLeftPos, 'Desno') # ZAVIJ DESNO
							Zelena = False
						elif GreenPos < 0:
							GreenOnLeft = True
							SpeedSerial(GreenLeftPos, 'Levo') # ZAVIJ LEVO
							Zelena = False
					else:
						SpeedSerial(LevaPos*(-1), 0)
					
				elif DesnaPos > LevaPos:
					if Zelena == True:
						GreenPos = GreenCheck((GreenRightPos - DesnaPos))
						print ('GreenPos: ' + str(GreenPos))
						if GreenPos == None:
							Zelena = False
							#SpeedSerial(0)
							pass
						elif GreenPos > 0:
							GreenOnRight = True
							SpeedSerial(GreenRightPos,'Desno') # ZAVIJ DESNO
							Zelena = False
						elif GreenPos < 0:
							GreenOnLeft = True
							SpeedSerial(GreenRightPos,'Levo') # ZAVIJ LEVO
							Zelena = False
					else:
						SpeedSerial(DesnaPos, 0)
					
				elif (DesnaPos == 0) and (LevaPos == 0): ##SEM NA BELI
					SpeedSerial(0, 0)
				
				if ((BL_ROC < -1000) or (1000 < BL_ROC)) and ((DesnaPos == 39) or (LevaPos == 39)):
					SpeedSerial(0, 'ShTurn')
					time.sleep(2)
					BL_TC = BL_TOC = BL_TOC_OLD = BL_C_OLD = BL_ROC = BL_C = 0
					
				BL_TOC = time.time()
				BL_TC = BL_TOC - BL_TOC_OLD
				BL_C = LevaPos - DesnaPos 
				BL_ROC = round(((BL_C_OLD - BL_C)/BL_TC), 1) #BL = BlackLine ROC = Rate of change TOC = TimeOfChange C = Change
				BL_C_OLD = BL_C
				BL_TOC_OLD = BL_TOC
				print('RATE OF CHANGE: ' + str(BL_ROC) + '___________TIME OF CHANGE:  ' + str(BL_TC))
				
				GreenPos = DesnaPos = LevaPos = GreenRightPos = GreenLeftPos = 0
				CelaSlika1 = CelaSlika
				
				#if -2000 > BL_ROC or BL_ROC > 2000:
				#	time.sleep(2)
				
		except:
			tb = traceback.format_exc()
			ErrorQueue.put(tb, False)
			break
				
			
def OpenCv():
	k = 0
	Process = False
	begin = 0
	n = 0
	CelaSlika = 0
	Zelena = False
	#####################################################################
	CameraValue = my_gui.CameraSettingsValues
	PozicijaXY1 = my_gui.PozicijaXY
	VzmetValue = my_gui.Vzmet1Settings						 ######

	HUE_MIN = int(VzmetValue[1])							 ######
	SAT_MIN = int(VzmetValue[2])							 ######
	VAL_MIN = int(VzmetValue[3])							 ######

	HUE_MAX = int(VzmetValue[4])							 ######
	SAT_MAX = int(VzmetValue[5])							 ######
	VAL_MAX = int(VzmetValue[6])							 ######

	thresh1 = (int(CameraValue[3])*2) + 3					 ######
	thresh2 = (int(CameraValue[4])*2) + 3					 ######
	gaussian = (int(CameraValue[5])*2) + 3					 ######
	median = (int(CameraValue[6]) * 2) + 3					 ######

	if HUE_MAX < HUE_MIN:									 ######
		HUE_MIN = HUE_MIN - ((HUE_MIN - HUE_MAX) + 1)		 ######

	if SAT_MAX < SAT_MIN:									 ######
		SAT_MIN = SAT_MIN - ((SAT_MIN - SAT_MAX) + 1)		 ######

	if VAL_MAX < VAL_MIN:									 ######
		VAL_MIN = VAL_MIN - ((VAL_MIN - VAL_MAX) + 1)		 ######

	LowerBound = np.array([HUE_MIN, SAT_MIN, VAL_MIN])		 ######
	UpperBound = np.array([HUE_MAX, SAT_MAX, VAL_MAX])		   ######
	##############################################################3
	
	while True:
		try:
			#t = time.time()
			CelaSlika = CamImgQueue.get(True, None)#0.05
			#print(str(time.time()-t))
			if begin == 0:
				begin = time.time()
			elif (time.time() - begin) > 1:
				print('FPS_OPENCV: ' + str(n) + '------->' +  str(time.time() - begin))
				begin = 0
				n = 0
			else:
				n = n + 1
				
			if CelaSlika is not 0:
	
				####Naredi HSV in poglej ce je kaj zelene, nato sporoci naprej					
				
				GreenCheck = CelaSlika#[0:10, 0:160]
				
				
				HSV_PIC = cv2.cvtColor(GreenCheck,cv2.COLOR_BGR2HSV)#0.00012
				
				HSV_MASK = cv2.inRange(HSV_PIC,LowerBound,UpperBound) #0.00001
				
				Zeleni = cv2.countNonZero(HSV_MASK) # 0.00001
				#print(str(Zeleni))
				if Zeleni > 100:
					#print("Green")
					Zelena = True
				else:
					Zelena = False
				
				grayL = cv2.cvtColor(CelaSlika, cv2.COLOR_BGR2GRAY)#0.00002
				
				#gray_blurL = cv2.GaussianBlur(grayL, (gaussian, gaussian), 0)# 0.001
				
				retL,thresh = cv2.threshold(grayL,thresh1,thresh2,cv2.THRESH_BINARY_INV) #0.00003
				
				if GreenQueue.empty() and Zelena == True:
					GreenQueue.put(HSV_MASK, False)
				
				if FollowQueue.empty():
					FollowQueue.put(thresh, False)
				
				
				CelaSlika = 0
	
		except:
			tb = traceback.format_exc()
			ErrorQueue.put(tb, False)
			break
	
	
def VideoGrab():
	
	
	CameraValue = my_gui.CameraSettingsValues

	camera = PiCamera()
	
	camera.framerate = 40
	camera.sensor_mode=4
	camera.resolution = (160, 100)
	camera.iso = 400
	camera.sharpness = 0
	camera.saturation = 0
	camera.meter_mode = 'spot'
	camera.video_denoise = False
	camera.video_stabilization = False
	
	time.sleep(5)
	for i in range(5000):
		g = camera.awb_gains
	
	camera.awb_mode = 'off'
	camera.awb_gains = g

	Shutter = 4000
	
	Brightness = int(CameraValue[0])
	Kontrast = int(CameraValue[1])
	
	print(str(Shutter))
	camera.shutter_speed = Shutter
	camera.contrast = Kontrast
	camera.brightness = Brightness
	camera.exposure_mode = 'off'
	
	begin = time.time()
	n = 0
	
	
	rawCapture = PiRGBArray(camera, size=(160, 100))
	k = 0
	
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		try:

			if begin == 0:
				begin = time.time()
			elif (time.time() - begin) > 1:
				print('FPS_KAMERA: ' + str(n) + '------->' +  str(time.time() - begin))
				begin = 0
				n = 0
			else:
				n = n + 1
			
			
			imageFirst = frame.array
			imageFirst = imageFirst[60:90, 0:160]
			if CamImgQueue.empty():
				CamImgQueue.put(imageFirst, False)
			
			if CamImgQueuePreview.empty():
				CamImgQueuePreview.put(imageFirst, False)
			
			rawCapture.truncate(0)
		except:
			tb = traceback.format_exc()
			ErrorQueue.put(tb, False)
			break
	
def ErrorGrabber():
	if ErrorQueue.empty() == False:
		Error = ErrorQueue.get(False)
		print(Error)
		GPIO.cleanup()
		return True
	else:
		return False
	
##########################	
my_gui = Settings()
#######################3
CamImgQueue = Queue.Queue(1)
CamImgQueuePreview = Queue.Queue(1)
ErrorQueue = Queue.Queue(0)
FollowQueue = Queue.Queue(1)
GreenQueue = Queue.Queue(1)
#########################
Speed = 0
StopInt = False
Smer = 0
Zelena = False
image = 0

CelaSlika1 = imageFirst = imageFollow = np.zeros((40,160,3), np.uint8)

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)## Izmik

#############################
t2 = threading.Thread(target=VideoGrab)
t2.setDaemon(True)
t2.start()

t3 = threading.Thread(target=OpenCv)
t3.setDaemon(True)
t3.start()

t = threading.Thread(target=Following)
t.setDaemon(True)
t.start()

while True:
	if CamImgQueuePreview.full():
		imageFirst = CamImgQueuePreview.get(False)
	#image = cv2.cvtColor(thresh3,cv2.COLOR_HSV2BGR)
	cv2.imshow('imageFirst', imageFirst)
	
	ch = cv2.waitKey(1)
	if ch == 27:
		ser.write("<" + str(000) + ">")
		break
		
	if ErrorGrabber():
		ser.write("<" + str(000) + ">")
		break
		