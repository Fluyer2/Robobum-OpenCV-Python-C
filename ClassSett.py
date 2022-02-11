from Tkinter import *
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from PIL import Image, ImageTk
import threading
import Queue
import RPi.GPIO as GPIO
import serial
import time

ser = serial.Serial(
	port= '/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
	baudrate = 9600
)

i = 0	

# coding= UTF-8

class Settings:
	def __init__(self, master):
		master.title("FRANKENSTEIN")
		master.attributes('-fullscreen', True)
		
		imgtk = PhotoImage(file = "Intro.gif")
		imgtkt = PhotoImage(file = "Intro.gif")
		
		
		self.PozicijaX = 0
		self.PozicijaY = 0

		self.SettingsFrame = Frame(master, height = 480, width = 380)
		self.SettingsFrame.grid(column = 1, row = 1)

		self.RadioFrame = Frame(self.SettingsFrame, width = 365)
		self.RadioFrame.grid(column = 1, row = 2)

		self.SettButtonsFrame = Frame(self.SettingsFrame, width = 365)
		self.SettButtonsFrame.grid(column = 1, row = 1)

		self.MoveButtonsFrame = Frame(self.SettingsFrame, width = 365)
		self.MoveButtonsFrame.grid(column = 1, row = 3)
		
		self.InfoFrame = Frame(self.SettingsFrame, width = 365)
		self.InfoFrame.grid(column = 1, row = 4, sticky = W)
		
		######################################################################
		self.PictureFrame = Frame(master, height = 480, width = 640)#### CelaSlika
		self.PictureFrame.grid_propagate(False)
		self.PictureFrame.grid(column = 2, row = 1, rowspan = 1)
		
		self.PictureFrameTh = Frame(master, height = 480, width = 640)####Narezane slike
		self.PictureFrameTh.grid_propagate(False)
		self.PictureFrameTh.grid(column = 2, row = 2, rowspan = 1)
		
		self.LabelPicL = Label(self.PictureFrame, image = imgtk) 
		self.LabelPicL.image = imgtk
		self.LabelPicL.grid(row = 1 , column = 1)
		
		self.LabelPicThreshL = Label(self.PictureFrameTh, image = imgtk) 
		self.LabelPicThreshL.image = imgtk
		self.LabelPicThreshL.grid(row = 1, column = 1)
		#################################################################
		master.protocol("WM_DELETE_WINDOW", self.on_closing)
		
		self.Buttons()
		self.LabelCameraSettingsSet()
		self.InfoPanelLabels()
	
	def on_closing(self):
		global StopInt
		GPIO.cleanup()
		StopInt = True
		root.destroy()
	
	#### Ustvari in nastavi Label za vrednosti kamere #### 
	def LabelCameraSettingsSet(self):
		self.ValueList = []
		
		##### Zapise si vrednosti iz file ####  
		with open("/home/pi/TriPike/KameraSettings.txt") as CameraSettings: # 45.100.4118.200.1.10.4
			CameraSettingsValues = CameraSettings.read().split('.')
			CameraSettings.close()
			
		with open("/home/pi/TriPike/Vzmet1.txt") as Vzmet1File: # 0.640.0.480.10000.0
			Vzmet1Settings = Vzmet1File.read().split('.')
			Vzmet1File.close()
			
		with open("/home/pi/TriPike/MaxSettingsValue.txt") as MaxSettings: # 100.100.10000.500.10.10.10.640.640.480.480.10000.10000
			self.MaxSettingsValue = MaxSettings.read().split('.')
			MaxSettings.close()
			
		with open("/home/pi/TriPike/PozicijaXY.txt") as Pozicija:
			self.PozicijaXY = Pozicija.read().split('.')
			Pozicija.close()
			
		#### Zapise vrednosti v Stringvar da se lahko po tem urejajo, izven funkcjie v real time####
		for i in range(len(CameraSettingsValues)):	
			ValueCamera = StringVar()
			ValueCamera.set(CameraSettingsValues[i])
			self.ValueList.append(ValueCamera)
			self.CameraIntValue = CameraSettingsValues
		
		for p in range(len(Vzmet1Settings)):
			ValueVzmet = StringVar()
			ValueVzmet.set(Vzmet1Settings[p])
			self.ValueList.append(ValueVzmet)
			self.Vzmet1SettingsValue =  Vzmet1Settings
		
		##### Ustvari Labele z vrednostmi #####
		for x in range(len(CameraSettingsValues) + len(Vzmet1Settings)):
			self.RbtnLabel = Label(self.RadioFrame, textvariable = self.ValueList[x], font = (None, 12))
			
			if x >= len(CameraSettingsValues):
				ColumnNmb = 4
				x = x - len(CameraSettingsValues)
			else:
				ColumnNmb = 2
			self.RbtnLabel.grid(column = ColumnNmb, row = x + 1)
			
			
	def WriteValues(self):
		
		CameraWriteValue = '.'.join(map(str, self.CameraIntValue))
		VzmetWriteValue = '.'.join(map(str, self.Vzmet1SettingsValue))
		PozicijaXYWriteValue = '.'.join(map(str, self.PozicijaXY))
		
		print(CameraWriteValue)
		print(VzmetWriteValue)
		print(PozicijaXYWriteValue)
		
		with open("/home/pi/TriPike/KameraSettings.txt", 'w') as CameraSettings:
			CameraSettings.write(CameraWriteValue)
			CameraSettings.close()
			
		with open("/home/pi/TriPike/Vzmet1.txt", 'w') as Vzmet1File:
			Vzmet1File.write(VzmetWriteValue)
			Vzmet1File.close()
			
		with open("/home/pi/TriPike/PozicijaXY.txt", 'w') as Pozicija:
			Pozicija.write(PozicijaXYWriteValue)
			Pozicija.close()
	
	def OkBtn(self):
		self.WriteValues()
	
	def ButtonClick(self, Increment):
		if self.RbtnVar.get() == 0:
			return
		else:
			RbtnVarLocal = self.RbtnVar.get() - 1 #### minus ena ker se List zacne z  nicno pozicijo
		
		
		if RbtnVarLocal < 0:
			print("ERROR: RADIO BUTTON VALUE IS LESS THEN ZERO")
			return

		##### Preveri iz kje je izbrana vrednost
		if RbtnVarLocal > 6:
			
			self.Vzmet1SettingsValue[RbtnVarLocal - 7] = int(self.Vzmet1SettingsValue[RbtnVarLocal - 7]) + Increment
			
			if  int(self.Vzmet1SettingsValue[RbtnVarLocal - 7]) <= 0:
				self.Vzmet1SettingsValue[RbtnVarLocal - 7] = 0
			elif int(self.MaxSettingsValue[RbtnVarLocal]) <= int(self.Vzmet1SettingsValue[RbtnVarLocal - 7]):
				self.Vzmet1SettingsValue[RbtnVarLocal - 7] = int(self.MaxSettingsValue[RbtnVarLocal])
			
			self.ValueList[RbtnVarLocal].set(self.Vzmet1SettingsValue[RbtnVarLocal - 7])
		
		else:
			
			self.CameraIntValue[RbtnVarLocal] = int(self.CameraIntValue[RbtnVarLocal]) + Increment ##### CameraIntValue = vrednosti v int obliki shranjene v List -> Ta bo uporabljena pri zapisovanju v file in branju vrednosti za nastavitve v opencv
			
			##### Preverja ce je vrednost nastavitve slucajno presegla mejno vrednost ####
			if  int(self.CameraIntValue[RbtnVarLocal]) <= 0:
				self.CameraIntValue[RbtnVarLocal] = 0
			elif int(self.MaxSettingsValue[RbtnVarLocal]) <= int(self.CameraIntValue[RbtnVarLocal]):
				self.CameraIntValue[RbtnVarLocal] = int(self.MaxSettingsValue[RbtnVarLocal])
			
			self.ValueList[RbtnVarLocal].set(self.CameraIntValue[RbtnVarLocal])##### V stringvar zapise ze pristeto/odsteto CameraIntValue
			 
			
	def RbtnCheck(self):
		pass
		
	def MoveBtnClick(self, Direction):
		
		if Direction == 'LEFT':
			self.PozicijaXY[0] = int(self.PozicijaXY[0]) - 1
			print(Direction)
			
		if Direction == 'RIGHT':
			self.PozicijaXY[0] = int(self.PozicijaXY[0]) + 1
			print(Direction)
			
		if Direction == 'UP':
			self.PozicijaXY[1] = int(self.PozicijaXY[1]) - 1
			print(Direction)
			
		if Direction == 'DOWN':
			self.PozicijaXY[1] = int(self.PozicijaXY[1]) + 1
			print(Direction)
			
		if (int(self.Vzmet1SettingsValue[1]) + int(self.PozicijaXY[0])) > 640:
			self.PozicijaXY[0] = 640 - int(self.Vzmet1SettingsValue[1])
			
		if (int(self.Vzmet1SettingsValue[0]) + int(self.PozicijaXY[1])) > 480:
			self.PozicijaXY[1] = 480 - int(self.Vzmet1SettingsValue[0])
			
		##### MEJNE VREDNOSTI ##### [0] == X-os   [1] == Y-os
		if int(self.PozicijaXY[0]) <= 0:
			self.PozicijaXY[0] = 0	
		elif int(self.PozicijaXY[0]) >= 640:
			self.PozicijaXY[0] = 640
		
		if int(self.PozicijaXY[1]) <= 0:
			self.PozicijaXY[1] = 0		
		elif int(self.PozicijaXY[1]) >= 480:
			self.PozicijaXY[1] = 480
	
	def Buttons(self):
		self.RbtnVar = IntVar()
		##### Naredi gumbe ki bodo settali vrednosti #####
		BtPPP = Button(self.SettButtonsFrame, font = (None, 8), command = lambda: self.ButtonClick(50), repeatdelay=500 , repeatinterval=50, text = "+++", width = 3)                
		BtPPP.grid(row = 1, column = 1 )
		BtPP  = Button(self.SettButtonsFrame, font = (None, 8), command = lambda: self.ButtonClick(20), repeatdelay=500 , repeatinterval=50, text = "++", width = 3)                 
		BtPP.grid(row = 1, column = 2 )
		BtP   = Button(self.SettButtonsFrame, font = (None, 8), command = lambda: self.ButtonClick(1), repeatdelay=500 , repeatinterval=50, text = "+", width = 3)
		BtP.grid(row = 1, column = 3 )
		BtOK  = Button(self.SettButtonsFrame, font = (None, 8), command = self.OkBtn, repeatdelay=500 , repeatinterval=50, text = "OK", width = 3)                
		BtOK.grid(row = 1, column = 4 )
		BtM   = Button(self.SettButtonsFrame, font = (None, 8), command = lambda: self.ButtonClick(-1), repeatdelay=500 , repeatinterval=50, text = "-", width = 3)                  
		BtM.grid(row = 1, column = 5 )
		BtMM  = Button(self.SettButtonsFrame, font = (None, 8), command = lambda: self.ButtonClick(-20), repeatdelay=500 , repeatinterval=50, text = "--", width = 3)                 
		BtMM.grid(row = 1, column = 6 )
		BtMMM = Button(self.SettButtonsFrame, font = (None, 8), command = lambda: self.ButtonClick(-50), repeatdelay=500 , repeatinterval=50, text = "---", width = 3) 
		BtMMM.grid(row = 1, column = 7 )
		
		#####Naredi gumbe za premikanje#####
		BtUP 	= Button(self.MoveButtonsFrame, font = (None, 20), repeatdelay=500 , repeatinterval=50, command = lambda: self.MoveBtnClick('UP'), text = 'UP', height = 1, width = 5)
		BtUP.grid(column = 2, row = 1)
		BtDOWN 	= Button(self.MoveButtonsFrame, font = (None, 20), repeatdelay=500 , repeatinterval=50, command = lambda: self.MoveBtnClick('DOWN'), text = 'DOWN', height = 1, width = 5)
		BtDOWN.grid(column = 2, row = 2)
		BtLEFT 	= Button(self.MoveButtonsFrame, font = (None, 20), repeatdelay=500 , repeatinterval=50, command = lambda: self.MoveBtnClick('LEFT'), text = 'LEFT', height = 1, width = 5)
		BtLEFT.grid(column = 1, row = 1)
		BtRIGHT = Button(self.MoveButtonsFrame, font = (None, 20), repeatdelay=500 , repeatinterval=50, command = lambda: self.MoveBtnClick('RIGHT'), text = 'RIGHT', height = 1, width = 5)
		BtRIGHT.grid(column = 3, row = 1)
		
		#####GUMB ZA AUTO#####
		BtnAuto = Button(self.MoveButtonsFrame, font = (None, 20), command = self.AutoButton, text = "AUTO", width = 5)
		BtnAuto.grid(column = 3, row = 2)
		
		#####GUMB ZA IZKLOP#####
		BtnOFF = Button(self.MoveButtonsFrame, font = (None, 20), command = self.on_closing, text = "QUIT", width = 5)
		BtnOFF.grid(column = 1, row = 2)
		
		##### Naredi Radio gumbe ##### Brightness:.Contrast:.Shutter:.Thresh1:.Thresh2:.Gaussian:.Median:.Left:.Right:.Up:.Down:.MaxArea:.MinArea:
		with open("/home/pi/TriPike/SettNamev2.txt", "r") as ButtonNames:
			ButtonNamesValue = ButtonNames.read().split('.')
			ButtonNames.close()
			
		for x in range(len(ButtonNamesValue)):
			Rbtn = Radiobutton(self.RadioFrame, text = ButtonNamesValue[x], font = (None, 12),
			variable = self.RbtnVar, value = x + 1, command = self.RbtnCheck, width = 10, anchor = W)
			if x > 6:
				CulumnNm = 3
				x = x - 7
			else:
				CulumnNm = 1
			Rbtn.grid(column = CulumnNm, sticky = W, row = x+1)
	
	def InfoPanelLabels(self):
		
		self.AreaZgVal = StringVar()
		self.AreaSpVal = StringVar()
		self.FPSVal = StringVar()
		self.SpeedVal = StringVar()
		self.OdstopanjeRefSpVal = StringVar()
	
		self.LbAreaZg     = Label(self.InfoFrame, textvariable = self.AreaZgVal, font = (None, 12), anchor = W)
		self.LbAreaZg.grid(column = 0, row =  1, sticky = W)
		self.LbAreaSp     = Label(self.InfoFrame, textvariable = self.AreaSpVal, font = (None, 12), anchor = W)
		self.LbAreaSp.grid(column = 0, row =  2, sticky = W)
		self.LbFPS        = Label(self.InfoFrame, textvariable = self.FPSVal	 , font = (None, 12), anchor = W)
		self.LbFPS.grid(column = 0, row =  3, sticky = W)
		self.Speed = Label(self.InfoFrame, textvariable = self.SpeedVal, font = (None, 12), anchor = W)
		self.Speed.grid(column = 0, row =  4, sticky = W)
		self.LbOdstopanjeSp = Label(self.InfoFrame, textvariable = self.OdstopanjeRefSpVal, font = (None, 12), anchor = W)
		self.LbOdstopanjeSp.grid(column = 0, row =  5, sticky = W)
	
	def InfoPanel(self, AreaZg, AreaSp, FPS, Speed, OdstopanjeRefSp):
		self.AreaZgVal.set("DesnaPos: " + str(AreaZg))
		self.AreaSpVal.set("LevaPos: " + str(AreaSp))
		self.FPSVal.set("FPS: " + str(FPS))
		self.SpeedVal.set("Speed: " + str(Speed))
		self.OdstopanjeRefSpVal.set("OdstopanjeRefSp: " + str(OdstopanjeRefSp))
		
	def AutoButton(self):
		global AutoFT
		AutoFT = True

def TkinterUpdater():
	global imgtkL
	global imgtkU
	global imgtktL
	global imgtktU
	
	##### Global za infopanel####
	global AreaZg
	global AreaSp
	global FPS
	global Speed
	global OdstopanjeSp
	
	my_gui.LabelPicL.configure(image = imgtkL)
	my_gui.LabelPicL.image = imgtkL
	# my_gui.LabelPicU.configure(image = imgtkU)
	# my_gui.LabelPicU.image = imgtkU
	
	my_gui.LabelPicThreshL.configure(image = imgtktL)
	my_gui.LabelPicThreshL.image = imgtktL
	# my_gui.LabelPicThreshU.configure(image = imgtktU)
	# my_gui.LabelPicThreshU.image = imgtktU
	
	my_gui.InfoPanel(AreaZg,AreaSp,int(round(FPS)),Speed, OdstopanjeSp)
	
	root.after(10,TkinterUpdater)
		
Count = 0
FPS = 0
Speed = 0
OdstopanjeSp = 0
StopInt = False
AutoFT = False
imgtkL = []
imgtkU = []
imgtktL = []
imgtktU = []
Smer = 0
Zelena = False

AreaZg = 2000
AreaSp = 2000

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)## Izmik

def OpenCv():
	global StopInt
	global imgtkL
	global imgtkU
	global imgtktL
	global imgtktU
	
	global FPS
	global AreaZg
	global AreaSp
	global OdstopanjeZg
	global OdstopanjeSp
	global Speed
	global Zelena
	
	global AutoFT
	
	
	UpperOK = False
	LowerOK = False
	Izmik = False
	Check = False
	CrtaNaLevi = False
	CrtaNaesniLevi = False
	Check = False
	
	#GPIO.output(21, GPIO.OUT)
	
	imgtkU = PhotoImage(file = "Intro.gif")
	imgtktU = PhotoImage(file = "Intro.gif")
	
	k = 0
	start  = 0
	end = 0
	FPS = 0
	AreaZgWanted = 2000
	AreaSpWanted = 2000
	OdstopanjeRefZg = 0
	OdstopanjeRefSp = 0
	Smer = 0
	Crta = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
	Step = 40
	CrtaLevi = [0,0,0,0,0,0,0,0]
	CrtaDesni= [0,0,0,0,0,0,0,0]
	
	OdsSp = 50
	OdsZg = 50
	
	DesnaPosOld = 0
	LevaPosOld = 0
	
	
	
	while True:
		
		CameraValue = my_gui.CameraIntValue
		PozicijaXY1 = my_gui.PozicijaXY
		VzmetValue = my_gui.Vzmet1SettingsValue
		
		thresh1 = (int(CameraValue[3])*2) + 3
		thresh2 = (int(CameraValue[4])*2) + 3
		gaussian = (int(CameraValue[5])*2) + 3
		median = (int(CameraValue[6]) * 2) + 3
		
		HeightVzmet = int(VzmetValue[0])
		
		HUE_MIN = int(VzmetValue[1])
		SAT_MIN = int(VzmetValue[2])
		VAL_MIN = int(VzmetValue[3])
		
		HUE_MAX = int(VzmetValue[4])
		SAT_MAX = int(VzmetValue[5])
		VAL_MAX = int(VzmetValue[6])
		
		if HUE_MAX < HUE_MIN:
			HUE_MIN = HUE_MIN - ((HUE_MIN - HUE_MAX) + 1)
		
		if SAT_MAX < SAT_MIN:
			SAT_MIN = SAT_MIN - ((SAT_MIN - SAT_MAX) + 1)
			
		if VAL_MAX < VAL_MIN:
			VAL_MIN = VAL_MIN - ((VAL_MIN - VAL_MAX) + 1)
			
		LowerBound = np.array([HUE_MIN, SAT_MIN, VAL_MIN])
		UpperBound = np.array([HUE_MAX, SAT_MAX, VAL_MAX])
		
		#PozicijaX = int(PozicijaXY1[0]) #### LEVA MEJA
		PozicijaY = int(PozicijaXY1[1]) #### SPODNJA MEJA
		
		# if WidthVzmet < 20:
			# WidthVzmet = 20
			
		if HeightVzmet < 20:
			HeightVzmet = 20
		
		#PozicijaX_Right = PozicijaX + WidthVzmet #### DESNA MEJA
		PozicijaY_Top = PozicijaY + HeightVzmet #### Zgornja MEJA
		
		# if PozicijaX_Right > 640:
			# PozicijaX_Right = 640
			# PozicijaX = 640 - WidthVzmet #### SKRAJNI ODMIK OD LEVE je od desnega roba manj debelina slike	
			
		if PozicijaY_Top > 480:
			PozicijaY_Top = 480
			PozicijaY = 480 - HeightVzmet
		
		if CamImgQueue.full():
		
			##### FPS #####
			if k > 10:
				end = time.time()
				FPS = 10/(end - start)
				k = 0
				print("TestSlikice" + str(FPS))
			if k == 0:
				start = time.time()	
			k = k + 1
			
			YofPicture = (PozicijaY_Top - PozicijaY)/2
			
			##### VZEMI SLIKO IZ QUEUE  in jo obrezi#####
			imageFirstInternal = CamImgQueue.get(False)
			CelaSlika = imageFirstInternal[PozicijaY:PozicijaY_Top, 0:640]
			
			
			####Naredi HSV in poglej ce je kaj zelene, ce je uporabi to sliko drugace osnovno
			HSV_PIC = cv2.cvtColor(CelaSlika,cv2.COLOR_BGR2HSV)
			HSV_MASK = cv2.inRange(HSV_PIC,LowerBound,UpperBound)
			#HSV_MASK = cv2.medianBlur(HSV_MASK, median)
			Zeleni = cv2.countNonZero(HSV_MASK)
			
			if Zeleni > 15000:
				thresh = HSV_MASK
				Zelena = True
				pass
			else:
				grayL = cv2.cvtColor(CelaSlika, cv2.COLOR_BGR2GRAY)
				#gray_blurL = cv2.GaussianBlur(grayL, (gaussian, gaussian), 0)
				retL,thresh = cv2.threshold(grayL,thresh1,thresh2,cv2.THRESH_BINARY_INV)
			
			BaseReso = (PozicijaY_Top - PozicijaY) * Step		
			
			for i in range(16):
				image = thresh[PozicijaY:PozicijaY_Top, (Step * i):((Step * i) + Step)] ## Narezi sliko
				Beli = cv2.countNonZero(image)
				if BaseReso*0.25 < Beli:
					Crta[i] = 1
					cv2.circle(CelaSlika,(((Step * i) + 20),YofPicture), 5, (0,0,255), -1)
				else:
					Crta[i] = 0
			
			for i in range(16):
				if i <= 7:
					CrtaLevi[i] = Crta[i]
				if i >= 8:
					CrtaDesni[i - 8] = Crta[i]
			
			# print("Levi:" + str(CrtaLevi))	
			# print("Desni: " + str(CrtaDesni))
			
			try:	
				LevaPos = CrtaLevi.index(1)
				LevaPos = 7 - LevaPos
			except ValueError:
				LevaPos = 0
				
			try:
				CrtaDesni.reverse()
				DesnaPos = CrtaDesni.index(1)
				DesnaPos = 7 - DesnaPos
			except ValueError:
				DesnaPos = 0
					
			if GPIO.input(20) == 1:
				Izmik = True
				Check = True
			
			###########################################################3
			if Izmik == True:
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
						elif DesnaPos < LevaPos: #Se kar na Levi
							LevaPos = 0
							DesnaPos = 0
							
					elif CrtaNaDesni == True:
						if DesnaPos < LevaPos: #Crta je zdaj na Levi strani
							Izmik = False
							CrtaNaLevi = False
						elif LevaPos < DesnaPos: #Se kar na desni
							LevaPos = 0
							DesnaPos = 0
			############################################################3
				
			if DesnaPos < LevaPos:
				if Zelena == True:
					LevaPos = 76 # L
				SpeedSerial(LevaPos*(-1)) #(320/7)
				
			elif DesnaPos > LevaPos:
				if Zelena == True:
					DesnaPos = 68 # D
				SpeedSerial(DesnaPos)
				
			elif (DesnaPos == 0) and (LevaPos == 0): ##SEM NA BELI
				SpeedSerial(0)
				
			AreaZg = DesnaPos
			AreaSp = LevaPos
			Zelena = False
			
			##### IMAGE PROCCESING PRED OBJAVO #####
			CelaSlika = cv2.cvtColor(CelaSlika, cv2.COLOR_BGR2RGB)
			imL = Image.fromarray(CelaSlika)
			imgtkL = ImageTk.PhotoImage(image=imL)
			
			threshL = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
			threshL = cv2.cvtColor(threshL, cv2.COLOR_BGR2RGB)
			imtL = Image.fromarray(thresh)
			imgtktL = ImageTk.PhotoImage(image=imtL)
			
		if StopInt is True:		
			break

def VideoGrab():

	CameraValue = my_gui.CameraIntValue
	Shutter = int(CameraValue[2]) + 1500

	camera = PiCamera()
	camera.resolution = (640, 480)
	camera.framerate = 20
	camera.iso = 400
	camera.sharpness = 0
	camera.saturation = 0
	camera.meter_mode = 'spot'
	
	time.sleep(5)
	g = camera.awb_gains
	camera.awb_mode = 'off'
	camera.awb_gains = g
	
	camera.shutter_speed = Shutter
	camera.exposure_mode = 'off'
	
	
	rawCapture = PiRGBArray(camera, size=(640, 480))
	
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		
		imageFirst = frame.array
		
		Shutter = int(CameraValue[2]) + 1500
		Brightness = int(CameraValue[0])
		Kontrast = int(CameraValue[1])
		
		
		camera.shutter_speed = Shutter
		camera.contrast = Kontrast
		camera.brightness = Brightness
		
		if CamImgQueue.empty():
			CamImgQueue.put(imageFirst)
			#print ('EMPTY')
			
		rawCapture.truncate(0)
		
def SpeedSerial(cx):
	global Speed
	global Smer
	# cx = cx - 320
	Speed = int(round((cx*cx*(20.0/49.0)), 0)) # y=k*x*x
	#Speed = int(round(cx*(20.0/7.0)))
	# print(str(Speed))
	
	if cx < 0: #Pri uporabi kvadratne funkcije
		Speed *= (-1)
	
	if -2 <= Speed < 2:
		Smer = 3
		Speed = abs(Speed)
	
	if 2 < Speed <= 20: # NAPAKA NA DESNI STRANI
		Smer = 2 # Zavij Na levo
		
	if -20 <= Speed < -2: # NAPAKA NA LEVI STRANI
		Smer = 1 # Zavij Na desno
		Speed *= (-1)
		
	if Speed > 20:
		Speed = 20
	
	
	cx = abs(cx)
	if cx == 76: # ZAVIJ LEVO
		Speed = 0
		Smer = 5
	elif cx == 68: # ZAVIJ DESNO
		Speed = 0
		Smer = 4
		
	SendPack = Smer*100 + Speed 
	print(SendPack)	
	#ser.write("<" + str(SendPack) + ">")
	
root = Tk()
my_gui = Settings(root)

CamImgQueue = Queue.Queue(1)

t2 = threading.Thread(target=VideoGrab)
t2.setDaemon(True)
t2.start()

t = threading.Thread(target=OpenCv)
t.setDaemon(True)
t.start()

root.after(0,TkinterUpdater)
root.mainloop()