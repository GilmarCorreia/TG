import threading
import time
from SerialArduino import SerialArduino

class TouchSensorArduino(SerialArduino,object):

	TS_START = 255
	TS_WRITE = 1
	TS_READ = 0
	TS_FORCE_BEGIN = 1
	TS_FORCE_END = 2
	TS_PIEZO = 3
	TS_POT = 4

	TS_FORCE_BEGIN_LENGTH = 4
	TS_FORCE_END_LENGTH = 3
	TS_PIEZO_LENGTH = 3
	TS_PIEZOWRITE_LENGTH = 4
	TS_POT_LENGTH = 3
	TS_POT_WRITE_LENGTH = 4

	TX_DELAY_TIME = 200 #Microseconds
	RX_DELAY_TIME = 10  #Millisseconds
	
	buf = [None] * 20
	force = None
	controlTime = True
	millisInitialTime = None
	millisFinalTime = None

	def __init__(self):
		super().__init__()

	## SETTERS

	def __setForce(self,force):
		self.force = force
	
	def __setMillisInitialTime(self,initialTime):
		self.millisInitialTime = initialTime
	
	def __setMillisFinalTime(self,finalTime):
		self.millisFinalTime = finalTime
	
	def __setControlTime(self,controlTime):
		self.controlTime = controlTime

	## GETTERS

	def getForce(self):
		return self.force
	
	def getMillisInitialTime(self):
		return self.millisInitialTime

	def getMillisFinalTime(self):
		return self.millisFinalTime
	
	def getControlTime(self):
		return self.controlTime

	## METHODS

	def convert(self, binary):
	  return binary*256

	def run(self):
		
		try:
			self.buf = [int(prot) for prot in self.getInputValue().split(",")]

			if(self.buf[0] == self.TS_START):
				if(self.buf[1] == self.TS_START):
					if(self.buf[2] == self.TS_WRITE):
						lengthMsg = self.buf[3]
						dataType = self.buf[4]
						checkSum = None

						if (dataType == self.TS_FORCE_BEGIN):
							force = 0
							force = self.buf[5]
							force = force + self.convert(self.buf[6])

							checkSum = (~(self.TS_WRITE+self.TS_FORCE_BEGIN_LENGTH+self.TS_FORCE_BEGIN+(force&0xFF)+(force>>8)))&0xFF

							if (checkSum == self.buf[7]):

								if(self.getControlTime()):
									self.__setMillisInitialTime(int(round(time.time() * 1000)))
									self.__setControlTime(False)

								self.__setForce(force)

							else:
								print("Falha na Mensagem")

						elif (dataType == self.TS_FORCE_END):
							dataChr = self.buf[5]

							checkSum = (~(self.TS_WRITE+self.TS_FORCE_END_LENGTH+self.TS_FORCE_END+dataChr))&0xFF

							if (checkSum == self.buf[6]):
								if(not(self.getControlTime())):
									self.__setMillisFinalTime(int(round(time.time() * 1000)))
									self.__setControlTime(True)

								self.__setForce(0)

							else:
								print("Falha na Mensagem")			

			else:
				print("Erro de Comunicacao")
		except:
			pass
		#	print("Erro de Comunicacao1")