import time
import abc
import sys
import glob
import serial
from serial import Serial
import select
import threading
#from Errors.NoAvailablePorts import NoAvailablePorts


class SerialArduino():

    __metaclass__ = abc.ABCMeta 

    __timeout = 0.010
    __baudrate = 9600

    __thread = None
    __threadFlag = False
    serialPort = None
    __portName = None
    __inputValue = None
    __time = None
    __initialTime = None

    def __init__(self, baudrate = 9600):
        self.__initialTime = int(round(time.time() * 1000))
        self.__showTexts()
        
        self.__setBaudrate(baudrate)

        try:
            self.__enableSerialComm()
        except (OSError, serial.SerialException):
            pass

        self.__thread = threading.Thread(target=self.serialEvent, args=())
        self.__threadFlag = True
        self.__thread.start()
        
    def __showTexts(self):
        print("## Habilitando Comunicacao Serial com o Microcontrolador ##")
        
        result = self.__seeAvailablePorts()

        print("")
        print("Qual Porta Deseja Habilitar (Somente Numeros)?")

        portNumber = 1 #input()
        self.__setPortName(result[int(portNumber)-1])
        print("\nPorta Habilitada: " + self.getPortName())
        
    def __seeAvailablePorts(self):

        i = 1

        print("\nPORTAS DISPONIVEIS")

        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass

        for port in result:
            print("Porta "+str(i)+": "+port)
            i = i + 1

        if (i==1):
            raise Exception('Nao Existem Portas Disponiveis')

        return result

    def __enableSerialComm(self):
        self.serialPort = serial.Serial(self.getPortName(), 
                                        self.getBaudrate(), 
                                        bytesize = serial.EIGHTBITS,
                                        parity = serial.PARITY_NONE,
                                        stopbits = serial.STOPBITS_ONE,
                                        timeout = self.getTimeout())

        self.__threadFlag = True

    def __close(self):
        if (self.serialPort != None):
            self.__threadFlag = False
            self.serialPort.close()


    def serialEvent(self):
        while self.__threadFlag:
            if (self.serialPort.inWaiting()):
                read, write , error = select.select([self.serialPort],[],[], self.getTimeout())
                output = self.serialPort.readline()
                self.__setInputValue(str(output).replace("b'","").replace("\\r\\n'",""))
                self.run()
                
                #print(self.getInputValue(),flush = True)   

    @abc.abstractmethod 
    def run(self):
        pass

    ## SETTERS
    
    def __setBaudrate(self,baudrate):
        self.__baudrate = baudrate
        
    def __setPortName(self,portName):
        self.__portName = portName

    def __setTimeout (self,timeout):
        self.__timeout = timeout

    def __setInputValue(self, inputLine):
        self.__inputValue = inputLine;

    ## GETTERS

    def getBaudrate(self):
        return self.__baudrate

    def getPortName(self):
        return self.__portName

    def getTimeout(self):
        return self.__timeout

    def getInputValue(self):
        return self.__inputValue
