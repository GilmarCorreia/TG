'''
Based on Jesse Merritt's script:
https://github.com/jes1510/python_dynamixels

and Josue Alejandro Savage's Arduino library:
http://savageelectronics.blogspot.it/2011/01/arduino-y-dynamixel-ax-12.html
'''
## -*- coding: utf-8 -*-
from time import sleep
from serial import Serial
import RPi.GPIO as GPIO

class Ax12(object):
    # important AX-12 constants
    # /////////////////////////////////////////////////////////// EEPROM AREA
    AX_MODEL_NUMBER_L = 0
    AX_MODEL_NUMBER_H = 1
    AX_VERSION = 2
    AX_ID = 3
    AX_BAUD_RATE = 4
    AX_RETURN_DELAY_TIME = 5
    AX_CW_ANGLE_LIMIT_L = 6
    AX_CW_ANGLE_LIMIT_H = 7
    AX_CCW_ANGLE_LIMIT_L = 8
    AX_CCW_ANGLE_LIMIT_H = 9
    AX_SYSTEM_DATA2 = 10
    AX_LIMIT_TEMPERATURE = 11
    AX_DOWN_LIMIT_VOLTAGE = 12
    AX_UP_LIMIT_VOLTAGE = 13
    AX_MAX_TORQUE_L = 14
    AX_MAX_TORQUE_H = 15
    AX_RETURN_LEVEL = 16
    AX_ALARM_LED = 17
    AX_ALARM_SHUTDOWN = 18
    AX_OPERATING_MODE = 19
    AX_DOWN_CALIBRATION_L = 20
    AX_DOWN_CALIBRATION_H = 21
    AX_UP_CALIBRATION_L = 22
    AX_UP_CALIBRATION_H = 23

    # ////////////////////////////////////////////////////////////// RAM AREA
    AX_TORQUE_STATUS = 24
    AX_LED_STATUS = 25
    AX_CW_COMPLIANCE_MARGIN = 26
    AX_CCW_COMPLIANCE_MARGIN = 27
    AX_CW_COMPLIANCE_SLOPE = 28
    AX_CCW_COMPLIANCE_SLOPE = 29
    AX_GOAL_POSITION_L = 30
    AX_GOAL_POSITION_H = 31
    AX_GOAL_SPEED_L = 32
    AX_GOAL_SPEED_H = 33
    AX_TORQUE_LIMIT_L = 34
    AX_TORQUE_LIMIT_H = 35
    AX_PRESENT_POSITION_L = 36
    AX_PRESENT_POSITION_H = 37
    AX_PRESENT_SPEED_L = 38
    AX_PRESENT_SPEED_H = 39
    AX_PRESENT_LOAD_L = 40
    AX_PRESENT_LOAD_H = 41
    AX_PRESENT_VOLTAGE = 42
    AX_PRESENT_TEMPERATURE = 43
    AX_REGISTERED_INSTRUCTION = 44
    AX_PAUSE_TIME = 45
    AX_MOVING = 46
    AX_LOCK = 47
    AX_PUNCH_L = 48
    AX_PUNCH_H = 49

    # /////////////////////////////////////////////////////////////// Status Return Levels
    AX_RETURN_NONE = 0
    AX_RETURN_READ = 1
    AX_RETURN_ALL = 2

    # /////////////////////////////////////////////////////////////// Instruction Set
    AX_PING = 1
    AX_READ_DATA = 2
    AX_WRITE_DATA = 3
    AX_REG_WRITE = 4
    AX_ACTION = 5
    AX_RESET = 6
    AX_SYNC_WRITE = 131

    # /////////////////////////////////////////////////////////////// Lengths
    AX_RESET_LENGTH = 2
    AX_ACTION_LENGTH = 2
    AX_ID_LENGTH = 4
    AX_LR_LENGTH = 4
    AX_SRL_LENGTH = 4
    AX_RDT_LENGTH = 4
    AX_LEDALARM_LENGTH = 4
    AX_SHUTDOWNALARM_LENGTH = 4
    AX_TL_LENGTH = 4
    AX_VL_LENGTH = 6
    AX_AL_LENGTH = 7
    AX_CM_LENGTH = 6
    AX_CS_LENGTH = 5
    AX_COMPLIANCE_LENGTH = 7
    AX_CCW_CW_LENGTH = 8
    AX_BD_LENGTH = 4
    AX_TEM_LENGTH = 4
    AX_MOVING_LENGTH = 4
    AX_RWS_LENGTH = 4
    AX_VOLT_LENGTH = 4
    AX_LOAD_LENGTH = 4
    AX_LED_LENGTH = 4
    AX_TORQUE_LENGTH = 4
    AX_POS_LENGTH = 4
    AX_GOAL_LENGTH = 5
    AX_MT_LENGTH = 5
    AX_PUNCH_LENGTH = 5
    AX_SPEED_LENGTH = 5
    AX_GOAL_SP_LENGTH = 7

    # /////////////////////////////////////////////////////////////// Specials
    AX_BYTE_READ = 1
    AX_BYTE_READ_POS = 2
    AX_INT_READ = 2
    AX_ACTION_CHECKSUM = 250
    AX_BROADCAST_ID = 254
    AX_START = 255
    AX_CCW_AL_L = 255
    AX_CCW_AL_H = 3
    AX_LOCK_VALUE = 1
    LEFT = 0
    RIGTH = 1
    RX_TIME_OUT = 10
    TX_DELAY_TIME = 0.0004

    # RPi constants
    RPI_DIRECTION_PIN = 23
    RPI_DIRECTION_TX = GPIO.HIGH
    RPI_DIRECTION_RX = GPIO.LOW
    RPI_DIRECTION_SWITCH_DELAY = 0.0001

    # static variables
    port = None
    gpioSet = False

    def __init__(self):
        if(self.port == None):
            self.port = Serial("/dev/ttyAMA0", baudrate=1000000, timeout=0.001)
            ##self.port.write('A');
        if(not self.gpioSet):
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.RPI_DIRECTION_PIN, GPIO.OUT)
            self.gpioSet = True
        self.direction(self.RPI_DIRECTION_RX)

    connectedServos = []

    # Error lookup dictionary for bit masking
    dictErrors = {  1 : "Input Voltage",
            2 : "Angle Limit",
            4 : "Overheating",
            8 : "Range",
            16 : "Checksum",
            32 : "Overload",
            64 : "Instruction"
            }

    # Custom error class to report AX servo errors
    class axError(Exception) : pass

    # Servo timeout
    class timeoutError(Exception) : pass

    def direction(self,d):
        GPIO.output(self.RPI_DIRECTION_PIN, d)
        sleep(self.RPI_DIRECTION_SWITCH_DELAY)

    def readData(self,id):
        self.direction(self.RPI_DIRECTION_RX)
        reply = self.port.read(8) # [0xff, 0xff, origin, length, error]
        
        try:
            assert ord(reply[0]) == 0xFF
        except:
            e = "Timeout on servo " + str(id)
            raise self.timeoutError(e)

        try :
            length = ord(reply[3]) - 2
            error = ord(reply[4])

            if(error != 0):
                print ("Error from servo: " + self.dictErrors[error] + ' (code  ' + hex(error) + ')')
                return -error
            # just reading error bit
            elif(length == 0):
                return error
            else:
                if(length > 1):
                    reply = self.port.read(2)
                    returnValue = (ord(reply[1])<<8) + (ord(reply[0])<<0)
                else:
                    reply = self.port.read(1)
                    returnValue = ord(reply[0])
                return returnValue
        except Exception as detail:
            raise self.axError(detail)

    def ping(self,id):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_READ_DATA + self.AX_PING))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_READ_DATA))
        self.port.write(chr(self.AX_PING))
        self.port.write(chr(checksum))
        sleep(self.TX_DELAY_TIME)
        
        self.direction(self.RPI_DIRECTION_RX)
        reply = self.port.read(8)
        print ('Ping of motor ' + str(id) + ' is ' + str(ord(reply[5])))

    def factoryReset(self,id, confirm = False):
        if(confirm):
            self.direction(self.RPI_DIRECTION_TX)
            self.port.flushInput()
            checksum = (~(id + self.AX_RESET_LENGTH + self.AX_RESET))&0xff
            self.port.write(chr(self.AX_START))
            self.port.write(chr(self.AX_START))
            self.port.write(chr(id))
            self.port.write(chr(self.AX_RESET_LENGTH))
            self.port.write(chr(self.AX_RESET))
            self.port.write(chr(checksum))
            
            sleep(self.TX_DELAY_TIME)
            # # return self.readData(id)
        else:
            print ("nothing done, please send confirm = True as this fuction reset to the factory default value, i.e reset the motor ID")
            return

    def setID(self, id, newId):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_ID_LENGTH + self.AX_WRITE_DATA + self.AX_ID + newId))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_ID_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_ID))
        self.port.write(chr(newId))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def setBaudRate(self, id, baudRate):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        br = ((2000000/long(baudRate))-1)
        checksum = (~(id + self.AX_BD_LENGTH + self.AX_WRITE_DATA + self.AX_BAUD_RATE + br))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_BD_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_BAUD_RATE))
        self.port.write(chr(br))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def setStatusReturnLevel(self, id, level):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_SRL_LENGTH + self.AX_WRITE_DATA + self.AX_RETURN_LEVEL + level))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_SRL_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_RETURN_LEVEL))
        self.port.write(chr(level))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def setReturnDelayTime(self, id, delay):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_RDT_LENGTH + self.AX_WRITE_DATA + self.AX_RETURN_DELAY_TIME + (int(delay)/2)&0xff))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_RDT_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_RETURN_DELAY_TIME))
        self.port.write(chr((int(delay)/2)&0xff))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def lockRegister(self, id):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_LR_LENGTH + self.AX_WRITE_DATA + self.AX_LOCK + self.AX_LOCK_VALUE))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_LR_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_LOCK))
        self.port.write(chr(self.AX_LOCK_VALUE))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def move(self, id, position):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        p = [position&0xff, position>>8]
        checksum = (~(id + self.AX_GOAL_LENGTH + self.AX_WRITE_DATA + self.AX_GOAL_POSITION_L + p[0] + p[1]))&0xff
    
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_GOAL_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_GOAL_POSITION_L))
        self.port.write(chr(p[0]))
        self.port.write(chr(p[1]))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def moveSpeed(self, id, position, speed):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        p = [position&0xff, position>>8]
        s = [speed&0xff, speed>>8]
        checksum = (~(id + self.AX_GOAL_SP_LENGTH + self.AX_WRITE_DATA + self.AX_GOAL_POSITION_L + p[0] + p[1] + s[0] + s[1]))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_GOAL_SP_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_GOAL_POSITION_L))
        self.port.write(chr(p[0]))
        self.port.write(chr(p[1]))
        self.port.write(chr(s[0]))
        self.port.write(chr(s[1]))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def moveRW(self, id, position):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        p = [position&0xff, position>>8]
        checksum = (~(id + self.AX_GOAL_LENGTH + self.AX_REG_WRITE + self.AX_GOAL_POSITION_L + p[0] + p[1]))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_GOAL_LENGTH))
        self.port.write(chr(self.AX_REG_WRITE))
        self.port.write(chr(self.AX_GOAL_POSITION_L))
        self.port.write(chr(p[0]))
        self.port.write(chr(p[1]))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def moveSpeedRW(self, id, position, speed):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        p = [position&0xff, position>>8]
        s = [speed&0xff, speed>>8]
        checksum = (~(id + self.AX_GOAL_SP_LENGTH + self.AX_REG_WRITE + self.AX_GOAL_POSITION_L + p[0] + p[1] + s[0] + s[1]))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_GOAL_SP_LENGTH))
        self.port.write(chr(self.AX_REG_WRITE))
        self.port.write(chr(self.AX_GOAL_POSITION_L))
        self.port.write(chr(p[0]))
        self.port.write(chr(p[1]))
        self.port.write(chr(s[0]))
        self.port.write(chr(s[1]))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def action(self):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_BROADCAST_ID))
        self.port.write(chr(self.AX_ACTION_LENGTH))
        self.port.write(chr(self.AX_ACTION))
        self.port.write(chr(self.AX_ACTION_CHECKSUM))
        
        #sleep(self.TX_DELAY_TIME)

    def setTorqueStatus(self, id, status):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        ts = 1 if ((status == True) or (status == 1)) else 0
        checksum = (~(id + self.AX_TORQUE_LENGTH + self.AX_WRITE_DATA + self.AX_TORQUE_STATUS + ts))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_TORQUE_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_TORQUE_STATUS))
        self.port.write(chr(ts))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def setLedStatus(self, id, status):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        ls = 1 if ((status == True) or (status == 1)) else 0
        checksum = (~(id + self.AX_LED_LENGTH + self.AX_WRITE_DATA + self.AX_LED_STATUS + ls))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_LED_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_LED_STATUS))
        self.port.write(chr(ls))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def setTemperatureLimit(self, id, temp):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_TL_LENGTH + self.AX_WRITE_DATA + self.AX_LIMIT_TEMPERATURE + temp))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_TL_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_LIMIT_TEMPERATURE))
        self.port.write(chr(temp))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def setVoltageLimit(self, id, lowVolt, highVolt):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_VL_LENGTH + self.AX_WRITE_DATA + self.AX_DOWN_LIMIT_VOLTAGE + lowVolt + highVolt))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_VL_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_DOWN_LIMIT_VOLTAGE))
        self.port.write(chr(lowVolt))
        self.port.write(chr(highVolt))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def setAngleLimit(self, id, cwLimit, ccwLimit):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        cw = [cwLimit&0xff, cwLimit>>8]
        ccw = [ccwLimit&0xff, ccwLimit>>8]
        checksum = (~(id + self.AX_AL_LENGTH + self.AX_WRITE_DATA + self.AX_CW_ANGLE_LIMIT_L + cw[0] + cw[1] + ccw[0] + ccw[1]))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_AL_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_CW_ANGLE_LIMIT_L))
        self.port.write(chr(cw[0]))
        self.port.write(chr(cw[1]))
        self.port.write(chr(ccw[0]))
        self.port.write(chr(ccw[1]))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def setTorqueLimit(self, id, torque):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        mt = [torque&0xff, torque>>8]
        checksum = (~(id + self.AX_MT_LENGTH + self.AX_WRITE_DATA + self.AX_MAX_TORQUE_L + mt[0] + mt[1]))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_MT_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_MAX_TORQUE_L))
        self.port.write(chr(mt[0]))
        self.port.write(chr(mt[1]))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def setPunchLimit(self, id, punch):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        p = [punch&0xff, punch>>8]
        checksum = (~(id + self.AX_PUNCH_LENGTH + self.AX_WRITE_DATA + self.AX_PUNCH_L + p[0] + p[1]))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_PUNCH_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_PUNCH_L))
        self.port.write(chr(p[0]))
        self.port.write(chr(p[1]))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def setCompliance(self, id, cwMargin, ccwMargin, cwSlope, ccwSlope):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_COMPLIANCE_LENGTH + self.AX_WRITE_DATA + self.AX_CW_COMPLIANCE_MARGIN + cwMargin + ccwMargin + cwSlope + ccwSlope))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_COMPLIANCE_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_CW_COMPLIANCE_MARGIN))
        self.port.write(chr(cwMargin))
        self.port.write(chr(ccwMArgin))
        self.port.write(chr(cwSlope))
        self.port.write(chr(ccwSlope))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def setLedAlarm(self, id, alarm):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_LEDALARM_LENGTH + self.AX_WRITE_DATA + self.AX_ALARM_LED + alarm))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_LEDALARM_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_ALARM_LED))
        self.port.write(chr(alarm))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def setShutdownAlarm(self, id, alarm):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_SHUTDOWNALARM_LENGTH + self.AX_WRITE_DATA + self.AX_ALARM_SHUTDOWN + alarm))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_SHUTDOWNALARM_LENGTH))
        self.port.write(chr(self.AX_WRITE_DATA))
        self.port.write(chr(self.AX_ALARM_SHUTDOWN))
        self.port.write(chr(alarm))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        # # return self.readData(id)

    def readTemperature(self, id):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_TEM_LENGTH + self.AX_READ_DATA + self.AX_PRESENT_TEMPERATURE + self.AX_BYTE_READ))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_TEM_LENGTH))
        self.port.write(chr(self.AX_READ_DATA))
        self.port.write(chr(self.AX_PRESENT_TEMPERATURE))
        self.port.write(chr(self.AX_BYTE_READ))
        self.port.write(chr(checksum))
        
        #sleep(self.TX_DELAY_TIME)
        
        self.direction(self.RPI_DIRECTION_RX)
        reply = self.port.read(8)
        print ('Temperature of motor ' + str(id) + ' is ' + str(ord(reply[5])))

    def readPosition(self, id):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_POS_LENGTH + self.AX_READ_DATA + self.AX_PRESENT_POSITION_L + self.AX_BYTE_READ_POS))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_POS_LENGTH))
        self.port.write(chr(self.AX_READ_DATA))
        self.port.write(chr(self.AX_PRESENT_POSITION_L))
        self.port.write(chr(self.AX_BYTE_READ_POS))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        
        self.direction(self.RPI_DIRECTION_RX)

        #Position_Long_Byte = -1
		#Time_Counter = 0
    	#while ((availableData() < 7) & (Time_Counter < self.RX_TIME_OUT)):
		#	Time_Counter++
		#	sleep(0.001)

		while self.port.inWaiting():
			Incoming_Byte = self.port.readLine()
			if ( (Incoming_Byte == 255) ):
				char1 = self.port.readLine()
				char2 = self.port.readLine()
				char3 = self.port.readLine()

				Error_Byte = self.port.readLine()

				if( Error_Byte != 0 ):
					return (Error_Byte * (-1))

				Position_Low_Byte = self.port.readLine();      
				Position_High_Byte = self.port.readLine();
				Position_Long_Byte = Position_High_Byte << 8; 
				Position_Long_Byte = Position_Long_Byte + Position_Low_Byte;

        
        print("Position of Motor " + str(id) + " is: " + str(ord(Position_Long_Byte)))
        return ord(Position_Long_Byte)

    def readVoltage(self, id):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_VOLT_LENGTH + self.AX_READ_DATA + self.AX_PRESENT_VOLTAGE + self.AX_BYTE_READ))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_VOLT_LENGTH))
        self.port.write(chr(self.AX_READ_DATA))
        self.port.write(chr(self.AX_PRESENT_VOLTAGE))
        self.port.write(chr(self.AX_BYTE_READ))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        
        self.direction(self.RPI_DIRECTION_RX)
        reply = self.port.read(8)
        
        print ('Voltage of motor ' + str(id) + ' is ' + str(ord(reply[5]))+ '??')
        

    def readSpeed(self, id):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_SPEED_LENGTH + self.AX_READ_DATA + self.AX_PRESENT_SPEED_L + self.AX_INT_READ))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_SPEED_LENGTH))
        self.port.write(chr(self.AX_READ_DATA))
        self.port.write(chr(self.AX_PRESENT_SPEED_L))
        self.port.write(chr(self.AX_INT_READ))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        
        self.direction(self.RPI_DIRECTION_RX)
        reply = self.port.read(8)
        
        if(ord(reply[6])==1):
            print("Speed of Motor" + str(id) + " is: " + str(ord(reply[5])+256))
        else:
            if(ord(reply[6])==2):
                print("Speed of Motor" + str(id) + " is: " + str(ord(reply[5])+512))
            else:
                if(ord(reply[6])==3):
                    print("Speed of Motor" + str(id) + " is: " + str(ord(reply[5])+768))
                else:
                    print("Speed of Motor" + str(id) + " is: " + str(ord(reply[5])))

    def readLoad(self, id):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_LOAD_LENGTH + self.AX_READ_DATA + self.AX_PRESENT_LOAD_L + self.AX_INT_READ))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_LOAD_LENGTH))
        self.port.write(chr(self.AX_READ_DATA))
        self.port.write(chr(self.AX_PRESENT_LOAD_L))
        self.port.write(chr(self.AX_INT_READ))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)

        self.direction(self.RPI_DIRECTION_RX)
        reply = self.port.read(8)
        
        if(ord(reply[6])==1):
            print("Load of Motor" + str(id) + " is: " + str((float(ord(reply[5])+256)/100)) + "V")
        else:
            if(ord(reply[6])==2):
                print("Load of Motor" + str(id) + " is: " + str((float(ord(reply[5])+512))/100) + "V")
            else:
                if(ord(reply[6])==3):
                    print("Load of Motor" + str(id) + " is: " + str((float(ord(reply[5])+768))/100) + "V")
                else:
                    if(ord(reply[6])==4):
                        print("Load of Motor" + str(id) + " is: " + str((float(ord(reply[5])+1024))/100) + "V")
                    else:
                        print("Load of Motor" + str(id) + " is: " + str(float(ord(reply[5])/100)) + "V")

    def readMovingStatus(self, id):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_MOVING_LENGTH + self.AX_READ_DATA + self.AX_MOVING + self.AX_BYTE_READ))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_MOVING_LENGTH))
        self.port.write(chr(self.AX_READ_DATA))
        self.port.write(chr(self.AX_MOVING))
        self.port.write(chr(self.AX_BYTE_READ))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        
        self.direction(self.RPI_DIRECTION_RX)
        reply = self.port.read(8)
        
        if(ord(reply[5]) == 1):
            print ('Motor ' + str(id) + ' has Moved')
        else:
            print ('Motor ' + str(id) + ' has not Moved')

    def readRWStatus(self, id):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.flushInput()
        checksum = (~(id + self.AX_RWS_LENGTH + self.AX_READ_DATA + self.AX_REGISTERED_INSTRUCTION + self.AX_BYTE_READ))&0xff
        self.port.write(chr(self.AX_START))
        self.port.write(chr(self.AX_START))
        self.port.write(chr(id))
        self.port.write(chr(self.AX_RWS_LENGTH))
        self.port.write(chr(self.AX_READ_DATA))
        self.port.write(chr(self.AX_REGISTERED_INSTRUCTION))
        self.port.write(chr(self.AX_BYTE_READ))
        self.port.write(chr(checksum))
        
        sleep(self.TX_DELAY_TIME)
        
        self.direction(self.RPI_DIRECTION_RX)
        reply = self.port.read(8)

        print ('RWStatus of motor ' + str(id) + ' is ' + str(ord(reply[5])))

    def learnServos(self,minValue=1, maxValue=6, verbose=False) :
        servoList = []
        for i in range(minValue, maxValue + 1):
            try :
                temp = self.ping(i)
                servoList.append(i)
                if verbose: print ("Found servo #" + str(i))
                time.sleep(0.1)

            except Exception as detail:
                if verbose : print ("Error pinging servo #" + str(i) + ': ' + str(detail))
                pass
        return servoList

#
#def playPose() :
#    '''
#    Open a file and move the servos to specified positions in a group move
#    '''
#    infile=open(Arguments.playpose, 'r')    # Open the file
#    poseDict = {}                           # Dictionary to hold poses and positions
#    if Arguments.verbose : print "Reading pose from", Arguments.playpose
#    for line in infile.readlines() :        # Read the file and step through it
#        servo = int(line.split(':')[0])     # Servo is first
#        position = int(line.split(':')[1])  # Position is second
#        poseDict[servo]=position            # add the servo to the Dictionary
#
#    groupMove2(poseDict)
#
#
#
#def writePose() :
#    '''
#    Read the servos and save the positions to a file
#    '''
#    of = open(Arguments.savepose, 'w')      # open the output file
#    pose = getPose2(connectedServos)        # get the positions
#    if Arguments.verbose :
#        print "Servo Positions"
#        print "---------------"
#
#    for key in  pose.keys():                # step through the keys, writing to the file
#        if Arguments.verbose : print "Servo " + str(key), pose[key]
#        of.write(str(key) + ':' + str(pose[key]) + '\n')    # Write to the file
#
#    if Arguments.verbose :
#        print "Wrote pose to " + Arguments.savepose
#        print
#
#    of.close()      # close the file
#
