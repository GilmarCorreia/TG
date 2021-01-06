import math
import time

from ax12 import Ax12
from TkinterArmController import TkinterArmController

class Arm(Ax12,object):
    
    _servos = [2,4,6]
    
    _L1 = 6.790  #centimeters
    _L2 = 6.855  #centimeters
    _L3 = 10.650 #centimeters
    _a1 = 1.320  #centimeters

    _pHome = None
    
    def __init__(self):
    	super()
        self.setHome()
        
    def FK(self,theta1, theta2, theta3):
        T1 = math.radians(theta1)
        T2 = math.radians(theta2)
        T3 = math.radians(theta3)
        
        A10 = [[math.cos(T1),  0, -math.sin(T1), (-self._a1*math.cos(T1))],
               [math.sin(T1),  0,  math.cos(T1), (-self._a1*math.sin(T1))],
               [           0, -1,             0,                 self._L1],
               [           0,  0,             0,                  1]]
        
        A21 = [[math.cos(-T2), -math.sin(-T2), 0, (self._L2*math.cos(-T2))],
               [math.sin(-T2),  math.cos(-T2), 0, (self._L2*math.sin(-T2))],
               [            0,              0, 1,                  0],
               [            0,              0, 0,                  1]]
        
        A32 = [[math.cos(-T3),  0, -math.sin(-T3), (self._L3*math.cos(-T3))],
               [math.sin(-T3),  0,  math.cos(-T3), (self._L3*math.sin(-T3))],
               [            0, -1,              0,                  0],
               [            0,  0,              0,                  1]]
        
        A30 = A10*A21*A32
        
        return A30
    
    def IK(self,x,y,z):
        T1 = math.atan2(y,x)
        T3 = -(math.acos((math.pow(x,2) +math.pow(y, 2) + math.pow((z-self._L1),2) - math.pow(self._L3,2) - math.pow((self._L2-self._a1),2))/(2*(self._L2-self._a1)*self._L3)));
        
        tangPhi = (z - self._L1)/(math.sqrt(math.pow(x,2)+math.pow(y,2)))
        tangBeta = (math.sin(T3)*self._L3)/((self._L2-self._a1)+(math.cos(T3)*self._L3))
        
        T2 = math.atan2((tangPhi - tangBeta),(1+(tangPhi*tangBeta)))
        
        angles = [math.degress(T1), math.degress(T2), math.degress(T3)]
        
        anglesToDec = self.anglesToDec(angles)
        
        return anglesToDec

    def getServos(self):
    	return self._servos
    
    def anglesToDec(self, angles):
        anglesToDec = [1023.0-(512.0+((511.0/150.0)*(90.0+angles[0]))), 1023.0-(512.0+((angles[1]-90.0)*(511.0/150.0))), 1023.0-(512.0+(511.0/150.0)*angles[2])]
        
        return anglesToDec
    
    def decToAngles(self, dec):
        decToAngles = [((150.0/511.0)*(511.0-dec[0]))-90.0, ((150.0/511.0)*(511.0-dec[1]))+90.0, (150.0/511.0)*(511.0-dec[2])]
        
        return decToAngles

    def setHome(self):
    	tac = TkinterArmController(self)

    	time.sleep(2)

    	angles = self.decToAngles(tac.getDec())
		orientation = self.FK(angles[0],angles[1],angles[2])
		
		self.pHome[0] = [orientation[0][3],orientation[1][3],orientation[2][3]]
		self.pHome[1] = [tac.getDec()[0],tac.getDec()[1],tac.getDec()[2]]
		
		print("pHome = ({:.2}, {:.2}, {:.2})\n".format(pHome[0][0], pHome[0][1], pHome[0][2]))

