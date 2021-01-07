## -*- coding: utf-8 -*-

import math
import cmath
import time
import numpy as np

from ax12 import Ax12
from TkinterArmController import TkinterArmController

class Arm():
    
    arm = None
    _servos = [2,4,6]
    
    _L1 = 6.790  #centimeters
    _L2 = 6.855  #centimeters
    _L3 = 10.650 #centimeters
    _a1 = 1.320  #centimeters

    maxT3 = 866
    minT3 = 160
    maxT2 = 823
    minT2 = 154 
    maxT1 = 1023
    minT1 = 0

    pHome = [[None, None, None],
             [None,None,None]]
    
    def __init__(self):
        self.arm = Ax12()
        self.setHome()
        
    def FK(self,theta1, theta2, theta3):

    	angles = [theta1,theta2,theta3]

    	[theta1,theta2,theta3] = self.verifyAngles(angles)

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
        
        A30 = np.matmul(A10,np.matmul(A21,A32))
        
        return A30
    
    def IK(self,x,y,z):
        T1 = math.atan2(y,x)
    	arg = (math.pow(x,2) +math.pow(y, 2) + math.pow((z-self._L1),2) - math.pow(self._L3,2) - math.pow((self._L2-self._a1),2))/(2*(self._L2-self._a1)*self._L3)

    	T3 = -(cmath.acos(arg)).real
        
    	tangPhi = (z - self._L1)/(math.sqrt(math.pow(x,2)+math.pow(y,2)))
    	tangBeta = (math.sin(T3)*self._L3)/((self._L2-self._a1)+(math.cos(T3)*self._L3))
       
    	T2 = math.atan2((tangPhi - tangBeta),(1+(tangPhi*tangBeta)))
        
        angles = [math.degrees(T1), math.degrees(T2), math.degrees(T3)]
        
        anglesToDec = self.verifyDec(self.anglesToDec(angles))
        
        return anglesToDec

    def verifyAngles(self, angles):
    
    	if angles[0] > self.maxT1:
    		angles[0] = self.maxT1
    	elif angles[0] < self.minT1:
    		angles[0] = self.minT1 

    	if angles[1] > self.maxT2:
    		angles[1] = self.maxT2
    	elif angles[1] < self.minT2:
    		angles[1] = self.minT2

    	if angles[2] > self.maxT3:
    		angles[2] = self.maxT3
    	elif angles[2] < self.minT3:
    		angles[2] = self.minT3

    	return angles

    def verifyDec(self, decs):

    	if decs[0] > self.maxT1:
    		decs[0] = self.maxT1
    	elif decs[0] < self.minT1:
    		decs[0] = self.minT1 

    	if decs[1] > self.maxT2:
    		decs[1] = self.maxT2
    	elif decs[1] < self.minT2:
    		decs[1] = self.minT2

    	if decs[2] > self.maxT3:
    		decs[2] = self.maxT3
    	elif decs[2] < self.minT3:
    		decs[2] = self.minT3

    	return decs

    def getServos(self):
        return self._servos

    def maxAngles(self):
    	maxAngles = self.decToAngles(self.maxT1, self.maxT2, self.maxT3)

    	return maxAngles

    def minAngles(self):
    	minAngles = self.decToAngles(self.minT1, self.minT2, self.minT3)

    	return minAngles
    
    def anglesToDec(self, angles):
        anglesToDec = [1023.0-(512.0+((511.0/150.0)*(90.0+angles[0]))), 1023.0-(512.0+((angles[1]-90.0)*(511.0/150.0))), 1023.0-(512.0+(511.0/150.0)*angles[2])]

        return anglesToDec
    
    def decToAngles(self, dec):
        decToAngles = [((150.0/511.0)*(511.0-dec[0]))-90.0, ((150.0/511.0)*(511.0-dec[1]))+90.0, (150.0/511.0)*(511.0-dec[2])]

        return decToAngles
    
    def coveredPath(self):

        x_p = np.arange(0,10,0.1)
        y_p = np.arange(-5,5,0.1)
        z_p = [0] * len(x_p)

        path = [[x,y,0] for x in x_p for y in y_p]
    
        return path

    def verifyPoint(self, point):
        angles = self.IK(point[0],point[1],point[2])
        angles = self.decToAngles(angles)
        posArm = self.FK(angles[0],angles[1],angles[2])
        
        print("desPoint: " + str(point[0]) + ", actualPoint: " + str(posArm[0][3]))

        x = point[0] - posArm[0][3]
        y = point[1] - posArm[1][3]
        z = point[2] - posArm[2][3]
        
        dist = math.sqrt(math.pow(x,2) + math.pow(y,2) + math.pow(z,2))

        print(dist)
        if dist <= 0.5:
            return True
        else:
            return False

    def runMapping(self):

        path = self.coveredPath()

        for point in path:
            if self.verifyPoint(point):
                pass
                #print("x: " + str(point[0]) + ", y: " + str(point[1]) + ", z: " + str(point[2]))
                #self.arm.move(self.IK(point[0],point[1],point[2]))
                #time.sleep(0.5)

    def setHome(self):
        self.arm.move(self._servos[0],204)
        self.arm.move(self._servos[1],818)
        self.arm.move(self._servos[2],512)
        
        tac = TkinterArmController(self)

        time.sleep(2)

        angles = self.decToAngles(tac.getDec())
        orientation = self.FK(angles[0],angles[1],angles[2])
        
        self.pHome[0] = [orientation[0][3],orientation[1][3],orientation[2][3]]
        self.pHome[1] = [tac.getDec()[0],tac.getDec()[1],tac.getDec()[2]]
        
        print("pHome = ({:.2f}, {:.2f}, {:.2f})\n".format(self.pHome[0][0], self.pHome[0][1], self.pHome[0][2]))

        self.runMapping()

    

