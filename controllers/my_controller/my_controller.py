arm1 = None
arm2 = None
arm3 = None


from controller import Robot
from controller import GPS
from controller import LightSensor
from controller import Motor
from controller import PositionSensor
from controller import Gyro
import math

kernel_size = 10
internal_angle = 0.0

def getLSColor(camImg):
 global kernel_size
 rgb = [0] * 3
 for x in range(0,kernel_size):
  for y in range(0,kernel_size):
   for i in range(0,3):
    rgb[i] += camImg[x][y][i]
 for i in range(0,3):
  rgb[i] = int(rgb[i]/(kernel_size*kernel_size))
 return rgb
def getLSGray(camImg):
 global kernel_size
 gray = 0
 for x in range(0,kernel_size):
  for y in range(0,kernel_size):
   for i in range(0,3):
    gray += camImg[x][y][i]
 gray = int(gray/(3*kernel_size*kernel_size))
 return gray

#updates angle variable according to angular velocity from gyro
#angleCurrent = anglePast + integral of angular velocity over one timeStep since last updated angle
#should be called every time main loop repeats
def updateGyro():
 global internal_angle
 internal_angle += (timeStep / 1000.0) * (gyro.getValues())[1]

#returns current angle of robot relative to starting angle
#angle does not drop to 0 after exceeding 360
#angle % 360 will yield relative angle with maximum 360
def getAngle():
 global internal_angle
 return internal_angle * 180.0 / 3.1415

def getEncoders(posSensor):
  global encCount
  encCount[posSensor] = posSensor.getValue() / 3.1415 * 180.0
  if encCount[posSensor] != encCount[posSensor]:
    encCount[posSensor] = 0
  return False

def getObjAng(coord):
    ang = math.degrees(math.atan(coord[0]/math.fabs(coord[2])))
    return ang

myRobot = Robot()
timeStep = 32
encObj = {}
lastTimeReset = 0
gyroEnable = False
encCount = {}
lastEncReset = {}
myRobot.step(timeStep)

arm1 = myRobot.getDevice("arm1")
encObj[arm1] = arm1.getPositionSensor()
arm1.setPosition(float("inf"))
arm1.setVelocity(0)
encObj[arm1].enable(timeStep)
encCount[arm1] = 0
lastEncReset[encObj[arm1]] = 0

arm2 = myRobot.getDevice("arm2")
encObj[arm2] = arm2.getPositionSensor()
arm2.setPosition(float("inf"))
arm2.setVelocity(0)
encObj[arm2].enable(timeStep)
encCount[arm2] = 0
lastEncReset[encObj[arm2]] = 0

arm3 = myRobot.getDevice("arm3")
encObj[arm3] = arm3.getPositionSensor()
arm3.setPosition(float("inf"))
arm3.setVelocity(0)
encObj[arm3].enable(timeStep)
encCount[arm3] = 0
lastEncReset[encObj[arm3]] = 0

arm2.setVelocity(((-30) / 100.0) * arm2.getMaxVelocity())
initTime = myRobot.getTime()
while myRobot.step(timeStep) != -1:
  if (myRobot.getTime() - initTime) * 1000.0 > 1000:
    break
arm2.setVelocity((0 / 100.0) * arm2.getMaxVelocity())
arm3.setVelocity((30 / 100.0) * arm3.getMaxVelocity())
initTime = myRobot.getTime()
while myRobot.step(timeStep) != -1:
  if (myRobot.getTime() - initTime) * 1000.0 > 400:
    break
arm3.setVelocity((0 / 100.0) * arm3.getMaxVelocity())
