colorSensor = None
right_Motor = None
left_Motor = None


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

colorSensor = myRobot.getDevice('colorSensor')
colorSensor.enable(timeStep)
right_Motor = myRobot.getDevice("right wheel")
encObj[right_Motor] = right_Motor.getPositionSensor()
right_Motor.setPosition(float("inf"))
right_Motor.setVelocity(0)
encObj[right_Motor].enable(timeStep)
encCount[right_Motor] = 0
lastEncReset[encObj[right_Motor]] = 0

left_Motor = myRobot.getDevice("left wheel")
encObj[left_Motor] = left_Motor.getPositionSensor()
left_Motor.setPosition(float("inf"))
left_Motor.setVelocity(0)
encObj[left_Motor].enable(timeStep)
encCount[left_Motor] = 0
lastEncReset[encObj[left_Motor]] = 0

while myRobot.step(timeStep) != -1 and True:
  if gyroEnable:
    updateGyro()
  print(getLSGray(colorSensor.getImageArray()))
