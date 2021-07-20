sensor = None
right_motor = None
left_motor = None


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

sensor = myRobot.getDevice('distance sensor')
sensor.enable(timeStep)
right_motor = myRobot.getDevice("right wheel")
encObj[right_motor] = right_motor.getPositionSensor()
right_motor.setPosition(float("inf"))
right_motor.setVelocity(0)
encObj[right_motor].enable(timeStep)
encCount[right_motor] = 0
lastEncReset[encObj[right_motor]] = 0

left_motor = myRobot.getDevice("left wheel")
encObj[left_motor] = left_motor.getPositionSensor()
left_motor.setPosition(float("inf"))
left_motor.setVelocity(0)
encObj[left_motor].enable(timeStep)
encCount[left_motor] = 0
lastEncReset[encObj[left_motor]] = 0

while myRobot.step(timeStep) != -1 and 1:
  if gyroEnable:
    updateGyro()
  if (sensor.getValue()) > 100:
    left_motor.setVelocity((70 / 100.0) * left_motor.getMaxVelocity())
    right_motor.setVelocity((70 / 100.0) * right_motor.getMaxVelocity())
  else:
    while myRobot.step(timeStep) != -1 and (sensor.getValue()) <= 100 and (sensor.getValue()) >= 50:
      if gyroEnable:
        updateGyro()
      right_motor.setVelocity(((sensor.getValue()) / 100.0) * right_motor.getMaxVelocity())
      left_motor.setVelocity(((sensor.getValue()) / 100.0) * left_motor.getMaxVelocity())
      print(sensor.getValue())
      print(sensor.getValue())
    right_motor.setVelocity((0 / 100.0) * right_motor.getMaxVelocity())
    left_motor.setVelocity((0 / 100.0) * left_motor.getMaxVelocity())
    break
