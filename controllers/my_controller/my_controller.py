rotations = None
leftSpeed = None
rightSpeed = None
left_wheel = None
right_wheel = None

# Describe this function...
def stop():
  global rotations, leftSpeed, rightSpeed, left_wheel, right_wheel
  getEncoders(encObj[left_wheel])
  lastEncReset[encObj[left_wheel]] = encCount[encObj[left_wheel]]
  getEncoders(encObj[right_wheel])
  lastEncReset[encObj[right_wheel]] = encCount[encObj[right_wheel]]
  set_speed_for_both_motors(0, 0)

# Describe this function...
def setup_motor_objs():
  global rotations, leftSpeed, rightSpeed, left_wheel, right_wheel
  left_wheel = myRobot.getDevice("left wheel")
  encObj[left_wheel] = left_wheel.getPositionSensor()
  left_wheel.setPosition(float("inf"))
  left_wheel.setVelocity(0)
  encObj[left_wheel].enable(timeStep)
  encCount[left_wheel] = 0
  lastEncReset[encObj[left_wheel]] = 0

  right_wheel = myRobot.getDevice("right wheel")
  encObj[right_wheel] = right_wheel.getPositionSensor()
  right_wheel.setPosition(float("inf"))
  right_wheel.setVelocity(0)
  encObj[right_wheel].enable(timeStep)
  encCount[right_wheel] = 0
  lastEncReset[encObj[right_wheel]] = 0


# Describe this function...
def turn_right():
  global rotations, leftSpeed, rightSpeed, left_wheel, right_wheel
  getEncoders(encObj[left_wheel])
  lastEncReset[encObj[left_wheel]] = encCount[encObj[left_wheel]]
  getEncoders(encObj[right_wheel])
  lastEncReset[encObj[right_wheel]] = encCount[encObj[right_wheel]]
  while myRobot.step(timeStep) != -1 and (getEncoders(encObj[left_wheel]) or encCount[encObj[left_wheel]] - lastEncReset[encObj[left_wheel]]) <= 160:
    if gyroEnable:
      updateGyro()
    set_speed_for_both_motors(50, -48)

# Describe this function...
def turn_left():
  global rotations, leftSpeed, rightSpeed, left_wheel, right_wheel
  getEncoders(encObj[left_wheel])
  lastEncReset[encObj[left_wheel]] = encCount[encObj[left_wheel]]
  getEncoders(encObj[right_wheel])
  lastEncReset[encObj[right_wheel]] = encCount[encObj[right_wheel]]
  while myRobot.step(timeStep) != -1 and (getEncoders(encObj[right_wheel]) or encCount[encObj[right_wheel]] - lastEncReset[encObj[right_wheel]]) <= 160:
    if gyroEnable:
      updateGyro()
    set_speed_for_both_motors(-48, 50)

# Describe this function...
def move_forward_for___of_rotations(rotations):
  global leftSpeed, rightSpeed, left_wheel, right_wheel
  getEncoders(encObj[left_wheel])
  lastEncReset[encObj[left_wheel]] = encCount[encObj[left_wheel]]
  getEncoders(encObj[right_wheel])
  lastEncReset[encObj[right_wheel]] = encCount[encObj[right_wheel]]
  while myRobot.step(timeStep) != -1 and (getEncoders(encObj[left_wheel]) or encCount[encObj[left_wheel]] - lastEncReset[encObj[left_wheel]]) <= rotations * 360:
    if gyroEnable:
      updateGyro()
    set_speed_for_both_motors(50, 50)

# Describe this function...
def set_speed_for_both_motors(leftSpeed, rightSpeed):
  global rotations, left_wheel, right_wheel
  left_wheel.setVelocity((leftSpeed / 100.0) * left_wheel.getMaxVelocity())
  right_wheel.setVelocity((rightSpeed / 100.0) * right_wheel.getMaxVelocity())


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

left_wheel = myRobot.getDevice("left wheel")
encObj[left_wheel] = left_wheel.getPositionSensor()
left_wheel.setPosition(float("inf"))
left_wheel.setVelocity(0)
encObj[left_wheel].enable(timeStep)
encCount[left_wheel] = 0
lastEncReset[encObj[left_wheel]] = 0

right_wheel = myRobot.getDevice("right wheel")
encObj[right_wheel] = right_wheel.getPositionSensor()
right_wheel.setPosition(float("inf"))
right_wheel.setVelocity(0)
encObj[right_wheel].enable(timeStep)
encCount[right_wheel] = 0
lastEncReset[encObj[right_wheel]] = 0

print(getEncoders(encObj[left_wheel]) or encCount[encObj[left_wheel]] - lastEncReset[encObj[left_wheel]])
stop()
for count in range(4):
  move_forward_for___of_rotations(3.55)
  turn_right()
  stop()
