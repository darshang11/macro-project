from basehat import UltrasonicSensor, LineFinder
from buildhat import Motor
import time

SPEED = 30
LEFT_FLIP = 1
RIGHT_FLIP = -1
STOP_DISTANCE = 20 # measured in cm
GENTLE = 0.5
SHARP = -0.3
LOST_TIME_LIMIT = 1

leftMotor = Motor('A')
rightMotor = Motor('B')
leftLineSensor = LineFinder(5)
middleLineSensor = LineFinder(16)
rightLineSensor = LineFinder(18)
ultraSensor = UltrasonicSensor(22)


def drive(leftSpeed, rightSpeed):
  leftMotor.start(leftSpeed * LEFT_FLIP)
  rightMotor.start(rightSpeed * RIGHT_FLIP)

def stop():
  leftMotor.stop()
  rightMotor.stop()

def detectObject(stopDistance):
  distance = ultraSensor.getDist
  if distance is not None and distance <= stopDistance:
    return True
  return False

def readLineFinders():
  leftSees = not leftLineSensor.line_detected
  middleSees = not middleLineSensor.line_detected
  rightSees = not rightLineSensor.line_detected
  return leftSees, middleSees, rightSees

def printLineFinders(leftSees, middleSees, rightSees):
  left = seesLine(leftSees)
  middle = seesLine(middleSees)
  right = seesLine(rightSees)
  print(f'{left} {middle} {right}')

def seesLine(value):
  if value:
    return "B"
  return "w"

def decideDrive(left, middle, right):
  values = (left, middle, right)
  if values == (False, True, False) or values == (True, True, True) or values == (True, False, True):
    return SPEED, SPEED                  # straight
  elif values == (True, True, False):
    return GENTLE * SPEED, SPEED         # gentle left
  elif values == (True, False, False):
    return SHARP * SPEED, SPEED          # sharp left
  elif values == (False, True, True):
    return SPEED, GENTLE * SPEED         # gentle right
  elif values == (False, False, True):
    return SPEED, SHARP * SPEED          # sharp right
  else:
    return None                          # lost
    

def main():
  counter = 0
  lastSeen = time.time()
  lastRight = SPEED
  lastLeft = SPEED
  blocked = False
  while True:
    time.sleep(0.02)
    if(counter % 5 == 0):
      blocked = detectObject(STOP_DISTANCE)
      if blocked:
        stop()
        lastSeen = time.time() # waiting doesn't count as being lost
    else:
      left, middle, right = readLineFinders()
      printLineFinders(left, middle, right)
      speeds = decideDrive(left, middle, right)
      if speeds is None:
        interval = time.time() - lastSeen
        while(interval < LOST_TIME_LIMIT):
          drive(lastLeft, lastRight)
          interval = time.time() - lastSeen
          stop()
      else:
        leftSpeed, rightSpeed = speeds
        lastSeen = time.time()
        lastRight = rightSpeed
        lastLeft = leftSpeed
        drive(leftSpeed, rightSpeed)
      
try: main()
except KeyboardInterrupt:
  print("Keyboard Interrupt")
  stop()
finally: 
  stop()
    
