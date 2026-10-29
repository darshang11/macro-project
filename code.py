from basehat import UltrasonicSensor, LineFinder, IMUSensor
from buildhat import Motor
import math
import time

SPEED = 30
LEFT_FLIP = 1
RIGHT_FLIP = -1
STOP_DISTANCE = 20 # measured in cm
GENTLE = 0.5
SHARP = -0.3
LOST_TIME_LIMIT = 1

# magnets - measure all of these on the robot
MAGNET_THRESHOLD = 300    # change needed to count as a magnet, set halfway between background bounce and the peak
DROP_MAGNET = 1           # which magnet to drop at (1 = the first one we pass)
WHEEL_DIAMETER_CM = 5.6   # drive wheel diameter with the tire on
IMU_AHEAD_CM = 15         # magnet sensor distance in front of the motor axle
DROPPER_BEHIND_CM = 8     # cargo center distance behind the motor axle
DROP_DEGREES = 90         # how far the cargo motor turns to release
CARGO_SPEED = 30

leftMotor = Motor('A')
rightMotor = Motor('B')
cargoMotor = Motor('C')
leftMotor.plimit(0.9)
rightMotor.plimit(0.9)
leftLineSensor = LineFinder(5)
middleLineSensor = LineFinder(16)
rightLineSensor = LineFinder(18)
ultraSensor = UltrasonicSensor(22)
imu = IMUSensor()


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

def measureNormal():
  total = [0, 0, 0]
  for i in range(20):
    x, y, z = imu.getMag()
    total[0] += x
    total[1] += y
    total[2] += z
    time.sleep(0.02)
  return total[0] / 20, total[1] / 20, total[2] / 20

def magnetChange(normal):
  x, y, z = imu.getMag()
  dx = x - normal[0]
  dy = y - normal[1]
  dz = z - normal[2]
  return math.sqrt(dx * dx + dy * dy + dz * dz)

def distanceTravelled():
  leftDegrees = leftMotor.get_position() * LEFT_FLIP
  rightDegrees = rightMotor.get_position() * RIGHT_FLIP
  degrees = (leftDegrees + rightDegrees) / 2
  return degrees * math.pi * WHEEL_DIAMETER_CM / 360

def dropCargo():
  cargoMotor.run_for_degrees(DROP_DEGREES, CARGO_SPEED)
  time.sleep(0.5)
  cargoMotor.run_for_degrees(-DROP_DEGREES, CARGO_SPEED)


def main():
  counter = 0
  lastSeen = time.time()
  lastRight = SPEED
  lastLeft = SPEED
  blocked = False

  print("Measuring normal magnet reading, keep magnets away")
  normal = measureNormal()
  magnetCount = 0
  onMagnet = False
  peakChange = 0
  peakPosition = 0
  dropAt = None             # distance where the dropper is over the magnet

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
        if interval < LOST_TIME_LIMIT:
          drive(lastLeft, lastRight)
        else:
          stop()
      else:
        leftSpeed, rightSpeed = speeds
        lastSeen = time.time()
        lastRight = rightSpeed
        lastLeft = leftSpeed
        drive(leftSpeed, rightSpeed)
      change = magnetChange(normal)
      if not onMagnet and change > MAGNET_THRESHOLD:
        onMagnet = True
        peakChange = change
        peakPosition = distanceTravelled()
      elif onMagnet:
        if change > peakChange:          # still getting stronger, remember where
          peakChange = change
          peakPosition = distanceTravelled()
        if change < MAGNET_THRESHOLD / 2: # off the magnet
          onMagnet = False
          magnetCount += 1
          print(f"Magnet {magnetCount}, peak {peakChange:.0f} at {peakPosition:.1f} cm")
          if magnetCount == DROP_MAGNET:
            dropAt = peakPosition + IMU_AHEAD_CM + DROPPER_BEHIND_CM
      if dropAt is not None and distanceTravelled() >= dropAt:
        stop()
        print("Dropping cargo")
        dropCargo()
        break
    counter += 1

try:
  main()
except KeyboardInterrupt:
  print("Keyboard Interrupt")
finally:
  stop()
