from basehat import UltrasonicSensor, LineFinder, IMUSensor
from buildhat import Motor
import math
import time

SPEED = 20
LEFT_FLIP = 1
RIGHT_FLIP = -1
STOP_DISTANCE = 20        # cm, stop when closer than this
RESUME_DISTANCE = 25      # cm, only start moving again once farther than this
GENTLE = 0.5
SHARP = -0.3
LOST_TIME_LIMIT = 1.5     # seconds of repeating the last command (longer since we're slower)
SEARCH_TIME_LIMIT = 2     # seconds of spinning toward the last side the line was seen
SEARCH_SPEED = 15
GENTLE_SCALE = 0.85       # fraction of normal speed in gentle turns
SHARP_SCALE = 0.7         # fraction of normal speed in sharp turns
MIN_SPEED = 8          # smallest wheel power we send, below this the motors may stall
MAX_STEP = 10             # most a wheel's power can change per loop (smoothing), bigger = snappier
CHECK_SENSORS = True      # run the startup sensor check before driving
CHECK_SAMPLES = 20        # readings per sensor in the check
USE_PID = True            # True = error number + PID steering, False = the old 5 state table
KP = 8                 # how hard to steer per step of error (error is -2 to 2)
KI = 0                    # leave at 0 at first, with 3 digital sensors it rarely helps
KD = 3                  # damping, resists fast changes in error
INTEGRAL_LIMIT = 5        # cap on the integral so it can't wind up
HOLD = "hold"             # marker for the impossible pattern (True, False, True)
DEBUG = True        # prints line sensor readings every loop (slow, turn off for speed tests)
RATE_EVERY = 100          # print loops per second every this many loops

# magnets - set USE_MAGNET = False to just test line following (no IMU or cargo motor needed)
USE_MAGNET = False
MAGNET_THRESHOLD = 300    # change in field strength (uT) that counts as a magnet, measure on the robot
MAGNET_CONFIRM = 3        # readings in a row above threshold before we believe it
BASELINE_ALPHA = 0.02     # how fast the baseline follows slow drift (0 = never, 1 = instantly)
DROP_MAGNET = 1           # which magnet to drop at (1 = the first one we pass)
WHEEL_DIAMETER_CM = 5.6   # drive wheel diameter with the tire on
IMU_AHEAD_CM = 15         # magnet sensor distance in front of the drive axle
DROPPER_BEHIND_CM = 8     # cargo center distance behind the drive axle
DROP_DEGREES = 90         # how far the cargo motor turns to release
CARGO_SPEED = 30
SLOW_ZONE_CM = 10         # start slowing down this far before the drop point
SLOW_SCALE = 0.5          # fraction of normal speed inside the slow zone

print("Setting up motors and sensors")
leftMotor = Motor('A')
rightMotor = Motor('B')
leftMotor.plimit(0.9)
rightMotor.plimit(0.9)
leftLineSensor = LineFinder(5)
middleLineSensor = LineFinder(16)
rightLineSensor = LineFinder(18)
ultraSensor = UltrasonicSensor(22)
if USE_MAGNET:
  cargoMotor = Motor('C')
  imu = IMUSensor()
print("Setup done")


currentLeft = 0           
currentRight = 0

def approach(current, target):
  # move current toward target by at most MAX_STEP
  if target > current + MAX_STEP:
    return current + MAX_STEP
  if target < current - MAX_STEP:
    return current - MAX_STEP
  return target

def drive(leftSpeed, rightSpeed):
  global currentLeft, currentRight
  currentLeft = approach(currentLeft, leftSpeed)
  currentRight = approach(currentRight, rightSpeed)
  leftMotor.start(currentLeft * LEFT_FLIP)
  rightMotor.start(currentRight * RIGHT_FLIP)

def stop():
  # stopping is immediate, not smoothed, so obstacles and Ctrl+C still stop right away
  global currentLeft, currentRight
  currentLeft = 0
  currentRight = 0
  leftMotor.stop()
  rightMotor.stop()

def sampleSensors():
  # returns how many of the samples each sensor saw the line in (left, middle, right)
  counts = [0, 0, 0]
  for i in range(CHECK_SAMPLES):
    readings = readLineFinders()
    for j in range(3):
      if readings[j]:
        counts[j] += 1
    time.sleep(0.02)
  return counts

def sensorCheck():
  names = ["LEFT", "MIDDLE", "RIGHT"]
  allGood = True
  input("Sensor check: put ALL three sensors over plain white, then press Enter ")
  counts = sampleSensors()
  for j in range(3):
    if counts[j] == 0:
      print(f"  {names[j]}: OK on white (0/{CHECK_SAMPLES} saw line)")
    else:
      print(f"  {names[j]}: WARNING saw line {counts[j]}/{CHECK_SAMPLES} times on white")
      allGood = False
  for j in range(3):
    input(f"Put the line under the {names[j]} sensor only, then press Enter ")
    counts = sampleSensors()
    if counts[j] == CHECK_SAMPLES:
      print(f"  {names[j]}: OK on line ({counts[j]}/{CHECK_SAMPLES})")
    else:
      print(f"  {names[j]}: WARNING saw line only {counts[j]}/{CHECK_SAMPLES} times, adjust sensitivity or height")
      allGood = False
    others = [counts[k] for k in range(3) if k != j]
    if max(others) > 0:
      print(f"  note: the other sensors also saw the line, the strip may be too wide or the sensors too close")
  if allGood:
    print("Sensor check passed")
  else:
    print("Sensor check found problems, see warnings above")
  input("Place the robot at the start and press Enter to begin ")

def updateBlocked(blocked):
  distance = ultraSensor.getDist
  if distance is None:                  # keep the old state
    return blocked, distance
  if blocked:
    return distance <= RESUME_DISTANCE, distance
  return distance <= STOP_DISTANCE, distance

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

def slowDown(leftSpeed, rightSpeed, scale):
  # scale both wheels; keep any moving wheel at least MIN_SPEED so it doesn't stall
  newLeft = leftSpeed * scale
  newRight = rightSpeed * scale
  if newLeft != 0 and abs(newLeft) < MIN_SPEED:
    newLeft = MIN_SPEED if newLeft > 0 else -MIN_SPEED
  if newRight != 0 and abs(newRight) < MIN_SPEED:
    newRight = MIN_SPEED if newRight > 0 else -MIN_SPEED
  return newLeft, newRight

def decideDrive(left, middle, right):
  values = (left, middle, right)
  if values == (False, True, False) or values == (True, True, True):
    return SPEED, SPEED                  # straight
  elif values == (True, False, True):
    return HOLD                          # impossible with touching sensors; treat as noise and keep going
  elif values == (True, True, False):
    return slowDown(GENTLE * SPEED, SPEED, GENTLE_SCALE)    # gentle left
  elif values == (True, False, False):
    return slowDown(SHARP * SPEED, SPEED, SHARP_SCALE)      # sharp left
  elif values == (False, True, True):
    return slowDown(SPEED, GENTLE * SPEED, GENTLE_SCALE)    # gentle right
  elif values == (False, False, True):
    return slowDown(SPEED, SHARP * SPEED, SHARP_SCALE)      # sharp right
  else:
    return None                          # lost

lastError = 0
integral = 0

def resetPid():
  global lastError, integral
  lastError = 0
  integral = 0

def lineError(left, middle, right):
  values = (left, middle, right)
  if values == (False, True, False) or values == (True, True, True):
    return 0
  elif values == (True, True, False):
    return -1
  elif values == (True, False, False):
    return -2
  elif values == (False, True, True):
    return 1
  elif values == (False, False, True):
    return 2
  elif values == (True, False, True):
    return HOLD
  else:
    return None                          # lost

def pidSpeeds(error):
  global lastError, integral
  integral = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, integral + error))
  derivative = error - lastError
  turn = KP * error + KI * integral + KD * derivative
  lastError = error
  # slow the base speed more the bigger the error
  base = SPEED * (1.0, GENTLE_SCALE, SHARP_SCALE)[abs(error)]
  # positive error = line on the right = turn right = left wheel faster
  leftSpeed = max(-SPEED, min(SPEED, base + turn))
  rightSpeed = max(-SPEED, min(SPEED, base - turn))
  # keep any moving wheel at least MIN_SPEED so it doesn't stall
  if leftSpeed != 0 and abs(leftSpeed) < MIN_SPEED:
    leftSpeed = MIN_SPEED if leftSpeed > 0 else -MIN_SPEED
  if rightSpeed != 0 and abs(rightSpeed) < MIN_SPEED:
    rightSpeed = MIN_SPEED if rightSpeed > 0 else -MIN_SPEED
  return leftSpeed, rightSpeed

def fieldStrength():
  # length of the magnetic field vector
  x, y, z = imu.getMag()
  return math.sqrt(x * x + y * y + z * z)

def measureNormal():
  total = 0
  for i in range(20):
    total += fieldStrength()
    time.sleep(0.02)
  return total / 20

def distanceTravelled():
  leftDegrees = leftMotor.get_position() * LEFT_FLIP
  rightDegrees = rightMotor.get_position() * RIGHT_FLIP
  degrees = (leftDegrees + rightDegrees) / 2
  return degrees * math.pi * WHEEL_DIAMETER_CM / 360

def dropCargo():
  cargoMotor.run_for_degrees(DROP_DEGREES, CARGO_SPEED)
  time.sleep(0.5)
  cargoMotor.run_for_degrees(-DROP_DEGREES, CARGO_SPEED)

def setMode(oldMode, newMode, message):
  if newMode != oldMode:
    print(f"[{time.time() - startTime:6.2f}s] {message}")
  return newMode


def main():
  counter = 0
  lastSeen = time.time()
  lastRight = SPEED
  lastLeft = SPEED
  lastSide = 0              # -1 line last seen on the left, 1 on the right, 0 unknown
  blocked = False
  mode = ""
  rateStart = time.time()

  # magnet state
  baseline = 0
  magnetCount = 0
  onMagnet = False
  aboveCount = 0
  peakChange = 0
  peakPosition = 0
  dropAt = None             # distance where the dropper is over the magnet
  inSlowZone = False
  if USE_MAGNET:
    print("Measuring normal magnet reading, keep magnets away")
    baseline = measureNormal()
    print(f"Baseline field strength: {baseline:.1f}")

  print("Starting line following, press Ctrl+C to stop")
  while True:
    time.sleep(0.02)
    if(counter % 5 == 0):
      wasBlocked = blocked
      blocked, distance = updateBlocked(blocked)
      if blocked != wasBlocked:
        print(f"Ultrasonic: {distance} cm, blocked changed {wasBlocked} -> {blocked}")
    if blocked:
      stop()
      lastSeen = time.time() # waiting doesn't count as being lost
      mode = setMode(mode, "blocked", "Object in the way, stopped")
    else:
      left, middle, right = readLineFinders()
      if DEBUG:
        printLineFinders(left, middle, right)
      oldSide = lastSide
      if left and not right:
        lastSide = -1
      elif right and not left:
        lastSide = 1
      if lastSide != oldSide:
        print(f"lastSide changed {oldSide} -> {lastSide} (-1 left, 1 right)")
      if USE_PID:
        error = lineError(left, middle, right)
        if error is None:
          speeds = None
        elif error == HOLD:
          speeds = HOLD
        else:
          speeds = pidSpeeds(error)
      else:
        speeds = decideDrive(left, middle, right)
      if speeds is None:
        resetPid()
        interval = time.time() - lastSeen
        if interval < LOST_TIME_LIMIT:
          mode = setMode(mode, "lost repeat", f"Line lost, repeating last command L{lastLeft:.0f} R{lastRight:.0f}")
          drive(lastLeft, lastRight)
        elif interval < LOST_TIME_LIMIT + SEARCH_TIME_LIMIT and lastSide != 0:
          mode = setMode(mode, "lost search", f"Still lost after {interval:.1f}s, spinning toward side {lastSide}")
          drive(lastSide * SEARCH_SPEED, -lastSide * SEARCH_SPEED)   # spin toward where the line was last seen
        else:
          mode = setMode(mode, "lost stop", f"Gave up after {interval:.1f}s, stopped (lastSide {lastSide})")
          stop()
      elif speeds == HOLD:
        lastSeen = time.time()           # sensors see something, so we are not lost
        mode = setMode(mode, "hold", "Impossible pattern B w B, holding last command")
        drive(lastLeft, lastRight)
      else:
        leftSpeed, rightSpeed = speeds
        lastSeen = time.time()
        lastRight = rightSpeed
        lastLeft = leftSpeed
        pattern = f"{seesLine(left)}{seesLine(middle)}{seesLine(right)}"
        if dropAt is not None and dropAt - distanceTravelled() < SLOW_ZONE_CM:
          if not inSlowZone:
            print(f"Within {SLOW_ZONE_CM} cm of the drop point, slowing down")
            inSlowZone = True
          leftSpeed = leftSpeed * SLOW_SCALE
          rightSpeed = rightSpeed * SLOW_SCALE
        mode = setMode(mode, f"drive {leftSpeed:.0f} {rightSpeed:.0f}", f"Pattern {pattern}, driving L{leftSpeed:.0f} R{rightSpeed:.0f}")
        drive(leftSpeed, rightSpeed)

      if USE_MAGNET:
        strength = fieldStrength()
        change = abs(strength - baseline)
        if not onMagnet:
          if change > MAGNET_THRESHOLD:
            aboveCount += 1
            if change > peakChange:
              peakChange = change
              peakPosition = distanceTravelled()
            if aboveCount >= MAGNET_CONFIRM:
              onMagnet = True
              print(f"Magnet detected, change {change:.0f}")
          else:
            aboveCount = 0
            peakChange = 0
            if change < MAGNET_THRESHOLD / 2:   # quiet, so let the baseline follow slow drift
              baseline = baseline * (1 - BASELINE_ALPHA) + strength * BASELINE_ALPHA
        else:
          if change > peakChange:          # still getting stronger, remember where
            peakChange = change
            peakPosition = distanceTravelled()
          if change < MAGNET_THRESHOLD / 2: # off the magnet
            onMagnet = False
            aboveCount = 0
            magnetCount += 1
            print(f"Magnet {magnetCount}, peak {peakChange:.0f} at {peakPosition:.1f} cm")
            peakChange = 0
            if magnetCount == DROP_MAGNET:
              dropAt = peakPosition + IMU_AHEAD_CM + DROPPER_BEHIND_CM
              print(f"Drop point set at {dropAt:.1f} cm (now at {distanceTravelled():.1f} cm)")
        if dropAt is not None and distanceTravelled() >= dropAt:
          stop()
          print("Dropping cargo")
          dropCargo()
          print("Cargo dropped, done")
          break
    counter += 1
    if counter % RATE_EVERY == 0:
      now = time.time()
      print(f"Loop rate: {RATE_EVERY / (now - rateStart):.1f} loops/sec")
      rateStart = now

try:
  if CHECK_SENSORS:
    sensorCheck()
  startTime = time.time()
  main()
except KeyboardInterrupt:
  print("Keyboard Interrupt")
finally:
  stop()
  print("Motors stopped")
