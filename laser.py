#!/usr/bin/python

# Not designed with Python 3 in mind

# Bring in libraries
from picamera.array import PiRGBArray
from picamera import PiCamera
from math import sqrt, asin
import RPi.GPIO as GPIO
import numpy as np
import time
import cv2

# Use XWindows?
XWIN = False

# Define constants for motor
FREQ = 60
MOTOR_LS = 13 # ENB
MOTOR_LF = 5  # IN3
MOTOR_LB = 6  # IN4
MOTOR_RS = 12 # ENA
MOTOR_RF = 16 # IN2
MOTOR_RB = 20 # IN1

# Define constraints for camera
dimX = 640
dimY = 480
centerX = float(dimX/2)
centerY = float(dimY)

# Define upper and lower limits of target color
upper = np.array([25, 25, 255], dtype="uint8")
lower = np.array([0, 0, 100], dtype="uint8")

# Define camera and set defaults
camera = PiCamera(resolution=(dimX, dimY), framerate=30)
camera.awb_mode = 'fluorescent'
camera.brightness = 60
camera.contrast = 60
camera.exposure_mode = 'off'
camera.image_effect = 'colorpoint'
camera.image_effect_params = 1
camera.iso = 100
camera.shutter_speed = 10320

# Define calibration option constants
CALIBRATE_NONE = 0
CALIBRATE_BRIGHTNESS = 1
CALIBRATE_CONTRAST = 2
CALIBRATE_ISO = 3
CALIBRATE_SPEED = 4
calibrating = CALIBRATE_NONE

# Define history for averaging
HISTORY_LEN = 6
bear = []
#speed = []

# Create array to store pixels from camera frames
rawCapture = PiRGBArray(camera, size=(dimX, dimY))

# Let the camera rest for a millisecond
time.sleep(0.1)

# Setup timing variables
cycletime = time.time()
deadline = cycletime
angle = 0

# Prepare GPIO for motor use
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Setup Right motor GPIO pins
GPIO.setup(MOTOR_RS, GPIO.OUT)
GPIO.setup(MOTOR_RF, GPIO.OUT)
GPIO.setup(MOTOR_RB, GPIO.OUT)

# Setup Left motor GPIO pins
GPIO.setup(MOTOR_LS, GPIO.OUT)
GPIO.setup(MOTOR_LF, GPIO.OUT)
GPIO.setup(MOTOR_LB, GPIO.OUT)

# Setup PWM on "enable" pins
motorR = GPIO.PWM(MOTOR_RS, FREQ)
motorL = GPIO.PWM(MOTOR_LS, FREQ)

# Start PWM
motorR.start(0)
motorL.start(0)

# Hard stop doesn't seem to work on these motors, but this should work
def RightHardStop():
    GPIO.output(MOTOR_RF, GPIO.LOW)
    GPIO.output(MOTOR_RB, GPIO.LOW)
    motorR.ChangeDutyCycle(100)

# Set speed for right motor. Argument takes percentage.
def RightMotor(speed):
    s = int(speed * 20) * 5
    if s > 0:
        if s > 100: s = 100
        GPIO.output(MOTOR_RF, GPIO.HIGH)
        GPIO.output(MOTOR_RB, GPIO.LOW)
        motorR.ChangeDutyCycle(s)
    elif s < 0:
        if s < -100: s = -100
        GPIO.output(MOTOR_RB, GPIO.HIGH)
        GPIO.output(MOTOR_RF, GPIO.LOW)
        motorR.ChangeDutyCycle(-s)
    else:
        GPIO.output(MOTOR_RF, GPIO.HIGH)
        GPIO.output(MOTOR_RB, GPIO.HIGH)
        motorR.ChangeDutyCycle(0)

# Hard stop doesn't seem to work on these motors, but this should work
def LeftHardStop():
    GPIO.output(MOTOR_LF, GPIO.LOW)
    GPIO.output(MOTOR_LB, GPIO.LOW)
    motorL.ChangeDutyCycle(100)

# Set speed for left motor. Argument takes percentage.
def LeftMotor(speed):
    s = int(speed * 20) * 5
    if s > 0:
        if s > 100: s = 100
        GPIO.output(MOTOR_LF, GPIO.HIGH)
        GPIO.output(MOTOR_LB, GPIO.LOW)
        motorL.ChangeDutyCycle(s)
    elif s < 0:
        if s < -100: s = -100
        GPIO.output(MOTOR_LB, GPIO.HIGH)
        GPIO.output(MOTOR_LF, GPIO.LOW)
        motorL.ChangeDutyCycle(-s)
    else:
        GPIO.output(MOTOR_LF, GPIO.HIGH)
        GPIO.output(MOTOR_LB, GPIO.HIGH)
        motorL.ChangeDutyCycle(0)

# Function to handle numerical input
def inputNumber(k):
    num = 0
    if calibrating == CALIBRATE_BRIGHTNESS:
        num = int(str(camera.brightness)+k)
        if num > 100: num = 100
        elif num < 0: num = 0
        camera.brightness = num
    elif calibrating == CALIBRATE_CONTRAST:
        num = int(str(camera.contrast) + k)
        if num > 100: num = 100
        elif num < 0: num = 0
        camera.contrast = num
    elif calibrating == CALIBRATE_ISO:
        num = int(str(camera.iso) + k)
        if num > 100: num = 100
        elif num < 0: num = 0
        camera.iso = num
    elif calibrating == CALIBRATE_SPEED:
        num = int(str(camera.shutter_speed) + k)
        if num > 100000: num = 100000
        elif num < 0: num = 0
        camera.shutter_speed = num
    else:
        print "Calibrating %d with %s" % calibrating, k

# Function to calibrate camera based on keyboard input
def calibrateCamera(k):
    global calibrating
    if k == 'b':
        calibrating = CALIBRATE_BRIGHTNESS
        camera.brightness = 0
        print "Calibrating brightness"
    elif k == 'c':
        calibrating = CALIBRATE_CONTRAST
        camera.contrast = 0
        print "Calibrating contrast"
    elif k == 'i':
        calibrating = CALIBRATE_ISO
        camera.iso = 0
        print "Calibrating ISO"
    elif k == 's':
        calibrating = CALIBRATE_SPEED
        camera.shutter_speed = 0
        print "Calibrating ISO"
    elif k == 'v':
        camera.vflip = not camera.vflip
        print "Calibrating vflip"
    elif k == 'w':
        if camera.awb_mode == 'off':
            camera.awb_mode = 'auto'
        elif camera.awb_mode == 'auto':
            camera.awb_mode = 'sunlight'
        elif camera.awb_mode == 'sunlight':
            camera.awb_mode = 'cloudy'
        elif camera.awb_mode == 'cloudy':
            camera.awb_mode = 'shade'
        elif camera.awb_mode == 'shade':
            camera.awb_mode = 'tungsten'
        elif camera.awb_mode == 'tungsten':
            camera.awb_mode = 'fluorescent'
        elif camera.awb_mode == 'fluorescent':
            camera.awb_mode = 'incandescent'
        elif camera.awb_mode == 'incandescent':
            camera.awb_mode = 'flash'
        elif camera.awb_mode == 'flash':
            camera.awb_mode = 'horizon'
        elif camera.awb_mode == 'horizon':
            camera.awb_mode = 'off'
        else:
            camera.awb_mode = 'off'
        print "Calibrating white balance"
    else:
        print "Calibrating %s" % k

# Quit and ignore argument
def stopTracker(k):
    GPIO.cleanup()
    quit()

# For non-XWIN keep incrementor to name image files
count = 0

# Loop through each frame (forever)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
  # Check elapsed time
    elapsed = cycletime - time.time()
    cycletime = time.time()
#  print "Cycletime: %0.3f Elapsed: %0.3f" % (cycletime, elapsed)

  # Stop motors if deadline reached
    if cycletime > deadline:
        LeftMotor(0)
        RightMotor(0)

  # Pull image data from frame
    image = frame.array
    bearX = 0.0
    bearY = 0.0
    speedL = 0.0
    speedR = 0.0

    mask = cv2.inRange(image, lower, upper)
    output = cv2.bitwise_and(image, image, mask=mask)
    gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
#  gray = cv2.GaussianBlur(gray, (3,3), 0)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
    (locX, locY) = maxLoc
    font = cv2.FONT_HERSHEY_SIMPLEX

  # Calculate new average X,Y bearing
    avgX = avgY = 0
    if len(bear) > 0:
        for f in bear:
            avgX += f[0]
            avgY += f[1]
        avgX /= len(bear)
        avgY /= len(bear)

  # Delete outdated coordinates from queue
    if len(bear) > HISTORY_LEN:
        del bear[0]

  # We only want points that have a maxVal
    if maxVal > 1:
        # Set bearing relative of our "center" point
        bearX = locX-centerX
        bearY = centerY-locY
        if XWIN:
          # Write target info on frame
            cv2.putText(image, "%.0f (%03d,%03d)" % (maxVal, locX, locY),
                        (10, 150), font, 1, (255, 255, 255), 2)
            cv2.circle(image, maxLoc, 21, (255, 0, 0), 2)
        else:
            print "%.0f (%03d,%03d)" % (maxVal, locX, locY)

      # Pulse motor and set deadline
        if bearX < -50:
            LeftMotor(-0.5)
            RightMotor(0.5)
        elif bearX > 50:
            LeftMotor(0.5)
            RightMotor(-0.5)
        elif bearY > 0:
            LeftMotor(1)
            RightMotor(1)
        elif bearY < 0:
            LeftMotor(-1)
            RightMotor(-1)
        angle = asin(bearX/sqrt(bearX*bearX+bearY*bearY))
        deadline = time.time() + (angle/90 * 1)
    else:
        # Substitute the missing point with three quarter of the average of previous points
        bearX = 3 * avgX / 4
        bearY = 3 * avgY / 4

    # Push new bearing onto queue
    bear.append([bearX, bearY])

  # Calculate Pythagorean distance relative max distance to determine power
    if bearY < 0:
        power = sqrt((avgX*avgX+avgY*avgY)/((centerX*centerX)+((dimY-centerY)*(dimY-centerY))))
    else:
        power = sqrt((avgX*avgX+avgY*avgY)/((centerX*centerX)+(centerY*centerY)))

    # Distribute that power based on location on X-axis
    speedL = (0.5 * power) * (1 + (avgX / centerX))
    speedR = (0.5 * power) * (1 - (avgX / centerX))

    if XWIN:
        # Draw cross at "center" point
        pts = np.array([
            [centerX+15-10, centerY-15+10],
            [centerX+15-10, centerY-15+0],
            [centerX+15-20, centerY-15+0],
            [centerX+15-20, centerY-15+10],
            [centerX+15-30, centerY-15+10],
            [centerX+15-30, centerY-15+20],
            [centerX+15-20, centerY-15+20],
            [centerX+15-20, centerY-15+30],
            [centerX+15-10, centerY-15+30],
            [centerX+15-10, centerY-15+20],
            [centerX+15-0, centerY-15+20],
            [centerX+15-0, centerY-15+10],
            [centerX+15-10, centerY-15+10]], np.int32)
        cv2.polylines(image, [pts], True, (0, 255, 255))

    # Prepare info for display
    cardinal = ("%s%s" %
                ((" " if bearY == 0 else ("%s" % ("S" if bearY < -5 else "N"))),
                 ("E" if bearX > 10 else ("%s" % ("W" if bearX < -10 else " ")))))
    msg = "[%03d,%03d] %s sL[%.2f] sR[%.2f] angle[%.2f]" % (bearX,
                                                            bearY,
                                                            cardinal,
                                                            speedL,
                                                            speedR,
                                                            angle)

    if XWIN:
        cv2.putText(image, msg, (12, 52), font, 0.7, (0, 0, 0), 2)
        cv2.putText(image, msg, (10, 50), font, 0.7, (255, 255, 255), 2)
    else:
        print msg

    # Prepare info for display
    msg = "B[%d] C[%d] I[%d] E[%s] S[%d] W[%s]" % (camera.brightness,
                                                   camera.contrast,
                                                   camera.iso,
                                                   camera.exposure_mode,
                                                   camera.exposure_speed,
                                                   camera.awb_mode)

    if XWIN:
        # Write camera calibration info on frame
        cv2.putText(image, msg, (12, 77), font, 0.7, (0, 0, 0), 2)
        cv2.putText(image, msg, (10, 75), font, 0.7, (255, 255, 255), 2)
    else:
        print msg

    # Update motor speed
#  LeftMotor(speedL)
#  RightMotor(speedR)

    if XWIN:
        # Display this frame
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF

        # Python-style switch statment to handle keyboard input
        switcher = {
            ord("0"): inputNumber,
            ord("1"): inputNumber,
            ord("2"): inputNumber,
            ord("3"): inputNumber,
            ord("4"): inputNumber,
            ord("5"): inputNumber,
            ord("6"): inputNumber,
            ord("7"): inputNumber,
            ord("8"): inputNumber,
            ord("9"): inputNumber,
            ord("b"): calibrateCamera,
            ord("c"): calibrateCamera,
            ord("i"): calibrateCamera,
            ord("q"): stopTracker,
            ord("s"): calibrateCamera,
            ord("v"): calibrateCamera,
            ord("w"): calibrateCamera,
            ord("x"): calibrateCamera,
        }
        func = switcher.get(key, lambda x: "nothing")
        func(chr(key))
    else:
        count += 1
        filename = "img/%04d.jpg" % count
        cv2.imwrite(filename, image)

    rawCapture.truncate(0)
# Nothing further



