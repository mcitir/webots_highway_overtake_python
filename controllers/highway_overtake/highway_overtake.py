# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""vehicle_driver controller."""

from vehicle import Driver

# Extended Kalman Filter
import math

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot


sensorsNames = [
    "front",
    "front right 0",
    "front right 1",
    "front right 2",
    "front left 0",
    "front left 1",
    "front left 2",
    "rear",
    "rear left",
    "rear right",
    "right",
    "left"]
sensors = {}

lidarNames = [
    "ibeo 1",  # front right
    "ibeo 2",  # front left
    "ibeo 3",  # rear left
    "ibeo 4"]

lidars = {}

lanePositions = [-10.6, -6.875, -3.2]  # Prerecorded Lane Positions
currentLane = 1
overtakingSide = None
maxSpeed = 80
safeOvertake = False


def apply_PID(position, targetPosition):
    """Apply the PID controller and return the angle command."""
    P = 0.05
    I = 0.000015
    D = 25
    diff = position - targetPosition
    if apply_PID.previousDiff is None:
        apply_PID.previousDiff = diff
    # anti-windup mechanism
    if diff > 0 and apply_PID.previousDiff < 0:
        apply_PID.integral = 0
    if diff < 0 and apply_PID.previousDiff > 0:
        apply_PID.integral = 0
    apply_PID.integral += diff
    # compute angle
    angle = P * diff + I * apply_PID.integral + \
        D * (diff - apply_PID.previousDiff)
    apply_PID.previousDiff = diff
    return angle


apply_PID.integral = 0
apply_PID.previousDiff = None


def get_filtered_speed(speed):
    """Filter the speed ommand to avoid abrupt speed changes."""
    get_filtered_speed.previousSpeeds.append(speed)
    if len(get_filtered_speed.previousSpeeds) > 100:  # keep only 80 values
        get_filtered_speed.previousSpeeds.pop(0)
    return sum(get_filtered_speed.previousSpeeds) / float(len(get_filtered_speed.previousSpeeds))


def is_vehicle_on_side(side):
    """Check (using the 3 appropriated front distance sensors) if there is a car in front."""
    for i in range(3):
        name = "front " + side + " " + str(i)
        if sensors[name].getValue() > 0.8 * sensors[name].getMaxValue():
            return True
    return False


def reduce_speed_if_vehicle_on_side(speed, side):
    """Reduce the speed if there is some vehicle on the side given in argument."""
    minRatio = 1
    for i in range(3):
        name = "front " + overtakingSide + " " + str(i)
        ratio = sensors[name].getValue() / sensors[name].getMaxValue()
        if ratio < minRatio:
            minRatio = ratio
    return minRatio * speed


get_filtered_speed.previousSpeeds = []
driver = Driver()
for name in sensorsNames:
    sensors[name] = driver.getDevice("distance sensor " + name)
    sensors[name].enable(10)

gps = driver.getDevice("gps")
gps.enable(10)

camera = driver.getDevice("camera")
# uncomment those lines to enable the camera
camera.enable(10)
camera.recognitionEnable(50)

for name in lidarNames:
    for name in lidarNames:
        lidars[name] = driver.getDevice(name)
        lidars[name].enable(10)
        lidars[name].enablePointCloud()

# Extended Kalman Filter
time = 0.0

# State Vector [x y yaw v]'
xEst = np.zeros((4, 1))
xTrue = np.zeros((4, 1))
PEst = np.eye(4)

xDR = np.zeros((4, 1))  # Dead reckoning

# history
hxEst = xEst
hxTrue = xTrue
hxDR = xTrue
hz = np.zeros((2, 1))
cloudData = {}

while driver.step() != -1:

    # adjust speed according to front vehicle
    #print("Ibeo 1: ", lidars["ibeo 1"].getRangeImage())

    rangeImage = lidars["ibeo 2"].getRangeImage()
    lidarPoints = lidars["ibeo 2"].getPointCloud()
    # print("x: " + str(lidarPoints[0].x) + " y: " + str(lidarPoints[0].y) +
    #       " z: " + str(lidarPoints[0].z) + " layer: " + str(lidarPoints[0].layer_id))
    cloudData = np.hstack((cloudData, lidarPoints))
    print("Data: ", cloudData[1])
    # cloudData.to_file("output.ply")

    for i in range(10):
        print(str(rangeImage[i]) + " ", end='')

    print('')

    print("Steering angle: ", driver.getSteeringAngle())
    frontDistance = sensors["front"].getValue()
    print("Front Distance: ", frontDistance)
    frontRange = sensors["front"].getMaxValue()
    print("Front Range: ", frontRange)
    speed = maxSpeed * frontDistance / frontRange
    if sensors["front right 0"].getValue() < 8.0 or sensors["front left 0"].getValue() < 8.0:
        # another vehicle is currently changing lane in front of the vehicle => emergency braking
        speed = min(0.5 * maxSpeed, speed)
    if overtakingSide is not None:
        # check if overtaking should be aborted
        if overtakingSide == 'right' and sensors["left"].getValue() < 0.8 * sensors["left"].getMaxValue():
            overtakingSide = None
            currentLane -= 1
        elif overtakingSide == 'left' and sensors["right"].getValue() < 0.8 * sensors["right"].getMaxValue():
            overtakingSide = None
            currentLane += 1
        else:  # reduce the speed if the vehicle from previous lane is still in front
            speed2 = reduce_speed_if_vehicle_on_side(speed, overtakingSide)
            if speed2 < speed:
                speed = speed2
    speed = get_filtered_speed(speed)
    print("Speed: ", speed)
    driver.setCruisingSpeed(speed)
    # brake if needed
    speedDiff = driver.getCurrentSpeed() - speed
    if speedDiff > 0:
        driver.setBrakeIntensity(min(speedDiff / speed, 1))
    else:
        driver.setBrakeIntensity(0)
    # car in front, try to overtake
    print("Over Taking Side: ", overtakingSide)  # MC
    print("Current Lane: ", currentLane)  # MC
    if frontDistance < 0.8 * frontRange and overtakingSide is None:
        if (is_vehicle_on_side("left") and
                (not safeOvertake or sensors["rear left"].getValue() > 0.8 * sensors["rear left"].getMaxValue()) and
                sensors["left"].getValue() > 0.8 * sensors["left"].getMaxValue() and
                currentLane < 2):
            currentLane += 1
            overtakingSide = 'right'
        elif (is_vehicle_on_side("right") and
                (not safeOvertake or sensors["rear right"].getValue() > 0.8 * sensors["rear right"].getMaxValue()) and
                sensors["right"].getValue() > 0.8 * sensors["right"].getMaxValue() and
                currentLane > 0):
            currentLane -= 1
            overtakingSide = 'left'

    # adjust steering to stay in the middle of the current lane
    position = gps.getValues()[0]
    print("Position: ", position)
    print("PID", apply_PID(position, lanePositions[currentLane]))
    angle = max(
        min(apply_PID(position, lanePositions[currentLane]), 0.5), -0.5)
    driver.setSteeringAngle(angle)

    # check if overtaking is over
    # the car is just in the lane
    if abs(position - lanePositions[currentLane]) < 1.5:
        overtakingSide = None
