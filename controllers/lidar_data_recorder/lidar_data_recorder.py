#from vehicle import Driver
from controller import Robot
from controller import Lidar

#driver = Driver()
robot = Robot()
#lidarIbeo1 = Lidar("ibeo 1")
lidars = {}
TIME_STEP = int(robot.getBasicTimeStep())


# while driver.step() != -1:
#     pass
lidarNames = [
    "ibeo 1",  # front right
    "ibeo 2",  # front left
    "ibeo 3",  # rear left
    "ibeo 4"]
# for name in lidarNames:
#     lidars[name] = robot.getDevice(name)
#     lidars[name].enable(TIME_STEP)

#print("Ibeo1: ", lidarIbeo1.getPointCloud(data_type='list'))

#print("Lidar Names", lidarNames)

dist1 = robot.getDevice("distance sensor front")
dist1.enable(TIME_STEP)
robot.init()

while robot.step(TIME_STEP) != -1:
    print("Timestep:", TIME_STEP)
    #print("Lidar", robot.getLidar(""))
    print("Lidar Names", lidarNames)
    #print("Ibeo1: ", lidars["ibeo 1"].getRangeImage())
    print("Dist Sensor: ", dist1.getValue())
