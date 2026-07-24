from pybricks.tools import wait
from pybricks.parameters import Color
import FeFunctions as fe

timer = fe.StopWatch()

def CalibrateSteer():
    fe.Steer.dc(-70)
    wait(300)
    MinSteer = fe.Steer.angle()
    fe.Steer.dc(70)
    wait(500)
    MaxSteer = fe.Steer.angle()
    Range = MaxSteer - MinSteer
    fe.Steer.reset_angle(Range / 2)
    print("range:", Range, " angle:", fe.Steer.angle())

def Run(sign):
    i = 0
    fe.legHeading = i * sign * 90              # corridor heading for the gyro filter
    timer.reset()

    # 3) count 11 more corners (3 laps)
    while i < 12:
        while fe.GetColor() == sign:
            fe.SteerObstacle(sign)
        timer.reset()
        while timer.time() < 500:
            fe.SteerObstacle(sign)
        while fe.GetColor() != sign:
            fe.SteerObstacle(sign)
        i += 1
        fe.legHeading = i * sign * 90          # one corner turned -> advance the corridor heading
        print(i)
        print(fe.legHeading)

    # 4) roll past the last line, then stop
    fe.Drive.reset_angle()
    fe.GyroDeg(600, sign*1080, Kp = 5)
    fe.StopAll()



# Start the robot Parking Anticlockwise direction
fe.ThetaCalibrate()          # measure gyro bias while the car is still
fe.ThetaReset(0)             # theta = 0 at the start line
fe.legHeading = 0
CalibrateSteer()
sign = fe.Escape()
Run(sign)
if sign == -1:
    fe.GyroAcc(-30, -60, 700, -1168, Kp=2)
    fe.GyroDeg(-100000, -1168, MaxSpeed=60, TimeOut=4000)
    fe.Drive.dc(-100)
    fe.Steer.dc(-70)
    wait(200)
    fe.Steer.dc(70)
    wait(400)
    fe.Stop()
    fe.clock.reset()
    while fe.clock.time()< 300:
        fe.ThetaUpdate()
        fe.SteerTo(0, Kp = 10, Limit=30)
    fe.Drive.dc(-60)
    wait(300)
    fe.Stop()
    wait(100)
    fe.ThetaReset(-90)       # re-datum theta to the bay axis
    fe.Drive.dc(40)
    while fe.Heading() < 7:
        fe.ThetaUpdate()
        fe.SteerTo(100)
    fe.Stop()
    fe.clock.reset()
    while fe.clock.time()< 300:
        fe.ThetaUpdate()
        fe.SteerTo(0, Kp = 10)
    fe.GyroToWall(-1, 40, 0)
    fe.GyroDeg(220, 0, MinSpeed= 30, MaxSpeed=40, Kp=10)
    fe.GyroToWall(-1,30,0)
    fe.GyroDeg(280, 0, MaxSpeed= 30, Kp=10)
    fe.Stop()
    fe.Steer.dc(100)
    wait(200)
    fe.Stop()
    fe.DriveDeg(-450, 100, MaxSpeed=50)
    fe.Stop()
    fe.Steer.dc(-100)
    wait(200)
    fe.DriveDeg(-480, -100, MaxSpeed=50)
    fe.Stop()
    fe.Steer.dc(100)
    wait(200)
    fe.Drive.dc(100)
    fe.GyroDeg(120, 0, MaxSpeed=30)
    fe.Stop()

else:
    fe.GyroAcc(-30, -60, 600, 1100, Kp=2)
    fe.GyroDeg(-100000, 1168, MaxSpeed=60, TimeOut=4000)
    fe.Drive.dc(-100)
    fe.Steer.dc(70)
    wait(200)
    fe.Steer.dc(-70)
    wait(400)
    fe.Stop()
    fe.clock.reset()
    while fe.clock.time()< 300:
        fe.ThetaUpdate()
        fe.SteerTo(0, Kp = 10, Limit=30)
    fe.Drive.dc(-60)
    wait(300)
    fe.Stop()
    wait(100)
    fe.ThetaReset(90)        # re-datum theta to the bay axis
    fe.Drive.dc(40)
    while fe.Heading() > -7:
        fe.ThetaUpdate()
        fe.SteerTo(-100)
    fe.Stop()
    fe.clock.reset()
    while fe.clock.time()< 300:
        fe.ThetaUpdate()
        fe.SteerTo(0, Kp = 10)
    fe.GyroToWall(1, 40, 0)
    fe.GyroDeg(220, 0, MinSpeed= 30, MaxSpeed=40, Kp=10)
    fe.GyroToWall(1,30,0)
    fe.GyroDeg(270, 0, MaxSpeed= 30, Kp=10)
    fe.Stop()
    fe.Steer.dc(-100)
    wait(200)
    fe.Stop()
    fe.DriveDeg(-450, -100, MaxSpeed=50)
    fe.Stop()
    fe.Steer.dc(100)
    wait(200)
    fe.DriveDeg(-480, 100, MaxSpeed=40)
    fe.Stop()
    fe.Steer.dc(-100)
    wait(200)
    fe.Drive.dc(100)
    fe.GyroDeg(120, 0, MaxSpeed=30)
    fe.Stop()
