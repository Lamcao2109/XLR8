from pybricks.tools import wait
from pybricks.parameters import Color
import FeFunctions as fe

sign = 0 
timer = fe.StopWatch()

def CalibrateSteer():
    fe.Steer.dc(-70)
    wait (300)
    MinSteer = fe.Steer.angle()
    fe.Steer.dc(70)
    wait(500)
    MaxSteer = fe.Steer.angle()
    Range = MaxSteer - MinSteer
    fe.Steer.reset_angle(Range/2)
    print ("range: ", Range)
    print ("current angle : ", fe.Steer.angle())
    
def Run():
    while fe.GetColor() == 0 :
        fe.SteerOpen(0)
    print(fe.GetColor())
    sign = fe.GetColor()
    i = 0
    timer.reset()
    while timer.time() < 300:
        fe.SteerOpen(sign)
    while i < 11:
        while fe.GetColor() == sign: 
            fe.SteerOpen(sign)
        timer.reset()
        while timer.time() < 300:
            fe.SteerOpen(sign)
        while fe.GetColor() != sign:
            fe.SteerOpen(sign)
        i+=1
        print(i)

    fe.Drive.reset_angle()
    while fe.Drive.angle() < 1000:
        fe.SteerOpen(sign)

# Start the robot
CalibrateSteer()
Run()
