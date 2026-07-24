from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Axis, Button, Color, Direction, Port, Stop
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice

# ---------------- Hub ----------------
hub = PrimeHub()
hub.system.set_stop_button(Button.BLUETOOTH)
hub.speaker.volume(50)

clock = StopWatch()

Drive  = Motor(Port.B, Direction.COUNTERCLOCKWISE)
Steer  = Motor(Port.C, Direction.COUNTERCLOCKWISE)
Camera = PUPDevice(Port.A)
ColorSens = PUPDevice(Port.E)
UltraL = PUPDevice(Port.F)
UltraR = PUPDevice(Port.D)

CAM_MODE  = 0
IMG_W     = 320
IMG_H     = 240
CENTER_X  = 160

RED, GREEN, NONE = 0, 1, -1

# ---------------- Behaviour targets ----------------
APPROACH_Y     = 130   # y at which the side target reaches its FULL value
TARGET_X_GREEN = 310   # green target when the block is close (y >= APPROACH_Y)
TARGET_X_RED   = 10    # red   target when the block is close
WALL_THRES = 500
BLACK_IDX   = 3
BLACK_THRES = 60
BLACK_TURN  = -1000

# ---- Approaching x-target ramp ----
# As a block approaches (y grows), shift the x-target from a milder "far" value
# up to the full side target by APPROACH_Y, so the car eases over early.
Y_RAMP_START = 35      # y where ramping begins (block just clears the top mask)
GREEN_FAR    = 200     # green target when far  -> ramps to TARGET_X_GREEN by APPROACH_Y
RED_FAR      = 120     # red   target when far  -> ramps to TARGET_X_RED   by APPROACH_Y

# ---------------- Steering tuning ----------------
STEER_KP     = 1.5
STEER_SIGN   = -1
STEER_LOAD_CAP = 70
OPEN_KP = 0.018
OPEN_KD = 0
OBSTACLE_KP = 0.05
GYRO_FLIP = 15   # flip the ultrasonic centring once the body is rotated more than this (deg) off the corridor

# ---------------- Drive tuning ----------------
DRIVE_FWD = 70

# ---------------- Utils ----------------
def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

# =====================================================================
#  LAGRANGE-INTERPOLATED THETA (yaw displacement from angular velocity)
# =====================================================================
# Instead of trusting hub.imu.heading() directly (its internal fusion can
# absorb 3 laps worth of vibration noise, wrap-arounds and micro-jumps),
# we rebuild the yaw angle ourselves:
#
#     theta(T) = theta(0) + integral( omega(t) dt )  from 0 to T
#
# omega(t) = hub.imu.angular_velocity(Axis.Z) is sampled every control
# loop (~10 ms, but NOT at perfectly regular intervals - Bluetooth,
# camera reads and garbage collection all jitter the loop period).
# A plain rectangle sum  theta += omega * dt  is only 1st-order accurate
# and the jitter makes it worse.  So we fit a LAGRANGE POLYNOMIAL of
# degree 3 through the last 4 samples (t_i, w_i) and integrate it
# EXACTLY over the newest interval.
#
# We build the polynomial in NEWTON DIVIDED-DIFFERENCE form:
#
#   P(t) = d0 + d1(t-t0) + d2(t-t0)(t-t1) + d3(t-t0)(t-t1)(t-t2)
#
# which is the same unique cubic as the classical Lagrange basis form,
# but is far cheaper to build and - crucially - can be expanded into
# ordinary coefficients c0..c3 and integrated with a plain antiderivative:
#
#   INT P dt = c0*t + c1*t^2/2 + c2*t^3/3 + c3*t^4/4
#
# evaluated at the two ends of the newest interval.  No quadrature rule
# and no magic constants are needed: the integral of a cubic is just
# another polynomial, so this is exact arithmetic, not an approximation.
#
# Uneven time steps cost nothing (the t_i appear explicitly), single
# noisy samples are averaged against their neighbours by the fit, and
# the result is 4th-order accurate instead of 1st.
# =====================================================================

THETA_N        = 4       # samples in the sliding window -> cubic Lagrange
OMEGA_DEADBAND = 0.5     # deg/s, kills stand-still gyro hiss so theta
                         # cannot creep while the robot is parked

theta_clock = StopWatch()    # DEDICATED clock: GyroDeg() etc. reset the
                             # shared 'clock', which would corrupt t_i
_theta_ts   = []             # ring buffer of sample times   t_i  [s]
_theta_ws   = []             # ring buffer of sample omegas  w_i  [deg/s]
_theta      = 0.0            # accumulated yaw displacement [deg]
_omega_bias = 0.0            # gyro bias measured at start-up [deg/s]

def _integrate_cubic(ts, ws):
    """Exact integral of the cubic through the 4 points (ts, ws), taken
    over the NEWEST interval [ts[2], ts[3]].

    Step 1: Newton divided differences d0..d3.
    Step 2: expand to ordinary coefficients c0..c3.
    Step 3: evaluate the antiderivative at both ends.
    All three steps are plain arithmetic - no quadrature nodes."""
    t0, t1, t2, t3 = ts
    w0, w1, w2, w3 = ws

    # --- divided differences (the Newton coefficients) ---
    d0 = w0
    d1 = (w1 - w0) / (t1 - t0)
    d12 = (w2 - w1) / (t2 - t1)
    d2 = (d12 - d1) / (t2 - t0)
    d23 = (w3 - w2) / (t3 - t2)
    d3 = ((d23 - d12) / (t3 - t1) - d2) / (t3 - t0)

    # --- expand P(t) into c0 + c1 t + c2 t^2 + c3 t^3 ---
    c3 = d3
    c2 = d2 - d3 * (t0 + t1 + t2)
    c1 = d1 - d2 * (t0 + t1) + d3 * (t0*t1 + t0*t2 + t1*t2)
    c0 = d0 - d1 * t0 + d2 * t0*t1 - d3 * t0*t1*t2

    # --- antiderivative, Horner form: F(x) = ((((c3/4)x + c2/3)x + c1/2)x + c0)x
    def F(x):
        return ((((c3 * 0.25 * x + c2 / 3.0) * x + c1 * 0.5) * x) + c0) * x

    return F(t3) - F(t2)

def ThetaCalibrate(ms=800):
    """Measure the gyro's zero-rate bias.  Call ONCE at program start
    while the robot is perfectly still (before Escape()/Run())."""
    global _omega_bias
    n = 0
    acc = 0.0
    theta_clock.reset()
    while theta_clock.time() < ms:
        acc += hub.imu.angular_velocity(Axis.Z)
        n += 1
        wait(5)
    _omega_bias = acc / n if n else 0.0
    print("gyro bias:", _omega_bias, "deg/s over", n, "samples")

def ThetaReset(value=0.0):
    """Restart the integrator (analogue of hub.imu.reset_heading)."""
    global _theta, _theta_ts, _theta_ws
    _theta = value
    _theta_ts = []
    _theta_ws = []
    theta_clock.reset()

def ThetaUpdate():
    """Sample omega, slide the window, integrate the newest interval.
    Call this once per control-loop iteration.  Returns the current
    theta so it can be used inline."""
    global _theta
    t = theta_clock.time() / 1000.0                  # ms -> s
    w = hub.imu.angular_velocity(Axis.Z) - _omega_bias
    if -OMEGA_DEADBAND < w < OMEGA_DEADBAND:
        w = 0.0

    # strictly increasing time stamps only (equal stamps would divide by
    # zero in the divided differences)
    if _theta_ts and t <= _theta_ts[-1]:
        return _theta

    _theta_ts.append(t)
    _theta_ws.append(w)
    if len(_theta_ts) > THETA_N:
        _theta_ts.pop(0)
        _theta_ws.pop(0)

    n = len(_theta_ts)
    if n == 4:
        # full window: exact integral of the cubic over the newest interval
        _theta += _integrate_cubic(_theta_ts, _theta_ws)
    elif n >= 2:
        # first few loops after a reset: trapezoid until the window fills
        _theta += 0.5 * (_theta_ws[-1] + _theta_ws[-2]) \
                      * (_theta_ts[-1] - _theta_ts[-2])
    return _theta

def Theta():
    """Current Lagrange-integrated yaw displacement in degrees.
    Unwrapped: after 3 clockwise laps it reads ~ -1080, not 0."""
    return _theta

# ---- heading helpers, now served by the Lagrange theta ----
USE_LAGRANGE_THETA = True    # set False to fall back on the raw IMU heading

def Heading():
    """Single point of truth for 'which way are we pointing'."""
    return _theta if USE_LAGRANGE_THETA else hub.imu.heading()

def Compass():
    return ((Heading() % 360) + 360) % 360

def AD_angle():
    c = Compass()
    return c if c <= 180 else c - 360

def RD_angle(angle):
    c = (((Heading() - angle) % 360) + 360) % 360
    return c if c <= 180 else c - 360

def ReadCam():
    # Camera mode 0. colour 0=red 1=green -1=none.
    # 'black' = size of the black region (wall proximity), returned ALWAYS.
    try:
        data = Camera.read(CAM_MODE)
        x, y, colour = data[0], data[1], data[2]
        black = data[BLACK_IDX]
        if colour != RED and colour != GREEN:
            colour = NONE
        return x, y, colour, black
    except Exception:
        return 0, 0, NONE, 0

def TargetX(colour, y):
    """Where the block should sit horizontally, ramped by how close it is.
    Far (y<=Y_RAMP_START) -> FAR value; near (y>=APPROACH_Y) -> full side target;
    linear in between."""
    if colour == GREEN:
        far, near = GREEN_FAR, TARGET_X_GREEN
    else:
        far, near = RED_FAR, TARGET_X_RED
    if y <= Y_RAMP_START:
        return far
    if y >= APPROACH_Y:
        return near
    frac = (y - Y_RAMP_START) / (APPROACH_Y - Y_RAMP_START)
    return far + (near - far) * frac

def SteerPower(power):
    power = clamp(power, -STEER_LOAD_CAP, STEER_LOAD_CAP)
    Steer.dc(power)
    return power

def SetSpeed(d):
    global DRIVE_FWD
    DRIVE_FWD = d

def Forward(duty=None):
    # Reads DRIVE_FWD live, so SetSpeed()/fe.DRIVE_FWD changes take effect.
    Drive.dc(DRIVE_FWD if duty is None else duty)

def StopAll():
    Drive.hold()
    Steer.hold()
    wait(100)

def SteerTo(angle, Kp=1, Limit=70, AngleLimit = 70):
    angle = clamp(angle, -AngleLimit, AngleLimit)
    Error = Kp * (angle - Steer.angle())
    Error = clamp(Error, -Limit, Limit)
    Steer.dc(Error)

def GetColor():  # blue = -1, orange = 1, white = 0
    Color_val = ColorSens.read(5)
    if Color_val[2] > 1.5 * Color_val[0] and (Color_val [2] < 700 and Color_val[1] < 500):
        ct = -1
    elif Color_val[0] > Color_val[1] * 1.3 and Color_val[2] < 800:
        ct = 1
    else:
        ct = 0
    return ct

# ---- Wall following (open round) ----
lastError = 0
legHeading = 0   # heading of the corridor currently being followed (set from FeObstacle as i*sign*90)

def SteerOpen(sign, wall_thres=WALL_THRES, Kp = OPEN_KP, wall = WALL_THRES):
    global lastError

    ThetaUpdate()                     # keep the Lagrange integrator fed
    x, y, colour, black = ReadCam()

    UltraL_Val = UltraL.read(0)
    UltraR_Val = UltraR.read(0)
    LeftDist  = UltraL_Val[0]
    RightDist = UltraR_Val[0]
    if LeftDist  == -1: LeftDist  = 2000
    if RightDist == -1: RightDist = 2000

    if sign == 0:
        error = LeftDist - RightDist
    elif sign == 1:
        error = wall - RightDist
    else:
        error = -(wall - LeftDist)

    if black > BLACK_THRES or LeftDist < 200 and RightDist < 200:
        error = sign * -1000

    output = error * Kp + (error - lastError) * OPEN_KD
    lastError = error

    SteerTo(-output, Limit=70)
    Forward()
    wait(10)

# ---- Wall following (obstacle round) ----
def SteerClear(sign, wall_thres=700, Kp=0.1, wall=WALL_THRES):
    global lastError, legHeading

    ThetaUpdate()                     # dev below now uses Lagrange theta
    x, y, colour, black = ReadCam()

    UltraL_Val = UltraL.read(0)
    UltraR_Val = UltraR.read(0)
    LeftDist  = UltraL_Val[0]
    RightDist = UltraR_Val[0]
    if LeftDist  == -1: LeftDist  = 1000     # no echo -> capped (NOT 100000)
    if RightDist == -1: RightDist = 1000
    LeftDist  = min(LeftDist, 1000)          # an open side must NOT blow the error up
    RightDist = min(RightDist, 1000)

    # ---- gyro filter ----
    # legHeading is set by FeObstacle from the corner count (i*sign*90), NOT from dev.
    # So a hard block-dodge that briefly swings dev past 90 inside the same corridor is
    # not mistaken for a corner.
    dev = RD_angle(legHeading)

    error = (RightDist - LeftDist) #- sign*300             # centre between walls

    closeFlag = True if black > 70 or abs(dev)>GYRO_FLIP else False
    if closeFlag:
        error = sign * 1000                  # corner -> hard turn in the lap direction                       # emergency: too close on the LEFT  -> steer right
    if abs(dev) > GYRO_FLIP:               # body reversed >90 deg vs corridor -> L/R swapped
        error = abs(dev)/dev * -1000                       # flip the ultrasonic centring
    if LeftDist < 150:
        error = 1000
        closeFlag = True
    if RightDist < 150:
        error = -1000
        closeFlag = True
    ALimit = 50 if closeFlag == True or LeftDist < 150 or RightDist < 150 else 15
    output = error * Kp

    SteerTo(output, Limit=100, AngleLimit=ALimit)
    Forward()
    wait(10)

# ---- Obstacle round ----
def SteerObstacle(sign):
    ThetaUpdate()
    x, y, colour, black = ReadCam()

    UltraL_Val = UltraL.read(0)
    UltraR_Val = UltraR.read(0)
    LeftDist  = UltraL_Val[0]
    RightDist = UltraR_Val[0]
    if LeftDist  == -1: LeftDist  = 2000
    if RightDist == -1: RightDist = 2000

    DistFlag = True if LeftDist < 200 or RightDist < 200 else False

    if colour == NONE or black > BLACK_THRES or DistFlag == True:
        if black>BLACK_THRES or DistFlag == True:
            SetSpeed(80)
        else:
            SetSpeed(80)
        SteerClear(sign, Kp=0.14, wall=1000)
        hub.light.on(Color.BLUE)
    else:
        hub.light.on(Color.RED if colour == RED else Color.GREEN)
        target_x = TargetX(colour, y)          # ramps in as the block approaches
        SetSpeed(80 if y < APPROACH_Y else 80)
        ALimit = 20 if y < APPROACH_Y - 20 else 40
        error = target_x - x
        target = STEER_SIGN * STEER_KP * error
        # if LeftDist < 170:
        #     target = -1000      # too close on the LEFT  -> steer away (right). Do NOT route through STEER_SIGN.
        # elif RightDist < 170:
        #     target = 1000     # too close on the RIGHT -> steer away (left)
        SteerTo(target, Kp=1.2, Limit=100, AngleLimit=ALimit)

    Forward()
    wait(10)
def Stop():
    Drive.brake()
    wait(50)
    Drive.hold()
    wait(100)

def Escape():
    x, y, colour, black = ReadCam()

    UltraL_Val = UltraL.read(0)
    UltraR_Val = UltraR.read(0)
    LeftDist  = UltraL_Val[0]
    RightDist = UltraR_Val[0]
    if LeftDist  == -1: LeftDist  = 2000
    if RightDist == -1: RightDist = 2000

    if LeftDist > RightDist:
        Steer.dc(-100)
        wait (300)
        while Heading() > -50:
            ThetaUpdate()
            Forward(duty=50)
        Steer.dc(70)
        sign = -1

    else:
        Steer.dc(100)
        wait (300)
        while Heading() < 40:
            ThetaUpdate()
            Forward(duty=50)
        Steer.dc(-70)
        sign = 1
    return sign

def GyroDeg(deg, target, Kp = 1, TimeOut = 3000, MinSpeed = 50, MaxSpeed = 100):
    x, y, colour, black = ReadCam()
    Drive.reset_angle()
    clock.reset()
    while abs(Drive.angle())<abs(deg) and clock.time() < TimeOut:
        ThetaUpdate()
        duty = (deg - Drive.angle())*0.5
        duty = clamp(duty, MinSpeed, MaxSpeed) if duty > 0 else clamp(duty, -MaxSpeed, -MinSpeed)
        Drive.dc(duty)
        Error = (target-Heading())*Kp*duty/abs(duty)
        SteerTo(Error)

def GyroColor(color, target, Kp = 5, TimeOut = 5000, Speed = 50):
    x, y, colour, black = ReadCam()
    Drive.reset_angle()
    clock.reset()
    while GetColor() != color and clock.time() < TimeOut:
        ThetaUpdate()
        Forward(duty=Speed)
        Error = (target-Heading())*Kp*Speed/abs(Speed)
        SteerTo(Error)

def GyroAcc(lo, hi, deg, target, Kp = 5):
    x, y, colour, black = ReadCam()
    Drive.reset_angle()

    while abs(Drive.angle())<deg:
        ThetaUpdate()
        duty = lo + (hi - lo) * abs(Drive.angle())/deg
        Forward(duty=duty)
        Error = (target-Heading())*Kp*duty/abs(duty)
        SteerTo(Error)

def GyroToWall(sign, duty, target, Kp = 13):
    x, y, colour, black = ReadCam()
    Drive.reset_angle()
    UltraL_Val = UltraL.read(0)
    UltraR_Val = UltraR.read(0)
    LeftDist  = UltraL_Val[0]
    RightDist = UltraR_Val[0]
    if LeftDist  == -1: LeftDist  = 2000
    if RightDist == -1: RightDist = 2000
    if sign ==  -1:
        while RightDist > 120:
            ThetaUpdate()
            UltraL_Val = UltraL.read(0)
            UltraR_Val = UltraR.read(0)
            LeftDist  = UltraL_Val[0]
            RightDist = UltraR_Val[0]
            if LeftDist  == -1: LeftDist  = 2000
            if RightDist == -1: RightDist = 2000
            Forward(duty=duty)
            Error = (target-Heading())*Kp*duty/abs(duty)
            SteerTo(Error)
    else:
        while LeftDist > 120:
            ThetaUpdate()
            UltraL_Val = UltraL.read(0)
            UltraR_Val = UltraR.read(0)
            LeftDist  = UltraL_Val[0]
            RightDist = UltraR_Val[0]
            if LeftDist  == -1: LeftDist  = 2000
            if RightDist == -1: RightDist = 2000
            Forward(duty=duty)
            Error = (target-Heading())*Kp*duty/abs(duty)
            SteerTo(Error)

def DriveDeg(deg, Steer, MaxSpeed = 70, MinSpeed = 50, TimeOut=5000):
    x, y, colour, black = ReadCam()
    Drive.reset_angle()
    clock.reset()
    while abs(Drive.angle())<abs(deg) and clock.time() < TimeOut:
        ThetaUpdate()
        duty = (deg - Drive.angle())*0.5
        duty = clamp(duty, MinSpeed, MaxSpeed) if duty > 0 else clamp(duty, -MinSpeed, -MaxSpeed)
        Forward(duty=duty)
        SteerTo(Steer)
