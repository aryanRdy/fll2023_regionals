from asyncio import run, sleep_ms
from hub import port
from hub import motion_sensor
import motor
import runloop
import motor_pair
import color_sensor
import color
import math
import time

# Ease in ease out


class EasingBase:
    limit = (0, 1)

    def __init__(self, start: float = 0, end: float = 1, duration: float = 1):
        self.start = start
        self.end = end
        self.duration = duration

    def func(self, t: float) -> float:
        raise NotImplementedError

    def ease(self, alpha: float) -> float:
        t = self.limit[0] * (1 - alpha) + self.limit[1] * alpha
        t /= self.duration
        a = self.func(t)
        return self.end * a + self.start * (1 - a)

    def __call__(self, alpha: float) -> float:
        return self.ease(alpha)


"""
Linear
"""


class LinearInOut(EasingBase):
    def func(self, t: float) -> float:
        return t


"""
Quadratic easing functions
"""


class QuadEaseInOut(EasingBase):
    def func(self, t: float) -> float:
        if t < 0.5:
            return 2 * t * t
        return (-2 * t * t) + (4 * t) - 1


class QuadEaseIn(EasingBase):
    def func(self, t: float) -> float:
        return t * t


class QuadEaseOut(EasingBase):
    def func(self, t: float) -> float:
        return -(t * (t - 2))


"""
Cubic easing functions
"""


class CubicEaseIn(EasingBase):
    def func(self, t: float) -> float:
        return t * t * t


class CubicEaseOut(EasingBase):
    def func(self, t: float) -> float:
        return (t - 1) * (t - 1) * (t - 1) + 1


class CubicEaseInOut(EasingBase):
    def func(self, t: float) -> float:
        if t < 0.5:
            return 4 * t * t * t
        p = 2 * t - 2
        return 0.5 * p * p * p + 1


"""
Quartic easing functions
"""


class QuarticEaseIn(EasingBase):
    def func(self, t: float) -> float:
        return t * t * t * t


class QuarticEaseOut(EasingBase):
    def func(self, t: float) -> float:
        return (t - 1) * (t - 1) * (t - 1) * (1 - t) + 1


class QuarticEaseInOut(EasingBase):
    def func(self, t: float) -> float:
        if t < 0.5:
            return 8 * t * t * t * t
        p = t - 1
        return -8 * p * p * p * p + 1


"""
Quintic easing functions
"""


class QuinticEaseIn(EasingBase):
    def func(self, t: float) -> float:
        return t * t * t * t * t


class QuinticEaseOut(EasingBase):
    def func(self, t: float) -> float:
        return (t - 1) * (t - 1) * (t - 1) * (t - 1) * (t - 1) + 1


class QuinticEaseInOut(EasingBase):
    def func(self, t: float) -> float:
        if t < 0.5:
            return 16 * t * t * t * t * t
        p = (2 * t) - 2
        return 0.5 * p * p * p * p * p + 1


"""
Sine easing functions
"""


class SineEaseIn(EasingBase):
    def func(self, t: float) -> float:
        return math.sin((t - 1) * math.pi / 2) + 1


class SineEaseOut(EasingBase):
    def func(self, t: float) -> float:
        return math.sin(t * math.pi / 2)


class SineEaseInOut(EasingBase):
    def func(self, t: float) -> float:
        return 0.5 * (1 - math.cos(t * math.pi))


"""
Circular easing functions
"""


class CircularEaseIn(EasingBase):
    def func(self, t: float) -> float:
        return 1 - math.sqrt(1 - (t * t))


class CircularEaseOut(EasingBase):
    def func(self, t: float) -> float:
        return math.sqrt((2 - t) * t)


class CircularEaseInOut(EasingBase):
    def func(self, t: float) -> float:
        if t < 0.5:
            return 0.5 * (1 - math.sqrt(1 - 4 * (t * t)))
        return 0.5 * (math.sqrt(-((2 * t) - 3) * ((2 * t) - 1)) + 1)


"""
Exponential easing functions
"""


class ExponentialEaseIn(EasingBase):
    def func(self, t: float) -> float:
        if t == 0:
            return 0
        return math.pow(2, 10 * (t - 1))


class ExponentialEaseOut(EasingBase):
    def func(self, t: float) -> float:
        if t == 1:
            return 1
        return 1 - math.pow(2, -10 * t)


class ExponentialEaseInOut(EasingBase):
    def func(self, t: float) -> float:
        if t == 0 or t == 1:
            return t

        if t < 0.5:
            return 0.5 * math.pow(2, (20 * t) - 10)
        return -0.5 * math.pow(2, (-20 * t) + 10) + 1


"""
Elastic Easing Functions
"""


class ElasticEaseIn(EasingBase):
    def func(self, t: float) -> float:
        return math.sin(13 * math.pi / 2 * t) * math.pow(2, 10 * (t - 1))


class ElasticEaseOut(EasingBase):
    def func(self, t: float) -> float:
        return math.sin(-13 * math.pi / 2 * (t + 1)) * math.pow(2, -10 * t) + 1


class ElasticEaseInOut(EasingBase):
    def func(self, t: float) -> float:
        if t < 0.5:
            return (
                0.5
                * math.sin(13 * math.pi / 2 * (2 * t))
                * math.pow(2, 10 * ((2 * t) - 1))
            )
        return 0.5 * (
            math.sin(-13 * math.pi / 2 * ((2 * t - 1) + 1))
            * math.pow(2, -10 * (2 * t - 1))
            + 2
        )


"""
Back Easing Functions
"""


class BackEaseIn(EasingBase):
    def func(self, t: float) -> float:
        return t * t * t - t * math.sin(t * math.pi)


class BackEaseOut(EasingBase):
    def func(self, t: float) -> float:
        p = 1 - t
        return 1 - (p * p * p - p * math.sin(p * math.pi))


class BackEaseInOut(EasingBase):
    def func(self, t: float) -> float:
        if t < 0.5:
            p = 2 * t
            return 0.5 * (p * p * p - p * math.sin(p * math.pi))

        p = 1 - (2 * t - 1)

        return 0.5 * (1 - (p * p * p - p * math.sin(p * math.pi))) + 0.5


"""
Bounce Easing Functions
"""


class BounceEaseIn(EasingBase):
    def func(self, t: float) -> float:
        return 1 - BounceEaseOut().func(1 - t)


class BounceEaseOut(EasingBase):
    def func(self, t: float) -> float:
        if t < 4 / 11:
            return 121 * t * t / 16
        elif t < 8 / 11:
            return (363 / 40.0 * t * t) - (99 / 10.0 * t) + 17 / 5.0
        elif t < 9 / 10:
            return (4356 / 361.0 * t * t) - (35442 / 1805.0 * t) + 16061 / 1805.0
        return (54 / 5.0 * t * t) - (513 / 25.0 * t) + 268 / 25.0


class BounceEaseInOut(EasingBase):
    def func(self, t: float) -> float:
        if t < 0.5:
            return 0.5 * BounceEaseIn().func(t * 2)
        return 0.5 * BounceEaseOut().func(t * 2 - 1) + 0.5

# end ease in ease out

# Aliases for Going Left right back and forward


class Direction:
    """ Direction of movement is defined here"""
    LEFT = -1
    BACKWARD = -1
    FORWARD = 1
    RIGHT = 1
    UP = 1
    DOWN = -1


class Arm:
    """Which Arm motor"""
    LEFT = port.E
    RIGHT = port.B


class DriverMotor:
    """Which Port is Left and Right Driver Motors"""
    LEFT = port.A
    RIGHT = port.F


def get_drift(tgt_yaw):
    """Get drift gives how much you are drifted from your tgt_yaw angle."""
    c_yaw = get_yaw()
    # When robot is close to 360, it can drift to 2 or drift to 359
    # This will take into consideration all the cases.
    if tgt_yaw > 270 and c_yaw < 90:
        # This condition is when your target yaw is in Q4 and Current yaw is in Q1
        drift = 360 - tgt_yaw + c_yaw
    else:
        drift = c_yaw - tgt_yaw

    return drift


def get_yaw() -> int:
    """Gives current yaw in between 0 to 359
    As our Motor Left is connected to A and Right is Connected to B
    When turning right we get negative Yaw values """
    yaw = motion_sensor.tilt_angles()[0]
    # Get Remainder, Yaw angle after one full circle.
    yaw = (round(yaw/10 * -1) + 360) % 360
    return yaw


def angleDiff(tgt_yaw):
    """Give the angle difference between current yaw and target yaw
    There are 4 Cases Here:
        1. When turnnig Right and When current yaw is 350 and Target yaw is 30
        2. When turning Left and When current yaw is 10 and Target yaw is 350
        3. When turning Right and When target yaw is 90 and current yaw is 30
        4. When turning Left and When target yaw is 270 and current yaw is 350
"""
    cur_yaw = get_yaw()
    # right turn for robot and crossing 360 degree boundry
    if tgt_yaw < 90 and cur_yaw > 270:
        return 360 - cur_yaw + tgt_yaw
    # left turn for robot and crossing 360 degree boundry
    elif cur_yaw < 90 and tgt_yaw > 270:
        return 360 - tgt_yaw + cur_yaw
    elif tgt_yaw > cur_yaw:
        return tgt_yaw - cur_yaw

    return cur_yaw - tgt_yaw


async def straight(direction: int, distance: int, speed: int = 1050, accel: int = 2000):
    """ Drives straight with acceleration and deceleration."""
    global g_yaw
    tgtYaw = g_yaw

    # Resets the relative position of one of the wheels
    motor.reset_relative_position(DriverMotor.LEFT, 0)
    motor.reset_relative_position(DriverMotor.RIGHT, 0)

    drift = get_drift(tgtYaw) * 2

    # Set up easing functions for smooth speed transitions
    # start was 1600
    # Using CubicEaseInOut for fast acceleration & smooth deceleration
    easing = CubicEaseIn(start=speed * 1.6, end=400, duration=1)

    count = 0
    while distance > abs(motor.relative_position(DriverMotor.LEFT)):
        count = count + 1
        # Get current drift value
        drift = get_drift(tgtYaw)

        # Calculate the distance fraction (alpha) between 0 and 1
        current_distance = abs(motor.relative_position(DriverMotor.LEFT))
        # Normalize alpha between 0 and 1
        alpha = min(current_distance / distance, 1)

        # Use easing function to calculate the current speed
        true_speed = int(easing(alpha))
        if true_speed < 400:
            true_speed = 400

        if direction == Direction.BACKWARD:
            motor_pair.move(motor_pair.PAIR_1, drift,
                            velocity=true_speed * -1, acceleration=accel)
        else:
            motor_pair.move(motor_pair.PAIR_1, drift * -1,
                            velocity=true_speed, acceleration=accel)

    # Stops the motors after the loop
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)
    await runloop.sleep_ms(100)


async def turn(direction: int, degrees: int, speed: int, targetYaw: int = -500):
    """Direction is Direction.RIGHT or Direction.LEFT
    degrees: Amount of degrees to turn
    speed: speed at which to turn
    """
    global g_yaw
    tgtYaw = g_yaw
    tgtSpeed = speed
    origDiff = abs(degrees)
    minSpeed = 100

    if targetYaw >= -360 and targetYaw < 0:
        targetYaw = 360 + targetYaw

    if targetYaw == -500:
        if direction == Direction.RIGHT:
            tgtYaw = (g_yaw + degrees) % 360

        if direction == Direction.LEFT:
            tgtYaw = (g_yaw - degrees + 360) % 360
    else:
        tgtYaw = targetYaw
        origDiff = angleDiff(targetYaw)

    while (agdiff := angleDiff(tgtYaw)) > 0:
        tgtSpeed = int(max((agdiff/origDiff) * speed, minSpeed))
        # We need to turn both wheels backwards to turn Right
        motor.run(DriverMotor.LEFT, tgtSpeed * direction * -1)
        motor.run(DriverMotor.RIGHT, tgtSpeed * direction * -1)

    motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)
    g_yaw = tgtYaw  # Save the target yaw into our Global yaw.
    await runloop.sleep_ms(100)


async def setGearsLeft():
    attachmentMotor(Arm.RIGHT, 15, 500, Direction.UP)
    await attachmentMotor_async(Arm.LEFT, 15, 500, Direction.DOWN)


async def setGearsRight():
    attachmentMotor(Arm.LEFT, 15, 500, Direction.UP)
    await attachmentMotor_async(Arm.RIGHT, 15, 500, Direction.DOWN)


def attachmentMotor(workerMotor: int, degrees: int, speed: int, direction: int):
    """workerMotor is Arm.LEFT or Arm.RIGHT
    direction is Direction.UP, Direction.RIGHT, Direction.FORWARD all equal to 1
    And the others -1 degrees to turn speed with which the motor should turn.
    This function will not wait until the Lift action is performed
    """
    motor.run_for_degrees(workerMotor, degrees * direction, speed)


async def attachmentMotor_async(workerMotor: int, degrees: int, speed: int, direction: int):
    """This function will wait until the Lift action is performed"""
    await motor.run_for_degrees(workerMotor, degrees * direction, speed)


g_yaw = 0  # Define the global variable at the module level


async def readyForRun():
    global g_yaw
    g_yaw = 0
    motion_sensor.reset_yaw(0)


async def Run_1():
    await straight(Direction.BACKWARD, 150, 400)
    await turn(Direction.RIGHT, 45, 1000)
    await straight(Direction.BACKWARD, 550, 700)
    await turn(Direction.LEFT, 65, 500)
    await straight(Direction.BACKWARD, 170, 400)
    await turn(Direction.RIGHT, 70, 500)
    attachmentMotor(Arm.RIGHT, 200, 200, Direction.DOWN)
    await straight(Direction.BACKWARD, 700, 1000)
    await turn(Direction.RIGHT, 28, 500)
    await straight(Direction.BACKWARD, 270, 200)
    await turn(Direction.LEFT, 21, 500)
    await straight(Direction.BACKWARD, 30, 100)
    await turn(Direction.RIGHT, 26, 500)
    await straight(Direction.FORWARD, 300, 200)
    await turn(Direction.LEFT, 21, 500)
    await straight(Direction.FORWARD, 300, 200)
    await turn(Direction.LEFT, 20, 500)
    await straight(Direction.FORWARD, 850, 1000)
    await turn(Direction.LEFT, 204, 700)
    await attachmentMotor_async(Arm.RIGHT, 120, 400, Direction.UP)
    await straight(Direction.FORWARD, 350, 500)
    await turn(Direction.RIGHT, 0, 500, 271)
    await straight(Direction.FORWARD, 450, 400)
    await straight(Direction.BACKWARD, 300, 200)
    await turn(Direction.LEFT, 50, 150)
    await straight(Direction.BACKWARD, 1000, 1000)


async def main():
    """Main function"""
    global g_yaw
    g_yaw = 0

    motion_sensor.reset_yaw(0)
    motor_pair.pair(motor_pair.PAIR_1, DriverMotor.LEFT, DriverMotor.RIGHT)

    while True:
        color_detected = color_sensor.color(port.D)  # Read sensor value once
        if color_detected is color.BLUE:
            await readyForRun()
            await speedyRun_1()

        if color_detected is color.RED:
            await readyForRun()
            await Run_2()

        elif color_detected is color.WHITE:
            await readyForRun()
            await Run_3()

        elif color_detected is color.MAGENTA:  # research vessel
            await readyForRun()
            await Run_5_2()

        elif color_detected is color.YELLOW:  # whale krill
            await readyForRun()
            await Run_5_3()

        elif color_detected is color.AZURE:  # whale krill
            await readyForRun()
            await backupRun1()

        elif color_detected is color.GREEN:  # whale krill
            await readyForRun()
            await Run_2_backup()

        elif color_detected is color.BLACK:
            await setGearsLeft()
            await setGearsRight()


runloop.run(main())
