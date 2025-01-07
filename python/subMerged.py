from asyncio import run, sleep_ms
from hub import port
from hub import motion_sensor
import motor
import runloop
import motor_pair
import color_sensor
import color
import time
import math


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


# end ese in ease out

g_yaw = 0  # Define the global variable at the module level


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


def angleDiff(direction, init_yaw, tgt_yaw, prev_diff=361):
    """Give the angle difference between current yaw and target yaw
    There are 4 Cases Here:
        1. When turnnig Right and When current yaw is 350 and Target yaw is 30
        2. When turning Left and When current yaw is 10 and Target yaw is 350
        3. When turning Right and When target yaw is 90 and current yaw is 30
        4. When turning Left and When target yaw is 270 and current yaw is 350
"""
    cur_yaw = get_yaw()
    if direction == Direction.RIGHT:
        if init_yaw < tgt_yaw:
            diff = tgt_yaw - cur_yaw
            if diff < prev_diff:
                prev_diff = diff
                return diff
            else:
                return -1  # Init yaw is 10 and target yaw is 5, If we overshoot that target Yaw then we will have to take another
            # turn instead just return -1 indicating we overshoot.
        else:
            if cur_yaw >= init_yaw and cur_yaw <= 360:
                diff = 360 - cur_yaw + tgt_yaw
                if diff < prev_diff:
                    prev_diff = diff
                    return diff
                else:
                    return -1
            else:
                diff = tgt_yaw - cur_yaw
                if diff < prev_diff:
                    prev_diff = diff
                    return diff
                else:
                    return -1
    else:  # When turning Left
        if init_yaw > tgt_yaw:
            diff = cur_yaw - tgt_yaw
            if diff < prev_diff:
                prev_diff = diff
                return diff
            else:
                return -1
        else:
            if cur_yaw <= init_yaw and cur_yaw >= 0:
                diff = cur_yaw + 360 - tgt_yaw
                if diff < prev_diff:
                    prev_diff = diff
                    return diff
                else:
                    return -1
            else:
                diff = cur_yaw - tgt_yaw
                if diff < prev_diff:
                    prev_diff = diff
                    return diff
                else:
                    return -1


async def straight(direction: int, distance: int, speed: int = 1050, accel: int = 2000):
    """ Drives straight with acceleration and deceleration."""
    global g_yaw
    tgtYaw = g_yaw

    # Resets the relative position of one of the wheels
    motor.reset_relative_position(DriverMotor.LEFT, 0)
    motor.reset_relative_position(DriverMotor.RIGHT, 0)

    drift = get_drift(tgtYaw) * 1

    # Set up easing functions for smooth speed transitions
    # start was 1600
    # Using CubicEaseInOut for fast acceleration & smooth deceleration
    easing = CubicEaseIn(start=speed*1.6, end=400, duration=1)

    while distance > abs(motor.relative_position(DriverMotor.LEFT)):
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


async def turn(direction: int, degrees: int, speed: int = -1, targetYaw: int = -500):
    """Direction is Direction.RIGHT or Direction.LEFT
    degrees: Amount of degrees to turn
    speed: speed at which to turn
    """
    global g_yaw
    if degrees == 0:
        degreesToTurn = targetYaw-g_yaw
    else:
        degreesToTurn = degrees

    if abs(degreesToTurn) == 300:
        degreesToTurn = 299

    if speed == -1:
        ref_speed = round(abs(degreesToTurn) * 9)
        if ref_speed > 1050:
            ref_speed = 1050
    else:
        ref_speed = speed

    ref_speed = abs(ref_speed)

    tgtYaw = g_yaw
    tgtSpeed = speed
    origDiff = abs(degreesToTurn)

    if targetYaw >= -360 and targetYaw < 0:
        targetYaw = 360 + targetYaw

    if targetYaw == -500:
        if direction == Direction.RIGHT:
            tgtYaw = (g_yaw + abs(degreesToTurn)) % 360

        if direction == Direction.LEFT:
            tgtYaw = (g_yaw - abs(degreesToTurn) + 360) % 360
    else:
        tgtYaw = targetYaw
        origDiff = angleDiff(targetYaw)

    easing = SineEaseIn(start=ref_speed, end=200, duration=1)

    while (agdiff := angleDiff(tgtYaw)) > (round(speed/(300-abs(degreesToTurn)))+6.9):
        alpha = min(1 - (agdiff / origDiff), 1)
        # Use easing function to calculate the current speed
        tgtSpeed = int(easing(alpha))

        if tgtSpeed < 200:
            tgtSpeed = 200
        # tgtSpeed = int(max((agdiff/origDiff) * speed, minSpeed))
        motor_pair.move_tank(motor_pair.PAIR_1, tgtSpeed * direction,
                             tgtSpeed * direction * -1, acceleration=2000)

    motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)
    g_yaw = tgtYaw  # Save the target yaw into our Global yaw.
    await runloop.sleep_ms(200)


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


async def readyForRun():
    global g_yaw
    g_yaw = 0
    motion_sensor.reset_yaw(0)


async def Run_1():
    "This is Run 1"
    await straight(Direction.FORWARD, 250, 500)
    await turn(Direction.LEFT, 135, 1000)
    await straight(Direction.BACKWARD, 600, 700)
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
    await turn(Direction.LEFT, 0, 800, 14)
    await attachmentMotor_async(Arm.RIGHT, 120, 400, Direction.UP)
    await straight(Direction.FORWARD, 350, 500)
    await turn(Direction.RIGHT, 0, 500, 89)
    await straight(Direction.FORWARD, 450, 200)
    await straight(Direction.BACKWARD, 500, 200)
    await attachmentMotor_async(Arm.RIGHT, 120, 400, Direction.DOWN)
    await turn(Direction.LEFT, 20, 150)
    await straight(Direction.FORWARD, 150, 400)
    await turn(Direction.LEFT, 0, 150, 20)
    await straight(Direction.BACKWARD, 1000, 1000)


async def Run_2():
    "This is Run_2"
    # Go Straight towards Coral Tree
    await straight(Direction.BACKWARD, 775, 800)
    # Drop the Coral Tree
    await attachmentMotor_async(Arm.RIGHT, 90, 300, Direction.DOWN)
    await runloop.sleep_ms(200)
    # Lift the arm after dropping the Coral Tree
    await attachmentMotor_async(Arm.RIGHT, 40, 300, Direction.UP)
    # Turn Right
    await turn(Direction.RIGHT, 0, -1, targetYaw=45)
    # Go At 45 degrees So we can turn towards the scuba diver
    await straight(Direction.BACKWARD, 750, 800)
    # Turn towards the Scuba diver mission
    await turn(Direction.LEFT, 0, -1, targetYaw=-90)
    # Parallely bend down so that easy to lift the scuba diver
    attachmentMotor(Arm.RIGHT, 35, 300, Direction.DOWN)
    # Move towards the Scuba diver mission
    await straight(Direction.BACKWARD, 460, 300)
    # Parallely Pick up the Scuba Diver
    attachmentMotor(Arm.RIGHT, 150, 300, Direction.UP)
    # Slam the shark mision
    await attachmentMotor_async(Arm.LEFT, 280, 1000, Direction.UP)
    # Lift the Shark arm completely back
    attachmentMotor(Arm.LEFT, 300, 1000, Direction.DOWN)
    # Back up from the Scuba diver mission
    # Use to be 125 changed the distance to 150
    await straight(Direction.FORWARD, distance=150, speed=300)
    a = time.ticks_us()
    # Turn Right to face the coral nursery
    await turn(Direction.RIGHT, 0, -1, targetYaw=0)
    b = time.ticks_us()
    print("Time took to turn towards coral nursery is ",
          (b-a)/1000000, " seconds.")
    # Move towards the Coral Nursery
    await straight(Direction.BACKWARD, 230, 300)
    # Hit the Coral Nursery
    await attachmentMotor_async(Arm.LEFT, degrees=250, speed=1000, direction=Direction.UP)
    # After hitting the coral nursery lift the ARM
    await attachmentMotor_async(Arm.LEFT, 100, 1000, Direction.DOWN)
    # Move Away from the Coral Nursery
    await straight(Direction.FORWARD, 130, 300)
    # Turn towards the post of the scuba diver or cora nursery
    await turn(Direction.RIGHT, 0, -1, targetYaw=60)
    # Move towards the Coral Nursery
    await straight(Direction.BACKWARD, 150, 300)
    # Deliver the scuba diver
    await attachmentMotor_async(Arm.RIGHT, 150, 300, Direction.DOWN)
    # After delivering scuba diver go back a bit
    await straight(Direction.FORWARD, 150, 300)
    # Turn away from coral nurssery
    await turn(Direction.LEFT, 0, -1, targetYaw=10)

    # Back to home in Arch turn.
    motor_pair.move_for_degrees(
        motor_pair.PAIR_1, 1500, 5, velocity=1000, acceleration=5000)


async def main():
    """Main function"""
    global g_yaw
    g_yaw = 0

    motion_sensor.reset_yaw(0)
    motor_pair.pair(motor_pair.PAIR_1, DriverMotor.LEFT, DriverMotor.RIGHT)

    a = time.ticks_ms()
    await turn(direction=Direction.LEFT, degrees=10, speed=200, targetYaw=-10)
    # await Run_1()
    # await Run_2()
    b = time.ticks_ms()
    print("Time it took to run Run_2 is ", (b-a)/1000, " Seconds\n")

    return

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
