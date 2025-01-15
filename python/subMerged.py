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
prev_diff = 1000  # Define the global variable at the module level


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


def get_yaw() -> int:
    """Gives current yaw in between 0 to 359
    As our Motor Left is connected to A and Right is Connected to B
    When turning right we get negative Yaw values """
    yaw = motion_sensor.tilt_angles()[0]
    # Get Remainder, Yaw angle after one full circle.
    yaw = (round(yaw/10 * -1) + 360) % 360
    return yaw


def get_drift(tgt_yaw):
    """
    Calculate the drift between the target yaw and the current yaw.

    Parameters:
        tgt_yaw (int): Target yaw angle (0 to 359)

    Returns:
        int: Drift value (+ve or -ve)
    """
    cur_yaw = get_yaw()  # Fetch the current yaw angle
    # Calculate the raw drift
    drift = cur_yaw - tgt_yaw

    # Adjust drift to handle circular yaw values (0 to 359)
    if drift > 180:
        drift -= 360
    elif drift < -180:
        drift += 360

    return drift


def angleDiff(direction: int, tgt_yaw: int, curYaw: int = -500, update: bool = True) -> int:
    """Calculate the angle difference between current yaw and target yaw.
    Handles cases where yaw crosses the 0/360 boundary.
    Args:
        direction (int): Direction.RIGHT or Direction.LEFT
        init_yaw (int): Initial yaw angle
        tgt_yaw (int): Target yaw angle
        prev_diff (int): Previous difference to check for overshoot
    Returns:
        int: Angle difference or -1 if overshoot occurs
    """
    global prev_diff
    cur_yaw = get_yaw()

    if curYaw != -500:
        cur_yaw = curYaw

    diff = prev_diff
    if direction == Direction.RIGHT:
        diff = (tgt_yaw - cur_yaw) % 360
    else:
        diff = (cur_yaw - tgt_yaw) % 360

    # print("AngleDiff curr_yaw=",cur_yaw, " diff=",diff, " prev_diff=",prev_diff, " diff-360=", diff-360)
    if (abs(diff - prev_diff) <= 10 and prev_diff != 1000) or diff <= prev_diff:
        prev_diff = diff if update else prev_diff
        return diff
    return diff - 360  # Indicating overshoot


async def straight(direction: int, distance: int, speed: int = 1050, accel: int = 2000, msg="Hello"):
    """ Drives straight with acceleration and deceleration."""
    global g_yaw
    tgtYaw = g_yaw
    a = time.ticks_ms()
    # Resets the relative position of one of the wheels
    motor.reset_relative_position(DriverMotor.LEFT, 0)
    motor.reset_relative_position(DriverMotor.RIGHT, 0)

    drift = get_drift(tgtYaw)

    print(msg, " Straight: ", "direction", direction, "cur_yaw=", get_yaw(), " drift=",
          drift, " tgtYaw or gyaw=", tgtYaw, " distance=", distance, " speed=", speed)

    # Set up easing functions for smooth speed transitions
    # start was 1600
    # Using CubicEaseInOut for fast acceleration & smooth deceleration
    easing = CubicEaseIn(start=speed*1.6, end=400, duration=1)

    while distance > abs(motor.relative_position(DriverMotor.LEFT)):
        # Get current drift value
        drift = int(get_drift(tgtYaw) * 1.5)

        # Calculate the distance fraction (alpha) between 0 and 1
        current_distance = abs(motor.relative_position(DriverMotor.LEFT))
        # Normalize alpha between 0 and 1
        alpha = min(current_distance / distance, 1)

        # Use easing function to calculate the current speed
        true_speed = int(easing(alpha))
        if true_speed < 400:
            true_speed = 400
        # print("Straight drift=",drift, " Speed=",true_speed, " yaw=",get_yaw(), " current_distance=",current_distance)
        if direction == Direction.BACKWARD:
            motor_pair.move(motor_pair.PAIR_1, drift,
                            velocity=true_speed * -1, acceleration=accel)
        else:
            motor_pair.move(motor_pair.PAIR_1, drift * -1,
                            velocity=true_speed, acceleration=accel)

    # Stops the motors after the loop
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)
    await runloop.sleep_ms(100)
    b = time.ticks_ms()

    print("End Straight: cur_yaw=", get_yaw(), " drift=", drift, " tgtYaw or gyaw=", tgtYaw,
          " distance=", abs(motor.relative_position(DriverMotor.LEFT)), " speed=", speed, " Time took=", (b-a)/1000, " Seconds\n")


async def turn(direction: int, degrees: int, speed: int = -1, targetYaw: int = -500, error: float = 0.02):
    """
    Turn the robot in a specific direction for a given number of degrees or to a target yaw angle.

    Parameters:
        direction: Direction.RIGHT or Direction.LEFT
        ent_degrees: Amount of degrees to turn (if provided)
        speed: Speed at which to turn
        targetYaw: The final yaw angle to turn to (if provided)
    """
    a = time.ticks_ms()
    global prev_diff
    global g_yaw
    minSpeed = 100
    prev_diff = 1000  # Reset previous difference for comparison

    # Calculate target yaw if ent_degrees are provided
    if degrees != 0:  # If amount of degrees to turn is provided
        targetYaw = (g_yaw + degrees * (1 if direction ==
                                        Direction.RIGHT else -1)) % 360
    else:  # then targetYaw is provided
        targetYaw %= 360

    origDiff = angleDiff(direction, targetYaw, g_yaw, update=False)
    actualDiff = angleDiff(direction, targetYaw, update=False)

    # if g_yaw is 0 and targetYaw=90 but robot already crossed 90 by hitting a mission then generally the robot spins fast to get to the 90 which is not required.
    # To avoid that we calculate the diff between g_yaw and target yaw and cur_yaw and target yaw.
    if abs(actualDiff - origDiff) > 40:
        print("Direction=", direction, "g_yaw=", g_yaw, " targetYaw=", targetYaw, " cur_yaw=", get_yaw(
        ), "\nDiff between g_yaw and targetYaw=", origDiff, "Diff between currYaw and targetYaw=", actualDiff)
        print("You do not need a turn, you are past the targetYaw\n")
        return

    breakAhead = error * origDiff

    if speed == -1:
        # speed = int(origDiff * 8.75)
        speed = 1000
        if origDiff < 60:
            breakAhead = 0.45 * origDiff
        elif origDiff < 85:
            breakAhead = 0.385 * origDiff
        elif origDiff >= 85 and origDiff < 100:
            breakAhead = 0.145 * origDiff
        elif origDiff >= 100 and origDiff <= 118:
            breakAhead = 0.109 * origDiff
        elif origDiff > 118 and origDiff <= 129:
            breakAhead = 0.05 * origDiff
        elif origDiff > 130:
            breakAhead = 0.025 * origDiff

    easing = SineEaseIn(start=speed, end=minSpeed, duration=1)

    print("Turn Start: Start Angle=", g_yaw, "Target Angle=", targetYaw, " Curent Angle=", get_yaw(),
          "Speed=", speed, " Degrees to Turn=", origDiff, " BreakAhead=", breakAhead)

    while (agdiff := angleDiff(direction, targetYaw)) > breakAhead:
        tgtSpeed = int(max((agdiff/origDiff) * speed, minSpeed))

        alpha = min(1 - (agdiff / origDiff), 1)
        # Use easing function to calculate the current speed
        tgtSpeed = int(easing(alpha))

        if tgtSpeed < minSpeed:
            tgtSpeed = minSpeed
        # tgtSpeed = int(max((agdiff/origDiff) * speed, 400))
        # motor_pair.move_tank(motor_pair.PAIR_1, tgtSpeed * direction,
        #                    tgtSpeed * direction * -1, acceleration=2000)
        motor.run(DriverMotor.LEFT, tgtSpeed * direction * -1)
        motor.run(DriverMotor.RIGHT, tgtSpeed * direction * -1)
        # print("speed=",tgtSpeed,"Agdiff:",agdiff)

    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)

    initangle = g_yaw

    g_yaw = targetYaw  # Save the target yaw into our Global yaw.
    await runloop.sleep_ms(200)
    b = time.ticks_ms()
    print("Turn End: Start Angle=", initangle, "Target Angle=", targetYaw, " Actual Reached Angle=", get_yaw(
    ), "Speed=", speed, " error=", angleDiff(direction, g_yaw, update=False), " Time=", (b-a)/1000, " Seconds\n")


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
    motor.run_for_degrees(workerMotor, degrees *
                          direction, speed, stop=motor.HOLD)


async def attachmentMotor_async(workerMotor: int, degrees: int, speed: int, direction: int):
    """This function will wait until the Lift action is performed"""
    await motor.run_for_degrees(workerMotor, degrees * direction, speed, stop=motor.HOLD)


def holdMotor(workerMotor: int, direction, hold_time: int):
    a = time.ticks_ms()
    while (time.ticks_ms() - a) < hold_time:
        motor.run_for_degrees(workerMotor, 1*direction, 100, stop=motor.HOLD)
    print("Held the motor for=", (time.ticks_ms()-a), " ms")


async def readyForRun():
    global g_yaw
    g_yaw = 0
    motion_sensor.reset_yaw(0)


async def Run_1():
    "This is Run 1"
    await straight(Direction.FORWARD, 400, 500)        # Go Straight
    # Turn right towards the boat
    await turn(Direction.RIGHT, 0, -1, targetYaw=90)
    # Go straight in the direction of boat
    await straight(Direction.FORWARD, 700, 800)
    # Take a sharp turn Towards the coral
    await turn(Direction.RIGHT, 0, -1, targetYaw=135)
    # Go straight to collect the Coral
    await straight(Direction.BACKWARD, 500, 800)
    # Turn Right to collect Krill and Water Sample
    await turn(Direction.RIGHT, 0, -1, targetYaw=135)
    # Go straight to collect water sample and krill
    await straight(Direction.BACKWARD, 200, 500)


async def Run_2():
    "This is Run_2"
    # Go Straight towards Coral Tree
    await straight(Direction.BACKWARD, 770, 1000)
    # Drop the Coral Tree
    await attachmentMotor_async(Arm.RIGHT, 90, 300, Direction.DOWN)
    await runloop.sleep_ms(200)
    # Lift the arm after dropping the Coral Tree
    await attachmentMotor_async(Arm.RIGHT, 40, 300, Direction.UP)

    # Turn Right
    await turn(Direction.RIGHT, 0, 1000, targetYaw=45, error=0.4)
    # Go At 45 degrees So we can turn towards the scuba diver
    await straight(Direction.BACKWARD, 760, 800)
    # Turn towards the Scuba diver mission
    await turn(Direction.LEFT, 0, 1000, targetYaw=-90, error=0.04)
    # Parallely bend down so that easy to lift the scuba diver
    attachmentMotor(Arm.RIGHT, 30, 300, Direction.DOWN)  # 35,300
    # Move towards the Scuba diver mission
    await straight(Direction.BACKWARD, 450, 300)  # 460
    # Parallely Pick up the Scuba Diver
    attachmentMotor(Arm.RIGHT, 150, 300, Direction.UP)
    # Slam the shark mision
    await attachmentMotor_async(Arm.LEFT, 270, 1000, Direction.UP)  # 280
    # Lift the Shark arm completely back
    attachmentMotor(Arm.LEFT, 250, 1000, Direction.DOWN)  # 300

    # Back up from the Scuba diver mission
    await straight(Direction.FORWARD, distance=70, speed=800)  # 125#150#160
    # Turn Right to face the coral nursery
    await turn(Direction.RIGHT, 0, 1000, targetYaw=0, error=0.12)
    # Move towards the Coral Nursery
    # 230,300. -> 230,500
    await straight(Direction.BACKWARD, 240, 500)

    # Hit the Coral Nursery
    # 255
    await attachmentMotor_async(Arm.LEFT, degrees=220, speed=1000, direction=Direction.UP)
    # After hitting the coral nursery lift the ARM
    await attachmentMotor_async(Arm.LEFT, 100, 1000, Direction.DOWN)
    # Move Away from the Coral Nursery
    await straight(Direction.FORWARD, 140, 300)                    # 130
    # Turn towards the post of the scuba diver or cora nursery
    await turn(Direction.RIGHT, 0, 800, targetYaw=60, error=0.20)  # .15 error
    # Move towards the Coral Nursery
    await straight(Direction.BACKWARD, 150, 300)
    # Deliver the scuba diver
    await attachmentMotor_async(Arm.RIGHT, 150, 300, Direction.DOWN)
    # wait for the scuba diver to settle.
    await runloop.sleep_ms(200)

    # After delivering scuba diver go back a bit
    await straight(Direction.FORWARD, 100, 300)  # 150
    # Turn away from coral nurssery
    await turn(Direction.LEFT, 0, 1000, targetYaw=10)

    # Lift the Arms so when at home we are not hanging out
    attachmentMotor(Arm.RIGHT, 150, 300, Direction.UP)
    # Back to home in Arch turn.
    await motor_pair.move_for_degrees(
        motor_pair.PAIR_1, 1500, 7, velocity=1050, acceleration=8000)


async def Run_atharv_old():
    "Run Artificial Habitat squid angular fish"
    await straight(Direction.BACKWARD, 77)
    await turn(Direction.LEFT, 0, 500, targetYaw=-45)
    await straight(Direction.BACKWARD, 900, 900)  # hitting the squid mission
    # await runloop.sleep_ms(500) #letting the squid fall in
    await straight(Direction.FORWARD, 300)  # backing up from squid mission

    # starting to go towars angler fish
    await turn(Direction.LEFT, 45, 500)
    await straight(Direction.BACKWARD, 250)  # going west
    await turn(Direction.RIGHT, 0, 500, -57)  # turning toward angler fish
    await straight(Direction.BACKWARD, 900, 1200)  # ram into angler fish
    # await straight(Direction.BACKWARD, 900, 700) #ram into angler fish
    # await turn(Direction.LEFT, 10, 500) # push the angler fish in
    # await turn(Direction.RIGHT, 0, 500, -55) # turn back into previous position
    await straight(Direction.FORWARD, 100, 700)  # go away from angler fish

    # drop squid
    await turn(Direction.RIGHT, 65, 500)
    await straight(Direction.BACKWARD, 120, 900)
    # lift arm To drop of Squid
    attachmentMotor(Arm.LEFT, 120, 700, Direction.DOWN)
    await runloop.sleep_ms(500)
    await straight(Direction.FORWARD, 180, 700)

    # start going towards artificial habitat
    await turn(Direction.LEFT, 70, 500)
    await straight(Direction.FORWARD, 800)
    # await turn(Direction.LEFT, 75, 700)
    await turn(Direction.LEFT, 0, 700, targetYaw=-135)
    await straight(Direction.BACKWARD, 40)

    # reached artificial habitat
    # Vam smacks the arm down in the next step
    # drop arm to turn the artificial habitat
    await attachmentMotor_async(Arm.LEFT, 150, 700, Direction.UP)
    await turn(Direction.RIGHT, 75, 900)  # folding the artificial habitat
    # aligning to get ready to flip artifical habitat
    await straight(Direction.BACKWARD, 150)
    # changed the spinning sometimes turn
    await turn(Direction.LEFT, 0, 900, -95)
    attachmentMotor(Arm.LEFT, 900, 600, Direction.DOWN)  # lift arm
    attachmentMotor(Arm.RIGHT, 900, 600, Direction.DOWN)  # lift arm
    await straight(Direction.FORWARD, 100, 800)

    await straight(Direction.BACKWARD, 200)  # ramming
    # Vam Increased the angle from 900 to 1200, to be more effective
    attachmentMotor(Arm.RIGHT, 1200, 1200, Direction.UP)
    # attachmentMotor(Arm.RIGHT, 900, 1200, Direction.UP)
    # aligning to get ready to flip artifical habitat
    await straight(Direction.BACKWARD, 215)
    # Up until here, it rams & lifts the habitat once
    # await attachmentMotor_async(Arm.RIGHT, 900, 1200, Direction.DOWN)# puts arm down for flip### Vam commented this line, as this is many times causing the attchment to get stuck in the habitat
    await turn(Direction.RIGHT, 0, 500, -90)  # turns to align for last flip
    await straight(Direction.BACKWARD, 100, 500)  # gets in range to flip it
    await attachmentMotor_async(Arm.RIGHT, 900, 1200, Direction.UP)  # flips it

    return
    # boat mission

    # backs up to turn, aprroach, and do mission
    await straight(Direction.FORWARD, 800, 900)
    attachmentMotor(Arm.RIGHT, 900, 1200, Direction.DOWN)
    attachmentMotor(Arm.LEFT, 100, 1200, Direction.DOWN)
    # turns to aprroach and do mission
    await turn(Direction.RIGHT, 0, 1000, 43)
    await straight(Direction.BACKWARD, 590, 500)  # approaches mission
    await attachmentMotor_async(Arm.RIGHT, 900, 1200, Direction.UP)

    # krill collection

    await straight(Direction.FORWARD, 350, 700)  # backing up
    attachmentMotor(Arm.RIGHT, 900, 1200, Direction.DOWN)
    await turn(Direction.LEFT, 0, 500, -40)  # turning to the kill
    await straight(Direction.BACKWARD, 325, 500)  # catches the first krill
    # aligning to get coral and second krill
    await turn(Direction.RIGHT, 0, 500, 3)
    # getting coral and second krill
    await straight(Direction.BACKWARD, 500, 500)
    await turn(Direction.RIGHT, 0, 500, 50)  # aligning the third krill
    await straight(Direction.BACKWARD, 150, 500)  # collecting third krill
    # closing gate on attachment
    await attachmentMotor_async(Arm.LEFT, 900, 1200, Direction.UP)
    # Vam At this step, the robot is at the whale? station (where krills are dropped)
    # return
    await straight(Direction.FORWARD, 250, 700)
    await turn(Direction.RIGHT, 0, 500, 165)

    attachmentMotor(Arm.LEFT, 900, 1200, Direction.DOWN)

    await straight(Direction.BACKWARD, 1250, 1000)


async def main():
    """Main function"""
    global g_yaw
    global prev_diff

    g_yaw = 0
    prev_diff = 1000

    motion_sensor.reset_yaw(0)
    motor_pair.pair(motor_pair.PAIR_1, DriverMotor.LEFT, DriverMotor.RIGHT)
    a = time.ticks_ms()
    """
    await straight(Direction.FORWARD, distance=180, speed=400)# 125#150#160
    await turn(Direction.RIGHT, 0, 1000, targetYaw=90,error=0.12)
    # Move towards the Coral Nursery
    await straight(Direction.BACKWARD, 275, 300)                    # 230
    # Hit the Coral Nursery
    await attachmentMotor_async(Arm.LEFT, degrees=255, speed=1000, direction=Direction.UP)
    """

    # await Run_2()

    # await attachmentMotor_async(Arm.LEFT, 120, 600, Direction.DOWN)#lift arm
    # c = time.ticks_ms()
    # holdMotor(Arm.LEFT,Direction.DOWN,4000)
    # d = time.ticks_ms()
    # print("TIme=", d-c)
    # await attachmentMotor_async(Arm.LEFT, 120, 600, Direction.UP)#lift arm
    # await Run_atharv_old()
    await Run_1()

    b = time.ticks_ms()
    print("Time it took to run Run 1 is ", (b-a)/1000, " Seconds\n")

    return
    while True:
        color_detected = color_sensor.color(port.D)  # Read sensor value once
        if color_detected is color.BLUE:
            await readyForRun()
            await Run_1()

        if color_detected is color.RED:
            await readyForRun()
            await Run_2()

        elif color_detected is color.WHITE:
            await readyForRun()

        elif color_detected is color.MAGENTA:  # research vessel
            await readyForRun()

        elif color_detected is color.YELLOW:  # whale krill
            await readyForRun()

        elif color_detected is color.AZURE:  # whale krill
            await readyForRun()

        elif color_detected is color.GREEN:  # whale krill
            await readyForRun()

        elif color_detected is color.BLACK:
            await setGearsLeft()
            await setGearsRight()


runloop.run(main())
