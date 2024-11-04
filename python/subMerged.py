from asyncio import run, sleep_ms
from hub import port
from hub import motion_sensor
import motor
import runloop
import motor_pair
import color_sensor
import color
# left motor is connected to port A and right motor is connected to port B

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


def angleDiff(tgt_yaw, direction):
    """Give the angle difference between current yaw and target yaw
    There are 4 Cases Here:
        1. When turnnig Right and When current yaw is 350 and Target yaw is 30
        2. When turning Left and When current yaw is 10 and Target yaw is 350
        3. When turning Right and When target yaw is 90 and current yaw is 30
        4. When turning Left and When target yaw is 270 and current yaw is 350
"""
    cur_yaw = get_yaw()
    if (cur_yaw == 0 and tgt_yaw == 360) or (cur_yaw == 360 and tgt_yaw == 0):
        return 0

    # right turn for robot and crossing 360 degree boundry
    if tgt_yaw > 0 and cur_yaw > tgt_yaw and direction == Direction.RIGHT:
        return 360 - cur_yaw + tgt_yaw
    # left turn for robot and crossing 360 degree boundry
    elif cur_yaw > 0 and tgt_yaw > cur_yaw and direction == Direction.LEFT:
        return 360 - tgt_yaw + cur_yaw

    if cur_yaw == 0 and direction == Direction.LEFT:
        cur_yaw = 360

    return abs(cur_yaw - tgt_yaw)


async def straight(direction: int, distance: int, speed: int, accel: int = 500):
    """ Drives straight"""
    global g_yaw
    tgtYaw = g_yaw
    minDistanceToFixYaw = 200
    minDistanceSpeed = 300

    # resets the relative position of one of the wheels
    motor.reset_relative_position(DriverMotor.LEFT, 0)
    motor.reset_relative_position(DriverMotor.RIGHT, 0)
    drift = get_drift(tgtYaw) * 2

    if (distance > minDistanceToFixYaw):
        while minDistanceToFixYaw > abs(motor.relative_position(DriverMotor.LEFT)):
            # sets the return value of the tuple to a tuple, so we can pull a specific value from it
            drift = get_drift(tgtYaw) * 2

            if direction == Direction.BACKWARD:
                motor_pair.move(motor_pair.PAIR_1, drift,
                                velocity=minDistanceSpeed * -1)
            else:
                motor_pair.move(motor_pair.PAIR_1, drift * -1,
                                velocity=minDistanceSpeed)
        distanceLeft = distance - \
            abs(motor.relative_position(DriverMotor.LEFT))

        if Direction.BACKWARD:
            distanceLeft = -distanceLeft
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, distanceLeft, 0, velocity=speed, stop=motor.BRAKE, acceleration=accel)

    else:
        while distance > abs(motor.relative_position(DriverMotor.LEFT)):
            # sets the return value of the tuple to a tuple, so we can pull a specific value from it
            drift = get_drift(tgtYaw)

            if direction == Direction.BACKWARD:
                motor_pair.move(motor_pair.PAIR_1, drift,
                                velocity=speed * -1)
            else:
                motor_pair.move(motor_pair.PAIR_1, drift * -1,
                                velocity=speed)

    # stops the motors after they are out of the while loop
    motor_pair.stop(motor_pair.PAIR_1)
    await runloop.sleep_ms(200)


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
        origDiff = angleDiff(targetYaw, direction)

    while (agdiff := angleDiff(tgtYaw, direction)) > 0:
        tgtSpeed = int(max((agdiff/origDiff) * speed, minSpeed))
        # We need to turn both wheels backwards to turn Right
        motor.run(DriverMotor.LEFT, tgtSpeed * direction * -1)
        motor.run(DriverMotor.RIGHT, tgtSpeed * direction * -1)

    motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)
    g_yaw = tgtYaw  # Save the target yaw into our Global yaw.
    await runloop.sleep_ms(200)


def attachmentMotor(workerMotor: int, degrees: int, speed: int, direction: int):
    """workerMotor is AttachMotor.LEFT or AttachMotor.RIGHT
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
    """ This is Run1 """
    await straight(Direction.BACKWARD, 2000, 1000, accel=1000)
    # await turn(Direction.LEFT, 45, 200,)
    # await straight(Direction.BACKWARD, 500, 1000, accel=1000)
    return
    await straight(Direction.FORWARD, 30, 500)
    await turn(Direction.LEFT, 30, 200,)
    await straight(Direction.FORWARD, 600, 500)
    await turn(Direction.RIGHT, 15, 200,)
    await straight(Direction.FORWARD, 500, 800)
    await turn(Direction.RIGHT, 70, 200)
    await straight(Direction.FORWARD, 100, 1050, accel=1000)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 360, 0, velocity=500)


async def Run_2():
    """ This is Run2 """
    print("This is Run2")


async def Run_3():
    """ This is Run3 ARYAN"""
    await turn(Direction.RIGHT, 0, 400, 90)
    return
    await runloop.sleep_ms(200)
    await turn(Direction.RIGHT, 0, 500, 90)


async def Run_4():
    """ This is Run4 """
    print("This is Run4")


async def Run_5():
    """ This is Run5 """
    print("This is Run5")

# Actual 1000 degrees = 21.377 inches


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
            await Run_1()

        if color_detected is color.RED:
            await readyForRun()
            await Run_2()

        elif color_detected is color.WHITE:
            await readyForRun()
            await Run_3()

        elif color_detected is color.YELLOW:
            await readyForRun()
            await Run_4()

        elif color_detected is color.MAGENTA:
            await readyForRun()
            await Run_5()

runloop.run(main())
