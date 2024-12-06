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

def angleDiff(tgt_yaw, direction, init_yaw = -500):
    """Give the angle difference between current yaw and target yaw
    There are 4 Cases Here:
        1. When turnnig Right and When current yaw is 350 and Target yaw is 30
        2. When turning Left and When current yaw is 10 and Target yaw is 350
        3. When turning Right and When target yaw is 90 and current yaw is 30
        4. When turning Left and When target yaw is 270 and current yaw is 350
    """
    cur_yaw = get_yaw()

# If we want the distance between a different angle than the current yaw then
# we pass on the angle as init_yaw and that will be used. 
    if init_yaw != -500:
        cur_yaw = init_yaw

    #By default lets take a Right turn 
    start = tgt_yaw
    end = cur_yaw

    if direction == Direction.LEFT:
        start = cur_yaw
        end =  tgt_yaw
    
    return (start - end) % 360


async def turn(direction: int, degrees: int, speed: int, targetYaw: int = -500, error: float =0.0):
    """Direction is Direction.RIGHT or Direction.LEFT
    degrees: Amount of degrees to turn
    speed: speed at which to turn
    """
    global g_yaw
    tgtYaw = g_yaw
    tgtSpeed = speed
    minSpeed = 100

    if targetYaw == -500:
        tgtYaw = (g_yaw + (degrees * direction)) % 360
    else:
        tgtYaw = (targetYaw + 360) % 360

    origDiff = angleDiff(tgtYaw, direction, g_yaw) + 30 #90
    actualDiff = angleDiff(tgtYaw, direction) #91


    print (" Turn direction ", direction, " Degrees to turn: ", degrees, " Global Yaw ", g_yaw, " Orig Diff = ",origDiff, "actualDiff = ", actualDiff, "Current Yaw", get_yaw(), "Target Yaw = ", tgtYaw, " Speed ", speed)
    print( actualDiff , " > ", int(speed *error)+1, " and ", actualDiff, " < ", origDiff+1)

    while (actualDiff > int(speed * error)+1) :
        motor.run(DriverMotor.LEFT, speed * direction * -1)
        motor.run(DriverMotor.RIGHT, speed * direction * -1)
        oldactualDiff = actualDiff
        actualDiff = angleDiff(tgtYaw, direction)
        if actualDiff > oldactualDiff:
            motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_COAST)
            break
    print( "END" , actualDiff , " > ", int(speed *error)+1, " and ", actualDiff, " < ", origDiff)

    motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_COAST)
    g_yaw = tgtYaw# Save the target yaw into our Global yaw.
    await runloop.sleep_ms(1000)
    print("Turn complete, curr yaw is ", get_yaw())


async def straight(direction: int, distance: int, speed: int, accel: int = 500, justGoFast: int = 0):
    """ Drives straight"""
    global g_yaw
    tgtYaw = g_yaw
    minDistanceToFixYaw = 200
    minDistanceSpeed = 300

    # resets the relative position of one of the wheels
    motor.reset_relative_position(DriverMotor.LEFT, 0)
    motor.reset_relative_position(DriverMotor.RIGHT, 0)
    drift = get_drift(tgtYaw) * 3

    if (distance > minDistanceToFixYaw and justGoFast is 0):
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

        if direction == Direction.BACKWARD:
            distanceLeft = - distanceLeft

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
    motor_pair.stop(motor_pair.PAIR_1,stop=motor.SMART_BRAKE)
    await runloop.sleep_ms(1000)
    print("Straight complete, curr yaw is ", get_yaw())


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


async def setGearsLeft():
    attachmentMotor(Arm.RIGHT, 15, 500, Direction.UP)
    await attachmentMotor_async(Arm.LEFT, 15, 500, Direction.DOWN)

async def setGearsRight():
    attachmentMotor(Arm.LEFT, 15, 500, Direction.UP)
    await attachmentMotor_async(Arm.RIGHT, 15, 500, Direction.DOWN)


g_yaw = 0# Define the global variable at the module level


async def readyForRun():
    global g_yaw
    g_yaw = 0
    motion_sensor.reset_yaw(0)


async def main():
    """Main function"""
    global g_yaw
    g_yaw = 0

    motion_sensor.reset_yaw(0)
    motor_pair.pair(motor_pair.PAIR_1, DriverMotor.LEFT, DriverMotor.RIGHT)

    await straight(Direction.BACKWARD, 200, 800)# Moving Straight
    await turn(Direction.RIGHT, 0, 500, 90, 0.039)# TURN 1
    await runloop.sleep_ms(5000)
    print(" -----------------ABout to start second turn ")
    await turn(Direction.RIGHT, 0, 500, 120, 0.039)# TURN 1
    #await runloop.sleep_ms(2000)
    #print(" -----------------ABout to start Third turn ")
    #await turn(Direction.RIGHT, 0, 500, 120, 0.009)# TURN 1
    print (" ***************************")




    return

    while True:
        color_detected = color_sensor.color(port.D)# Read sensor value once
        if color_detected is color.BLUE:
            await readyForRun()
            await speedyRun_1()

        if color_detected is color.RED:
            await readyForRun()
            await Run_2()

        elif color_detected is color.WHITE:
            await readyForRun()
            await Run_3()

        elif color_detected is color.MAGENTA:# research vessel
            await readyForRun()
            await Run_5_2()

        elif color_detected is color.YELLOW:# whale krill
            await readyForRun()
            await Run_5_3()

        elif color_detected is color.AZURE:# whale krill
            await readyForRun()
            await backupRun1()

        elif color_detected is color.GREEN:# whale krill
            await readyForRun()
            await Run_2_backup()

        elif color_detected is color.BLACK:
            await setGearsLeft()
            await setGearsRight()


runloop.run(main())
