from asyncio import run, sleep_ms
from hub import port
from hub import motion_sensor
import motor
import runloop
import motor_pair
import color_sensor
import color
import time
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


def accelDeccel(distance, speed, min_speed: int = 300):
    #185 used to be distance by 6.5
    if abs(motor.relative_position(port.A)) < 160:
        acceldeccel = (abs(motor.relative_position(port.A))/distance) * speed * distance/263.16

        #distance-185 used to be distance/4 * 3
    elif abs(motor.relative_position(port.A)) > distance - 185:
        acceldeccel = distance/abs(motor.relative_position(port.A))-(distance/5) + (abs(motor.relative_position(port.A))/5)

    else:
        acceldeccel = speed

    if acceldeccel < min_speed + 1:
        acceldeccel = min_speed

    return acceldeccel

async def straight(direction: int, distance: int, speed: int = 1050, accel: int = 500, justGoFast: int = 1):
    """ Drives straight"""
    global g_yaw
    tgtYaw = g_yaw
    minDistanceToFixYaw = 200
    minDistanceSpeed = 300

    # resets the relative position of one of the wheels
    motor.reset_relative_position(DriverMotor.LEFT, 0)
    motor.reset_relative_position(DriverMotor.RIGHT, 0)
    drift = get_drift(tgtYaw) * 2

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

            true_speed = int(accelDeccel(distance, speed))

            if direction == Direction.BACKWARD:
                motor_pair.move(motor_pair.PAIR_1, drift,
                                velocity=true_speed * -1)
            else:
                motor_pair.move(motor_pair.PAIR_1, drift * -1,
                                velocity=true_speed)

    # stops the motors after they are out of the while loop
    motor_pair.stop(motor_pair.PAIR_1)
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
    g_yaw = tgtYaw# Save the target yaw into our Global yaw.
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


g_yaw = 0# Define the global variable at the module level


async def readyForRun():
    global g_yaw
    g_yaw = 0
    motion_sensor.reset_yaw(0)


async def speedyRun_1():
    """ This is speedyRun1 """
    print("This is speedyRun1")

    # start collecting octopus
    await straight(Direction.BACKWARD, 100, 900)# moving towards the octopus
    await turn(Direction.LEFT, 45, 600)# turning towards the octopos
    await straight(Direction.BACKWARD, 620, 400)
    # octopus falls down, we move towards the next mission
    await straight(Direction.FORWARD, 100, 800)

    # start going towards angler fish
    await turn(Direction.LEFT, 27, 600)
    # move forward towards the angular mission
    await straight(Direction.BACKWARD, 770, 800)
    await turn(Direction.RIGHT, 16, 600)
    # solve the angular mission
    await straight(Direction.BACKWARD, 310, 700, justGoFast=1)
    await turn(Direction.LEFT, 15, 500)# secure the angular mission

    # drop off octopus
    await turn(Direction.RIGHT, 80, 600)
    await attachmentMotor_async(port.E, 550, 250, Direction.DOWN)

    # move towards the artifical habitat

    await turn(Direction.LEFT, 60, 600)
    await straight(Direction.FORWARD, 900, 500)
    await turn(Direction.LEFT, 60, 500)

    # ram into the artificial habitat
    await straight(Direction.BACKWARD, 170, 700)

    # # start folding the artificial habitat
    await turn(Direction.RIGHT, 55, 1050)
    await straight(Direction.BACKWARD, 440, 900)

    # moving backward to align the robot to the reef
    await straight(Direction.FORWARD, 200, 900)
    await turn(Direction.LEFT, 35, 1050)
    await straight(Direction.FORWARD, 100, 500)

    # start lifting the artificial habitat
    await attachmentMotor_async(port.B, 425, 1000, Direction.DOWN)
    await straight(Direction.BACKWARD, 250, 900)
    await attachmentMotor_async(port.B, 450, 1050, Direction.UP)

    # #get ready for second flick
    await attachmentMotor_async(port.B, 475, 1050, Direction.DOWN)
    await straight(Direction.BACKWARD, 100, 900)
    await attachmentMotor_async(port.B, 425, 1050, Direction.UP)

    # move towards the shipping lane
    # change this line of code if the flicker does not align well with shiping lane mission
    await straight(Direction.FORWARD, 620, 700)
    await turn(Direction.RIGHT, 135, 500)

    # start solving the shipping lane
    attachmentMotor(port.B, 450, 800, Direction.DOWN)
    await straight(Direction.BACKWARD, 720, 900)
    await attachmentMotor_async(port.B, 450, 800, Direction.UP)

    # #pick up the kelp sample
    await straight(Direction.FORWARD, 100, 700)
    await turn(Direction.LEFT, 30, 500)
    await straight(Direction.BACKWARD, 150, 700)
    await attachmentMotor_async(port.E, 450, 450, Direction.UP)
    await turn(Direction.LEFT, 15, 750)

    # go home
    await straight(Direction.FORWARD, 1100, 1000)


async def backupRun1():
    # go towards shipping lanes
    await attachmentMotor_async(port.E, 450, 450, Direction.DOWN)
    attachmentMotor(port.B, 450, 800, Direction.DOWN)
    await straight(Direction.BACKWARD, 800, 800)
    await turn(Direction.RIGHT, 45, 500)
    await straight(Direction.BACKWARD, 200, 500)
    await attachmentMotor_async(port.B, 450, 800, Direction.UP)

    # #pick up the kelp sample
    await straight(Direction.FORWARD, 100, 700)
    await turn(Direction.LEFT, 30, 500)
    await straight(Direction.BACKWARD, 150, 700)
    await attachmentMotor_async(port.E, 450, 600, Direction.UP)
    await turn(Direction.LEFT, 15, 750)

    await straight(Direction.FORWARD, 1100, 1100)


async def Run_3():
    """ This is Run3 ARYAN"""
    # Scuba diver arm Lift up
    # attachmentMotor(Arm.LEFT, 200, 300, Direction.DOWN)
    attachmentMotor(Arm.RIGHT, 15, 300, Direction.UP)

    # Bring Scuba diver arm to position
    # await attachmentMotor_async(Arm.RIGHT, 200, 500, Direction.DOWN)

    # Start moving forward from the Launch Area
    await straight(Direction.BACKWARD, 280, 150)

    # Lift Shark Arm to hit Krakens Treasure puller
    attachmentMotor(Arm.LEFT, 200, 500, Direction.DOWN)

    # Turn Left to bring Krakens Treasure Arm forward
    await turn(Direction.LEFT, 0, 550, -150)

    # Now the front face forward, Go forward towards the Krakens Treasure at an angle
    await straight(Direction.FORWARD, 520, 300)

    # Turn Perfectly towards Krakens Treasure Mission
    await turn(Direction.RIGHT, 0, 500, -90)

    # Ram fast into the Krakens Treasure Mission capture Box
    await straight(Direction.FORWARD, 550, 1000, justGoFast=1)

    # Now backup a bit
    await straight(Direction.BACKWARD, 270, 150)# from 280

    # Now turn my Motorized attachment Towards North Which is facing 0
    await turn(Direction.RIGHT, 90, 500)

    # Now go straight with motorized attachment facing forward
    await straight(Direction.BACKWARD, 415, 400)# was 400 dis

    # Bring the Scuba Diver Arm to position
    await attachmentMotor_async(Arm.RIGHT, 160, 300, Direction.DOWN)

    await turn(Direction.LEFT, 90, 500)# Now turn towards Scuba Diver

    # Go towards suba diver        110 to 115
    await straight(Direction.BACKWARD, 115, 100)

    # Lift the scuba Diver
    await attachmentMotor_async(Arm.RIGHT, 100, 90, Direction.UP)

    # await straight(Direction.FORWARD, 200, 400)# Go back a bit

    # Ram into the coral mission fast
    # await straight(Direction.BACKWARD, 260, 1000, justGoFast=1)

    # Ram into the coral mission fast
    await straight(Direction.BACKWARD, 80, 1000, justGoFast=1)

    # Now After raming, Go back
    await straight(Direction.FORWARD, 75, 200)

    await turn(Direction.RIGHT, 0, 400, -60)# Turn towards Shark Mission 30
    await straight(Direction.BACKWARD, 22, 200)# Go towards Shark to hit

    # Hit the SHark Lever
    await attachmentMotor_async(Arm.LEFT, 300, 1100, Direction.UP)

    # Lift the Shark Arm After Shark Mission
    attachmentMotor(Arm.LEFT, 200, 500, Direction.DOWN)

    await straight(Direction.FORWARD, 200, 450, 1000)# Back up a bit

    await turn(Direction.RIGHT, 70, 300)    # Turn Towards Coral bud nursery
    # Go forward towards coral bud nursery
    await straight(Direction.BACKWARD, 235, 250, 1000)
    # Hit the coral nursery
    await attachmentMotor_async(Arm.LEFT, 300, 1100, Direction.UP)
    # Lift the ARM after hitting the coral nursery
    await attachmentMotor_async(Arm.LEFT, 300, 1100, Direction.DOWN)
    await straight(Direction.FORWARD, 20, 100)# Back up a bit
    # Turn Little more Towards Coral bud nursery to drop Scuba diver
    await turn(Direction.RIGHT, 30, 500)
    # Drop the Scuba diver
    await attachmentMotor_async(Arm.RIGHT, 110, 50, Direction.DOWN)

    # Return home
    # Turn Little more Towards Coral bud nursery to drop Scuba diver
    await turn(Direction.LEFT, 20, 500)
    await straight(Direction.FORWARD, 2000, 1000, justGoFast=1)


async def Run_2_backup():
    """ This is Run2 """
    attachmentMotor(port.E, 200, 400, Direction.UP)
    await straight(Direction.FORWARD, 50, 900)
    #turn left to pick up the first krill
    await turn(Direction.LEFT, 20, 750)
    await straight(Direction.FORWARD, 500, 600, justGoFast=1)

    #turn right to pick up the coral
    await turn(Direction.RIGHT, 30, 750)
    await straight(Direction.FORWARD, 650, 600, justGoFast=1) # This will pick up the krill near the whale

    #picking up the last krill
    await straight(Direction.BACKWARD, 390, 600, justGoFast=1)

    await turn(Direction.LEFT, 35, 750)
    await straight(Direction.FORWARD, 300, 600, justGoFast=1)

    await turn(Direction.LEFT, 50, 750) #move more left getting parallel to sonar discovery
    attachmentMotor(port.E, 200, 400, Direction.DOWN)

    # # start turning the motor for attachment
    # await attachmentMotor_async(port.E, 270, 400, Direction.DOWN)
    # # ======#
    await straight(Direction.FORWARD, 720, 600, justGoFast=1) # going towards the mission
    await turn(Direction.LEFT, 15, 800)
    await straight(Direction.FORWARD, 860, 600, justGoFast=1)

    await turn(Direction.LEFT, 13, 650)

    await straight(Direction.FORWARD, 580, 600)
    await turn(Direction.LEFT, 60, 600)
    await straight(Direction.FORWARD, 1100, 600)
    await turn(Direction.RIGHT, 90, 900)
    await straight(Direction.FORWARD, 450, 600, justGoFast=1)


async def Run_2():
    """ This is Run2 """
    attachmentMotor(port.E, 200, 400, Direction.UP)
    await straight(Direction.FORWARD, 50, 600, justGoFast=1)
    # turn left to pick up the first krill
    await turn(Direction.LEFT, 20, 750)
    await straight(Direction.FORWARD, 500, 600, justGoFast=1)

    # turn right to pick up the coral
    await turn(Direction.RIGHT, 30, 750)
    # This will pick up the krill near the whale
    await straight(Direction.FORWARD, 650, 600, justGoFast=1)

    # picking up the last krill
    await straight(Direction.BACKWARD, 390, 600, justGoFast=1)

    await turn(Direction.LEFT, 30, 750)
    await straight(Direction.FORWARD, 340, 900)

    # this impacts the turn angle effecting the sonar discovery mission.
    await turn(Direction.RIGHT, 167, 750)

    # start turning the motor for attachment
    await attachmentMotor_async(port.E, 270, 400, Direction.DOWN)
    # ======#
    await straight(Direction.BACKWARD, 420, 900)# going towards the mission
    # turning the arm a little bit towards the other side
    await attachmentMotor_async(port.E, 60, 500, Direction.UP)
    await runloop.sleep_ms(500)
    await straight(Direction.FORWARD, 420, 900)

    # ==== Sonar discovery mission end

    await straight(Direction.BACKWARD, 100, 900)
    # make the hammer straight
    await attachmentMotor_async(port.E, 45, 400, Direction.DOWN)
    await turn(Direction.LEFT, 40, 600)# line up with submersible

    # # move forward towards the submersible
    await straight(Direction.BACKWARD, 300, 1000)
    await attachmentMotor_async(port.E, 60, 400, Direction.UP)
    await runloop.sleep_ms(500)
    # HIT THE SUBMERSIBLE
    await straight(Direction.BACKWARD, 550, 750, justGoFast=1)

    await straight(Direction.FORWARD, 150, 900)
    await attachmentMotor_async(port.E, 120, 400, Direction.DOWN)

    # =======#
    await straight(Direction.BACKWARD, 300, 900)
    await turn(Direction.LEFT, 10, 600)
    await straight(Direction.BACKWARD, 600, 1000)# cross the submersible
    await turn(Direction.LEFT, 25, 600)

    await straight(Direction.BACKWARD, 100, 300)
    # close the latch using the bar
    await attachmentMotor_async(port.E, 75, 600, Direction.UP)
    await straight(Direction.BACKWARD, 760, 800)
    await attachmentMotor_async(port.E, 195, 600, Direction.UP)
    await turn(Direction.LEFT, 65, 350)
    await straight(Direction.BACKWARD, 1200, 800, justGoFast=1)
    await turn(Direction.LEFT, 90, 900)
    await straight(Direction.FORWARD, 450, 600, justGoFast=1)


async def Run_5_2():
    """ This is JUST BOAT """
    ## ----------------##
    ## HOME1 TO WHALE ##
    ## ----------------##

    print("This is Run5_2")
    # moving towards HOME1 - STEP 2
    await straight(Direction.FORWARD, 300, 40)
    await straight(Direction.BACKWARD, 500, 700)


async def Run_5_3():
    """ This is Run5_3 """
    """ FEED THE WHALE """

    print("This is Run5_3")

    await straight(Direction.BACKWARD, 500, 800)# Moving Straight
    await turn(Direction.RIGHT, 0, 500, 90)# TURN 1
    # Moving Straight to HOME1
    await straight(Direction.BACKWARD, 2850, 1000, accel=1000)
    await turn(Direction.LEFT, 0, 500, 0)# TURN 2
    # moving towards the whale mission #500
    await straight(Direction.BACKWARD, 750, 500)
    await turn(Direction.RIGHT, 0, 300, 45)# turn 90 wrt to start point
    # moving towards the whale mission #500
    await straight(Direction.BACKWARD, 410, 500)
    # Feed the WHALE
    await attachmentMotor_async(Arm.RIGHT, 1050, 500, Direction.UP)
    await runloop.sleep_ms(200)
    # moving away the whale mission
    await straight(Direction.FORWARD, 200, 200)

# Actual 1000 degrees = 21.377 inches


async def main():
    """Main function"""
    global g_yaw
    g_yaw = 0

    motion_sensor.reset_yaw(0)
    motor_pair.pair(motor_pair.PAIR_1, DriverMotor.LEFT, DriverMotor.RIGHT)
    a=time.ticks_us()
    await straight(Direction.FORWARD, 300, 1000)
    await turn (Direction.LEFT, 90, 1000)
    await straight(Direction.FORWARD, 300, 1000)
    b = time.ticks_us()
    print ("Time it took is " , b-a)
    return
    #1.305725

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
