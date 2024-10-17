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


class WorkerMotor:
    """Which Worker motor"""
    LEFT = port.D
    RIGHT = port.F


def get_drift(tgt_yaw):
    """Get drift gives how much you are drifted from your tgt_yaw angle."""
    c_yaw = get_yaw()
    # When robot is close to 360, it can drift to 2 or drift to 359
    # This will take into consideration all the cases.
    if tgt_yaw > 270 and c_yaw < 90:  # This condition is when your target yaw is in Q4 and Current yaw is in Q1
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


async def straight(speed: int, distance: int, direction: int):
    """ Drives straight"""
    global g_yaw
    tgtYaw = g_yaw

    # resets the relative position of one of the wheels
    motor.reset_relative_position(port.B, 0)
    drift = get_drift(tgtYaw)

    while distance > abs(motor.relative_position(port.B)):
        # sets the return value of the tuple to a tuple, so we can pull a specific value from it
        drift = get_drift(tgtYaw)

        if direction == Direction.BACKWARD:
            motor_pair.move(motor_pair.PAIR_1, drift,
                            velocity=speed * -1, acceleration=1000)
        else:
            motor_pair.move(motor_pair.PAIR_1, drift * -1,
                            velocity=speed, acceleration=1000)

    # stops the motors after they are out of the while loop
    motor_pair.stop(motor_pair.PAIR_1)
    await runloop.sleep_ms(400)


async def turn(direction: int, degrees: int, speed: int, targetYaw: int = -500):
    """Direction is Direction.RIGHT or Direction.LEFT
    degrees: Amount of degrees to turn
    speed: speed at which to turn
    """
    global g_yaw
    tgtYaw = g_yaw
    tgtSpeed = speed
    origDiff = abs(degrees)
    minSpeed = 75

    if targetYaw >= -360 and targetYaw < 0:
        targetYaw = 360 + targetYaw

    if direction == Direction.RIGHT:
        if targetYaw == -500:
            tgtYaw = (g_yaw + degrees) % 360
        else:
            tgtYaw = targetYaw
            origDiff = angleDiff(targetYaw)

        while (agdiff := angleDiff(tgtYaw)) > 0:
            tgtSpeed = int(max((agdiff/origDiff) * speed, minSpeed))
            # We need to turn both wheels backwards to turn Right
            motor.run(port.A, tgtSpeed * Direction.RIGHT * -1)
            motor.run(port.B, tgtSpeed * Direction.RIGHT * -1)
    elif direction == Direction.LEFT:
        if targetYaw == -500:
            tgtYaw = (g_yaw - degrees + 360) % 360
        else:
            tgtYaw = targetYaw
            origDiff = angleDiff(targetYaw)

        # Angle diff gives us the difference between my current yaw and the target yaw
        while (agdiff := angleDiff(tgtYaw)) > 0:
            tgtSpeed = int(max((agdiff/origDiff) * speed, minSpeed))
            motor.run(port.A, tgtSpeed)
            motor.run(port.B, tgtSpeed)

    motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)
    g_yaw = tgtYaw  # Save the target yaw into our Global yaw.
    await runloop.sleep_ms(400)


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


async def run_1():
    """Run 1"""
    # await straight(400,825,1)
    # motor.run_for_degrees(port.D,300,1000) # puts the sonar discovery attachment back
    # await turn(Direction.RIGHT,45,200) #turn towards the boat mission
    # await straight(300,200,1)#puts attachment on boat
    await straight(500, 200, 1)
    await turn(Direction.LEFT, 45, 200)
    await straight(500, 650, 1)
    await straight(500, 100, -1)
    await turn(Direction.RIGHT, 0, 100, 60)
    await straight(500, 450, 1)
    await turn(Direction.LEFT, 20, 200)
    await straight(500, 100, 1)
    # this and the line under this does the boat mission
    await motor.run_for_degrees(port.F, 800, 1000)
    await turn(Direction.RIGHT, 15, 200)
    await straight(500, 395, -1)  # backs up from the boat mission
    await turn(Direction.LEFT, 55, 200)  # turns to get coral
    await straight(800, 325, 1)  # collects the coral
    await turn(Direction.LEFT, 20, 200)  # turns to align with second krill
    await straight(1000, 240, 1)  # collect second krill
    await turn(Direction.RIGHT, 45, 200)  # turn to align with 3rd krill
    await straight(1000, 250, 1)  # goes to collect third krill
    await straight(300, 250, -1)  # backs up from whale
    # turns to align with sonar discovery mission
    await turn(Direction.RIGHT, 100, 200)
    await straight(500, 650, -1)  # reaches sonar discovery
    await turn(Direction.LEFT, 15, 200)  # aligns robot with sonar discovery
    await motor.run_for_degrees(port.D, 2000, 1000)  # does sonar discovery
    await straight(500, 100, -1)
    await turn(Direction.RIGHT, 20, 200)
    await straight(500, 800, 1)
    await turn(Direction.RIGHT, 30, 200)
    await straight(800, 700, 1)


async def run_2():
    """RUN 2"""
    # attachmentMotor(WorkerMotor.LEFT, 300, 150, Direction.LEFT)
    # attachmentMotor(WorkerMotor.RIGHT, 300, 220, Direction.LEFT)
    await straight(450, 460, Direction.FORWARD)  # We start sideways
    await turn(Direction.RIGHT, 0, 300, 40)
    # Go towards dropping the Squid
    await straight(450, 1400, Direction.FORWARD)
    # Turn towards Red squid drop off location
    await turn(Direction.RIGHT, 0, 300, 90)
    await straight(450, 120, Direction.FORWARD)  # Drop of Red Squid
    await straight(450, 145, Direction.BACKWARD)  # Backup

    await turn(Direction.LEFT, 0, 200, 40)  # Angler fish mission
    # attachmentMotor(WorkerMotor.LEFT, 300, 150, Direction.RIGHT)
    await straight(800, 790, Direction.FORWARD)  # Angler fish mission 650
    await turn(Direction.RIGHT, 0, 200, 65)

    # Go straight to pick up the Sea bed sample
    await straight(350, 75, Direction.FORWARD)
    # This will pick up the sea bed sample
    await motor.run_for_degrees(port.D, -300, 150)
    # Backup a bit to after picking up the sea bed sample
    await straight(150, 65, Direction.BACKWARD)

    # Turn to 0 degrees towards Water sample
    await turn(Direction.LEFT, 0, 400, 0)
    await straight(150, 290, Direction.FORWARD)  # Go towards the Water Sample
    # Pick up water sample
    await attachmentMotor_async(WorkerMotor.LEFT, 140, 200, Direction.RIGHT)

    await straight(450, 357, Direction.FORWARD)  # Go towards the Coral Nursery
    # Lift the tire hammer
    await attachmentMotor_async(WorkerMotor.RIGHT, 65, 120, Direction.LEFT)
    attachmentMotor(WorkerMotor.LEFT, 65, 120, Direction.RIGHT)
    # Coral nursery Push down
    await attachmentMotor_async(WorkerMotor.RIGHT, 800, 2000, Direction.RIGHT)
    # Coral nursery Lift up
    await attachmentMotor_async(WorkerMotor.RIGHT, 100, 200, Direction.LEFT)
    attachmentMotor(WorkerMotor.RIGHT, 300, 350,
                    Direction.LEFT)  # Lift up the hammer
    await straight(285, 365, Direction.FORWARD)    # Move towards Shark
    runloop.sleep_ms(200)
    # Hit the Shark button
    await attachmentMotor_async(WorkerMotor.RIGHT, 600, 2000, Direction.RIGHT)

    # Lifting the arm off the Shark button
    attachmentMotor(WorkerMotor.RIGHT, 600, 200, Direction.LEFT)
    runloop.sleep_ms(300)
    # Step back at coral buds / Shark mission
    await straight(300, 400, Direction.BACKWARD)
    await turn(Direction.LEFT, 50, 200)  # Turn left towards Home
    await straight(800, 1680, Direction.FORWARD)  # Drive to Home


async def run_3():
    """Run 3"""
    await straight(900, 550, Direction.BACKWARD)
    await turn(Direction.RIGHT, 45, 130)
    await straight(900, 350, Direction.BACKWARD)
    await turn(Direction.RIGHT, 45, 130)
    await straight(900, 800, Direction.BACKWARD)
    await straight(500, 250, Direction.FORWARD)

    await turn(Direction.LEFT, 135, 350)
    # end of kracken

    # move towards artifical habitat
    await straight(900, 1055, Direction.FORWARD)
    await turn(Direction.LEFT, 45, 150)

    # Artificial Habitat
    await straight(300, 330, Direction.FORWARD)
    await motor.run_for_degrees(port.F, -1500, 600)
    motor.run_for_degrees(port.F, 1100, 113)
    await straight(100, 400, Direction.BACKWARD)
    await turn(Direction.LEFT, 135, 150)
    await straight(400, 300, Direction.FORWARD)
    await turn(Direction.RIGHT, 90, 200)
    await straight(200, 320, Direction.FORWARD)

    # pick up trident
    
    await motor.run_for_degrees(port.D, -4000, 1100)
    await straight(50, 25, Direction.BACKWARD)
    runloop.sleep_ms(5000)
    await motor.run_for_degrees(port.D, 200, 200)
    await motor.run_for_degrees(port.D, 200, 200)
    await straight(500, 325, Direction.BACKWARD)

    # go back home
    await turn(Direction.RIGHT, 45, 100)
    await straight(700, 1000, Direction.BACKWARD)


async def run_4():
    """ Run 4 """
    await straight(800, 1100, Direction.FORWARD)  # start moving
    await turn(Direction.RIGHT, 90, 100)  # take first turn
    await straight(800, 65, Direction.FORWARD)  # got toward the boat
    # await turn(Direction.RIGHT, 90, 100)
    await motor.run_for_degrees(port.D, 2300, 4000)  # Drop the stuff was 2300
    await motor.run_for_degrees(port.D, -2300, 4000)  # w move te box up
    await straight(400, 65, Direction.FORWARD)
    # await straight(800, 200, Direction.FORWARD)
    await turn(Direction.LEFT, 90, 100)
    # was 400 pushes the boat forward
    await straight(600, 450, Direction.FORWARD)
    # await turn(Direction.LEFT, 20, 200)
    # print("driving straight")
    await straight(300, 125, Direction.BACKWARD)  # Moves backMowas. 150
    await turn(Direction.LEFT, 90, 100)
    await straight(200, 400, Direction.BACKWARD)  # Original was 400
    await turn(Direction.LEFT, 90, 100)
    # await turn(Direction.LEFT, 55, 10)
    # 875 worked Vamshi:
    # SRI NOTE: -> This is the line that will push the boat into latch,
    # may need adjustment to this line. FYIoriginal entries: (800,900)
    await straight(800, 875, Direction.BACKWARD)
    # await straight(800, 1000, Direction.FORWARD)
    # code for robot to come back
    await straight(800, 500, Direction.FORWARD)  # 500
    # print("At line 148")
    await turn(Direction.RIGHT, 30, 100)
    await straight(800, 900, Direction.FORWARD)
    # print("about to return")
    await turn(Direction.LEFT, 45, 300)  # was 30
    # 800, 900 :SRI NOTE THIS -> Vamshi: change to 500, 800 if doesnt work
    await straight(800, 600, Direction.FORWARD)


async def run_5():
    """Run 5"""
    # attachmentMotor(WorkerMotor.LEFT, 300, 150, Direction.LEFT)
    await straight(450, 3100, Direction.FORWARD)  # Going straight
    await turn(Direction.LEFT, 0, 110, -90)  # turns
    # await straight (450, 650, Direction.FORWARD)
    # goes to the mission but not purfectly in there
    await straight(450, 630, Direction.FORWARD)
    await turn(Direction.RIGHT, 0, 110, -45)  # turns to the mission
    # await straight (450, 550, Direction.FORWARD)
    # goes into the whale pushy thing that makes the whales mouth open
    await straight(450, 550, Direction.FORWARD)
    # Makes the krill go into the whales mouth
    await attachmentMotor_async(WorkerMotor.LEFT, 1300, 400, Direction.RIGHT)
    # await attachmentMotor_async(WorkerMotor.LEFT, 1300, 400, Direction.LEFT)
    # Run for Last Mission
    await straight(350, 700, Direction.BACKWARD)  # Backup from Whale
    await turn(Direction.LEFT, 92, 98)  # Change Direction
    await straight(320, 880, Direction.FORWARD)  # Head to Final Station
    # await straight (320, 150, Direction.FORWARD)
    # attachmentMotor(WorkerMotor.RIGHT, 320, 150, Direction.LEFT) #Make the bar go up
    # Need this backward to help the bar
    await straight(320, 200, Direction.BACKWARD)


async def run_5_2():
    """Speed version of Run5"""
    # attachmentMotor(WorkerMotor.LEFT, 300, 150, Direction.LEFT)
    await straight(800, 3100, Direction.FORWARD)  # Going straight
    await turn(Direction.LEFT, 0, 110, -90)  # turns
    # await straight (450, 650, Direction.FORWARD)
    # goes to the mission but not purfectly in there
    await straight(800, 630, Direction.FORWARD)
    await turn(Direction.RIGHT, 0, 200, -45)  # turns to the mission
    # await straight (450, 550, Direction.FORWARD)
    # goes into the whale pushy thing that makes the whales mouth open
    await straight(500, 515, Direction.FORWARD)
    # Makes the krill go into the whales mouth
    await attachmentMotor_async(WorkerMotor.LEFT, 1350, 450, Direction.RIGHT)
    # await attachmentMotor_async(WorkerMotor.LEFT, 1300, 400, Direction.LEFT)
    # Run for Last Mission
    await straight(800, 690, Direction.BACKWARD)  # Backup from Whale
    await turn(Direction.LEFT, 91, 200)  # Change Direction
    await straight(500, 860, Direction.FORWARD)  # Head to Final Station
    # await straight (320, 150, Direction.FORWARD)
    # attachmentMotor(WorkerMotor.RIGHT, 320, 150, Direction.LEFT) #Make the bar go up
    # Need this backward to help the bar
    await straight(320, 200, Direction.BACKWARD)


g_yaw = 0  # Define the global variable at the module level


async def main():
    """Main function"""
    global g_yaw
    g_yaw = 0

    motion_sensor.reset_yaw(0)
    motor_pair.pair(motor_pair.PAIR_1, port.B, port.A)

    while True:
        color_detected = color_sensor.color(port.C)  # Read sensor value once
        if color_detected is color.BLUE:
            await run_1()
        if color_detected is color.RED:
            await run_2()
        elif color_detected is color.GREEN:
            await run_3()
        elif color_detected is color.YELLOW:
            await run_4()
        elif color_detected is color.PURPLE:
            await run_5_2()

runloop.run(main())
