from asyncio import sleep
from hub import port
from hub import motion_sensor
import motor
import runloop
import motor_pair
import asyncio

# left motor is connected to port A and right motor is connected to port B

# Aliases for Going Left right back and forward


class Direction:
    LEFT = -1
    BACKWARD = -1
    FORWARD = 1
    RIGHT = 1
    UP = 1
    DOWN = -1


class WorkerMotor:
    LEFT = port.D
    RIGHT = port.F


# Get drift gives how much you are drifted from your tgt_yaw angle.

def get_drift(tgt_yaw):
    c_yaw = get_yaw()
    # When robot is close to 360, it can drift to 2 or drift to 359
    # This will take into consideration all the cases.
    if tgt_yaw > 270 and c_yaw < 90:# This condition is when your target yaw is in Q4 and Current yaw is in Q1
        drift = 360 - tgt_yaw + c_yaw
    else:
        drift = c_yaw - tgt_yaw

    return drift


"""
Gives current yaw in between 0 to 359
As our Motor Left is connected to A and Right is Connected to B
When turning right we get negative Yaw values
"""


def get_yaw() -> int:
    yaw = motion_sensor.tilt_angles()[0]
    # Get Remainder, Yaw angle after one full circle.
    yaw = (round(yaw/10 * -1) + 360) % 360
    return yaw


"""
Give the angle difference between current yaw and target yaw
There are 4 Cases Here:
    1. When turnnig Right and When current yaw is 350 and Target yaw is 30
    2. When turning Left and When current yaw is 10 and Target yaw is 350
    3. When turning Right and When target yaw is 90 and current yaw is 30
    4. When turning Left and When target yaw is 270 and current yaw is 350
"""


def angleDiff(tgt_yaw):
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



#Takes straight



async def straight(speed: int, distance: int, direction: int):
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


"""
direction is Direction.RIGHT or Direction.LEFT
degrees: Amount of degrees to turn
speed: speed at which to turn
"""
async def turn(direction: int, degrees: int, speed: int, targetYaw: int = -500):
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
            tgtSpeed = int (max((agdiff/origDiff) * speed, minSpeed))
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
            tgtSpeed = int (max((agdiff/origDiff) * speed, minSpeed))
            motor.run(port.A, tgtSpeed)
            motor.run(port.B, tgtSpeed)

    motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)
    g_yaw = tgtYaw # Save the target yaw into our Global yaw.
    await runloop.sleep_ms(400)


"""
workerMotor is AttachMotor.LEFT or AttachMotor.RIGHT
direction is Direction.UP, Direction.RIGHT, Direction.FORWARD all equal to 1
And the others -1
degrees to turn
speed with which the motor should turn.
This function will not wait until the Lift action is performed
"""


def attachmentMotor(workerMotor: int, degrees: int, speed: int, dir: int):
    motor.run_for_degrees(workerMotor, degrees * dir, speed)

# This function will wait until the Lift action is performed


async def attachmentMotor_async(workerMotor: int, degrees: int, speed: int, dir: int):
    await motor.run_for_degrees(workerMotor, degrees * dir, speed)


async def Run_2_reverted():
    attachmentMotor(WorkerMotor.LEFT, 300, 150, Direction.LEFT)
    attachmentMotor(WorkerMotor.RIGHT, 300, 150, Direction.LEFT)
    await straight(350, 460, Direction.FORWARD)# We start sideways
    await turn(Direction.RIGHT, 38, 110)
    await straight(350, 1400, Direction.FORWARD)
    await turn(Direction.RIGHT, 52, 110)
    await straight(350, 135, Direction.FORWARD)# Drop of Red Squid
    await straight(350, 110, Direction.BACKWARD)# Backup
    await turn(Direction.LEFT, 49, 110)# Angler fish mission
    attachmentMotor(WorkerMotor.LEFT, 300, 150, Direction.RIGHT)# Parallel lift
    await straight(600, 790, Direction.FORWARD)# Angler fish mission
    await turn(Direction.RIGHT, 27, 110)
    # Go straight to pick up the Sea bed sample
    await straight(350, 75, Direction.FORWARD)
    await motor.run_for_degrees(port.D, -300, 150)# This will pick up the sea bed sample
    await straight(350, 65, Direction.FORWARD)
    await turn(Direction.LEFT, 68, 110)
    await straight(450, 415, Direction.FORWARD)
    # Coral nursery Push down
    await attachmentMotor_async(WorkerMotor.RIGHT, 550, 1000, Direction.RIGHT)
    # Coral nursery Lift up
    await attachmentMotor_async(WorkerMotor.RIGHT, 450, 400, Direction.LEFT)
    await straight(450, 430, Direction.FORWARD)    # Move towards Shark
    await turn(Direction.LEFT, 30, 110)
    await straight(300, 100, Direction.FORWARD)
    await attachmentMotor_async(WorkerMotor.RIGHT, 600, 1000, Direction.RIGHT)# Hit the shark
    await attachmentMotor_async(WorkerMotor.RIGHT, 400, 400, Direction.LEFT)    # Lift up the arm
    await turn (Direction.RIGHT, 15, 110)
    await straight (500, 50, Direction.FORWARD)


async def Run_2():
    #attachmentMotor(WorkerMotor.LEFT, 300, 150, Direction.LEFT)
    #attachmentMotor(WorkerMotor.RIGHT, 300, 220, Direction.LEFT)
    await straight(350, 460, Direction.FORWARD)# We start sideways
    await turn(Direction.RIGHT, 0, 300, 40)
    await straight(350, 1400, Direction.FORWARD) #Go towards dropping the Squid
    await turn(Direction.RIGHT, 0, 300, 90) # Turn towards Red squid drop off location
    await straight(350, 135, Direction.FORWARD)# Drop of Red Squid
    await straight(350, 135, Direction.BACKWARD)# Backup
    await turn(Direction.LEFT, 0, 200, 40)# Angler fish mission
    attachmentMotor(WorkerMotor.LEFT, 300, 150, Direction.RIGHT)
    await straight(800, 790, Direction.FORWARD) # Angler fish mission 650
    await turn(Direction.RIGHT, 0, 200, 70)
    await straight(350, 75, Direction.FORWARD)    # Go straight to pick up the Sea bed sample
    await motor.run_for_degrees(port.D, -300, 150) # This will pick up the sea bed sample
    await straight(150, 45, Direction.BACKWARD)    # Backup a bit to after picking up the sea bed sample
    await turn(Direction.LEFT, 0, 400, 0)
    await straight(450, 615, Direction.FORWARD)  # Go towards the Coral Nursery 625
    attachmentMotor (WorkerMotor.LEFT, 65, 120, Direction.RIGHT) 
    await attachmentMotor_async(WorkerMotor.RIGHT, 800, 2000, Direction.RIGHT) # Coral nursery Push down
    await attachmentMotor_async(WorkerMotor.RIGHT, 100, 200, Direction.LEFT) # Coral nursery Lift up
    attachmentMotor(WorkerMotor.RIGHT, 300, 350, Direction.LEFT) # Coral nursery Lift up
    await straight(285, 371, Direction.FORWARD)    # Move towards Shark
    runloop.sleep_ms(200)
    await attachmentMotor_async (WorkerMotor.RIGHT, 600, 2000, Direction.RIGHT)
    await attachmentMotor_async (WorkerMotor.LEFT, 65, 300, Direction.LEFT)
    await straight (300, 100, Direction.BACKWARD)
    '''
    await straight(450, 170, Direction.FORWARD)
    await turn (Direction. LEFT, 17, 110)
    await straight (350, 185, Direction.FORWARD)
    await turn (Direction.RIGHT, 17, 110)
    await straight (150, 100, Direction.FORWARD)
    '''
async def run1():
   # await straight(400,825,1)
   # motor.run_for_degrees(port.D,300,1000) # puts the sonar discovery attachment back
   # await turn(Direction.RIGHT,45,200) #turn towards the boat mission
   # await straight(300,200,1)#puts attachment on boat
    await straight(500,200,1)
    await turn(Direction.LEFT,45,200)
    await straight(500,650,1)
    await straight(500,100,-1)
    await turn(Direction.RIGHT,0,100,60)
    await straight(500,450,1)
    await turn(Direction.LEFT,20,200)
    await straight(500,100,1)
    await motor.run_for_degrees(port.F,800,1000)#this and the line under this does the boat mission
    await turn(Direction.RIGHT,15,200)
    await straight(500,395,-1)# backs up from the boat mission
    await turn(Direction.LEFT,55,200) # turns to get coral
    await straight(800,325,1)# collects the coral
    await turn(Direction.LEFT,20,200)# turns to align with second krill
    await straight(1000,240,1)# collect second krill
    await turn(Direction.RIGHT,45,200)# turn to align with 3rd krill
    await straight(1000,250,1)# goes to collect third krill
    await straight(300,250,-1)#backs up from whale
    await turn(Direction.RIGHT,100,200)#turns to align with sonar discovery mission
    await straight(500,650,-1)#reaches sonar discovery
    await turn(Direction.LEFT,15,200)# aligns robot with sonar discovery
    await motor.run_for_degrees(port.D,2000,1000)# does sonar discovery
    await straight(500,100,-1)
    await turn(Direction.RIGHT,20,200)
    await straight(500,800,1)
    await turn(Direction.RIGHT,30,200)
    await straight(800,700,1)

async def main():
    global g_yaw
    g_yaw = 0
    motion_sensor.reset_yaw(0)
    motor_pair.pair(motor_pair.PAIR_1, port.B, port.A)
    await run1()
    #await motor.run_for_degrees(port.F,1000,1000)
runloop.run(main())

