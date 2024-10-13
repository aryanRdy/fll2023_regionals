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


#Get drift gives how much you are drifted from your tgt_yaw angle.

def get_drift(tgt_yaw):
    c_yaw = get_yaw()
    # When robot is close to 360, it can drift to 2 or drift to 359
    # This will take into consideration all the cases.
    if tgt_yaw > 270 and c_yaw < 90:#This condition is when your target yaw is in Q4 and Current yaw is in Q1
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

"""
Takes straight
"""
async def straight(speed :int , distance :int, direction):
    global g_yaw
    tgtYaw = g_yaw
    # resets the relative position of one of the wheels
    motor.reset_relative_position(port.B, 0)
    drift = get_drift(tgtYaw)

    while distance > abs(motor.relative_position(port.B)):
        # sets the return value of the tuple to a tuple, so we can pull a specific value from it
        drift = get_drift(tgtYaw)
        if direction == Direction.BACKWARD:
            motor_pair.move(motor_pair.PAIR_1, drift, velocity = speed * -1, acceleration = 500)
        else:
            motor_pair.move(motor_pair.PAIR_1, drift * -1, velocity = speed, acceleration = 500)

    # stops the motors after they are out of the while loop
    motor_pair.stop(motor_pair.PAIR_1)

    await runloop.sleep_ms(100)


"""
direction is Direction.RIGHT or Direction.LEFT
degrees: Amount of degrees to turn
speed: speed at which to turn
"""


async def turn(direction, degrees, speed, error=0.0):
    global g_yaw
    tgtYaw = g_yaw
    
    if direction == Direction.RIGHT:
        tgtYaw = (g_yaw + degrees) % 360
        while angleDiff(tgtYaw) > 0:
            tgtYaw = (g_yaw + degrees) % 360
            # We need to turn both wheels backwards to turn Right
            motor.run(port.A, speed * Direction.RIGHT * -1)
            motor.run(port.B, speed * Direction.RIGHT * -1)
    elif direction == Direction.LEFT:
        tgtYaw = (g_yaw - degrees + 360) % 360
        # Angle diff gives us the difference between my current yaw and the target yaw
        while angleDiff(tgtYaw) > 0:
            motor.run(port.A, speed)
            motor.run(port.B, speed)

    motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)
    g_yaw = tgtYaw# Save the target yaw into our Global yaw.
    await runloop.sleep_ms(100)


"""
Arm function parameters: (arm, speed, degrees)
arm is ARM.Right or ARM.Left
"""

"""async def motor(arm, speed, degrees):
    if arm == "arm.Right":
        motor.run_for_degrees(port.F, degrees, speed)
    elif arm == "arm.Left":
        motor.run_for_degrees(port.D, degrees, speed)
    else:
        print("ERR. ONLY EXCEPTS ARM.Right OR ARM.Left for motor() function
"""






async def RUN3():
    
    await straight(500, 550, Direction.BACKWARD) 
    await turn(Direction.RIGHT, 45, 100, 0.6)
    await straight(500, 350, Direction.BACKWARD)
    await turn(Direction.RIGHT, 45, 100, 0.6) 
    await straight(700, 800, Direction.BACKWARD)
    await straight(300, 250, Direction.FORWARD)
    
    await turn(Direction.LEFT, 135, 100, 0.6)
    #end of kracken


    await straight(500, 1000, Direction.FORWARD)  # move towards trident mission
    await turn(Direction.LEFT, 45, 100, 0.6)

    

    

    
    #Artificial Habitat
    await straight(500, 350, Direction.FORWARD)
    await motor.run_for_degrees(port.F, -1400, 600)
    motor.run_for_degrees(port.F, 1100, 100)
    await straight(100, 400, Direction.BACKWARD)
    await straight(300, 350, Direction.BACKWARD)
    await turn(Direction.LEFT, 45, 100, 0.6)
    await straight(300, 500, Direction.FORWARD)

    #pick up trident
    await motor.run_for_degrees(port.D, -4000, 1100) # motor to pick up trident
    await straight(50, 25, Direction.BACKWARD)
    runloop.sleep_ms(5000)
    await motor.run_for_degrees(port.D, 200, 200)
    await motor.run_for_degrees(port.D, 200, 200)
    await straight(500, 350, Direction.BACKWARD)

    #go back home
    await turn(Direction.RIGHT, 45, 100, 0.6)
    await straight(700, 1000, Direction.BACKWARD)
    

async def main():
    global g_yaw
    g_yaw = 0
    motion_sensor.reset_yaw(0)
    motor_pair.pair(motor_pair.PAIR_1, port.B, port.A)
    await RUN3()
    
# This is the starting point
runloop.run(main())

