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
    LEFT = port.B
    RIGHT = port.E


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


async def straight(direction: int, distance: int, speed: int, accel: int = 500, justGoFast: int = 0):
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
        origDiff = angleDiff(targetYaw)

    while (agdiff := angleDiff(tgtYaw)) > 0:
        tgtSpeed = int(max((agdiff/origDiff) * speed, minSpeed))
        # We need to turn both wheels backwards to turn Right
        motor.run(DriverMotor.LEFT, tgtSpeed * direction * -1)
        motor.run(DriverMotor.RIGHT, tgtSpeed * direction * -1)

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


g_yaw = 0  # Define the global variable at the module level


async def readyForRun():
    global g_yaw
    g_yaw = 0
    motion_sensor.reset_yaw(0)


async def speedyRun_1():
    """ This is speedyRun1 """
    print("This is speedyRun1")

    #start collecting octopus
    await straight(Direction.BACKWARD, 100, 900) #moving towards the octopus
    await turn(Direction.LEFT, 45, 600) # turning towards the octopos
    await straight(Direction.BACKWARD, 620, 400)
    await straight(Direction.FORWARD, 100, 800) # octopus falls down, we move towards the next mission

    #start going towards angler fish
    await turn(Direction.LEFT, 26, 600)
    await straight(Direction.BACKWARD, 770, 800) # move forward towards the angular mission
    await turn(Direction.RIGHT, 15 , 600) 
    await straight(Direction.BACKWARD, 310, 900) # solve the angular mission
    await turn(Direction.LEFT, 20, 500) # secular the angular mission

    #drop off octopus
    await turn(Direction.RIGHT, 80, 600)
    await attachmentMotor_async(port.E, 650, 400, Direction.DOWN)

    # move towards the artifical habitat
    
    await turn(Direction.LEFT, 55, 600)
    await straight(Direction.FORWARD, 890, 500)
    await turn(Direction.LEFT, 60, 500)

    # ram into the artificial habitat
    await straight(Direction.BACKWARD, 160, 700)

    #start folding the artificial habitat
    await turn(Direction.RIGHT, 55, 1050)
    await straight(Direction.BACKWARD, 420, 900)

    #moving backward to align the robot to the reef
    await straight(Direction.FORWARD, 160, 900)
    await turn(Direction.LEFT, 35, 1050)
    attachmentMotor(port.B, 500, 1000, Direction.DOWN)
    await straight(Direction.FORWARD, 100, 900)

    # start lifting the artificial habitat
    await straight(Direction.BACKWARD, 325, 900)
    await attachmentMotor_async(port.B, 500, 1050, Direction.UP)

    # #get ready for second flick
    await attachmentMotor_async(port.B, 450, 1000, Direction.DOWN)
    await straight(Direction.BACKWARD, 145, 900)
    await attachmentMotor_async(port.B, 500, 1050, Direction.UP)

    # move towards the shipping lane
    await straight(Direction.FORWARD, 675, 700) # change this line of code if the flicker does not align well with shiping lane mission
    await turn(Direction.RIGHT, 135, 500)

    #start solving the shipping lane
    attachmentMotor(port.B, 450, 800, Direction.DOWN)
    await straight(Direction.BACKWARD, 720, 900)
    await attachmentMotor_async(port.B, 450, 800, Direction.UP)

    # #pick up the kelp sample
    await straight(Direction.FORWARD, 100, 700)
    await turn(Direction.LEFT, 30, 500)
    await straight(Direction.BACKWARD, 150, 700)
    await attachmentMotor_async(port.E, 450, 200, Direction.UP)
    await turn(Direction.LEFT, 15, 750)

    #go home
    await straight(Direction.FORWARD, 1100, 1000)


async def backupRun1():


    #go towards shipping lanes
    await straight(Direction.BACKWARD, 800, 500)
    await turn(Direction.RIGHT, 45, 500)
    await straight(Direction.BACKWARD, 200, 500)
    await attachmentMotor_async(port.B, 450, 800, Direction.UP)

    # #pick up the kelp sample
    await straight(Direction.FORWARD, 100, 500)
    await turn(Direction.LEFT, 15, 200)
    await straight(Direction.BACKWARD, 150, 500)
    attachmentMotor(port.E, 450, 800, Direction.UP)
    await turn(Direction.LEFT, 15, 200)

    await straight(Direction.FORWARD, 1100, 500)


async def Run_3():
    """ This is Run3 ARYAN"""
    # Scuba diver arm Lift up
    #attachmentMotor(Arm.LEFT, 200, 300, Direction.DOWN)
    attachmentMotor(Arm.LEFT, 15, 300, Direction.UP)


    # Bring Scuba diver arm to position
    #await attachmentMotor_async(Arm.LEFT, 200, 500, Direction.DOWN)

    # Start moving forward from the Launch Area
    await straight(Direction.BACKWARD, 280, 150)

    # Lift Shark Arm to hit Krakens Treasure puller
    attachmentMotor(Arm.RIGHT, 200, 500, Direction.DOWN)

    # Turn Left to bring Krakens Treasure Arm forward
    await turn(Direction.LEFT, 0, 550, -150)

    # Now the front face forward, Go forward towards the Krakens Treasure at an angle
    await straight(Direction.FORWARD, 520, 300)

    # Turn Perfectly towards Krakens Treasure Mission
    await turn(Direction.RIGHT, 0, 500, -90)

    # Ram fast into the Krakens Treasure Mission capture Box
    await straight(Direction.FORWARD, 550, 1000, justGoFast=1)

    # Now backup a bit
    await straight(Direction.BACKWARD, 270, 150)  #from 280

    # Now turn my Motorized attachment Towards North Which is facing 0
    await turn(Direction.RIGHT, 90, 500)

    # Now go straight with motorized attachment facing forward
    await straight(Direction.BACKWARD, 415, 400)   #was 400 dis

    # Bring the Scuba Diver Arm to position
    await attachmentMotor_async(Arm.LEFT, 160, 300, Direction.DOWN)

    await turn(Direction.LEFT, 90, 500)# Now turn towards Scuba Diver

    await straight(Direction.BACKWARD, 115, 100)# Go towards suba diver         110 to 115

    # Lift the scuba Diver
    await attachmentMotor_async(Arm.LEFT, 100, 90, Direction.UP)
    
    #await straight(Direction.FORWARD, 200, 400)# Go back a bit

    # Ram into the coral mission fast
    #await straight(Direction.BACKWARD, 260, 1000, justGoFast=1)
    
    # Ram into the coral mission fast
    await straight(Direction.BACKWARD, 80, 1000, justGoFast=1)

    # Now After raming, Go back
    await straight(Direction.FORWARD, 75, 200)

    await turn(Direction.RIGHT, 0, 400, -60)# Turn towards Shark Mission 30
    await straight(Direction.BACKWARD, 22, 200)  # Go towards Shark to hit

    # Hit the SHark Lever
    await attachmentMotor_async(Arm.RIGHT, 300, 1100, Direction.UP)

    # Lift the Shark Arm After Shark Mission
    attachmentMotor(Arm.RIGHT, 200, 500, Direction.DOWN)

    await straight(Direction.FORWARD, 200, 450, 1000)# Back up a bit

    await turn(Direction.RIGHT, 70, 300)    # Turn Towards Coral bud nursery
    # Go forward towards coral bud nursery
    await straight(Direction.BACKWARD, 235, 250, 1000)
    await attachmentMotor_async(Arm.RIGHT, 300, 1100, Direction.UP)  #Hit the coral nursery
    await attachmentMotor_async(Arm.RIGHT, 300, 1100, Direction.DOWN)#Lift the ARM after hitting the coral nursery
    await straight(Direction.FORWARD, 20, 100)# Back up a bit
    await turn(Direction.RIGHT, 30, 500)    # Turn Little more Towards Coral bud nursery to drop Scuba diver
    # Drop the Scuba diver
    await attachmentMotor_async(Arm.LEFT, 110, 50, Direction.DOWN)

    #Return home
    await turn(Direction.LEFT, 20, 500)    # Turn Little more Towards Coral bud nursery to drop Scuba diver
    await straight(Direction.FORWARD, 2000, 1000, justGoFast=1)


async def Run_2():
    """ This is Run2 """

    await straight(Direction.FORWARD, 50, 900)
    #turn left to pick up the first krill
    await turn(Direction.LEFT, 20, 600)
    await straight(Direction.FORWARD, 500, 900)

    #turn right to pick up the coral
    await turn(Direction.RIGHT, 30, 600)
    await straight(Direction.FORWARD, 650, 900) # This will pick up the krill near the whale

    #picking up the last krill
    await straight(Direction.BACKWARD, 390, 900)

    await turn(Direction.LEFT, 30, 600)
    await straight(Direction.FORWARD, 330, 900)

    await turn(Direction.RIGHT, 170, 600) # this impacts the turn angle effecting the sonar discovery mission.

    # start turning the motor for attachment
    await attachmentMotor_async(port.E, 270, 400, Direction.DOWN)
    # ======#
    await runloop.sleep_ms(1000)
    await straight(Direction.BACKWARD, 420, 900) # going towards the mission
    await attachmentMotor_async(port.E, 60, 400, Direction.UP) # turning the arm a little bit towards the other side
    await runloop.sleep_ms(750)
    await straight(Direction.FORWARD, 420, 900)

    # ==== Sonar discovery mission end


    await straight(Direction.BACKWARD, 100, 900)
    await attachmentMotor_async(port.E, 45, 400, Direction.DOWN)  # make the hammer straight
    await turn(Direction.LEFT, 45, 300) #line up with submersible
    #await attachmentMotor_async(port.E, 67, 400, Direction.UP)
    

    # # move forward towards the submersible
    await straight(Direction.BACKWARD, 300, 900)
    await attachmentMotor_async(port.E, 60, 400, Direction.UP)
    await runloop.sleep_ms(500)
    await straight(Direction.BACKWARD, 550, 900) # HIT THE SUBMERSIBLE

    await straight(Direction.FORWARD, 150, 900)
    await attachmentMotor_async(port.E, 120, 400, Direction.DOWN)
    await turn(Direction.LEFT, 10, 600)
    await straight(Direction.BACKWARD,1000, 900)
    await turn(Direction.LEFT, 18, 600)
    await straight(Direction.BACKWARD, 750, 900)

    await turn(Direction.LEFT, 75, 600)
    await straight(Direction.BACKWARD, 900, 900)


async def Run_4():
    """ This is Run4 Aakash&Aarav- Boat dropping Mission"""
    print("This is Run4")
    #await attachmentMotor_async(Arm.RIGHT, 200,100, Direction.UP)
    #await motor.run_for_degrees(port.B, -590,100) # moving attachment arm to the trident
    await straight(Direction.FORWARD, 300, 100) # moving to the boat mission
    await straight(Direction.BACKWARD, 500,600) #going back home
    await runloop.sleep_ms(3000)
    await turn(Direction.LEFT,0,300,-32) #turning to the trident mission
    await straight(Direction.FORWARD,1000,500) #moving towards the trident mission
    await turn(Direction.RIGHT,0,300,0) #turning to the trident mission
    await straight(Direction.FORWARD,460,500) #moving towards the trident mission (original)
    await turn(Direction.LEFT,0,300,-45) #turning to the trident mission
    await straight(Direction.FORWARD,35,500) #moving towards the trident mission
    await attachmentMotor_async(Arm.LEFT, 650, 500, Direction.DOWN) #arm down to pick trident

    #await turn(Direction.LEFT,3,300) #grip trident?
    #await turn(Direction.RIGHT,3,300) #one more try to grip
    await straight(Direction.BACKWARD,10,50)
    #await attachmentMotor_async(Arm.LEFT, 200, 50, Direction.UP) #move the arm up
    attachmentMotor(Arm.LEFT, 200, 75, Direction.UP) #move the arm up
    #await attachmentMotor_async(Arm.LEFT, 200, 300, Direction.UP)
    await straight(Direction.BACKWARD,100,50)

    ### AFTER TRIDENT ##
    await turn(Direction.RIGHT,0,300,-10)
    await straight(Direction.FORWARD,500,500) #moving towards the whale mission

    await turn(Direction.RIGHT,0,300,0)
    await straight(Direction.FORWARD,900,500) #moving towards the whale mission

    await turn(Direction.RIGHT,0,300,135) #turn 45 after moving to the another start point
    await straight(Direction.BACKWARD,200,500) #moving towards the whale mission
    await turn(Direction.LEFT,0,300,90) #turn 90 wrt to start point
    await straight(Direction.BACKWARD,360,500) #moving towards the whale mission
    await turn(Direction.RIGHT,0,300,135) #turn to facing the whale mission
    await straight(Direction.BACKWARD,650,500) #moving towards the whale mission
    await attachmentMotor_async(Arm.RIGHT, 950,500, Direction.UP) ##Feed the WHALE


    #await attachmentMotor_async(Arm.RIGHT, 500,300, Direction.UP)


async def Run_5():
    """ This is Run5 """

    print("This is Run4_2")
    """ This is Run4 Aakash&Aarav- NO TRIDENT DROP SHARK"""
    print("This is Run4_2")

    await straight(Direction.FORWARD, 300,40) # moving to the boat mission FROM HOME2 (slowing to 40)
    await straight(Direction.BACKWARD, 500,700) #going back home
    await runloop.sleep_ms(5000) #REMOVE BOAT ATTACHEMENT AND PLACE SHARK ATTACHEMENT
    await turn(Direction.LEFT,0,500,-32) #turning to the SHARK PLACEMENT
    await straight(Direction.FORWARD,900,800) #moving towards the SHARK PLACEMENT - STEP 1
    await turn(Direction.RIGHT,0,500,0) #turning to SHARK PLACEMENT - TURN 1
    await straight(Direction.FORWARD,400,500) #moving towards SHARK PLACEMENT - STEP 2
    await turn(Direction.LEFT,0,300,-25) #turning to SHARK PLACEMENT - TURN 2
    await straight(Direction.FORWARD,240,500) #PLACE THE SHARK
    await straight(Direction.BACKWARD,210,500) #LEAVE THE SHARK BOX
    await turn(Direction.RIGHT,0,300,-10) #turning to move to HOME1 - TURN 1
    await straight(Direction.FORWARD,200,400) #moving towards HOME1 - STEP 1

    await turn(Direction.RIGHT,0,300,0) #turning to move to HOME1 - TURN 2

    #await straight(Direction.FORWARD,1600,800) #moving towards HOME1 - STEP 2

    await straight(Direction.FORWARD,1300,1000, accel=1000) #moving towards HOME1 - STEP 2

    ##----------------##
    ## HOME1 TO WHALE ##
    ##----------------##
    await turn(Direction.RIGHT,0,300,135) #turn 45 after moving to the another start point
    await straight(Direction.BACKWARD,200,500) #moving towards the whale mission
    await turn(Direction.LEFT,0,300,90) #turn 90 wrt to start point
    await straight(Direction.BACKWARD,475,500) #moving towards the whale mission

    await turn(Direction.RIGHT,0,300,135) #turn to facing the whale mission
    await straight(Direction.BACKWARD,600,500) #moving towards the whale mission
    await attachmentMotor_async(Arm.RIGHT, 950,500, Direction.UP) ##Feed the WHALE

    '''
    ##----------------##
    ## HOME1 TO WHALE ##
    ##----------------##
    #await turn(Direction.RIGHT,0,300,135) #turn 45 after moving to the another start point
    #await straight(Direction.BACKWARD,200,500) #moving towards the whale mission

    await turn(Direction.LEFT,0,500,270) #turn 90 wrt to start point

    await straight(Direction.FORWARD,800,800) #moving towards the whale mission (Can go to 800???)
    await turn(Direction.RIGHT,0,400,135) #turn to facing the whale mission
    await straight(Direction.BACKWARD,550,300) #moving towards the whale mission
    await attachmentMotor_async(Arm.RIGHT, 950,600, Direction.UP) ##Feed the WHALE
    '''
    await straight(Direction.FORWARD,300,300) #moving towards the whale mission


    '''

    await attachmentMotor_async(Arm.LEFT, 650, 500, Direction.DOWN) #arm down to pick trident

    #await turn(Direction.LEFT,3,300) #grip trident?
    #await turn(Direction.RIGHT,3,300) #one more try to grip
    await straight(Direction.BACKWARD,10,50)
    #await attachmentMotor_async(Arm.LEFT, 200, 50, Direction.UP) #move the arm up
    attachmentMotor(Arm.LEFT, 200, 75, Direction.UP) #move the arm up
    #await attachmentMotor_async(Arm.LEFT, 200, 300, Direction.UP)
    await straight(Direction.BACKWARD,100,50)

    ### AFTER TRIDENT ##
    await turn(Direction.RIGHT,0,300,-10)
    await straight(Direction.FORWARD,500,500) #moving towards the whale mission

    await turn(Direction.RIGHT,0,300,0)
    await straight(Direction.FORWARD,900,500) #moving towards the whale mission

    await turn(Direction.RIGHT,0,300,135) #turn 45 after moving to the another start point
    await straight(Direction.BACKWARD,200,500) #moving towards the whale mission
    await turn(Direction.LEFT,0,300,90) #turn 90 wrt to start point
    await straight(Direction.BACKWARD,360,500) #moving towards the whale mission
    await turn(Direction.RIGHT,0,300,135) #turn to facing the whale mission
    await straight(Direction.BACKWARD,650,500) #moving towards the whale mission
    await attachmentMotor_async(Arm.RIGHT, 950,500, Direction.UP) ##Feed the WHALE
    '''


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

        elif color_detected is color.BLACK:
            await setGearsLeft()
            await setGearsRight()


runloop.run(main())
