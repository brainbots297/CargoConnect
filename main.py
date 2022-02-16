# **************************************************
# import general python libraries
from math import *
import time
import math

# **************************************************
# import spike prime libraries
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer

# **************************************************
# declare spike prime objects for hub, motors, and sensors
hub = PrimeHub()

motor_X = Motor('D')
motor_X.set_stop_action("hold")

motor_Y = Motor('A')
motor_Y.set_stop_action("hold")

motor_B = Motor('B')
motor_B.set_stop_action("hold")

motor_C = Motor('C')
motor_C.set_stop_action("hold")

motor_pair = MotorPair('B','C')
motor_pair.set_stop_action('hold')

sensor_left = ColorSensor('E')
sensor_right = ColorSensor('F')


# **************************************************
# **************************************************
def Mission1-1():
    global hub
    global motor_pair
    global motor_C
    global motor_Y
    global sensor_right
    global sensor_left

    # leave launch area using gyro tracking
    power = 40
    hub.motion_sensor.reset_yaw_angle()
    heading = 0
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() < 1.5 * 360:
        error = heading - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(10 * error))
    motor_pair.stop()

    # shift lift to clear transportation journey plane
    motor_X.run_for_rotations(1, 100)

    # track line to push truck to bridge
    motor_Y.run_for_degrees(350, speed=-100)
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() <= 2.75 * 360:
        error = 50 - sensor_right.get_reflected_light()
        motor_pair.start_at_power(power, int(1.5 * error))
    motor_pair.stop()

    hub.motion_sensor.reset_yaw_angle()

    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() < 0.3 * 360:
        error = 50 - sensor_right.get_reflected_light()
        motor_pair.start_at_power(power, int(1.5 * error))
    motor_pair.stop()

    #raises the lift to clear the truck
    motor_Y.run_for_degrees(250, speed=-100)

    #Drives robot along line until left sensor sees the crossline by the bridge
    #Robot will knock down west bridge
    while sensor_left.get_reflected_light() > 50:
        error = 50 - sensor_right.get_reflected_light()
        motor_pair.start_at_power(power, int(1 * error))
    motor_pair.move(0.5, 'rotations', steering=0, speed=power)

    #raises the lift to clear the east bridge
    motor_Y.run_for_degrees(250, speed=-100)

    #drives past east bridge
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() < 0.95 * 360:
        error = 50 - sensor_right.get_reflected_light()
        motor_pair.start_at_power(power, int(1 * error))
    motor_pair.stop()

    #Lower Y and back up to knock down east bridge
    motor_Y.run_for_degrees(250, speed=100)
    motor_pair.move_tank(0.6, 'rotations', left_speed=-power, right_speed=-power)


# **************************************************
# **************************************************
def Mission1-2():
    global hub
    global motor_pair
    global motor_B
    global motor_C
    global motor_X
    global motor_Y
    global sensor_right
    global sensor_left

    # move lift to bottom concurrently with robot move on mat
    motor_Y.set_stall_detection(True)
    motor_Y.start(speed=50)

    # turn robot towards ship
    while hub.motion_sensor.get_yaw_angle() > -87:
        motor_C.start_at_power(30)
    motor_C.stop()

    # raise lift to clear ship
    motor_Y.run_for_rotations(1.75, -100)

    # drive up to ship
    power = 40
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() < 1.4 * 360:
        error = -90 - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(10 * error))
    motor_pair.stop()

    # run arm out and back in to push crane to unload ship
    motor_X.run_for_rotations(4.4, -100)
    motor_X.run_for_rotations(3.4, 100)

    # backup from ship
    power = -40
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() > -1.4 * 360:
        error = -89 - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(-10 * error))
    motor_pair.stop()

    # turn back to face train tracks
    while hub.motion_sensor.get_yaw_angle() < 0:
        motor_C.start_at_power(-30)
    motor_C.stop()

    # move lift to bottom
    motor_Y.set_stall_detection(True)
    motor_Y.start(speed=50)

    # move forward using gyro
    power = 40
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() < 2.75 * 360:
        error = 0 - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(10 * error))
    motor_pair.stop()

    # raise lift
    motor_Y.run_for_rotations(3, -100)

    # move forward using gyro until black line by train is reached
    power = 40
    while sensor_left.get_reflected_light() > 25:
        error = 0 - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(10 * error))
    motor_pair.stop()
    motor_pair.move_tank(0.1, 'rotations', left_speed=-power, right_speed=-power)

    # turn clockwise to face the three containers
    while hub.motion_sensor.get_yaw_angle() < 87:
        motor_B.start_at_power(-30)
    motor_B.stop()

    # move forward tracking line to align to repair train track
    power = 30
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() < 0.3 * 360:
        error = sensor_left.get_reflected_light() - 60
        motor_pair.start_at_power(power, int(1.5 * error))
    motor_pair.stop()

    # move lift to repair train track
    motor_X.run_for_rotations(1.85, 100)
    motor_Y.run_for_rotations(1.1, 100)
    motor_X.run_for_rotations(1.6, -100)
    motor_Y.run_for_rotations(1.1, -100)

    # move backwards using gyro to end of train tracks
    power = -40
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() > -2 * 360:
        error = 89 - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(-10 * error))
    motor_pair.stop()

    # position lift to push train cars
    motor_X.run_for_degrees(230, 100)
    motor_Y.run_for_rotations(1.7, 100)

    # move forward tracking line to move train cars
    motor_C.set_degrees_counted(0)
    power = 40
    while motor_C.get_degrees_counted() < 2.5 * 360:
        error = sensor_left.get_reflected_light() - 60
        motor_pair.start_at_power(power, int(1.5 * error))
    motor_pair.stop()

    # raise lift above train cars
    motor_Y.run_for_rotations(2, -100)
    motor_X.run_for_degrees(400, -100)
    while hub.motion_sensor.get_yaw_angle() > 45:
        motor_B.start_at_power(30)
    motor_B.stop()

    motor_pair.move_tank(1.15, 'rotations', -25 , -25)

    while hub.motion_sensor.get_yaw_angle() < 88:
        motor_B.start_at_power(-30)
    motor_B.stop()

    LiftToBottom()

    # adjust lift and drive in to pickup containers
    motor_Y.run_for_degrees(195, -100)

    # move forward a bit using gyro
    power = 40
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() < 0.95 * 360:
        error = 90 - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(10 * error))
    motor_pair.stop()

    # raise containers
    motor_Y.run_for_rotations(3, -100)

    # back up with containers using gyro to get near the white line
    power = -40
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() > -2 * 360:
        error = 87 - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(-10 * error))
    motor_pair.stop()

    # continue back up until white line is reached
    while sensor_left.get_reflected_light() < 90:
        error = 89 - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(-10 * error))
    motor_pair.stop()

    # turn robot to align with line
    motor_B.run_for_rotations(0.5, -25)

    # track line to go home
    motor_C.set_degrees_counted(0)
    power = 40
    while motor_C.get_degrees_counted() < 5 * 360:
        error = sensor_left.get_reflected_light() - 60
        motor_pair.start_at_power(power, int(1.5 * error))
    motor_pair.stop()

    # continue towards home using gyro
    power = 50
    hub.motion_sensor.reset_yaw_angle()
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() < 2 * 360:
        error = 0 - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(10 * error))

    # finish getting home using gyro
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() < 3 * 360:
        error = 10 - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(10 * error))
    motor_pair.stop()

    time.sleep(2)
    motor_X.run_for_degrees(590, -100)
    motor_Y.set_stall_detection(True)
    motor_Y.run_for_seconds(5, 50)


# **************************************************
# **************************************************
def SohamTest():
    global hub
    global motor_pair
    global motor_B
    global motor_C
    global motor_X
    global motor_Y
    global sensor_right
    global sensor_left

    power = 30

    # lift arm with TEU to clear plane
    motor_Y.run_for_degrees(360, speed=-100)

    #move out of launch area
    motor_pair.move_tank(1, 'rotations', 20 , 20)

    # follow line and use rotation first towards green circle
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() <= 1 * 360:
        error = 60 - sensor_right.get_reflected_light()
        motor_pair.start_at_power(power, int(1.5 * error))
    motor_pair.stop()

    # track line with right sensor
    # stop at black to drop green container
    while sensor_left.get_reflected_light() > 50:
        error = 60 - sensor_right.get_reflected_light()
        motor_pair.start_at_power(power, int(1.5 * error))
    motor_pair.stop()

    #reach green circle
    motor_pair.move_tank(0.5, 'rotations', 20 , 20)

    # drop TEU in green circle - adjust x axis also to center in circle
    motor_Y.run_for_degrees(365, speed=100)
    motor_X.run_for_degrees(360, speed=100)

    #move out of green circle
    motor_pair.move_tank(1.5, 'rotations', -20 , -20)

    # remove cargo from plane
    motor_Y.run_for_rotations(3, speed=-100)
    motor_X.run_for_rotations(1.5, speed=100)
    motor_Y.run_for_rotations(3.1, speed=100)

    # wait for cargo to drop
    time.sleep(0.5)

    # raise arm to clear gray cargo
    motor_Y.run_for_rotations(0.5, speed=-100)
    motor_X.run_for_rotations(1.5, speed=-100)

    # move ahead to catch cargo for adjustment
    motor_pair.move_tank(0.6, 'rotations', 20 , 20)

    # lower arm to engage cargo
    motor_Y.run_for_rotations(0.5, speed=100)

    # adjust cargo in black circle
    motor_pair.move_tank(0.6, 'rotations', -20 , -20)

    # lift arm to clear cargo
    motor_Y.run_for_rotations(2, speed=-100)

    # return to home
    motor_pair.move_tank(2.5, 'rotations', -20 , -21)


# **************************************************
# **************************************************
def LiftToBottom():
    global motor_Y

    motor_Y.set_stall_detection(False)
    motor_Y.set_degrees_counted(0)
    last_degrees_counted = 0
    motor_Y.start(speed=100)
    while True:
        time.sleep(0.25)
        if abs(motor_Y.get_degrees_counted() - last_degrees_counted) < 10:
            break
        last_degrees_counted = motor_Y.get_degrees_counted()
    motor_Y.stop()


# **************************************************
# **************************************************
def Mission2():
    global hub
    global motor_pair
    global motor_C
    global motor_Y
    global sensor_right
    global sensor_left

    # lower lift to bottom then raise up a bit for no drag
    motor_Y.run_for_rotations(0.5, speed=-100)

    # leave launch area using gyro tracking
    hub.motion_sensor.reset_yaw_angle()
    heading = 0
    power = 40
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() < 1 * 360:
        error = heading - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(10 * error))

    # continue tracking black line to first black cross line
    while sensor_left.get_reflected_light() > 20:
        error = sensor_right.get_reflected_light() - 60
        motor_pair.start_at_power(power, int(1.25 * error))

    # move past black line
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() < 0.25 * 360:
        error = sensor_right.get_reflected_light() - 60
        motor_pair.start_at_power(power, int(1.25 * error))
    motor_pair.stop()

    # raise lift to get over truck
    motor_Y.run_for_rotations(1.25, speed=-100)

    # turn to adjust to track other side of line
    motor_B.run_for_rotations(0.25, -25)

    # continue tracking black line to second black cross line
    power = 30
    while sensor_left.get_reflected_light() > 20:
        error = 60 - sensor_right.get_reflected_light()
        motor_pair.start_at_power(power, int(1.25 * error))

    # move past second black cross line
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() < .25 * 360:
        error = 60 - sensor_right.get_reflected_light()
        motor_pair.start_at_power(power, int(1.25 * error))

    # continue tracking black line to third black cross line
    while sensor_left.get_reflected_light() > 20:
        error = 60 - sensor_right.get_reflected_light()
        motor_pair.start_at_power(power, int(1.25 * error))
    motor_pair.stop()

    # reset gyro
    hub.motion_sensor.reset_yaw_angle()

    # move to get into position for cargo connect TEU delivery
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() < 0.45 * 360:
        error = heading - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(10 * error))
    motor_pair.stop()

    # turn clockwise to face the cargo connect circle
    while hub.motion_sensor.get_yaw_angle() < 80:
        motor_pair.start_tank_at_power(30, -30)
    motor_pair.stop()

    # lower lift to deliver TEU to cargo connect circle
    motor_Y.run_for_rotations(0.8, speed=100)

    # Eject TEU into CC circle
    motor_X.run_for_rotations(3.25, 100)
    motor_X.run_for_rotations(3.25, -100)

    # Get away from CC circle
    motor_Y.run_for_rotations(0.8, speed=-100)
    while hub.motion_sensor.get_yaw_angle() > -30:
        motor_pair.start_tank_at_power(-30, 30)
    motor_pair.stop()

    # Drive to blue circle
    power = 40
    while sensor_left.get_reflected_light() > 20:
        error = sensor_right.get_reflected_light() - 60
        motor_pair.start_at_power(power, int(1.25 * error))
    motor_pair.stop()

    # Move forward to do helicopter
    motor_pair.move_tank(0.5, "rotations", 30, 30)

    # Move to blue circle
    motor_pair.move_tank(-1.4, "rotations", 28, 30)
    motor_C.run_for_rotations(0.15, speed=30)

    # Deliver blue TEU
    motor_Y.run_for_rotations(1, speed=100)
    motor_X.run_for_rotations(3.25, 100)
    motor_X.run_for_rotations(3.25, -100)

    # Turn clockwise towards gray circle
    while hub.motion_sensor.get_yaw_angle() < 90:
        motor_pair.start_tank_at_power(25, -25)
    motor_pair.stop()

    # Move up to gray circle and deliver gray TEU
    motor_pair.move_tank(0.5, "rotations", 40, 40)
    motor_X.run_for_rotations(3.25, 100)
    motor_X.run_for_rotations(3.25, -100)

    # Move away from gray circle
    while sensor_right.get_reflected_light() > 20:
        motor_C.start_at_power(-30)
    while sensor_right.get_reflected_light() < 90:
        motor_C.start_at_power(-30)
    motor_C.stop()
    motor_Y.run_for_rotations(1, speed=-100)

    # track some rotations to get back towards bridge
    power = 40
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() < 2.5 * 360:
        error = sensor_left.get_reflected_light() - 55
        motor_pair.start_at_power(power, int(1.1 * error))

    # continue tracking to black cross line
    while sensor_right.get_reflected_light() > 20:
        error = sensor_left.get_reflected_light() - 60
        motor_pair.start_at_power(power, int(1 * error))
    motor_pair.stop()

    # turn towards cargo ship
    hub.motion_sensor.reset_yaw_angle()
    while(hub.motion_sensor.get_yaw_angle() < 88):
        motor_B.start_at_power(-35)
    motor_B.stop()

    # track towards cargo ship using gyro until white cross line
    power = 40
    heading = 90
    while sensor_left.get_reflected_light() > 20:
        error = heading - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(10 * error))
    motor_pair.stop()

    # continue for rotations to get closer to cargo ship
    hub.motion_sensor.reset_yaw_angle()
    power = 30
    heading = 0
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() < 0.5 * 360:
        error = heading - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(10 * error))
    motor_pair.stop()

    # turn towards cargo ship
    while(hub.motion_sensor.get_yaw_angle() < 88):
        motor_B.start_at_power(-30)
    motor_B.stop()

    # track towards cargo ship using gyro until black cross line
    power = 25
    heading = 89
    while sensor_right.get_reflected_light() > 20:
        error = heading - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(10 * error))
    motor_pair.stop()

    # backup and unload first teu
    motor_pair.move_tank(100, 'degrees', -20, -20)
    motor_Y.run_for_rotations(0.75, speed=100)
    motor_X.run_for_rotations(3.25, 100)
    motor_X.run_for_rotations(3.25, -100)

    # backup and unload second teu
    motor_pair.move_tank(100, 'degrees', -20, -20)
    motor_X.run_for_rotations(3.25, 100)
    motor_X.run_for_rotations(1.25, -100)
    motor_Y.run_for_rotations(0.75, speed=-100)

    # backup towards accident avoidance using gyro
    heading = 90
    while sensor_right.get_reflected_light() < 90:
        error = heading - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(-power, int(-10 * error))
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() > -0.6 * 360:
        error = heading - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(-power, int(-10 * error))
    motor_pair.stop()

    # turn robot to align lift with accident avoidance
    hub.motion_sensor.reset_yaw_angle()
    while(hub.motion_sensor.get_yaw_angle() < 105):
        motor_C.start_at_power(-35)
    motor_C.stop()

    # run lift arm out to push yellow panel on accident avoidance
    motor_X.run_for_rotations(2, -100)


# **************************************************
# **************************************************
# main program
while True:
    hub.light_matrix.show_image('ASLEEP')

    while True:

        if hub.left_button.is_pressed():
            hub.light_matrix.show_image('SMILE')
            motor_X.set_stop_action("hold")
            motor_Y.set_stop_action("hold")
            Mission1-1()
            Mission1-2()
            motor_X.set_stop_action("coast")
            motor_X.run_for_degrees(1, speed=-100)
            motor_Y.set_stop_action("coast")
            motor_Y.run_for_degrees(1, speed=-100)
            break

        if hub.right_button.is_pressed():
            hub.light_matrix.show_image('HAPPY')
            motor_X.set_stop_action("hold")
            motor_Y.set_stop_action("hold")
            Mission2()
            break

#SohamTest()
#LineTrackTest()

raise SystemExit

# end of main program
# **************************************************
# **************************************************
