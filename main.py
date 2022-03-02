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

LineEdge1 = 50
LineEdge2 = 50

# **************************************************
# **************************************************

def GyroTrackRotations(power, rotations, gain, heading, stop):
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() < rotations * 360:
        error = heading - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(gain * error))
    if stop == True:
        motor_pair.stop()

def GyroTrackSensorBlack(power, gain, heading, sensor, stop):
    while sensor.get_reflected_light() > 20:
        error = heading - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(gain * error))
    if stop == True:
        motor_pair.stop()

def LineTrackRotations(power, rotations, gain, sensor, edge, stop):
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() <= rotations * 360:
        if edge == True:
            error = LineEdge1 - sensor.get_reflected_light()
        else:
            error = sensor.get_reflected_light() - LineEdge1
        motor_pair.start_at_power(power, int(gain * error))
    if stop == True:
        motor_pair.stop()

def LineTrackSensorBlack(power, gain, sensor_track, edge, sensor_stop, stop):
    while sensor_stop.get_reflected_light() > 20:
        if edge == True:
            error = LineEdge1 - sensor_track.get_reflected_light()
        else:
            error = sensor_track.get_reflected_light() - LineEdge1
        motor_pair.start_at_power(power, int(gain * error))
    if stop == True:
        motor_pair.stop()


# **************************************************
# **************************************************
def Ben():
    global hub
    global motor_pair
    global motor_C
    global motor_Y
    global sensor_right
    global sensor_left

    global LineEdge1
    global LineEdge2

    hub.motion_sensor.reset_yaw_angle()

    # leave launch area using gyro tracking
    GyroTrackRotations(40, 1.5, 10, 0, True)

    # adjust lift to clear transportation journey plane
    motor_X.run_for_rotations(1, 100)
    motor_Y.run_for_degrees(350, speed=-100)

    # track line to push truck to bridge
    LineTrackRotations(35, 2.75, 1.5, sensor_right, True, True)

    # after drive around the line bend, reset gyro
    hub.motion_sensor.reset_yaw_angle()

    # track a bit more to push truck
    LineTrackRotations(40, 0.4, 1.5, sensor_right, True, True)

    # raises the lift to clear the truck
    motor_Y.run_for_degrees(250, speed=-100)

    # drive robot along line until left sensor sees the crossline by the bridge
    # robot will knock down west bridge
    LineTrackSensorBlack(40, 1, sensor_right, True, sensor_left, False)
    motor_pair.move_tank(0.5, 'rotations', left_speed=40, right_speed=40)

    # raise the lift to clear the east bridge
    motor_Y.run_for_degrees(250, speed=-100)

    # drive past east bridge
    LineTrackRotations(40, 0.95, 1, sensor_right, True, True)

    # lower Y and back up to knock down east bridge
    motor_Y.run_for_degrees(250, speed=100)
    motor_pair.move_tank(0.6, 'rotations', left_speed=-40, right_speed=-40)


# **************************************************
# **************************************************
def Soham():
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

    # move forward using gyro
    power = 60
    GyroTrackRotations(power, 2.75, 10, 0, True)

    # raise lift
    motor_Y.run_for_rotations(3, -100)

    # move forward using gyro until black line by train is reached
    power = 50
    GyroTrackSensorBlack(power, 10, 0, sensor_left, True)

    # turn clockwise to face the three containers
    while hub.motion_sensor.get_yaw_angle() < 88:
        motor_pair.start_at_power(25, 100)
    motor_pair.stop()

    # move lift to repair train track
    motor_X.run_for_rotations(1.85, 100)
    motor_Y.run_for_rotations(1.1, 100)
    motor_X.run_for_rotations(2, -100)
    motor_Y.run_for_rotations(1.1, -100)

    # move backwards using gyro to end of train tracks
    power = -40
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() > -1.8 * 360:
        error = 88 - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(-10 * error))
    motor_pair.stop()

    # position lift to push train cars
    motor_X.run_for_degrees(374, 100)
    motor_Y.run_for_rotations(1.7, 100)

    # move forward tracking line to move train cars
    power = 40
    LineTrackRotations(power, 2.6, 1, sensor_left, True, True)

    # raise lift above train cars
    motor_Y.run_for_rotations(2, -100)
    motor_X.run_for_rotations(2.1, -100)

    # turn to back away from train tracks
    while hub.motion_sensor.get_yaw_angle() > 50:
        motor_pair.start_at_power(25, -100)
    motor_pair.stop()

    # move backwards using gyro
    power = -40
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() > -1 * 360:
        error = 45 - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(power, int(-10 * error))
    motor_pair.stop()

    # turn towards shipping containers
    while hub.motion_sensor.get_yaw_angle() < 85:
        motor_pair.start_at_power(25, 100)
    motor_pair.stop()

    # adjust lift and drive in to pickup containers
    motor_Y.run_for_degrees(1030, 100)

    # move forward using gyro
    power = 40
    GyroTrackRotations(power, 0.5, 10, 86, False)
    while sensor_right.get_reflected_light() > 25:
        motor_pair.start_at_power(power, 0)
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
        motor_pair.start_at_power(-40, int(-10 * error))
    motor_pair.stop()

    # turn robot to align with line
    while sensor_left.get_reflected_light() > 50:
        motor_pair.start_at_power(25, 100)
    motor_pair.stop()

    # track line to go home
    LineTrackRotations(40, 4, 1.5, sensor_left, False, True)

    # continue towards home using gyro
    hub.motion_sensor.reset_yaw_angle()
    GyroTrackRotations(50, 4, 10, 0, False)
    GyroTrackSensorBlack(40, 10, 10, sensor_right, False)
    GyroTrackRotations(30, 0.7, 10, 10, True)

    # turn parallel to home mat edge
    while hub.motion_sensor.get_yaw_angle() < 85:
        motor_pair.start_at_power(25, 100)
    motor_pair.stop()

    # run along home line
    GyroTrackRotations(30, 0.65, 10, 90, True)

    # turn towards plane
    while hub.motion_sensor.get_yaw_angle() < 130:
        motor_pair.start_at_power(25, 100)
    motor_pair.stop()

    # run up to plane
    motor_X.run_for_rotations(1, -100)
    GyroTrackRotations(30, 0.5, 10, 135, True)
    hub.motion_sensor.reset_yaw_angle()

    # unload plane cargo
    motor_X.run_for_rotations(1.5, -100)
    LiftToBottom()
    motor_Y.run_for_rotations(3, -100)
    motor_X.run_for_rotations(2, 100)

    # turn to drive around plane
    while hub.motion_sensor.get_yaw_angle() < 40:
        motor_pair.start_at_power(25, 100)

    # move to black line
    while sensor_right.get_reflected_light() > 20:
        error = 45 - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(30, int(10 * error))

    # move to green TEU circle
    GyroTrackSensorBlack(30, 10, 0, sensor_left, False)
    GyroTrackRotations(30, 0.25, 10, 0, True)

    # drop green TEU
    motor_Y.run_for_rotations(2.25, 100)
    motor_X.run_for_rotations(0.5, 100)

    # move backwards using gyro
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() > -1.2 * 360:
        error = 0 - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(-30, int(-10 * error))
    motor_pair.stop()

    # raise lift above grey teu
    motor_Y.run_for_rotations(-2, 100)

    # move backwards into home using gyro
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() > -3 * 360:
        error = 0 - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(-60, int(-10 * error))
    motor_pair.stop()


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
def CargoConnect1():
    global hub
    global motor_pair
    global motor_C
    global motor_Y
    global sensor_right
    global sensor_left

    # leave launch area using gyro tracking
    hub.motion_sensor.reset_yaw_angle()
    GyroTrackRotations(40, 1, 10, 0, False)

    # continue forward tracking black line for alignment
    LineTrackRotations(40, 1.5, 1.25, sensor_right, False, True)

    # aligned with line, so reset gyro
    hub.motion_sensor.reset_yaw_angle()

    # move past black line and turn to face east
    GyroTrackRotations(50, 1.3, 10, 0, False)
    while hub.motion_sensor.get_yaw_angle() < 40:
        motor_pair.start_at_power(25, 100)
    motor_pair.stop();

    # gyro track accross the mat until the cross line by the bridge is detected
    GyroTrackSensorBlack(50, 10, 45, sensor_left, False)

    # continue past cross line to setup the hinged blue TEU delivery
    GyroTrackRotations(50, 2, 10, 45, False)

    while hub.motion_sensor.get_yaw_angle() < 100:
        motor_pair.start_at_power(25, 100)

    # drive in to the Cargo Connect circle
    GyroTrackRotations(30, 0.2, 10, 115, True)

    # raise lift to release hinged blue TEU
    motor_Y.run_for_rotations(1.1, speed=-100)

    # turn away from Cargo Connect circle
    while sensor_right.get_reflected_light() > 25:
        motor_pair.start_at_power(25, -100)
    motor_pair.stop();

    # lower lift to be able to run faster
    motor_Y.run_for_rotations(1.1, speed=100)

    # track to end of black line, finishing by pushing the helicopter
    LineTrackRotations(40, 2.4, 1.5, sensor_right, False, True)

    # back away from the helicopter
    motor_pair.move(0.2, 'rotations', 0, -25)

    # turn CCW to the blue circle
    while hub.motion_sensor.get_yaw_angle() > -15:
        motor_pair.start_at_power(25, -100)
    motor_pair.stop();

    # deliver blue TEU
    motor_X.run_for_rotations(3.25, 100)
    motor_X.run_for_rotations(3.25, -100)

    # raise lift to avoid contact with train tracks
    motor_Y.run_for_rotations(0.5, speed=-100)

    # turn CW to the black circle
    while hub.motion_sensor.get_yaw_angle() < 162:
        motor_pair.start_at_power(25, 100)
    motor_pair.stop();

    # drive in to the black circle
    motor_pair.move(0.3, 'rotations', 0, 25)

    # lower lift to setup TEU delivery
    motor_Y.run_for_rotations(0.5, speed=100)

    # deliver grey TEU
    motor_X.run_for_rotations(3.25, 100)
    motor_X.run_for_rotations(3.25, -100)

    # turn back to tracking line
    while sensor_left.get_reflected_light() > 25:
        motor_pair.start_at_power(25, 100)
    motor_pair.stop();

    # raise lift to clear truck ahead
    motor_Y.run_for_rotations(1, speed=-100)

    # track line back to the cross line by the bridge
    LineTrackRotations(40, 3, 1.25, sensor_left, True, False)
    LineTrackSensorBlack(40, 1.25, sensor_left, True, sensor_right, True)

def CargoConnect2():

    # turn towards cargo ship
    hub.motion_sensor.reset_yaw_angle()
    while(hub.motion_sensor.get_yaw_angle() < 85):
        motor_pair.start_at_power(25, 100)
    motor_pair.stop()

    # track line towards cargo ship
    LineTrackRotations(40, 0.8, 1.25, sensor_left, True, False)

    # reset gyro to use for tracking
    hub.motion_sensor.reset_yaw_angle()

    # continue tracking with gyro to navigate the break in the line
    GyroTrackRotations(40, 0.5, 10, 0, False)

    # resume tracking line to contact north wall
    LineTrackRotations(40, 0.8, 1.25, sensor_left, True, True)
    time.sleep(0.25)

    # reset gyro to use to turn to cargo ship
    hub.motion_sensor.reset_yaw_angle()

    # back away from wall
    motor_pair.move(-0.15, 'rotations', 0, 25)

    # turn towards cargo ship
    while(hub.motion_sensor.get_yaw_angle() < 85):
        motor_pair.start_at_power(25, 100)
    motor_pair.stop()

    # gyro track towards cargo ship and push crane
    GyroTrackRotations(30, 1.05, 10, 90, True)

    # back away from crane
    motor_pair.move_tank(220, 'degrees', -25, -25)

    # unload first TEU
    motor_Y.run_for_rotations(0.9, speed=100)
    motor_X.run_for_rotations(3.25, 100)
    motor_X.run_for_rotations(3.35, -100)

    # backup and unload second TEU
    motor_pair.move_tank(100, 'degrees', -20, -20)
    motor_X.run_for_rotations(3.35, 100)
    motor_X.run_for_rotations(1.25, -100)
    motor_Y.run_for_rotations(0.80, speed=-100)

    # backup towards accident avoidance using gyro
    heading = 90
    power = 40
    while sensor_right.get_reflected_light() < 90:
        error = heading - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(-power, int(-10 * error))
    motor_C.set_degrees_counted(0)
    while motor_C.get_degrees_counted() > -0.7 * 360:
        error = heading - hub.motion_sensor.get_yaw_angle()
        motor_pair.start_at_power(-power, int(-10 * error))
    motor_pair.stop()

    # turn robot to align lift with accident avoidance
    hub.motion_sensor.reset_yaw_angle()
    while hub.motion_sensor.get_yaw_angle() < 100:
        motor_pair.start_at_power(25, 100)
    motor_pair.stop();

    # run lift arm out to push yellow panel on accident avoidance
    motor_X.run_for_rotations(2.75, -100)
    motor_X.run_for_rotations(0.25, 100)


# **************************************************
# **************************************************
# main program

while True:

    motor_X.set_stop_action("hold")
    motor_Y.set_stop_action("hold")

    hub.light_matrix.show_image('ASLEEP')

    while True:

        if hub.left_button.is_pressed():
            hub.light_matrix.show_image('SMILE')
            Ben()
            Soham()
            motor_X.run_for_degrees(498, speed=-100)
            LiftToBottom()
            motor_Y.run_for_rotations(1, speed=-100)
            break

        if hub.right_button.is_pressed():
            hub.light_matrix.show_image('HAPPY')
            CargoConnect1()
            CargoConnect2()
            break

# end of main program
# **************************************************
# **************************************************
