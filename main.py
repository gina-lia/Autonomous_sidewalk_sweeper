from serial_connection import read_sensors, close_connection
from tracking import DeadReckoning
from path_planning import Bug2Planner
from relay import cleanup as cleanup_relay
from relay import cleaning_off, cleaning_on
from motor_driver import move, stop
import time


tracker = DeadReckoning()

planner = Bug2Planner(
    goal_x=5.0, #5 meters is goal distance 
    goal_y=0.0,
    start_x=0.0,
    start_y=0.0
)

try:
    cleaning_on()
    start_time = time.time()

    while True:
        if time.time() - start_time > 600:
            print("Max run time reached.")
            break

        sensors = read_sensors()

        if sensors is None:
            continue

        encoder = sensors["E"]
        heading = sensors["H"]

        x, y, theta = tracker.update(encoder, heading)

        pose = {
            "x": x,
            "y": y,
            "theta": theta
        }

        sensor_data = {
            "front_left": sensors["FL"],
            "front_right": sensors["FR"],
            "right": sensors["R"],
            "left": sensors["L"]
        }

        #planner decides action
        action = planner.choose_action(sensor_data, pose)
        #------move robot-----------
        move(action, 0.5)

        time.sleep(0.1)

finally:
    stop()
    cleaning_off()
    close_connection()
    cleanup_relay()
    print("sweeper stopped.")
