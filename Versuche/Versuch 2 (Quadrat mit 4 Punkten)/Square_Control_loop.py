import sys
import os
import time

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config


ROBOT_HOST = "localhost"
ROBOT_PORT = 30004
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
config_filename = os.path.join(BASE_DIR,"control_loop_configuration.xml")


# -----------------------------------------------------------
# Helper functions
# -----------------------------------------------------------

def setp_to_list(setp):
    return [
        setp.input_double_register_0,
        setp.input_double_register_1,
        setp.input_double_register_2,
        setp.input_double_register_3,
        setp.input_double_register_4,
        setp.input_double_register_5,
    ]


def list_to_setp(setp, values):
    for i in range(6):
        setp.__dict__["input_double_register_%i" % i] = values[i]
    return setp


# -----------------------------------------------------------
# RTDE setup
# -----------------------------------------------------------

conf = rtde_config.ConfigFile(config_filename)

state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

con.send_output_setup(state_names, state_types)

setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

if not con.send_start():
    sys.exit()


# -----------------------------------------------------------
# START POSE
# Robot first moves here once
# -----------------------------------------------------------

start_pose = [-0.20, -0.45, 0.32, 0, 3.11, 0.04]


# -----------------------------------------------------------
# SQUARE TRAJECTORY
# Robot loops forever through these 4 poses
# -----------------------------------------------------------

square_points = [

    # bottom left
    [-0.30, -0.50, 0.32, 0, 3.11, 0.04],

    # top left
    [-0.30, -0.25, 0.32, 0, 3.11, 0.04],

    # top right
    [-0.05, -0.25, 0.32, 0, 3.11, 0.04],

    # bottom right
    [-0.05, -0.50, 0.32, 0, 3.11, 0.04],
]


# -----------------------------------------------------------
# Initialize registers
# -----------------------------------------------------------

list_to_setp(setp, start_pose)

watchdog.input_int_register_0 = 0

con.send(setp)
con.send(watchdog)

time.sleep(2)


# -----------------------------------------------------------
# Control loop
# -----------------------------------------------------------

keep_running = True
move_completed = True

current_index = 0

while keep_running:

    # Receive newest robot state
    state = con.receive()

    if state is None:
        break

    # -------------------------------------------------------
    # Robot ready for next pose
    # -------------------------------------------------------
    if move_completed and state.output_int_register_0 == 1:

        move_completed = False

        # Select next square point
        new_pose = square_points[current_index]

        # Copy values into RTDE input object
        list_to_setp(setp, new_pose)

        print("Moving to:", new_pose)

        # Send pose
        con.send(setp)

        # Notify robot that new pose is available
        watchdog.input_int_register_0 = 1

        # Advance to next point
        current_index += 1

        # Loop square forever
        if current_index >= len(square_points):
            current_index = 0

    # -------------------------------------------------------
    # Robot reports movement completed
    # -------------------------------------------------------
    elif not move_completed and state.output_int_register_0 == 0:

        print("Pose reached")

        move_completed = True

        # Acknowledge completion
        watchdog.input_int_register_0 = 0

    # -------------------------------------------------------
    # Send watchdog every cycle
    # -------------------------------------------------------
    con.send(watchdog)

    time.sleep(0.01)


# -----------------------------------------------------------
# Shutdown
# -----------------------------------------------------------

con.send_pause()
con.disconnect()