import sys
import os
import time

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

# SVG library
from svgpathtools import svg2paths

# Optional visualization
import matplotlib.pyplot as plt


ROBOT_HOST = "localhost"
ROBOT_PORT = 30004
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
config_filename = os.path.join(BASE_DIR,"control_loop_configuration.xml")

SVG_FILE = os.path.join(BASE_DIR,"square.svg")


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
# LOAD SVG
# -----------------------------------------------------------

paths, attributes = svg2paths(SVG_FILE)

print("SVG loaded")


# -----------------------------------------------------------
# SAMPLE SVG POINTS
# -----------------------------------------------------------

svg_points = []

samples_per_segment = 15

for path in paths:

    for segment in path:

        for i in range(samples_per_segment):

            t = i / (samples_per_segment - 1)

            p = segment.point(t)

            x = p.real
            y = p.imag

            svg_points.append((x, y))


print("Extracted SVG points:", len(svg_points))


# -----------------------------------------------------------
# OPTIONAL VISUALIZATION
# -----------------------------------------------------------

x_plot = []
y_plot = []

for p in svg_points:
    x_plot.append(p[0])
    y_plot.append(p[1])

plt.plot(x_plot, y_plot)
plt.axis("equal")
plt.title("SVG Trajectory")
plt.show()


# -----------------------------------------------------------
# CONVERT SVG POINTS TO ROBOT POSES
# -----------------------------------------------------------

trajectory = []

# scaling factor
scale = 0.003

# robot workspace offsets
x_offset = -0.30
y_offset = -0.45

# constant robot pose parameters
z = 0.32
rx = 0
ry = 3.11
rz = 0.04

for point in svg_points:

    svg_x = point[0]
    svg_y = point[1]

    robot_x = x_offset + scale * svg_x
    robot_y = y_offset + scale * svg_y

    pose = [
        robot_x,
        robot_y,
        z,
        rx,
        ry,
        rz
    ]

    trajectory.append(pose)


print("Robot trajectory poses:", len(trajectory))


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
# Initialize registers
# -----------------------------------------------------------

watchdog.input_int_register_0 = 0

con.send(watchdog)

time.sleep(1)


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

        # Select next SVG trajectory pose
        new_pose = trajectory[current_index]

        # Copy values into RTDE input object
        list_to_setp(setp, new_pose)

        print("Moving to:", new_pose)

        # Send pose
        con.send(setp)

        # Notify robot that new pose is available
        watchdog.input_int_register_0 = 1

        # Advance to next point
        current_index += 1

        # Loop trajectory forever
        if current_index >= len(trajectory):
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