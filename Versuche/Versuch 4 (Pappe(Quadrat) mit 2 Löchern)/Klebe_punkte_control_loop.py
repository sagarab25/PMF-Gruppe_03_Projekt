import sys
import os
import time

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

from svgpathtools import svg2paths
from shapely.geometry import Polygon, Point
import matplotlib.pyplot as plt


# --------------------------------------------------
# CONFIG
# --------------------------------------------------

ROBOT_HOST = "localhost"
ROBOT_PORT = 30004
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
config_filename = os.path.join(BASE_DIR,"control_loop_configuration.xml")

SVG_FILE = os.path.join(BASE_DIR,"test_panel.svg")


# --------------------------------------------------
# HELPER FUNCTIONS
# --------------------------------------------------

def list_to_setp(setp, values):

    for i in range(6):

        setp.__dict__["input_double_register_%i" % i] = values[i]

    return setp


# --------------------------------------------------
# LOAD SVG
# --------------------------------------------------

paths, attributes = svg2paths(SVG_FILE)

print("SVG loaded")


# --------------------------------------------------
# CONVERT SVG TO POLYGONS
# --------------------------------------------------

polygons = []

samples_per_path = 200

for path in paths:

    points = []

    for i in range(samples_per_path):

        t = i / (samples_per_path - 1)

        p = path.point(t)

        x = p.real
        y = p.imag

        points.append((x, y))

    poly = Polygon(points)

    polygons.append(poly)


# --------------------------------------------------
# OUTER POLYGON
# --------------------------------------------------

outer_polygon = max(polygons, key=lambda p: p.area)


# --------------------------------------------------
# HOLES
# --------------------------------------------------

holes = []

for poly in polygons:

    if poly != outer_polygon:

        holes.append(poly)


# --------------------------------------------------
# FREE SURFACE
# --------------------------------------------------

hole_coords = []

for hole in holes:

    hole_coords.append(list(hole.exterior.coords))


free_surface = Polygon(
    outer_polygon.exterior.coords,
    holes=hole_coords
)


# --------------------------------------------------
# PARAMETERS
# --------------------------------------------------

spacing = 15

min_edge_distance = 10


# --------------------------------------------------
# SAFE REGION
# --------------------------------------------------

safe_region = free_surface.buffer(-min_edge_distance)


# --------------------------------------------------
# BOUNDING BOX
# --------------------------------------------------

minx, miny, maxx, maxy = outer_polygon.bounds


# --------------------------------------------------
# VALID GLUE POINTS
# --------------------------------------------------

valid_points = []

for x in range(int(minx), int(maxx), spacing):

    for y in range(int(miny), int(maxy), spacing):

        p = Point(x, y)

        if safe_region.contains(p):

            valid_points.append(p)


print("Valid glue points:", len(valid_points))


# --------------------------------------------------
# CONVERT TO ROBOT POSES
# --------------------------------------------------

trajectory = []

scale = 0.001

x_offset = -0.30
y_offset = -0.45

z = 0.34

rx = 0
ry = 3.11
rz = 0.04


for p in valid_points:

    robot_x = x_offset + scale * p.x
    robot_y = y_offset + scale * p.y

    pose = [
        robot_x,
        robot_y,
        z,
        rx,
        ry,
        rz
    ]

    trajectory.append(pose)


print("Robot poses:", len(trajectory))

# --------------------------------------------------
# ROBOT TRAJECTORY VISUALIZATION
# --------------------------------------------------

fig, ax = plt.subplots()


# outer contour
x, y = outer_polygon.exterior.xy
ax.plot(x, y, 'b')


# holes
for hole in holes:

    hx, hy = hole.exterior.xy

    ax.plot(hx, hy, 'r')


# safe region
if safe_region.geom_type == "Polygon":

    sx, sy = safe_region.exterior.xy

    ax.plot(sx, sy, 'm')

elif safe_region.geom_type == "MultiPolygon":

    for poly in safe_region.geoms:

        sx, sy = poly.exterior.xy

        ax.plot(sx, sy, 'm')


# glue points
for p in valid_points:

    ax.plot(p.x, p.y, 'go')


# trajectory lines
for i in range(len(valid_points) - 1):

    x1 = valid_points[i].x
    y1 = valid_points[i].y

    x2 = valid_points[i + 1].x
    y2 = valid_points[i + 1].y

    ax.plot([x1, x2], [y1, y2], 'k--')


ax.set_aspect('equal')

plt.title("Robot Glue Trajectory Preview")

plt.show()


# --------------------------------------------------
# RTDE SETUP
# --------------------------------------------------

conf = rtde_config.ConfigFile(config_filename)

state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)

con.connect()

con.send_output_setup(state_names, state_types)

setp = con.send_input_setup(setp_names, setp_types)

watchdog = con.send_input_setup(
    watchdog_names,
    watchdog_types
)

if not con.send_start():

    sys.exit()


# --------------------------------------------------
# INITIALIZE
# --------------------------------------------------

watchdog.input_int_register_0 = 0

con.send(watchdog)

time.sleep(1)


# --------------------------------------------------
# CONTROL LOOP
# --------------------------------------------------

keep_running = True

move_completed = True

current_index = 0


while keep_running:

    state = con.receive()

    if state is None:

        break


    # ----------------------------------------------
    # ROBOT READY FOR NEXT POSE
    # ----------------------------------------------

    if move_completed and state.output_int_register_0 == 1:

        move_completed = False

        pose = trajectory[current_index]

        print("Moving to:", pose)

        # pose
        list_to_setp(setp, pose)

        # glue ON command
        setp.input_int_register_1 = 1

        con.send(setp)

        # new pose available
        watchdog.input_int_register_0 = 1

        current_index += 1

        if current_index >= len(trajectory):

            current_index = 0


    # ----------------------------------------------
    # ROBOT REPORTS DONE
    # ----------------------------------------------

    elif not move_completed and state.output_int_register_0 == 0:

        print("Glue point completed")

        move_completed = True

        # reset handshake
        watchdog.input_int_register_0 = 0

        # glue OFF command
        setp.input_int_register_1 = 0


    # ----------------------------------------------
    # SEND WATCHDOG
    # ----------------------------------------------

    con.send(watchdog)

    time.sleep(0.01)


# --------------------------------------------------
# SHUTDOWN
# --------------------------------------------------

con.send_pause()

con.disconnect()