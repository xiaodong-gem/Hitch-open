#!/usr/bin/env python3
#
# Copyright (c) 2019-2021 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

# get which .env to use from the command line
import argparse
import time

import math
import json
from environs import Env

import lgsvl

parser = argparse.ArgumentParser()
parser.add_argument("--env", help="Environment file to use", default="svl.env", required=False)
parser.add_argument("--num_cars", help="Number of cars to spawn", default=1, required=False)
args = parser.parse_args()

# Read the mapping config file
uuids = json.load(open("src/launch/svl_launch/config/uuids.json", "r"))

# Read the environment variables from the .env file
env = Env()
env.read_env(f"src/launch/svl_launch/config/{args.env}")
env.read_env("race.env")

# MAP_NAME = args.map
NUM_CARS = int(args.num_cars)
MAP_NAME = env.str("MAP_NAME", "LVMS")
SENSORS = env.list("SENSORS", ["GPS_LIDAR"])
SIMULATOR_HOST = env.str("OSSDC__SIMULATOR_HOST")
SIMULATOR_API_PORT = env.int("OSSDC__SIMULATOR_API_PORT")

SIMULATOR_BRIDGE_HOST = env.str("OSSDC__SIMULATOR_BRIDGE_HOST")
SIMULATOR_BRIDGE_PORT = env.int("OSSDC__SIMULATOR_BRIDGE_PORT")

# Create the sim instance and connect to the SVL Simulator
sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_API_PORT)

# Map selection
map_info = uuids["MAPS"].get(MAP_NAME, None)
if map_info is None:
    raise ValueError(
        f"Map {MAP_NAME} not found in mapping config file.\n"
        + f"Available maps: {list(uuids['MAPS'].keys())}"
    )

# get map UUID
map_uuid = map_info["UUID"]
if sim.current_scene == map_uuid:
    sim.reset()
else:
    sim.load(map_uuid)
sim.set_time_of_day(env.int("TIME_OF_DAY"))

print(f"Map {MAP_NAME} with UUID {map_info['UUID']} loaded")

# get base start position
# [x, y, z, rx, ry, rz]
base_start_pos = map_info["BASE_START_POSE"]
dist_offset = -10

# Get initial velocity from config
INITIAL_VELOCITY = uuids.get("INITIAL_VELOCITY")
ANGULAR_VELOCITY = uuids.get("ANGULAR_VELOCITY")

egos = []
for idx in range(NUM_CARS):
    s = lgsvl.AgentState()
    ry = base_start_pos[4]
    s.position.x = base_start_pos[0] + dist_offset * idx * math.sin(math.radians(ry))
    s.position.y = base_start_pos[1]
    s.position.z = base_start_pos[2] + dist_offset * idx * math.cos(math.radians(ry))
    s.rotation.x = base_start_pos[3]
    s.rotation.y = base_start_pos[4]
    s.rotation.z = base_start_pos[5]
    s.velocity = lgsvl.Vector(INITIAL_VELOCITY[0], INITIAL_VELOCITY[1], INITIAL_VELOCITY[2])
    s.angular_velocity = lgsvl.Vector(ANGULAR_VELOCITY[0], ANGULAR_VELOCITY[1], ANGULAR_VELOCITY[2])
    # get sensor UUID
    sensor_uuid = uuids["SENSORS"].get(SENSORS[idx % len(SENSORS)], None)
    if sensor_uuid is None:
        raise ValueError(
            f"Sensor {SENSORS} not found in mapping config file.\n"
            + f"Available sensors: {list(uuids['SENSORS'].keys())}"
        )

    ego = sim.add_agent(
        name=sensor_uuid,
        agent_type=lgsvl.AgentType.EGO,
        state=s,
    )

    CAR_SIMULATOR_BRIDGE_PORT = SIMULATOR_BRIDGE_PORT + idx

    print(
        f"Spawned ego {idx} at {s.position}\n" + f"with rotation {s.rotation}\n"
        f"with sensor {sensor_uuid}\n"
        f"lgsvl_bridge {SIMULATOR_BRIDGE_HOST}:{CAR_SIMULATOR_BRIDGE_PORT}\n"
    )

    ego.connect_bridge(SIMULATOR_BRIDGE_HOST, CAR_SIMULATOR_BRIDGE_PORT)
    while not ego.bridge_connected:
        print(f"Waiting for ego {idx} to connect to ROS2 bridge")
        time.sleep(1)

    print(f"Ego {idx} connected to ROS2 bridge")
    egos.append(ego)

while True:
    try:
        sim.run(time_limit=env.float("STEP_TIME"))
    except KeyboardInterrupt:
        print("Keyboard interrupted. Exiting")
        exit()
