#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Spawn NPCs into the simulation"""

import glob
import os
import sys
import time
import getopt

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import argparse
import logging
from numpy import random

def main(argv):
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=10,
        type=int,
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=50,
        type=int,
        help='number of walkers (default: 50)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='vehicles filter (default: "vehicle.*")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='pedestrians filter (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Enanble')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        type=int,
        help='Random device seed')
    argparser.add_argument(
        '--car-lights-on',
        action='store_true',
        default=False,
        help='Enanble car lights')
    argparser.add_argument(
        '--scene',
        default='long',
        help='scene <long/compare>')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        world = client.get_world()

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(1.0)
        if args.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)


        if args.sync:
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
                world.apply_settings(settings)
            else:
                synchronous_master = False

        blueprints = world.get_blueprint_library().filter('vehicle.nissan.*')

        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
            blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('t2')]

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = []

        if args.scene == 'long':
            spawn_points.append(carla.Transform(carla.Location(x=103.32, y=306.74, z=0.4), carla.Rotation(yaw=0)))
            spawn_points.append(carla.Transform(carla.Location(x=83.32, y=306.74, z=0.4), carla.Rotation(yaw=0)))

            spawn_points.append(carla.Transform(carla.Location(x=193.9, y=230.74, z=0.4), carla.Rotation(yaw=-90)))
            spawn_points.append(carla.Transform(carla.Location(x=192.9, y=190.74, z=0.4), carla.Rotation(yaw=240)))
            spawn_points.append(carla.Transform(carla.Location(x=189.6, y=210.74, z=0.4), carla.Rotation(yaw=90)))

            spawn_points.append(carla.Transform(carla.Location(x=153.4, y=105.6, z=0.4), carla.Rotation(yaw=180)))
            spawn_points.append(carla.Transform(carla.Location(x=133.4, y=105.6, z=0.4), carla.Rotation(yaw=180)))
            spawn_points.append(carla.Transform(carla.Location(x=113.4, y=105.6, z=0.4), carla.Rotation(yaw=180)))
        elif args.scene == 'compare':
            spawn_points.append(carla.Transform(carla.Location(x=72.32, y=306.74, z=0.4), carla.Rotation(yaw=0)))


        # transform6 = carla.Transform(carla.Location(x=83.32, y=306.74, z=0.4), carla.Rotation(yaw=0))

        number_of_spawn_points = len(spawn_points)

        # if args.number_of_vehicles < number_of_spawn_points:
        #     random.shuffle(spawn_points)
        # elif args.number_of_vehicles > number_of_spawn_points:
        #     msg = 'requested %d vehicles, but could only find %d spawn points'
        #     logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
        #     args.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        SetVehicleLightState = carla.command.SetVehicleLightState
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')

            # prepare the light state of the cars to spawn
            light_state = vls.NONE
            if args.car_lights_on:
                light_state = vls.Position | vls.LowBeam | vls.LowBeam

            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                # .then(SetAutopilot(FutureActor, True, traffic_manager.get_port()))
                .then(SetVehicleLightState(FutureActor, light_state)))

        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        print('spawned %d vehicles, press Ctrl+C to exit.' % (len(vehicles_list)))

        # example of how to use parameters
        traffic_manager.global_percentage_speed_difference(30.0)

        while True:
            if args.sync and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()

    finally:

        if args.sync and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

        time.sleep(0.5)

if __name__ == '__main__':

    try:
        main(sys.argv[1:])
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
