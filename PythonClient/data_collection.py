#!/usr/bin/env python3

# Copyright (c) 2019, Intelligent Robotics Lab, DLUT.
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Automatically annotated LiDAR point cloud extraction example."""

from __future__ import print_function

import argparse
import logging
import random
import time

from carla.client import make_carla_client
from carla.sensor import Camera, Lidar
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.util import print_over_same_line

import numpy
import os
import math


# compute distance between 2D points.
def distance2D(pos1, pos2):
    xDiff = pos1[0] - pos2[0]
    yDiff = pos1[1] - pos2[1]
    return math.sqrt(xDiff ** 2 + yDiff ** 2)


def run_carla_client(args):
    # Here we collect 300 frames of annotated point clouds. The minimum distance
    # each two frames is 5.0 meters.
    max_frames = 300
    min_distance_between_frames = 5.0

    # We assume the CARLA server is already waiting for a client to connect at
    # host:port. To create a connection we can use the `make_carla_client`
    # context manager, it creates a CARLA client object and starts the
    # connection. It will throw an exception if something goes wrong. The
    # context manager makes sure the connection is always cleaned up on exit.
    with make_carla_client(args.host, args.port) as client:
        print('CarlaClient connected')

        count = 0
        pos_list = []
        episode = -1

        while count < max_frames:
            episode += 1
            # Start a new episode.

            if args.settings_filepath is None:

                print('Create a CarlaSettings object...')

                # Create a CarlaSettings object. This object is a wrapper around
                # the CarlaSettings.ini file. Here we set the configuration we
                # want for the new episode.
                settings = CarlaSettings()
                settings.set(
                    SynchronousMode=False,
                    SendNonPlayerAgentsInfo=True,
                    NumberOfVehicles=40,
                    NumberOfPedestrians=100,
                    WeatherId=1, #random.choice([1, 3, 7, 8, 14]),
                    QualityLevel=args.quality_level)
                settings.randomize_seeds()

                # The default camera captures RGB images of the scene.
                camera0 = Camera('CameraRGB', PostProcessing='SceneFinal')
                # Set image resolution in pixels.
                camera0.set_image_size(640, 480)
                # Set its position relative to the car in meters.
                camera0.set_position(2.0, 0, 1.4)
                settings.add_sensor(camera0)

                camera1 = Camera('CameraDepth', PostProcessing='Depth')
                camera1.set_image_size(640, 480)
                camera1.set_position(2.0, 0, 1.4)
                settings.add_sensor(camera1)

                # To get labels for point clouds, a camera with 'SemanticSegmentation'
                # postprocessing has to be added for now.
                camera2 = Camera('CameraSemSeg', PostProcessing='SemanticSegmentation')
                camera2.set_image_size(640, 480)
                camera2.set_position(2.0, 0, 1.4)
                settings.add_sensor(camera2)

                # Velodyne HDL64 settings
                lidar = Lidar('Lidar64')
                lidar.set_position(0, 0, 1.73)
                lidar.set_rotation(0, 0, 0)
                lidar.set(
                        Channels=64,
                        Range=100,
                        PointsPerSecond=2560000,
                        RotationFrequency=10,
                        UpperFovLimit=2.0,
                        LowerFovLimit=-24.8)
                settings.add_sensor(lidar)

            else:
                # Alternatively, we can load these settings from a file.
                with open(args.settings_filepath, 'r') as fp:
                    settings = fp.read()

            # Now we load these settings into the server. The server replies
            # with a scene description containing the available start spots for
            # the player. Here we can provide a CarlaSettings object or a
            # CarlaSettings.ini file as string.
            scene = client.load_settings(settings)

            # Choose one player start at random.
            number_of_player_starts = len(scene.player_start_spots)
            player_start = random.randint(0, max(0, number_of_player_starts - 1))

            # Notify the server that we want to start the episode at the
            # player_start index. This function blocks until the server is ready
            # to start the episode.
            print('Starting new episode...')
            client.start_episode(player_start)

            # waiting for initialization
            for i in range(20):
                measurements, sensor_data = client.read_data()
                print('initialization')
                control = measurements.player_measurements.autopilot_control
                control.steer += random.uniform(-0.1, 0.1)
                client.send_control(control)

            # We consider the simulation is blocked by unexpected things, e.g. a car crashes
            # into other objects, when the vehicle runs at a low speed in a certain number of
            # periods. Then the simulation is restarted. 
            low_speed = 1e-4
            low_speed_count = 0
            low_speed_limit = 500

            # Iterate every frame in the episode.
            while count < max_frames:

                # Read the data produced by the server this frame.
                measurements, sensor_data = client.read_data()
                player_measurements = measurements.player_measurements

                # when not move for a while, stop this episode
                if player_measurements.forward_speed < low_speed:
                    low_speed_count += 1
                else:
                    low_speed_count = 0

                if low_speed_count > low_speed_limit:
                    break

                filename = args.out_log_filename_format.format(episode)
                # Create folder to save if does not exist.
                folder = os.path.dirname(filename)
                if not os.path.isdir(folder):
                    os.makedirs(folder)

                log_info = '{:d} {:d}\n'.format(count, low_speed_count)
                with open(filename, 'a') as out:
                    out.write(log_info)

                # If the distance between current location and every saved measurement location
                # is larger than min_distance_between_frames, then current measurement will be 
                # saved and current location will be added the pos_list.
                pos_cur = (player_measurements.transform.location.x,
                            player_measurements.transform.location.y)
                is_new = True
                for p in pos_list:
                    if distance2D(pos_cur, p) < min_distance_between_frames:
                        is_new = False
                        break

                if is_new:
                    # Print some of the measurements.
                    print_measurements(measurements)

                    # Save the images and point clouds to disk if requested.
                    for name, measurement in sensor_data.items():
                        filename = args.out_filename_format.format(episode, name, count)
                        measurement.save_to_disk(filename)

                    pos_list.append(pos_cur)
                    count += 1

                    # Save poses to file.
                    pos_string = '{:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.0f}\n'.format(
                            player_measurements.transform.location.x,
                            player_measurements.transform.location.y,
                            player_measurements.transform.location.z,
                            player_measurements.transform.rotation.pitch,
                            player_measurements.transform.rotation.yaw,
                            player_measurements.transform.rotation.roll,
                            player_measurements.forward_speed * 3.6)

                    # Open the file and save .
                    filename = args.out_pose_filename_format.format(episode)
                    with open(filename, 'a') as ply_file:
                        ply_file.write(pos_string)
                        

                # Now we have to send the instructions to control the vehicle.
                # Together with the measurements, the server has sent the
                # control that the in-game autopilot would do this frame. We
                # can enable autopilot by sending back this control to the
                # server. We can modify it if wanted, here for instance we
                # will add some noise to the steer.

                print('autopilot control')
                control = measurements.player_measurements.autopilot_control
                client.send_control(control)


# print some of the measurements
def print_measurements(measurements):
    number_of_agents = len(measurements.non_player_agents)
    player_measurements = measurements.player_measurements
    message = 'Vehicle at ({pos_x:.1f}, {pos_y:.1f}), '
    message += '{speed:.0f} km/h, '
    message += 'Collision: {{vehicles={col_cars:.0f}, pedestrians={col_ped:.0f}, other={col_other:.0f}}}, '
    message += '{other_lane:.0f}% other lane, {offroad:.0f}% off-road, '
    message += '({agents_num:d} non-player agents in the scene)'
    message = message.format(
        pos_x=player_measurements.transform.location.x,
        pos_y=player_measurements.transform.location.y,
        speed=player_measurements.forward_speed * 3.6, # m/s -> km/h
        col_cars=player_measurements.collision_vehicles,
        col_ped=player_measurements.collision_pedestrians,
        col_other=player_measurements.collision_other,
        other_lane=100 * player_measurements.intersection_otherlane,
        offroad=100 * player_measurements.intersection_offroad,
        agents_num=number_of_agents)
    print_over_same_line(message)


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-q', '--quality-level',
        choices=['Low', 'Epic'],
        type=lambda s: s.title(),
        default='Epic',
        help='graphics quality level, a lower level makes the simulation run considerably faster.')
    argparser.add_argument(
        '-c', '--carla-settings',
        metavar='PATH',
        dest='settings_filepath',
        default=None,
        help='Path to a "CarlaSettings.ini" file')

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    args.out_filename_format = './data_collection/episode_{:0>4d}/{:s}/{:0>6d}'
    args.out_pose_filename_format = './data_collection/episode_{:0>4d}/pose.txt'
    args.out_log_filename_format = './data_collection/episode_{:0>4d}/log.txt'

    while True:
        try:

            run_carla_client(args)

            print('Done.')
            return

        except TCPConnectionError as error:
            logging.error(error)
            time.sleep(1)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
