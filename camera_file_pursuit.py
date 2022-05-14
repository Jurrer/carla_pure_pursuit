#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.


from __future__ import print_function

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import csv

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla
import time
import random
import math

from carla import ColorConverter as cc
from typing import Tuple
import pygame
import numpy as np

DISPLAY = None  ## Nie zmieniać!
WINDOW_SIZE = (1280, 720)  ## Można zmieniać żeby mieć inne okienko


def get_display(dim: Tuple[int, int]):
    global DISPLAY
    DISPLAY = pygame.display.set_mode(
        (dim[0], dim[1]),
        pygame.HWSURFACE | pygame.DOUBLEBUF)
    DISPLAY.fill((0, 0, 0))
    pygame.display.flip()


def render_img(surf):
    global DISPLAY  ##!!!
    if surf is not None:
        DISPLAY.blit(surf, (0, 0))


def parse_image(img, color_conversion):
    img.convert(color_conversion)
    array = np.frombuffer(img.raw_data, dtype=np.uint8)
    array = np.reshape(array, (img.height, img.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))  ## NOTE Swapaxes? Why?
    render_img(surface)


def get_cam(world: carla.World,
            _parent: carla.Vehicle,
            dim: Tuple[int, int]
            ):
    # Zmienić wartości x,z,[y],pitch żeby zmienić pozycję kamery
    camera_transform: Tuple[carla.Transform, carla.AttachmentType] = \
        (carla.Transform(carla.Location(x=-5.5, z=1.75), carla.Rotation(pitch=0.5)),
         carla.AttachmentType.Rigid)

    sensor_data = ['sensor.camera.rgb', cc.Raw]

    blueprint_library = world.get_blueprint_library()
    blueprint = blueprint_library.find(sensor_data[0])

    blueprint.set_attribute('image_size_x', str(dim[0]))
    blueprint.set_attribute('image_size_y', str(dim[1]))
    if blueprint.has_attribute('gamma'):
        blueprint.set_attribute('gamma', str(2.2))

    sensor = world.spawn_actor(
        blueprint,
        camera_transform[0],
        attach_to=_parent,
        attachment_type=camera_transform[1]
    )

    sensor.listen(lambda image: parse_image(image, sensor_data[1]))
    return sensor


get_display(WINDOW_SIZE)


def game_loop():
    actor_list = []
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        world = client.get_world()

        blueprint_library = world.get_blueprint_library()

        bp = blueprint_library.filter('model3')[0]

        spawn_point = world.get_map().get_spawn_points()[34]  # 38 najlepszy
        vehicle = world.spawn_actor(bp, spawn_point)

        actor_list.append(vehicle)

        cam = get_cam(world, vehicle, WINDOW_SIZE)
        actor_list.append(cam)

        map = world.get_map()
        odl = 1.8
        thr_val = 0.7
        kn = 1
        kd = 0
        wp = map.get_waypoint(vehicle.get_location(), project_to_road=True, lane_type=(carla.LaneType.Driving))
        vehicle.apply_control(carla.VehicleControl(throttle=thr_val, steer=0.0))

        # waypoint_previous(wp), waypoint_next(wn), vehicle_location(car), vehicle_velocity(cer_vel), throttle_value(thr_val)
        # cross_track_error(e), cross_track_steering(es), heding_error(he)

        with open('Przejazd_16.csv', mode='w') as moj_plik:
            zapis = csv.writer(moj_plik, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            zapis.writerow(
                ['Car x', 'Car y', 'Car z', 'Car yaw', 'Car vel', 'Wn x', 'Wn y', 'Wn z', 'Wn yaw', 'Wp x', 'Wp y',
                 'Wp z', 'Wp yaw', 'a', 'b', 'e', 'es', 'he', 'steer_input'])

            while True:
                world.wait_for_tick()
                pygame.event.pump()

                car_loc = vehicle.get_location()
                car_yaw = vehicle.get_transform().rotation.yaw

                car_loc.x = car_loc.x + (odl * math.cos(math.radians(car_yaw)))
                car_loc.y = car_loc.y + (odl * math.sin(math.radians(car_yaw)))

                car_vel = vehicle.get_velocity()
                car_vel = math.sqrt(
                    (car_vel.x * car_vel.x) + (car_vel.y * car_vel.y) + (car_vel.z * car_vel.z))  # [m/s]

                wn = map.get_waypoint(car_loc, project_to_road=True, lane_type=(carla.LaneType.Driving))

                a = (wn.transform.location.y - wp.transform.location.y) / (
                            wn.transform.location.x - wp.transform.location.x + 0.000000001)
                b = wp.transform.location.y - (wp.transform.location.x * a)

                e = ((a * car_loc.x) - car_loc.y + b) / math.sqrt((a * a) + 1)

                waypoint_yaw_n = wn.transform.rotation.yaw
                waypoint_yaw_p = wp.transform.rotation.yaw

                # Normalizacja yaw na zakres 0-360
                if car_yaw < 0:
                    car_yaw = 360 + car_yaw

                if waypoint_yaw_n < 0:
                    waypoint_yaw_n = 360 + waypoint_yaw_n

                if waypoint_yaw_p < 0:
                    waypoint_yaw_p = 360 + waypoint_yaw_p

                # Normalizacja strony po ktorej sie znajdujemy wzgledem prostej
                if car_yaw > 180 and car_yaw <= 360:
                    if a > 0:
                        e = e * (-1)

                if car_yaw > 0 and car_yaw <= 180:
                    if a < 0:
                        e = e * (-1)

                if car_vel == 0:
                    es = 0
                else:
                    es = math.atan((kn * e) / (car_vel + kd))

                he = waypoint_yaw_n - car_yaw

                if he < 0:
                    skret1 = he
                    skret2 = 360 + he

                else:
                    skret1 = he
                    skret2 = -(360 - he)

                if (abs(skret1) < abs(skret2)):
                    he = skret1
                else:
                    he = skret2

                he = (he * 2 * math.pi) / 360

                steer_input = he + es

                vehicle.apply_control(carla.VehicleControl(throttle=thr_val, steer=steer_input))

                # print(e, '\n')
                world.debug.draw_string(car_loc, 'O', draw_shadow=False, color=carla.Color(r=255, g=0, b=0),
                                        life_time=0.1, persistent_lines=True)
                world.debug.draw_string(wn.transform.location, 'O', draw_shadow=False,
                                        color=carla.Color(r=0, g=255, b=0), life_time=0.1, persistent_lines=True)
                world.debug.draw_string(wp.transform.location, 'O', draw_shadow=False,
                                        color=carla.Color(r=0, g=0, b=255), life_time=0.1, persistent_lines=True)

                zapis.writerow([car_loc.x, car_loc.y, car_loc.z, car_yaw, car_vel, wn.transform.location.x,
                                wn.transform.location.y, wn.transform.location.z, waypoint_yaw_n,
                                wp.transform.location.x, wp.transform.location.y, wp.transform.location.z,
                                waypoint_yaw_p, a, b, e, es, he, steer_input])

                wp = wn
                pygame.display.flip()

    finally:
        moj_plik.close()

        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        print('done.\n\n')

    # ==============================================================================


# -- main() --------------------------------------------------------------------
# ==============================================================================

def main():
    game_loop()


if __name__ == '__main__':
    main()