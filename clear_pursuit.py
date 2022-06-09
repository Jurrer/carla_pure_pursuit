#!/usr/bin/env python
# -*- coding: cp1250 -*-

from __future__ import print_function

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import carla
import math
from carla import ColorConverter as cc
from typing import Tuple
import pygame
import numpy as np

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


DISPLAY = None
WINDOW_SIZE = (1280, 720)  ## Zmiana wielko�ci wy�wietlanego okna


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
    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    render_img(surface)


def get_cam(world: carla.World,
            _parent: carla.Vehicle,
            dim: Tuple[int, int]
            ):
    # Zmieni� warto�ci x,z,[y],pitch �eby zmieni� pozycj� kamery
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


# liczenie odleg�o�ci mi�dzy punktami
def jak_daleko(pntLoc, vehLoc):
    return math.sqrt(pow(((pntLoc.x) - vehLoc.x), 2) + pow(((pntLoc.y) - vehLoc.y), 2))


def game_loop():
    actor_list = []
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(100.0)

        world = client.get_world()

        blueprint_library = world.get_blueprint_library()

        bp = blueprint_library.filter('model3')[0]

        spawn_point = world.get_map().get_spawn_points()[79]  # 82 najlepszy
        vehicle = world.spawn_actor(bp, spawn_point)

        actor_list.append(vehicle)

        cam = get_cam(world, vehicle, WINDOW_SIZE)
        actor_list.append(cam)

        map = world.get_map()

        odl_tyl_osi = -1.35  # odleg�o�� osi od �rodka samochodu
        gaz_max = 0.7  # ograniczenie maksymalnej warto�ci zadawanego gazu
        zas_widz_cel = 2  # maksymalny zasi�g widzenia, w kt�rym wyszukuje docelowy punkt
        zas_widz_pred = zas_widz_cel * 3  # zasi�g widzenia potrzebny do regulacji pr�dko�ci
        kat_glob_cel = 0  # k�t mi�dzy prost� ��cz�ca punkt docelowy z pocz�tkiem uk�adu wzgl�dnego a globaln� osi� "y"
        kat_glob_pred = 0  # k�t mi�dzy prost� ��cz�ca punkt regulacji pr�dko�ci z pocz�tkiem uk�adu wzgl�dnego a globaln� osi� "y"
        gain_kat_skret = 5  # przemno�enie k�ta krzywizny w celu uzyskania lepszego sterowania
        gain_zas_widz = 1  # dynamicznie regulowany gain zasi�gu widzenia

        lok_wysz_cel = vehicle.get_location()  # inicjalizacja punktu wyszukuj�cego cel
        lok_wysz_pred = vehicle.get_location()  # inicjalizacja punktu reguluj�cego pr�dko��
        lok_wzgl_cel = vehicle.get_location()  # inicjalizacja punktu celu ktory jest przeniesieniem waypointa do uk�adu wsp�rz�dnych opartego o samoch�d
        lok_wzgl_pred = vehicle.get_location()  # inicjalizacja punktu reguluj�cego pr�dko�� w lokalnym uk�adzie wsp�rz�dnych

        vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0))
        cel = map.get_waypoint(lok_wysz_cel, project_to_road=True, lane_type=(carla.LaneType.Driving))
        pred = map.get_waypoint(lok_wysz_pred, project_to_road=True, lane_type=(carla.LaneType.Driving))

        while True:
            world.wait_for_tick()

            sam_lok = vehicle.get_location()  # lokalizacja samochodu

            sam_kat = vehicle.get_transform().rotation.yaw  # kat skierowania samochodu
            sam_kat = math.radians(sam_kat)

            # przeniesienie lokalizacji samochodu na jego tyln� o�
            sam_lok.x = sam_lok.x + (odl_tyl_osi * math.cos(sam_kat))
            sam_lok.y = sam_lok.y + (odl_tyl_osi * math.sin(sam_kat))

            # obliczenie lokalizacji wyszukuj�cej punkt docelowy
            if kat_glob_cel != 0:
                lok_wysz_cel.x = sam_lok.x + (zas_widz_cel * gain_zas_widz * math.cos(sam_kat))
                lok_wysz_cel.y = sam_lok.y + (zas_widz_cel * gain_zas_widz * math.sin(sam_kat))

            # obliczenie lokalizacji wyszukuj�cej punkt reguluj�cy pr�dko��
            if kat_glob_pred != 0:
                lok_wysz_pred.x = sam_lok.x + (zas_widz_pred * gain_zas_widz * math.cos(kat_glob_pred))
                lok_wysz_pred.y = sam_lok.y + (zas_widz_pred * gain_zas_widz * math.sin(kat_glob_pred))
            else:
                lok_wysz_pred.x = sam_lok.x + (zas_widz_pred * gain_zas_widz * math.cos(sam_kat))
                lok_wysz_pred.y = sam_lok.y + (zas_widz_pred * gain_zas_widz * math.sin(sam_kat))

            # zdobycie lokalizacji punktu docelowego w okolicach rozgl�dania si� przez samoch�d
            typ_linii = carla.LaneType.Driving
            cel = map.get_waypoint(lok_wysz_cel, project_to_road=True, lane_type=typ_linii)

            # zdobycie lokalizacji punktu reguluj�cego pr�dko��
            pred = map.get_waypoint(lok_wysz_pred, project_to_road=True, lane_type=typ_linii)

            # obliczenie kata miedzy prosta ��cz�c� punkt docelowy z pocz�tkiem lokalnego uk�adu wsp�rz�dnych a osi� globaln� y
            kat_glob_cel = math.atan2(cel.transform.location.y - sam_lok.y, cel.transform.location.x - sam_lok.x)

            # obliczenie k�ta mi�dzy prost� ��cz�c� punkt reguluj�cy pr�dko�� z pocz�tkiem lokalnego uk�adu wsp�rz�dnych a osi� globaln� y
            kat_glob_pred = math.atan2(pred.transform.location.y - sam_lok.y, pred.transform.location.x - sam_lok.x)

            # obliczenie rzeczywistej odleg�o�ci mi�dzy pocz�tkiem lokalnego uk�adu wsp�rz�dnych a celem
            odl_cel = jak_daleko(cel.transform.location, sam_lok)

            # obliczenie rzeczywistej odleg�o�ci mi�dzy pocz�tkiem lokalnego uk�adu wsp�rz�dnych a punktem reg pr�dko��
            odl_pred = jak_daleko(pred.transform.location, sam_lok)

            # obliczenie warto�ci x i y celu w odniesieniu do lokalnego uk�adu wsp�rz�dnych
            lok_wzgl_cel.x = odl_cel * math.sin(kat_glob_cel - sam_kat)  # odleg�o�� x
            lok_wzgl_cel.y = odl_cel * math.cos(kat_glob_cel - sam_kat)  # odleg�o�� y

            # obliczenie warto�ci x i y punktu regulacji pr�dko�ci w odniesieniu do lokalnego uk�adu wsp�rz�dnych
            lok_wzgl_pred.x = odl_pred * math.sin(kat_glob_pred - sam_kat)  # odleglosc x
            lok_wzgl_pred.y = odl_pred * math.cos(kat_glob_pred - sam_kat)  # odleglosc y

            # kat krzywizny celu
            kat_krzyw_cel = 2 * lok_wzgl_cel.x / math.pow(odl_cel, 2) * gain_kat_skret

            # k�t krzywizny punktu regulacji pr�dko�ci
            kat_krzyw_pred = 2 * lok_wzgl_pred.x / math.pow(odl_pred, 2) * gain_kat_skret

            # sterowanie wraz konwersja kata obrotu kol na warto�� w zakresie -1 do 1
            kat_krzyw_cel = max(kat_krzyw_cel, -math.pi / 4)
            kat_krzyw_cel = min(kat_krzyw_cel, math.pi / 4)
            kat_krzyw_cel = kat_krzyw_cel * (4 / math.pi)

            # gazowanie(pradowanie?) i hamowanie wraz z konwersja na wartosci przyjmowane przez pojazd
            gaz = gaz_max - abs(math.sin(kat_krzyw_pred))

            if gaz < gaz_max * 0.8:
                ham = abs(math.sin(kat_krzyw_cel) * math.pow(gaz_max, 2))
            else:
                ham = 0

            # wysterowanie samochodu
            vehicle.apply_control(carla.VehicleControl(throttle=gaz, steer=kat_krzyw_cel, brake=ham))

            pred_war = vehicle.get_velocity()
            pred_war = math.sqrt((pred_war.x * pred_war.x) + (pred_war.y * pred_war.y) + (pred_war.z * pred_war.z))
            print(pred_war)

            # rysowanie punktow
            world.debug.draw_string(sam_lok, 'O', draw_shadow=False, color=carla.Color(r=255, g=0, b=0),
                                    life_time=0.1, persistent_lines=True)
            world.debug.draw_string(cel.transform.location, 'O', draw_shadow=False, color=carla.Color(r=0, g=255, b=0),
                                    life_time=0.1, persistent_lines=True)
            world.debug.draw_string(lok_wysz_cel, 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=255),
                                    life_time=0.1, persistent_lines=True)
            world.debug.draw_string(pred.transform.location, 'O', draw_shadow=False, color=carla.Color(r=0, g=255, b=0),
                                    life_time=0.1, persistent_lines=True)
            world.debug.draw_string(lok_wysz_pred, 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=255),
                                    life_time=0.1, persistent_lines=True)
            # for j in gp:
            #    world.debug.draw_string(j.transform.location, 'O', draw_shadow=False, color=carla.Color(r=255, g=255, b=0), life_time=0.1, persistent_lines=True)

            gain_zas_widz = pow(max(pred_war / 10, 1), 2)
            # print(gain_zas_widz)
            # gain_zas_widz = 1
            pygame.display.flip()

    finally:
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        print('done.\n\n')


if __name__ == '__main__':
    game_loop()
