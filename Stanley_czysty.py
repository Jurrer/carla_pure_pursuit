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



def game_loop():
    actor_list = []
    try:
        client = carla.Client('localhost',2000) #tutaj wpisujemy adres ip komputera jako nasz serwer
        #jesli robimy to na innym komputerze
        client.set_timeout(10.0) #tutaj wprowadzamy timeout
        #mozemy zwiekszyc jesli wyskoczy blad, wtedy np 20-50 sekund
    
        world = client.get_world() #odczytujemy obecna mape

        blueprint_library = world.get_blueprint_library() #biblioteka z pojazdami i pogoda
        
    
        bp = blueprint_library.filter('model3')[0] #dodanie pojazdu
        #filter przeszukuje mape i zwraa tablice
    
        spawn_point = world.get_map().get_spawn_points()[38] #38 najlepszy - punkty na mapie 
        #decydujemy gdzie chcemy dodac pojazd na mapie
        #bierzemy cala mape i szukamy punktow na niej
        
        vehicle = world.spawn_actor(bp, spawn_point) #dodajemy pojazdy, pieszych itd na mape
        actor_list.append(vehicle)

        map = world.get_map() #wczytywanie aktualnej mapy
        odl = 1.8
        thr_val = 0.7
        kn = 1
        kd = 0
        wp = map.get_waypoint(vehicle.get_location(), project_to_road=True, lane_type=(carla.LaneType.Driving))
        #dostajemy tytaj waypont, podajemy arguemnty, podajemy punkt trasy/pobocze, mamy linie, ktora wytycza pas ruchu
        vehicle.apply_control(carla.VehicleControl(throttle = thr_val, steer = 0.0))
        #wartosc throttle musimy znalezc w dokumentacji, zeby wiedziec jaka dac predkosc i kat
        
        # waypoint_previous(wp), waypoint_next(wn), vehicle_location(car), vehicle_velocity(cer_vel), throttle_value(thr_val)
        # cross_track_error(e), cross_track_steering(es), heding_error(he)
        

        while True:
            world.wait_for_tick() #dopoki skrypt nie otrzyma kolejnego ticku
            #to bedzie czekal, a pozniej serwer znowu wysyla tick
            
            car_loc = vehicle.get_location() #daje lokalizacje
            car_yaw = vehicle.get_transform().rotation.yaw #daje rotacje poszczegolnych katow
                
                

            car_loc.x = car_loc.x + (odl*math.cos(math.radians(car_yaw)))
            car_loc.y = car_loc.y + (odl*math.sin(math.radians(car_yaw)))



            car_vel = vehicle.get_velocity()
            car_vel = math.sqrt((car_vel.x*car_vel.x) + (car_vel.y*car_vel.y) + (car_vel.z*car_vel.z)) # [m/s]



            wn = map.get_waypoint(car_loc, project_to_road=True, lane_type=(carla.LaneType.Driving))
            #daje nam waypoint najblizej naszej lokalizacji 


            a = (wn.transform.location.y - wp.transform.location.y)/(wn.transform.location.x - wp.transform.location.x + 0.000000001)
            b = wp.transform.location.y - (wp.transform.location.x * a)

            e = ((a*car_loc.x) - car_loc.y + b) / math.sqrt((a*a) + 1)



            waypoint_yaw_n = wn.transform.rotation.yaw
            waypoint_yaw_p = wp.transform.rotation.yaw



            # Normalizacja yaw na zakres 0-360
            if car_yaw < 0:
                car_yaw = 360+car_yaw
              
            if waypoint_yaw_n < 0:
                waypoint_yaw_n = 360+waypoint_yaw_n

            if waypoint_yaw_p < 0:
                waypoint_yaw_p = 360+waypoint_yaw_p



            # Normalizacja strony po ktorej sie znajdujemy wzgledem prostej
            if car_yaw>180 and car_yaw<=360:
                if a>0:
                    e = e * (-1)
                
            if car_yaw>0 and car_yaw<=180:
                if a<0:
                    e = e * (-1)



            if car_vel == 0:
                es = 0
            else:
                es = math.atan((kn*e)/(car_vel+kd))



            he = waypoint_yaw_n - car_yaw

            if he < 0:
                skret1 = he
                skret2 = 360+he
                
            else:
                skret1 = he
                skret2 = -(360-he)
            

            if (abs(skret1) < abs(skret2)):
                he = skret1
            else:
                he = skret2

            he = (he * 2 * math.pi)/360


            steer_input = he + es
            
            vehicle.apply_control(carla.VehicleControl(throttle = thr_val, steer = steer_input))
            

            #print(e, '\n')
            world.debug.draw_string(car_loc, 'O', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=0.1, persistent_lines=True)
            world.debug.draw_string(wn.transform.location, 'O', draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=0.1, persistent_lines=True)
            world.debug.draw_string(wp.transform.location, 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=0.1, persistent_lines=True)
            #tutaj rysujemy niebieskie kolko wokol pojazdu
            
            wp = wn


     
    finally:
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


#cd F:\WindowsNoEditor
#.\CarlaUE4

#cd F:\WindowsNoEditor\pythonapi\util
#python config.py -m Town07
#cd F:\WindowsNoEditor\PythonAPI\examples
#python Stanley_czysty.py
