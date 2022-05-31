#!/usr/bin/env python
# -*- coding: cp1250 -*-

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
        client = carla.Client('localhost',2000)
        client.set_timeout(100.0)
    
        world = client.get_world()

        blueprint_library = world.get_blueprint_library()
    
        bp = blueprint_library.filter('model3')[0]
    
        spawn_point = world.get_map().get_spawn_points()[38] #38 najlepszy
        vehicle = world.spawn_actor(bp, spawn_point)
        
        actor_list.append(vehicle)

        #co potrzeba do pure pursuit:
        # polozenia ukladu odniesienia (srodek tylnej osi, obrocenie tak jak obrocenie samochodu)
        # punkty do ktorych ma jechac samochod
        # lookahead distance (pozwala on na znalezienie najdalszego goal pointu na podstawie ktorego bedziemy liczyli kat skretu
        
        
        
        map = world.get_map()
        odl = -1.4
        thr_val = 0.5
        kn = 1
        kd = 0
        wp = map.get_waypoint(vehicle.get_location(), project_to_road=True, lane_type=(carla.LaneType.Driving))
        vehicle.apply_control(carla.VehicleControl(throttle = thr_val, steer = 0.0))
        
        goal_transformed = wp
        tymczasowa_zmienna = vehicle.get_location()
        tymczasowa_zmienna.x = tymczasowa_zmienna.x+15
        tymczasowa_zmienna.y = tymczasowa_zmienna.y+20
        # waypoint_previous(wp), waypoint_next(wn), vehicle_location(car), vehicle_velocity(cer_vel), throttle_value(thr_val)
        # cross_track_error(e), cross_track_steering(es), heding_error(he)


        
        # (chyba do poprawy) dlaczego poprzedni waypoint jest wczytywany tylko raz?                                                         ///to jest pierwszy waypoint, pewnie bez niego program si� wykrzacza� czy co�
        while True:
            world.wait_for_tick() #czekamy na info
            
            car_loc = vehicle.get_location() #lokalizacja samochodu
            car_yaw = vehicle.get_transform().rotation.yaw #kat skierowania samochodu
                
            kat = math.atan2(tymczasowa_zmienna.y-car_loc.y, tymczasowa_zmienna.x-car_loc.x)

            car_loc.x = car_loc.x + (odl*math.cos(math.radians(car_yaw))) #przeniesienie lokalizacji samochodu na jego tylnia os
            car_loc.y = car_loc.y + (odl*math.sin(math.radians(car_yaw))) #przeniesienie lokalizacji samochodu na jego tylnia os
            print(car_loc.x, car_loc.y, wp.transform.location.x, wp.transform.location.y, car_yaw)
            
            l = math.sqrt(pow((( tymczasowa_zmienna.x)-car_loc.x),2)+pow(((tymczasowa_zmienna.y)-car_loc.y),2))

            goal_transformed.x =l*math.sin(kat-math.radians(car_yaw))
            goal_transformed.y =l*math.cos(kat-math.radians(car_yaw))
            # print(goal_transformed.x,  goal_transformed.y)

            # po tej linijce mamy ju� polozenie naszego ukladu odniesienia wraz z katem obrotu wzgledem globalnego ukladu odniesienia

            #get waypoints i najblizszy punkt drogi do pokonania a nastepnie idac dalej po tej drodze znajdz najdalszy punkt drogi lapiacy sie do lookahead distance on bedzie goalem     //fajnie jakby waypointy by�y o wiele rzadziej, teraz s� bardzo g�ste
            #the_goal to jest nasz cel zawierajacy w sobie lokalizacje globalna wrzucic wyznaczanie celu w funkcje osobna aby bylo czysto

            #(tutaj albo po funkcji?) update polozenia samochodu

            #tranformacja globalnej lokalizacji na lokalizacje wzgledem naszego stworzonego ukladu odniesienia
            #znowu osobna funkcja na to
            
            #obliczenie kurwatury skretu
            #konwersja kurwatury skretu na odpowiedni steering input

            #(tutaj albo przed funkcja?) update polozenia samochodu


            #vel nas nie obchodziz chwilowo
            car_vel = vehicle.get_velocity() #informacja o pradkosci
            car_vel = math.sqrt((car_vel.x*car_vel.x) + (car_vel.y*car_vel.y) + (car_vel.z*car_vel.z)) # modul predkosci[m/s]



            wn = map.get_waypoint(car_loc, project_to_road=True, lane_type=(carla.LaneType.Driving)) #(do sprawdzenia) stworzenie waypointa opisujacego umiejscowienie samochodu


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
            #(tutaj albo przed funkcja?) update polozenia samochodu

            #print(e, '\n')
            world.debug.draw_string(car_loc, 'O', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=0.1, persistent_lines=True)
            world.debug.draw_string(wn.transform.location, 'O', draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=0.1, persistent_lines=True)
            world.debug.draw_string(wp.transform.location, 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=0.1, persistent_lines=True)
            world.debug.draw_string(tymczasowa_zmienna, 'O', draw_shadow=False, color=carla.Color(r=0, g=255, b=255), life_time=0.1, persistent_lines=True)
            # world.debug.draw_string(niewiemcosiedzieje.transform.location, 'O', draw_shadow=False, color=carla.Color(r=150, g=8, b=75), life_time=0.1, persistent_lines=True)
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
