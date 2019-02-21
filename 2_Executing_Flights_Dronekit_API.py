#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
READ ME: IN ORDER FOR THIS TOOL TO WORK ON YOUR OWN COMPUTER...

1) SHAPEFILES FOR ALL MISSIONS MUST BE LOCATED IN A COMMON FOLDER WHERE EACH MISSION IS NAMED BY 
THE CONVENTION "m{#}.shp". {#} STARTS AT 1 AND GOES UP TO THE TOTAL NUMBER OF MISSIONS.

2) SET THE 'missions_path' VARIABLE BELOW TO THE PATH OF THE FOLDER ON YOUR OWN COMPUTER
THAT CONTAINS THE SHAPEFILE MISSIONS.

3) SET THE 'lz_path' VARIABLE BELOW TO THE PATH OF THE LANDING ZONE SHAPEFILE ON YOUR OWN COMPUTER

4) SET THE 'csv_path' VARIABLE BELOW MUST TO THE PATH OF THE FOLDER ON YOUR OWN COMPUTER CONTAINING
THE .CSV OF FLIGHTS

5) SET THE 'total_missions' VARIABLE BELOW TO THE TOTAL NUMBER OF SHAPEFILE MISSIONS IN THE 
SHAPEFILE MISSIONS FOLDER

6) SET THE 'tcp1', 'tcp2', AND 'tcp3' VARIABLES BELOW TO 3 AVAILABLE TCP PORTS TO BE USED FOR 
THE SITL FLIGHT SIMULATOR. AVAILABLE TCP PORTS CAN BE DISCOVERED ON A MAC VIA THE 'NETWORK UTILITY' APPLICATION. 
NETWORK UTILITY --> PORT SCAN (TAB) --> INTERNET ADDRESS: 127.0.0.1 --> "TEST PORTS BETWEEN 5760 AND 6160" --> 
SELECT 3 AVAILABLE PORTS.  SIMULATOR WILL FAIL IF PORTS ARE UNAVAILABLE!

7) ALL DEPENDENCIES MUST BE INSTALLED (PYTHON 2).
    - GDAL/OGR (https://pypi.org/project/GDAL/)
    - DRONEKIT + DRONEKIT-SITL (http://python.dronekit.io/guide/quick_start.html)
    - PYMAVLINK (https://pypi.org/project/pymavlink/)
    - (os/csv/math/time/argparse)
"""

#********************************************************************************
# TO DO!
# Set path to folder containing shapefiles of missions:
missions_path = "/Users/dylancicero/Desktop/Missions_Renamed"
# Set path to folder containing shapefile of landing zones:
lz_path = "/Users/dylancicero/Desktop/Landing_Zone"
# Set path to folder containing the .csv output file of flights (generated from PyQGIS tool):
csv_path = "/Users/dylancicero/Desktop/"
# Set total number of missions:
total_missions = 31
# Set 3 open TCP ports on your computer to be used for the SITL flight simulator:
tcp1 = 'tcp:127.0.0.1:6160'
tcp2 = 'tcp:127.0.0.1:6162'
tcp3 = 'tcp:127.0.0.1:6163'
#********************************************************************************


# Import modules
import ogr
from os import chdir
import csv
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from dronekit_sitl import SITL
import time
import math
from pymavlink import mavutil


#********************************************************************************
# THIS SECTION CREATES DICTIONARY OBJECTS POPULATED WITH ALL INFORMATION FROM THE SHAPEFILES OF MISSIONS
# AND THE SHAPFILE OF LANDING ZONES.  THIS INFORMATION WILL LATER BE USED AS INPUT INTO DRONE FLIGHT COMMANDS.

# Set directory to folder containing shapefiles of missions
chdir(missions_path)

# Get the field names of attributes in shapefiles of missions
driver = ogr.GetDriverByName("ESRI Shapefile")
field_names = []
shapefile = "m1.shp"
dataSource = driver.Open(shapefile, 0)
layer = dataSource.GetLayer()
layerDefinition = layer.GetLayerDefn()
for i in range(layerDefinition.GetFieldCount()):
    field_names.append(layerDefinition.GetFieldDefn(i).GetName())

# Store the attributes for each feature (sortie) for each mission in a dictionary object 'all_info'
shapefiles = []
for i in range(total_missions):
    shapefile = "m" + str(i+1) + ".shp"
    shapefiles.append(shapefile)
all_info = {}
for i in range(len(shapefiles)):
    dataSource = driver.Open(shapefiles[i], 0)
    # Check to see if shapefile is found.
    if dataSource is None:
        print 'Could not open %s' % (shapefile)
    else:
        layer = dataSource.GetLayer()
        temp_dict2 = {}
        j = 0
        for feature in layer:
            temp_dict3 = {}
            for k in range(len(field_names)):
                temp_dict3[field_names[k]] = feature.GetField(field_names[k])
            temp_dict2["feature{0}".format(j)] = temp_dict3
            j = j + 1
        layer.ResetReading()
        all_info["m{0}".format(i+1)] = temp_dict2

# Set directory to folder containing landing zones shapefile
chdir(lz_path)

# Get the field names of attributes in landing zones shapefile
field_names_landing = []
shapefile = "LZ_points_WGS.shp"
dataSource = driver.Open(shapefile, 0)
layer = dataSource.GetLayer()
layerDefinition = layer.GetLayerDefn()
for i in range(layerDefinition.GetFieldCount()):
    field_names_landing.append(layerDefinition.GetFieldDefn(i).GetName())

# Store the attributes for each landing zone in a dictionary object 'landing_info'
landing_info = {}
if dataSource is None:
    print 'Could not open %s' % (shapefile)
else:
    landing_features = {}
    for j in range(len(layer)):
        temp_dict3 = {}
        for k in range(len(field_names_landing)):
            temp_dict3[field_names_landing[k]] = layer[j].GetField(field_names_landing[k])
        layer.ResetReading()
        landing_info['LZ' + str(j+1)] = temp_dict3


#********************************************************************************
# THIS SECTION READS THE GENERATED CSV OF FLIGHTS AND ASKS THE OPERATOR TO SELECT A FLIGHT FOR EXECUTION

# Set directory to folder containing the .csv output file of flights
chdir(csv_path)
        
# Create a dictionary object 'flights_dict' {key:value} where key is equal to the flight 
# name and the value is equal to a list of missions grouped for that flight
csv_list = []
with open('flights.csv', "r") as file:
    reader = csv.reader(file)
    for row in reader:
        csv_list.append(list(row))
flights_dict = {}
for i in range(1, len(csv_list)):
    flights_dict[csv_list[i][0]] = csv_list[i][1:4]
    flights_dict

# Initiate some helper variables
selected = []
selected_dict = {}
mission_aborted = {}
counter = -1

# While loop causes script to continuously ask the operator to select a new flight for execution
# until all flights have been executed.
while True:
    
    # Delete already executed flights from the list of available flights
    if counter >= 0:
        del flights_dict[selected[counter]]
    
    # Break the while loop when there are no more available flights remaining
    if len(flights_dict) == 0:
        print "CONGRATULATIONS! ALL FLIGHTS EXECUTED."
        print
        print "The following missions were aborted:"
        for k,v in mission_aborted.items():
            print k,v
        break
    
    # Print to the console flights that already have been executed, any missions that have been aborted,
    # and flights still available for execution.
    print
    print "These flights have already been executed:"
    for k,v in selected_dict.items():
        print k,v
    print
    print
    print "These missions have been aborted:"
    for k,v in mission_aborted.items():
        print k,v
    print
    print
    keys = []
    print "These are your available flights:"
    for k,v in flights_dict.items():
        keys.append(k)
        print k,v
    print
    
    # Define a function to ensure that the user input is valid
    def check_input(flight):
        if not flight.isdigit():
            flight = raw_input("Please enter flight number. Which flight would you like to run? ")
            check_input(flight)
        if ('flight_' + str(flight)) in selected:
            flight = raw_input("Flight already executed.  Please select a different flight. ")
            check_input(flight)
        if ('flight_' + str(flight)) not in keys:
            flight = raw_input("Flight number not available.  Please select a different flight. ")
            check_input(flight)
        return flight

    # Check that the user input is valid
    flight = raw_input("Which flight would you like to run? ")
    flight = check_input(flight)
    print "flight_" + str(flight) + " selected for execution"
    
    # Append the newly selected flight to a list of previously selected flights.
    # 'selected' used in 'check_input' function above
    selected.append('flight_' +str(flight))
    print
    
    # Create a list of missions in the selected flight    
    missions = flights_dict['flight_' + str(flight)]
    missions_list = []
    first_mission='m'+str(missions[0])
    second_mission='m'+str(missions[1])
    third_mission='m'+str(missions[2])
    
    missions_list.append(first_mission)
    if second_mission != 'mNone':
        missions_list.append(second_mission)
    if third_mission != 'mNone':
        missions_list.append(third_mission)
    
    # Add the selected flight and its missions to a dictionary object of executed {flights:[missions]}
    selected_dict['flight_' + str(flight)] = missions_list
    
    
#********************************************************************************      
# THIS SECTION INITIATES THE VEHICLE SIMULATOR, AND CREATES 3 VEHICLE OBJECTS (Ardupilot Copter 3.3)
    
    # Initiate SITL simulator. The "home" location of the simulator is set to the
    # location of one of the landing zones, such that if this script is connected to a ground
    # control station (i.e. QGroundControl), the simulated vehicles will be correctly positioned in space.
    # The simulator is also set to run at 5X 'real-time' speed.
    sitl = SITL()
    sitl.download('copter', '3.3', verbose=True)
    sitl_args = ['--instance=40', '--model', 'quad', '--home=' + str(landing_info["LZ1"]["latitude"])+
                 ',' + str(landing_info["LZ1"]["longitude"]) + ',0,180', '--speedup=5']
    sitl.launch(sitl_args, await_ready=True, restart=True)
    connection_string1 = tcp1
    connection_string2 = tcp2
    connection_string3 = tcp3
    
    # Create vehicle objects
    if len(missions_list) ==1:
        # Connect to the first vehicle
        print 'Connecting to vehicle1 on: %s' % connection_string1
        vehicle1 = connect(connection_string1, wait_ready=True)

        # Connect to the second vehicle
        print 'Connecting to vehicle2 on: %s' % connection_string2
        vehicle2 = 0
        
        # Connect to the third vehicle
        print 'Connecting to vehicle3 on: %s' % connection_string3
        vehicle3 = 0
    
    if len(missions_list) ==2:
        # Connect to the first vehicle
        print 'Connecting to vehicle1 on: %s' % connection_string1
        vehicle1 = connect(connection_string1, wait_ready=True)
        
        # Connect to the second vehicle
        print 'Connecting to vehicle2 on: %s' % connection_string2
        vehicle2 = connect(connection_string2, wait_ready=True)

        # Connect to the third vehicle
        print 'Connecting to vehicle3 on: %s' % connection_string3
        vehicle3 = 0
    
    
    if len(missions_list) ==3:
        # Connect to the first vehicle
        print 'Connecting to vehicle1 on: %s' % connection_string1
        vehicle1 = connect(connection_string1, wait_ready=True)
        
        # Connect to the second vehicle
        print 'Connecting to vehicle2 on: %s' % connection_string2
        vehicle2 = connect(connection_string2, wait_ready=True)
        
        # Connect to the third vehicle
        print 'Connecting to vehicle3 on: %s' % connection_string3
        vehicle3 = connect(connection_string3, wait_ready=True)
    print

    
#********************************************************************************      
# THIS SECTION DEFINES A SET OF FUNCTIONS (FOR EACH OF 3 VEHICLES) FOR MISSION MONITORING AND EXECUTION

    # SET GENERIC LOCATION AND DISTANCE FUNCTIONS
    def get_location_metres(original_location, dNorth, dEast):
        """
        (From http://python.dronekit.io/examples/mission_basic.html)
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
        specified `original_location`. The returned Location has the same `alt` value
        as `original_location`.
    
        """
        earth_radius=6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    
        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        return LocationGlobal(newlat, newlon,original_location.alt)
    
    
    def get_distance_metres(aLocation1, aLocation2):
        """
        (From http://python.dronekit.io/examples/mission_basic.html)
        Returns the ground distance in metres between two LocationGlobal objects.
    
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    

    # VEHICLE1
    #**1****1****1****1****1****1****1****1****1****1****1****1
    if vehicle1 != 0:
        def distance_to_current_waypoint1():
            """
            (From http://python.dronekit.io/guide/auto_mode.html)
            Gets distance in metres to the current waypoint. 
            It returns None for the first waypoint (Home location).
            """
            nextwaypoint = vehicle1.commands.next
            if nextwaypoint==0:
                return 0
            missionitem=vehicle1.commands[nextwaypoint-1] #commands are zero indexed
            lat = missionitem.x
            lon = missionitem.y
            alt = missionitem.z
            targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
            distancetopoint = get_distance_metres(vehicle1.location.global_frame, targetWaypointLocation)
            return distancetopoint
        
        
        def download_mission1():
            """
            Download the current mission from the vehicle.
            """
            cmds = vehicle1.commands
            cmds.download()
            cmds.wait_ready() # wait until download is complete.
        
        
        def arm_and_takeoff1(aTargetAltitude):
            """
            (From http://python.dronekit.io/examples/mission_basic.html)
            Arms vehicle and fly to aTargetAltitude.
            """
            print "Basic pre-arm checks"
            # Don't let the user try to arm until autopilot is ready
            while not vehicle1.is_armable:
                print " Waiting for vehicle to initialise..."
                time.sleep(1)
        
            print "Arming motors"
            # Copter should arm in GUIDED mode
            vehicle1.mode = VehicleMode("GUIDED")
            vehicle1.armed = True
        
            while not vehicle1.armed:      
                print " Waiting for arming..."
                time.sleep(1)
            print    
            print "Taking off!"
            vehicle1.simple_takeoff(aTargetAltitude) # Take off to target altitude
        
            # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
            #  after Vehicle.simple_takeoff will execute immediately).
            while True:
                print " Altitude: ", vehicle1.location.global_relative_frame.alt      
                if vehicle1.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
                    print
                    print "Reached target altitude"
                    break
                time.sleep(1)
        
    
    # VEHICLE2
    #**2****2****2****2****2****2****2****2****2****2****2****2
    if vehicle2 != 0:
        def distance_to_current_waypoint2():
            """
            Gets distance in metres to the current waypoint. 
            It returns None for the first waypoint (Home location).
            """
            nextwaypoint = vehicle2.commands.next
            if nextwaypoint==0:
                return 0
            missionitem=vehicle2.commands[nextwaypoint-1] #commands are zero indexed
            lat = missionitem.x
            lon = missionitem.y
            alt = missionitem.z
            targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
            distancetopoint = get_distance_metres(vehicle2.location.global_frame, targetWaypointLocation)
            return distancetopoint
        
        
        def download_mission2():
            """
            Download the current mission from the vehicle.
            """
            cmds = vehicle2.commands
            cmds.download()
            cmds.wait_ready() # wait until download is complete.
        
        
        def arm_and_takeoff2(aTargetAltitude):
            """
            Arms vehicle and fly to aTargetAltitude.
            """
            print "Basic pre-arm checks"
            # Don't let the user try to arm until autopilot is ready
            while not vehicle2.is_armable:
                print " Waiting for vehicle to initialise..."
                time.sleep(1)
                
            print "Arming motors"
            # Copter should arm in GUIDED mode
            vehicle2.mode = VehicleMode("GUIDED")
            vehicle2.armed = True
        
            while not vehicle2.armed:      
                print " Waiting for arming..."
                time.sleep(1)
            print    
            print "Taking off!"
            vehicle2.simple_takeoff(aTargetAltitude) # Take off to target altitude
        
            # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
            #  after Vehicle.simple_takeoff will execute immediately).
            while True:
                print " Altitude: ", vehicle2.location.global_relative_frame.alt      
                if vehicle2.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
                    print
                    print "Reached target altitude"
                    break
                time.sleep(1)
    
    
    # VEHICLE3
    #**3****3****3****3****3****3****3****3****3****3****3****3
    if vehicle3 != 0:
        def distance_to_current_waypoint3():
            """
            Gets distance in metres to the current waypoint. 
            It returns None for the first waypoint (Home location).
            """
            nextwaypoint = vehicle3.commands.next
            if nextwaypoint==0:
                return 0
            missionitem=vehicle3.commands[nextwaypoint-1] #commands are zero indexed
            lat = missionitem.x
            lon = missionitem.y
            alt = missionitem.z
            targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
            distancetopoint = get_distance_metres(vehicle3.location.global_frame, targetWaypointLocation)
            return distancetopoint
        
        
        def download_mission3():
            """
            Download the current mission from the vehicle.
            """
            cmds = vehicle3.commands
            cmds.download()
            cmds.wait_ready() # wait until download is complete.
        
        
        def arm_and_takeoff3(aTargetAltitude):
            """
            Arms vehicle and fly to aTargetAltitude.
            """
            print "Basic pre-arm checks"
            # Don't let the user try to arm until autopilot is ready
            while not vehicle3.is_armable:
                print " Waiting for vehicle to initialise..."
                time.sleep(1)
        
            print "Arming motors"
            # Copter should arm in GUIDED mode
            vehicle3.mode = VehicleMode("GUIDED")
            vehicle3.armed = True
        
            while not vehicle3.armed:      
                print " Waiting for arming..."
                time.sleep(1)
            print    
            print "Taking off!"
            vehicle3.simple_takeoff(aTargetAltitude) # Take off to target altitude
        
            # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
            #  after Vehicle.simple_takeoff will execute immediately).
            while True:
                print " Altitude: ", vehicle3.location.global_relative_frame.alt      
                if vehicle3.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
                    print
                    print "Reached target altitude"
                    break
                time.sleep(1)


#********************************************************************************
# THIS SECTION DEFINES MAVLINK AUTOPILOT COMMANDS FOR EACH MISSION AND EXECUTES AND
# MONITORS THE FLIGHT

    # Initiate a 'commands' object for each vehicle and clear prior commands
    print
    if vehicle1 != 0:
        print 'COMMENCING MISSION ' + str(first_mission)
        cmds1 = vehicle1.commands
        cmds1.clear()
    if vehicle2 != 0:
        print 'COMMENCING MISSION ' + str(second_mission)
        cmds2 = vehicle2.commands
        cmds2.clear()
    if vehicle3 != 0:
        print 'COMMENCING MISSION ' + str(third_mission)
        cmds3 = vehicle3.commands
        cmds3.clear()
    print
    
    # Define command for home location of the vehicles
    print "Defining new commands"
    print
    for i in range(len(missions_list)):
        lz = landing_info['LZ' +str(i+1)]
        cmd=Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                 mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, 0, 0, 
                 lz['latitude'], lz['longitude'], 0)
        if i == 0:
            cmds1.add(cmd)
        if i == 1:
            cmds2.add(cmd)
        if i == 2:
            cmds3.add(cmd)
    
    # Define commands for navigating from waypoint to waypoint.  The 'Start_Lat', 'Start_Lon', 
    # and 'Alt_m' shapefile attributes are referred to for all features (sorties) in a shapefile.
    # The 'End_Lat' and 'End_Lon' attributes are referred to complete the final navigation segment.
    for i in range(len(missions_list)):
        mission = all_info[missions_list[i]]
        for j in range(len(mission)):
            cmd=Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                 mission["feature{0}".format(j)]['Start_Lat'], mission["feature{0}".format(j)]['Start_Lon'], mission["feature{0}".format(j)]['Alt_m_'])
            if i == 0:    
                cmds1.add(cmd)
            if i == 1:
                cmds2.add(cmd)
            if i == 2:
                cmds3.add(cmd)
            if j==(len(mission)-1):
                cmd=Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                     mission["feature{0}".format(j)]['End_Lat'], mission["feature{0}".format(j)]['End_Lon'], mission["feature{0}".format(j)]['Alt_m_'])
                if i == 0:    
                    cmds1.add(cmd)
                if i == 1:
                    cmds2.add(cmd)
                if i == 2:
                    cmds3.add(cmd)
    
    # Define commands for mission landing.  Add it twice to the commands list (the second addition acts as a dummy-
    # once activated I know that the copter has landed and the mission is over).  Upload all commands.
    for i in range(len(missions_list)):
        lz = landing_info['LZ' +str(i+1)]
        cmd=Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                 mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 
                 lz['latitude'], lz['longitude'], 0)
        if i == 0:
            cmds1.add(cmd)
            cmds1.add(cmd)
            cmds1.upload()
        if i == 1:
            cmds2.add(cmd)
            cmds2.add(cmd)
            cmds2.upload()
        if i == 2:
            cmds3.add(cmd)
            cmds3.add(cmd)
            cmds3.upload() 
    print "Uploading new commands to vehicles"
    print
    
    
    # BEGIN EXECUTING MISSIONS!!!!
    if vehicle1 != 0:
        arm_and_takeoff1(10)
        vehicle1.commands.next=1
    if vehicle2 != 0:  
        arm_and_takeoff2(10)
        vehicle2.commands.next=1
    if vehicle3 != 0: 
        arm_and_takeoff3(10)
        vehicle3.commands.next=1        
    print "Commencing navigation to waypoints"
    print
        
    # Set mode to AUTO to begin automated commands. Set the vehicle airspeed attributes.
    # (For the example set, vehicle airspeed was constant for all features (sorties) for all shapefiles.
    # Therefore, I opted to set the airspeed as a constant vehicle attribute.  If sorties were to require 
    # variable airspeeds, this script would need to be amended to incorporate 'MAV_CMD_DO_CHANGE_SPEED' commands)
    if vehicle1 != 0:
        vehicle1.mode = VehicleMode("AUTO")
        vehicle1.airspeed = all_info['m1']['feature1']['Speed_m_s_']
    if vehicle2 != 0:  
        vehicle2.mode = VehicleMode("AUTO")
        vehicle2.airspeed = all_info['m1']['feature1']['Speed_m_s_']
    if vehicle3 != 0:
        vehicle3.mode = VehicleMode("AUTO")
        vehicle3.airspeed = all_info['m1']['feature1']['Speed_m_s_']

    # Monitor progress of mission. Vehicle distances to their next waypoints are streamed continuously 
    # at 1-second intervals for all missions in the current flight. When all missions reach their landing command,
    # the monitoring stream stops.  A fail-safe mechanism is built into the monitoring protocol, whereby if 
    # a given mission does not progress more than 10cm towards its next waypoint in 2 seconds, the autopilot tool
    # is assumed to have failed (for one reason or another) and the mission is aborted.  (This was a necessary
    # implementation to handle m30 in the example set). Aborted missions are tracked and listed during flight selection.
    a=0
    b=0
    c=0
    list_distance1 = []
    list_distance2 = []
    list_distance3 = []
    while True:
        if len(missions_list) == 1:
            if vehicle1.commands.next > len(vehicle1.commands):
                nextwaypoint1 = None
            else:
                nextwaypoint1 = vehicle1.commands.next # Gets the currently active waypoint (command) number
                print missions_list[0] + ': Distance to waypoint (%s): %s' % (nextwaypoint1, distance_to_current_waypoint1())
            if len(list_distance1) > 5 and max(list_distance1[-3:])-min(list_distance1[-3:]) < .1 and nextwaypoint1 < len(all_info[missions_list[0]])-1:
                a = 1
                mission_aborted[missions_list[0]] = 'aborted at sortie ' + str(nextwaypoint1)
            if nextwaypoint1 >= len(all_info[missions_list[0]])-1:
                a = 1
            if a == 1:
                print "Exit 'standard' mission"
                break;
            time.sleep(1)
            print
        if len(missions_list) == 2:
            if vehicle1.commands.next > len(vehicle1.commands):
                nextwaypoint1 = None
            else:
                nextwaypoint1 = vehicle1.commands.next # Gets the currently active waypoint (command) number
                print missions_list[0] + ': Distance to waypoint (%s): %s' % (nextwaypoint1, distance_to_current_waypoint1())
            
            if vehicle2.commands.next > len(vehicle2.commands):
                nextwaypoint2 = None
            else:    
                nextwaypoint2 = vehicle2.commands.next # Gets the currently active waypoint (command) number
                print missions_list[1] + ': Distance to waypoint (%s): %s' % (nextwaypoint2, distance_to_current_waypoint2())
                
            if nextwaypoint1 >= len(all_info[missions_list[0]])-1:
                a = 1
            if nextwaypoint2 >= len(all_info[missions_list[1]])-1:
                b = 1            
            if len(list_distance1) > 5 and max(list_distance1[-3:])-min(list_distance1[-3:]) < .1 and nextwaypoint1 < len(all_info[missions_list[0]])-1:
                a = 1
                mission_aborted[missions_list[0]] = 'aborted at sortie ' + str(nextwaypoint1)
            if len(list_distance2) > 5 and max(list_distance2[-3:])-min(list_distance2[-3:]) < .1 and nextwaypoint2 < len(all_info[missions_list[1]])-1:
                b = 1             
            if a == 1 and b == 1:
                print "Exit 'standard' mission"
                break;            
            time.sleep(1) 
            print
        if len(missions_list) == 3:
            if vehicle1.commands.next > len(vehicle1.commands):
                nextwaypoint1 = None
            else:
                nextwaypoint1 = vehicle1.commands.next # Gets the currently active waypoint (command) number
                print missions_list[0] + ': Distance to waypoint (%s): %s' % (nextwaypoint1, distance_to_current_waypoint1())
                list_distance1.append(distance_to_current_waypoint1())
                
            if vehicle2.commands.next > len(vehicle2.commands):
                nextwaypoint2 = None
            else:    
                nextwaypoint2 = vehicle2.commands.next # Gets the currently active waypoint (command) number
                print missions_list[1] + ': Distance to waypoint (%s): %s' % (nextwaypoint2, distance_to_current_waypoint2())
                list_distance2.append(distance_to_current_waypoint2())
            
            if vehicle3.commands.next > len(vehicle3.commands):
                nextwaypoint3 = None
            else:    
                nextwaypoint3 = vehicle3.commands.next # Gets the currently active waypoint (command) number
                print missions_list[2] + ': Distance to waypoint (%s): %s' % (nextwaypoint3, distance_to_current_waypoint3())
                list_distance3.append(distance_to_current_waypoint3())
            
            if nextwaypoint1 >= len(all_info[missions_list[0]])-1:
                a = 1
            if nextwaypoint2 >= len(all_info[missions_list[1]])-1:
                b = 1
            if nextwaypoint3 >= len(all_info[missions_list[2]])-1:
                c = 1
            if len(list_distance1) > 5 and max(list_distance1[-3:])-min(list_distance1[-3:]) < .1 and nextwaypoint1 < len(all_info[missions_list[0]])-1:
                a = 1
                mission_aborted[missions_list[0]] = 'aborted at sortie ' + str(nextwaypoint1)
            if len(list_distance2) > 5 and max(list_distance2[-3:])-min(list_distance2[-3:]) < .1 and nextwaypoint2 < len(all_info[missions_list[1]])-1:
                b = 1 
                mission_aborted[missions_list[1]] = 'aborted at sortie ' + str(nextwaypoint2)
            if len(list_distance3) > 5 and max(list_distance3[-3:])-min(list_distance3[-3:]) < .1 and nextwaypoint3 < len(all_info[missions_list[2]])-1:
                c = 1
                mission_aborted[missions_list[2]] = 'aborted at sortie ' + str(nextwaypoint3)

            if a == 1 and b == 1 and c == 1:
                print "Exit 'standard' mission"
                break;
            time.sleep(1)
            print
            
    # Initiate "Return to Launch" mode.
    print 'Returning to launch'
    if vehicle1 != 0:
        vehicle1.mode = VehicleMode("RTL")
    if vehicle2 != 0:
        vehicle2.mode = VehicleMode("RTL")
    if vehicle3 != 0:
        vehicle3.mode = VehicleMode("RTL")
    print
    
    
    # Close vehicle objects before exiting script.
    print "Closing vehicle objects"
    if vehicle1 != 0:
        vehicle1.close()
    if vehicle2 != 0:
        vehicle2.close()
    if vehicle3 != 0:
        vehicle3.close()    
    print
    print "FLIGHT COMPLETE!"
    print
    
    # Shut down simulator if it was started.  Occasionally, the sitl.stop() command fails.  
    # This case is handled by the 'try'/'except' statement.
    if sitl is not None:
        try:
            sitl.stop()
        except:
            print "Simulator error.  Please restart script"
            break;

    # Add 1 to counter
    counter = counter + 1

"""
Additional notes:

1) If the SITL simulator parameters are changed so that simulation proceeds at real-time, 
and if time.sleep() functions are positioned strategically throughout the script (to account for lag-time 
between the script execution and GCS execution), the script may be simulated using a ground control station.
However, at least for QGroundControl, the GCS is not built to listen to multiple vehicles at the time.  The GCS will
show a single vehicle rapidly hopping between coordinates, where there should actually be multiple vehicles gliding 
smoothly along their flight paths.

2) The Ardupilot Copter model 3.3 (built into the SITL simulator) is not endowed with a parameter for spraying.
Thus, commands for spraying were left out of the mission.  During execution of the actual flights with Copter 
model 3.4 and higher, spraying could be incorporated via the 'MAV_CMD_DO_SET_PARAMETER' command with the
appropriate sprayer parameter enumerations and values.
http://ardupilot.org/copter/docs/sprayer.html
https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_PARAMETER

"""