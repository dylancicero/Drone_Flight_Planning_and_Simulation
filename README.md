# Drone Flight Planning and Simulation

This project showcases two tools: (1) The first , written in PyQGIS (QGIS 3.2.1) and utilizing the Shapely library, 
groups a given set of drone missions into flights (maximum of 3 concurrent missions per flight) according to an algorithm
that (a) prevents any chance of collision between concurrent missions in a flight, (b) minimizes the total number of
flights, and (c) exludes missions that fly too close to a landing zone.  (2) The second tool, written in Python2, and
utilizing both the Ogr library and the DroneKit-Python API (a) takes as input the flight plans generated from the first tool,
(b) asks a ground-operator to select a flight plan, and (c) simulates its execution.  Please follow the instructions in the
"ReadMe" section (at the top of both scripts) before attempting to run either tool on your own machine.


## Tool 1: Flight Planning 
An operator is in the field with three drone copters available and many missions to fly. Some of these missions have
overlapping paths (on the X,Y plane).  The task is to maximize efficiency by running the three drones concurrently 
(referred to as a "flight"), but ensuring that missions in a given flight do not overlap!  Each mission is contained 
in a shapefile, where each feature in a given shapefile represents a straight-line component (point x,y,z to point x,y,z) 
of the mission.  See diagram below, based off of the example set:







## Tool 2: Flight Simulation