# Drone Flight Planning and Simulation

This project showcases two tools: (1) The first, written in PyQGIS (QGIS 3.2.1) and utilizing the Shapely library, 
groups a given set of drone missions into flights (maximum of 3 concurrent missions per flight) according to an algorithm
that (a) prevents any chance of collision between concurrent missions in a flight, (b) minimizes the total number of
flights, and (c) exludes missions that fly too close to a takeoff/landing zone.  (2) The second tool, written in Python2, and utilizing both the Ogr library and the DroneKit-Python API (http://python.dronekit.io/) (a) takes as input the flight plans generated from the first tool, (b) asks a ground-operator to select a flight plan, and (c) simulates its execution.  Please follow the instructions in the "ReadMe" section (at the top of both scripts) before attempting to run either tool on your own machine.


## Tool 1: Flight Planning
Scenario: an operator is in the field with three drone copters available and many missions to fly. Some of these missions have overlapping paths (on the X,Y plane).  The task is to maximize efficiency by running the three drones concurrently 
(referred to as a "flight"), but ensuring that missions in a given flight do not overlap!  Each mission is contained 
in a shapefile, where each feature in a given shapefile represents a straight-line component (point x,y,z to point x,y,z) 
of the mission.  See diagram below, based off of the example set:

<img width="1012" alt="screen shot 2019-02-21 at 2 46 35 pm" src="https://user-images.githubusercontent.com/43111524/53197108-7f3e0800-35e7-11e9-98eb-4f0cdf1b66e9.png">

Missions that fly within 20m of a takeoff/landing zone are excluded from flights.  See diagram below:

<img width="844" alt="screen shot 2019-02-21 at 2 56 51 pm" src="https://user-images.githubusercontent.com/43111524/53197791-ef995900-35e8-11e9-9af0-794fe1f2ee58.png">


The tool will output two files.  The first, "flights.csv", describes the optimal grouping of drone-missions into 3-mission flights.  The second, "no_fly_missions.txt", lists the missions that aren't grouped for flights due to flight proximity to a takeoff/landing zone.  File outputs for the example set are displayed below:

<img width="1044" alt="screen shot 2019-02-21 at 3 02 29 pm" src="https://user-images.githubusercontent.com/43111524/53198171-b9a8a480-35e9-11e9-83ad-4b8945729267.png">




## Tool 2: Flight Simulation
Now that we have our flights planned out, time to simulate the actual flights.  This tool first asks a ground operator to select a flight plan from the options in "flights.csv".  Then, it uploads mission information into 3 different vehicle objects in the Dronekit SITL simulator, and finally simulates flight execution.  Replacement of the simulator by actual drone copters would only require a simple substitution of TCP Port information.  Watch simulator run in link below!

https://www.youtube.com/watch?v=u70SuqdbRZc
