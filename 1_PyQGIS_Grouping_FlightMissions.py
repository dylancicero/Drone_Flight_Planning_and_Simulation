'''
READ ME: IN ORDER FOR THIS TOOL TO WORK ON YOUR OWN COMPUTER...

1) SHAPEFILES FOR ALL MISSIONS MUST BE LOCATED IN A COMMON FOLDER WHERE EACH MISSION IS NAMED BY 
THE CONVENTION "m{#}.shp". {#} STARTS AT 1 AND GOES UP TO THE TOTAL NUMBER OF MISSIONS.

2) SET THE 'mission_path' VARIABLE BELOW TO THE PATH OF THE FOLDER ON YOUR OWN COMPUTER
THAT CONTAINS THE SHAPEFILE MISSIONS.

3) SET THE 'lz_path' VARIABLE BELOW TO THE PATH OF THE LANDING ZONE SHAPEFILE ON YOUR OWN COMPUTER

4) SET THE 'out_path' VARIABLE BELOW MUST TO THE PATH OF THE FOLDER ON YOUR OWN COMPUTER
IN WHICH YOU WISH TO OUTPUT THE .CSV OF FLIGHTS AND THE .TXT OF NO-FLY MISSIONS

5) SET THE 'total_missions' VARIABLE BELOW TO THE TOTAL NUMBER OF SHAPEFILE MISSIONS TO EVALUATE
(IF EXECUTING THE EXAMPLE SET, THE DEFAULT OF 31 WILL REMAIN UNCHANGED)
'''

#********************************************************************************
# TO DO!
# Set path to folder containing shapefile missions:
mission_path = "U:/Work Sample Test/Missions_Renamed/"
# Set path to shapefile of landing zones:
lz_path = "U:/Work Sample Test/Landing Zone/LZ_points_WGS.shp"
# Set path to folder in which the script artifacts will be outputted
out_path = 'U:/Work Sample Test/'
# Set total number of missions:
total_missions = 31
#********************************************************************************



from qgis.core import *
from qgis.core import QgsCoordinateReferenceSystem, QgsCoordinateTransform, QgsProject, QgsGeometry
import qgis.utils
from qgis.utils import iface
from shapely.geometry import Point
from shapely.ops import linemerge
from sys import *
import numpy as np
import csv



# Import all missions from specified folder as qgis vector layer
missions = {}
for i in range(total_missions):
    missions["vlayer{0}".format(i)] = mission_path + "m" + str(i+1) + ".shp"
list_vectors = []
for k,v in missions.items():
    k = qgis.utils.iface.addVectorLayer(v, k, "ogr")
    list_vectors.append(k)

# Import the landing zone shapefile
landing_zone = iface.addVectorLayer(lz_path,"landing_zone", "ogr")



# Get the qgis coordinate transform object so that the imported WGS84 geographic coordinates 
# can later be reprojected into a planar coordinate system.  This is essential for two purposes:
# (1) First, this allows the proper specification of a "No Fly Zone" measured in meters 
# from a given landing zone (as opposed to decimal degrees).
# (2) Second, this allows the proper specification of a buffer region for each flight path,
# measured in meaters.  (This offers a more realistic assessment for the 6-foot diameter drones 
# of missions prone to collision).
crsSrc = QgsCoordinateReferenceSystem(4326) 
crsDest = QgsCoordinateReferenceSystem(42303)
xform = QgsCoordinateTransform(crsSrc, crsDest, QgsProject.instance())



# For all shapefiles, create a single mission flightpath from vector features
# Buffer these flightpaths by 2 meters and store in a list of all flightpaths
flightpaths = []
for item in list_vectors:
    iter = item.getFeatures()
    lines_list = []
    # retrieve every feature with its geometry and attributes
    for feature in iter:
        # fetch geometry
        geom = feature.geometry()
        coords = geom.asPolyline()
        transformed = []
        for point in coords:
                point = xform.transform(point)
                transformed.append(point)
        lines_list.append(transformed)
    flightpaths.append(linemerge(lines_list).buffer(2))



# Create a "No Fly Zone" around a 20m radius for each of the landing points
iter = landing_zone.getFeatures()
attrs = []
NoFlyZones = []
for feature in iter:
    attrs.append(feature.attributes())
    for i in range(len(attrs)):
        point = QgsPointXY(attrs[i][3], attrs[i][2])
        point = xform.transform(point)
        point = QgsGeometry.fromPointXY(point)
        coords = point.asPoint()
        point = Point(coords)
        polygon = point.buffer(20, 10)
    NoFlyZones.append(polygon)



# Evaluate the set of missions that enter the "No Fly Zone".
# These will later be excluded from grouping into executable flights.
tooclose = set()
for i in range(len(flightpaths)):
    for j in range(len(NoFlyZones)):
        bool = flightpaths[i].intersects(NoFlyZones[j])
        if bool == True:
            tooclose.add(str(i+1))
tooclose = list(tooclose)



# Output a .txt file with missions that enter the "No Fly Zone"
out_path_nofly = out_path + 'no_fly_missions.txt'
with open(out_path_nofly, "w") as file:
    file.write('The following missions get too close to the landing zones: \n')
    for item in tooclose:
        file.write('m' + item + "\n")



# Evaluate the set of missions that do not enter the "No Fly Zone"
flyable = []
for i in range(1,32):
    flyable.append(str(i))
for mission in tooclose:
    if mission in flyable:
        flyable.remove(mission)



# Create a dictionary, where {key:value} pair exists for each mission, key is 
# the mission name, and value is a list of all missions where there is chance of collision.
collision_dict = {}
lengths = {}
for i in range(len(flightpaths)):
    intersects_temp = []
    for j in range(len(flightpaths)):
        bool = flightpaths[i].intersects(flightpaths[j])
        if bool == True:
            intersects_temp.append(str(j+1))
    collision_dict[str(i+1)] = intersects_temp
    
# Sort this dictionary by the length of its collision list, where missions most prone to collision
# appear earlier in the dictionary.
sorted1 = sorted(collision_dict.items(), key=lambda kv: len(kv[1]), reverse = True)
    


# Build a dictionary with {mission:[positive_set]} for each mission, sorted in order of most
# restrictive missions
# Only include missions in the dictionary that do not fly over the "No Fly Zone"
noncollision_dict = {}
for i in range(total_missions):
    full_set_strings =['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14', '15', '16', '17', '18', '19', '20', '21', '22', '23', '24', '25', '26', '27', '28', '29', '30', '31']
    for item in sorted1[i][1]:
        if item in full_set_strings:
            full_set_strings.remove(item)
        positive_set = full_set_strings
    if str(i+1) not in tooclose: 
        noncollision_dict[sorted1[i][0]]= positive_set

# Sort this dictionary by the length of its collision list, where missions most prone to collision
# appear earlier in the dictionary
list_tuples_sorted = sorted(noncollision_dict.items(), key=lambda kv: len(kv[1]))



# Develop algorithm to pair missions together in flights:
# (1) Start with the most restrictive mission.
# (2) Look at this list of missions that it can fly with 
# (3) Select the two missions it can fly with that themselves are the most restrictive.
# (4) Remove those missions from flying with all other potential missions
# (5) Move on to the second most restrictive mission that has not yet been flown...
flights_dict = {}
#while len(list_tuples_sorted) > 1:
i = 1
while len(list_tuples_sorted) > 0:
    # Insert mission into temporary list
    temp_list = [list_tuples_sorted[0][0]]
    # Add two more missions to flight based on missions that are most restrictive.
    # The 'if' statement ensures that there are still available non_collision missions to fly with
    for j in range(1,len(list_tuples_sorted)):
        if len(temp_list) == 3:
            break
        if len(list_tuples_sorted[0][1]) == 0:
            break
        for k in range(len(list_tuples_sorted[0][1])):
            a = list_tuples_sorted[j][0]
            b = list_tuples_sorted[0][1][k]
            if list_tuples_sorted[j][0] == list_tuples_sorted[0][1][k]:
                # Insert mission into temporary list
                temp_list.append(list_tuples_sorted[j][0])
                break
    # Remove all missions in current flight from dictionary of available missions
    counter = 0
    for l in range(len(list_tuples_sorted)):
        for mission in temp_list:
            if counter == len(temp_list):
                break
            if mission == list_tuples_sorted[l][0]:
                list_tuples_sorted.remove(list_tuples_sorted[l])
                counter = counter+1
    for l in range(len(list_tuples_sorted)):
        for mission in temp_list:
            if mission in list_tuples_sorted[l][1]:
                list_tuples_sorted[l][1].remove(mission)
    # Add flight to flights_dict
    flights_dict['flight_' + str(i)] = temp_list
    i = i + 1



# Output results to csv.
csv_list = []
# If a flight doesn't have 3 missions, insert a 'None' placeholder
for k,v in flights_dict.items():
    csv_dict = {}
    csv_dict['flights']=k
    try:
        csv_dict['drone_1']=v[0]
    except:
        csv_dict['drone_1']= 'None'
    try:
        csv_dict['drone_2']=v[1]
    except:
        csv_dict['drone_2']= 'None'
    try:
        csv_dict['drone_3']=v[2]
    except:
        csv_dict['drone_3']= 'None'
    csv_list.append(csv_dict)

out_path_flights = out_path + 'flights.csv'

with open(out_path_flights, "w", newline = '') as file:
    fieldnames = ['flights', 'drone_1', 'drone_2', 'drone_3']
    writer = csv.DictWriter(file, fieldnames=fieldnames)
    writer.writeheader()
    for i in range(len(csv_list)):
        writer.writerow(csv_list[i])

