from dronekit import LocationGlobalRelative,connect,VehicleMode
import math
import numpy as np
import matplotlib.pyplot as plt
import geotag
import time

vehicle = connect("127.0.0.1:14552",wait_ready=False)

def distance_calculation(homeLattitude, homeLongitude, destinationLattitude, destinationLongitude):


    """

    This function returns the distance between two geographiclocations using
    the haversine formula.

    Inputs:
        1.  homeLattitude          -   Home or Current Location's  Latitude
        2.  homeLongitude          -   Home or Current Location's  Longitude
        3.  destinationLattitude   -   Destination Location's  Latitude
        4.  destinationLongitude   -   Destination Location's  Longitude

    """

    # Radius of earth in metres
    R = 6371e3

    rlat1, rlon1 = homeLattitude * (math.pi/180), homeLongitude * (math.pi/180)
    rlat2, rlon2 = destinationLattitude * (math.pi/180), destinationLongitude * (math.pi/180)
    dlat = (destinationLattitude - homeLattitude) * (math.pi/180)
    dlon = (destinationLongitude - homeLongitude) * (math.pi/180)

    # Haversine formula to find distance
    a = (math.sin(dlat/2) * math.sin(dlat/2)) + (math.cos(rlat1) * math.cos(rlat2) * (math.sin(dlon/2) * math.sin(dlon/2)))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    # Distance (in meters)
    distance = R * c

    return distance

def theta_calculator(lat1,lon1,lat2,lon2):
  d = lon2-lon1
  x = math.cos(math.radians(lat2))*math.sin(math.radians(d))
  y = math.cos(math.radians(lat1))*math.sin(math.radians(lat2))-math.sin(math.radians(lat1))*math.cos(math.radians(lat2))*math.cos(math.radians(d))
  theta = math.degrees(math.atan2(x,y))
  return theta

def goto(location):
        """ Go to a location
        
        Input:
            location    - LocationGlobal or LocationGlobalRelative object
        
        """
        vehicle.simple_goto(location)
        time.sleep(0.1)

def arm_and_takeoff(targetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(targetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=targetAltitude*0.95:
            print ("Reached target altitude")
            break
        # time.sleep(1)

d = np.load("/home/vasudevan/Desktop/editcode/arr_0.npy")
a = np.load("/home/vasudevan/Desktop/editcode/arr_1.npy")
lat = np.load("/home/vasudevan/Desktop/editcode/arr_0.npy")
lon = np.load("/home/vasudevan/Desktop/editcode/arr_1.npy")

j = len(lat)
print(lat)
print(lon)
k = 0
dis = []
thet = []
angu = []
yoyo = []
latnew = []
lonnew  = []

arm_and_takeoff(15)


# for i in range(j):
#    dd = distance_calculation(lat[k],lon[k],28.7539183,77.1155394)
#    bearing = theta_calculator(lat[k],lon[k],28.7539183,77.1155394)
#    if bearing > 0:
    #   thet.append(bearing)
#    else:
    #   thet.append(360+bearing)
#    dis.append(dd)
#    angu.append(math.degrees(a[k]))
  #  thet.append(bearing)
  #  angu.append(math.degrees(a[k])
#    lato,lono = geotag.geotag(dd,a[k],lat[k],lon[k])
#    latnew.append(lato)
#    lonnew.append(lono)
#    yoyo.append(d[k])
#    k+=1

oioio = len(latnew)
s = 0

for fol in range(j):
   ll = LocationGlobalRelative(lat[s],lon[s],15)
   goto(ll)
   time.sleep(3)
   s = s+1

# plt.plot(angu)
# plt.plot(thet)
# plt.show()
