from dronekit import LocationGlobalRelative,connect,VehicleMode
import math
import numpy as np
import matplotlib.pyplot as plt

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

d = np.load("/home/vasudevan/Desktop/editcode/distance.npy")
a = np.load("/home/vasudevan/Desktop/editcode/angle.npy")
lat = np.load("/home/vasudevan/Desktop/editcode/lat.npy")
lon = np.load("/home/vasudevan/Desktop/editcode/lon.npy")

j = len(lat)
print(lat)
print(lon)
k = 0
dis = []
thet = []
angu = []
yoyo = []

for i in range(j):
   dd = distance_calculation(lat[k],lon[k],28.7539183,77.1155394)
   bearing = theta_calculator(lat[k],lon[k],28.7539183,77.1155394)
   if bearing > 0:
      thet.append(bearing)
   else:
      thet.append(360+bearing)
   dis.append(dd)
   angu.append(math.degrees(a[k]))
  #  thet.append(bearing)
  #  angu.append(math.degrees(a[k])
   yoyo.append(d[k])
   k+=1

plt.plot(angu)
plt.plot(thet)
plt.show()
