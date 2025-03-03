import cv2 
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import geotag
import time
import math

vehicle = connect("127.0.0.1:14555", wait_ready=False) #initalize vehicle object

pan = 0         #initial gimbal pan
tilt = -90      #initial gimbal tilt
rot_time = 0    #gimbal rotation timer

#Hard coded default loop path if lost tracking, lats-lons to be stored in an array imported from JSON
lats = [28.75455867,28.75395321,28.75390808,28.75369373,28.75348690,28.75309580]
lons = [77.11538911,77.11480573,77.11475855,77.11454836,77.11432530,77.11395211]


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

def get_aruco(frame, id):

    dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
    param = cv2.aruco.DetectorParameters_create()
    corner, ids, rejected = cv2.aruco.detectMarkers(frame, dict, parameters = param)


    if len(corner) > 0:
        counter = -1
        for i in ids:
            counter += 1
    
            corners = np.ravel(corner[counter])
            #Find center of the aruco
            center_x = ( float(corners[0]) + float(corners[2]) + float(corners[4]) + float(corners[6]) ) / 4
            center_y = ( float(corners[1]) + float(corners[3]) + float(corners[5]) + float(corners[7]) ) / 4
            list_x = [float(corners[0]),float(corners[2]),float(corners[4]),float(corners[6])]
            list_y = [float(corners[1]),float(corners[3]),float(corners[5]),float(corners[7])]
            list_x.sort()
            list_y.sort()
            bbox = (int(list_x[0]), int(list_y[0]), int(list_x[3] - list_x[0]), int(list_y[3]-list_y[0]))
        try:        
            return True, corners, bbox, center_x, center_y

        except:
            return False, None, None, None, None

    else:
        return False, None, None, None, None
    
def drawBox(frame, bbox):
    x,y,w,h = int(bbox[0]),int(bbox[1]),int(bbox[2]),int(bbox[3])
    cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
    return frame


def goto(location):
        """ Go to a location
        
        Input:
            location    - LocationGlobal or LocationGlobalRelative object
        
        """
        vehicle.airspeed = 8
        vehicle.simple_goto(location)


def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle

    vehicle.send_mavlink(msg)
    time.sleep(0.1)

def theta_calculator(lat,lon,vehicle):
  
  d = lon-vehicle.location.global_relative_frame.lon
  x = math.cos(math.radians(lat))*math.sin(math.radians(d))
  y = math.cos(math.radians(vehicle.location.global_relative_frame.lat))*math.sin(math.radians(lat))-math.sin(math.radians(vehicle.location.global_relative_frame.lat))*math.cos(math.radians(lat))*math.cos(math.radians(d))
  theta = math.degrees(math.atan2(x,y))
  return theta

def calc_open_loop(lt,ln): 

    min_dis = 0
    for i in range(len(lats)):
        d = distance_calculation(lt,ln,lats[i],lons[i])
        if d < min_dis or min_dis == 0:
            min_dis = d
            index = i
        
    return index
    

# open video stream and resize
video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

alt = 15 #alititude
ok, frame = video.read()
# tracker = cv2.TrackerKCF_create()
# bbox = (287,23,86,320)
# ok = tracker.init(frame,bbox)
tracker_locked=False

arm_and_takeoff(alt)

initial_point = LocationGlobalRelative(28.7536786,77.1156315,15) #initial goto point

vehicle.simple_goto(initial_point)
while distance_calculation(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, initial_point.lat, initial_point.lon) >= 0.2:
    pass

initiate_search = False

while True:

    ok, frame = video.read()
    detected, corners_det, bbox, center_x, center_y = get_aruco(frame, 700)
    curr_time = time.time()
    
    if tracker_locked:
            
            success, tracked_bbox =  tracker.update(frame)
            if success:

                # Main follow code

                initiate_search = False
                wait_detect = True
                counter = 1
                frame = drawBox(frame, tracked_bbox)
                tracking = True
                x_track, y_track, w, h = tracked_bbox
                target_x = tracked_bbox[0] + w/2
                target_y = tracked_bbox[1] + h/2
                p,t = geotag.theta_calc_xy(target_x,target_y) #get target gimbal angle

                if curr_time-rot_time >= 1:
                   
                   tilt = t + tilt
                   pan = p + pan
                   vehicle.gimbal.rotate(int(tilt),0,0)
                
                   rot_time = time.time()

                # Find aruco position in global frame
                lat, lon, d = geotag.geo_tag_karo(target_y, vehicle.location.global_relative_frame.alt-1.9, tilt + 90, p, vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.heading)
                target = LocationGlobalRelative(lat,lon,alt)

                # Follow
                if d >= 5:
                    goto(target)

            else:

                print('Tracking Lost')
                tracking = False
                tracker_locked = False
                initiate_search = True

                # Find index of closest waypoint in hard coded path
                i_open_loop = calc_open_loop(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon)
                gimbal_rot = False
                k = 0

    if detected:
        print('Locked!')
        tracker = cv2.TrackerKCF_create()
        
        ret = tracker.init(frame, bbox)
        if ret:
            tracker_locked = True

        target_x = center_x
        target_y = center_y

    elif initiate_search == True:

        print("###SEARCH MODE###")
        
        # Rotate gimbal to last known aruco location to retrack it
        if gimbal_rot == False:

            p_angle,t_angle = geotag.theta_calc_xy(target_x,target_y)
            tilt = t_angle + tilt
            vehicle.gimbal.rotate(int(tilt),0,0)
            gimbal_rot = True
            tim = time.time()

        if curr_time - tim >= 3:
    
            goloc = LocationGlobalRelative(lats[i_open_loop+counter],lons[i_open_loop+counter], alt)
            vehicle.simple_goto(goloc)
            print("SENT GOTO")

            if distance_calculation(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, lats[i_open_loop+counter], lons[i_open_loop+counter]) <= 1:
                counter += 1 
                if (i_open_loop + counter) > (len(lats)-1):
                    i_open_loop = 0
                    counter = 0

            
    else:
        print("No Detection")



