import cv2 
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import geotag
import socket
import struct
import time
import math

vehicle = connect("127.0.0.1:14550", wait_ready=False)

# Define the IP and port of the ground station
# ground_station_ip = '192.168.191.112'
# ground_station_port = 5780

pp = 0
tt = -90
pan = 0
pitch = -90
rot_time = 0
timerboi = 0
dissy = []
tets = []
vecloclat = []
vecloclon = []
distanceboibuddha = 0

# Initialize the socket connection
# client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# client_socket.connect((ground_station_ip, ground_station_port))

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

def send_video(frames):
    # Encode the frame to JPEG format
    _, encoded_frame = cv2.imencode('.jpg', frames)
    # Get the size of the frame and pack it into a struct
    frame_size = struct.pack('<L', len(encoded_frame))
    # Send the frame size and then the frame data
    client_socket.sendall(frame_size)
    client_socket.sendall(encoded_frame.tobytes())

def goto(location):
        """ Go to a location
        
        Input:
            location    - LocationGlobal or LocationGlobalRelative object
        
        """
        vehicle.simple_goto(location)

def goto2(location):
        """ Go to a location
        
        Input:
            location    - LocationGlobal or LocationGlobalRelative object
        
        """
        vehicle.simple_goto(location)
        while True:
          print("hello",distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,location.lat,location.lon))
          if distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,location.lat,location.lon) <= 0.95*2:
            break

def theta_calculator(lat,lon,vehicle):
  d = lon-vehicle.location.global_relative_frame.lon
  x = math.cos(math.radians(lat))*math.sin(math.radians(d))
  y = math.cos(math.radians(vehicle.location.global_relative_frame.lat))*math.sin(math.radians(lat))-math.sin(math.radians(vehicle.location.global_relative_frame.lat))*math.cos(math.radians(lat))*math.cos(math.radians(d))
  theta = math.degrees(math.atan2(x,y))
  return theta

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


video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
ok, frame = video.read()
# tracker = cv2.TrackerKCF_create()
# bbox = (287,23,86,320)
# ok = tracker.init(frame,bbox)
tracker_locked=False

arm_and_takeoff(13)

startkaro = LocationGlobalRelative(28.75342012,77.11552749,15)
xvel = 0
yvel = 0
plat = vehicle.location.global_relative_frame.lat
plon = vehicle.location.global_relative_frame.lon

#goto2(startkaro)

while True:
    ok, frame = video.read()
    detected, corners_det, bbox, center_x, center_y = get_aruco(frame, 700)
    curr_time = time.time()
    print("bbox",bbox)
    # if not bbox==None:
        # frame = drawBox(frame, bbox)
    

    if tracker_locked:

            success, tracked_bbox =  tracker.update(frame)
            if success:
                #print('Tracked')
                frame=drawBox(frame, tracked_bbox)
                tracking = True
                x_track,y_track,w,h = tracked_bbox
                target_x = tracked_bbox[0] + w/2
                target_y = tracked_bbox[1]+h/2
                p,t = geotag.theta_calc_xy(target_x,target_y)
                print(p,t)
                # if (target_x <= 50 or target_x >= 590) or (target_y <= 50 or target_y >= 430):
                if curr_time-rot_time >= 1:
                   vehicle.gimbal.rotate(int(t+tt),0,0)
                   pitch = t + tt
                   pitch = 90 + pitch
                   pan = p + pp
                   pp = p + pp
                   tt = t + tt
                   rot_time = time.time()

                lat1,lon1,distanceboi,teta = geotag.geo_tag_karo(target_x,target_y,vehicle.location.global_relative_frame.alt,pitch,p,vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,vehicle.heading)
                print("Lat: ",lat1,"Lon: ",lon1)
                lat,lon = geotag.geotag(distanceboi,math.radians(p+vehicle.heading),vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon)
                # duri = distance_calculation(lat,lon,plat,plon)
                duri = distanceboi - distanceboibuddha
                print("DURI:",duri)


                if curr_time - timerboi >=1:
                    distanceboibuddha = distanceboi
                    spdo = duri/1
                    plat = lat
                    plon = lon
                    timerboi = time.time()

                spd = vehicle.airspeed + spdo

                if spd >= 3:
                    spd = 3

                # yvel = spd*math.cos(math.radians(p+vehicle.heading))
                # xvel = spd*math.sin(math.radians(p+vehicle.heading))
                if distanceboi >= 8:
                    yvel = (spd+1)*math.cos(math.radians(p+vehicle.heading))
                    xvel = (spd+1)*math.sin(math.radians(p+vehicle.heading))
                else:
                    yvel = spd*math.cos(math.radians(p+vehicle.heading))
                    xvel = spd*math.sin(math.radians(p+vehicle.heading))

                # if distanceboi >= 5:
                send_global_velocity(yvel,xvel,0,0)



            if not success:
                #print('Tracking Lost')
                tracking = False
                tracker_locked = False
    if detected:
        print('locked ')
        tracker = cv2.TrackerKCF_create()
        print("tracker", tracker)
        ret=tracker.init(frame, bbox)
        print("ret:",ret)
        if ret:
            tracker_locked = True
        else:
            print("nhi chal rha open cv check karo ")
        target_x = center_x
        target_y = center_y
    else:
        print("no detection")

    # send_video(frame)



