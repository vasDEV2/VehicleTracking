import cv2 
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import geotag
import socket
import struct
import time
import math

vehicle = connect("127.0.0.1:14555", wait_ready=False)

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
tim = 0
# lats = [28.75377521,28.75363418,28.75347749,28.75329886,28.75313277,28.75305442,28.75293847,28.75291653,28.75292907,28.75309203,28.75328006,28.75347436,28.75361538,28.75387862,28.75410739,28.75430482,28.75439884,28.75439257,28.75437063,28.75413873,28.75385669]
# lons = [77.11563280,77.11563995,77.11564710,77.11566497,77.11570429,77.11575433,77.11592949,77.11607247,77.11621903,77.11648713,77.11655504,77.11654789,77.11653360,77.11651930,77.11649070,77.11637989,77.11617614,77.11601885,77.11586515,77.11565425,77.11562565]

lats = [28.75455867,28.75395321,28.75390808,28.75369373,28.75348690,28.75309580]
lons = [77.11538911,77.11480573,77.11475855,77.11454836,77.11432530,77.11395211]

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
        vehicle.airspeed = 8
        vehicle.simple_goto(location)

def goto2(location):
        """ Go to a location
        
        Input:
            location    - LocationGlobal or LocationGlobalRelative object
        
        """
        vehicle.airspeed = 3
        vehicle.simple_goto(location)
        while True:
          print("hello",distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,location.lat,location.lon))
          if distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,location.lat,location.lon) <= 0.95*2:
            break

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

def calc_spd(lt,ln):
    # lats = [28.75377521,28.75363418,28.75347749,28.75329886,28.75313277,28.75305442,28.75293847,28.75291653,28.75292907,28.75309203,28.75328006,28.75347436,28.75361538,28.75387862,28.75410739,28.75430482,28.75439884,28.75439257,28.75437063,28.75413873,28.75385669]
    # lons = [77.11563280,77.11563995,77.11564710,77.11566497,77.11570429,77.11575433,77.11592949,77.11607247,77.11621903,77.11648713,77.11655504,77.11654789,77.11653360,77.11651930,77.11649070,77.11637989,77.11617614,77.11601885,77.11586515,77.11565425,77.11562565]   
    
    lats = [28.75455867,28.75395321,28.75390808,28.75369373,28.75348690,28.75309580]
    lons = [77.11538911,77.11480573,77.11475855,77.11454836,77.11432530,77.11395211]
    
    l = len(lats)
    s = 0
    ds = []
    for i in range(l):
        dur = distance_calculation(lt,ln,lats[s],lons[s])
        ds.append(dur)
        s+=1
    xl = min(ds)
    index = ds.index(xl)
    return(index)
    


k = 0
video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
ok, frame = video.read()
# tracker = cv2.TrackerKCF_create()
# bbox = (287,23,86,320)
# ok = tracker.init(frame,bbox)
tracker_locked=False

arm_and_takeoff(15)

startkaro = LocationGlobalRelative(28.7536786,77.1156315,15)

goto2(startkaro)

initiate_search = False
wait_detect = True

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
                initiate_search = False
                wait_detect = True
                counter = 1
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

                lat1,lon1,distanceboi,teta = geotag.geo_tag_karo(314.70596524,target_y,vehicle.location.global_relative_frame.alt-1.9,pitch,p,vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,vehicle.heading)
                print("Lat: ",lat1,"Lon: ",lon1)

                #if curr_time - timerboi >= 1:
                 # if target_x >= 270 and target_x <= 370:
                lat,lon = geotag.geotag(distanceboi,math.radians(vehicle.heading+p),vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon)
                jao = LocationGlobalRelative(lat,lon,15)
                if distanceboi >= 5:
                    goto(jao)
                vecloclat.append(lat)
                vecloclon.append(lon)
                np.savez('tuning_data4',vecloclat,vecloclon)
                #timerboi = time.time()



            if not success:
                #print('Tracking Lost')
                tracking = False
                tracker_locked = False
                initiate_search = True
                sppeeed = vehicle.groundspeed
                ind = calc_spd(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon)
                gimbal_rot = False
                k = 0
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
    elif initiate_search == True:
        print("###SEARCH MODE###")
        
        if gimbal_rot == False:
            pangle,tangle = geotag.theta_calc_xy(target_x,target_y)
            vehicle.gimbal.rotate(int(tangle+tt),0,0)
            pitch = tangle+tt
            pitch = pitch + 90
            tt = tangle + tt
            gimbal_rot = True
        if curr_time - tim >= 3:
            if k != 0:
                wait_detect = False
            tim = time.time()
            k += 1
        
        if wait_detect == False:
            goloc = LocationGlobalRelative(lats[ind+counter],lons[ind+counter],15)
            vehicle.simple_goto(goloc)
            print("SENT GOTO")
            if distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,lats[ind+counter],lons[ind+counter]) <= 2:
                counter += 1 
                if (ind+counter) > (len(lats)-1):
                    ind = 0
                    counter = 0

            
    else:
        print("no detection")

    # send_video(frame)



