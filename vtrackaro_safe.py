import cv2 
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import geotag
import socket
import struct
import time

vehicle = connect("127.0.0.1:14450", wait_ready=False)

# Define the IP and port of the ground station
ground_station_ip = '192.168.191.112'
ground_station_port = 5780

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

# Initialize the socket connection
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((ground_station_ip, ground_station_port))

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


video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
ok, frame = video.read()
# tracker = cv2.TrackerKCF_create()
# bbox = (287,23,86,320)
# ok = tracker.init(frame,bbox)
tracker_locked=False
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
                   vehicle.gimbal.rotate(int(t+tt),0,int(p+pp))
                   pitch = t + tt
                   pitch = 90 + pitch
                   pan = p + pp
                   pp = p + pp
                   tt = t + tt
                   rot_time = time.time()

                lat,lon,distanceboi,teta = geotag.geo_tag_karo(target_x,target_y,vehicle.location.global_relative_frame.alt,pitch,pan,vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,vehicle.heading)
                print("Lat: ",lat,"Lon: ",lon)

                if curr_time - timerboi >= 0.1:
                    dissy.append(distanceboi)
                    tets.append(teta)
                    vecloclat.append(vehicle.location.global_relative_frame.lat)
                    vecloclon.append(vehicle.location.global_relative_frame.lon)
                    np.savez("datadekhlebsdk", dissy,tets,vecloclat,vecloclon)
                    timerboi = time.time()



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



