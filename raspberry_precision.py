import time
import math
import cv2
import cv2.aruco as aruco
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import argparse
from array import array
from AP.cpp import *

cap = cv2.VideoCapture(0)
##Aruco
ids_to_find = [72,129]
marker_sizes = [40,19] #cm
marker_heights = [10,5]
takeoff_height = 10
velocity = .5

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters()

##Camera
horizontal_res = 640
vertical_res = 480


horizontal_fov = 62.2 * (math.pi / 180 ) ##Pi cam V1: 53.5 V2: 62.2
vertical_fov = 48.8 * (math.pi / 180)    ##Pi cam V1: 41.41 V2: 48.8

calib_path = "AP/"
camera_matrix = np.loadtxt(calib_path + "cameraMatrix.txt", delimiter=",")
camera_distortion = np.loadtxt(calib_path + "cameraDistortion.txt", delimiter=",")

##Counters and script triggers
found_count=0
notfound_count=0

## Functions for Precision Landing
def arm_and_takeoff(takeoff_Height):
    while not vehicle.is_armable:
        print("waiting to be armable")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(1)

    print("Taking Off")
    vehicle.simple_takeoff(takeoff_Height)

    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.1f m" % v_alt)
        if v_alt >= takeoff_Height - 1.0:
            print("Target altitude reached")
            break
        time.sleep(1)

    vehicle.simple_takeoff(takeoff_height)

    while True:
        print('Current Altitude: %d'%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= .95*takeoff_height:
            break
        time.sleep(1)
    print('Target altitude reached!!!')

    return None

def send_local_ned_velocity(vx,vy,vz):

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       
        0, 
        0,   
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
        0b0000111111000111, 
        0, 
        0, 
        0, 
        vx,
        vy,
        vz, # x, y, z velocity in m/s
        0, 0, 0, 0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_land_message(x,y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,0,0
        )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def lander():

    id_found=0
    frame = cap.read()
    frame = cv2.resize(frame[1],(horizontal_res,vertical_res))
    frame_np = frame
    gray_img = cv2.cvtColor(frame_np,cv2.COLOR_BGR2GRAY)
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
    cv2.imshow("frame", frame)
    cv2.waitKey(1)
    ids=''
    corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)

    altitude = vehicle.location.global_relative_frame.alt 

    counter=0
    corners_np = np.asarray(corners)

    id_to_find=0
    marker_height=0
    marker_size=0

    if altitude > marker_heights[1]:
        id_to_find=ids_to_find[0]
        marker_height=marker_heights[0]
        marker_size=marker_sizes[0]
    elif altitude < marker_heights[1]:
        vehicle.mode = VehicleMode('GUIDED')
        print("Vehicle in GUIDED mode")
        time.sleep(2)

        
    
    ids_array_index=0
    found_id=0
    print("Looking for Marker: "+str(id_to_find)) 
    # Initialize notfound_count here
    notfound_count = 0

    try:
        if ids is not None:
            for id in ids:
                if id == id_to_find:
                    corners_single = [corners[ids_array_index]]
                    corners_single_np = np.asarray(corners_single)
                    ret = aruco.estimatePoseSingleMarkers(corners_single,marker_size,cameraMatrix=camera_matrix,distCoeffs=camera_distortion)
                    (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
                    x = '{:.2f}'.format(tvec[0])
                    y = '{:.2f}'.format(tvec[1])
                    z = '{:.2f}'.format(tvec[2])

                    y_sum = 0
                    x_sum = 0

                    x_sum = corners_single_np[0][0][0][0]+ corners_single_np[0][0][1][0]+ corners_single_np[0][0][2][0]+ corners_single_np[0][0][3][0]
                    y_sum = corners_single_np[0][0][0][1]+ corners_single_np[0][0][1][1]+ corners_single_np[0][0][2][1]+ corners_single_np[0][0][3][1]

                    x_avg = x_sum*.25
                    y_avg = y_sum*.25
            
                    x_ang = (x_avg - horizontal_res*.5)*(horizontal_fov/horizontal_res)
                    y_ang = (y_avg - vertical_res*.5)*(vertical_fov/vertical_res)
            
                    if vehicle.mode!='LAND':
                        vehicle.mode = VehicleMode('LAND')
                        # while vehicle.mode!='LAND':
                        #     time.sleep(1)
                        print("------------------------")
                        print("Vehicle now in LAND mode")
                        print("------------------------")
                        send_land_message(x_ang,y_ang)
                    else:
                        send_land_message(x_ang,y_ang)
                        pass
                        print("X CENTER PIXEL: "+str(x_avg)+" Y CENTER PIXEL: "+str(y_avg))
                        print("FOUND COUNT: "+str(found_count)+" NOTFOUND COUNT: "+str(notfound_count))
                        print("MARKER POSITION: x=" +x+" y= "+y+" z="+z)
                        found_count = found_count + 1
                        found_id=1
                        break
                    ids_array_index=ids_array_index+1
                if found_id==0:
                    notfound_count=notfound_count +1
        else:
            notfound_count=notfound_count+1
    except Exception as e:
        print('Target likely not found. Error: '+str(e))
        notfound_count=notfound_count+1
    else:
        return None


parser = argparse.ArgumentParser()
parser.add_argument("--connect", default="0.0.0.0:14552")
args = parser.parse_args()
print("Connecting")
vehicle = connect(args.connect)


##
##SETUP PARAMETERS TO ENABLE PRECISION LANDING
##
vehicle.parameters['PLND_ENABLED'] = 1
vehicle.parameters['PLND_TYPE'] = 1 ##1 for companion computer
vehicle.parameters['PLND_EST_TYPE'] = 0 ##0 for raw sensor, 1 for kalman filter pos estimation
vehicle.parameters['LAND_SPEED'] = 20 ##Descent speed of 30cm/s

while True:
    lander()
    #time.sleep(2)

