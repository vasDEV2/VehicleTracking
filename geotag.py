import math
import numpy as np

## Camera Intrinsics
fx = 351.99349305 
fy = 473.05344122 
cx = 314.70596524 
cy = 251.9527701 

s = 0 

#focal length
f=21

#FOV
v_fov = 57.6
h_fov = 90.5




def geo_tag_karo(yf , alti,gimble_pitch,gimbal_yaw,v_lat,v_lon,yaw):
    xn,yn = normalization(cx,yf)
    zp = zp_calc(alti , gimble_pitch)
    theta= theta_calc( v_fov , h_fov , cx , yf)
    z = z_calc(zp,theta)
    xc,yc,zc=camera_frame_trans(xn,yn,z)
    xb,yb,zb = drone_frame_trans(xc,yc,zc,gimble_pitch,gimbal_yaw)
    dis, the = global_frame_trans(xb,yb,zb,yaw)
    lat, lon = geotag(dis,gimbal_yaw+yaw,v_lat,v_lon)
    return lat, lon, dis





def normalization(xf,yf):
    xn = (xf-cx)/fx
    yn = -(yf-cy)/fy
    return xn,yn

def zp_calc(alti,g_pitch):
    zp = float(alti/math.cos(math.radians(g_pitch)))
    print("alti:",alti)
    print("zp:",zp)
    return zp

def theta_calc_xy(xf , yf):
    aop = v_fov/2
    bop = h_fov/2


    dx = f*math.tan(math.radians(bop))
    dy = f*math.tan(math.radians(aop))

    sx = cx/dx
    sy = cy/dy

    x_ = (xf - cx)/sx
    y_ = -(yf - cy)/sy
    theta_pan = math.degrees(math.atan2(x_,f))
    theta_tilt = math.degrees(math.atan2(y_,f))


    return theta_pan, theta_tilt

def theta_calc( vf , hf , xf , yf):
    aop = vf/2
    bop = hf/2


    dx = f*math.tan(math.radians(bop))
    dy = f*math.tan(math.radians(aop))

    sx = cx/dx
    sy = cy/dy

    d=math.sqrt((((cx-xf)/sx)**2)+(((cy-yf)/sy)**2))

    x_ = (xf - cx)/sx
    y_ = -(yf - cy)/sy

    theta = math.atan2(d,f)
    print(math.degrees(theta))

    return theta

def z_calc(zp,thet):
    z = zp/math.cos(thet)
    print("z:",z)

    return z

def camera_frame_trans(xn,yn,z):
    xc = xn*z
    yc = yn*z
    zc = z 
# 
    print("xc:",xc)
    print("yc:",yc)
    print("zc:",zc)
    #okay (arjun) not ok (vasu)
    return xc,yc,zc 

def drone_frame_trans(x,y,z,tilt,pan):

    tilt = math.radians(tilt)
    pan =  math.radians(pan)

    xb_ = x
    yb_ = z*math.sin(tilt) - y*math.cos(tilt)
    zb_ = z*math.cos(tilt) + y*math.sin(tilt)

    zb = zb_
    xb = yb_*math.sin(pan) + xb_*math.cos(pan)
    yb = yb_*math.cos(pan) - xb_*math.sin(pan)

    return xb,yb,zb

def global_frame_trans(xb,yb,zb,hed):
    m = math.sqrt(xb**2 + yb**2)
    th = math.asin(xb/m)
    theta_final = hed + math.degrees(th)
    
    return m, math.radians(theta_final)

def geotag(d,theta,lat1,lon1):
    ad = d/6371e3
    lat1 = math.radians(lat1)
    lat = math.degrees(math.asin(math.sin(lat1)*math.cos(ad) + math.cos(lat1)*math.sin(ad)*math.cos(theta)))
    lon = lon1 + math.degrees(math.atan2(math.sin(theta)*math.sin(ad)*math.cos(lat1),math.cos(ad)-math.sin(lat1)*math.sin(lat)))
    return lat, lon

    


