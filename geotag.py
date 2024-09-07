import math
import numpy as np

fx = 351.99349305 
fy = 473.05344122 
cx = 314.70596524 
cy = 251.9527701 
# cx = 320
# cy = 240
s = 0 

f=21


v_fov = 57.6
# v_fov = 50
h_fov = 90.5




# cam_mat_pixel = np.array[[fx, 0., 0.],
                #    [s, fy, 0.],
                #    [cx, cy, 1]]

def geo_tag_karo(xf , yf , alti,gimble_pitch,gimbal_yaw,v_lat,v_lon,yaw):
    xn,yn = normalization(xf,yf)
    zp = zp_calc(alti , gimble_pitch)
    theta= theta_calc( v_fov , h_fov , xf , yf)
    z = z_calc(zp,theta)
    xc,yc,zc=camera_frame_trans(xn,yn,z)
    xb,yb,zb = drone_frame_trans(xc,yc,zc,gimble_pitch,gimbal_yaw)
    dis, the = global_frame_trans(xb,yb,zb,yaw)
    # print("dis:",dis,"theta:",the)
    lat, lon = geotag(dis,the,v_lat,v_lon)
    return lat, lon,dis,the





def normalization(xf,yf):
    xn = (xf-cx)/fx
    yn = -(yf-cy)/fy
    # print("xf:",xf)
    # print("yf:",yf)
    # print("xn:",xn)
    # print("yn:",yn)
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

    # d=math.sqrt((((cx-xf)/sx)**2)+(((cy-yf)/sy)**2))
    x_ = (xf - cx)/sx
    y_ = -(yf - cy)/sy
    theta_pan = math.degrees(math.atan2(x_,f))
    theta_tilt = math.degrees(math.atan2(y_,f))
    

    # x = (xf-cx)/sx
    # y = -(yf-cy)/sy
    # print('x:',x)
    # print('y:',y)


    # print(f)
    # theta = math.atan2(d,f)
    # print(math.degrees(theta))

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
    
    # d = math.sqrt(x_**2 + y_**2)

    # print("d",d)

    # x = (xf-cx)/sx
    # y = -(yf-cy)/sy
    # print('x:',x)
    # print('y:',y)


    # print(f)
    theta = math.atan2(d,f)
    print(math.degrees(theta))

    return theta

def z_calc(zp,thet):
    z = zp/math.cos(thet)
    print("z:",z)
    # print("z",z)
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

    # xb = x*math.cos(pan)+(((y*math.cos(tilt))+(z*math.sin(tilt)))*math.sin(pan))
    # yb = -x*math.sin(pan)+(((y*math.cos(tilt))+(z*math.sin(tilt)))*math.cos(pan))
    # zb = -y*math.sin(tilt)+z*math.cos(tilt)

    # xb = x
    # yb = y*math.cos(tilt) + z*math.sin(tilt)
    # zb = -y*math.sin(tilt) + z*math.cos(tilt)

    xb_ = x
    yb_ = z*math.sin(tilt) - y*math.cos(tilt)
    zb_ = z*math.cos(tilt) + y*math.sin(tilt)
    
    # print("xb",xb)
    # print("yb",yb)
    # print("zb:",zb)

    # zb = zb 
    # xb = xb*math.cos(pan) + yb*math.sin(pan)
    # yb = -xb*math.sin(pan) + yb*math.cos(pan)

    # zo = -yb
    # xo = zb*math.sin(pan) + xb*math.cos(pan)
    # yo = zb*math.cos(pan) - xb*math.sin(pan)

    zb = zb_
    xb = yb_*math.sin(pan) + xb_*math.cos(pan)
    yb = yb_*math.cos(pan) - xb_*math.sin(pan)

    print("xb:", xb)
    print("yb:",yb)
    print("zb:",zb)


    #okay by both

    return xb,yb,zb

def global_frame_trans(xb,yb,zb,hed):
    m = math.sqrt(xb**2 + yb**2)
    th = math.asin(xb/m)
    print("deviation",math.degrees(th))
    print("Distance:", m)
    theta_final = hed + math.degrees(th)
    print("ye vaala theta:", theta_final)
    # print("yaw:",hed)
    return m, math.radians(theta_final)

def geotag(d,theta,lat1,lon1):
    ad = d/6371e3
    lat1 = math.radians(lat1)
    lat = math.degrees(math.asin(math.sin(lat1)*math.cos(ad) + math.cos(lat1)*math.sin(ad)*math.cos(theta)))
    lon = lon1 + math.degrees(math.atan2(math.sin(theta)*math.sin(ad)*math.cos(lat1),math.cos(ad)-math.sin(lat1)*math.sin(lat)))
    return lat, lon

    


