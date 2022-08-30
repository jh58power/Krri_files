import numpy as np
from numpy.core.records import fromfile
import pandas as pd
import math

#input type (Obs의 8개의 points)
input_type = [[-1585.75939941,  1710.73718262,    29.77194977], #flh
 [-1585.75756836,  1710.73828125,    28.34895134], #fll
 [-1585.25549316 , 1709.03417969,    29.77128983], #frh
 [-1585.25378418,  1709.03527832,    28.3482914 ], #frl
 [-1581.6361084 ,  1711.95727539,    29.77194977], #rlh
 [-1581.63427734,  1711.95837402,    28.34895134], #rll
 [-1581.13220215,  1710.25427246,    29.77128983], #rrh
 [-1581.13049316,  1710.25537109,    28.3482914 ]] #rrl

#Return Yaw (Yaw 값 계산)
def get_yaw(Object):
    frl = [Object[0][0],Object[0][1]] # [x,y]
    rrl = [Object[4][0],Object[4][1]] # [x,y] 
    
    x = frl[0] - rrl[0]
    y = frl[1] - rrl[1]
    return math.atan2(y,x)

# Return [x,y,z] (Obs 의 중심점 계산)
def get_centerXYZ(Object):
    center = np.array([sum(Object[:,0]),sum(Object[:,1]),sum(Object[:,2])])/8
    return center

#두 점사이의 거리 계산
def find_dist(p1,p2):
    a = p1[0] - p2[0]
    b = p1[1] - p2[1]
    c = math.sqrt((a*a)+(b*b))
    return c


#Return [width,len,height]   (w, l, h 계산)         
def get_wlh(Object):
    wid = find_dist([Object[0][0],Object[0][1]],[Object[2][0],Object[2][1]]) #flh, frh
    len = find_dist([Object[0][0],Object[0][1]],[Object[4][0],Object[4][1]]) #flh, rlh
    hei = Object[0][2] - Object[1][2] #flh, fll
    
    return [len,wid,hei]
