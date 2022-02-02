               #-----Code for pose estimation  
import time
import cv2
import os
import numpy as np
import glob
import scipy.io as sio
from math import atan
from numpy.linalg import inv
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D

def intersectCirclesRaysToBoard(circles, rvec, t, K, dist_coef):
    imk=cv2.undistortPoints(circles, K, dist_coef)
    circles_normalized = cv2.convertPointsToHomogeneous(imk)
    #circles_normalized = cv2.convertPointsToHomogeneous(circles)
    if not rvec.size:
        return None
 
    R, _ = cv2.Rodrigues(rvec)
 
    # https://stackoverflow.com/questions/5666222/3d-line-plane-intersection
 
    plane_normal = R[:,2] # last row of plane rotation matrix is normal to plane
    plane_point = t.T     # t is a point on the plane
 
    epsilon = 1e-06
 
    circles_3d = np.zeros((0,3), dtype=np.float32)
 
    for p in circles_normalized:
        ray_direction = p / np.linalg.norm(p)
        ray_point = p
 
        ndotu = plane_normal.dot(ray_direction.T)
 
        if abs(ndotu) < epsilon:
            print ("no intersection or line is within plane") 
 
        w = ray_point - plane_point
        si = -plane_normal.dot(w.T) / ndotu
        Psi = w + si * ray_direction + plane_point
 
        circles_3d = np.append(circles_3d, Psi, axis = 0)
 
    return circles_3d
#%%
def into(circles_c,rvec, tvec, K):
    #imk=cv2.undistortPoints(circles, K, dist_coef)
    circles_normalized = cv2.convertPointsToHomogeneous(circles_c)
    circles_normalized=np.squeeze(circles_normalized)
    #circles_normalized = cv2.convertPointsToHomogeneous(circles)
    #if not rvec.size:
        #return None
    R, _ = cv2.Rodrigues(rvec)

    RT=R.copy()
    RT[:,2]=np.squeeze(tvec)
    tform=RT.copy()
    tform=K.dot(tform)
    circles_3d=np.matmul(inv(tform),circles_normalized.T)
    circles_3d=circles_3d.T
    circles_3d[:,0]=np.divide(circles_3d[:,0],circles_3d[:,2])
    circles_3d[:,1]=np.divide(circles_3d[:,1],circles_3d[:,2])
    circles_3d[:,2]=0
    return circles_3d
#%%
    

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img
#%%
def show3d(circles3D):
    fig = pyplot.figure()
    ax = Axes3D(fig)
    #RT=Ip1[0,n].astype(np.float32)
    sequence_containing_x_vals = circles3D[:,0]
    sequence_containing_y_vals = circles3D[:,1]
    sequence_containing_z_vals = circles3D[:,2]
    ax.scatter(sequence_containing_x_vals, sequence_containing_y_vals, sequence_containing_z_vals)
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    '''
    ax.set_xlim([-5,5])
    ax.set_ylim([-5,5])
    '''
    #ax.set_zlim([3,7])
    pyplot.show() 
#%%
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
markerLength = 3.643  # Here, our measurement unit is cm.
markerSeparation = markerLength/5   # Here, our measurement unit is inches.
board = cv2.aruco.GridBoard_create(4, 6, markerLength, markerSeparation, dictionary)
img = board.draw((200*4,200*6))
cv2.imshow('o',img)
save_path='fisheycal_Demo.npz'
with np.load(save_path) as X:
    K, D= [X[i] for i in ('K','D')]
n=0
org_list=[]
dist=np.array([[0,0,0,0]],dtype=np.float32)

Knew=K.copy() 
Knew[0,0]=470
Knew[0,1]=0
Knew[1,1]=Knew[0,0]
Knew[0,2]=831.5
Knew[1,2]=615.5
'''
#images = glob.glob('camt*.jpg')
dist=np.array([[0,0,0,0]],dtype=np.float32)
#images = glob.glob('camt*.jpg')
rvec=np.array([[0,0,0]],dtype=np.float32)
#images = glob.glob('camt*.jpg')
tvec=np.array([[0,0,0 ]],dtype=np.float32)
'''
#%%
img_dir = ".\\UndistortedImageDirectory\\"
data_path = os.path.join(img_dir,'im*.png')

deb=0
rvecs=[] 
tvecs=[]
images = sorted(glob.glob(data_path))
Count=len(images)
for fname in images:
    img = cv2.imread(fname)
    #print(fname)
    #img = cv2.fisheye.undistortImage(img, K, D=D, Knew=Knew)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    #gray=cv2.flip(gray,1) 
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray,dictionary)
    cv2.aruco.refineDetectedMarkers(gray, board, corners, ids, rejectedImgPoints)
    if ids is not None: # if there is at least one marker detected
        retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board,Knew,dist)
        #retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board,Knew,dist, rvec, tvec)
        R, _ = cv2.Rodrigues(rvec)
        #print(rvec)
        #print(R)
        rvecs.append(rvec)
        tvecs.append(tvec)
        if retval != 0:
            #rint(fname)
            im_with_aruco_board = cv2.aruco.drawAxis(img, Knew, dist, rvec, tvec, 10)  # axis length 100 can be changed according to your requirement
    else:
        print(fname)
        im_with_aruco_board=gray 
    if deb:
        cv2.imwrite('ImageX.png',im_with_aruco_board)
        imS = cv2.resize(im_with_aruco_board, (640, 480))
        cv2.imshow('img', imS)
        cv2.waitKey(0)             
cv2.destroyAllWindows()


#%%
#K_proj = cv2.initCameraMatrix2D(objectPointsAccum,Wp, imsize)  
dist_coef_proj=np.array([0,0,0,0])
K_proj=np.zeros([3,3])
K_proj[0,0]=800
K_proj[1,1]=800
K_proj[0,2]=831.5
K_proj[1,2]=615.5


#%%
 
#sio.savemat('Calresults.mat',{'K':K,'D':D,'Kp':K_proj,'Dp':dist_coef_proj,'Knew':Knew})

sio.savemat('orient.mat',{'rvecs':rvecs,'tvecs':tvecs} )