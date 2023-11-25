import numpy as np
import cv2 as cv
import glob
import argparse
import sys
from datetime import datetime

#get eventual input arguments
parser = argparse.ArgumentParser(description='Chessboard pattern specifications.')
parser.add_argument('-x', '--rows', help="number of inside corners in x. [default=9]")
parser.add_argument('-y', '--columns', help="number of inside corners in y. [default=6]")
args = parser.parse_args()
print("input arguments: "+str(args))

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# chessboard pattern angles
nx = 9  #number of inside corners in x
ny = 6  #number of inside corners in y

#if specified, set nx and ny according to the input parameters
if args.rows:
    nx = int(args.rows)
if args.columns:
    ny = int(args.columns)

print("nx = "+str(nx))
print("ny = "+str(ny))

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((nx*ny,3), np.float32)

objp[:,:2] = np.mgrid[0:nx,0:ny].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('*.png')
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    #cv.imshow('gray', gray)
    #cv.waitKey(500)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (nx,ny), None)

    print(ret)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (nx,ny), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)


print("All calibration images have been processed. Press q to get the parameters.")

while True:
    key = cv.waitKey(1)

    if key == ord('q'):
        break

cv.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

#Intrinsic camera matrix
print("Camera matrix : \n")
print(mtx)

#Lens distortion coefficients.
print("dist : \n")
print(dist)

#Rotation specified as a 3×1 vector. The direction of the vector specifies the axis of rotation and the magnitude of the vector specifies the angle of rotation.
print("rvecs : \n")
print(rvecs)

#3×1 Translation vector
print("tvecs : \n")
print(tvecs)

################################################
# Re-projection error
# The closer the re-projection error is to
# zero, the more accurate the parameters we
# found are.
################################################
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)) )
################################################

################################################
#Write a log file with the parameters
################################################
f = open(f"calibration_parameters_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt", "w")
f.write("Camera matrix : \n")
f.write(str(mtx))

#Lens distortion coefficients.
f.write("dist : \n")
f.write(str(dist))
f.write("\n")

#Rotation specified as a 3×1 vector. The direction of the vector specifies the axis of rotation and the magnitude of the vector specifies the angle of rotation.
f.write("rvecs : \n")
f.write(str(rvecs))
f.write("\n")

#3×1 Translation vector
f.write("tvecs : \n")
f.write(str(tvecs))
f.write("\n")

#Re-projection error
f.write("Re-projection error : \n")
f.write( "total error: {}".format(mean_error/len(objpoints)) )

f.close()
################################################