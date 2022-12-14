
import numpy as np
import cv2 as cv
import glob


# Termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((6*4,3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:4].T.reshape(-1,2)
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Import images
images = glob.glob('C:/*.jpg')
counter = 0
print(len(images), "images imported")
cv.waitKey(500)

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray, (6,4), None)
    
    # If corners found
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display corners
        cv.drawChessboardCorners(img, (6,4), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)
        counter +=1
print(counter, " out of ", len(images), "images calibrated")
cv.destroyAllWindows()

#%%
# Calibrate camera (returns camera matrix, dist coeff, rot and trans vectors)
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera matrix : \n")
print(mtx)
print("dist : \n")
print(dist)
