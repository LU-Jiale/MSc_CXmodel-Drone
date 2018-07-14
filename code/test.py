import numpy as np
import cv2
from numpy import linalg as LA
import time

a=np.ones([100,100,3], dtype = float)

start_time = time.time()
mag2 = np.sqrt(np.power(a[:,:,0], 2)+np.power(a[:,:,1], 2)+np.power(a[:,:,2], 2))
elapsed_time = time.time() - start_time
print elapsed_time

start_time = time.time()
mag1 = LA.norm(a, axis = 2)
elapsed_time = time.time() - start_time
print elapsed_time

start_time = time.time()
mag2 = np.sqrt(np.power(a[:,:,0], 2)+np.power(a[:,:,1], 2)+np.power(a[:,:,2], 2))
elapsed_time = time.time() - start_time
print elapsed_time

start_time = time.time()
mag1 = LA.norm(a, axis = 2)
elapsed_time = time.time() - start_time
print elapsed_time

t1=0
t2=0
for i in range(100):

    start_time = time.time()
    mag2 = np.sqrt(np.power(a[:,:,0], 2)+np.power(a[:,:,1], 2)+np.power(a[:,:,2], 2))
    elapsed_time = time.time() - start_time
    t1 += elapsed_time

    start_time = time.time()
    mag1 = LA.norm(a, axis = 2)
    elapsed_time = time.time() - start_time
    t2 += elapsed_time

print t1,t2

