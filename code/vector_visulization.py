import matplotlib.pyplot as plt
import numpy as np
import cv2
from numpy import ma
from numpy import linalg as LA

def draw_flow(img, flow, step=20):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1)
    fx, fy = flow[y,x].T *18
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = img
    cv2.polylines(vis, lines, 0, (0, 255, 0))
    #print(flow.shape)
    #for (x1, y1), (x2, y2) in lines:
        #cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis

vertical_views = (np.arange(244, dtype=float)-122.0)/244.0*(100.0/180.0*np.pi)
horizontal_views = (np.arange(324, dtype=float)-162.0)/324.0*(100.0/180.0*np.pi)

D = np.zeros([244,324,3])
for i in range(244):
    for j in range(324):
        D[i,j]=np.array([np.tan(vertical_views[i]), np.tan(horizontal_views[j]),-1])
        D[i,j] /= LA.norm(D[i,j])

a = np.array([0, 1, 0])
matched_filter = np.cross(np.cross(D,a),D)[:,:,0:2]

print(matched_filter.shape)

img = np.ones([244,324,3])
vector_map = draw_flow(img, matched_filter)
cv2.imshow('vector_map', vector_map)
cv2.waitKey()
cv2.destroyAllWindows()
