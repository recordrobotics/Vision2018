import cv2
import numpy as np
import time

print("OpenCV version: " + cv2.__version__)

cap = cv2.VideoCapture(1)

params = cv2.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 300
params.maxArea = 6000
params.filterByCircularity = False
params.filterByColor = False
params.blobColor = 255
params.filterByConvexity = True
params.minConvexity = 0.3
params.maxConvexity = 1.0
params.filterByInertia = True
params.minInertiaRatio = 0.04
params.maxInertiaRatio = 0.5
detector = cv2.SimpleBlobDetector_create(params)

LBOUND = np.array([45, 0, 250])
UBOUND = np.array([75, 100, 255])

def kernel(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)  
    mask = cv2.inRange(hsv, LBOUND, UBOUND)
    blurred = cv2.blur(mask, (5, 5))
    
    return blurred

test = cv2.resize(cv2.imread("test2.jpg"), (500, 500))

while 1:
    ### MAIN LOOP

    r, bgr = cap.read()
    
    if r:
        start = time.time()
        masked = kernel(bgr)
        points = detector.detect(masked)

        end = time.time()

        ### END MAIN LOOP


        print("FPS: " + str(1.0 / (end - start)))

        if len(points) > 1:
            min_i = 0
            min_j = 1
            min_score = 100000000.0
            for i in range(len(points)):
                for j in range(i + 1, len(points)):
                    s = points[i].size + points[j].size
                    score = abs(points[i].pt[1] - points[j].pt[1]) / s + \
                            abs(points[i].size - points[j].size) / s

                    if points[i].pt[1] > 220 and points[j].pt[1] > 220 and score < min_score:
                        min_i = i
                        min_j = j
                        min_score = score

            display = cv2.drawKeypoints(bgr, [ points[min_i], points[min_j] ], np.array([]), (0, 0, 255),
                                     cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cv2.imshow("yay", display)
        
        cv2.imshow("masked", masked)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
