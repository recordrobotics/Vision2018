import cv2

cap = cv2.VideoCapture(1)

while 1:
    r, im = cap.read()

    cv2.imshow("cam", im)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
