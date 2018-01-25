import cv2

cap = cv2.VideoCapture(1)

while 1:
	_, im = cap.read()

	im = cv2.Canny(im, 70, 200)

	cv2.imshow("image", im)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
