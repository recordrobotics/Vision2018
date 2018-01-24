import cv2

cap = cv2.VideoCapture(1)

idx = 0

while 1:
	_, im = cap.read()

	cv2.circle(im, (im.shape[1] / 2, im.shape[0] / 2), 10, (0, 0, 250), 2)

	cv2.imshow("cam", im)

	c = cv2.waitKey(1)

	if c & 0xFF == ord('q'):
		break
	elif c & 0xFF == ord('c'):
		cv2.imwrite("./imgs/" + str(idx) + "cap.jpg", im)
		idx = idx + 1

cap.release()
cv2.destroyAllWindows()
