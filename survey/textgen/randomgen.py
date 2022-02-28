import numpy as np
import cv2

def main():
	image = cv2.imread("original.jpg")

	# print(image)
	h, w = np.shape(image)[0:2]
	cv2.circle(image, (int(w/2), int(h/2)), 20, (0,0,0), -1)
	print(h,w)

	# cv2.imwrite("final.jpg",image)

	cv2.imshow("image",image)
	cv2.waitKey(0)

if __name__ == '__main__':
	main()