# import the necessary packages
from imutils import paths
import numpy as np
import imutils
import cv2
import time
import math
 
def find_marker(image):
	
	# convert the image to grayscale, blur it, and detect edges	
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (9, 9), 0) 

	edged = cv2.Canny(gray, 35, 125)
 
	# find the contours in the edged image and keep the largest one;
	cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if imutils.is_cv2() else cnts[1]
	c = max(cnts, key = cv2.contourArea)
	# compute the bounding box of the of the paper region and return it
	return  cv2.minAreaRect(c)
	
		
def distance_to_camera(knownWidth, focalLength, perWidth):
	# compute and return the distance from the maker to the camera
	return (knownWidth * focalLength) / perWidth
	
def filter_to_color (frame, ranges3):
	#Save the ranges that are imported
	ranges4 = ranges3
	
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
	#Create boundaries
	lower_green = np.array([ranges4[0], ranges4[2], ranges4[4]])
	upper_green = np.array([ranges4[1], ranges4[3], ranges4[5]])
	mask = cv2.inRange(hsv, lower_green, upper_green) 
	res = cv2.bitwise_and(frame,frame, mask= mask) 
	
	boundaries = [
		( [ranges4[0], ranges4[2], ranges4[4]], [ranges4[1], ranges4[3], ranges4[5]])]
	for (lower, upper) in boundaries:
			# create NumPy arrays from the boundaries
			lower = np.array(lower, dtype = "uint8")
			upper = np.array(upper, dtype = "uint8")
			# find the colors within the specified boundaries and apply
			# the mask
			mask = cv2.inRange(hsv, lower, upper)
			output = cv2.bitwise_and(frame, frame, mask = mask)
	return output
		
def	drunkTest(ranges1):
	#Save ranges that are imported
	ranges2 = ranges1
	
	frame_height = 960
	frame_width = 1280
	
	#Set the thresholds for failing
	fail = False
	failcount = 0;
	
	
	# initialize the known distance from the camera to the object
	KNOWN_DISTANCE = 12.0
	 
	# initialize the known object width
	KNOWN_WIDTH = 9.5
	 
	#Set distances needed to walk back and fourth
	desiredDistance = 60
	returnThreshold  = 18
	

	time.sleep(3)
	cap = cv2.VideoCapture(0)
	
	#Take frame, filter, and find object
	while(True):
		try:
			ret, frame = cap.read()
			frame = cv2.flip(frame,1)
			output = filter_to_color(frame, ranges2)
			marker = find_marker(output)
			break
		except:
			pass

	focalLength = (450 * KNOWN_DISTANCE) / KNOWN_WIDTH

	distanceNotReached = True;
	returnNotReached = True;
	distanceReachedCount = 0;
	
	#Continually repeat until distances are reached
	while(distanceNotReached or returnNotReached):
		inches = 200
		rect_width = 330
		rect_length = 430
		change_threshold = 0.7
		flag = True
		while (flag == True):
			while(True):
				try:
					ret, frame = cap.read()
					frame = cv2.flip(frame,1)
					output = filter_to_color(frame, ranges2)
					marker = find_marker(output)
					break
				except:
					pass

			
			
			inches = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])
			
			#The object should never be more than eight feet away
			if inches < 120:
				flag = False
				
		#Prompt player to walk forward at seven feet		
		if distanceNotReached == False:
		#if True:
			cv2.putText(output, "Walk Forward, Now", (200, 480), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
			cv2.putText(frame, "Walk Forward, Now", (200, 480), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 4)
			if inches < returnThreshold:
				returnNotReached = False
				
		#Player is back within two feet
		if inches > desiredDistance:
			distanceReachedCount = distanceReachedCount +1
			if distanceReachedCount == 15:
				distanceNotReached = False
		
		Fail_Range = (frame_width/1.5)*12/inches;
		leftThreshold = int(math.floor(frame_width/2-Fail_Range/2))
		rightThreshold = int(math.floor(frame_width/2 + Fail_Range/2))
		
		#Show "straight" boundaries
		cv2.line(output,(leftThreshold,0),(leftThreshold, frame_height),(0,0,255),3)
		cv2.line(output,(rightThreshold,0),(rightThreshold,frame_height),(0,0,255),3)
		
		#If player goes outside boundaries at any time, he/she fails
		if marker[0][0] < leftThreshold or marker[0][0] > rightThreshold:
			failcount = failcount+1
		
		#Display marker and distance
		box = cv2.boxPoints(marker)
		box = np.int0(box)
		cv2.drawContours(output, [box], -1, (0, 255, 0), 2)
		cv2.putText(output, "%.2fft" % (inches / 12), (640, 480), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 3)
		#cv2.imshow('output',output) 
		#cv2.putText(frame, "%.2fft" % (inches / 12), (frame_height - 200, frame_width - 20), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 3)
		cv2.imshow('frame',frame)	
		#print(output.shape[0])

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()
	print(failcount)
	if (failcount > 10):
		fail = True
	if(fail == True):
		return False
	else:
		return True

def calibration ():
	#Set parameters for box
	frame_height = 960
	frame_width = 1280
	center_x = int(round(frame_width/2))
	center_y = int(round(frame_height/2))
	box_width = int(round(frame_width/10))
	box_height = int(round(frame_height/8))

	while True:
		try:
			#Intitialize camera and timer
			start_time = time.time()
			cap1 = cv2.VideoCapture(0)
			
			#Countdown and display box
			i  = 0
			count_down = 15
			count_down_color = (0,255,0)
			flag = True
			while flag:
				ret, frame = cap1.read()
				frame = cv2.flip(frame,1)
				if (time.time() - start_time > 15):
					flag = False
				if(time.time() - start_time > i):
					count_down  = (count_down-1)
					i = i +1
				if (count_down > 3):
					count_down_color = (0,255,0)
				else:
					count_down_color = (0,0,255)
				#cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
				#cv2.resizeWindow('frame', ((frame_width), (frame_height)))
				count_down = count_down +1
				cv2.rectangle(frame, (center_x + box_width+10, center_y + box_height+10), (center_x - box_width- 10, center_y - box_height -10), (255,0,0), 5)
				cv2.putText(frame, "%.0f" % count_down, (center_x-30, frame_height - 800), cv2.FONT_HERSHEY_SIMPLEX, 2.0, count_down_color, 3)
				#frame = frame[center_y - box_height:center_y + box_height, center_x - box_width: center_x + box_width]
				cv2.imshow('frame',frame)
				count_down = count_down -1;
				if cv2.waitKey(1) == 27:
					break  # esc to quit
			break
		except:
			print("error caught")
			pass
	
	#Crop only box from image
	cropped = frame[center_y - box_height:center_y + box_height, center_x - box_width: center_x + box_width]
	cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
	
	#Initialize parameters
	H_min = 179
	H_max = 0
	S_min = 255
	S_max = 0
	V_min = 255
	V_max = 0
	
	#Find minimum and maxmimum pixel by pixel
	i = 1
	j = 1
	while (i< 2*box_height):
		while(j< 2*box_width):
			color = cropped[i,j]
			if(color[0] < H_min):
				H_min = color[0]
			if (color[0] > H_max):
				H_max = color[0]
			if(color[1] < S_min):
				S_min = color[1]
			if(color[1] > S_max):
				S_max = color[1]
			if(color[2] < V_min):
				V_min = color[2]
			if(color[2] > V_max):
				V_max = color[2]
			j = j+1;
		i = i+1
	
	
			
	cap1.release()
	cv2.destroyAllWindows()
	
	#Widen ranges by multiples
	H_min = H_min*0.85
	H_max = min(H_max*1.2, 179)
	S_min = S_min*0.7
	S_max = min(S_max*1.7, 255)
	V_min = V_min*0.5
	V_max = min(V_max*1.7, 255)
	
	print(H_min)
	print(H_max)
	print(S_min)
	print(S_max)
	print(V_min)
	print(V_max)
	return	[H_min, H_max, S_min, S_max, V_min, V_max]


a = calibration()
b = drunkTest(a)
print(b)