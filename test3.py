import cv2
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

def find_histogram(clt):
    """
    create a histogram with k clusters
    :param: clt
    :return:hist
    """
    numLabels = np.arange(0, len(np.unique(clt.labels_)) + 1)
    (hist, _) = np.histogram(clt.labels_, bins=numLabels)

    hist = hist.astype("float")
    hist /= hist.sum()

    return hist
def plot_colors2(hist, centroids):
    bar = np.zeros((50, 300, 3), dtype="uint8")
    startX = 0

    for (percent, color) in zip(hist, centroids):
        # plot the relative percentage of each cluster
        endX = startX + (percent * 300)
        cv2.rectangle(bar, (int(startX), 0), (int(endX), 50),
                      color.astype("uint8").tolist(), -1)
        startX = endX

    # return the bar chart
    return bar

cap = cv2.VideoCapture(0)

# Specify HSV range of object
objectRGBLower = np.array([80, 0, 0])
objectRGBUpper = np.array([255, 120, 120])

while True:
	# Read captured webcam frame
    _, frame = cap.read()

	# Threshold the HSV image to get Tennis ball only
    mask = cv2.inRange(frame, objectRGBLower, objectRGBUpper)

	# Find contours from HSV masked image
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    areas = [cv2.contourArea(c) for c in contours]

    # If there are countours
    if len(areas) > 0:
        max_index = np.argmax(areas)
        count = contours[max_index]
        x,y,w,h = cv2.boundingRect(count)
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0), 2)

        # Display the resulting frame
        cv2.imshow('frame',frame)
        
        rect = frame[y:y+h,x:x+w]
        img = cv2.cvtColor(rect, cv2.COLOR_BGR2RGB)

        img = img.reshape((img.shape[0] * img.shape[1],3)) #represent as row*column,channel number
        clt = KMeans(n_clusters=3) #cluster number
        clt.fit(img)

        hist = find_histogram(clt)
        bar = plot_colors2(hist, clt.cluster_centers_)

        plt.axis("off")
        plt.imshow(bar)
        plt.show()

	# If "q" is pressed on the keyboard, 
    # exit this loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
 
cap.release()
cv2.destroyAllWindows()
