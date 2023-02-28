from picamera2 import Picamera2, Preview
import time
import cv2 as cv
import numpy as np

picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(main={"size": (640, 640)})

picam2.configure(camera_config)
picam2.start()

prevCircle = None
prevPrevCircle = None

def dist(x1, y1, x2, y2):
    return (x1-x2)**2+(y1-y2)**2

start = time.time()
framesCaptured = 0

while True:
    framesCaptured += 1
    frame = picam2.capture_array()
    frame = cv.flip(frame, 1)

    grayFrame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    blurFrame = cv.medianBlur(grayFrame, 5)
    blurFrame = cv.convertScaleAbs(blurFrame, alpha=0.7, beta=10)
    # blurFrame = cv.adaptiveThreshold(blurFrame, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C,
    #                                  cv.THRESH_BINARY, 11, 3)

    rows = blurFrame.shape[0] / 8
    circles = cv.HoughCircles(blurFrame, cv.HOUGH_GRADIENT, 1.1, rows,
                              param1=100, param2=30, minRadius=40, maxRadius=300)

    # print(circles)
    # should we predict the position if hough doesn't find any circle?
    if circles is not None:
        # there seems to be always at least one circle
        circles = np.round(circles).astype("int")
        chosen = None
        for i in circles[0, :]:
            if chosen is None:
                chosen = i
            if prevCircle is not None:
                d = dist(i[0], i[1], prevCircle[0], prevCircle[1])
                if dist(chosen[0], chosen[1], prevCircle[0], prevCircle[1]) > d:
                    chosen = i
        cv.circle(frame, (chosen[0], chosen[1]), 1, (0, 100, 100), 3)
        cv.circle(frame, (chosen[0], chosen[1]),
                  chosen[2], (255, 0, 0), 3)
        prevPrevCircle = prevCircle
        prevCircle = chosen
        now = time.time()

    cv.imshow("circles", frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

stop = time.time()
fps = framesCaptured / (stop - start)
print(fps)

# picam2.start_preview(Preview.QTGL)
# picam2.start()
# time.sleep(100)
# picam2.capture_file("test.jpg")