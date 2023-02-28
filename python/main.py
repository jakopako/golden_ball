import cv2 as cv
import numpy as np
import time
import rtmidi

videoCapture = cv.VideoCapture(0)
videoCapture.set(3, 1280)
videoCapture.set(4, 720)
prevCircle = None
prevPrevCircle = None

last = time.time()
lastMidiCC = [0, 0, 0]
midiCC = [0, 0, 0]

midiout = rtmidi.MidiOut()
midiout.open_port(1)


def dist(x1, y1, x2, y2):
    # x1 = np.uint64(x1)
    # y1 = np.uint64(y1)
    # x2 = np.uint64(x2)
    # y2 = np.uint64(y2)
    return (x1-x2)**2+(y1-y2)**2


def mapRange(n, i1, i2, o1, o2):
    if n < i1:
        n = i1
    if n > i2:
        n = i2
# n - i1 / (i2 -i1) = r - o1 / (o2 -o1)
    return np.round((n - i1)/(i2 - i1)*(o2 - o1)+o1).astype("int")


print(videoCapture.get(cv.CAP_PROP_FPS))

start = time.time()
framesCaptured = 0
while True:
    framesCaptured += 1
    ret, frame = videoCapture.read()
    if not ret:
        break
    frame = cv.flip(frame, 1)

    grayFrame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # grayFrame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # blurFrame = cv.GaussianBlur(grayFrame, (5, 5), 0)
    blurFrame = cv.medianBlur(grayFrame, 5)
    blurFrame = cv.convertScaleAbs(blurFrame, alpha=0.7, beta=10)
    # blurFrame = cv.adaptiveThreshold(blurFrame, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C,
    #                                  cv.THRESH_BINARY, 11, 3)

    rows = blurFrame.shape[0] / 8
    circles = cv.HoughCircles(blurFrame, cv.HOUGH_GRADIENT, 1.1, rows,
                              param1=100, param2=30, minRadius=40, maxRadius=300)

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
                # if d < 160000:
                #     if chosen is not None:
                if dist(chosen[0], chosen[1], prevCircle[0], prevCircle[1]) > d:
                    # if chosen[2] - prevCircle[2]
                    chosen = i
                    # else:
                    #     chosen = i
            # else:
            #     chosen = i
        # if prevCircle is not None:
        #     print(chosen[2])
        #     print(prevCircle)
            # print(dist(chosen[0], chosen[1], prevCircle[0], prevCircle[1]))
        # if chosen is not None:
        #     prevCircle = chosen
        # else:
        #     print("chosen was None")
        #     chosen = prevCircle
        cv.circle(frame, (chosen[0], chosen[1]), 1, (0, 100, 100), 3)
        cv.circle(frame, (chosen[0], chosen[1]),
                  chosen[2], (255, 0, 0), 3)
        prevPrevCircle = prevCircle
        prevCircle = chosen
        now = time.time()
        if now - last >= 0.2:
            midiCC[2] = mapRange(chosen[2], 60, 260, 0, 127)
            if abs(midiCC[2] - lastMidiCC[2]) > 0:
                # print(midiCC)
                note_on = [0xB0, 6, midiCC[2]]
                midiout.send_message(note_on)
                lastMidiCC[2] = midiCC[2]
            midiCC[0] = mapRange(chosen[0], 150, 1150, 0, 127)
            if abs(midiCC[0] - lastMidiCC[0]) > 0:
                # print(midiCC)
                note_on = [0xB0, 4, midiCC[0]]
                # midiout.send_message(note_on)
                lastMidiCC[0] = midiCC[0]
            midiCC[1] = mapRange(chosen[1], 70, 650, 0, 127)
            if abs(midiCC[1] - lastMidiCC[1]) > 0:
                # print(midiCC)
                note_on = [0xB0, 5, midiCC[1]]
                # midiout.send_message(note_on)
                lastMidiCC[1] = midiCC[1]
            # print(chosen)
            last = now
    else:
        # we try to predict the ball's location based on the previous locations
        # not accurate enough...
        # -> linear regression? polynomial regression?
        if prevCircle is not None:
            if prevPrevCircle is not None:
                chX = 2*prevCircle[0] - prevPrevCircle[0]
                chY = 2*prevCircle[1] - prevPrevCircle[1]
                chR = prevCircle[2]
                chosen = (chX, chY, chR)
                cv.circle(frame, (chosen[0], chosen[1]), 1, (0, 100, 100), 3)
                cv.circle(frame, (chosen[0], chosen[1]),
                          chosen[2], (255, 0, 0), 3)
                prevPrevCircle = prevCircle
                prevCircle = chosen

    cv.imshow("circles", frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

stop = time.time()
fps = framesCaptured / (stop - start)
print(fps)
videoCapture.release()
cv.destroyAllWindows()
del midiout
