import cv2
import numpy as np
import os
import time
y1 = 360
y2 = 1560
path = []
visited = []


def ham_cycle(graph, current):
    global path
    global visited
    if (current == len(graph)):
        return True
    for i in range(1, len(graph)):
        if (visited[i] == False and i in graph[path[current - 1]][2]):
            visited[i] = True
            path.append(i)
            if (ham_cycle(graph, current + 1)):
                return True
            path.pop()
            visited[i] = False
    return False
def act(pt):
    os.system('adb shell input tap %d %d' % (pt[0], y1+pt[1]))
    print('acting',pt)


def ham_cycle_start(graph):
    global path
    global visited
    init_path = []
    init_path.append(0)
    init_visited = [False for i in range(0, len(graph))]
    visited.insert(0, True)
    path = init_path
    visited = init_visited
    if (ham_cycle(graph, 1) == None):
        print 'no path'
        return None
    for i in path:
        act(graph[i][0:2])
    time.sleep(1)
    act([540,700])


def get_roi(input):
    img = input[y1:y2, :]
    cv2.namedWindow('img', 0)

    ret, binary = cv2.threshold(255 - img, 7, 255, cv2.THRESH_BINARY)
    # cv2.imshow('origin', binary)
    _, contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (0, 0, 255), 3)
    result = []
    box_size = 0
    for contour in contours:
        if(cv2.contourArea(contour)<36):
            continue
        contour = np.array(np.squeeze(contour))
        max_x, max_y = np.max(contour, axis=0)
        min_x, min_y = np.min(contour, axis=0)
        dx = max_x - min_x
        dy = max_y - min_y
        box_size = dx
        if (dy -dx > 5):
            ave_x = (max_x + min_x) / 2
            for i in range(0, dy / dx):
                ave_y = max_y - (max_x - min_x) * (i * 2 + 1) / 2
                if (img[ave_y, ave_x] !=249 and img[ave_y, ave_x] !=209):
                    result.insert(0, [ave_x, ave_y])
                else:
                    result.append([ave_x, ave_y])
        else:

            ave_x = (max_x + min_x) / 2
            ave_y = (max_y + min_y) / 2
            result.append([ave_x, ave_y])
    for i, pt in enumerate(result):
        cv2.putText(img, str(i), (pt[0], pt[1]), 0, 2, (0, 0, 0), 5)
        cv2.circle(img, (pt[0], pt[1]), 5, (0, 0, 0), -1)
    for i, pt in enumerate(result):
        near = []
        for j, pt2 in enumerate(result):
            if pt2 != pt and abs((pt[0] - pt2[0])) + abs((pt[1] - pt2[1])) < box_size * 3 / 2:
                near.append(j)
        result[i].append(near)

    cv2.imshow("img", img)
    cv2.waitKey(10)
    ham_cycle_start(result)


def get_img():
    os.system('adb shell screencap -p /sdcard/test.png')
    os.system('adb pull /sdcard/test.png .')
    img = cv2.imread('test.png', 0)
    return img




while True:
    image = get_img()
    get_roi(image)
    time.sleep(1)
    print('next')
