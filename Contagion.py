import os
import sys
import time
import csv
import pdb
from queue import Queue
from threading import Thread
import time
import math
import numpy as np


def cvt(path):
    return_vector = []
    for i in range(7):
        return_vector.append(float(path[i]))
    return return_vector


def playDance(step,robots):
    length = len(step)
    # for a in range (3):
    #     print(2-a)
    #     time.sleep(1)
    for i in range(length):
        start_time = time.time()
        # board.digital[4].mode = pyfirmata.INPUT
        # board.digital[2].mode = pyfirmata.INPUT
        # red = board.digital[4].read()

        j_angles = (step[i])
        # b=6
        # arms[b].set_servo_angle_j(angles=j_angles[(b * 7):((b + 1) * 7)], is_radian=False)

        for b in robots:
            arms[b].set_servo_angle_j(angles=j_angles, is_radian=False)
        tts = time.time() - start_time
        sleep = 0.006 - tts

        if tts > 0.006:
            sleep = 0

        print(tts)
        time.sleep(sleep)


def readFile(csvFile):
    flower = []
    with open(csvFile, newline='') as csvfile:
        paths_reader_j = csv.reader(csvfile, delimiter=',', quotechar='|')
        for path in paths_reader_j:
            flower.append(cvt(path))
    return flower


def setup():
    for a in arms:
        a.set_simulation_robot(on_off=False)
        #a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        a.set_servo_angle(angle=[0.0, 0.0, 0.0, 1.57, 0.0, 0, 0.0], wait=False, speed=0.4, acceleration=0.25,
                          is_radian=True)



def findAngles(origin, newpoint):
    angle = math.atan2(newpoint[1] - origin[1], newpoint[0] - origin[0])
    return angle


def findDistance(origin, newpoint):
    distance = ((((newpoint[0] - origin[0]) ** 2) + ((newpoint[1] - origin[1]) ** 2)) ** 0.5)
    return distance


def anglesOfBot(origin, otherbots):
    list = []
    dist = []
    for newpoint in otherbots:
        list.append(findAngles(origin, newpoint))
        dist.append(findDistance(origin, newpoint))
    return list, dist

def findNeighbors(robot,robots):
    otherbotname = np.delete(robots, robot)
    potneighbor = a[robot]["Distance"]
    closestN = np.amin(potneighbor)
    possibleN = np.argwhere(potneighbor == closestN)
    neighbors = []
    for n in possibleN:
        neighbors.append(otherbotname[int(n)])
    return neighbors


if __name__ == "__main__":
    ROBOT = "xArms"
    PORT = 5004

    sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
    from xarm.wrapper import XArmAPI

    arm1 = XArmAPI('192.168.1.208')
    arm2 = XArmAPI('192.168.1.244')
    arm3 = XArmAPI('192.168.1.203')
    arm4 = XArmAPI('192.168.1.236')
    arm5 = XArmAPI('192.168.1.226')
    arm6 = XArmAPI('192.168.1.242')
    arm7 = XArmAPI('192.168.1.215')
    arm8 = XArmAPI('192.168.1.234')
    arm9 = XArmAPI('192.168.1.237')
    arm10 = XArmAPI('192.168.1.204')


    arms = [arm1, arm2, arm3, arm4, arm5, arm6, arm7, arm8, arm9, arm10]
    totalArms = len(arms)


    directory = '/home/forest/Desktop/xArm/contagion/'
    dances = []
    trajectories = sorted(os.listdir(directory))
    s = time.time()
    for filename in trajectories:
        if filename.endswith(".csv"):
            print(filename)
            currentDance = (readFile(os.path.join(directory, filename)))
            dances.append(currentDance)
            continue
    e = time.time()
    print("the loading time is", (e-s))
    setup()
    repeat = input("do we need to repeat? [y/n]")
    if repeat == 'y':
        print('state:', arm1.state)
        print('mode:', arm1.mode)
        setup()
    for a in arms:
        a.set_mode(1)
        a.set_state(0)

    angle = []
    distance = []
    robots = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    coordinates = np.array(
        [[0.0, 0.0], [2.0, 0.0], [4.0, 0.0], [6.0, 0.0], [1.0, 1.0], [3.0, 1.0], [5.0, 1.0], [2.0, 2.0], [3.0, 3.0],
         [4.0, 2.0]])
    for i in range(len(coordinates)):
        otherbots = np.delete(coordinates, i, 0)
        [list, dist] = anglesOfBot(coordinates[i], otherbots)
        angle.append(list)
        distance.append(dist)
    print(coordinates)
    # print({idx: {"coord": i, "angle": angle[idx]} for idx, i in enumerate(coordinates)})
    a = {idx: {"coord": i, "angle": angle[idx], "Distance": distance[idx], "name": robots[idx]} for idx, i in
         enumerate(coordinates)}
    print(a[2]["angle"][5])



    # plt.show()
    startbot = []
    dance = int(input("Please Type the Dance"))
    firstbot = int(input("Please type the starting Robot"))

    startbot.append(firstbot)

    # endbot = int(input("Please type the finishing Robot"))
    neighbors = []
    neighbors.append(firstbot)
    while len(startbot) < len(robots):
        playDance(dances[dance-1], startbot)
        # otherbotname = np.delete(robots, startbot)
        # startcoord = coordinates[startbot, :]
        # find neighbors pf current bot

        for r in startbot:
            neighborstoAdd = findNeighbors(r, robots)
            for n in neighborstoAdd:
                neighbors.append(int(n))
        print("neighbors are:", neighbors)

        test = np.sort(neighbors, axis=None)
        print("sorted are:", test)
        test = np.unique(test)
        print(test)
        #input("continue?")

        #        robotminspot = otherbotname[int(possibleD[int(minspot)])]

        startbot = test
    playDance(dances[dance-1], startbot)


