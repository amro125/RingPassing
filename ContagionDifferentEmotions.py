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
import random
from pythonosc import udp_client


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

        #print(tts)
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

def sendSound(robot,sound):
    IP = "192.168.1.73"
    PORT_TO_MAX = 7980
    message = [int(robot), int(sound)]
    time.sleep(1)

    client = udp_client.SimpleUDPClient(IP, PORT_TO_MAX)
    client.send_message('arms', message)

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

    def playRobot(in_q):
        happy = [3, 4, 5]
        angry = [6, 1, 10]
        sad = [7, 8, 9]
        calm = [11]
        emotions = [sad, happy, angry, calm]
        while True:
            todo = in_q.get()
            robot = todo[1]
            cat = todo[0]
            category = emotions[cat]
            chaos = random.randint(0, 10)
            if chaos == 5:
                newcat = random.randint(0,3)
                category = emotions[newcat]
            toplay = random.choice(category)
            print("sending to robot", robot, "sound", toplay)
            sendSound(robot, toplay)

            playDance(dances[toplay], [robot])


    q = Queue()
    q1 = Queue()
    q2 = Queue()
    q3 = Queue()
    q4 = Queue()
    q5 = Queue()
    q6 = Queue()
    q7 = Queue()
    q8 = Queue()
    q9 = Queue()

    quay = [q, q1, q2, q3, q4, q5, q6, q7, q8, q9]



    t2 = Thread(target=playRobot, args=(q,))
    t3 = Thread(target=playRobot, args=(q1,))
    t4 = Thread(target=playRobot, args=(q2,))
    t5 = Thread(target=playRobot, args=(q3,))
    t6 = Thread(target=playRobot, args=(q4,))
    t7 = Thread(target=playRobot, args=(q5,))
    t8 = Thread(target=playRobot, args=(q6,))
    t9 = Thread(target=playRobot, args=(q7,))
    t10 = Thread(target=playRobot, args=(q8,))
    t11 = Thread(target=playRobot, args=(q9,))
    t2.start()
    t3.start()
    t4.start()
    t5.start()
    t6.start()
    t7.start()
    t8.start()
    t9.start()
    t10.start()
    t11.start()


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
    dance = int(input("Please Type the Quadrant"))
    firstbot = int(input("Please type the starting Robot"))
    time.sleep(5)
    lastPlayed = []
    startbot.append(firstbot)

    # endbot = int(input("Please type the finishing Robot"))
    neighbors = []
    neighbors.append(firstbot)
    Mainlist = startbot
    while len(startbot) < len(robots):

        for play in Mainlist:
            quay[play].put([dance, play])
        #playDance(dances[dance-1], startbot)
        #input()
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
        Mainlist = np.setdiff1d(test, startbot)
        print("Main list is:", Mainlist)
        #input("continue?")
        time.sleep(2)

        #        robotminspot = otherbotname[int(possibleD[int(minspot)])]

        startbot = test
    for play in Mainlist:
        quay[play].put([dance, play])


