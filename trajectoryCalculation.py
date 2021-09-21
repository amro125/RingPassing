import math
import matplotlib.pyplot as plt
import numpy as np
# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.

def findAngles(origin,newpoint):
    angle = math.atan2(newpoint[1]-origin[1], newpoint[0] - origin[0])
    return angle

def findDistance(origin,newpoint):
    distance = ((((newpoint[0] - origin[0]) ** 2) + ((newpoint[1] - origin[1]) ** 2)) ** 0.5)
    return distance

def anglesOfBot(origin,otherbots):
    list = []
    dist =[]
    for newpoint in otherbots:
        list.append(findAngles(origin,newpoint))
        dist.append(findDistance(origin, newpoint))
    return list,dist

class position:
    numArms = 10

    def __init__(self, name):
        self.arm = name
        self.position = []    # creates a new empty list for each dog

    def addposition(self, trick):
        self.tricks.append(trick)



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')
    angle = []
    distance =[]
    coordinates = np.array([[0.0, 0.0], [2.0, 0.0], [4.0, 0.0], [6.0, 0.0], [1.0, 1.0], [3.0, 1.0], [5.0, 1.0], [2.0, 2.0], [3.0, 3.0], [4.0, 2.0]])
    for i in range(len(coordinates)):
        otherbots = np.delete(coordinates, i, 0)
        [list,dist] = anglesOfBot(coordinates[i], otherbots)
        angle.append(list)
        distance.append(dist)
    print(coordinates)
    # print({idx: {"coord": i, "angle": angle[idx]} for idx, i in enumerate(coordinates)})
    a = {idx: {"coord": i, "angle": angle[idx], "Distance": distance[idx]} for idx, i in enumerate(coordinates)}
    print(a[2]["angle"][5])


    robotx = coordinates[:, 0]
    roboty = coordinates[:, 1]
    plt.scatter(robotx, roboty)
    plt.show()
    startbot = input("Please type the starting Robot")
    endx,endy = input("Please type the finishing Robot as x y").split
    endx =int(endx)
    endy = int(endy)
    end = [endx, endy]
    
    startcoord = coordinates[startbot,:]
    desiredA = findAngles(startcoord,end)
    availableA = a[startbot]["angle"]
    deltaA = abs(desiredA - availableA)
    closestd = np.amin(deltaA)
    possibleD = np.
    
    
    
    
