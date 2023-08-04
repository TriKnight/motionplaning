#!/usr/bin/env python
#Depth First Search (DFS) – Iterative and Recursive Implementation
#import what we need
import numpy as np
import matplotlib.pyplot as plt
import random

#load the grid 

grid_load = np.load('cspace.npy')
#grid =np.array(grid_load[:,1])
grid =np.transpose(grid_load)
grid[:1800][:1120]
print('Grid dimensions')
print(grid.shape)
print('Grid data')
#start and goal positions as numpy arrays
start = np.array([100.0, 100.0])
goal = np.array([1600.0, 750.0])
#the goal region
goalCircle = plt.Circle((goal[0], goal[1]), 25, color='b', fill = False)
#a new figure
fig = plt.figure("RRT Algorithm")
plt.imshow(grid[:][:1120], cmap='binary')
#plot the start and goal points
plt.plot(start[0],start[1],'ro') # plot the start pose with red color
plt.plot(goal[0],goal[1],'bo') # plot the goal pose with blue color
#plot the goal region circle
ax = fig.gca()
ax.add_patch(goalCircle)
#label the X and Y axes
ax.xaxis.set_ticks_position('top')
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

#tree Node class
class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX                #X Location
        self.locationY = locationY                #Y Location  
        self.children = []                        #children list   
        self.parent = None                        #parent node reference 
    """
    function add child of the tree node
    """
    def addChild(self, child_node):
        child_node.locationX
        child_node.locationY
        print("Adding X ", child_node.locationX)
        print("Adding Y ", child_node.locationY)
        self.children.append(child_node)

#function to sample a point within grid limits (note the [y,x])
def sampleAPoint():
    x = random.randint(1, grid.shape[0])
    y = random.randint(1, grid.shape[1])
    point = np.array([x, y])
    return point   
     
#set the root of the tree
root = treeNode(start[0],start[1])

#sample 3 points
point1 = sampleAPoint()
point1 = treeNode(point1[0],point1[1])
point2 = sampleAPoint()
point2 = treeNode(point2[0],point2[1])
# point3 = sampleAPoint()
# point3 = treeNode(point3[0],point3[1])

# Structure the tree
root.addChild(point1)
point1.addChild(point2)
# point2.addChild(point3)

#traverse the entire tree (recursion)
def traverseTree(root):
    if not root:
        return
    print(root.locationX, root.locationY)
    for child in root.children:
        traverseTree(child)

#function print point array
point_array=[]
def point_array_list(root):
    if not root:    
        return
    point_array.append([root.locationX, root.locationY])
    #print(point_array)
    for child in root.children:
        point_array_list(child)
    return point_array

print("array of points")
print(point_array_list(root))
point_list = point_array_list(root)
print("array list:", point_list[0][0])


#traverse tree
print('\nTree nodes point list')
point  = traverseTree(root)
print(point)

#plot on figure
for i in range(len(point_list)-1):
    print("Interation: ", i)
    plt.plot([point_list[i][0], point_list[i+1][0]], [point_list[i][1], point_list[i+1][1]],'go', linestyle="--")  
    
plt.show()
    

    