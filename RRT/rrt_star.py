import numpy as np
import matplotlib.pyplot as plt
import random

from matplotlib.pyplot import rcParams
class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.children =[]   # children list
        self.parrent = None # parent node reference

class RRT_Algorithms():
    def __init__(self, start, goal, numIter, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1])
        self.goal = treeNode(goal[0], goal[1])
        self.nearestNode = None
        self.interations = min(numIter, 250) # number of interation
        self.grid = grid
        self.rho = stepSize                  # length of each branch
        self.path_distance =0
        self.nearestDist = 10000             # distance to nearest node (initialize with large number)
        self.numWaypoints =0                 # number of waypoints
        self.Waypoints = []                   #the waypoints

    """
    Add the point to the nearest node and add goal when reached
    :param tree: int, tree to which to add vertex
    :param child: tuple, child vertex
    :param parent: tuple, parent vertex
    """
    def addChild(self, locationX, locationY):
        if (locationX == self.goal.locationX):
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            tempNode = treeNode(locationX, locationY)
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode

    def sampleAPoint(self):
        """
        Sample the random point within grid limits
        """
        x = random.randint(1, self.grid.shape[0])
        y = random.randint(1, self.grid.shape[1])
        point = np.array([x, y])
        return point   

    def steerToPoint(self, locationStart, locationEnd):
        """
        Steer the distance stepsize from the start to the end location
        """
        offset = self.rho*self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY +offset[1]])
        ## Check the point position, make sure the point is alway in the grid map
        if point[0] >= self.grid.shape[0]:
            point[0] = self.grid.shape[0] -1
        if point[1] >= self.grid.shape[1]:
            point[1] = self.grid.shape[1] -1
        return point
    
    def isInObstacle(self, locationStart, locationEnd):
        """
        check if the obstacle lies between the start node and the end node
        """
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(self.rho): #-> check each grid point of the line
            testPoint[0] = min(self.grid.shape[0]-1, locationStart.locationX + i*u_hat[0])
            testPoint[1] = min(self.grid.shape[1]-1, locationStart.locationY + i*u_hat[1])
            if self.grid[round(testPoint[0]),round(testPoint[1])] == 1:
                return True # Line in the obstacles
        return False    # Line ouside the obstacle
       

    def unitVector(self, locationStart, locationEnd):
        """
        find the unit vector between 2 points which form a vector
        """
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] -locationStart.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat
    
    def findNearest(self, root, point):
        """
        find the nearest node from a given unconnected point(Euclidean distance)
        """
        if not root:
            return
        """
        find the distance between the root and point use distance mehod,
        if it is lower than or equal to nearestDist then
        update nearestNode to root
        update nearestDist to the distance 
        """
        dist = self.distance(root, point)
        if dist <=self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for child in root.children:
            self.findNearest(child, point)

    def distance(self, node_new, point):
        """
        find the Euclidean distance between a node and XY point
        """
        dist = np.sqrt(np.power([node_new.locationX - point[0]], 2)+ np.power([node_new.locationY - point[1]], 2) )
        return dist

    def goalFound(self, point):
        """
        check if the goal have been reached within the stepsize
        """
        dist = self.distance(self.goal, point)
        if dist <= self.rho:
            return True
        else:
            return False

    def resetNearestValue(self):
        """
        reset the nearest node and nearest distance
        """
        self.nearestNode = None
        self.nearestDist = 10000 

    def retraceRRTPath(self, goal):
        """
        trace the path from the goal to start
        """
        if goal.locationX == self.randomTree.locationX:
            return
        # add 1 to numWaypoints
        self.numWaypoints +=1
        # extract the  X Y location of goal in a numpy array
        currentPoint = np.array([goal.locationX, goal.locationY])
        # install this array to waypoint (from beginning)
        self.Waypoints.insert(0, currentPoint)
        self.path_distance +=self.rho
        # add rho to path_distance
        self.retraceRRTPath(goal.parent)

##--------------------------
## Load grid
grid_load = np.load('./RRT/cspace.npy')
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


#start and goal positions as numpy arrays
start = np.array([100.0, 100.0])
goal = np.array([1600.0, 750.0])

# Begin define the RRT
RRT = RRT_Algorithms(start, goal, 150, grid, 120)  
# RRT Algorithm iteration
for i in range(RRT.interations):
    # Reset the nearest value
    RRT.resetNearestValue()
    print("Interation: ", i)
    # Start RTT Algorithms
    # sample a point 
    point = RRT.sampleAPoint()
    # find the nearest node
    RRT.findNearest(RRT.randomTree, point)
    # Steer to the point
    new = RRT.steerToPoint(RRT.nearestNode, point)
    # Check obstacle
    isObs = RRT.isInObstacle(RRT.nearestNode, new)
    if isObs == False:
        RRT.addChild(new[0], new[1])
        plt.pause(0.1)
        plt.plot([RRT.nearestNode.locationX, new[0]],[RRT.nearestNode.locationY, new[1]], 'go', linestyle="--")
        if (RRT.goalFound(new)):
            RRT.addChild(goal[0], goal[1])
            RRT.retraceRRTPath(RRT.goal)
            print("Goal found!")
            break

## Trace back the path from goal to start
RRT.Waypoints.insert(0,start)
print("Number of waypoints: ", RRT.numWaypoints)
print("Path Distance (m): ", RRT.path_distance)    
print("Waypoints: ", RRT.Waypoints)

#plot the waypoints in red (DONE)
for i in range(len(RRT.Waypoints)-1):
    plt.plot([RRT.Waypoints[i][0], RRT.Waypoints[i+1][0]], [RRT.Waypoints[i][1], RRT.Waypoints[i+1][1]],'ro', linestyle="--")
    plt.pause(0.10)
## Show the plot
plt.show()