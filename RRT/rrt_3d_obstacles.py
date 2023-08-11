## This RRT(Rapidly-exploring Random Tree) code implementation
import numpy as np
from random import random
#using these packages for visualization
import matplotlib.pyplot as plt
from collections import deque
from mpl_toolkits.mplot3d.art3d import Line3DCollection
#using this package for calculate excutable time
from timeit import default_timer as timer


class Line():
    def __init__(self, p0, p1):
            self.p0 = np.array(p0)
            self.p1 = np.array(p1)
            self.dirn = np.array(p1) - np.array(p0)
            self.dist = np.linalg.norm(self.dirn)
            self.dirn /= self.dist # normalize
    def path(self, t):
            return self.p0 + t * self.dirn
    
class Sphere(object):
    #Cube define by it center
    def __init__(self, x=0, y=0, z=0, radius =1):
        self.x = x
        self.y = y
        self.z = z
        self.center = [x, y, z]
        self.radius = radius
    def isPointinside(self, p):
        dist = p.distance(self.center)
        if(dist <= self.radius):
            return True
        return False 

def distance(x, y):
    return np.linalg.norm(np.array(x) - np.array(y))

#https://scikit-spatial.readthedocs.io/en/stable/gallery/intersection/plot_sphere_line.html
# http://paulbourke.net/geometry/circlesphere/index.html#linesphere

def isInObstacle(point, spheres):
    for obs in spheres:
        dist = distance(point, obs.center)
        if dist < obs.radius:
            return True
    return False

def Intersection(line, sphere):
     """
     http://paulbourke.net/geometry/circlesphere/index.html#linesphere
     """
     a = np.dot(line.dirn, line.dirn)
     b = 2 * np.dot(line.dirn, line.p0 - sphere.center)
     c = np.dot(line.p0 - sphere.center, line.p0 - sphere.center) - sphere.radius * sphere.radius
     discriminant = b * b - 4 * a * c
     if discriminant < 0:
          return False # the line does not intersect the sphere.
     return True       # the line intersect the sphere.

def isThruObstacle(line, obstacles):
    for obs in obstacles:
        if Intersection(line, obs):
            return True # the line intersect the sphere.
    return False # the line does not intersect the sphere.

class Graph:
    """
    Class initialize Graph
    """
    def __init__(self, startpos, endpos):
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {startpos:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]
        self.sz = endpos[2] - startpos[2]

    def add_vertex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))


    def randomPos(self):
        rx = random()
        ry = random()
        rz = random()
        posx = self.startpos[0] - (self.sx / 2.) + rx * self.sx * 2
        posy = self.startpos[1] - (self.sy / 2.) + ry * self.sy * 2
        posz = self.startpos[2] - (self.sz / 2.) + rz * self.sz * 2
        return posx, posy, posz
    


def nearest(G, vex, obstacles):
    Nvex = None
    Nidx = None
    minDist = float("inf")

    for idx, v in enumerate(G.vertices):
        line = Line(v, vex)
        if isThruObstacle(line, obstacles):
            continue
        
        dist = distance(v, vex)
        if dist < minDist:
            minDist = dist
            Nidx = idx
            Nvex = v
    return Nvex, Nidx


def steerVertex(randvex, nearvex, stepSize):
    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min (stepSize, length)
    newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1], nearvex[2]+dirn[2])
    return newvex


##--- Plot function
def drawSphere(xCenter, yCenter, zCenter, r):
    #draw sphere
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x=np.cos(u)*np.sin(v)
    y=np.sin(u)*np.sin(v)
    z=np.cos(v)
    # shift and scale sphere
    x = r*x + xCenter
    y = r*y + yCenter
    z = r*z + zCenter
    return (x,y,z)



# This function for animation

def dijkstra(G):
    srcIdx = G.vex2idx[G.startpos]
    dstIdx = G.vex2idx[G.endpos]

    # build dijkstra
    nodes = list(G.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[srcIdx] = 0

    while nodes:
        curNode = min(nodes, key=lambda node: dist[node])
        nodes.remove(curNode)
        if dist[curNode] == float('inf'):
            break

        for neighbor, cost in G.neighbors[curNode]:
            newCost = dist[curNode] + cost
            if newCost < dist[neighbor]:
                dist[neighbor] = newCost
                prev[neighbor] = curNode

    # retrieve path
    path = deque()
    curNode = dstIdx
    while prev[curNode] is not None:
        path.appendleft(G.vertices[curNode])
        curNode = prev[curNode]
    path.appendleft(G.vertices[curNode])
    return list(path)

def plot(G, Obstacles, path = None):
    px = [x for x, y, z in G.vertices]
    py = [y for y, y, z in G.vertices]
    pz = [z for x, y, z in G.vertices]
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.scatter(px, py, pz,  c='black', alpha=0.1) # add alpha=0 --> node transparent
    ax.scatter(G.startpos[0], G.startpos[1],G.startpos[2], c='red') #start point
    ax.scatter(G.endpos[0], G.endpos[1], G.endpos[2], c='blue') #end point
   
    #Line RRT
    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    lc = Line3DCollection(lines, colors='green', linewidth=0.7, linestyle="--")
    ax.add_collection(lc)

     # Obstacles plot
    for obs in Obstacles:
        (xs,ys,zs) = drawSphere(obs.x,obs.y,obs.z,obs.radius)
        ax.plot_wireframe(xs, ys, zs, color="b")

    # Shortest path
    if path is not None:
        paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
        lc2 = Line3DCollection(paths, colors='red', linewidths=3)
        ax.add_collection(lc2)

    #ax.autoscale()
    ax.margins(0.1)
    plt.show()

def RRT(startpos, endpos, n_iter, radius, stepSize, goal_radius, obstacles):
    start = timer()
    G = Graph(startpos, endpos)
    for _ in range(n_iter):
        randvex = G.randomPos()

        if isInObstacle(randvex, obstacles): #check vertex inside the Sphere Obstacles
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles)
        if nearvex is None:
            continue

        newvex = steerVertex(randvex, nearvex, stepSize)

        newidx = G.add_vertex(newvex)
        dist = distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)

        #check if the point reach the goal
        dist = distance(newvex, G.endpos)
        if dist < 2 * goal_radius:
            endidx = G.add_vertex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            G.success = True
            # print('Found Goadl!')
            # break
        #plot_animation(G)
    end = timer()
    print("Execution time RRT:", end - start, "seconds")
    return G

def RRT_star(startpos, endpos, n_iter, radius, goal_radius, stepSize, obstacles):
    start = timer()
    G = Graph(startpos, endpos)

    for _ in range(n_iter):
        # 1. Sample a random Vertice
        randvex = G.randomPos()
        if isInObstacle(randvex, obstacles): #check vertex inside the Sphere Obstacles
            continue
        # 2. Find the near vertices
        nearvex, nearidx = nearest(G, randvex, obstacles)
        if nearvex is None:
            continue
        
        # 3. Steer steerVertex
        newvex = steerVertex(randvex, nearvex, stepSize)
        
         #4. Add vertex and edge with nearest vertex
        newidx = G.add_vertex(newvex)
        dist = distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)
        G.distances[newidx] = G.distances[nearidx] + dist

        # 5. Rewire step: update nearby vertices distance (if shorter)
        for vex in G.vertices:
            if vex == newvex:
                continue

            dist = distance(vex, newvex)
            line = Line(vex, newvex)

            if isThruObstacle(line, obstacles):
                continue

            if dist > radius:
                continue
            
            # When dist < radius implement this code
            idx = G.vex2idx[vex]
            if G.distances[newidx] + dist < G.distances[idx]:
                G.add_edge(idx, newidx, dist)
                G.distances[idx] = G.distances[newidx] + dist


        # check the goal
        dist = distance(newvex, G.endpos)  
        if dist < 2 * goal_radius:
            endidx = G.add_vertex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            try:
                G.distances[endidx] = min(G.distances[endidx], G.distances[newidx]+dist)
            except:
                G.distances[endidx] = G.distances[newidx]+dist

            G.success = True
            print('success')
            break
        #plot_animation(G)
    end = timer()
    print("Execution time RRT*:", end - start, "seconds")
    return G
        #plot_animation(G)
    
if __name__ == '__main__':
     #Basis example RRT 
    startpos = (0.8, -0.6, 0.30)
    endpos = (0.5, 0.6, 0.60)
    n_iter = 1000
    radius = 0.2 # radius of the new vertex for rewire step
    goal_radius = 0.1 # check the goal radius 
    stepSize = 0.1 #the default stepSize when steer
    ## Create Obstacles
    S1 = Sphere(0.5, 0.3, 0.4, 0.1)
    S2 = Sphere(0.8, -0.2, 0.4, 0.1)
    S3 = Sphere(0.8, 0.6, 0.6, 0.1)
    Obstacles=[S1, S2, S3]
    # print(isInObstacle([1, 1, 1], Obstacles, 1.0))
    G1 = RRT_star(startpos, endpos, n_iter, radius, goal_radius, stepSize, Obstacles)
    print(G1.success)
    if G1.success:
        path = dijkstra(G1)
        print(path)
        plot(G1,Obstacles, path)
    else:
        plot(G1, Obstacles)  
    

