## This RRT(Rapidly-exploring Random Tree) code implementation
import numpy as np
from random import random
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from collections import deque
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from timeit import default_timer as timer

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
    
def distance(pos1, pos2):
    return np.linalg.norm(np.array(pos1) - np.array(pos2))

def nearest(G, vex, radius):
    Nvex = None
    Nidx = None
    minDist = float("inf")

    for idx, v in enumerate(G.vertices):
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


def plot(G, path = None):
    px = [x for x, y, z in G.vertices]
    py = [y for y, y, z in G.vertices]
    pz = [z for x, y, z in G.vertices]
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.scatter(px, py, pz,  c='black', alpha=0.1) # add alpha=0 --> node transparent
    ax.scatter(G.startpos[0], G.startpos[1],G.startpos[2], c='red')
    ax.scatter(G.endpos[0], G.endpos[1], G.endpos[2], c='blue')
    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    lc = Line3DCollection(lines, colors='green', linewidth=0.7, linestyle="--")
    ax.add_collection(lc)

    if path is not None:
        paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
        lc2 = Line3DCollection(paths, colors='red', linewidths=3)
        ax.add_collection(lc2)

    ax.autoscale()
    ax.margins(0.1)
    plt.show()

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

def RRT(startpos, endpos, n_iter, radius, stepSize):
    start = timer()
    G = Graph(startpos, endpos)
    for _ in range(n_iter):
        randvex = G.randomPos()
        nearvex, nearidx = nearest(G, randvex, radius)
        if nearvex is None:
            continue

        newvex = steerVertex(randvex, nearvex, stepSize)

        newidx = G.add_vertex(newvex)
        dist = distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)

        #check if the point reach the goal
        dist = distance(newvex, G.endpos)
        if dist < 2 * radius:
            endidx = G.add_vertex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            G.success = True
            # print('Found Goadl!')
            # break
        #plot_animation(G)
    end = timer()
    print("Execution time RRT:", end - start, "seconds")
    return G

def RRT_star(startpos, endpos, n_iter, radius, goal_radius, stepSize):
    start = timer()
    G = Graph(startpos, endpos)

    for _ in range(n_iter):
        # 1. Sample a random Vertice
        randvex = G.randomPos()
        # 2. Find the near vertices
        nearvex, nearidx = nearest(G, randvex, radius)
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
    #Basis example Graph
    # G1 = Graph((0.0, 0.0, 0.0), (5.0, 5.0, 5.0))
    # p1 = (1.0, 1.0, 1.0)
    # G1.add_node(p1)
    # G1.add_edge(0, 1, 1)                                              
    # p2 =(2.0, 2.0, 2.0)
    # G1.add_node(p2)
    # G1.add_edge(1, 2, 1)    
    # print("list of node",G1.vertices)
    # print("list of index",G1.vex2idx)
    # print("edge",G1.edges)
    # plot(G1)
    # for idx, v in enumerate(G1.vertices):
    #     print("index", idx)
    #     print("vertices", v)
    # --------------------------
    #Basis example RRT 
    startpos = (0.7, -0.3, 0.3)
    endpos = (1.0, 0.4, 0.3)
    n_iter = 1000
    radius = 0.4 # radius of the new vertex for rewire step
    goal_radius = 0.05 # check the goal radius 
    stepSize = 0.06 #the default stepSize when steer


    G1 = RRT_star(startpos, endpos, n_iter, radius, goal_radius, stepSize)
    print(G1.success)
    if G1.success:
        path = dijkstra(G1)
        print(path)
        plot(G1, path)
    else:
        plot(G1)  

    # G2 = RRT(startpos, endpos, n_iter, goal_radius, stepSize)
    # plot(G2)
    # plt.show()
    # G = RRT(startpos, endpos, obstacles, n_iter, radius, stepSize)
    # plot(G)
    

