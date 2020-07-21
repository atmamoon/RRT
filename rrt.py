import numpy as np
import random

import matplotlib.pyplot as plt
import math


nodes=[]
class tree_rrt:
    tree_size=0
    def __init__(self,x,y):
        self.x=x
        self.y=y
        self.parent=None
        tree_rrt.tree_size+=1
        self.index=tree_rrt.tree_size
        print(self.index)
'''
def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd
'''
def nearest_node(sample):
    dist=999999
    index=0
    for i in nodes:
        d=check_distance([i.x,i.y],sample)
        if d<dist:
            dist=d
            index=i.index
    return [dist,index]

def collision(local,obstacles):
    collide=False
    for item in local:
        for object in obstacles:
            if check_distance([item[0],item[1]],[object[0],object[1]])<(object[2])/2:
                collide=True
                return collide
    return collide


def retrive_path():
    path=[]
    i=tree_rrt.tree_size

    while True:
        path.append(i)
        i=nodes[int(i-1)].parent
        print(i)
        if i==1:
            break
    path.append(i)
    return path

def check_distance(p1,p2):
    return np.sqrt(np.square(p1[0]-p2[0])+np.square(p1[1]-p2[1]))

def rrt(obstacles,start,goal,max_size,scale=1):
    #nodes=[]
    animation=False
    edges=[]
    path=[]
    dmin=0.05*scale
    nodes.append(tree_rrt(start[0],start[1]))
    while tree_rrt.tree_size < max_size:
        x_sample=random.uniform(start[0]-0*scale,goal[0]+0.02*scale)
        y_sample=random.uniform(start[1]-0*scale,goal[1]+0.02*scale)
        #x_sample=random.uniform(nodes[int(tree_rrt.tree_size-1)].x,goal[0]+0.1*scale)
        #y_sample=random.uniform(nodes[int(tree_rrt.tree_size-1)].y,goal[1]+0.1*scale)
        sample=[x_sample,y_sample]
        edge=nearest_node(sample)
        if edge[0]>(dmin*2):
            continue
        #step=(nodes[int(edge[1]-1)].x-sample[0])/100)
        local_x=[i for i in np.linspace(nodes[int(edge[1]-1)].x,sample[0],5)]
        local_y=[i for i in np.linspace(nodes[int(edge[1]-1)].y,sample[1],5)]
        local=zip(local_x,local_y)
        #print('local',local)
        if collision(local,obstacles):
            continue
        nodes.append(tree_rrt(x_sample,y_sample))
        nodes[int(tree_rrt.tree_size-1)].parent=edge[1]
        edges.append([edge[1],nodes[int(tree_rrt.tree_size-1)].index,edge[0]])
        if animation and tree_rrt.tree_size % 5 == 0:
            draw_graph(edge,obstacles)
        if check_distance(goal,sample)<dmin:
            print("found something")
            local_x=[i for i in np.linspace(goal[0],sample[0],5)]
            local_y=[i for i in np.linspace(goal[1],sample[1],5)]
            local=zip(local_x,local_y)
            if collision(local,obstacles):
                continue
            nodes.append(tree_rrt(goal[0],goal[1]))
            nodes[int(tree_rrt.tree_size-1)].parent=nodes[int(tree_rrt.tree_size-2)].index
            edges.append([nodes[int(tree_rrt.tree_size-2)].index,tree_rrt.tree_size-1,check_distance(goal,sample)])
            path=retrive_path()
            break
        if animation and tree_rrt.tree_size % 5 == 0:
            draw_graph(edge,obstacles)

    nodes1=[]
    #print(edges)
    for j in nodes:
        nodes1.append([j.index,j.x,j.y])
        print([j.x,j.y])
    return [path[::-1],nodes1,edges]



def plot_circle(x, y, size, color="-b"):  # pragma: no cover
    size=size/2
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
    yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
    plt.plot(xl, yl, color)

def draw_graph(rnd,obstacle_list):
    plt.clf()
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
                                 lambda event: [exit(0) if event.key == 'escape' else None])
    if rnd is not None:
        plt.plot(rnd[0], rnd[1], "^k")
    for node in nodes:
        if node.parent:
            plt.plot(node.x, node.y, "-g")

    for (ox, oy, size) in obstacle_list:
        plot_circle(ox, oy, size)

    plt.plot(start_node[0],start_node[1], "xr")
    plt.plot(goal_node[0],goal_node[1], "xr")
    plt.axis("equal")
    plt.axis([-0.5,0.6,-0.5,0.6])
    plt.grid(True)
    plt.pause(0.01)


obstacle_list=np.genfromtxt("obstacles.csv",delimiter=',')
scale=1
start_node=[-0.5*scale,-0.5*scale]
goal_node=[0.5*scale,0.5*scale]
avenue=rrt(obstacle_list,start_node,goal_node,1000,scale)
if len(avenue[0])==0:
    print("failure")
else:
    print("path",avenue[0])
    nodes1=np.array(avenue[1],float)
    edges=np.array(avenue[2],float)
    path=np.array(avenue[0],int)
    path=np.reshape(path,(1,path.size))
    np.savetxt("nodes.csv",nodes1,delimiter=',')
    np.savetxt("edges.csv",edges,delimiter=',')
    np.savetxt("path.csv",path,delimiter=',')
