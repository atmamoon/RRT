'''
SAMPLING-BASED PLANNING

SK MOHAMMED MAMOON MONDAL

'''

import numpy as np
import random

#list to store address of the objects of tree_rrt class
nodes=[]

#Data structure to store nodes
class tree_rrt:
    tree_size=0
    def __init__(self,x,y):
        self.x=x
        self.y=y
        self.parent=None
        tree_rrt.tree_size+=1
        self.index=tree_rrt.tree_size
        print(self.index)

#finds the nearest node to the sample node
def nearest_node(sample):
    dist=999999
    index=0
    for i in nodes:
        d=check_distance([i.x,i.y],sample)
        if d<dist:
            dist=d
            index=i.index
    return [dist,index]

#checks for collision
def collision(local,obstacles):
    collide=False
    for item in local:
        for object in obstacles:
            if check_distance([item[0],item[1]],[object[0],object[1]])<(object[2])/2:
                collide=True
                return collide
    return collide

#returns the path at the end after the goal is reached
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

#returns euclidean distance between two points
def check_distance(p1,p2):
    return np.sqrt(np.square(p1[0]-p2[0])+np.square(p1[1]-p2[1]))


#function to implement the rrt sampling based path finding algorithm
def rrt(obstacles,start,goal,max_size,scale=1):

    edges=[]
    path=[]
    dmin=0.1*scale
    nodes.append(tree_rrt(start[0],start[1]))
    while tree_rrt.tree_size < max_size:
        x_sample=random.uniform(start[0]-0*scale,goal[0]+0.02*scale)
        y_sample=random.uniform(start[1]-0*scale,goal[1]+0.02*scale)

        sample=[x_sample,y_sample]
        edge=nearest_node(sample)
        if edge[0]>(dmin*2):
            continue

        local_x=[i for i in np.linspace(nodes[int(edge[1]-1)].x,sample[0],5)]#creates a list of straight line points between nearest node and sample
        local_y=[i for i in np.linspace(nodes[int(edge[1]-1)].y,sample[1],5)]#creates a list of straight line points between nearest node and sample
        local=zip(local_x,local_y)

        if collision(local,obstacles):
            continue
        nodes.append(tree_rrt(x_sample,y_sample))
        nodes[int(tree_rrt.tree_size-1)].parent=edge[1]
        edges.append([edge[1],nodes[int(tree_rrt.tree_size-1)].index,edge[0]])

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


    nodes1=[]
    for j in nodes:
        nodes1.append([j.index,j.x,j.y])
        print([j.x,j.y])
    return [path[::-1],nodes1,edges]


#calls the rrt function and creates obstacle list from csv file
obstacle_list=np.genfromtxt("obstacles.csv",delimiter=',')#reads csv file
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
    np.savetxt("nodes.csv",nodes1,delimiter=',')#creates csv file
    np.savetxt("edges.csv",edges,delimiter=',')#creates csv file
    np.savetxt("path.csv",path,delimiter=',')#creates csv file
