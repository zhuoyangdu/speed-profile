#!/usr/bin/python
# -*- coding: UTF-8 -*-

from xml.dom.minidom import parse
import xml.dom.minidom
import os
import math

if (__name__ == "__main__"):
    
    DOMTree = xml.dom.minidom.parse("../junction.net.xml")
    collection = DOMTree.documentElement
    
    edges = collection.getElementsByTagName("edge")
    edge_dict = dict()

    for edge in edges:
        lane = edge.getElementsByTagName('lane')[0]
        edge_dict[edge.getAttribute("id")] = lane.getAttribute("shape")

    rouTree = xml.dom.minidom.parse("../junction.rou.xml")
    rou_collect = rouTree.documentElement
    rou_xml = rou_collect.getElementsByTagName("route")
    rou_dict = dict()

    for rou in rou_xml:
        rou_dict[rou.getAttribute("id")] = rou.getAttribute("edges")

    target_route = "route03"
    target_edges = rou_dict[target_route].split(" ")

    path_file = open("junction_path.txt", 'w')

    len_edges = len(target_edges)

    for i in range(0, len_edges):
        points = edge_dict[target_edges[i]].split(" ")
        points1 = points[0].split(",")
        points2 = points[1].split(",")
        x1 = float(points1[0])
        y1 = float(points1[1])
        x2 = float(points2[0])
        y2 = float(points2[1])      
        print "points1: ", x1, y1
        print "points2: ", x2, y2 
        # path_file.write(string)

        length = math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
        for s in range(0, int(length), 5):
            x = s/length * (x2-x1) + x1
            y = s/length * (y2-y1) + y1
            string = str(x) + "," + str(y) + "\n"
            print string
            path_file.write(string)

        string = str(x2) + "," + str(y2) + "\n"
        path_file.write(string)        
    
    path_file.close()

    path_file = open("junction_route.txt", 'w')
    route = []
    for edge in edges:
        if not edge.hasAttribute("function"):
            route.append(edge.getElementsByTagName("lane")[0].getAttribute("shape"))
            string = edge.getElementsByTagName("lane")[0].getAttribute("shape") + '\n'
            path_file.write(string)

    path_file.close()

    import matplotlib.pyplot as plt
    plt.figure("route")
    plt.clf()
    for sline in route:
        points = sline.split(" ")
        point1 = points[0].split(",")
        point2 = points[1].split(",")
        plt.plot([float(point1[0]), float(point2[0])], [float(point1[1]), float(point2[1])])
    
    plt.show()
