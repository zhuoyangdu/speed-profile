#!/usr/bin/python
# -*- coding: UTF-8 -*-

from xml.dom.minidom import parse
import xml.dom.minidom
import os

if (__name__ == "__main__"):
    
    DOMTree = xml.dom.minidom.parse("../junction.net.xml")
    collection = DOMTree.documentElement
    
    edges = collection.getElementsByTagName("edge")
    edge_dict = dict()

    for edge in edges:
        print "****edge***"
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

    for edge in target_edges:
        string = edge_dict[edge]+'\n'
        path_file.write(string)
    
    path_file.close()

   
    route = []
    for edge in edges:
        if edge.hasAttribute("function"):
            route.append(edge.getAttribute("shape"))

    print route
