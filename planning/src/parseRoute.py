import xml.etree.cElementTree as ET

target_id = "route02"
print "target route:", target_id

tree = ET.parse("../data/crossing.rou.xml")
target_edges = ""
for route_tree in tree.iter(tag = "route"):
   route_id = route_tree.attrib["id"]
   if route_id == target_id:
        target_edges = route_tree.attrib["edges"]

if target_edges == "":
    print "Route does not exist."
else:
    print "target_edges:", target_edges

tree = ET.parse("../data/crossing.edge.xml")
for edge in target_edges.split():
    print "current edge:", edge
    for edge_tree in tree.iter(tag = "edge"):



