#################################################################
## This message is used to transport a graph between two nodes.
## Each vertex has a set of points and a width describing the
## shape of it and additionally successors and predecessors
## to describe the overall structure of the graph 
##
## Remark for the TUW_Multi_Robot_Router
## !! all vertices have to be sorted by id
## !! every id must be present...
#################################################################

Header header               # Header for Timestamp and Id
float32 resolution          # The resolution of the map (meter/pixel)
geometry_msgs/Pose origin   # The origin of the map (respectively: offset (the map is moved))
Vertex[] vertices           # The vertices in the graph (in pixels)
