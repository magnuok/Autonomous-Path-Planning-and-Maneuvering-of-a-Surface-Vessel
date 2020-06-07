#################################################################
## Route Preconditions are used to sync robots on a route
## e.g.: Each robot publishes its current step of its route 
## with such a message
## The specific segments of a route are marked with such 
## preconditions to block a robot from entering a segment
## until the sync message of the other robot has the right 
## route_segment_nr
#################################################################

string robot_id                  # the robot name for the precondition 
int32 current_route_segment      # the segment nr of the route executed by the given robot