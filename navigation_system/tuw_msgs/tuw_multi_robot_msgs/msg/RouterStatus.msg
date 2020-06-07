#################################################################
## A message to publish the status of the last route 
## generation
#################################################################

int32 id # the unique id of the new plan
bool success # true if a routing table was found to the given scenario
string[] missing_robots # if the plan fails because of absent robots, these robots are listed here
int32 duration # the time until a routing table was found
int32 overall_path_length # the overall path length in the routing table (calculated using weights)
int32 longest_path_length # the longest path length in a routing table (calculated using weights)
int32 priority_scheduling_attemps # shows how often robot priorities are exchanged
int32 speed_scheduling_attemps # shows how many robots speed was reduced during planning