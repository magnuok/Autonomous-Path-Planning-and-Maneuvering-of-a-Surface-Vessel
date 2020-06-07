Header header           
int32[] passed   # A list of already passed route segment id's.
int32 current    # The route segment id where the robot is currently located.
int32[] todo     # A list of route segment id's which will be visited by the robot. If this list is empty, the robot has finished its route.
float32 progress        # The route progress in %.
