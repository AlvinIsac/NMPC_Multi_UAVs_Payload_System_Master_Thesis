# message to pass the setpoints from the algorithm to the offboard node

uint64 timestamp		# time since system start (microseconds)

float32[4] setpoint

int32 setpoint_type		# 0: for pretakeoff and takeoff
				# 1: for move
				
bool out_of_mocap
