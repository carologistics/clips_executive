string name
string[] args

int64 order

# for temporal plans
float64 start_time
float64 duration

# for partial order plans
int64[] predecessors
int64 action_id

# for hierarchical
string task_id
