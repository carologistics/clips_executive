# An abstract task (method decomposition node) in a hierarchical plan.
# Methods are not executable – they represent the decomposition reasoning.
# Useful for task-level monitoring, failure handling, and replanning triggers.

string pddl_instance
string goal_instance
string name
string[] args

# Stable decomposition-path identifier, e.g. "t1::achieve_goal"
string task_id
