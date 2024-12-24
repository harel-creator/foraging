# foraging
This files should replace the files in /argos3-examples/controllers/footbot_foraging.

At the start of the simulation every robot create a log file for himself,
and in it there will be data of every collision he do.
This data includes what was the UCB value if every behavior, and the highest is the choosen one.

The behavior type of the robot is being determined in Behavior Choose Method.
There are three different types of behavior choosing methods: random, default, 
or by UCB. To switch between them, uncomment the wanted method.

The UCB score of each behavior is:
ucb_score = -avg_reward + exploration_factor * sqrt(log((m_collision_number)) / metrics.times_selected),
where avg_reward is the avg time of collision in this behavior. Its negetive because the goal is to reduce 
the duraion of collisions.
