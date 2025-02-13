# foraging
This is an extension of the argos 3 simulation, which uses the foraging example as a base.
There are 2 folders in this project: footbot_learning_foraging should go in the controllers folder (next footbot_foraging).The cmake file in the controller folder replace the one that in it by default (the only change there is "add_subdirectory(footbot_learning_foraging)"). Finaly, the file in the experiments folder, (.argos file) sould go with all the others .argos in the experiments folder.

After you do that you can build the project acording to the regular instructions, but run it with: 
    $ argos3 -c experiments/learning_foraging.argos

Some explaining about the code itself:
At the start of the simulation every robot create a log file for himself,
and in it there will be data of every collision he do.
This data includes what was the UCB value if every behavior, and the highest is the choosen one.

The behavior type of the robot is being determined in Behavior Choose Method.
There are three different types of behavior choosing methods: random, default, 
or by UCB. To switch between them, uncomment the wanted method.

The UCB score of each behavior is:
ucb_score = -avg_reward + exploration_factor * sqrt(log((m_collision_number)) / metrics.times_selected),
where avg_reward is the avg time of collision in this behavior. Its negetive because the goal is to reduce the duraion of collisions.

