# foraging

At the start of the simulation every robot create a log file for himself,
and in it there will be data of every collision he do.
This data includes what was the UCB value if every behavior, and the highest is the choosen one.

The behavior type of the robot is being determined in Behavior Choose Method.
There are three different types of behavior choosing methods: random, default, 
or by UCB. To switch between them, uncomment the wanted method.