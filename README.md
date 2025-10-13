Script and sample input for the mTSP with makespan optimization. 
The input has to be of the following format:
The first line of the input is an integer n
followed by an n x n weight matrix M. M_{i,j} is the 
weight of the edge from vertex i to vertex j. 
The last line of the input is a Python dictionary, it was 
used for pre-processing purposes and is not used in the script 
at the moment (input file still has to contain this line, otherwise you
get an error or you can remove line 12 in the script).

The output has the following format:
The first line is an integer that is the makespan value
followed by m lines of lists where each list is the travel 
route of one of the drones.
The m value is hard-coded in the script, you can change it to different
values to get different results.
