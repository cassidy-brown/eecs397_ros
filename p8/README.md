# p8
Adapting Part_5's baxter reachability code to test a UR10

The program tests reachability for a full plane of x and y values 
at 6 z-positions of interest, taken from the NIST magic number document

## Node
	ur10_reachability_from_above

## Usage
`rosrun p8 ur10_reachability_from_above`

Will output a file of comma-separated triples
This can be read into a program for analysis like a csv 

    
