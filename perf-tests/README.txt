This directory contains a benchmark for kd-tree building and querying with 
both the original flann and nanoflann. 

The two test programs have handmade Makefiles and are intended for GCC under 
Linux only, assuming flann is installed in the system. 

To reproduce the benchmark: 

$ cd flann
$ make
$ ./test_flann >  ../stats_flann.txt
$ cd ../nanoflann
$ make
$ ./test_nanoflann > ../stats_nanoflann.txt

And execute the MATLAB/Octave script "analyze_stats.m" to generate the graphs. 
Note that raw stats data are also saved in a .tar.bz file for the case you don't 
want to reproduce the experiments but want to generate the graphs yourself.


Jose Luis Blanco
Aug 26, 2011


=== ADDED ON May, 2012 ===

More performance tests: 

$ nanoflann/test_leaf_max_size > LEAF_STATS.txt

And visualize the results with the MATLAB/Octave "analyze_leafsize_stats.m". 
Graphs online have been generated with these tools.

