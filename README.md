HKUST COMP 5421 Spring 2014 Computer Vision Project 3
outlier rejector using tensor voting
==========================================================================

Need to setup opencv environment first. 

Global variable usage
1. change the integer variable SIZE to size of data set. Size of data set needs to divide 2 if data type is line instead of point.

2. DISTANCE is nearest neighbor distance, only points smaller than this distance will be counted.

3. sigma is used when calculating c.

4. tolerance is for preventing dividing 0, if two points are too close to each other.


Things below need to be changed first.

Change path of iofstream

If it is line segments dataset, enable input x2, y2, z2.

Change the type of matrix K accordingly.
