#ifndef DS_H
#define DS_H

#include <vector>
using std::vector;

class DisjointSets
{

public:
	//Creates n unconnected root nodes at the end of the vector.
	void addelements(int num);
	//This function should compress paths and works as described in lecture.
	int find(int elem);

	//This function should be implemented as union-by-size
	//a and b are roots of two sets per se
	void setunion(int a, int b);

	//this function is just to get the parent value of the idx
	//as long as it returns a value that is negative of the dimension
	//of the maze, we have a spanning tree
	int Size();
private:
	//a vector containing all the disjoint sets 
	vector<int> set;
	
};

#endif // DS_H