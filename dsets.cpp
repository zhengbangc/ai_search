#include <vector>
#include "dsets.h"

void DisjointSets::addelements(int num)
{
	for(int i=0; i < num; i++)
	set.push_back(-1);
}

int DisjointSets::find(int elem)
{
	if(set[elem] < 0)
		return elem;
	else return set[elem] = find(set[elem]); //assgin the root as the parent of
											 // the current node to compress the up tree
}

void DisjointSets::setunion(int a, int b)
{
	int roota = find(a);
	int rootb = find(b);

	int newSize = set[roota] + set[rootb];
	if(set[roota]>=set[rootb]) //height of a is smaller than height of b
	{
		set[rootb] = roota;
		set[roota] = newSize;
	}
	else
	{
		set[roota] = rootb;
		set[rootb] = newSize;
	}
}

int DisjointSets::Size()
	{
		int rootidx = find(1);
		return set[rootidx];
	}