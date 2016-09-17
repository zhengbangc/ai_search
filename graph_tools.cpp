/**
 * @file graph_tools.cpp
 * This is where you will implement several functions that operate on graphs.
 * Be sure to thoroughly read the comments above each function, as they give
 *  hints and instructions on how to solve the problems.
 */

#include "graph_tools.h"
using namespace std;
/**
 * Returns the shortest distance (in edges) between the Vertices
 *  start and end.
 * THIS FUNCTION IS GRADED.
 *
 * @param graph - the graph to search
 * @param start - the vertex to start the search from
 * @param end - the vertex to find a path to
 * @return the minimum number of edges between start and end
 *
 * @todo Label each edge "MINPATH" if it is part of the minimum path
 *
 * @note Remember this is the shortest path in terms of edges,
 *  not edge weights.
 * @note Again, you may use the STL stack and queue.
 * @note You may also use the STL's unordered_map, but it is possible
 *  to solve this problem without it.
 *
 * @hint In order to draw (and correctly count) the edges between two
 *  vertices, you'll have to remember each vertex's parent somehow.
 */
int GraphTools::findShortestPath(Graph & graph, Vertex start, Vertex end)
{

    std::vector<Vertex> Vertices = graph.getVertices();
    for(size_t i=0;i<Vertices.size();i++)
    {
        graph.setVertexLabel(Vertices[i] , "UNEXPLORED");
        std::vector<Vertex> Adjacents = graph.getAdjacent(Vertices[i]);
        for(size_t j=0;j<Adjacents.size();j++)
        {
            graph.setEdgeLabel(Vertices[i], Adjacents[j], "UNEXPLORED");    
        }
    }

 
            std::unordered_map<Vertex, Vertex> predecessor;
            std::unordered_map<Vertex, int> distance;

            predecessor.insert(std::pair<Vertex, Vertex>(start, start));
            distance.insert(std::pair<Vertex, int>(start, 0));

            queue<Vertex> BFSQueue;
            graph.setVertexLabel(start, "VISITED");
            BFSQueue.push(start);
            while(!BFSQueue.empty())
            {
                Vertex & curVertex = BFSQueue.front();
                BFSQueue.pop();
                std::vector<Vertex> Adjacents = graph.getAdjacent(curVertex);

                for(size_t j=0;j<Adjacents.size();j++)
                {
                    if(graph.getVertexLabel(Adjacents[j]) == "UNEXPLORED")
                    {
                        graph.setEdgeLabel(Adjacents[j],curVertex, "DISCOVERY");
                        graph.setVertexLabel(Adjacents[j], "VISITED");
                        BFSQueue.push(Adjacents[j]);
                        predecessor.insert(std::pair<Vertex, Vertex>(Adjacents[j], curVertex));
                        distance.insert(std::pair<Vertex, int>(Adjacents[j], distance[curVertex]+1));
                    }
                    else if(graph.getEdgeLabel(curVertex,Adjacents[j]) == "UNEXPLORED")
                    {
                        graph.setEdgeLabel(curVertex,Adjacents[j], "CROSS");
                    }
                }
            }

    Vertex curVertex = end;
    while(curVertex != start)
    {
        Vertex parent = predecessor[curVertex];
        graph.setEdgeLabel(curVertex, parent, "MINPATH");
        curVertex = parent;
    }

    return distance[end]; 
}

/**
 * Finds the minimum edge weight in the Graph graph.
 * THIS FUNCTION IS GRADED.
 *  
 * @param graph - the graph to search
 * @return the minimum weighted edge
 *
 * @todo Label the minimum edge as "MIN". It will appear blue when
 *  graph.savePNG() is called in minweight_test.
 *
 * @note You must do a traversal.
 * @note You may use the STL stack and queue.
 * @note You may assume the graph is connected.
 *
 * @hint Initially label vertices and edges as unvisited.
 */
int GraphTools::findMinWeight(Graph & graph)
{
    std::vector<Vertex> Vertices = graph.getVertices();
    for(size_t i=0;i<Vertices.size();i++)
    {
    	graph.setVertexLabel(Vertices[i] , "UNEXPLORED");
    	std::vector<Vertex> Adjacents = graph.getAdjacent(Vertices[i]);
    	for(size_t j=0;j<Adjacents.size();j++)
    	{
    		graph.setEdgeLabel(Vertices[i], Adjacents[j], "UNEXPLORED");	
    	}
    }


    	//initialize a weight
        std::vector<Vertex> v = graph.getAdjacent(Vertices[0]);
     	int EdgeWeight = graph.getEdgeWeight(Vertices[0], v[0]);
        Vertex minVertex1 = Vertices[0];
        Vertex minVertex2 = v[0]; 

    for(size_t i=0;i<Vertices.size();i++)
    {
    	if(graph.getVertexLabel(Vertices[i]) == "UNEXPLORED")
    	{
    	    queue<Vertex> BFSQueue;
    	    graph.setVertexLabel(Vertices[i], "VISITED");
    	    BFSQueue.push(Vertices[i]);

    	    while(!BFSQueue.empty())
    	    {
    	    	Vertex & curVertex = BFSQueue.front();
    	    	BFSQueue.pop();
    	    	std::vector<Vertex> Adjacents = graph.getAdjacent(curVertex);

    	    	for(size_t j=0;j<Adjacents.size();j++)
    	    	{
    	    		if(graph.getVertexLabel(Adjacents[j]) == "UNEXPLORED")
    	    		{
    	    			graph.setEdgeLabel(Adjacents[j],curVertex, "DISCOVERY");
    	    			graph.setVertexLabel(Adjacents[j], "VISITED");
    	    			BFSQueue.push(Adjacents[j]);
    	    			
                        if(EdgeWeight > graph.getEdgeWeight(Adjacents[j], curVertex))
    	    				{
                                EdgeWeight = graph.getEdgeWeight(Adjacents[j], curVertex);
                                minVertex1 = curVertex;
                                minVertex2 = Adjacents[j];
                            }
                    }
    	    		else if(graph.getEdgeLabel(curVertex,Adjacents[j]) == "UNEXPLORED")
    	    		{
    	    			graph.setEdgeLabel(curVertex,Adjacents[j], "CROSS");
    	    			if(EdgeWeight > graph.getEdgeWeight(Adjacents[j], curVertex))
                            {
                                EdgeWeight = graph.getEdgeWeight(Adjacents[j], curVertex);
                                minVertex1 = curVertex;
                                minVertex2 = Adjacents[j];
                            }
    	    		}
    	    	}
    	    }


    	}
    }

    graph.setEdgeLabel(minVertex1, minVertex2, "MIN");
    return EdgeWeight;
}

/**
 * Finds a minimal spanning tree on a graph.
 * THIS FUNCTION IS GRADED.
 *
 * @param graph - the graph to find the MST of
 *
 * @todo Label the edges of a minimal spanning tree as "MST"
 *  in the graph. They will appear blue when graph.savePNG() is called.
 *
 * @note Use your disjoint sets class from MP 7.1 to help you with
 *  Kruskal's algorithm. Copy the files into the libdsets folder.
 * @note You may call std::sort (http://www.cplusplus.com/reference/algorithm/sort/)
 *  instead of creating a priority queue.
 */
void GraphTools::findMST(Graph & graph)
{
    std::vector<Vertex> Vertices = graph.getVertices();
    for(size_t i=0;i<Vertices.size();i++)
    {
        graph.setVertexLabel(Vertices[i] , "UNEXPLORED");
        std::vector<Vertex> Adjacents = graph.getAdjacent(Vertices[i]);
        for(size_t j=0;j<Adjacents.size();j++)
        {
            graph.setEdgeLabel(Vertices[i], Adjacents[j], "UNEXPLORED");    
        }
    }


        //initialize a list of Edge
        std::vector<Edge> EdgeList;
        //std::vector<Vertex> Vertices = graph.getVertices();

    for(size_t i=0;i<Vertices.size();i++)
    {
        if(graph.getVertexLabel(Vertices[i]) == "UNEXPLORED")
        {
            queue<Vertex> BFSQueue;
            graph.setVertexLabel(Vertices[i], "VISITED");
            BFSQueue.push(Vertices[i]);

            while(!BFSQueue.empty())
            {
                Vertex & curVertex = BFSQueue.front();
                BFSQueue.pop();
                std::vector<Vertex> Adjacents = graph.getAdjacent(curVertex);

                for(size_t j=0;j<Adjacents.size();j++)
                {
                    if(graph.getVertexLabel(Adjacents[j]) == "UNEXPLORED")
                    {
                        graph.setEdgeLabel(Adjacents[j],curVertex, "DISCOVERY");
                        graph.setVertexLabel(Adjacents[j], "VISITED");
                        BFSQueue.push(Adjacents[j]);
                        EdgeList.push_back(graph.getEdge(Adjacents[j], curVertex)); //push all the edge into the list
                        
                    }
                    else if(graph.getEdgeLabel(curVertex,Adjacents[j]) == "UNEXPLORED")
                    {
                        graph.setEdgeLabel(curVertex,Adjacents[j], "CROSS");
                        EdgeList.push_back(graph.getEdge(Adjacents[j], curVertex));
                    }
                }
            }
        }
    }
        std::sort(EdgeList.begin(), EdgeList.end());
        DisjointSets vertexDset;
        vertexDset.addelements(graph.getVertices().size());
        for(size_t n=0;n<EdgeList.size();n++)
        {
            if(vertexDset.find(EdgeList[n].source) != vertexDset.find(EdgeList[n].dest))
            {
                vertexDset.setunion(EdgeList[n].source, EdgeList[n].dest);
                graph.setEdgeLabel(EdgeList[n].source, EdgeList[n].dest, "MST");
            }            
        } 
}
