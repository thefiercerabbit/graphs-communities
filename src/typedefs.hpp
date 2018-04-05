#ifndef TYPEDEFS_FOR_GRAPHS_HPP
#define TYPEDEFS_FOR_GRAPHS_HPP
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/subgraph.hpp>

// Community property of a vertex (default should be zero)
//struct vertex_community {
//	int community = -1; // -1 as undefined
//};

//enum vertex_community_t { vertex_community };
//namespace boost {
//	BOOST_INSTALL_PROPERTY(vertex, community);
//}
//typedef boost::property<vertex_community_t,int> VertexCommunityProperty;


// Graph
typedef boost::adjacency_list<
	boost::vecS,
	boost::vecS,
	boost::undirectedS,
	boost::property<boost::vertex_index_t, size_t>
	// a graph6 name?
> Graph;

// Vertex
typedef Graph::vertex_descriptor Vertex;

// VertexSet
typedef std::unordered_set<Vertex> VertexSet;

// Subgraph
typedef boost::subgraph<Graph> Subgraph;

#endif
