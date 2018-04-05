#include "typedefs.hpp"
#include "2cs.hpp"
#include "graph6.hpp"
#include "boost_helper.hpp"
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>
#include <stdexcept>
#include <utility>

using namespace std;
using namespace boost;


//typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;
//typedef Graph::vertex_descriptor Vertex;

int main(int argc, char** argv) {
	if (argc<2) {
		std::cerr << "No input, exiting..." << std::endl;
		return EXIT_FAILURE;
	}
	char* filename = argv[1];
	ifstream file(filename);
	typedef read_graph6_edges<istream_iterator<unsigned char> > edge_reader;
	edge_reader end;
	Graph* g_ptr = new Graph();
	Graph& g = *g_ptr;
	string g6_string;
	size_t offset;
	while (getline(file,g6_string)) {
		istringstream pipe(g6_string+"\n");
		istream_iterator<unsigned char> ipipe(pipe);
		read_graph6<Graph>(ipipe,g);
		/*
		auto twocom = findTwoCommunityStructure(g,true);
		if (not twocom.first || not twocom.second){
			cout << g6_string << endl;
			//write_graph6(g, boost::edges(g).first, boost::edges(g).second, cout);
			//cout << "==========" << endl;
		}
		*/
		auto com = findMaximumCommunity(g,false);
		// only for cubic graphs
		size_t suspected_max_size = floor ((2*num_vertices(g)+1)/3);
		if (suspected_max_size != com->size()) {
			cout << g6_string << " | " << num_vertices(g) <<" vertices | ";
			cout << "community of size " << com->size() << endl;
		}
	}
	return EXIT_SUCCESS;
}
