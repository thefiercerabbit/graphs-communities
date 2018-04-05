#ifndef TWO_COM_STRUCT_HPP
#define TWO_COM_STRUCT_HPP
#include "typedefs.hpp"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/connected_components.hpp>
#include <iostream>
#include <assert.h>
#include <math.h>

// All the necessary functions to make sure our graph has a 2-community structure, or not.
std::ostream& operator<<(std::ostream& out, const VertexSet C) {
	if (C.size() <= 0)
		return out;
	std::string s = std::to_string(*C.begin());
	for (auto v = C.begin(); ++v != C.end();) { // see the ++ BEFORE (as we did the first value)
		s+=" "+std::to_string(*v);
	}
	out << s;
	return out;
}			

int incrementBitVector(std::vector<bool>& bits, const bool default_value = false) {
	assert(!bits.empty());
	// The idea is to do a binary +1
	size_t k = 0;
	while (k < bits.size() && (bits[k] == not default_value)) {
		bits[k]=default_value;
		++k;
	}
	if (k == bits.size()) {
		return -1;
	}
	bits[k]=(not default_value);
	return k;			
}

/*
std::shared_ptr<VertexSet> getSubset(const std::vector<bool> bits, const Graph& G, bool default_value = true) {
	auto C = std::make_shared<VertexSet>();
	size_t k = 0;
	for (auto v = vertices(G); v.first != v.second; v.first++) {
		if (bits[k++] == default_value) {
			//std::cout << "addding to the com: " << *v.first << std::endl;
			C->insert(*v.first);
		}
	}
	return C;
}

std::shared_ptr<VertexSet> getAntiSubset(const std::vector<bool> bits, const Graph G) {
	return getSubset(bits,G,false);
}
*/

void getPartition(const std::vector<bool> bits, std::pair<VertexSet&,VertexSet&>& P, const Graph& G, bool default_value = true) {
	// if bits[v], then g[v] = 1, else g[v] =0
	size_t k = 0;
	for (auto v = vertices(G); v.first != v.second; v.first++) {
		if (bits[k++] == default_value) {
			//std::cout << "addding to the com: " << *v.first << std::endl;
			P.first.insert(*v.first);
		}
		else P.second.insert(*v.first);
	}
	//return std::make_unique<std::map<Vertex,int> >(P);
}

void updatePartition(const int last_changed, std::pair<VertexSet&,VertexSet&>& P, const Graph& G) {
	// if bits[v], then g[v] = 1, else g[v] =0
	VertexSet& A = P.first;
	VertexSet& B = P.second;
	auto index = boost::get(boost::vertex_index,G);
	Vertex u = index[last_changed];
	if (A.erase(u)) { // returns 1 iff u was in A
		B.insert(u);
	}
	else {
		B.erase(u);
		A.insert(u);
	}
}

void updateCommunity(const int last_changed, VertexSet& C, const Graph& G) {
	auto index = boost::get(boost::vertex_index,G);
	Vertex u = index[last_changed];
	if (not C.erase(u)) { // only if erase fails (because u is not in C)
		C.insert(u);
	}	
}

bool isSatisfied(const Vertex u, const VertexSet& C, const Graph& G, bool verbose = false) {
	size_t c_size = C.size();
	size_t g_size = boost::num_vertices(G);
	size_t dC = 0;
	size_t dG = boost::degree(u,G);
	for (auto neighbors = boost::adjacent_vertices(u,G); neighbors.first != neighbors.second; ++neighbors.first) {
		if (C.find(*neighbors.first) != C.end()) { // vertex in C
			++dC;
		}
	}
	if (verbose)
		std::cout << c_size << " " << g_size << " " << dC << " " << dG << std::endl;
 	return (g_size-1)*dC >= (c_size-1)*dG;
}

bool isCommunity(const VertexSet& C, const Graph& G) {
	for (auto v = C.begin(); v != C.end(); v++) {
		if (!isSatisfied(*v,C,G)) {
			return false;
		}
	}
	return true;
}

std::shared_ptr<VertexSet> findMaximumCommunity(const Graph& G, const bool connected = false) {
	Graph::vertex_iterator vi, vi_end;
	boost::tie(vi,vi_end) = boost::vertices(G);
	VertexSet C(vi,vi_end), max_C;
	typedef struct Predicate { // both edge and vertex
		bool operator()(Graph::edge_descriptor) const      { return true; } // all
		bool operator()(Graph::vertex_descriptor vd) const { return C_->find(vd) !=C_->end(); }
		const Graph* g;
		const VertexSet* C_;
	} filter;
	std::vector<bool> bits(boost::num_vertices(G),true); // start with all the vertices
	bool bits_are_full = false;
	size_t k=1;
	int last_changed;
	std::vector<int> components(num_vertices(G));

	while ((last_changed=incrementBitVector(bits,true))>-1) {
		updateCommunity(last_changed,C,G);
		if (isCommunity(C,G)) {
				if (connected) {
					filter predicate {&G, &C};
					using Filtered = boost::filtered_graph<Graph, Predicate, Predicate>;
					Filtered subG(G, predicate, predicate); // one predicate for edges, the other for vertices
					// check if connected
					size_t num_components = boost::connected_components(subG,&components[0]);
					if (num_components > 1)
						continue;
				}
				if (C.size() > max_C.size()) { // only reached if C is connected or does not have to be
					max_C.clear();
					std::copy(C.begin(),C.end(),std::inserter(max_C,max_C.begin())); // should work, right?
					//std::cout << max_C << " is a new max community :D" << std::endl;		
				}
		}
	}
	return std::make_unique<VertexSet>(max_C);
}
	
std::pair<std::shared_ptr<VertexSet>,std::shared_ptr<VertexSet>> findTwoCommunityStructure(const Graph& G, const bool connected = true) {
	//std::shared_ptr<VertexSet> B,C;
	VertexSet A;
	VertexSet B(vertices(G).first,vertices(G).second);
	std::pair<VertexSet&,VertexSet&> P(A,B); // as partition
	typedef struct Predicate { // both edge and vertex
		bool operator()(Graph::edge_descriptor) const      { return true; } // all
		bool operator()(Graph::vertex_descriptor vd) const { return C_->find(vd) !=C_->end(); }
		const Graph* g;
		const VertexSet* C_;
	} filter;
	std::vector<bool> bits(boost::num_vertices(G),false);
	bool bits_are_full = false;
	size_t k=1;
	size_t max_nb_to_check = ceil(pow(2,num_vertices(G)) / float(2));
	int last_changed;
	std::vector<int> components(num_vertices(G));

	while ((last_changed=incrementBitVector(bits))>-1) {
		//C = getSubset(bits,G);
		updatePartition(last_changed,P,G); // Give a community (0 or 1) to each vertex
		//std::cout << "C " << std::endl; for (auto e = C->begin(); e != C->end(); e++) std::cout << " " << *e;	std::cout << std::endl;
		if (isCommunity(A,G)) {
			if (isCommunity(B,G)) {
				if (connected) { // go to this check first to save some time
					// Get induced subgraph from A
					filter predicate {&G, &A};
					using Filtered = boost::filtered_graph<Graph, Predicate, Predicate>;
					Filtered subG(G, predicate, predicate); // one predicate for edges, the other for vertices
					// check if connected
					size_t num_components = boost::connected_components(subG,&components[0]);
					if (num_components == 1){
						
						// Get induced subgraph from B
						filter predicate {&G, &B};
						using Filtered = boost::filtered_graph<Graph, Predicate, Predicate>;
						Filtered subG(G, predicate, predicate); // one predicate for edges, the other for vertices
						//check if connected
						size_t num_components = boost::connected_components(subG,&components[0]);
						//std::cout << "nb v in subG: " << num_vertices(subG) << " B: " << B << " nb cc in SubG: " << num_components  << std::endl;
						if (num_components==1) {
							return std::make_pair(std::make_unique<VertexSet>(A),std::make_unique<VertexSet>(B));
						}
					}
				}
				else {
					return std::make_pair(std::make_unique<VertexSet>(A),std::make_unique<VertexSet>(B));
				}
			}
		}
		//A.clear();
		//B.clear();
		if (++k > max_nb_to_check)
			break;
		//std::cout << " ---------------- " << std::endl;
	}
	assert(last_changed!=-1);
	return std::make_pair(std::unique_ptr<VertexSet>(),std::unique_ptr<VertexSet>());
}

#endif
