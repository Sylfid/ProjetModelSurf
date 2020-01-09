#include <iostream>
#include <fstream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/undirected_dfs.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/filesystem.hpp>
#include "../../src/cloud.h"

typedef boost::property<boost::edge_weight_t, double> EdgeWeight;
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeight> UndirectedGraph;
typedef boost::graph_traits<UndirectedGraph>::edge_iterator edge_iterator;
typedef boost::graph_traits <UndirectedGraph>::edge_descriptor Edge;
typedef boost::graph_traits<UndirectedGraph>::vertex_descriptor Vertex;

/**
  * \test test_dfs.cpp
  * \brief Programme qui teste la librairie Boost : undirected DFS.
            Récupère l'arbre couvrant de poid minimum issu de l'algo de kruskal.
            L'algo récupère l'arbre sont la forme d'une liste d'objet de type
            Edge. On récupère la sortie du DFS appliqué au MST grâce à une liste
            des noeuds qui est rangé de telle sorte qu'elle converse l'ordre de
            parcours (possible grace à la classe dfs_time_visitor).
*/

void flip_normal(Plane &P) {
    P.setNormal(-P.getNormal());
}


template <typename TimeMap> class dfs_time_visitor : public boost::default_dfs_visitor {
typedef typename boost::property_traits < TimeMap >::value_type T;
public:
    TimeMap m_dtimemap;
    T & m_time;
    dfs_time_visitor(TimeMap dmap, T & t):m_dtimemap(dmap), m_time(t) {}

    void discover_vertex(Vertex v, const UndirectedGraph& g) const {
    std::cout << v << std::endl;
    put(m_dtimemap, v, m_time++);
    return;
    }
};

int main() {

    UndirectedGraph g;
    boost::add_edge(0, 1, 5, g);
    boost::add_edge(4, 5, 7, g);
    boost::add_edge(0, 4, 2, g);
    boost::add_edge(1, 5, 2, g);
    boost::add_edge(0, 2, 1, g);
    boost::add_edge(2, 3, 6, g);
    boost::add_edge(1, 3, 4, g);
    boost::add_edge(2, 4, 2, g);
    boost::add_edge(3, 5, 1, g);

    boost::property_map<UndirectedGraph, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight_t(), g);

    std::vector<Edge> spanning_tree;
    boost::kruskal_minimum_spanning_tree(g, std::back_inserter(spanning_tree));

    // save the graph on a .dot file
    std::ofstream fout("mst_final.dot");
    fout  << "graph A {\n"
          << " rankdir=LR\n"
          << " size=\"3,3\"\n"
          << " ratio=\"filled\"\n"
          << " edge[style=\"bold\"]\n" << " node[shape=\"circle\"]\n";
    boost::graph_traits<UndirectedGraph>::edge_iterator eiter, eiter_end;
    for (boost::tie(eiter, eiter_end) = edges(g); eiter != eiter_end; ++eiter) {
      fout << source(*eiter, g) << " -- " << target(*eiter, g);
      if (std::find(spanning_tree.begin(), spanning_tree.end(), *eiter)
          != spanning_tree.end())
        fout << "[color=\"black\", label=\"" << EdgeWeightMap[*eiter]
             << "\"];\n";
      else
        fout << "[color=\"gray\", label=\"" << EdgeWeightMap[*eiter]
             << "\"];\n";
    }
    fout << "}\n";

    std::cout << "graphe stocké dans : " << "mst_final.dot" << std::endl;

    //==============================================================
    // DEPTH FIRST SEARCH
    // graphe non-orienté
    UndirectedGraph mstree;
    for (std::vector<Edge>::iterator ei = spanning_tree.begin();
            ei != spanning_tree.end(); ++ei) {
      boost::add_edge(source(*ei, g), target(*ei, g), EdgeWeightMap[*ei], mstree);
    }

    std::ofstream foutt("mstree_graph.dot");
    foutt  << "graph A {\n"
          << " rankdir=LR\n"
          << " size=\"3,3\"\n"
          << " ratio=\"filled\"\n"
          << " edge[style=\"bold\"]\n" << " node[shape=\"circle\"]\n";
    boost::graph_traits<UndirectedGraph>::edge_iterator eitert, eiter_endt;
    for (boost::tie(eitert, eiter_endt) = edges(mstree); eitert != eiter_endt; ++eitert) {
      foutt << source(*eitert, mstree) << " -- " << target(*eitert, mstree);
        foutt << "[color=\"black\", label=\"" << EdgeWeightMap[*eitert]
             << "\"];\n";
    }
    foutt << "}\n";

    std::cout << "graphe stocké dans : " << "mstree_graph.dot" << std::endl;

    std::vector<int> dtime(num_vertices(mstree));
    typedef boost::iterator_property_map<std::vector<int>::iterator,
            boost::property_map<UndirectedGraph, boost::vertex_index_t>::const_type>
                    time_pm_type;
    time_pm_type dtime_pm(dtime.begin(), get(boost::vertex_index, mstree));
    int t = 0;
    dfs_time_visitor <time_pm_type> vis(dtime_pm, t);
    // DFS
    boost::depth_first_search(mstree,
        boost::visitor(vis).root_vertex(boost::vertex(1, mstree)));

    // use std::sort to order the vertices by their discover time
    std::vector<int> discover_order(6);
    boost::integer_range <int> r(0, 6);
    std::copy(r.begin(), r.end(), discover_order.begin());
    std::sort(discover_order.begin(), discover_order.end(),
        boost::indirect_cmp < time_pm_type, std::less < double > >(dtime_pm));
    std::cout << "order of discovery: ";

    int i;
    for (i = 0; i < 6; ++i)
      std::cout << discover_order[i] << " ";
    std::cout << std::endl;

    return 0;
}
