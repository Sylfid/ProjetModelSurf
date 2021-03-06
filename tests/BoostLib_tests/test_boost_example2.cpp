#include <iostream>
#include <fstream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>

typedef boost::property<boost::edge_weight_t, double> EdgeWeight;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeight> UndirectedGraph;
typedef boost::graph_traits<UndirectedGraph>::edge_iterator edge_iterator;
typedef boost::graph_traits <UndirectedGraph>::edge_descriptor Edge;

/**
  * \test test_boost_example2.cpp
  * \brief Programme simple qui teste la librairie Boost : création d'un graphe,
            ajouter des sommets et arrêtes dans le graphe, algorithme de Kruskal
            pour la recherche d'un arbre couvrant minimal et sauvegarde du
            résultat dans un fichier .dot pour la visualisation du graphe.
            Pour afficher le .dot, dans le shell taper les commandes suivantes :
            dot -Tjpg -oexample2.jpg example2.dot
*/
int main() {
    UndirectedGraph g;

    boost::add_edge(0, 2, 1, g);
    boost::add_edge(1, 3, 1, g);
    boost::add_edge(1, 4, 1, g);
    boost::add_edge(2, 1, 7, g);
    boost::add_edge(2, 3, 3, g);
    boost::add_edge(3, 4, 1, g);
    boost::add_edge(4, 0, 1, g);
    boost::add_edge(4, 1, 2, g);

    boost::property_map<UndirectedGraph, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight_t(), g);

    std::vector < Edge > spanning_tree;
    boost::kruskal_minimum_spanning_tree(g, std::back_inserter(spanning_tree));

    std::ofstream fout("example2.dot");
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

    return 0;
}
