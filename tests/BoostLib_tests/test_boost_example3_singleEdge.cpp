#include <iostream>
#include <fstream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>

typedef boost::property<boost::edge_weight_t, double> EdgeWeight;
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeight> UndirectedGraph;
typedef boost::graph_traits<UndirectedGraph>::edge_iterator edge_iterator;
typedef boost::graph_traits <UndirectedGraph>::edge_descriptor Edge;

/**
  * \test test_boost_example3_singleEdge.cpp
  * \brief Programme qui teste la librairie Boost : création d'un graphe avec
            arrête simple seulement (arrêtes parallèles non autorisés),
            algorithme de Kruskal pour la recherche d'un arbre couvrant minimal
            et sauvegarde du résultat dans un fichier .dot + opérations sur les
            edges et vertex pour ajouter ou non un egde entre deux vertexes.
            operation : boost::adjacency_list<boost::setS, boost::vecS, ... >
            Pour afficher le .dot, dans le shell taper les commandes suivantes :
            dot -Tjpg -oexample2.jpg example2.dot
*/
int main() {
    UndirectedGraph g;

    // // cela ne marche pas car les sommets 0 et 1 n'existent pas
    // if (!boost::edge(0, 1, g).second) boost::add_edge(0, 0, 0.1, g);
    boost::add_edge(0, 1, 5, g);
    if (!boost::edge(0, 0, g).second) {
        boost::add_edge(0, 0, 0.1, g);
    }
    // arrête multiple
    boost::add_edge(0, 1, 5, g);
    boost::add_edge(4, 5, 7, g);
    boost::add_edge(0, 4, 2, g);
    if (!boost::edge(0, 4, g).second) {
        boost::add_edge(0, 4, 0.123456, g);
    }
    boost::add_edge(1, 5, 2, g);
    boost::add_edge(0, 2, 1, g);
    boost::add_edge(2, 3, 6, g);
    boost::add_edge(1, 3, 4, g);
    boost::add_edge(2, 4, 2, g);
    boost::add_edge(3, 5, 1, g);
    boost::add_edge(5, 3, 0.0003, g);
    // pour ajouter une arrête entre deux sommets qui sont présents dans le graphe
    // ici : ajoute l'arrête
    if (boost::vertex(3, g) == g.null_vertex()) {
        std::cout << "source vertex (" << 4 << ") not present" << std::endl;
    } else if (boost::vertex(0, g) == g.null_vertex()) {
        std::cout << "target vertex (" << 5 << ") not present" << std::endl;
    } else {
        boost::add_edge(3, 0, 0.7, g);
    }

    boost::property_map<UndirectedGraph, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight_t(), g);

    std::vector < Edge > spanning_tree;
    boost::kruskal_minimum_spanning_tree(g, std::back_inserter(spanning_tree));

    std::ofstream fout("example3_simpleEdge.dot");
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

    std::cout << "resultats dans : " << "example3_simpleEdge.dot" << std::endl;

    return 0;
}
