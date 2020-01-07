#ifndef CONSISTENT_ORIENTATION_H
#define CONSISTENT_ORIENTATION_H

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/depth_first_search.hpp>
#include "../src/cloud.h"

typedef boost::property<boost::edge_weight_t, double> EdgeWeight;
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeight> UndirectedGraph;
typedef boost::graph_traits<UndirectedGraph>::edge_iterator edge_iterator;
typedef boost::graph_traits <UndirectedGraph>::edge_descriptor Edge;
typedef boost::graph_traits<UndirectedGraph>::vertex_descriptor Vertex;

/**
  * \brief Algorithme qui oriente les plans tangents de façon "consistante"
  * \details
  * \param planes la liste des plans tangents
  * \param size la taille de la liste
  * \param K le nombre de k voisins à considérer pour la recherches des k plus
            proches voisins
*/
void orientation_algorithm(std::vector<Plane> &plans_t, const int size, const int K);

/**
  * \brief Change le sens de la normale du plan donnée en paramètre
  * \details Si on note n la normale, alors l'algorithme réalise : n devient -n
  * \param P le plan dont il faut changer le sens de la normale
*/
void flip_normal(Plane &P);

/**
  * \brief Propage l'orientation de la normale pour avoir une "orientation
            consistante"
  * \details Utilise l'algorithme DFS (depth first search) ou algorithme de
                parcours en profondeur pour parcourir un arbre non orienté.
  * \param spanning_tree l'arbre couvrant de poids minimum (MST)
  * \param plans_t la liste des plans tangents
*/
void propagate_orientation_dfs(UndirectedGraph &g,
    std::vector<Edge> &spanning_tree, int root, std::vector<Plane> &plans_t);

#endif // CONSISTENT_ORIENTATION_H
