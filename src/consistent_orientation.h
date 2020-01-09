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
            comme décrit dans la section 3.3 de l'article de Hoppe 92.
  * \param plans_t la liste des plans tangents
  * \param size la taille de la liste
  * \param K le nombre de voisins à considérer pour la recherches des k plus
            proches voisins
*/
void orientation_algorithm(std::vector<Plane> &plans_t, const int size, const int K);

/**
  * \brief Change le sens de la normale du plan donnée en paramètre
  * \details Si on note n la normale, alors n devient -n
  * \param P le plan dont il faut changer le sens de la normale
*/
void flip_normal(Plane &P);

/**
  * \brief Propage l'orientation des normales pour avoir une "orientation
            consistante"
  * \details Utilise l'algorithme DFS (depth first search ou en français
                algorithme de parcours en profondeur pour parcourir un arbre
                non orienté) pour trouver une façon de parcourir le MST à partir
                d'une racine qui correspond au plan dont la coordonnées z est la
                plus grand.
  * \param g le Riemmanian graph
  * \param spanning_tree l'arbre couvrant de poind minimum (MST) associé à g
  * \param root la racine de l'arbre
  * \param plans_t la liste des plans tangents
*/
void propagate_orientation_dfs(UndirectedGraph &g,
    std::vector<Edge> &spanning_tree, int root, std::vector<Plane> &plans_t);

#endif // CONSISTENT_ORIENTATION_H
