#ifndef CONSISTENT_ORIENTATION_H
#define CONSISTENT_ORIENTATION_H

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include "../src/cloud.h"

typedef boost::property<boost::edge_weight_t, double> EdgeWeight;
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeight> UndirectedGraph;
typedef boost::graph_traits<UndirectedGraph>::edge_iterator edge_iterator;
typedef boost::graph_traits <UndirectedGraph>::edge_descriptor Edge;

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
  * \brief Change le sens de la normal du plan donnée en paramètre
  * \details Si on note n la normale, alors l'algorithme réalise : -n
  * \param P le plan dont il faut changer le sens de la normale
*/
void flip_normal(Plane &P);

/**
  * \brief
  * \details
  * \param spanning_tree
  * \param plans_t
  * \param from
*/
void propagate_orientation(std::vector<Edge> &spanning_tree,
    std::vector<Plane> &plans_t, int current, int parent);

#endif // CONSISTENT_ORIENTATION_H
