#include <iostream>
#include <fstream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/filesystem.hpp>
#include "../../src/cloud.h"
#include <ctime>

typedef boost::property<boost::edge_weight_t, double> EdgeWeight;
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeight> UndirectedGraph;
typedef boost::graph_traits<UndirectedGraph>::edge_iterator edge_iterator;
typedef boost::graph_traits <UndirectedGraph>::edge_descriptor Edge;
typedef boost::graph_traits<UndirectedGraph>::vertex_descriptor Vertex;

/**
  * \test test_dfs.cpp
  * \brief Programme qui teste que les étapes jusque "consistent plane orientation"
            fonctionnent bien, en particulier teste les méthode de la classe
            consistent_plane + librairie Boost.
            Lecture d'un fichier OFF, contruction des plans tangents, construction de
            l'arbre couvrant de poid minimum issu de l'algo de kruskal.
            L'algo récupère l'arbre sont la forme d'une liste d'objet de type
            Edge.
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

void propagate_orientation(UndirectedGraph &g, std::vector<Edge> &spanning_tree,
    int root, std::vector<Plane> &plans_t) {
    boost::property_map<UndirectedGraph, boost::edge_weight_t>::type
        EdgeWeightMap = get(boost::edge_weight_t(), g);
    UndirectedGraph mstree;
    for (std::vector<Edge>::iterator ei = spanning_tree.begin();
            ei != spanning_tree.end(); ++ei) {
      boost::add_edge(source(*ei, g), target(*ei, g), EdgeWeightMap[*ei], mstree);
    }

    std::vector<int> dtime(num_vertices(mstree));
    typedef boost::iterator_property_map<std::vector<int>::iterator,
            boost::property_map<UndirectedGraph, boost::vertex_index_t>::const_type>
                    time_pm_type;
    time_pm_type dtime_pm(dtime.begin(), get(boost::vertex_index, mstree));
    int t = 0;
    dfs_time_visitor <time_pm_type> vis(dtime_pm, t);
    // DFS ALGO
    boost::depth_first_search(mstree,
        boost::visitor(vis).root_vertex(boost::vertex(root, mstree)));

    // use std::sort to order the vertices by their discover time
    int N = plans_t.size();
    std::vector<int> discover_order(N);
    boost::integer_range <int> r(0, N);
    std::copy(r.begin(), r.end(), discover_order.begin());
    std::sort(discover_order.begin(), discover_order.end(),
        boost::indirect_cmp < time_pm_type, std::less < double > >(dtime_pm));

    std::cout << "order of discovery: " << std::endl;
    for (int i = 0; i < N; ++i) {
        int node = discover_order[i];
        std::cout << "neighbors of node " << node << ": " << std::endl;
        auto neighbors = boost::adjacent_vertices(node, mstree);
        for (auto vd: boost::make_iterator_range(neighbors)) {
            std::cout << vd << ", weight : ";
            std::cout << dot(plans_t[vd].getNormal(), plans_t[node].getNormal()) << std::endl;
            if (dot(plans_t[vd].getNormal(), plans_t[node].getNormal())<0) {
                flip_normal(plans_t[vd]);
                plans_t[vd].display(std::cout);
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

int main() {

    // Construction des plans tangents
    double debut_lecture = clock();
    std::string filename = "../../../models/cube.off";
    if (!(boost::filesystem::exists(filename))) {
        filename = "../models/cube.off";
    }
    Cloud cloud(filename);
    double fin_lecture = clock();

    int K = 4;
    double debut_pt = clock();
    cloud.construct_tangent_planes(K);
    double fin_pt = clock();

    // Construction de l'EMST
    double debut_emst = clock();

    //======================================
    // CONSTRUCTION DU EMST
    // --> during and after the contruction of the Riemannian Graph because of
    // time and memory optimization
    UndirectedGraph euclidean_graph;

    double size = cloud.getSize();
    std::vector<Plane> plans_t = cloud.getPlanes();

    std::cout << "Les plans tangents : " << std::endl;
    for (size_t i=0 ; i<plans_t.size();i++)
        plans_t[i].display(std::cout);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_float(new pcl::PointCloud<pcl::PointXYZ>);

    // initialise le pointcloud data
    cloud_float->width = size;
    cloud_float->height = 1;
    cloud_float->points.resize (cloud_float->width * cloud_float->height);

    for (int i=0 ; i<size ; ++i) {
        cloud_float->points[i].x = plans_t[i].getCenter().getX();
        cloud_float->points[i].y = plans_t[i].getCenter().getY();
        cloud_float->points[i].z = plans_t[i].getCenter().getZ();
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_float);

    UndirectedGraph g;

    int index_max_z = 0;
    double max_z = plans_t[0].getNormal().getZ();
    double z;

    for (int i=0 ; i<size ; i++) {
        Vector3d ni = plans_t[i].getNormal();
        z = ni.getZ();
        if (fabs(z) > fabs(max_z)) {
            index_max_z = i;
            max_z = z;
        }

        // std::cout << "The concerned point : " << cloud.getPlanes()[i].getCenter() << std::endl;
        pcl::PointXYZ searchPoint = cloud_float->points[i];
        // K nearest neighbor search
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

        // construct the Riemannian Graph et en même temps on construit le EMST
        int nb_nbhd = pointIdxNKNSearch.size();
        // pour la construction de EMST (le nombre de voisins utilisés)
        int nb_emst = 5;
        if (nb_emst > nb_nbhd) {
            nb_emst = nb_nbhd;
        }
        for (int k=1; k<nb_nbhd ;k++) {
            int index = pointIdxNKNSearch[k];
            Vector3d nk = plans_t[index].getNormal();
            double cost = 1-fabs(ni.getScalarProduct(nk));
            boost::add_edge(i, index, cost, g);
            if (k<nb_emst) {
                double weight_rg = pointNKNSquaredDistance[k];
                boost::add_edge(i, index, weight_rg, euclidean_graph);
            }
        }
    }
    // construction de l'EMST
    std::vector<Edge> emst;
    boost::property_map<UndirectedGraph, boost::edge_weight_t>::type
        EdgeWeightMap0 = get(boost::edge_weight_t(), g);
    boost::kruskal_minimum_spanning_tree(euclidean_graph, std::back_inserter(emst));
    // on rajoute les arrêtes de l'EMST au Riemann Graph
    for (std::vector<Edge>::iterator ei = emst.begin(); ei != emst.end(); ++ei) {
        int ind_s = source(*ei, euclidean_graph);
        int ind_t = target(*ei, euclidean_graph);
        Vector3d ni = plans_t[ind_s].getNormal();
        Vector3d nk = plans_t[ind_t].getNormal();
        double w = 1-fabs(ni.getScalarProduct(nk));
        boost::add_edge(source(*ei, euclidean_graph), target(*ei, euclidean_graph), w, g);
    }
    double fin_emst = clock();

    std::ofstream foutt("dfs_complex_riemanEMST.dot");
    foutt  << "graph A {\n"
          << " rankdir=LR\n"
          << " size=\"10\"\n"
          << " ratio=\"filled\"\n"
          << " edge[style=\"bold\"]\n" << " node[shape=\"circle\"]\n";
    boost::graph_traits<UndirectedGraph>::edge_iterator eitert, eiter_endt;
    for (boost::tie(eitert, eiter_endt) = edges(euclidean_graph); eitert != eiter_endt; ++eitert) {
      foutt << source(*eitert, euclidean_graph) << " -- " << target(*eitert, euclidean_graph);
      if (std::find(emst.begin(), emst.end(), *eitert) != emst.end())
        foutt << "[color=\"black\", label=\"" << EdgeWeightMap0[*eitert]
             << "\"];\n";
      else
        foutt << "[color=\"gray\", label=\"" << EdgeWeightMap0[*eitert]
             << "\"];\n";
    }
    foutt << "}\n";

    double debut_mst = clock();
    boost::property_map<UndirectedGraph, boost::edge_weight_t>::type
        EdgeWeightMap = get(boost::edge_weight_t(), g);
    std::vector<Edge> spanning_tree;
    boost::kruskal_minimum_spanning_tree(g, std::back_inserter(spanning_tree));
    double fin_mst = clock();

    std::ofstream fout("dfs_complex_mst.dot");
    fout  << "graph A {\n"
          << " rankdir=LR\n"
          << " size=\"10\"\n"
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

    std::cout << "max indice z = " << index_max_z
        << " avec z = " << max_z << std::endl;
    plans_t[index_max_z].display(std::cout);

    std::cout << "flip normal" << std::endl;
    flip_normal(plans_t[index_max_z]);
    plans_t[index_max_z].display(std::cout);

    //==============================================================
    // DEPTH FIRST SEARCH
    int debut_dfs = clock();
    propagate_orientation(g, spanning_tree, index_max_z, plans_t);
    int fin_dfs = clock();

    std::cout << "Voilà le résultat du flip" << std::endl;
    for (size_t i=0 ; i<plans_t.size() ; i++) {
        plans_t[i].display(std::cout);
    }

    std::cout << "resultats RG dans : " << "dfs_complex_riemanEMST.dot" << std::endl;
    std::cout << "resultats du MST dans : " << "dfs_complex_mst.dot" << std::endl;
    std::cout << std::endl;

    std::cout << "===== TEMPS DEXECUTION : =====" << std::endl;
    std::cout << "LECTURE DU .OFF : " << (fin_lecture-debut_lecture) / double(CLOCKS_PER_SEC)
                << "s" << std::endl;
    std::cout << "CONSTRUCTION DES PT : " << (fin_pt-debut_pt) / double(CLOCKS_PER_SEC)
                << "s" << std::endl;
    std::cout << "CONSTRUCTION DE LEMST : " << (fin_emst-debut_emst) / double(CLOCKS_PER_SEC)
                << "s" << std::endl;
    std::cout << "CONSTRUCTION MST : " << (fin_mst-debut_mst) / double(CLOCKS_PER_SEC)
                << "s" << std::endl;
    std::cout << "ALGO DFS : " << (fin_dfs-debut_dfs) / double(CLOCKS_PER_SEC)
                << "s" << std::endl;
    std::cout << "=====TOTAL CONSISTENT PLAN ORIENTATION STEP : " << (fin_dfs-debut_lecture) / double(CLOCKS_PER_SEC)
                << "s =====" << std::endl;

    return 0;
}
