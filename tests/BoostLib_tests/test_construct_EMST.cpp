#include <iostream>
#include <fstream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/filesystem.hpp>
#include "../../src/cloud.h"

typedef boost::property<boost::edge_weight_t, double> EdgeWeight;
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeight> UndirectedGraph;
typedef boost::graph_traits<UndirectedGraph>::edge_iterator edge_iterator;
typedef boost::graph_traits <UndirectedGraph>::edge_descriptor Edge;

/**
  * \test test_construct_EMST.cpp
  * \brief Programme qui teste la librairie Boost : construit un EMST pour un
            nuage de points.
            Pour afficher le .dot, dans le shell taper les commandes suivantes :
            dot -Tjpg -oexample2.jpg example2.dot
            (inspiré de : libs/graph/example/kruskal-example.cpp)
*/

void flip_normal(Plane &P) {
    P.setNormal(-P.getNormal());
}

int main() {

    std::string filename = "../../../models/tetrahedron.off";
    if (!(boost::filesystem::exists(filename))) {
        filename = "../models/tetrahedron.off";
    }
    Cloud cloud(filename);
    std::cout << "Nuage de points : " << std::endl;
    cloud.displayCloud(std::cout);
    std::cout << "Test de displayPlanes :" << std::endl;
    cloud.displayPlanes(std::cout);
    std::cout << std::endl;

    std::cout << "Construction des plans tangents :" << std::endl;
    int K = 4;
    cloud.construct_tangent_planes(K);
    std::cout << "Plans tangents avec k = " << K-1 << ": " <<std::endl;
    cloud.displayPlanes(std::cout);

    int size = cloud.getSize();
    std::vector<Plane> plans_t = cloud.getPlanes();

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
        // for (std::size_t j = 0; j < pointIdxNKNSearch.size (); ++j)
        //     std::cout << "    "  << cloud.getPlanes()[ pointIdxNKNSearch[j] ].getCenter()
        //               << " (squared distance: " << pointNKNSquaredDistance[j]
        //               << ")" << std::endl;

        // construct the Riemannian Graph
        int nb_nbhd = pointIdxNKNSearch.size();
        for (int k=1; k<nb_nbhd ;k++) {
            int index = pointIdxNKNSearch[k];
            Vector3d nk = plans_t[index].getNormal();
            double cost = 1-fabs(ni.getScalarProduct(nk));
            boost::add_edge(i, index, cost, g);
        }
    }

    boost::property_map<UndirectedGraph, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight_t(), g);

    std::vector<Edge> spanning_tree;
    boost::kruskal_minimum_spanning_tree(g, std::back_inserter(spanning_tree));

    // save the graph on a .dot file
    std::ofstream fout("emst.dot");
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

    std::cout << "max indice z = " << index_max_z
                << " avec z = " << max_z << std::endl;
    plans_t[index_max_z].display(std::cout);

    // flip the orientation of the normal with the largest z coordinate
    std::cout << "flip normal" << std::endl;
    flip_normal(plans_t[index_max_z]);
    plans_t[index_max_z].display(std::cout);

    std::cout << "resultats dans : " << "emst.dot" << std::endl;

    return 0;
}
