#include "consistent_orientation.h"
#include "vector3d.h"
#include <fstream>

void orientation_algorithm(std::vector<Plane> &plans_t, const int size, const int K) {
    //======================================
    // CONSTRUCTION DU EMST
    // --> during and after the contruction of the Riemannian Graph because of
    // time and memory optimization
    UndirectedGraph euclidean_graph;

    //======================================
    // CONSTRUCTION DU RIEMANNIAN GRAPH
    int debut_rg = clock();
    // contruction du EMST
    UndirectedGraph g;

    // initialisation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_float(new pcl::PointCloud<pcl::PointXYZ>);

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

    int index_max_z = 0;
    double max_z = plans_t[0].getNormal().getZ();
    double z;

    for (int i=0 ; i<size ; i++) {
        // récupère l'indice du plan qui à la coordonée z la plus grande
        Vector3d ni = plans_t[i].getNormal();
        z = ni.getZ();
        if (fabs(z) > fabs(max_z)) {
            index_max_z = i;
            max_z = z;
        }

        // K nearest neighbor search
        pcl::PointXYZ searchPoint = cloud_float->points[i];
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

        // pour la construction de EMST, le 2e élément de la liste étant le
        // centroide le plus proche du centroide courant
        Vector3d n1 = plans_t[1].getNormal();
        double emst_cost = pointNKNSquaredDistance[1];
        boost::add_edge(i, pointIdxNKNSearch[1], emst_cost, euclidean_graph);

        // construct the Riemannian Graph
        int nb_nbhd = pointIdxNKNSearch.size();
        for (int k=1; k<nb_nbhd ;k++) {
            int index = pointIdxNKNSearch[k];
            Vector3d nk = plans_t[index].getNormal();
            double cost = 1-fabs(ni.getScalarProduct(nk));
            boost::add_edge(i, index, cost, g);
        }
    }

    // construction de l'EMST
    std::vector<Edge> emst;
    boost::kruskal_minimum_spanning_tree(euclidean_graph, std::back_inserter(emst));
    // on rajoute les arrêtes de l'EMST au Riemann Graph
    for (std::vector<Edge>::iterator ei = emst.begin(); ei != emst.end(); ++ei) {
        int ind_s = source(*ei, euclidean_graph);
        int ind_t = target(*ei, euclidean_graph);
        Vector3d ni = plans_t[ind_s].getNormal();
        Vector3d nk = plans_t[ind_t].getNormal();
        double w = 1-fabs(ni.getScalarProduct(nk));
        boost::add_edge(source(*ei, euclidean_graph),
            target(*ei, euclidean_graph), w, g);
    }
    double fin_rg = clock();

    //==============================================================
    // CONSTRUCTION DU MST
    double debut_mst = clock();
    std::vector<Edge> spanning_tree;
    boost::kruskal_minimum_spanning_tree(g, std::back_inserter(spanning_tree));
    double fin_mst = clock();

    //==============================================================
    // DEPTH FIRST SEARCH
    double debut_dfs = clock();
    propagate_orientation_dfs(g, spanning_tree, index_max_z, plans_t);
    double fin_dfs = clock();

    std::cout << "----- TEMPS DEXECUTION (CONSISTENT PLANE ORIENTATION) -----" << std::endl;
    std::cout << "Construction du EMST et du Riemannian Graph : " << (fin_rg-debut_rg) / double(CLOCKS_PER_SEC)
                << "s" << std::endl;
    std::cout << "Construction du MST : " << (fin_mst-debut_mst) / double(CLOCKS_PER_SEC)
                << "s" << std::endl;
    std::cout << "Algo DFS : " << (fin_dfs-debut_dfs) / double(CLOCKS_PER_SEC)
                << "s" << std::endl;
    std::cout << "-----TOTAL : " << (fin_dfs-debut_rg) / double(CLOCKS_PER_SEC)
                << "s -----" << std::endl << std::endl;
}

void flip_normal(Plane &P) {
    P.setNormal(-P.getNormal());
}

//==============================================================================
// DFS pour la propagation de l'orientation des normales

// cette classe permet de stocker l'ordre des indices des noeuds découvert en
// premier par l'algo DFS, notamment grâce à l'attribut m_dtimemap
template <typename TimeMap> class dfs_time_visitor : public boost::default_dfs_visitor {
typedef typename boost::property_traits<TimeMap>::value_type T;
public:
    TimeMap m_dtimemap;
    T & m_time;
    dfs_time_visitor(TimeMap dmap, T & t):m_dtimemap(dmap), m_time(t) {}

    void discover_vertex(Vertex v, const UndirectedGraph& g) const {
        put(m_dtimemap, v, m_time++);
        return;
    }
};

// DEPTH FIRST SEARCH
void propagate_orientation_dfs(UndirectedGraph &g, std::vector<Edge> &spanning_tree,
    int root, std::vector<Plane> &plans_t) {
    boost::property_map<UndirectedGraph, boost::edge_weight_t>::type
        EdgeWeightMap = get(boost::edge_weight_t(), g);
    // construction d'un objet graphe associé à l'arbre spanning_tree
    UndirectedGraph mstree;
    for (std::vector<Edge>::iterator ei = spanning_tree.begin();
            ei != spanning_tree.end(); ++ei) {
      boost::add_edge(source(*ei, g), target(*ei, g), EdgeWeightMap[*ei], mstree);
    }

    //============================================================
    // DFS + recupération de l'ordre de découverte des noeuds
    std::vector<int> dtime(num_vertices(mstree));
    typedef boost::iterator_property_map<std::vector<int>::iterator,
            boost::property_map<UndirectedGraph, boost::vertex_index_t>::const_type>
                    time_pm_type;
    time_pm_type dtime_pm(dtime.begin(), get(boost::vertex_index, mstree));
    int t = 0;
    dfs_time_visitor <time_pm_type> vis(dtime_pm, t);

    boost::depth_first_search(mstree,
        boost::visitor(vis).root_vertex(boost::vertex(root, mstree)));

    // use std::sort to order the vertices by their discover time
    int N = plans_t.size();
    std::vector<int> discover_order(N);
    boost::integer_range <int> r(0, N);
    std::copy(r.begin(), r.end(), discover_order.begin());
    std::sort(discover_order.begin(), discover_order.end(),
        boost::indirect_cmp < time_pm_type, std::less < double > >(dtime_pm));

    // use the discover_order list to explore the graph and flip the normal if
    // necessary
    for (int i = 0; i < N; ++i) {
        int node = discover_order[i];
        auto neighbors = boost::adjacent_vertices(node, mstree);
        for (auto vd: boost::make_iterator_range(neighbors)) {
            if (dot(plans_t[vd].getNormal(), plans_t[node].getNormal())<0) {
                flip_normal(plans_t[vd]);
            }
        }
    }
}
