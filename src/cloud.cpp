#include "cloud.h"
#include "load_off_file.h"

Cloud::Cloud():size(0) {}

Cloud::Cloud(const std::string &filename) {
    bool load_ok = load_OFF_file(filename, cloud);
    if (load_ok) {
        size = cloud.size();
    }
}

Cloud::~Cloud() {}

void Cloud::setPointCloud(const std::string &filename) {
    bool load_ok = load_OFF_file(filename, cloud);
    if (load_ok) {
        size = cloud.size();
    }
}

void Cloud::displayCloud(std::ostream& str) const {
    str.precision(std::numeric_limits<double>::digits10+1);
    for(std::size_t i=0 ; i<cloud.size() ; i++) {
        str << cloud[i].getX() << " " << cloud[i].getY() << " " << cloud[i].getZ() << std::endl;
    }
}

void Cloud::displayPlanes(std::ostream& str) const {
    str.precision(std::numeric_limits<double>::digits10+1);
    for (std::size_t i=0 ; i<planes.size() ; i++) {
        str << "center : " << planes[i].getCenter()
            << "normal : " << planes[i].getNormal()
            << "u : " << planes[i].getU()
            << "v : " << planes[i].getV()
            << std::endl;
    }
}

int Cloud::getSize() const {
    return size;
}

std::vector<Vector3d> &Cloud::getCloud() {
    return cloud;
}

std::vector<Vector3d> Cloud::getCloud() const {
    return cloud;
}

std::vector<Plane> &Cloud::getPlanes() {
    return planes;
}

std::vector<Plane> Cloud::getPlanes() const {
    return planes;
}

void Cloud::construct_tangent_planes(int K) {
    planes.resize(size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_float(new pcl::PointCloud<pcl::PointXYZ>);

    // initialise le pointcloud data
    cloud_float->width = size;
    cloud_float->height = 1;
    cloud_float->points.resize (cloud_float->width * cloud_float->height);

    for (int i=0 ; i<size ; ++i) {
        cloud_float->points[i].x = cloud[i].getX();
        cloud_float->points[i].y = cloud[i].getY();
        cloud_float->points[i].z = cloud[i].getZ();
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_float);

    for (int i=0 ; i<size ; i++) {
        pcl::PointXYZ searchPoint = cloud_float->points[i];
        // K nearest neighbor search
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        //   for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        //     std::cout << "    "  <<   cloud_float->points[ pointIdxNKNSearch[i] ].x
        //               << " " << cloud_float->points[ pointIdxNKNSearch[i] ].y
        //               << " " << cloud_float->points[ pointIdxNKNSearch[i] ].z
        //               << " (squared distance: " << pointNKNSquaredDistance[i]
        //               << ")" << std::endl;
        // }
        // on recupere la liste des voisins à partir de la liste des indices
        // attention : cette liste contient le point searchPoint!
        // Comme les listes sont  ordonnée par distance croissante, l'indice de
        // searchPoint est 0 (ie : premier élément des listes)
        int nb_nbhd = pointIdxNKNSearch.size()-1;
        std::vector<Vector3d> nbhd(nb_nbhd);
        // std::cout << "k-nbhd : " << std::endl;
        for (int i=0 ; i<nb_nbhd ; i++) {
            nbhd[i].set(cloud[pointIdxNKNSearch[i+1]].getX(),
                        cloud[pointIdxNKNSearch[i+1]].getY(),
                        cloud[pointIdxNKNSearch[i+1]].getZ());
            // std::cout << nbhd[i];
        }

        // plans tangents
        Vector3d centroid = compute_3d_centroid(nbhd);
        // std::cout << "assoiated point : " << cloud[i];
        // std::cout << "centroid : " << centroid;
        planes[i].setCenter(centroid);
        compute_normal(nbhd, centroid, planes[i]);
    }
}

Vector3d compute_3d_centroid(std::vector<Vector3d> nbhd) {
    int n = nbhd.size();
    Vector3d centroid;
    for (std::vector<Vector3d>::iterator it = nbhd.begin() ; it != nbhd.end() ; ++it) {
        centroid += *it;
    }
    centroid /= n;
    return centroid;
}

void compute_normal(std::vector<Vector3d> nbhd, Vector3d o, Plane &P) {
    // covariance matrix CV (positive semi-definite)
	Eigen::MatrixXd cv(3,3);
    cv.setZero();
    int n = nbhd.size();
    for (int k=0 ; k<n ; k++) {
        // the outer product
        for (int i=0 ; i<3 ; i++) {
            for (int j=i ; j<3 ; j++) {
                cv(i,j) += (nbhd[k][i]-o[i]) * (nbhd[k][j]-o[j]);
            }
        }
    }
    cv(1,0) = cv(0,1);
    cv(2,0) = cv(0,2);
    cv(2,1) = cv(1,2);

    // std::cout << "Here is a CV matrix:" << std::endl << cv << std::endl << std::endl;

    // compute eigenvalues of the covariance matrix CV
    Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(cv);
    // third eigen vector = the min
    int pos_min = 0;
    int pos_max = 0;
    double min = eigensolver.eigenvalues()[0].real();
    double max = eigensolver.eigenvalues()[0].real();
    for (int i = 1; i < 3; i++) {
        double eigenvalue = eigensolver.eigenvalues()[i].real();
        if (eigenvalue < min) {
            min = eigenvalue;
            pos_min = i;
        }
        if (eigenvalue >= max) {
            max = eigenvalue;
            pos_max = i;
        }
    }
    int pos = 3 - (pos_min + pos_max);

    // std::cout << "pos min : " << pos_min << std::endl;
    // std::cout << "pos max : " << pos_max << std::endl;
    // std::cout << "autre : " << pos << std::endl << std::endl;
    // std::cout << "The eigenvalues : " << std::endl << eigensolver.eigenvalues() << std::endl;

    // extract the third eigen vector
    Eigen::VectorXcd v1 = eigensolver.eigenvectors().col(pos_max);
    Eigen::VectorXcd v2 = eigensolver.eigenvectors().col(pos);
	Eigen::VectorXcd v3 = eigensolver.eigenvectors().col(pos_min);

    // std::cout << "The first eigenvector of the 3x3 matrix of A is:"
    //  << std::endl << v1 << std::endl;
    // std::cout << "The second eigenvector of the 3x3 matrix of A is:"
    //   << std::endl << v2 << std::endl;
    // std::cout << "The third eigenvector of the 3x3 matrix of A is:"
    //    << std::endl << v3 << std::endl;
    // std::cout << std::endl << "Pseudo-decomposition : " << std::endl;
    // Eigen::MatrixXd D = eigensolver.pseudoEigenvalueMatrix();
    // Eigen::MatrixXd V = eigensolver.pseudoEigenvectors();
    // Eigen::MatrixXd A_dec = V * D * V.inverse();
    // std::cout << "The pseudo-eigenvalue matrix D is:" << std::endl << D << std::endl;
    // std::cout << "The pseudo-eigenvector matrix V is:" << std::endl << V << std::endl;
    // std::cout << "Finally, V * D * V^(-1) = " << std::endl << V * D * V.inverse() << std::endl;

    // normal vector
    Vector3d normal(v3(0).real(),v3(1).real(),v3(2).real());
    Vector3d u(v1(0).real(),v1(1).real(),v1(2).real());
    Vector3d v(v2(0).real(),v2(1).real(),v2(2).real());

    // P.setCenter(o);
	P.setNormal(normal);
    P.setU(u);
    P.setV(v);

    // std::cout << "----------------------" << std::endl;
}
