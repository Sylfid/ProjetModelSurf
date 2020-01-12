#include "cloud.h"
#include "load_off_file.h"

Cloud::Cloud():size(0), rho_plus_delta(INFINITY) {}

Cloud::Cloud(const std::string &filename, const double d): rho_plus_delta(d) {
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
            << std::endl;
    }
}

int Cloud::getSize() const {
    return size;
}

double Cloud::getRhoPlusDelta() const {
    return rho_plus_delta;
}

double Cloud::getMinX() const {
    return minX;
}

double Cloud::getMinY() const {
    return minY;
}

double Cloud::getMinZ() const {
    return minZ;
}

double Cloud::getMaxX() const {
    return maxX;
}

double Cloud::getMaxY() const {
    return maxY;
}

double Cloud::getMaxZ() const {
    return maxZ;
}

void Cloud::setRhoPlusDelta(const double d) {
    rho_plus_delta = d;
}

std::vector<Vector3d> &Cloud::getCloud() {
    return cloud;
}

std::vector<Vector3d> Cloud::getCloud() const {
    return cloud;
}

Vector3d &Cloud::getCloudPrecise(const int i) {
    assert(i<size && i>=0);
    return cloud[i];
}

Vector3d Cloud::getCloudPrecise(const int i) const {
    assert(i<size && i>=0);
    return cloud[i];
}

std::vector<Plane> &Cloud::getPlanes() {
    return planes;
}

std::vector<Plane> Cloud::getPlanes() const {
    return planes;
}

Plane Cloud::getPlanePrecise(const int i) const {
    assert(i<size && i>=0);
    return planes[i];
}

Plane &Cloud::getPlanePrecise(const int i) {
    assert(i<size && i>=0);
    return planes[i];
}

void Cloud::construct_tangent_planes(const int K) {
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

        // à la recherche des coordonées min/max
        if (cloud[i].getX()<minX) {minX = cloud[i].getX() ;}
        if (cloud[i].getY()<minY) {minY = cloud[i].getY() ;}
        if (cloud[i].getZ()<minZ) {minZ = cloud[i].getZ() ;}

        if (cloud[i].getX()>maxX) {maxX = cloud[i].getX() ;}
        if (cloud[i].getY()>maxY) {maxY = cloud[i].getY() ;}
        if (cloud[i].getZ()>maxZ) {maxZ = cloud[i].getZ() ;}
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_float);

    for (int i=0 ; i<size ; i++) {
        pcl::PointXYZ searchPoint = cloud_float->points[i];
        // K nearest neighbor search
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

        // on recupere la liste des voisins à partir de la liste des indices
        // attention : cette liste contient le point searchPoint!
        // Comme les listes sont  ordonnée par distance croissante, l'indice de
        // searchPoint est 0 (ie : premier élément des listes)
        int nb_nbhd = pointIdxNKNSearch.size();
        std::vector<Vector3d> nbhd(nb_nbhd);
<<<<<<< HEAD
=======
        //printf("%f %f %f searchpoint\n",searchPoint.x,searchPoint.y,searchPoint.z);
>>>>>>> 8323ae03fdb23d7521a776e72a093ace6743371c
        for (int k=0 ; k<nb_nbhd ; k++) {
            nbhd[k].set(cloud[pointIdxNKNSearch[k]].getX(),
                        cloud[pointIdxNKNSearch[k]].getY(),
                        cloud[pointIdxNKNSearch[k]].getZ());
            //printf("%f %f %f \n",cloud[pointIdxNKNSearch[k]].getX(),cloud[pointIdxNKNSearch[k]].getY(),cloud[pointIdxNKNSearch[k]].getZ());
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
    /*for (std::vector<Vector3d>::iterator it = nbhd.begin() ; it != nbhd.end() ; ++it) {
        centroid += *it;
    }*/
    for(int i(0); i<n; i++){
        centroid += nbhd[i];
    }
    centroid /= n;
    //printf("centroid %f %f %f %d\n", centroid.getX(), centroid.getY(), centroid.getZ(), n);
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

    // compute eigenvalues of the covariance matrix CV
    Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(cv);
    // third eigen vector = the min
    int pos_min = 0;
    double min = eigensolver.eigenvalues()[0].real();
    for (int i = 1; i < 3; i++) {
        double eigenvalue = eigensolver.eigenvalues()[i].real();
        if (eigenvalue < min) {
            min = eigenvalue;
            pos_min = i;
        }
    }

    // extract the third eigen vector
	Eigen::VectorXcd v3 = eigensolver.eigenvectors().col(pos_min);

    // normal vector
    Vector3d normal(v3(0).real(),v3(1).real(),v3(2).real());

	P.setNormal(normal);
}
