#include <numeric>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <ANN/ANN.h>

const int MAX_ITERATION = 50;
const float EPSILON = 1e-8;
const float DISTANCE_THRESHOLD = 0.1f;

void readPoints(const std::string& path, std::vector<Eigen::Vector3f>& points) {
    std::ifstream fin(path);
    std::string s;
    while (fin >> s) {
        if (s == "v") {
            float x, y, z, weight;
            int cluster;
            fin >> x >> y >> z >> cluster >> weight;
            points.push_back(Eigen::Vector3f(x, y, z));
        }
        getline(fin, s);
    }
}

Eigen::Vector3f calculateCentroid(const std::vector<Eigen::Vector3f>& points) {
    return std::accumulate(points.begin(), points.end(), Eigen::Vector3f(0.0f, 0.0f, 0.0f)) / (float)points.size();
}

int main() {
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>());
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>());

    //readPointCloud("../data/generated1.dat", cloudIn);
    //readPointCloud("../data/generated2.dat", cloudOut);

    //pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //icp.setInputSource(cloudIn);
    //icp.setInputTarget(cloudOut);
    //
    //icp.setMaxCorrespondenceDistance(0.1);
    //icp.setMaximumIterations(50);
    //icp.setTransformationEpsilon(1e-8);
    //icp.setEuclideanFitnessEpsilon(1);

    //pcl::PointCloud<pcl::PointXYZ> final;
    //icp.align(final);

    //std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    //std::cout << icp.getFinalTransformation() << std::endl;

    //std::ofstream fout("../data/aligned.dat");
    //for (pcl::PointXYZ& point : final)
    //    fout << "v " << point.x << ' ' << point.y << ' ' << point.z << " 0 0" << std::endl;

    //for (pcl::PointXYZ& point : *cloudOut)
    //    fout << "v " << point.x << ' ' << point.y << ' ' << point.z << " 1 0" << std::endl;

    std::vector<Eigen::Vector3f> source, target;
    readPoints("../data/generated1.dat", source);
    readPoints("../data/generated2.dat", target);

    ANNpointArray pointArray;
    ANNkd_tree* tree;
    pointArray = annAllocPts(target.size(), 3);
    for (int i = 0; i < target.size(); i++)
        for (int j = 0; j < 3; j++)
            pointArray[i][j] = target[i](j);
    tree = new ANNkd_tree(pointArray, target.size(), 3);

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t = Eigen::Vector3f::Zero();
    for (int iter = 0; iter < MAX_ITERATION; iter++) {
        std::vector<Eigen::Vector3f> current;
        for (const Eigen::Vector3f& point : source)
            current.push_back(R * point + t);

        Eigen::Vector3f centroidCurrent = calculateCentroid(current);
        Eigen::Vector3f centroidTarget = calculateCentroid(target);

        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        float loss = 0.0f;
        int num = 0;
        ANNidxArray indices = new ANNidx[1];
        ANNdistArray distances = new ANNdist[1];
        for (const Eigen::Vector3f& point : current) {
            ANNpoint pointTemp = annAllocPt(3);
            for (int j = 0; j < 3; j++)
                pointTemp[j] = point(j);
            
            tree->annkSearch(pointTemp, 1, indices, distances);
            if (distances[0] < DISTANCE_THRESHOLD) {
                H += (point - centroidCurrent) * (target[indices[0]] - centroidTarget).transpose();
                loss += (point - target[indices[0]]).squaredNorm();
                num++;
            }

            annDeallocPt(pointTemp);
        }
        delete[] indices;
        delete[] distances;

        std::cout << loss / (float)num << std::endl;

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
        R = svd.matrixV() * svd.matrixU().transpose();
        t = centroidTarget - R * centroidCurrent;
    }

    annDeallocPts(pointArray);
    delete tree;

    std::cout << R << std::endl << t << std::endl;

    std::ofstream fout("../data/aligned.dat");
    for (const Eigen::Vector3f& pointTemp : source) {
        Eigen::Vector3f point = R * pointTemp + t;
        fout << "v " << point(0) << ' ' << point(1) << ' ' << point(2) << " 0 0" << std::endl;
    }

    for (const Eigen::Vector3f& point : source)
        fout << "v " << point(0) << ' ' << point(1) << ' ' << point(2) << " 1 0" << std::endl;

    for (const Eigen::Vector3f& point : target)
        fout << "v " << point(0) << ' ' << point(1) << ' ' << point(2) << " 2 0" << std::endl;

    return 0;
}