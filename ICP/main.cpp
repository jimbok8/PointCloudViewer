#include <string>
#include <fstream>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

void readPointCloud(const std::string& path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::ifstream fin(path);
    std::string s;
    while (fin >> s) {
        if (s == "v") {
            float x, y, z, weight;
            int cluster;
            fin >> x >> y >> z >> cluster >> weight;
            cloud->push_back(pcl::PointXYZ(x, y, z));
        }
        getline(fin, s);
    }
}

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>());

    readPointCloud("../data/generated1.dat", cloudIn);
    readPointCloud("../data/generated2.dat", cloudOut);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloudIn);
    icp.setInputTarget(cloudOut);
    
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1);

    pcl::PointCloud<pcl::PointXYZ> final;
    icp.align(final);

    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    std::ofstream fout("../data/aligned.dat");
    for (pcl::PointXYZ& point : final)
        fout << "v " << point.x << ' ' << point.y << ' ' << point.z << " 0 0" << std::endl;

    for (pcl::PointXYZ& point : *cloudOut)
        fout << "v " << point.x << ' ' << point.y << ' ' << point.z << " 1 0" << std::endl;

    return 0;
}