#include <numeric>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <ANN/ANN.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

#include "CPoint.h"

const int MAX_ITERATION = 50;
const float EPSILON = 1e-8;
const float DISTANCE_THRESHOLD = 0.1f;

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

void readPoints(const std::string& path, std::vector<CPoint>& points) {
    std::ifstream fin(path);
    std::string s;
    while (fin >> s) {
        if (s == "v") {
            float x, y, z, weight;
            int cluster;
            fin >> x >> y >> z >> cluster >> weight;
            points.push_back(CPoint(Eigen::Vector3f(x, y, z)));
        }
        getline(fin, s);
    }
}

Eigen::Vector3f calculateCentroid(const std::vector<CPoint>& points) {
    Eigen::Vector3f ans(0.0f, 0.0f, 0.0f);
    for (const CPoint& point : points)
        ans += point.m_position;

    return ans / (float)points.size();
}

int main() {
    /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>());
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

    for (pcl::PointXYZ& point : *cloudIn)
        fout << "v " << point.x << ' ' << point.y << ' ' << point.z << " 1 0" << std::endl;

    for (pcl::PointXYZ& point : *cloudOut)
        fout << "v " << point.x << ' ' << point.y << ' ' << point.z << " 2 0" << std::endl;*/

    std::vector<CPoint> source, target;
    readPoints("../data/generated1.dat", source);
    readPoints("../data/generated2.dat", target);

    ANNpointArray pointArray;
    ANNkd_tree* tree;
    pointArray = annAllocPts(target.size(), 3);
    for (int i = 0; i < target.size(); i++)
        for (int j = 0; j < 3; j++)
            pointArray[i][j] = target[i].m_position(j);
    tree = new ANNkd_tree(pointArray, target.size(), 3);

    int k = 25;
    for (int i = 0; i < target.size(); i++) {
        ANNidxArray indices = new ANNidx[k];
        ANNdistArray distances = new ANNdist[k];
        tree->annkSearch(pointArray[i], k, indices, distances);

        Eigen::Vector3f avg(0.0f, 0.0f, 0.0f);
        for (int j = 0; j < k; j++)
            avg += target[indices[j]].m_position;
        avg /= (float)k;

        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (int j = 0; j < k; j++) {
            Eigen::Vector3f x = target[indices[j]].m_position - avg;
            cov += x * x.transpose();
        }
        cov /= (float)k;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver;
        solver.compute(cov);
        target[i].m_normal = solver.eigenvectors().col(0);
    }

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t = Eigen::Vector3f::Zero();
    for (int iter = 0; iter < MAX_ITERATION; iter++) {
        std::vector<CPoint> current;
        for (const CPoint& point : source)
            current.push_back(CPoint(R * point.m_position + t));

        Eigen::Vector3f centroidCurrent = calculateCentroid(current);
        Eigen::Vector3f centroidTarget = calculateCentroid(target);

        /*Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        float loss = 0.0f;
        int num = 0;*/

        Eigen::MatrixXf A(6, 6);
        Eigen::VectorXf b(6);
        A.setZero();
        b.setZero();

        ANNidxArray indices = new ANNidx[1];
        ANNdistArray distances = new ANNdist[1];
        for (const CPoint& point : current) {
            ANNpoint pointTemp = annAllocPt(3);
            for (int j = 0; j < 3; j++)
                pointTemp[j] = point.m_position(j);
            
            tree->annkSearch(pointTemp, 1, indices, distances);
            if (distances[0] < DISTANCE_THRESHOLD) {
                /*H += (point.m_position - centroidCurrent) * (target[indices[0]].m_position - centroidTarget).transpose();
                loss += (point.m_position - target[indices[0]].m_position).squaredNorm();
                num++;*/

                Eigen::Vector3f p = point.m_position;
                Eigen::Vector3f q = target[indices[0]].m_position;
                Eigen::Vector3f n = target[indices[0]].m_normal;
                Eigen::Vector3f c = p.cross(n);

                Eigen::MatrixXf At(6, 6);
                At.block(0, 0, 3, 3) = c * c.transpose();
                At.block(0, 3, 3, 3) = c * n.transpose();
                At.block(3, 0, 3, 3) = c * n.transpose();
                At.block(3, 3, 3, 3) = n * n.transpose();

                Eigen::VectorXf bt(6);
                bt.block(0, 0, 3, 1) = c;
                bt.block(3, 0, 3, 1) = n;
                bt *= (p - q).dot(n);

                A += At;
                b -= bt;
            }

            annDeallocPt(pointTemp);
        }
        delete[] indices;
        delete[] distances;

        //std::cout << loss / (float)num << std::endl;

        //Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
        //R = svd.matrixV() * svd.matrixU().transpose();
        //t = centroidTarget - R * centroidCurrent;

        Eigen::LLT<Eigen::MatrixXf> cholesky;
        cholesky.compute(A);
        Eigen::VectorXf x = cholesky.solve(b);

        Eigen::AngleAxisf rotationVectorX(x(0), Eigen::Vector3f(1, 0, 0));
        Eigen::Matrix3f rotationMatrixX = rotationVectorX.toRotationMatrix();

        Eigen::AngleAxisf rotationVectorY(x(1), Eigen::Vector3f(0, 1, 0));
        Eigen::Matrix3f rotationMatrixY = rotationVectorY.toRotationMatrix();

        Eigen::AngleAxisf rotationVectorZ(x(2), Eigen::Vector3f(0, 0, 1));
        Eigen::Matrix3f rotationMatrixZ = rotationVectorZ.toRotationMatrix();

        R *= rotationMatrixX * rotationMatrixY * rotationMatrixZ;
        t += x.block(3, 0, 3, 1);
    }

    annDeallocPts(pointArray);
    delete tree;

    std::cout << R << std::endl << t << std::endl;

    std::ofstream fout("../data/aligned.dat");
    for (const CPoint& pointTemp : source) {
        Eigen::Vector3f point = R * pointTemp.m_position + t;
        fout << "v " << point(0) << ' ' << point(1) << ' ' << point(2) << " 0 0" << std::endl;
    }

    for (const CPoint& pointTemp : source) {
        Eigen::Vector3f point = pointTemp.m_position;
        fout << "v " << point(0) << ' ' << point(1) << ' ' << point(2) << " 1 0" << std::endl;
    }

    for (const CPoint& pointTemp : target) {
        Eigen::Vector3f point = pointTemp.m_position;
        fout << "v " << point(0) << ' ' << point(1) << ' ' << point(2) << " 2 0" << std::endl;
    }

    return 0;
}