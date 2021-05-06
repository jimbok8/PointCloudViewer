#include <algorithm>
#include <string>
#include <vector>
#include <set>
#include <iostream>

#include <ANN/ANN.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/edge_aware_upsample_point_set.h>
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Scale_space_reconstruction_3/Weighted_PCA_smoother.h>
#include <CGAL/Scale_space_reconstruction_3/Advancing_front_mesher.h>

#include "CPointSet.h"
#include "CMesh.h"
#include "CSimplifyParameter.h"
#include "CResampleParameter.h"
#include "CSmoothParameter.h"
#include "CReconstructParameter.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef std::pair<Point, Vector> PointNormal;
typedef CGAL::Surface_mesh<Point> SurfaceMesh;
typedef SurfaceMesh::Vertex_index VertexIndex;
typedef SurfaceMesh::Face_index FaceIndex;
typedef SurfaceMesh::Halfedge_index HalfedgeIndex;

const unsigned int WINDOW_WIDTH = 1920;
const unsigned int WINDOW_HEIGHT = 1080;

static std::vector<PointNormal> points;
static std::vector<CPoint> cpoints;
static std::vector<unsigned int> indices, cindices;
static CSimplifyParameter simplifyParameter(0.3f);
static CResampleParameter resampleParameter(25.0f, 0.0f, 3.0f, 3000);
static CSmoothParameter smoothParameter(64, 30.0f);
static CReconstructParameter reconstructParameter(4, 1.0f);

static bool comparePointSet(const float epsilon) {
    if (points.size() != cpoints.size()) {
        std::cout << "size: " << points.size() << ' ' << cpoints.size() << std::endl;
        return false;
    }

    ANNpointArray pointArray = annAllocPts(points.size(), 3);
    for (int i = 0; i < points.size(); i++)
        for (int j = 0; j < 3; j++)
            pointArray[i][j] = points[i].first[j];
    ANNkd_tree *tree = new ANNkd_tree(pointArray, points.size(), 3);

    float avgPosition = 0.0f, avgNormal = 0.0f;
    ANNidxArray indices = new ANNidx[1];
    ANNdistArray distances = new ANNdist[1];
    for (int i = 0; i < cpoints.size(); i++) {
        ANNpoint point = annAllocPt(3);
        for (int j = 0; j < 3; j++)
            point[j] = cpoints[i].m_position[j];

        tree->annkSearch(point, 1, indices, distances);
        Eigen::Vector3f position(points[indices[0]].first.x(), points[indices[0]].first.y(), points[indices[0]].first.z());
        Eigen::Vector3f normal(points[indices[0]].second.x(), points[indices[0]].second.y(), points[indices[0]].second.z());
        avgPosition += (cpoints[i].m_position - position).squaredNorm();
        avgNormal += std::min((cpoints[i].m_normal - normal).squaredNorm(), (cpoints[i].m_normal + normal).squaredNorm());

        annDeallocPt(point);
    }
    avgPosition /= points.size();
    avgNormal /= points.size();

    annDeallocPts(pointArray);
    delete tree;
    delete[] indices;
    delete[] distances;

    std::cout << avgPosition << ' ' << avgNormal << std::endl;
    return avgPosition < epsilon&& avgNormal < epsilon;
}

static bool compareMesh() {
    std::set<std::tuple<int, int, int>> triangles;
    for (int i = 0; i < indices.size(); i += 3) {
        std::vector<int> points;
        points.push_back(indices[i]);
        points.push_back(indices[i + 1]);
        points.push_back(indices[i + 2]);
        std::sort(points.begin(), points.end());
        triangles.insert(std::make_tuple(points[0], points[1], points[2]));
    }

    int num = 0;
    for (int i = 0; i < cindices.size(); i++) {
        std::vector<int> points;
        points.push_back(cindices[i]);
        points.push_back(cindices[i + 1]);
        points.push_back(cindices[i + 2]);
        std::sort(points.begin(), points.end());

        if (triangles.find(std::make_tuple(points[0], points[1], points[2])) != triangles.end())
            num++;
    }

    float rate = (float)num / ((float)std::max(indices.size(), cindices.size()) / 3.0f);
    std::cout << rate << std::endl;
    return rate > 0.6f;
}

static void refresh() {
    std::vector<CPoint>().swap(cpoints);
    for (const PointNormal& point : points) {
        Eigen::Vector3f position(point.first.x(), point.first.y(), point.first.z());
        Eigen::Vector3f normal(point.second.x(), point.second.y(), point.second.z());
        cpoints.push_back(CPoint(position, normal));
    }
}

static void calculateNormals() {
    CGAL::pca_estimate_normals<CGAL::Sequential_tag>(points, 50,
        CGAL::parameters::
        point_map(CGAL::First_of_pair_property_map<PointNormal>()).
        normal_map(CGAL::Second_of_pair_property_map<PointNormal>()));
    CGAL::mst_orient_normals(points, 50,
        CGAL::parameters::
        point_map(CGAL::First_of_pair_property_map<PointNormal>()).
        normal_map(CGAL::Second_of_pair_property_map<PointNormal>()));
}

static void simplify(const CSimplifyParameter& parameter) {
    float epsilon = parameter.m_epsilon;

    points.erase(CGAL::grid_simplify_point_set(points, epsilon,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointNormal>()).
        normal_map(CGAL::Second_of_pair_property_map<PointNormal>())), points.end());
    std::vector<PointNormal>(points).swap(points);
    calculateNormals();
}

static void resample(const CResampleParameter& parameter) {
    float sharpnessAngle = parameter.m_sharpnessAngle;
    float edgeSensitivity = parameter.m_edgeSensitivity;
    float neighborRadius = parameter.m_neighborRadius;
    int size = parameter.m_size;

    CGAL::edge_aware_upsample_point_set<CGAL::Parallel_if_available_tag>(points, std::back_inserter(points),
        CGAL::parameters::
        point_map(CGAL::First_of_pair_property_map<PointNormal>()).
        normal_map(CGAL::Second_of_pair_property_map<PointNormal>()).
        sharpness_angle(sharpnessAngle).
        edge_sensitivity(edgeSensitivity).
        neighbor_radius(neighborRadius).
        number_of_output_points(size));
    calculateNormals();
}

static void smooth(const CSmoothParameter& parameter) {
    int k = parameter.m_k;
    float sharpnessAngle = parameter.m_sharpnessAngle;

    CGAL::bilateral_smooth_point_set<CGAL::Sequential_tag>(points, k,
        CGAL::parameters::
        point_map(CGAL::First_of_pair_property_map<PointNormal>()).
        normal_map(CGAL::Second_of_pair_property_map<PointNormal>()).
        sharpness_angle(sharpnessAngle));
    calculateNormals();
}

static void reconstruct(const CReconstructParameter& parameter) {
    int iterationNumber = parameter.m_iterationNumber;
    float maximumRadius = parameter.m_maximumRadius;

    std::vector<Point> pointsTemp;
    for (const PointNormal& point : points)
        pointsTemp.push_back(point.first);

    CGAL::Scale_space_surface_reconstruction_3<Kernel> reconstruct(pointsTemp.begin(), pointsTemp.end());
    reconstruct.increase_scale(4, CGAL::Scale_space_reconstruction_3::Weighted_PCA_smoother<Kernel>());
    reconstruct.reconstruct_surface(CGAL::Scale_space_reconstruction_3::Advancing_front_mesher<Kernel>(maximumRadius));

    for (auto iter = reconstruct.facets_begin(); iter != reconstruct.facets_end(); iter++)
        for (unsigned int index : *iter)
            indices.push_back(index);

    /*CGAL::Scale_space_surface_reconstruction_3<Kernel> reconstruct(pointsTemp.begin(), pointsTemp.end());
    reconstruct.increase_scale(iterationNumber, CGAL::Scale_space_reconstruction_3::Weighted_PCA_smoother<Kernel>());
    reconstruct.reconstruct_surface(CGAL::Scale_space_reconstruction_3::Advancing_front_mesher<Kernel>(maximumRadius));

    for (auto iter = reconstruct.facets_begin(); iter != reconstruct.facets_end(); iter++)
        for (unsigned int index : *iter)
            indices.push_back(index);*/
}

static bool testConstructor() {
    refresh();

    calculateNormals();
    cpoints = (new CPointSet(cpoints))->getPoints();

    return comparePointSet(1e-6);
}

static bool testSimplify() {
    refresh();

    simplify(simplifyParameter);
    cpoints = (new CPointSet(cpoints))->simplify(simplifyParameter)->getPoints();

    return comparePointSet(1e-12);
}

static bool testResample() {
    refresh();

    resample(resampleParameter);
    cpoints = (new CPointSet(cpoints))->resample(resampleParameter)->getPoints();

    return comparePointSet(1e-4);
}

static bool testSmooth() {
    refresh();
    
    smooth(smoothParameter);
    cpoints = (new CPointSet(cpoints))->smooth(smoothParameter)->getPoints();

    return comparePointSet(1e-12);
}

static bool testReconstruct() {
    refresh();

    reconstruct(reconstructParameter);
    cindices = (new CPointSet(cpoints))->reconstruct(reconstructParameter)->getIndices();

    return compareMesh();
}

int main() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "PointCloudViewer", nullptr, nullptr);
    if (window == nullptr) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    glEnable(GL_DEPTH_TEST);

    std::string s;
    std::ifstream fin("../data/pool.dat");
    while (fin >> s) {
        if (s == "v") {
            float x, y, z, weight;
            int cluster;
            fin >> x >> y >> z >> cluster >> weight;

            points.push_back(std::make_pair(Point(x, y, z), Vector(0.0f, 0.0f, 0.0f)));
        }
        getline(fin, s);
    }

    if (!testConstructor())
        std::cout << "Constructor test failed." << std::endl;
    else if (!testSimplify())
        std::cout << "Simplify test failed." << std::endl;
    else if (!testResample())
        std::cout << "Resample test failed." << std::endl;
    else if (!testSmooth())
        std::cout << "Smooth test failed." << std::endl;
    else if (!testReconstruct())
        std::cout << "Reconstruct test failed." << std::endl;
    else
        std::cout << "All tests passed." << std::endl;

    return 0;
}