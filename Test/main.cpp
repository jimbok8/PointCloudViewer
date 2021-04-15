#include <algorithm>
#include <string>
#include <vector>
#include <iostream>

#include <ANN/ANN.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/edge_aware_upsample_point_set.h>
#include <CGAL/bilateral_smooth_point_set.h>

#include "CPointSet.h"
#include "CSimplifyParameter.h"
#include "CResampleParameter.h"
#include "CSmoothParameter.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef std::pair<Point, Vector> PointNormal;

const unsigned int WINDOW_WIDTH = 1920;
const unsigned int WINDOW_HEIGHT = 1080;

static std::vector<PointNormal> points;
static std::vector<CPoint> cpoints;
static CSimplifyParameter simplifyParameter(0.3f);
static CResampleParameter resampleParameter(25.0f, 0.0f, 3.0f, 3000);
static CSmoothParameter smoothParameter(64, 30.0f);

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
    else
        std::cout << "All tests passed." << std::endl;

    return 0;
}