#include <algorithm>
#include <string>
#include <vector>
#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "ParameterHelper.h"
#include "PointSet.h"
#include "CPointSet.h"

const unsigned int WINDOW_WIDTH = 1920;
const unsigned int WINDOW_HEIGHT = 1080;
const float EPSILON = 1e-10f;

static std::vector<CPoint> points;
static std::vector<Vertex> vertices;

static bool comparePoint(const CPoint& a, const CPoint& b) {
    for (int i = 0; i < 3; i++)
        if (a.m_position[i] != b.m_position[i])
            return a.m_position[i] < b.m_position[i];

    return true;
}

static bool compareVertex(const Vertex& a, const Vertex& b) {
    for (int i = 0; i < 3; i++)
        if (a.position[i] != b.position[i])
            return a.position[i] < b.position[i];

    return true;
}

static bool compare(std::vector<CPoint>& points, std::vector<Vertex>& vertices) {
    if (points.size() != vertices.size())
        return false;

    std::sort(points.begin(), points.end(), comparePoint);
    std::sort(vertices.begin(), vertices.end(), compareVertex);
    for (int i = 0; i < points.size(); i++) {
        Eigen::Vector3f position(vertices[i].position.x, vertices[i].position.y, vertices[i].position.z);

        float sumPosition = 0.0f, sumNormal = 0.0f;
        for (int j = 0; j < 3; j++) {
            float diffPosition = points[i].m_position[j] - vertices[i].position[j];
            sumPosition += diffPosition * diffPosition;
            float diffNormal = points[i].m_normal[j] - vertices[i].normal[j];
            sumNormal += diffNormal * diffNormal;
        }
        if (sumPosition > EPSILON || sumNormal > EPSILON)
            return false;
    }

    return true;
}

static bool TestCalculateNormals() {
    points = (new CPointSet(points))->getPoints();
    vertices = PointSet(vertices).getVertices();
    return compare(points, vertices);
}

static bool TestSimplify() {
    points = (new CPointSet(points))->simplify(epsilon)->getPoints();
    vertices = PointSet(vertices).simplify(epsilon).getVertices();
    return compare(points, vertices);
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

            points.push_back(CPoint(Eigen::Vector3f(x, y, z)));
            vertices.push_back(Vertex(x, y, z));
        }
        getline(fin, s);
    }

    if (!compare(points, vertices))
        std::cout << "FUCK" << std::endl;
    if (!TestCalculateNormals())
        std::cout << "TestCalculateNormals failed." << std::endl;
    else if (!TestSimplify())
        std::cout << "TestSimplify failed." << std::endl;
    else
        std::cout << "Test passed." << std::endl;
    
    return 0;
}