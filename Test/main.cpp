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
const float EPSILON = 1e-8f;

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

static bool comparePointSet() {
    std::ofstream fout;
    fout.open("own.txt");
    for (const CPoint& point : points)
        fout << point.m_position(0) << ' ' << point.m_position(1) << ' ' << point.m_position(2) << std::endl;
    fout.close();
    
    fout.open("lib.txt");
    for (const Vertex& vertex : vertices)
        fout << vertex.position.x << ' ' << vertex.position.y << ' ' << vertex.position.z << std::endl;
    fout.close();

    if (points.size() != vertices.size()) {
        std::cout << points.size() << ' ' << vertices.size() << std::endl;
        return false;
    }

    std::sort(points.begin(), points.end(), comparePoint);
    std::sort(vertices.begin(), vertices.end(), compareVertex);

    //std::ofstream fout;
    //fout.open("own.txt");
    //for (const CPoint& point : points)
    //    fout << point.m_position(0) << ' ' << point.m_position(1) << ' ' << point.m_position(2) << std::endl;
    //fout.close();
    //
    //fout.open("lib.txt");
    //for (const Vertex& vertex : vertices)
    //    fout << vertex.position.x << ' ' << vertex.position.y << ' ' << vertex.position.z << std::endl;
    //fout.close();

    for (int i = 0; i < points.size(); i++) {
        Eigen::Vector3f position(vertices[i].position.x, vertices[i].position.y, vertices[i].position.z);
        if ((points[i].m_position - position).squaredNorm() > EPSILON) {
            std::cout << i << std::endl;
            std::cout << points[i].m_position << std::endl;
            std::cout << position << std::endl;
            return false;
        }
        
        Eigen::Vector3f normal(vertices[i].normal.x, vertices[i].normal.y, vertices[i].normal.z);
        if ((points[i].m_normal - normal).squaredNorm() > 0.01f && (points[i].m_normal + normal).squaredNorm() > 0.01f) {
            std::cout << i << std::endl;
            std::cout << points[i].m_normal << std::endl;
            std::cout << normal << std::endl;
            return false;
        }
    }

    return true;
}

static bool testConstructor() {
    points = (new CPointSet(points))->getPoints();
    vertices = PointSet(vertices).getVertices();
    return comparePointSet();
}

static bool testSimplify() {
    points = (new CPointSet(points))->simplify(epsilon)->getPoints();
    vertices = PointSet(vertices).simplify(epsilon).getVertices();
    return comparePointSet();
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

    if (!testConstructor())
        std::cout << "TestConstructor failed." << std::endl;
    //else if (!testSimplify())
    //    std::cout << "TestSimplify failed." << std::endl;
    else
        std::cout << "Test passed." << std::endl;

    return 0;
}