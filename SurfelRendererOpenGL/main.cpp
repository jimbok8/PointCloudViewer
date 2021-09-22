#include <vector>
#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "TypeHelper.h"
#include "CRenderer.h"

const int WINDOW_WIDTH = 1024;
const int WINDOW_HEIGHT = 768;

static CRenderer* g_renderer;

int main() {
    std::ifstream fin("../data/qjhdl/hd.dat");
    float x, y, z, ux, uy, uz, vx, vy, vz;
    float minX, maxX, minY, maxY, minZ, maxZ;
    minX = minY = minZ = FLT_MAX;
    maxX = maxY = maxZ = -FLT_MAX;
    std::vector<Eigen::Vector3f> positions, us, vs;
    while (fin >> x >> y >> z >> ux >> uy >> uz >> vx >> vy >> vz) {
        Eigen::Vector3f position(x, y, z), u(ux, uy, uz), v(vx, vy, vz);
        position *= 100.0f;
        u *= 100.0f;
        v *= 100.0f;
        positions.push_back(position);
        us.push_back(u);
        vs.push_back(v);
        minX = std::min(minX, position[0]);
        maxX = std::max(maxX, position[0]);
        minY = std::min(minY, position[1]);
        maxY = std::max(maxY, position[1]);
        minZ = std::min(minZ, position[2]);
        maxZ = std::max(maxZ, position[2]);
    }
    int numSurfels = positions.size();
    Eigen::Vector3f center((minX + maxX) * 0.5f, (minY + maxY) * 0.5f, (minZ + maxZ) * 0.5f);

    Surfel* surfels = new Surfel[numSurfels];
    for (int i = 0; i < numSurfels; i++) {
        Eigen::Vector3f position = positions[i] - center;
        Eigen::Vector3f normal = us[i].cross(vs[i]).normalized();

        float f = (positions[i][0] - minX) / (maxX - minX), r, g, b;
        if (f <= 0.5f) {
            r = 0.0f;
            g = f * 2.0f;
            b = 1.0f - g;
        }
        else {
            b = 0.0f;
            r = (f - 0.5f) * 2.0f;
            g = 1.0f - r;
        }

        surfels[i].position = Eigen::Vector4f(position(0), position(1), position(2), 1.0f);
        surfels[i].normal = Eigen::Vector4f(normal(0), normal(1), normal(2), 0.0f);
        surfels[i].color = Eigen::Vector4f(r, g, b, 1.0f);
        surfels[i].radius = us[i].norm();
    }

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "PointCloudViewer", nullptr, nullptr);
    if (window == nullptr) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    /*glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetKeyCallback(window, keyCallback);*/

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    glEnable(GL_DEPTH_TEST);

    g_renderer = new CRenderer(positions.size(), surfels, WINDOW_WIDTH, WINDOW_HEIGHT, 25, 25, 25);
    g_renderer->render();

    return 0;
}