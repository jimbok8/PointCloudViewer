#include <vector>
#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "TypeHelper.h"

int main() {
    std::ifstream fin("../data/qjhdl/hd.dat");
    float x, y, z, ux, uy, uz, vx, vy, vz;
    float minX, maxX, minY, maxY, minZ, maxZ;
    minX = minY = minZ = FLT_MAX;
    maxX = maxY = maxZ = -FLT_MAX;
    std::vector<Eigen::Vector4f> positions, us, vs;
    while (fin >> x >> y >> z >> ux >> uy >> uz >> vx >> vy >> vz) {
        Eigen::Vector4f position(x, y, z, 1.0f), u(ux, uy, uz, 0.0f), v(vx, vy, vz, 0.0f);
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
    Eigen::Vector4f center((minX + maxX) * 0.5f, (minY + maxY) * 0.5f, (minZ + maxZ) * 0.5f, 0.0f);

    Surfel* surfels = new Surfel[positions.size()];
    for (int i = 0; i < positions.size(); i++) {
        Vector3D position = positions[i] - center;
        Vector3D normal = Vector3D::crossProduct(us[i], vs[i]);
        normal.normalize();

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

        surfels[i].position = position;
        surfels[i].normal = normal;
        surfels[i].radius = us[i].getLength();
        surfels[i].red = r * 255.0f;
        surfels[i].green = g * 255.0f;
        surfels[i].blue = b * 255.0f;
    }

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    /*GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "PointCloudViewer", nullptr, nullptr);
    if (window == nullptr) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetKeyCallback(window, keyCallback);*/

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    glEnable(GL_DEPTH_TEST);



    return 0;
}