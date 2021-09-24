#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <vector>
#include <fstream>
#include <iostream>

#include <stb_image_write.h>
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
        surfels[i].color = Eigen::Vector4f(r * 255.0f, g * 255.0f, b * 255.0f, 1.0f);
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
    stbi_write_png("out.png", g_renderer->getWidth(), g_renderer->getHeight(), 3, g_renderer->getImage(), 0);

    CRenderShader renderShader("shader/Vertex.glsl", "shader/Fragment.glsl");
    float vertices[] = {
         1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
         1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
        -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
        -1.0f,  1.0f, 0.0f, 0.0f, 1.0f
    };
    unsigned int indices[] = {
        0, 1, 3,
        1, 2, 3
    };
    unsigned int vao, vbo, ebo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    renderShader.use();
    renderShader.setInt("tex", 0);

    unsigned int texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    double last, current;
    int frames = 0;
    while (!glfwWindowShouldClose(window)) {
        if (frames == 0)
            last = glfwGetTime();

        g_renderer->render();
        
        glClear(GL_COLOR_BUFFER_BIT);
        stbi_write_png("out.png", g_renderer->getWidth(), g_renderer->getHeight(), 3, g_renderer->getImage(), 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_renderer->getWidth(), g_renderer->getHeight(), 0, GL_RGB, GL_UNSIGNED_BYTE, g_renderer->getImage());

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);

        renderShader.use();
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        glfwSwapBuffers(window);
        glfwPollEvents();

        frames++;
        if (frames == 10) {
            current = glfwGetTime();
            std::cout << 10.0 / (current - last) << std::endl;
            frames = 0;
        }
    }

    glfwTerminate();

    return 0;
}