#include <algorithm>
#include <fstream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "CShader.h"
#include "CRenderer.h"
#include "MatrixHelper.h"

const int WINDOW_WIDTH = 1024;
const int WINDOW_HEIGHT = 768;

static int g_leftLastX, g_leftLastY, g_rightLastX, g_rightLastY;
static float g_factor = 1.0f;
static bool g_leftPress = false, g_rightPress = false;
static CRenderer* g_renderer;

static void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    //g_renderer->resize(width, height);
    //g_renderer->render();
    glViewport(0, 0, width, height);
}

static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        g_leftPress = true;
        g_leftLastX = g_leftLastY = INT_MIN;
    }
    else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
        g_rightPress = true;
        g_rightLastX = g_rightLastY = INT_MIN;
    }
    else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
        g_leftPress = false;
    else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE)
        g_rightPress = false;
}

static void cursorPosCallback(GLFWwindow* window, double x, double y) {
    int newX = (int)x, newY = (int)y;
    if (g_leftPress && g_leftLastX != INT_MIN && g_leftLastY != INT_MIN && (newX != g_leftLastX || newY != g_leftLastY)) {
        Vector3D a = Vector3D((float)g_leftLastX / WINDOW_WIDTH - 0.5f, 0.5f - (float)g_leftLastY / WINDOW_HEIGHT, 1.0f);
        Vector3D b = Vector3D((float)newX / WINDOW_WIDTH - 0.5f, 0.5f - (float)newY / WINDOW_HEIGHT, 1.0f);
        a.normalize();
        b.normalize();
        Vector3D axis = Vector3D::crossProduct(a, b);
        float angle = Vector3D::dotProduct(a, b);
        g_renderer->rotate(5.0f * std::acos(angle), axis[0], axis[1], axis[2]);
    }
    if (g_rightPress && g_rightLastX != INT_MIN && g_rightLastY != INT_MIN) {
        Vector3D v((float)(newX - g_rightLastX), (float)(g_rightLastY - newY), 0.0f);
        g_renderer->translate(v[0], v[1], v[2]);
    }

    g_leftLastX = g_rightLastX = newX;
    g_leftLastY = g_rightLastY = newY;
}

static void scrollCallback(GLFWwindow* window, double x, double y) {
    float newFactor = g_factor + 0.1f * (float)y;
    newFactor = std::max(newFactor, 0.10f);
    float ratio = newFactor / g_factor;
    g_renderer->scale(ratio, ratio, ratio);
    g_factor = newFactor;
}

static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

int main() {
    std::ifstream fin("../data/qjhdl/hd.dat");
    float x, y, z, ux, uy, uz, vx, vy, vz;
    float minX, maxX, minY, maxY, minZ, maxZ;
    minX = minY = minZ = FLT_MAX;
    maxX = maxY = maxZ = -FLT_MAX;
    std::vector<Vector3D> positions, us, vs;
    while (fin >> x >> y >> z >> ux >> uy >> uz >> vx >> vy >> vz) {
        Vector3D position(x, y, z), u(ux, uy, uz), v(vx, vy, vz);
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
    Vector3D center((minX + maxX) * 0.5f, (minY + maxY) * 0.5f, (minZ + maxZ) * 0.5f);

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
    g_renderer = new CRenderer(positions.size(), surfels, WINDOW_WIDTH, WINDOW_HEIGHT, 25, 25, 25, false);

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
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetKeyCallback(window, keyCallback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    CShader shader("shader/vertex.glsl", "shader/fragment.glsl");
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

    shader.use();
    shader.setInt("tex", 0);

    //unsigned int fbo;
    //glGenFramebuffers(1, &fbo);
    //glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    unsigned int texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    //glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);

    double last, current;
    int frames = 0;
    while (!glfwWindowShouldClose(window)) {
        if (frames == 0)
            last = glfwGetTime();

        //glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glClear(GL_COLOR_BUFFER_BIT);

        g_renderer->render();
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_renderer->getWidth(), g_renderer->getHeight(), 0, GL_RGB, GL_UNSIGNED_BYTE, g_renderer->getImage());

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);

        shader.use();
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
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

    delete[] surfels;
    delete g_renderer;

    return 0;
}