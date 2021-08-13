#include <algorithm>
#include <fstream>
#include <Windows.h>

#include <Eigen/Dense>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "CPoint.h"
#include "CShader.h"
#include "Matrix.h"
#include "Scene.h"
#include "SurfelGPRenderer/SurfelGPRenderer.h"

const unsigned int WINDOW_WIDTH = 1024;
const unsigned int WINDOW_HEIGHT = 768;

static int g_leftLastX, g_leftLastY, g_rightLastX, g_rightLastY;
static float g_factor = 1.0f;
static bool g_leftPress = false, g_rightPress = false;
static SurfelGPRenderer* g_renderer;

static void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    g_renderer->setViewPortSize(CSize(width, height));
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
        Eigen::Vector3f a = Eigen::Vector3f((float)g_leftLastX / WINDOW_WIDTH - 0.5f, 0.5f - (float)g_leftLastY / WINDOW_HEIGHT, 1.0f).normalized();
        Eigen::Vector3f b = Eigen::Vector3f((float)newX / WINDOW_WIDTH - 0.5f, 0.5f - (float)newY / WINDOW_HEIGHT, 1.0f).normalized();
        Eigen::Vector3f axis = a.cross(b);
        float angle = a.dot(b);
        Scene::getInstance()->rotate(5.0f * std::acos(angle), axis.x(), axis.y(), axis.z());
    }
    if (g_rightPress && g_rightLastX != INT_MIN && g_rightLastY != INT_MIN) {
        Eigen::Vector3f v((float)(newX - g_rightLastX), (float)(g_rightLastY - newY), 0.0f);
        Scene::getInstance()->translate(v.x(), v.y(), v.z());
    }

    g_leftLastX = g_rightLastX = newX;
    g_leftLastY = g_rightLastY = newY;
}

static void scrollCallback(GLFWwindow* window, double x, double y) {
    float newFactor = g_factor + 0.1f * (float)y;
    newFactor = std::max(newFactor, 0.10f);
    float ratio = newFactor / g_factor;
    Scene::getInstance()->scale(ratio, ratio, ratio);
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
    std::vector<CPoint> points;
    while (fin >> x >> y >> z >> ux >> uy >> uz >> vx >> vy >> vz) {
        Eigen::Vector3f position(x, y, z), u(ux, uy, uz), v(vx, vy, vz);
        position *= 100.0f;
        u *= 100.0f;
        v *= 100.0f;
        points.push_back(CPoint(position, u, v));
        minX = std::min(minX, position.x());
        maxX = std::max(maxX, position.x());
        minY = std::min(minY, position.y());
        maxY = std::max(maxY, position.y());
        minZ = std::min(minZ, position.z());
        maxZ = std::max(maxZ, position.z());
    }
    float centerX = (minX + maxX) * 0.5f, centerY = (minY + maxY) * 0.5f, centerZ = (minZ + maxZ) * 0.5f;

    Scene* scene = Scene::getInstance();
	scene->reset(true, false);
    scene->setAutoDelete(true);
    MyDataTypes::CameraPosition cameraPosition;
    MtrUtil::MtrUnity4x4f(cameraPosition.scaleTranslationMatrix);
    cameraPosition.scaleTranslationMatrix[14] = -1000.0f;
    MtrUtil::MtrUnity4x4f(cameraPosition.rotationMatrix);
    scene->setCameraPosition(cameraPosition, false);

    Object* object = new Object();
    object->setName("object", false);
    object->setScale(1.0f, 1.0f, 1.0f, false);
    object->setPosition(0.0f, 0.0f, 0.0f, false);
    object->setRotationMatrix(cameraPosition.rotationMatrix, false);

    SurfelCollection* surfelCollection = object->getSurfelCollection();
    SurfelInterface::PropertyDescriptor actualPropertyDescriptor = surfelCollection->getPropertyDescriptor();
    surfelCollection->reserve(surfelCollection->capacity() + points.size());

    for (const CPoint& point : points) {
        SurfelInterface* surfel = surfelCollection->addSurfel(false);
        Vector3D position(point.m_position.x() - centerX, point.m_position.y() - centerY, point.m_position.z() - centerZ);
        Eigen::Vector3f normalTemp = point.m_u.cross(point.m_v).normalized();
        Vector3D normal(normalTemp.x(), normalTemp.y(), normalTemp.z());

        float f = (point.m_position.x() - minX) / (maxX - minX), r, g, b;
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
        COLORREF diffuseColor = RGB((unsigned char)(r * 255), (unsigned char)(g * 255), (unsigned char)(b * 255));
        COLORREF specularColor = RGB(205, 205, 205);

        scene->setSurfelPosition(surfel, position, false);
        scene->setSurfelNormal(surfel, normal, false);
        scene->setSurfelRadius(surfel, point.m_u.norm() * 0.5, false);
        scene->setSurfelDiffuseColor(surfel, diffuseColor, false);
        scene->setSurfelSpecularColor(surfel, specularColor, false);
        scene->setSurfelFlags(surfel, 0, false);
    }

    scene->addObject(object, true, true);

    g_renderer = new SurfelGPRenderer();
    g_renderer->initialize(true);

    MyDataTypes::ViewFrustum viewFrustum;
    viewFrustum.fieldOfView = 30.0f;
    viewFrustum.aspectRatio = 1.0f;
    viewFrustum.nearPlane = 10.0f;
    viewFrustum.farPlane = 100000.0f;
    g_renderer->setViewFrustum(viewFrustum);
    g_renderer->setViewPortSize(CSize(WINDOW_WIDTH, WINDOW_HEIGHT));
    g_renderer->setTwoSidedNormalsEnabled(true);
    g_renderer->setShadowsEnabled(false);
    g_renderer->setShadingEnabled(true);
    g_renderer->setLightDirection(Vector3D(0.0f, 0.0f, -1.0f));
    g_renderer->setBackgroundColor(RGB(25, 25, 25));

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

        MyDataTypes::TransformationMatrix16f sceneViewMatrix;
        Scene::getInstance()->getTransformationMatrix(sceneViewMatrix);
        g_renderer->setSceneView(sceneViewMatrix);

        g_renderer->renderFrame();
        CImage* image = g_renderer->getRenderedImage();
        
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image->getWidth(), image->getHeight(), 0, GL_RGB, GL_UNSIGNED_BYTE, image->getData());

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

    return 0;
}