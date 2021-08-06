#include <vector>
#include <string>

#include <Eigen/Dense>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_opengl3.h>

#include "TransformHelper.h"
#include "CPoint.h"
#include "CPointSet.h"
#include "CShader.h"

const unsigned int WINDOW_WIDTH = 1920;
const unsigned int WINDOW_HEIGHT = 1080;
const float PI = std::acos(-1.0f);

static int g_leftLastX, g_leftLastY, g_rightLastX, g_rightLastY;
static float g_factor, g_factorDiff;
static bool g_leftPress = false, g_rightPress = false;
static Eigen::Matrix4f g_translate = Eigen::Matrix4f::Identity(), g_rotation = Eigen::Matrix4f::Identity();

static void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
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
        g_rotation = rotate(axis, 5.0f * std::acos(angle)) * g_rotation;
    }
    if (g_rightPress && g_rightLastX != INT_MIN && g_rightLastY != INT_MIN) {
        Eigen::Vector3f v((float)(newX - g_rightLastX), (float)(g_rightLastY - newY), 0.0f);
        g_translate = translate(v * 0.005f) * g_translate;
    }

    g_leftLastX = g_rightLastX = newX;
    g_leftLastY = g_rightLastY = newY;
}

static void scrollCallback(GLFWwindow* window, double x, double y) {
    g_factor += g_factorDiff * (float)y;
    g_factor = std::max(g_factor, g_factorDiff);
}

static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
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
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetKeyCallback(window, keyCallback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    glEnable(GL_DEPTH_TEST);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    CShader normalShader("shader/NormalVertex.glsl", "shader/NormalFragment.glsl");

    CPointSet* set = new CPointSet("../data/qjhdl/hd.dat");

    g_factor = 1.0f / set->scale();
    g_factorDiff = 0.1f * g_factor;
    
    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Options");

        if (ImGui::Button("Reset")) {
            g_factor = 1.0f / set->scale();
            g_rotation = g_translate = Eigen::Matrix4f::Identity();
        }

        Eigen::Vector3f lightDirection(0.0f, 0.0f, -1.0f), cameraPosition(0.0f, 0.0f, 2.0f);
        Eigen::Matrix4f modelMat, viewMat, projectionMat;
        modelMat = g_translate * g_rotation * scale(g_factor);
        viewMat = lookAt(cameraPosition, Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
        projectionMat = perspective(PI / 4.0f, (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 0.1f, 100.0f);

        normalShader.use();
        normalShader.setMatrix4D("model", modelMat);
        normalShader.setMatrix4D("view", viewMat);
        normalShader.setMatrix4D("projection", projectionMat);
        normalShader.setFloat("minX", set->getMinX());
        normalShader.setFloat("maxX", set->getMaxX());
        normalShader.setVector3D("lightDirection", lightDirection);
        normalShader.setVector3D("cameraPosition", cameraPosition);
        
        set->render();

        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();

    return 0;
}