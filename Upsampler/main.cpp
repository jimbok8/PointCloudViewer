#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_opengl3.h>

#include "Vertex.h"
#include "Shader.h"
#include "PointSet.h"

const unsigned int WINDOW_WIDTH = 1920;
const unsigned int WINDOW_HEIGHT = 1080;
const float PI = std::acos(-1);

int lastX = INT_MIN, lastY = INT_MIN, display = 0, size;
double sharpnessAngle = 25.0, edgeSensitivity = 0.0, neighborRadius = 0.1;
float factor = 1.0f;
bool press;
glm::mat4 rotate(1.0f);
PointSet origin, upsample;

void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        press = true;
        lastX = lastY = INT_MIN;
    } else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
        press = false;
}

void cursorPosCallback(GLFWwindow* window, double x, double y) {
    if (press && lastX != INT_MIN && lastY != INT_MIN) {
        glm::vec3 a = glm::normalize(glm::vec3((float)lastX / WINDOW_WIDTH - 0.5f, 0.5f - (float)lastY / WINDOW_HEIGHT, 1.0f));
        glm::vec3 b = glm::normalize(glm::vec3((float)x / WINDOW_WIDTH - 0.5f, 0.5f - (float)y / WINDOW_HEIGHT, 1.0f));
        glm::vec3 axis = glm::cross(a, b);
        float angle = glm::dot(a, b);
        rotate = glm::rotate(glm::mat4(1.0f), 10.0f * acos(angle), axis) * rotate;
    }

    lastX = (int)x;
    lastY = (int)y;
}

void scrollCallback(GLFWwindow* window, double x, double y) {
    factor += 0.01f * (float)y;
    factor = std::max(factor, 0.01f);
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

void calculate() {
    upsample = origin.upsample(sharpnessAngle, edgeSensitivity, neighborRadius, size);
}

int main(int argc, char** argv) {
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

    Shader shader("shader/Vertex.glsl", "shader/Fragment.glsl");
    std::vector<Vertex> points;
    std::string s;
    std::ifstream fin("../data/pool.dat");
    float minX, maxX, minY, maxY, minZ, maxZ;
    minX = minY = minZ = FLT_MAX;
    maxX = maxY = maxZ = -FLT_MAX;
    while (fin >> s) {
        if (s == "v") {
            float x, y, z, weight;
            int cluster;
            fin >> x >> y >> z >> cluster >> weight;
            weight = log(weight);

            minX = std::min(minX, x);
            maxX = std::max(maxX, x);
            minY = std::min(minY, y);
            maxY = std::max(maxY, y);
            minZ = std::min(minZ, z);
            maxZ = std::max(maxZ, z);

            points.push_back(Vertex(x, y, z));
        }
        getline(fin, s);
    }

    glm::vec3 center((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2);
    for (int i = 0; i < points.size(); i++)
        points[i].position -= center;

    origin = PointSet(points);
    size = 500;
    calculate();

    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Options");

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNodeEx("Displaying option", true)) {
            ImGui::RadioButton("Display original point cloud", &display, 0);
            ImGui::RadioButton("Display upsampled point cloud", &display, 1);
            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNodeEx("Upsampling options", true)) {
            ImGui::InputDouble("sharpnessAngle", &sharpnessAngle);
            ImGui::InputDouble("edgeSensitivity", &edgeSensitivity);
            ImGui::InputDouble("neighborRadius", &neighborRadius);
            ImGui::InputInt("size", &size);
            ImGui::TreePop();
        }

        if (ImGui::Button("Calculate"))
            calculate();

        glm::vec3 lightDirection(0.0f, 0.0f, -1.0f), cameraPosition(0.0f, 0.0f, 1.5f);
        glm::mat4 modelMat, viewMat, projectionMat;
        modelMat = glm::scale(rotate, glm::vec3(factor));
        viewMat = glm::lookAt(cameraPosition, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        projectionMat = glm::perspective(PI / 4.0f, (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 0.1f, 100.0f);

        shader.use();
        shader.setMat4("model", modelMat);
        shader.setMat4("view", viewMat);
        shader.setMat4("projection", projectionMat);
        shader.setVec3("lightDirection", lightDirection);
        shader.setVec3("cameraPosition", cameraPosition);

        if (display == 0)
            origin.render();
        else if (display == 1)
            upsample.render();

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
