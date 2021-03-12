#include <algorithm>
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

#include "Vertex.h"
#include "Shader.h"
#include "PointSet.h"

const unsigned int WINDOW_WIDTH = 1920;
const unsigned int WINDOW_HEIGHT = 1080;
const glm::vec3 COLORS[] = {
    glm::vec3(1.0f, 0.0f, 0.0f),
    glm::vec3(0.0f, 1.0f, 0.0f),
    glm::vec3(0.0f, 0.0f, 1.0f),
    glm::vec3(0.0f, 1.0f, 1.0f),
    glm::vec3(1.0f, 0.0f, 1.0f),
    glm::vec3(1.0f, 1.0f, 0.0f)
};
const int COLOR_SIZE = sizeof(COLORS) / sizeof(glm::vec3);

int lastX = INT_MIN, lastY = INT_MIN, numCluster, display = 0, color = 1;
float factor = 1.0f, threshold = 30.0f;
double epsilon = 2.5;
bool press, *saves;
glm::mat4 rotate(1.0f);
std::vector<PointSet> origins, divides;

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
    int newX = (int)x, newY = (int)y;
    if (press && lastX != INT_MIN && lastY != INT_MIN && (newX != lastX || newY != lastY)) {
        glm::vec3 a = glm::normalize(glm::vec3((float)lastX / WINDOW_WIDTH - 0.5f, 0.5f - (float)lastY / WINDOW_HEIGHT, 1.0f));
        glm::vec3 b = glm::normalize(glm::vec3((float)newX / WINDOW_WIDTH - 0.5f, 0.5f - (float)newY / WINDOW_HEIGHT, 1.0f));
        glm::vec3 axis = glm::cross(a, b);
        float angle = glm::dot(a, b);
        rotate = glm::rotate(glm::mat4(1.0f), 10.0f * acos(angle), axis) * rotate;
    }

    lastX = newX;
    lastY = newY;
}

void scrollCallback(GLFWwindow* window, double x, double y) {
    factor += 0.01f * (float)y;
    factor = std::max(factor, 0.01f);
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

int main(int argc, char** argv) {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "PointCloudViewer", NULL, NULL);
    if (window == NULL) {
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

    Shader normalShader("shader/NormalVertex.glsl", "shader/NormalFragment.glsl");
    Shader clusterShader("shader/ClusterVertex.glsl", "shader/ClusterFragment.glsl");

    std::vector<Vertex> points;
    std::vector<int> clusters;
    std::string s;
    std::ifstream fin("../data/model_with_pool.dat");
    float minX, maxX, minY, maxY, minZ, maxZ;
    minX = minY = minZ = FLT_MAX;
    maxX = maxY = maxZ = -FLT_MAX;
    while (fin >> s) {
        if (s == "v") {
            float x, y, z, weight;
            int cluster;
            fin >> x >> y >> z >> cluster >> weight;

            minX = std::min(minX, x);
            maxX = std::max(maxX, x);
            minY = std::min(minY, y);
            maxY = std::max(maxY, y);
            minZ = std::min(minZ, z);
            maxZ = std::max(maxZ, z);

            points.push_back(Vertex(x, y, z));
            clusters.push_back(cluster);
        }
        getline(fin, s);
    }
    
    numCluster = *std::max_element(clusters.begin(), clusters.end()) + 1;
    glm::vec3 center((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2);
    std::vector<std::vector<Vertex>> vertices(numCluster);
    for (int i = 0; i < points.size(); i++) {
        points[i].position -= center;
        vertices[clusters[i]].push_back(points[i]);
    }

    for (int i = 0; i < numCluster; i++)
        origins.push_back(PointSet(vertices[i]));

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
            ImGui::RadioButton("Display divided point cloud", &display, 1);
            ImGui::NewLine();
            ImGui::RadioButton("Color by normal", &color, 0);
            ImGui::RadioButton("Color by cluster", &color, 1);
            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);

        if (ImGui::TreeNodeEx("Parameter option", true)) {
            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            if (ImGui::TreeNodeEx("Divide", true)) {
                ImGui::InputDouble("epsilon", &epsilon);
                ImGui::InputFloat("threshold", &threshold);
                ImGui::TreePop();
            }

            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            if (ImGui::TreeNodeEx("Save option", true)) {
                if (divides.size() <= 10)
                    for (int i = 0; i < divides.size(); i++)
                        ImGui::Checkbox(("Cluster" + std::to_string(i)).c_str(), &saves[i]);
                ImGui::TreePop();
            }

            ImGui::TreePop();
        }

        if (ImGui::Button("Divide")) {
            std::vector<PointSet>().swap(divides);
            for (PointSet& set : origins) {
                std::vector<PointSet> temp = set.divide(epsilon, glm::radians(threshold));
                for (int i = 0; i < temp.size(); i++) {
                    std::cout << "Cluster " << i << " contains " << temp[i].size() << " point(s)." << std::endl;
                    divides.push_back(temp[i]);
                }
            }
            delete saves;
            saves = new bool[divides.size()];
            memset(saves, false, sizeof(bool) * divides.size());
        }
        if (ImGui::Button("Save")) {
            std::vector<std::vector<Vertex>> vertices;
            for (int i = 0; i < divides.size(); i++)
                if (saves[i])
                    vertices.push_back(divides[i].getVertices());

            std::ofstream fout("../data/output.dat");
            for (int i = 0; i < vertices.size(); i++)
                for (Vertex& vertex : vertices[i])
                    fout << "v " << vertex.position.x << ' ' << vertex.position.y << ' ' << vertex.position.z << ' ' << i << " 0" << std::endl;
        }

        glm::vec3 lightDirection(0.0f, 0.0f, -1.0f), cameraPosition(0.0f, 0.0f, 2.0f);
        glm::mat4 modelMat, viewMat, projectionMat;
        modelMat = glm::scale(rotate, glm::vec3(factor));
        viewMat = glm::lookAt(cameraPosition, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        projectionMat = glm::perspective((float)M_PI / 4.0f, (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 0.1f, 100.0f);

        switch (color) {
        case 0:
            normalShader.use();
            normalShader.setMat4("model", modelMat);
            normalShader.setMat4("view", viewMat);
            normalShader.setMat4("projection", projectionMat);
            normalShader.setVec3("lightDirection", lightDirection);
            normalShader.setVec3("cameraPosition", cameraPosition);
            switch (display) {
            case 0:
                for (int i = 0; i < origins.size(); i++)
                    origins[i].render();
                break;
            case 1:
                for (int i = 0; i < divides.size(); i++)
                    divides[i].render();
                break;
            default:
                break;
            }
            break;

        case 1:
            clusterShader.use();
            clusterShader.setMat4("model", modelMat);
            clusterShader.setMat4("view", viewMat);
            clusterShader.setMat4("projection", projectionMat);
            clusterShader.setVec3("lightDirection", lightDirection);
            clusterShader.setVec3("cameraPosition", cameraPosition);
            switch (display) {
            case 0:
                for (int i = 0; i < origins.size(); i++) {
                    clusterShader.setVec3("color", COLORS[i % COLOR_SIZE]);
                    origins[i].render();
                }
                break;
            case 1:
                for (int i = 0; i < divides.size(); i++) {
                    clusterShader.setVec3("color", COLORS[i % COLOR_SIZE]);
                    divides[i].render();
                }
                break;
            default:
                break;
            }
            break;

        default:
            break;
        }

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
