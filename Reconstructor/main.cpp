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
#include "Mesh.h"

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

int lastX = INT_MIN, lastY = INT_MIN;
float factor = 1.0f;
bool press;
glm::mat4 rotate(1.0f);

int sizeThreshold = 10, numCluster, removeK = 24, smoothK = 30, resolution = 64, type = 0;
double divideScale = 3.0, removeScale = 2.0, simplifyScale = 2.0;
float divideThreshold = 30.0f;
std::vector<PointSet> origins, removes, simplifies, smoothes, divides;
std::vector<Mesh> meshes;

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
    std::vector<PointSet>().swap(divides);
    for (PointSet& set : origins) {
        std::vector<PointSet> temp = set.divide(divideScale, glm::radians(divideThreshold));
        for (PointSet& tmp : temp)
            if (tmp.size() >= sizeThreshold)
                divides.push_back(tmp);
    }

    std::vector<PointSet>().swap(removes);
    for (PointSet& set : divides)
        if (set.size() > 0)
            removes.push_back(set.removeOutliers(removeK, removeScale));

    std::vector<PointSet>().swap(simplifies);
    for (PointSet& set : removes)
        if (set.size() > 0)
            simplifies.push_back(set.simplify(simplifyScale));

    std::vector<PointSet>().swap(smoothes);
    for (PointSet& set : simplifies)
        if (set.size() > 0)
            smoothes.push_back(set.smooth(smoothK));

    std::vector<Mesh>().swap(meshes);
    for (PointSet& set : smoothes)
        if (set.size() > 0)
            meshes.push_back(set.reconstruct(type, resolution));
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
    std::ifstream fin("../data/model.dat");
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
    float pointScale = std::max(maxX - minX, std::max(maxY - minY, maxZ - minZ));
    numCluster = *std::max_element(clusters.begin(), clusters.end()) + 1;

    glm::vec3 center((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2);
    std::vector<std::vector<Vertex>> vertices(numCluster);
    for (int i = 0; i < points.size(); i++) {
        points[i].position -= center;
        vertices[clusters[i]].push_back(points[i]);
    }

    for (int i = 0; i < numCluster; i++)
        origins.push_back(PointSet(vertices[i]));
    calculate();

    int display = 0, color = 0;

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
            ImGui::RadioButton("Display removed point cloud", &display, 2);
            ImGui::RadioButton("Display simplified point cloud", &display, 3);
            ImGui::RadioButton("Display smoothed point cloud", &display, 4);
            ImGui::RadioButton("Display reconstructed mesh", &display, 5);
            ImGui::NewLine();
            ImGui::RadioButton("Color by normal", &color, 0);
            ImGui::RadioButton("Color by cluster", &color, 1);
            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);

        if (ImGui::TreeNodeEx("Parameter option", true)) {
            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            if (ImGui::TreeNodeEx("Divide", true)) {
                ImGui::InputDouble("scale", &divideScale);
                ImGui::InputFloat("threshold", &divideThreshold);
                ImGui::InputInt("sizeThreshold", &sizeThreshold);
                ImGui::TreePop();
            }

            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            if (ImGui::TreeNodeEx("Remove outliers", true)) {
                ImGui::InputInt("k", &removeK);
                ImGui::InputDouble("scale", &removeScale);
                ImGui::TreePop();
            }

            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            if (ImGui::TreeNodeEx("Simplify", true)) {
                ImGui::InputDouble("scale", &simplifyScale);
                ImGui::TreePop();
            }

            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            if (ImGui::TreeNodeEx("Smooth", true)) {
                ImGui::InputInt("k", &smoothK);
                ImGui::TreePop();
            }

            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            if (ImGui::TreeNodeEx("Reconstruct", true)) {
                ImGui::InputInt("resolution", &resolution);
                ImGui::TreePop();
            }

            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            if (ImGui::TreeNodeEx("Reconstruct method", true)) {
                ImGui::RadioButton("Use advancing front surface reconstruction", &type, 0);
                ImGui::RadioButton("Use scale space surface reconstruction", &type, 1);
                ImGui::RadioButton("Use Poisson surface reconstruction", &type, 2);
                ImGui::RadioButton("Use greedy projection triangulation", &type, 3);
                ImGui::RadioButton("Use marching cubes Hoppe", &type, 4);
                ImGui::RadioButton("Use marching cubes RBF", &type, 5);
                ImGui::TreePop();
            }

            ImGui::TreePop();
        }

        if (ImGui::Button("Calculate"))
            calculate();

        glm::vec3 lightDirection(0.0f, 0.0f, -1.0f), cameraPosition(0.0f, 0.0f, 1.5f * pointScale);
        glm::mat4 modelMat, viewMat, projectionMat;
        modelMat = glm::scale(rotate, glm::vec3(factor));
        viewMat = glm::lookAt(cameraPosition, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        projectionMat = glm::perspective((float)M_PI / 4.0f, (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 0.1f, 100.0f);

        if (color == 0) {
            normalShader.use();
            normalShader.setMat4("model", modelMat);
            normalShader.setMat4("view", viewMat);
            normalShader.setMat4("projection", projectionMat);
            normalShader.setVec3("lightDirection", lightDirection);
            normalShader.setVec3("cameraPosition", cameraPosition);
        }
        else if (color == 1) {
            clusterShader.use();
            clusterShader.setMat4("model", modelMat);
            clusterShader.setMat4("view", viewMat);
            clusterShader.setMat4("projection", projectionMat);
            clusterShader.setVec3("lightDirection", lightDirection);
            clusterShader.setVec3("cameraPosition", cameraPosition);
        }

        if (display == 0)
            for (unsigned int i = 0; i < origins.size(); i++) {
                clusterShader.setVec3("color", COLORS[i % COLOR_SIZE]);
                origins[i].render();
            }
        else if (display == 1)
            for (unsigned int i = 0; i < divides.size(); i++) {
                clusterShader.setVec3("color", COLORS[i % COLOR_SIZE]);
                divides[i].render();
            }
        else if (display == 2)
            for (unsigned int i = 0; i < removes.size(); i++) {
                clusterShader.setVec3("color", COLORS[i % COLOR_SIZE]);
                removes[i].render();
            }
        else if (display == 3)
            for (unsigned int i = 0; i < simplifies.size(); i++) {
                clusterShader.setVec3("color", COLORS[i % COLOR_SIZE]);
                simplifies[i].render();
            }
        else if (display == 4)
            for (unsigned int i = 0; i < smoothes.size(); i++) {
                clusterShader.setVec3("color", COLORS[i % COLOR_SIZE]);
                smoothes[i].render();
            }
        else if (display == 5)
            for (unsigned int i = 0; i < meshes.size(); i++) {
                clusterShader.setVec3("color", COLORS[i % COLOR_SIZE]);
                meshes[i].render();
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
