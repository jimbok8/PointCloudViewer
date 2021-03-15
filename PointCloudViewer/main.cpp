#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_opengl3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "Vector3D.h"
#include "Matrix4D.h"
#include "Point.h"
#include "PointSet.h"
#include "Shader.h"

const unsigned int WINDOW_WIDTH = 1920;
const unsigned int WINDOW_HEIGHT = 1080;
const float PI = std::acos(-1);
const Vector3D COLORS[] = {
    Vector3D(1.0f, 0.0f, 0.0f),
    Vector3D(0.0f, 1.0f, 0.0f),
    Vector3D(0.0f, 0.0f, 1.0f),
    Vector3D(0.0f, 1.0f, 1.0f),
    Vector3D(1.0f, 0.0f, 1.0f),
    Vector3D(1.0f, 1.0f, 0.0f)
};
const int COLOR_SIZE = sizeof(COLORS) / sizeof(Vector3D);

int lastX = INT_MIN, lastY = INT_MIN, numCluster, display = 0, color = 0, cluster = 0, size = 10000, k = 64;
float factor = 1.0f;
double epsilon = 0.3, sharpnessAngle = 25.0, edgeSensitivity = 0.0, neighborRadius = 3.0, maximumFacetLength = 1.0;
bool press/*, * simplified, * upsampled, * smoothed, * reconstructed*/;
Matrix4D rotate(1.0f);
glm::mat4 rotateT(1.0f);
std::vector<PointSet> origins/*, simplifies, upsamples, smoothes*/;
//std::vector<Mesh> reconstructs;

void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        press = true;
        lastX = lastY = INT_MIN;
    }
    else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
        press = false;
}

void cursorPosCallback(GLFWwindow* window, double x, double y) {
    int newX = (int)x, newY = (int)y;
    if (press && lastX != INT_MIN && lastY != INT_MIN && (newX != lastX || newY != lastY)) {
        Vector3D a = Vector3D((float)lastX / WINDOW_WIDTH - 0.5f, 0.5f - (float)lastY / WINDOW_HEIGHT, 1.0f).normalize();
        Vector3D b = Vector3D((float)newX / WINDOW_WIDTH - 0.5f, 0.5f - (float)newY / WINDOW_HEIGHT, 1.0f).normalize();
        Vector3D axis = a.cross(b);
        float angle = a.dot(b);
        rotate = Matrix4D::rotate(axis, 10.0f * std::acos(angle)) * rotate;

        glm::vec3 aT = glm::normalize(glm::vec3((float)lastX / WINDOW_WIDTH - 0.5f, 0.5f - (float)lastY / WINDOW_HEIGHT, 1.0f));
        glm::vec3 bT = glm::normalize(glm::vec3((float)newX / WINDOW_WIDTH - 0.5f, 0.5f - (float)newY / WINDOW_HEIGHT, 1.0f));
        glm::vec3 axisT = glm::cross(aT, bT);
        float angleT = glm::dot(aT, bT);
        rotateT = glm::rotate(glm::mat4(1.0f), 10.0f * acos(angleT), axisT) * rotateT;
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

    Shader normalShader("shader/NormalVertex.glsl", "shader/NormalFragment.glsl");
    Shader clusterShader("shader/ClusterVertex.glsl", "shader/ClusterFragment.glsl");

    std::vector<Point> points;
    std::vector<int> clusters;
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

            points.push_back(Point(Vector3D(x, y, z)));
            clusters.push_back(cluster);
        }
        getline(fin, s);
    }

    numCluster = *std::max_element(clusters.begin(), clusters.end()) + 1;
    Vector3D center((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2);
    std::vector<std::vector<Point>> vertices(numCluster);
    for (int i = 0; i < points.size(); i++) {
        points[i].position = points[i].position - center;
        vertices[clusters[i]].push_back(points[i]);
    }

    for (int i = 0; i < numCluster; i++)
        origins.push_back(PointSet(vertices[i]));
    /*simplified = new bool[numCluster];
    memset(simplified, false, numCluster * sizeof(bool));
    simplifies = std::vector<PointSet>(numCluster);
    upsampled = new bool[numCluster];
    memset(upsampled, false, numCluster * sizeof(bool));
    upsamples = std::vector<PointSet>(numCluster);
    smoothed = new bool[numCluster];
    memset(smoothed, false, numCluster * sizeof(bool));
    smoothes = std::vector<PointSet>(numCluster);
    reconstructed = new bool[numCluster];
    memset(reconstructed, false, numCluster * sizeof(bool));
    reconstructs = std::vector<Mesh>(numCluster);*/

    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Options");

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNodeEx("Displaying options", true)) {
            ImGui::RadioButton("Display original point cloud", &display, 0);
            ImGui::RadioButton("Display simplified point cloud", &display, 1);
            ImGui::RadioButton("Display upsampled point cloud", &display, 2);
            ImGui::RadioButton("Display smoothed point cloud", &display, 3);
            ImGui::RadioButton("Display reconstructed mesh", &display, 4);
            ImGui::NewLine();
            ImGui::RadioButton("Color by normal", &color, 0);
            ImGui::RadioButton("Color by cluster", &color, 1);
            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNodeEx("Cluster options", true)) {
            for (int i = 0; i < numCluster; i++)
                ImGui::RadioButton(("Cluster " + std::to_string(i)).c_str(), &cluster, i);
            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNodeEx("Simplifying options", true)) {
            ImGui::InputDouble("epsilon", &epsilon);
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

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNodeEx("Smoothing options", true)) {
            ImGui::InputInt("k", &k);
            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNodeEx("Reconstructing options", true)) {
            ImGui::InputDouble("maximumFacetLength", &maximumFacetLength);
            ImGui::TreePop();
        }

        /*if (ImGui::Button("Simplify")) {
            simplifies[cluster] = origins[cluster].simplify(epsilon);
            simplified[cluster] = true;
        }
        if (ImGui::Button("Upsample") && simplified[cluster]) {
            upsamples[cluster] = simplifies[cluster].upsample(sharpnessAngle, edgeSensitivity, neighborRadius, size);
            upsampled[cluster] = true;
        }
        if (ImGui::Button("Smooth") && upsampled[cluster]) {
            smoothes[cluster] = upsamples[cluster].smooth(k);
            smoothed[cluster] = true;
        }
        if (ImGui::Button("Reconstruct") && smoothed[cluster]) {
            reconstructs[cluster] = smoothes[cluster].reconstruct(maximumFacetLength);
            reconstructed[cluster] = true;
        }*/

        Vector3D lightDirection(0.0f, 0.0f, -1.0f), cameraPosition(0.0f, 0.0f, 2.0f);
        Matrix4D modelMat, viewMat, projectionMat;
        modelMat = rotate * Matrix4D(factor);
        glm::mat4 modelMatT = glm::scale(rotateT, glm::vec3(factor));
        viewMat = Matrix4D::lookAt(cameraPosition, Vector3D(0.0f, 0.0f, 0.0f), Vector3D(0.0f, 1.0f, 0.0f));
        projectionMat = Matrix4D::perspective(PI / 4.0f, (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 0.1f, 100.0f);

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++)
                std::cout << rotate.values[i][j] << ' ';
            std::cout << std::endl;
        }
        std::cout << std::endl;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++)
                std::cout << rotateT[i][j] << ' ';
            std::cout << std::endl;
        }
        std::cout << std::endl;

        switch (color) {
        case 0:
            normalShader.use();
            normalShader.setMatrix4D("model", modelMat);
            normalShader.setMatrix4D("view", viewMat);
            normalShader.setMatrix4D("projection", projectionMat);
            normalShader.setVector3D("lightDirection", lightDirection);
            normalShader.setVector3D("cameraPosition", cameraPosition);
            for (int i = 0; i < numCluster; i++)
                /*if (display == 4 && reconstructed[i])
                    reconstructs[i].render();
                else if (display == 3 && smoothed[i])
                    smoothes[i].render();
                else if (display == 2 && upsampled[i])
                    upsamples[i].render();
                else if (display == 1 && simplified[i])
                    simplifies[i].render();
                else*/
                    origins[i].render();
            break;

        case 1:
            clusterShader.use();
            clusterShader.setMatrix4D("model", modelMat);
            clusterShader.setMatrix4D("view", viewMat);
            clusterShader.setMatrix4D("projection", projectionMat);
            clusterShader.setVector3D("lightDirection", lightDirection);
            clusterShader.setVector3D("cameraPosition", cameraPosition);
            for (int i = 0; i < numCluster; i++) {
                clusterShader.setVector3D("color", COLORS[i % COLOR_SIZE]);
                /*if (display == 4 && reconstructed[i])
                    reconstructs[i].render();
                else if (display == 3 && smoothed[i])
                    smoothes[i].render();
                else if (display == 2 && upsampled[i])
                    upsamples[i].render();
                else if (display == 1 && simplified[i])
                    simplifies[i].render();
                else*/
                    origins[i].render();
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

    //delete[] simplified;
    //delete[] upsampled;
    //delete[] smoothed;
    //delete[] reconstructed;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();

    return 0;
}
