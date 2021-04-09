#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <bitset>

#include <Eigen/Dense>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_opengl3.h>

#include "UtilsHelper.h"
#include "CPoint.h"
#include "CPointSet.h"
#include "CMesh.h"
#include "CShader.h"
#include "CSimplifyParameter.h"
#include "CResampleParameter.h"

const unsigned int WINDOW_WIDTH = 1920;
const unsigned int WINDOW_HEIGHT = 1080;
const float PI = std::acos(-1.0f);
const Eigen::Vector3f COLORS[] = {
    Eigen::Vector3f(1.0f, 0.0f, 0.0f),
    Eigen::Vector3f(0.0f, 1.0f, 0.0f),
    Eigen::Vector3f(0.0f, 0.0f, 1.0f),
    Eigen::Vector3f(0.0f, 1.0f, 1.0f),
    Eigen::Vector3f(1.0f, 0.0f, 1.0f),
    Eigen::Vector3f(1.0f, 1.0f, 0.0f)
};
const int COLOR_SIZE = sizeof(COLORS) / sizeof(Eigen::Vector3f);

static int g_lastX = INT_MIN, g_lastY = INT_MIN;
static float g_factor = 1.0f;
static bool g_press = false;
static Eigen::Matrix4f g_rotation = Eigen::Matrix4f::Identity();

static void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        g_press = true;
        g_lastX = g_lastY = INT_MIN;
    }
    else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
        g_press = false;
}

static void cursorPosCallback(GLFWwindow* window, double x, double y) {
    int newX = (int)x, newY = (int)y;
    if (g_press && g_lastX != INT_MIN && g_lastY != INT_MIN && (newX != g_lastX || newY != g_lastY)) {
        Eigen::Vector3f a = Eigen::Vector3f((float)g_lastX / WINDOW_WIDTH - 0.5f, 0.5f - (float)g_lastY / WINDOW_HEIGHT, 1.0f).normalized();
        Eigen::Vector3f b = Eigen::Vector3f((float)newX / WINDOW_WIDTH - 0.5f, 0.5f - (float)newY / WINDOW_HEIGHT, 1.0f).normalized();
        Eigen::Vector3f axis = a.cross(b);
        float angle = a.dot(b);
        g_rotation = rotate(axis, 10.0f * std::acos(angle)) * g_rotation;
    }

    g_lastX = newX;
    g_lastY = newY;
}

static void scrollCallback(GLFWwindow* window, double x, double y) {
    g_factor += 0.01f * (float)y;
    g_factor = std::max(g_factor, 0.01f);
}

static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
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

    CShader normalShader("shader/NormalVertex.glsl", "shader/NormalFragment.glsl");
    CShader clusterShader("shader/ClusterVertex.glsl", "shader/ClusterFragment.glsl");

    std::vector<CPoint> points;
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

            minX = std::min(minX, x);
            maxX = std::max(maxX, x);
            minY = std::min(minY, y);
            maxY = std::max(maxY, y);
            minZ = std::min(minZ, z);
            maxZ = std::max(maxZ, z);

            points.push_back(CPoint(Eigen::Vector3f(x, y, z)));
            clusters.push_back(cluster);
        }
        getline(fin, s);
    }

    int numCluster = *std::max_element(clusters.begin(), clusters.end()) + 1;
    Eigen::Vector3f center((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2);
    std::vector<std::vector<CPoint>> vertices(numCluster);
    for (int i = 0; i < points.size(); i++) {
        points[i].m_position = points[i].m_position - center;
        vertices[clusters[i]].push_back(points[i]);
    }

    std::vector<CPointSet*> origins;
    for (int i = 0; i < numCluster; i++)
        origins.push_back(new CPointSet(vertices[i]));
    std::vector<CPointSet*> simplifies(numCluster, nullptr);
    std::vector<CPointSet*> resamples(numCluster, nullptr);
    //std::vector<CPointSet*> smoothes(numCluster, nullptr);
    //std::vector<CMesh*> reconstructs(numCluster, nullptr);

    int display = 0, color = 0, cluster = 0;
    CSimplifyParameter simplifyParameter(0.3f);
    CResampleParameter resampleParameter(25.0f, 0.0f, 3.0f, 10000);

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
            ImGui::InputFloat("epsilon", &simplifyParameter.m_epsilon);
            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNodeEx("Upsampling options", true)) {
            ImGui::InputFloat("sharpnessAngle", &resampleParameter.m_sharpnessAngle);
            ImGui::InputFloat("edgeSensitivity", &resampleParameter.m_edgeSensitivity);
            ImGui::InputFloat("neighborRadius", &resampleParameter.m_neighborRadius);
            ImGui::InputInt("size", &resampleParameter.m_size);
            ImGui::TreePop();
        }

        //ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        //if (ImGui::TreeNodeEx("Smoothing options", true)) {
        //    ImGui::InputInt("k", &k);
        //    ImGui::TreePop();
        //}

        //ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        //if (ImGui::TreeNodeEx("Reconstructing options", true)) {
        //    ImGui::InputFloat("maximumFacetLength", &maximumFacetLength);
        //    ImGui::TreePop();
        //}

        if (ImGui::Button("Simplify"))
            simplifies[cluster] = origins[cluster]->simplify(simplifyParameter);
        if (ImGui::Button("Upsample") && simplifies[cluster] != nullptr)
            resamples[cluster] = simplifies[cluster]->resample(resampleParameter);
        /*if (ImGui::Button("Smooth") && resamples[cluster] != nullptr)
            smoothes[cluster] = resamples[cluster]->smooth(k);
        if (ImGui::Button("Reconstruct") && smoothes[cluster] != nullptr)
            reconstructs[cluster] = smoothes[cluster]->reconstruct(maximumFacetLength);*/

        Eigen::Vector3f lightDirection(0.0f, 0.0f, -1.0f), cameraPosition(0.0f, 0.0f, 2.0f);
        Eigen::Matrix4f modelMat, viewMat, projectionMat;
        modelMat = g_rotation * scale(g_factor);
        viewMat = lookAt(cameraPosition, Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
        projectionMat = perspective(PI / 4.0f, (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 0.1f, 100.0f);

        switch (color) {
        case 0:
            normalShader.use();
            normalShader.setMatrix4D("model", modelMat);
            normalShader.setMatrix4D("view", viewMat);
            normalShader.setMatrix4D("projection", projectionMat);
            normalShader.setVector3D("lightDirection", lightDirection);
            normalShader.setVector3D("cameraPosition", cameraPosition);
            for (int i = 0; i < numCluster; i++)
                /*if (display == 4 && reconstructs[i] != nullptr)
                    reconstructs[i]->render();
                else if (display == 3 && smoothes[i] != nullptr)
                    smoothes[i]->render();
                else */if (display == 2 && resamples[i] != nullptr)
                    resamples[i]->render();
                else if (display == 1 && simplifies[i] != nullptr)
                    simplifies[i]->render();
                else
                    origins[i]->render();
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
                /*if (display == 4 && reconstructs[i] != nullptr)
                    reconstructs[i]->render();
                else if (display == 3 && smoothes[i] != nullptr)
                    smoothes[i]->render();
                else */if (display == 2 && resamples[i] != nullptr)
                    resamples[i]->render();
                else if (display == 1 && simplifies[i] != nullptr)
                    simplifies[i]->render();
                else
                    origins[i]->render();
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
