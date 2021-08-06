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

    //CPointSet* last = new CPointSet("../data/qjhdl/hd.dat");
    CPointSet* last = new CPointSet("../data/qjhdl/wrmlh/0thFrame.pcd");

    CSimplifyParameter simplifyParameter(last->averageSpacing() * 4.0f);
    CSmoothParameter smoothParameter(64, 30.0f);
    CPointSet* simplified = last->simplify(simplifyParameter, true);
    CPointSet* smoothed = simplified->smooth(smoothParameter, true);
    delete last;
    delete simplified;
    last = smoothed;

    for (int i = 1; i < 424; i++) {
        CPointSet* current = new CPointSet("../data/qjhdl/wrmlh/" + std::to_string(i) + "thFrame.pcd");
        simplifyParameter.m_epsilon = current->averageSpacing() * 4.0f;
        CPointSet* simplified = current->simplify(simplifyParameter, true);
        CPointSet* smoothed = simplified->smooth(smoothParameter, true);
        CPointSet* combined = last->combine(smoothed, false);
        delete last;
        delete current;
        delete simplified;
        delete smoothed;
        last = combined;
    }

    last = last->simplify(simplifyParameter, false)->smooth(smoothParameter, false);
    last->save("../data/qjhdl/wrmlh.dat");

    g_factor = 1.0f / last->scale();
    g_factorDiff = 0.1f * g_factor;

    //CPointSet* result = last->simplify(simplifyParameter);
    //Eigen::Matrix3f rotate = Eigen::Matrix3f::Identity();
    //Eigen::Vector3f translate = Eigen::Vector3f::Zero();

    //for (int i = 1; i < 5; i++) {
    //    CPointSet* current = new CPointSet("../data/qjhdl/hd/" + std::to_string(i) + "thFrame.pcd");
    //    
    //    Eigen::Matrix3f rotateTemp;
    //    Eigen::Vector3f translateTemp;
    //    current->registrate(last, rotateTemp, translateTemp);
    //    rotate *= rotateTemp;
    //    translate += translateTemp;

    //    std::cout << rotate << std::endl << translate << std::endl << std::endl;

    //    result->combine(current, rotate, translate);
    //    CPointSet* newResult = result->simplify(simplifyParameter);

    //    delete last;
    //    last = current;
    //    delete result;
    //    result = newResult;
    //}
    
    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Options");

        if (ImGui::Button("Reset")) {
            g_factor = 1.0f / last->scale();
            g_rotation = g_translate = Eigen::Matrix4f::Identity();
        }

        //ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        //if (ImGui::TreeNodeEx("Displaying options", true)) {
        //    ImGui::RadioButton("Display original point cloud", &display, 0);
        //    ImGui::RadioButton("Display simplified point cloud", &display, 1);
        //    ImGui::RadioButton("Display resampled point cloud", &display, 2);
        //    ImGui::RadioButton("Display smoothed point cloud", &display, 3);
        //    ImGui::RadioButton("Display reconstructed mesh", &display, 4);
        //    ImGui::NewLine();
        //    ImGui::RadioButton("Color by normal", &color, 0);
        //    ImGui::RadioButton("Color by cluster", &color, 1);
        //    ImGui::TreePop();
        //}

        //ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        //if (ImGui::TreeNodeEx("Cluster options", true)) {
        //    for (int i = 0; i < numCluster; i++)
        //        ImGui::RadioButton(("Cluster " + std::to_string(i)).c_str(), &cluster, i);
        //    ImGui::TreePop();
        //}

        //ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        //if (ImGui::TreeNodeEx("Simplifying options", true)) {
        //    ImGui::InputFloat("epsilon", &simplifyParameter.m_epsilon);
        //    ImGui::TreePop();
        //}

        //ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        //if (ImGui::TreeNodeEx("Upsampling options", true)) {
        //    ImGui::InputFloat("sharpnessAngle", &resampleParameter.m_sharpnessAngle);
        //    ImGui::InputFloat("edgeSensitivity", &resampleParameter.m_edgeSensitivity);
        //    ImGui::InputFloat("neighborRadius", &resampleParameter.m_neighborRadius);
        //    ImGui::InputInt("size", &resampleParameter.m_size);
        //    ImGui::TreePop();
        //}

        //ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        //if (ImGui::TreeNodeEx("Smoothing options", true)) {
        //    ImGui::InputInt("k", &smoothParameter.m_k);
        //    ImGui::InputFloat("sharpnessAngle", &smoothParameter.m_sharpnessAngle);
        //    ImGui::TreePop();
        //}

        //ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        //if (ImGui::TreeNodeEx("Reconstructing options", true)) {
        //    ImGui::InputInt("numberIteration", &reconstructParameter.m_iterationNumber);
        //    ImGui::InputFloat("maximumFacetLength", &reconstructParameter.m_maximumFacetLength);
        //    ImGui::InputFloat("radius", &reconstructParameter.m_radius);
        //    ImGui::InputFloat("epsilon", &reconstructParameter.m_epsilon);
        //    ImGui::TreePop();
        //}

        //if (ImGui::Button("Simplify"))
        //    simplifies[cluster] = origins[cluster]->simplify(simplifyParameter);
        //if (ImGui::Button("Resample") && simplifies[cluster] != nullptr)
        //    resamples[cluster] = simplifies[cluster]->resample(resampleParameter);
        //if (ImGui::Button("Smooth") && simplifies[cluster] != nullptr)
        //    smoothes[cluster] = simplifies[cluster]->smooth(smoothParameter);
        //if (ImGui::Button("Reconstruct") && smoothes[cluster] != nullptr)
        //    reconstructs[cluster] = smoothes[cluster]->reconstruct(reconstructParameter);

        Eigen::Vector3f lightDirection(0.0f, 0.0f, -1.0f), cameraPosition(0.0f, 0.0f, 2.0f);
        Eigen::Matrix4f modelMat, viewMat, projectionMat;
        modelMat = g_translate * g_rotation * scale(g_factor);
        viewMat = lookAt(cameraPosition, Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
        projectionMat = perspective(PI / 4.0f, (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 0.1f, 100.0f);

        normalShader.use();
        normalShader.setMatrix4D("model", modelMat);
        normalShader.setMatrix4D("view", viewMat);
        normalShader.setMatrix4D("projection", projectionMat);
        normalShader.setVector3D("lightDirection", lightDirection);
        normalShader.setVector3D("cameraPosition", cameraPosition);
        
        last->render();
        //result->render();

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