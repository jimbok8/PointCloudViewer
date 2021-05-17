#include <climits>
#include <cfloat>
#include <string>
#include <random>
#include <iostream>
#include <fstream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_opengl3.h>

#include "ConfigHelper.h"
#include "Vertex.h"
#include "Mesh.h"
#include "PointSet.h"
#include "Triangle.h"
#include "BVH.h"
#include "Shader.h"

int lastX = INT_MIN, lastY = INT_MIN;
float factor = 1.0f;
bool press;
glm::mat4 rotate(1.0f);

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

PointSet generate(const Mesh& mesh, const glm::mat4& model, const glm::vec3& camera, const float sigma) {
    std::vector<Vertex> vertices = mesh.getVertices();
    std::vector<unsigned int> indices = mesh.getIndices();
    std::vector<Triangle> triangles;

    for (auto iter = indices.begin(); iter != indices.end(); ) {
        glm::vec3 p0 = vertices[*(iter++)].position;
        glm::vec3 p1 = vertices[*(iter++)].position;
        glm::vec3 p2 = vertices[*(iter++)].position;

        p0 = glm::vec3(model * glm::vec4(p0, 1.0f));
        p1 = glm::vec3(model * glm::vec4(p1, 1.0f));
        p2 = glm::vec3(model * glm::vec4(p2, 1.0f));

        triangles.push_back(Triangle(p0, p1, p2));
    }

    BVH bvh(triangles);

    glm::vec3 f = glm::normalize(-camera);
    glm::vec3 u(0.0f, 1.0f, 0.0f);
    glm::vec3 l = glm::cross(u, f);
    float scale = 2.0f * std::tan(PI / 8.0f) / (float)WINDOW_HEIGHT;
    glm::vec3 du = u * scale;
    glm::vec3 dl = l * scale;
    glm::vec3 o = camera + f + (du * (float)WINDOW_HEIGHT + dl * (float)WINDOW_WIDTH) * 0.5f;
    std::default_random_engine engine;
    std::normal_distribution<float> distribution(0.0f, sigma);
    vertices.clear();

    for (int i = 0; i < WINDOW_WIDTH; i++)
        for (int j = 0; j < WINDOW_HEIGHT; j++)
            if (i % 4 == 0 && j % 4 == 0) {
                glm::vec3 direction = glm::normalize(o - dl * ((float)i + 0.5f) - du * ((float)j + 0.5f) - camera);
                Ray ray(camera, direction);
                float t = bvh.trace(ray);
                if (t < FLT_MAX) {
                    glm::vec3 point = ray.point(t);
                    vertices.push_back(Vertex(glm::vec3(point.x + distribution(engine), point.y + distribution(engine), point.z + distribution(engine))));
                }
            }

    return PointSet(vertices);
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

    Shader shader("shader/NormalVertex.glsl", "shader/NormalFragment.glsl");
    Mesh mesh("model/bunny.obj");
    PointSet pointSet;

    int display = 0;
    float lightPower = 30.0f, sigma = 1.0f;
    glm::vec3 lightPosition(3.0f, 3.0f, 3.0f), cameraPosition(0.0f, 0.0f, 2.0f);
    char buf[128];

    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Options");

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNodeEx("Displaying options", true)) {
            ImGui::RadioButton("Display mesh", &display, 0);
            ImGui::RadioButton("Display generated point cloud", &display, 1);
            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNodeEx("Light options", true)) {
            ImGui::SliderFloat("Light power", &lightPower, 0.0f, 50.0f);
            ImGui::SliderFloat("Light position x", &lightPosition.x, -10.0f, 10.0f);
            ImGui::SliderFloat("Light position y", &lightPosition.y, -10.0f, 10.0f);
            ImGui::SliderFloat("Light position z", &lightPosition.z, -10.0f, 10.0f);
            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNodeEx("Camera options", true)) {
            ImGui::SliderFloat("Camera position x", &cameraPosition.x, -10.0f, 10.0f);
            ImGui::SliderFloat("Camera position y", &cameraPosition.y, -10.0f, 10.0f);
            ImGui::SliderFloat("Camera position z", &cameraPosition.z, -10.0f, 10.0f);
            ImGui::TreePop();
        }
        
        glm::mat4 modelMat, viewMat, projectionMat;
        modelMat = glm::scale(rotate, glm::vec3(factor));
        viewMat = glm::lookAt(cameraPosition, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        projectionMat = glm::perspective(PI / 4.0f, (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 0.1f, 100.0f);

        shader.use();
        shader.setMat4("model", modelMat);
        shader.setMat4("view", viewMat);
        shader.setMat4("projection", projectionMat);
        shader.setFloat("lightPower", lightPower);
        shader.setVec3("lightPosition", lightPosition);
        shader.setVec3("cameraPosition", cameraPosition);

        switch (display) {
        case 0:
            mesh.render();
            break;

        case 1:
            pointSet.render();
            break;

        default:
            break;
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNodeEx("Generate options", true)) {
            ImGui::SliderFloat("Sigma", &sigma, 0.0f, 5.0f);
            ImGui::InputText("File name", buf, 128);
            if (ImGui::Button("Generate"))
                pointSet = generate(mesh, modelMat, cameraPosition, sigma);
            if (ImGui::Button("Save")) {
                std::ofstream fout("../data/" + std::string(buf));
                std::vector<Vertex> vertices = pointSet.getVertices();

                for (const Vertex& vertex : vertices)
                    fout << "v " << vertex.position.x << ' ' << vertex.position.y << ' ' << vertex.position.z << " 0 0" << std::endl;
            }
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