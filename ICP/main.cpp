#include <numeric>
#include <ctime>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <ANN/ANN.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_opengl3.h>

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/registration/icp.h>

#include "CPoint.h"
#include "CPointSet.h"
#include "CShader.h"
#include "TransformHelper.h"

const int MAX_ITERATION = 50;
const float EPSILON = 1e-8;
const float DISTANCE_THRESHOLD = 0.01f;
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

//void readPointCloud(const std::string& path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
//    std::ifstream fin(path);
//    std::string s;
//    while (fin >> s) {
//        if (s == "v") {
//            float x, y, z, weight;
//            int cluster;
//            fin >> x >> y >> z >> cluster >> weight;
//            cloud->push_back(pcl::PointXYZ(x, y, z));
//        }
//        getline(fin, s);
//    }
//}

void readPoints(const std::string& path, std::vector<CPoint>& points) {
    std::ifstream fin(path);
    std::string s;
    while (fin >> s) {
        if (s == "v") {
            float x, y, z, weight;
            int cluster;
            fin >> x >> y >> z >> cluster >> weight;
            points.push_back(CPoint(Eigen::Vector3f(x, y, z)));
        }
        getline(fin, s);
    }

    ANNpointArray pointArray;
    ANNkd_tree* tree;
    pointArray = annAllocPts(points.size(), 3);
    for (int i = 0; i < points.size(); i++)
        for (int j = 0; j < 3; j++)
            pointArray[i][j] = points[i].m_position(j);
    tree = new ANNkd_tree(pointArray, points.size(), 3);

    int k = 25;
    for (int i = 0; i < points.size(); i++) {
        ANNidxArray indices = new ANNidx[k];
        ANNdistArray distances = new ANNdist[k];
        tree->annkSearch(pointArray[i], k, indices, distances);

        Eigen::Vector3f avg(0.0f, 0.0f, 0.0f);
        for (int j = 0; j < k; j++)
            avg += points[indices[j]].m_position;
        avg /= (float)k;

        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (int j = 0; j < k; j++) {
            Eigen::Vector3f x = points[indices[j]].m_position - avg;
            cov += x * x.transpose();
        }
        cov /= (float)k;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver;
        solver.compute(cov);
        points[i].m_normal = solver.eigenvectors().col(0);
    }

    annDeallocPts(pointArray);
    delete tree;
}

Eigen::Vector3f calculateCentroid(const std::vector<CPoint>& points) {
    Eigen::Vector3f ans(0.0f, 0.0f, 0.0f);
    for (const CPoint& point : points)
        ans += point.m_position;

    return ans / (float)points.size();
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

    /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>());

    readPointCloud("../data/generated1.dat", cloudIn);
    readPointCloud("../data/generated2.dat", cloudOut);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloudIn);
    icp.setInputTarget(cloudOut);
    
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1);

    pcl::PointCloud<pcl::PointXYZ> final;
    icp.align(final);

    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    std::ofstream fout("../data/aligned.dat");
    for (pcl::PointXYZ& point : final)
        fout << "v " << point.x << ' ' << point.y << ' ' << point.z << " 0 0" << std::endl;

    for (pcl::PointXYZ& point : *cloudIn)
        fout << "v " << point.x << ' ' << point.y << ' ' << point.z << " 1 0" << std::endl;

    for (pcl::PointXYZ& point : *cloudOut)
        fout << "v " << point.x << ' ' << point.y << ' ' << point.z << " 2 0" << std::endl;*/

    std::vector<CPoint> source, target;
    readPoints("../data/bunny1.dat", source);
    readPoints("../data/bunny2.dat", target);

    ANNpointArray pointArray;
    ANNkd_tree* tree;
    pointArray = annAllocPts(target.size(), 3);
    for (int i = 0; i < target.size(); i++)
        for (int j = 0; j < 3; j++)
            pointArray[i][j] = target[i].m_position(j);
    tree = new ANNkd_tree(pointArray, target.size(), 3);

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t = Eigen::Vector3f::Zero();
    std::vector<CPointSet> sets;
    for (int iter = 0; iter < MAX_ITERATION; iter++) {
        std::vector<CPoint> current;
        for (const CPoint& pointTemp : source)
            current.push_back(CPoint(R * pointTemp.m_position + t, R * pointTemp.m_normal));

        Eigen::Vector3f centroidCurrent = calculateCentroid(current);
        Eigen::Vector3f centroidTarget = calculateCentroid(target);

        /*Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        float loss = 0.0f;
        int num = 0;*/

        Eigen::MatrixXf A(6, 6);
        Eigen::VectorXf b(6);
        A.setZero();
        b.setZero();

        std::vector<int> indicesTemp;

        ANNidxArray indices = new ANNidx[1];
        ANNdistArray distances = new ANNdist[1];
        for (const CPoint& point : current) {
            ANNpoint pointTemp = annAllocPt(3);
            for (int j = 0; j < 3; j++)
                pointTemp[j] = point.m_position(j);
            
            tree->annkSearch(pointTemp, 1, indices, distances);
            if (distances[0] < DISTANCE_THRESHOLD) {
                /*H += (point.m_position - centroidCurrent) * (target[indices[0]].m_position - centroidTarget).transpose();
                loss += (point.m_position - target[indices[0]].m_position).squaredNorm();
                num++;*/

                Eigen::Vector3f p = point.m_position;
                Eigen::Vector3f q = target[indices[0]].m_position;
                Eigen::Vector3f n = target[indices[0]].m_normal;
                Eigen::Vector3f c = p.cross(n);

                Eigen::MatrixXf At(6, 6);
                At.block(0, 0, 3, 3) = c * c.transpose();
                At.block(0, 3, 3, 3) = c * n.transpose();
                At.block(3, 0, 3, 3) = c * n.transpose();
                At.block(3, 3, 3, 3) = n * n.transpose();

                Eigen::VectorXf bt(6);
                bt.block(0, 0, 3, 1) = c;
                bt.block(3, 0, 3, 1) = n;
                bt *= (p - q).dot(n);

                A += At;
                b -= bt;

                indicesTemp.push_back(indices[0]);
            }
            else
                indicesTemp.push_back(-1);

            annDeallocPt(pointTemp);
        }
        delete[] indices;
        delete[] distances;

        sets.push_back(CPointSet(current, target, indicesTemp));

        //std::cout << loss / (float)num << std::endl;

        //Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
        //R = svd.matrixV() * svd.matrixU().transpose();
        //t = centroidTarget - R * centroidCurrent;

        Eigen::LLT<Eigen::MatrixXf> cholesky;
        cholesky.compute(A);
        Eigen::VectorXf x = cholesky.solve(b);

        Eigen::AngleAxisf rotationVectorX(x(0), Eigen::Vector3f(1, 0, 0));
        Eigen::Matrix3f rotationMatrixX = rotationVectorX.toRotationMatrix();

        Eigen::AngleAxisf rotationVectorY(x(1), Eigen::Vector3f(0, 1, 0));
        Eigen::Matrix3f rotationMatrixY = rotationVectorY.toRotationMatrix();

        Eigen::AngleAxisf rotationVectorZ(x(2), Eigen::Vector3f(0, 0, 1));
        Eigen::Matrix3f rotationMatrixZ = rotationVectorZ.toRotationMatrix();

        R *= rotationMatrixX * rotationMatrixY * rotationMatrixZ;
        t += x.block(3, 0, 3, 1);
    }

    annDeallocPts(pointArray);
    delete tree;

    std::cout << R << std::endl << t << std::endl;

    std::ofstream fout("../data/aligned.dat");
    for (const CPoint& pointTemp : source) {
        Eigen::Vector3f point = R * pointTemp.m_position + t;
        fout << "v " << point(0) << ' ' << point(1) << ' ' << point(2) << " 0 0" << std::endl;
    }

    for (const CPoint& pointTemp : source) {
        Eigen::Vector3f point = pointTemp.m_position;
        fout << "v " << point(0) << ' ' << point(1) << ' ' << point(2) << " 1 0" << std::endl;
    }

    for (const CPoint& pointTemp : target) {
        Eigen::Vector3f point = pointTemp.m_position;
        fout << "v " << point(0) << ' ' << point(1) << ' ' << point(2) << " 2 0" << std::endl;
    }

    CShader shader("shader/ClusterVertex.glsl", "shader/ClusterFragment.glsl");
    int frame = 0;
    bool play = false, flag = true;
    clock_t last;

    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Options");
        
        ImGui::Checkbox("Display Lines", &flag);
        ImGui::InputInt("Frame", &frame);
        if (!play && ImGui::Button("Play")) {
            play = true;
            last = clock();
        } 
        else if (play && ImGui::Button("Pause"))
            play = false;

        if (play && (clock() - last >= 1000)) {
            frame = (frame + 1) % sets.size();
            last = clock();
        }

        Eigen::Vector3f lightDirection(0.0f, 0.0f, -1.0f), cameraPosition(0.0f, 0.0f, 2.0f);
        Eigen::Matrix4f modelMat, viewMat, projectionMat;
        modelMat = g_rotation * scale(g_factor);
        viewMat = lookAt(cameraPosition, Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
        projectionMat = perspective(PI / 4.0f, (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 0.1f, 100.0f);

        shader.use();
        shader.setMatrix4D("model", modelMat);
        shader.setMatrix4D("view", viewMat);
        shader.setMatrix4D("projection", projectionMat);
        shader.setVector3D("lightDirection", lightDirection);
        shader.setVector3D("cameraPosition", cameraPosition);

        sets[frame].render(flag);

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