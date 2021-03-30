#include "PointSet.h"

PointSet::PointSet(const std::vector<Point>& points) :
    points(points) {
    init();
}

PointSet::PointSet(const PointSet& pointSet) :
    points(pointSet.points) {
    init();
}

PointSet::~PointSet() {
    annDeallocPts(pointArray);
    delete tree;
    annClose();
}

void PointSet::init() {
    pointArray = annAllocPts(points.size(), 3);
    for (int i = 0; i < points.size(); i++)
        for (int j = 0; j < 3; j++)
            pointArray[i][j] = points[i].position(j);

    tree = new ANNkd_tree(pointArray, points.size(), 3);

    calculateNormals();

    unsigned int vbo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, this->points.size() * sizeof(Point), this->points.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)offsetof(Point, normal));

    glBindVertexArray(0);
}

void PointSet::calculateNormals(const int k) {
    ANNidxArray indices = new ANNidx[k];
    ANNdistArray distances = new ANNdist[k];

    std::vector<std::vector<std::pair<int, float>>> graph(points.size());
    for (int i = 0; i < points.size(); i++) {
        tree->annkSearch(pointArray[i], k, indices, distances);

        Eigen::Vector3f avg(0.0f, 0.0f, 0.0f);
        for (int j = 0; j < k; j++) {
            avg += points[indices[j]].position;
            graph[i].push_back(std::make_pair(indices[j], distances[j]));
            graph[indices[j]].push_back(std::make_pair(i, distances[j]));
        }
        avg /= k;

        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (int j = 0; j < k; j++) {
            Eigen::Vector3f x = points[indices[j]].position - avg;
            cov += x * x.transpose();
        }
        cov /= k;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver;
        solver.compute(cov);
        points[i].normal = solver.eigenvectors().col(0);
    }

    std::vector<float> dist(points.size(), FLT_MAX);
    dist[0] = 0.0f;
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<>> heap;
    heap.push(std::make_pair(dist[0], 0));
    bool* flag = new bool[points.size()];
    memset(flag, false, sizeof(bool) * points.size());
    while (!heap.empty()) {
        int now = heap.top().second;
        heap.pop();
        if (flag[now])
            continue;
        flag[now] = true;
        for (const std::pair<int, float>& pair : graph[now]) {
            int next = pair.first;
            if (pair.second < dist[next]) {
                dist[next] = pair.second;
                heap.push(std::make_pair(dist[next], next));
                if (points[next].normal.dot(points[now].normal) < 0.0f)
                    points[next].normal = -points[next].normal;
            }
        }
    }

    delete[] indices;
    delete[] distances;
}

PointSet PointSet::simplify(float epsilon) {
    float minX, maxX, minY, maxY, minZ, maxZ;
    minX = minY = minZ = FLT_MAX;
    maxX = maxY = maxZ = -FLT_MAX;

    for (const Point& point : points) {
        minX = std::min(minX, point.position(0));
        maxX = std::max(maxX, point.position(0));
        minY = std::min(minY, point.position(1));
        maxY = std::max(maxY, point.position(1));
        minZ = std::min(minZ, point.position(2));
        maxZ = std::max(maxZ, point.position(2));
    }

    
}

void PointSet::render() const {
    glPointSize(5);
    glBindVertexArray(vao);
    glDrawArrays(GL_POINTS, 0, points.size());
    glBindVertexArray(0);
}