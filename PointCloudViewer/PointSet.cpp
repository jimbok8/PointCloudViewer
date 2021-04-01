#include "PointSet.h"

PointSet::PointSet() {}

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
    //delete tree;
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

void PointSet::calculateNormals(int k) {
    k = std::min(k, (int)points.size());

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
    std::vector<bool> flag(points.size(), false);
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<>> heap;

    int seed = randomUniform(points.size());
    dist[seed] = 0.0f;
    heap.push(std::make_pair(dist[seed], seed));
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

float PointSet::averageSpacing(int k) const {
    k = std::min(k, (int)points.size());

    ANNidxArray indices = new ANNidx[k];
    ANNdistArray distances = new ANNdist[k];
    float ans = 0.0f;

    for (int i = 0; i < points.size(); i++) {
        ANNpoint point = pointArray[i];
        tree->annkSearch(pointArray[i], k, indices, distances);
        float sum = 0.0f;
        for (int j = 0; j < k; j++)
            sum += distances[j];
        ans += sum / (float)k;
    }

    delete[] indices;
    delete[] distances;

    return ans / (float)points.size();
}

std::vector<std::vector<int>> PointSet::calculateNeighbors(const std::vector<Point>& points, const float radius) const {
    ANNpointArray pointArray = annAllocPts(points.size(), 3);
    for (int i = 0; i < points.size(); i++)
        for (int j = 0; j < 3; j++)
            pointArray[i][j] = points[i].position(j);
    ANNkd_tree* tree = new ANNkd_tree(pointArray, points.size(), 3);

    float radius2 = radius * radius;
    std::vector<std::vector<int>> ans;
    for (int i = 0; i < points.size(); i++) {
        ans.push_back(std::vector<int>());

        int k = tree->annkFRSearch(pointArray[i], radius2, 0);
        ANNidxArray indices = new ANNidx[k];
        ANNdistArray distances = new ANNdist[k];
        tree->annkFRSearch(pointArray[i], radius2, k, indices, distances);

        for (int j = 0; j < k; j++)
            if (indices[j] != i)
                ans[i].push_back(indices[j]);
    }

    return ans;
}

void PointSet::selectBasePoint(const std::vector<Point>& points, const int index, const std::vector<int>& neighbors, const float edgeSensitivity, float& density2, int& baseIndex) const {
    if (neighbors.empty()) {
        density2 = 0.0f;
        baseIndex = index;
        return;
    }

    Point v = points[index];
    float bestDist2 = -FLT_MAX;
    for (int i = 0; i < neighbors.size(); i++) {
        Point t = points[neighbors[i]];
        Eigen::Vector3f midPoint = (v.position + t.position) * 0.5f;
        float dotProduce = std::pow(2.0f - v.normal.dot(t.normal), edgeSensitivity);
        Eigen::Vector3f diffT = midPoint - t.position;
        float projectionT = diffT.dot(t.normal);
        float minDist2 = diffT.squaredNorm() - projectionT * projectionT;

        for (int j = 0; j < neighbors.size(); j++) {
            Point s = points[neighbors[j]];
            Eigen::Vector3f diffS = midPoint - s.position;
            float projectionS = diffS.dot(s.normal);
            float dist2 = diffS.squaredNorm() - projectionS * projectionS;
            if (dist2 < minDist2)
                minDist2 = dist2;
        }

        minDist2 *= dotProduce;

        if (minDist2 > bestDist2) {
            bestDist2 = minDist2;
            baseIndex = neighbors[i];
        }
    }

    density2 = bestDist2;
}

void PointSet::updateNewPoint(std::vector<Point>& points, std::vector<std::vector<int>>& neighbors, const int newIndex, const int fatherIndex, const int motherIndex, const float radius, const float sharpnessAngle) const {
    // TODO
    Point v = points[newIndex];
    Point father = points[fatherIndex];
    Point mother = points[motherIndex];
    
    std::set<int> neighborIndices;
    for (const int neighbor : neighbors[fatherIndex])
        neighborIndices.insert(neighbor);
    for (const int neighbor : neighbors[motherIndex])
        neighborIndices.insert(neighbor);
    neighborIndices.insert(fatherIndex);
    neighborIndices.insert(motherIndex);

    float radius2 = radius * radius;
    neighbors.push_back(std::vector<int>());
    for (const int index : neighborIndices)
        if ((points[index].position - v.position).squaredNorm() > radius2)
            neighbors[newIndex].push_back(index);

    std::vector<Eigen::Vector3f> normalCandidates;
    normalCandidates.push_back(father.normal);
    normalCandidates.push_back(mother.normal);
    std::vector<float> projectDistSum(2, 0.0f);
    std::vector<Eigen::Vector3f> normalSum(2);
    std::vector<float> weightSum(2, 0.0f);
    float radius16 = -4.0f / radius2;
    for (int i = 0; i < neighbors[newIndex].size(); i++) {
        Point t = points[neighbors[newIndex][i]];
        float dist2 = (v.position - t.position).squaredNorm();
        float theta = std::exp(dist2 * radius16);

        for (int j = 0; j < 2; j++) {
            float psi = std::exp(-std::pow(1.0f - normalCandidates[j].dot(t.normal), 2) / sharpnessAngle);
            float projectDiff = (t.position - v.position).dot(t.normal);
            float weight = psi * theta;

            projectDistSum[j] += projectDiff * weight;
            normalSum[j] += t.normal * weight;
            weightSum[j] += weight;
        }
    }

    float minProjectDist = FLT_MAX;
    int best;
    for (int i = 0; i < 2; i++) {
        float absoluteDist = std::fabs(projectDistSum[i] / weightSum[i]);
        if (absoluteDist < minProjectDist) {
            minProjectDist = absoluteDist;
            best = i;
        }
    }

    Eigen::Vector3f updateNormal = normalSum[best] / weightSum[best];
    points[newIndex].normal = updateNormal / updateNormal.norm();
    float projectDist = projectDistSum[best] / weightSum[best];
    points[newIndex].position += points[newIndex].normal * projectDist;

    v = points[newIndex];
    neighbors[newIndex].clear();
    for (const int index : neighborIndices) {
        Point t = points[index];
        float dist2 = (v.position - t.position).squaredNorm();
        if (dist2 < radius2) {
            neighbors[newIndex].push_back(index);
            neighbors[index].push_back(newIndex);
        }
    }
}

PointSet PointSet::simplify(const float epsilon) const {
    float minX, minY, minZ;
    minX = minY = minZ = FLT_MAX;

    for (const Point& point : points) {
        minX = std::min(minX, point.position(0));
        minY = std::min(minY, point.position(1));
        minZ = std::min(minZ, point.position(2));
    }

    std::map<std::tuple<int, int, int>, std::vector<int>> map;
    for (int i = 0; i < points.size(); i++) {
        int x = (int)((points[i].position(0) - minX) / epsilon);
        int y = (int)((points[i].position(1) - minY) / epsilon);
        int z = (int)((points[i].position(2) - minZ) / epsilon);
        map[std::make_tuple(x, y, z)].push_back(i);
    }

    std::vector<Point> points;
    for (const std::pair<std::tuple<int, int, int>, std::vector<int>>& pair : map)
        points.push_back(this->points[pair.second[randomUniform(pair.second.size())]]);

    return PointSet(points);
}

PointSet PointSet::resample(float sharpnessAngle, float edgeSensitivity, float neighborRadius, int size) const {
    edgeSensitivity *= 10.0f;
    float spacing = averageSpacing();
    if (neighborRadius < spacing)
        neighborRadius = spacing * 3.0f;

    std::vector<Point> points(this->points);
    std::vector<std::vector<int>> neighbors = calculateNeighbors(points, neighborRadius);

    float currentRadius = neighborRadius;
    float densityPassThreshold = 0.0f;
    float sumDensity = 0.0f;
    int countDensity = 1;
    int maxIteration = 20;
    for (int iter = 0; iter < maxIteration; iter++) {
        if (iter > 0) {
            currentRadius *= 0.75f;
            if (currentRadius < densityPassThreshold * 3.0f)
                currentRadius = densityPassThreshold * 3.0f;
            neighbors = calculateNeighbors(points, currentRadius);
        }
        else {
            for (int i = 0; i < points.size() * 0.05f; i++) {
                if (neighbors[i].empty())
                    continue;

                float density2;
                int baseIndex;
                selectBasePoint(points, i, neighbors[i], edgeSensitivity, density2, baseIndex);

                if (density2 < 0.0f)
                    continue;

                sumDensity += density2;
                countDensity++;
            }
        }

        densityPassThreshold = std::sqrt(sumDensity / (float)countDensity) * 0.65f;
        float densityPassThreshold2 = densityPassThreshold * densityPassThreshold;
        sumDensity = 0.0f;
        countDensity = 1;
        int maxLoop = 3;
        std::vector<bool> isPassThreshold(points.size(), false);
        
        for (int loop = 0; loop < maxLoop; loop++) {
            int countNotPass = 0;
            for (int i = 0; i < points.size(); i++) {
                if (isPassThreshold[i])
                    continue;

                if (neighbors[i].empty())
                    continue;

                float density2;
                int baseIndex;
                selectBasePoint(points, i, neighbors[i], edgeSensitivity, density2, baseIndex);

                if (density2 < densityPassThreshold2) {
                    isPassThreshold[i] = true;
                    continue;
                }

                countNotPass++;
                sumDensity += density2;
                countDensity++;

                points.push_back(Point((points[i].position + points[baseIndex].position) * 0.5f));
                updateNewPoint(points, neighbors, points.size() - 1, i, baseIndex, currentRadius, sharpnessAngle);

                if (points.size() >= size)
                    break;
            }

            if (countNotPass == 0 || points.size() >= size)
                break;
        }

        if (points.size() >= size)
            break;
    }

    return PointSet(points);
}

void PointSet::render() const {
    glPointSize(5);
    glBindVertexArray(vao);
    glDrawArrays(GL_POINTS, 0, points.size());
    glBindVertexArray(0);
}