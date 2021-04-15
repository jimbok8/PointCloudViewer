#include "CPointSet.h"

CPointSet::CPointSet(const std::vector<CPoint>& points) :
    m_points(points) {
    m_pointArray = annAllocPts(points.size(), 3);
    for (int i = 0; i < points.size(); i++)
        for (int j = 0; j < 3; j++)
            m_pointArray[i][j] = points[i].m_position(j);
    m_tree = new ANNkd_tree(m_pointArray, points.size(), 3);

    calculateNormals();

    unsigned int vbo;
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, m_points.size() * sizeof(CPoint), m_points.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(CPoint), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(CPoint), (void*)offsetof(CPoint, m_normal));

    glBindVertexArray(0);
}

CPointSet::~CPointSet() {
    annDeallocPts(m_pointArray);
    delete m_tree;
}

std::vector<std::vector<int>> CPointSet::calculateKNeighbors(int k) const {
    k = std::min(k + 1, (int)m_points.size());

    ANNidxArray indices = new ANNidx[k];
    ANNdistArray distances = new ANNdist[k];
    std::vector<std::vector<int>> ans;
    for (int i = 0; i < m_points.size(); i++) {
        ans.push_back(std::vector<int>());
        m_tree->annkSearch(m_pointArray[i], k, indices, distances);
        for (int j = 0; j < k; j++)
            ans[i].push_back(indices[j]);
    }
    delete[] indices;
    delete[] distances;

    return ans;
}

void CPointSet::calculateNormals(const int k) {
    std::vector<std::vector<int>> neighbors = calculateKNeighbors(k);
    std::vector<std::vector<std::pair<int, float>>> graph(m_points.size());
    for (int i = 0; i < m_points.size(); i++) {
        Eigen::Vector3f avg(0.0f, 0.0f, 0.0f);
        for (const int neighbor : neighbors[i]) {
            avg += m_points[neighbor].m_position;
            float dist2 = (m_points[i].m_position - m_points[neighbor].m_position).squaredNorm();
            graph[i].push_back(std::make_pair(neighbor, dist2));
            graph[neighbor].push_back(std::make_pair(i, dist2));
        }
        avg /= (float)neighbors[i].size();

        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (const int neighbor : neighbors[i]) {
            Eigen::Vector3f x = m_points[neighbor].m_position - avg;
            cov += x * x.transpose();
        }
        cov /= (float)neighbors[i].size();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver;
        solver.compute(cov);
        m_points[i].m_normal = solver.eigenvectors().col(0);
    }

    std::vector<float> dist(m_points.size(), FLT_MAX);
    std::vector<bool> flag(m_points.size(), false);
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<>> heap;
    dist[0] = 0.0f;
    heap.push(std::make_pair(dist[0], 0));
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
                if (m_points[next].m_normal.dot(m_points[now].m_normal) < 0.0f)
                    m_points[next].m_normal = -m_points[next].m_normal;
            }
        }
    }
}

float CPointSet::averageSpacing(const int k) const {
    std::vector<std::vector<int>> neighbors = calculateKNeighbors(k);
    float ans = 0.0f;
    for (int i = 0; i < m_points.size(); i++) {
        float sum = 0.0f;
        for (const int neighbor : neighbors[i])
            sum += (m_points[i].m_position - m_points[neighbor].m_position).norm();
        ans += sum / (float)neighbors[i].size();
    }
    ans /= (float)m_points.size();

    return ans;
}

std::vector<std::vector<int>> CPointSet::calculateRadiusNeighbors(const std::vector<CPoint>& points, const float radius) const {
    ANNpointArray pointArray = annAllocPts(points.size(), 3);
    for (int i = 0; i < points.size(); i++)
        for (int j = 0; j < 3; j++)
            pointArray[i][j] = points[i].m_position(j);
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

        delete[] indices;
        delete[] distances;
    }

    annDeallocPts(pointArray);
    delete tree;

    return ans;
}

void CPointSet::selectBasePoint(const std::vector<CPoint>& points, const int index, const std::vector<int>& neighbors, const float edgeSensitivity, float& density2, int& baseIndex) const {
    if (neighbors.empty()) {
        density2 = 0.0f;
        baseIndex = index;
        return;
    }

    CPoint v = points[index];
    float bestDist2 = -FLT_MAX;
    for (int i = 0; i < neighbors.size(); i++) {
        CPoint t = points[neighbors[i]];
        Eigen::Vector3f midPoint = (v.m_position + t.m_position) * 0.5f;
        float dotProduct = std::pow(2.0f - v.m_normal.dot(t.m_normal), edgeSensitivity);
        Eigen::Vector3f diffT = midPoint - t.m_position;
        float projectT = diffT.dot(t.m_normal);
        float minDist2 = diffT.squaredNorm() - projectT * projectT;

        for (int j = 0; j < neighbors.size(); j++) {
            CPoint s = points[neighbors[j]];
            Eigen::Vector3f diffS = midPoint - s.m_position;
            float projectS = diffS.dot(s.m_normal);
            float dist2 = diffS.squaredNorm() - projectS * projectS;
            if (dist2 < minDist2)
                minDist2 = dist2;
        }
        minDist2 *= dotProduct;

        if (minDist2 > bestDist2) {
            bestDist2 = minDist2;
            baseIndex = neighbors[i];
        }
    }

    density2 = bestDist2;
}

void CPointSet::updateNewPoint(std::vector<CPoint>& points, std::vector<std::vector<int>>& neighbors, const int newIndex, const int fatherIndex, const int motherIndex, const float radius, const float sharpnessAngle) const {
    CPoint v = points[newIndex];
    CPoint father = points[fatherIndex];
    CPoint mother = points[motherIndex];
    
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
        if ((points[index].m_position - v.m_position).squaredNorm() < radius2)
            neighbors[newIndex].push_back(index);

    std::vector<Eigen::Vector3f> normalCandidates;
    normalCandidates.push_back(father.m_normal);
    normalCandidates.push_back(mother.m_normal);
    std::vector<float> projectDistSum(2, 0.0f);
    std::vector<Eigen::Vector3f> normalSum(2, Eigen::Vector3f::Zero());
    std::vector<float> weightSum(2, 0.0f);
    float radius16 = -4.0f / radius2;
    for (const int neighbor : neighbors[newIndex]) {
        CPoint t = points[neighbor];
        float dist2 = (v.m_position - t.m_position).squaredNorm();
        float theta = std::exp(dist2 * radius16);

        for (int i = 0; i < 2; i++) {
            float psi = std::exp(-std::pow(1.0f - normalCandidates[i].dot(t.m_normal), 2) / sharpnessAngle);
            float projectDiff = (t.m_position - v.m_position).dot(t.m_normal);
            float weight = psi * theta;

            projectDistSum[i] += projectDiff * weight;
            normalSum[i] += t.m_normal * weight;
            weightSum[i] += weight;
        }
    }

    float minProjectDist = FLT_MAX;
    int best = 0;
    for (int i = 0; i < 2; i++) {
        float absoluteDist = std::fabs(projectDistSum[i] / weightSum[i]);
        if (absoluteDist < minProjectDist) {
            minProjectDist = absoluteDist;
            best = i;
        }
    }

    Eigen::Vector3f updateNormal = normalSum[best] / weightSum[best];
    points[newIndex].m_normal = updateNormal / updateNormal.norm();
    float projectDist = projectDistSum[best] / weightSum[best];
    points[newIndex].m_position += points[newIndex].m_normal * projectDist;

    v = points[newIndex];
    neighbors[newIndex].clear();
    for (const int index : neighborIndices) {
        CPoint t = points[index];
        float dist2 = (v.m_position - t.m_position).squaredNorm();
        if (dist2 < radius2) {
            neighbors[newIndex].push_back(index);
            neighbors[index].push_back(newIndex);
        }
    }
}

std::vector<CPoint> CPointSet::getPoints() const {
    return m_points;
}

CPointSet* CPointSet::simplify(const CSimplifyParameter& parameter) const {
    float epsilon = parameter.m_epsilon;

    std::map<std::tuple<int, int, int>, std::vector<int>> map;
    for (int i = 0; i < m_points.size(); i++) {
        int x = (int)std::floor(m_points[i].m_position(0) / epsilon);
        int y = (int)std::floor(m_points[i].m_position(1) / epsilon);
        int z = (int)std::floor(m_points[i].m_position(2) / epsilon);
        map[std::make_tuple(x, y, z)].push_back(i);
    }

    std::vector<CPoint> points;
    for (const std::pair<std::tuple<int, int, int>, std::vector<int>>& pair : map)
        points.push_back(m_points[*pair.second.begin()]);

    return new CPointSet(points);
}

CPointSet* CPointSet::resample(const CResampleParameter& parameter) const {
    float sharpnessAngle = parameter.m_sharpnessAngle;
    float edgeSensitivity = parameter.m_edgeSensitivity;
    float neighborRadius = parameter.m_neighborRadius;
    int size = parameter.m_size;

    edgeSensitivity *= 10.0f;
    float spacing = averageSpacing();
    if (neighborRadius < spacing)
        neighborRadius = spacing * 3.0f;
    float cosSigma = std::cos(sharpnessAngle / 180.0f * std::acos(-1.0f));
    sharpnessAngle = std::pow(std::max(1e-8f, 1.0f - cosSigma), 2.0f);

    std::vector<CPoint> points(m_points);
    std::vector<std::vector<int>> neighbors = calculateRadiusNeighbors(points, neighborRadius);

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
            neighbors = calculateRadiusNeighbors(points, currentRadius);
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

                points.push_back(CPoint((points[i].m_position + points[baseIndex].m_position) * 0.5f));
                isPassThreshold.push_back(false);
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

    return new CPointSet(points);
}

CPointSet* CPointSet::smooth(const CSmoothParameter& parameter) const {
    int k = parameter.m_k;
    float sharpnessAngle = parameter.m_sharpnessAngle;

    float cosSigma = std::cos(sharpnessAngle / 180.0f * std::acos(-1.0f));
    sharpnessAngle = std::pow(std::max(1e-8f, 1.0f - cosSigma), 2.0f);

    std::vector<std::vector<int>> neighbors = calculateKNeighbors(k);
    float radius = 0.0f;
    for (int i = 0; i < m_points.size(); i++)
        for (const int neighbor : neighbors[i])
            radius = std::max(radius, (m_points[i].m_position - m_points[neighbor].m_position).norm());
    radius *= 0.95;

    float radius2 = radius * radius;
    float radius16 = -4.0f / radius2;
    std::vector<CPoint> points;
    for (int i = 0; i < m_points.size(); i++) {
        CPoint v = m_points[i];
        float projectDistSum = 0.0f;
        Eigen::Vector3f normalSum(0.0f, 0.0f, 0.0f);
        float weightSum = 0.0f;
        for (const int neighbor : neighbors[i]) {
            CPoint t = m_points[neighbor];
            float dist2 = (v.m_position - t.m_position).squaredNorm();
            if (dist2 < radius2) {
                float theta = std::exp(dist2 * radius16);
                float psi = std::exp(-std::pow(1.0f - v.m_normal.dot(t.m_normal), 2.0f) / sharpnessAngle);
                float projectDiff = (t.m_position - v.m_position).dot(t.m_normal);
                float weight = theta * psi;

                projectDistSum += projectDiff * weight;
                normalSum += t.m_normal * weight;
                weightSum += weight;
            }
        }

        Eigen::Vector3f updateNormal = normalSum / weightSum;
        Eigen::Vector3f normal = updateNormal / updateNormal.norm();
        float projectDist = projectDistSum / weightSum;
        Eigen::Vector3f position = v.m_position + normal * projectDist;
        points.push_back(CPoint(position, normal));
    }

    return new CPointSet(points);
}

CMesh* CPointSet::reconstruct(const double maximumFacetLength) const {
    // TODO
    return nullptr;
}

void CPointSet::render() const {
    glPointSize(5);
    glBindVertexArray(m_vao);
    glDrawArrays(GL_POINTS, 0, m_points.size());
    glBindVertexArray(0);
}