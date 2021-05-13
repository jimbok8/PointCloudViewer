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

std::vector<std::vector<int>> CPointSet::calculateRadiusNeighbors(const float radius) const {
    float radius2 = radius * radius;
    std::vector<std::vector<int>> ans;
    for (int i = 0; i < m_points.size(); i++) {
        ans.push_back(std::vector<int>());

        int k = m_tree->annkFRSearch(m_pointArray[i], radius2, 0);
        ANNidxArray indices = new ANNidx[k];
        ANNdistArray distances = new ANNdist[k];
        m_tree->annkFRSearch(m_pointArray[i], radius2, k, indices, distances);

        for (int j = 0; j < k; j++)
            if (indices[j] != i)
                ans[i].push_back(indices[j]);

        delete[] indices;
        delete[] distances;
    }

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

float CPointSet::calculateRadius(const std::vector<Eigen::Vector3f>& points, const int p0, const int p1, const int p2) const {
    Eigen::Vector3f v0 = points[p0];
    Eigen::Vector3f v1 = points[p1];
    Eigen::Vector3f v2 = points[p2];

    float a = (v1 - v0).norm();
    float b = (v2 - v1).norm();
    float c = (v0 - v2).norm();
    float p = (a + b + c) / 2.0f;

    return a * b * c / std::sqrt(4.0f * p * (p - a) * (p - b) * (p - c));
}

Eigen::Vector3f CPointSet::calculateNormal(const std::vector<Eigen::Vector3f>& points, const int p0, const int p1, const int p2) const {
    Eigen::Vector3f v0 = points[p0];
    Eigen::Vector3f v1 = points[p1];
    Eigen::Vector3f v2 = points[p2];
    return (v1 - v0).cross(v2 - v1).normalized();
}

CCandidate CPointSet::calculateCandidate(const std::vector<Eigen::Vector3f>& points, const std::vector<float>& radii, const std::vector<bool>& flag, const std::vector<std::pair<int, int>>& candidates, const int source, const int target, const Eigen::Vector3f& normal) const {
    float minRadius = FLT_MAX;
    CCandidate ans;

    for (const std::pair<int, int>& candidate : candidates) {
        int opposite = candidate.first;
        int index = candidate.second;
        float radius = radii[index];

        if (!flag[index]) {
            Eigen::Vector3f normalTemp = calculateNormal(points, source, opposite, target);
            float beta = std::acos(normal.dot(normalTemp));
            if (beta < ALPHA && radius < minRadius) {
                minRadius = radius;
                ans = CCandidate(source, target, opposite, index, normalTemp, beta, radius);
            }
        }
    }

    return ans;
}

void CPointSet::addEdge(const std::vector<Eigen::Vector3f>& points, std::map<std::pair<int, int>, Eigen::Vector3f>& edges, std::priority_queue<CCandidate>& heap, const std::vector<float>& radii, const std::vector<bool>& flag, const std::vector<std::pair<int, int>>& candidates, const int source, const int target, const Eigen::Vector3f& normal) const {
    CCandidate candidate = calculateCandidate(points, radii, flag, candidates, source, target, normal);
    edges[std::make_pair(source, target)] = normal;
    if (!candidate.empty())
        heap.push(candidate);
}

CMesh* CPointSet::reconstruct(const CReconstructParameter& parameter) const {
    int iterationNumber = parameter.m_iterationNumber;
    float maximumFacetLength = parameter.m_maximumFacetLength;

    std::vector<Eigen::Vector3f> points;
    for (const CPoint& point : m_points)
        points.push_back(point.m_position);

    std::vector<std::vector<int>> neighbors = calculateKNeighbors(12);
    float radius = 0.0f;
    for (int i = 0; i < points.size(); i++)
        radius += (points[i] - points[*neighbors[i].rbegin()]).norm();
    radius /= points.size();
    neighbors = calculateRadiusNeighbors(radius);

    for (int iter = 0; iter < iterationNumber; iter++) {
        for (int i = 0; i < points.size(); i++)
            if (neighbors[i].size() >= 4) {
                float weightSum = 0.0f;
                std::vector<float> weights;
                Eigen::Vector3f avg(0.0f, 0.0f, 0.0f);
                for (int j = 0; j < neighbors[i].size(); j++) {
                    weights.push_back(1.0f / (points[i] - points[neighbors[i][j]]).norm());
                    weightSum += weights[j];
                    avg += weights[j] * points[neighbors[i][j]];
                }
                avg /= weightSum;

                Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
                for (int j = 0; j < neighbors[i].size(); j++) {
                    Eigen::Vector3f x = points[neighbors[i][j]] - avg;
                    cov += weights[j] * x * x.transpose();
                }
                cov /= weightSum;

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver;
                solver.compute(cov);
                Eigen::Vector3f normal = solver.eigenvectors().col(0);

                points[i] -= normal.dot(points[i] - avg) * normal;
            }
    }

    float minX, maxX, minY, maxY, minZ, maxZ;
    minX = minY = minZ = FLT_MAX;
    maxX = maxY = maxZ = -FLT_MAX;
    for (const Eigen::Vector3f& point : points) {
        minX = std::min(minX, point(0));
        maxX = std::max(maxX, point(0));
        minY = std::min(minY, point(1));
        maxY = std::max(maxY, point(1));
        minZ = std::min(minZ, point(2));
        maxZ = std::max(maxZ, point(2));
    }

    float lowerX = minX - 1.0f;
    float upperX = maxX + 1.0f;
    float lowerY = minY - 1.0f;
    float upperY = maxY + 1.0f;
    float lowerZ = minZ - 1.0f;
    float upperZ = maxZ + 1.0f;

    points.push_back(Eigen::Vector3f(lowerX, lowerY, lowerZ));
    points.push_back(Eigen::Vector3f(lowerX + (upperX - lowerX) * 3.0f, lowerY, lowerZ));
    points.push_back(Eigen::Vector3f(lowerX, lowerY + (upperY - lowerY) * 3.0f, lowerZ));
    points.push_back(Eigen::Vector3f(lowerX, lowerY, lowerZ + (upperZ - lowerZ) * 3.0f));

    std::list<CTetrahedron> tetrahedrons;
    tetrahedrons.push_back(CTetrahedron(points, points.size() - 4, points.size() - 3, points.size() - 2, points.size() - 1));
    for (int i = 0; i < points.size() - 4; i++) {
        std::set<CTriangle> triangles;
        for (auto iter = tetrahedrons.begin(); iter != tetrahedrons.end(); )
            if (iter->contain(points[i])) {
                std::vector<CTriangle> trianglesTemp = iter->getTriangles();
                for (const CTriangle& triangleTemp : trianglesTemp) {
                    auto jter = triangles.find(triangleTemp);
                    if (jter != triangles.end())
                        triangles.erase(jter);
                    else
                        triangles.insert(triangleTemp);
                }

                iter = tetrahedrons.erase(iter);
            }
            else
                iter++;

        for (const CTriangle& triangle : triangles)
            tetrahedrons.push_back(CTetrahedron(points, i, triangle));
    }

    std::set<CTriangle> triangles;
    for (const CTetrahedron& tetrahedron : tetrahedrons) {
        std::vector<CTriangle> trianglesTemp = tetrahedron.getTriangles();
        for (const CTriangle& triangleTemp : trianglesTemp) {
            std::vector<int> indices = triangleTemp.getIndices();
            std::vector<float> lengths = triangleTemp.getLengths();
            bool flag = true;
            for (int i = 0; i < 3; i++)
                if (indices[i] >= m_points.size() || lengths[i] > maximumFacetLength) {
                    flag = false;
                    break;
                }

            if (flag)
                triangles.insert(triangleTemp);
        }
    }

    float minRadius = FLT_MAX;
    int seedIndex;
    std::vector<int> seedIndices;
    std::vector<float> radii;
    std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> candidates;
    for (const CTriangle& triangle : triangles) {
        std::vector<int> indices = triangle.getIndices();

        float radius = triangle.getRadius();
        if (radius < minRadius) {
            minRadius = radius;
            seedIndex = radii.size();
            seedIndices = indices;
        }

        int p0 = indices[0];
        int p1 = indices[1];
        int p2 = indices[2];
        candidates[std::make_pair(p0, p1)].push_back(std::make_pair(p2, radii.size()));
        candidates[std::make_pair(p1, p0)].push_back(std::make_pair(p2, radii.size()));
        candidates[std::make_pair(p0, p2)].push_back(std::make_pair(p1, radii.size()));
        candidates[std::make_pair(p2, p0)].push_back(std::make_pair(p1, radii.size()));
        candidates[std::make_pair(p1, p2)].push_back(std::make_pair(p0, radii.size()));
        candidates[std::make_pair(p2, p1)].push_back(std::make_pair(p0, radii.size()));
        radii.push_back(radius);
    }

    std::map<std::pair<int, int>, Eigen::Vector3f> edges;
    std::set<int> allPoints;
    std::priority_queue<CCandidate> heap;
    std::vector<CCandidate> later;
    std::vector<bool> flag(triangles.size(), false);
    std::vector<unsigned int> indices;
    int p0 = seedIndices[0];
    int p1 = seedIndices[1];
    int p2 = seedIndices[2];
    Eigen::Vector3f normal = calculateNormal(points, p0, p1, p2);
    flag[seedIndex] = true;
    addEdge(points, edges, heap, radii, flag, candidates[std::make_pair(p0, p1)], p0, p1, normal);
    addEdge(points, edges, heap, radii, flag, candidates[std::make_pair(p1, p2)], p1, p2, normal);
    addEdge(points, edges, heap, radii, flag, candidates[std::make_pair(p2, p0)], p2, p0, normal);
    indices.push_back(p0);
    indices.push_back(p1);
    indices.push_back(p1);
    indices.push_back(p2);
    indices.push_back(p2);
    indices.push_back(p0);
    allPoints.insert(p0);
    allPoints.insert(p1);
    allPoints.insert(p2);
    CMeshBoundary boundary(p0, p1, p2);

    while (!heap.empty()) {
        CCandidate now = heap.top();
        heap.pop();
        int source = now.getSource();
        int target = now.getTarget();
        int opposite = now.getOpposite();
        int index = now.getIndex();
        Eigen::Vector3f normal = now.getNormal();
        std::pair<int, int> edge = std::make_pair(source, target);
        if (flag[index]) {
            CCandidate candidate = calculateCandidate(points, radii, flag, candidates[std::make_pair(source, target)], source, target, edges[edge]);
            if (!candidate.empty())
                heap.push(candidate);
            continue;
        }

        int lastTemp, nextTemp;
        boundary.neighbors(source, lastTemp, nextTemp);
        if (boundary.contain(source) > -1 && boundary.contain(target) > -1 && nextTemp == target) {
            if (allPoints.find(opposite) == allPoints.end()) {
                allPoints.insert(opposite);
                boundary.insert(source, target, opposite);

                edges.erase(edge);
                addEdge(points, edges, heap, radii, flag, candidates[std::make_pair(source, opposite)], source, opposite, normal);
                addEdge(points, edges, heap, radii, flag, candidates[std::make_pair(opposite, target)], opposite, target, normal);

                indices.push_back(source);
                indices.push_back(opposite);
                indices.push_back(target);

                flag[index] = true;

                for (const CCandidate& candidate : later)
                    heap.push(candidate);
                std::vector<CCandidate>().swap(later);
            }
            else if (boundary.contain(source) == boundary.contain(opposite)) {
                int last, next;
                boundary.neighbors(opposite, last, next);
                if (last == target && next == source) {
                    boundary.erase(opposite);

                    edges.erase(edge);

                    indices.push_back(source);
                    indices.push_back(opposite);
                    indices.push_back(target);

                    flag[index] = true;

                    for (const CCandidate& candidate : later)
                        heap.push(candidate);
                    std::vector<CCandidate>().swap(later);
                }
                else if (last == target) {
                    boundary.erase(target);

                    edges.erase(edge);
                    addEdge(points, edges, heap, radii, flag, candidates[std::make_pair(source, opposite)], source, opposite, normal);

                    indices.push_back(source);
                    indices.push_back(opposite);
                    indices.push_back(target);

                    flag[index] = true;

                    for (const CCandidate& candidate : later)
                        heap.push(candidate);
                    std::vector<CCandidate>().swap(later);
                }
                else if (next == source) {
                    boundary.erase(source);

                    edges.erase(edge);
                    addEdge(points, edges, heap, radii, flag, candidates[std::make_pair(opposite, target)], opposite, target, normal);

                    indices.push_back(source);
                    indices.push_back(opposite);
                    indices.push_back(target);

                    flag[index] = true;

                    for (const CCandidate& candidate : later)
                        heap.push(candidate);
                    std::vector<CCandidate>().swap(later);
                }
                else {
                    int index0 = -1, index1 = -1;
                    for (const std::pair<int, int>& pair : candidates[std::make_pair(source, opposite)])
                        if (pair.first == next && !flag[pair.second]) {
                            index0 = pair.second;
                            break;
                        }
                    for (const std::pair<int, int>& pair : candidates[std::make_pair(target, opposite)])
                        if (pair.first == last && !flag[pair.second]) {
                            index1 = pair.second;
                            break;
                        }

                    Eigen::Vector3f normal0 = calculateNormal(points, next, opposite, source);
                    Eigen::Vector3f normal1 = calculateNormal(points, target, opposite, last);

                    float beta0 = std::acos(normal.dot(normal0));
                    float beta1 = std::acos(normal.dot(normal1));

                    float radius0 = index0 > -1 ? radii[index0] : FLT_MAX;
                    float radius1 = index1 > -1 ? radii[index1] : FLT_MAX;

                    if (index0 > -1 && beta0 < ALPHA && radius0 < radius1) {
                        boundary.split(source, target, opposite, next);

                        edges.erase(edge);
                        addEdge(points, edges, heap, radii, flag, candidates[std::make_pair(opposite, target)], opposite, target, normal);
                        addEdge(points, edges, heap, radii, flag, candidates[std::make_pair(source, next)], source, next, normal0);

                        indices.push_back(source);
                        indices.push_back(opposite);
                        indices.push_back(target);

                        indices.push_back(next);
                        indices.push_back(opposite);
                        indices.push_back(source);

                        flag[index] = true;
                        flag[index0] = true;

                        for (const CCandidate& candidate : later)
                            heap.push(candidate);
                        std::vector<CCandidate>().swap(later);
                    }
                    else if (index1 > -1 && beta1 < ALPHA && radius0 > radius1) {
                        boundary.split(source, target, last, opposite);

                        edges.erase(edge);
                        addEdge(points, edges, heap, radii, flag, candidates[std::make_pair(source, opposite)], source, opposite, normal);
                        addEdge(points, edges, heap, radii, flag, candidates[std::make_pair(last, target)], last, target, normal1);

                        indices.push_back(source);
                        indices.push_back(opposite);
                        indices.push_back(target);

                        indices.push_back(target);
                        indices.push_back(opposite);
                        indices.push_back(last);

                        flag[index] = true;
                        flag[index1] = true;

                        for (const CCandidate& candidate : later)
                            heap.push(candidate);
                        std::vector<CCandidate>().swap(later);
                    }
                    else
                        later.push_back(now);
                }
            }
            else {
                flag[index] = true;

                CCandidate candidate = calculateCandidate(points, radii, flag, candidates[std::make_pair(source, target)], source, target, edges[edge]);
                if (!candidate.empty())
                    heap.push(candidate);
            }
        }
    }

    std::vector<CBoundary> boundaries = boundary.getBoundaries();
    for (const CBoundary& boundary : boundaries)
        if (boundary.size() == 3) {
            std::vector<int> indicesTemp = boundary.getIndices();
            
            int x = indicesTemp[0];
            int y = indicesTemp[1];
            int z = indicesTemp[2];

            float a = (points[x] - points[y]).norm();
            float b = (points[y] - points[z]).norm();
            float c = (points[z] - points[x]).norm();

            if (a <= maximumFacetLength && b <= maximumFacetLength && c <= maximumFacetLength) {
                indices.push_back(x);
                indices.push_back(y);
                indices.push_back(z);
            }
        }

    return new CMesh(m_points, indices);
}

void CPointSet::render() const {
    glPointSize(5);
    glBindVertexArray(m_vao);
    glDrawArrays(GL_POINTS, 0, m_points.size());
    glBindVertexArray(0);
}