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
    annClose();
}

std::vector<Point> CPointSet::toPoint(const std::vector<CPoint>& points) const {
    std::vector<Point> ans;
    for (const CPoint& point : points)
        ans.push_back(Point(point.m_position(0), point.m_position(1), point.m_position(2)));

    return ans;
}

std::vector<CPoint> CPointSet::fromPoint(const std::vector<Point>& points) const {
    std::vector<CPoint> ans;
    for (const Point& point : points)
        ans.push_back(CPoint(Eigen::Vector3f(point.x(), point.y(), point.z())));

    return ans;
}

std::vector<PointNormal> CPointSet::toPointNormal(const std::vector<CPoint>& points) const {
    std::vector<PointNormal> ans;
    for (const CPoint& point : points)
        ans.push_back(std::make_pair(Point(point.m_position(0), point.m_position(1), point.m_position(2)), Vector(point.m_normal(0), point.m_normal(1), point.m_normal(2))));

    return ans;
}

std::vector<CPoint> CPointSet::fromPointNormal(const std::vector<PointNormal>& points) const {
    std::vector<CPoint> ans;
    for (const PointNormal& point : points)
        ans.push_back(CPoint(Eigen::Vector3f(point.first.x(), point.first.y(), point.first.z()), Eigen::Vector3f(point.second.x(), point.second.y(), point.second.z())));

    return ans;
}

void CPointSet::calculateNormals(int k) {
    k = std::min(k, (int)m_points.size());

    ANNidxArray indices = new ANNidx[k];
    ANNdistArray distances = new ANNdist[k];

    std::vector<std::vector<std::pair<int, float>>> graph(m_points.size());
    for (int i = 0; i < m_points.size(); i++) {
        m_tree->annkSearch(m_pointArray[i], k, indices, distances);

        Eigen::Vector3f avg(0.0f, 0.0f, 0.0f);
        for (int j = 0; j < k; j++) {
            avg += m_points[indices[j]].m_position;
            graph[i].push_back(std::make_pair(indices[j], distances[j]));
            graph[indices[j]].push_back(std::make_pair(i, distances[j]));
        }
        avg /= k;

        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (int j = 0; j < k; j++) {
            Eigen::Vector3f x = m_points[indices[j]].m_position - avg;
            cov += x * x.transpose();
        }
        cov /= k;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver;
        solver.compute(cov);
        m_points[i].m_normal = solver.eigenvectors().col(0);
    }

    std::vector<float> dist(m_points.size(), FLT_MAX);
    std::vector<bool> flag(m_points.size(), false);
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<>> heap;

    int seed = randomUniform(m_points.size());
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
                if (m_points[next].m_normal.dot(m_points[now].m_normal) < 0.0f)
                    m_points[next].m_normal = -m_points[next].m_normal;
            }
        }
    }

    delete[] indices;
    delete[] distances;
}

float CPointSet::averageSpacing(int k) const {
    k = std::min(k, (int)m_points.size());

    ANNidxArray indices = new ANNidx[k];
    ANNdistArray distances = new ANNdist[k];
    float ans = 0.0f;

    for (int i = 0; i < m_points.size(); i++) {
        ANNpoint point = m_pointArray[i];
        m_tree->annkSearch(m_pointArray[i], k, indices, distances);
        float sum = 0.0f;
        for (int j = 0; j < k; j++)
            sum += distances[j];
        ans += sum / (float)k;
    }

    delete[] indices;
    delete[] distances;

    return ans / (float)m_points.size();
}

std::vector<std::vector<int>> CPointSet::calculateNeighbors(const std::vector<CPoint>& points, const float radius) const {
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
        float dotProduce = std::pow(2.0f - v.m_normal.dot(t.m_normal), edgeSensitivity);
        Eigen::Vector3f diffT = midPoint - t.m_position;
        float projectionT = diffT.dot(t.m_normal);
        float minDist2 = diffT.squaredNorm() - projectionT * projectionT;

        for (int j = 0; j < neighbors.size(); j++) {
            CPoint s = points[neighbors[j]];
            Eigen::Vector3f diffS = midPoint - s.m_position;
            float projectionS = diffS.dot(s.m_normal);
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
    for (int i = 0; i < neighbors[newIndex].size(); i++) {
        CPoint t = points[neighbors[newIndex][i]];
        float dist2 = (v.m_position - t.m_position).squaredNorm();
        float theta = std::exp(dist2 * radius16);

        for (int j = 0; j < 2; j++) {
            float psi = std::exp(-std::pow(1.0f - normalCandidates[j].dot(t.m_normal), 2) / sharpnessAngle);
            float projectDiff = (t.m_position - v.m_position).dot(t.m_normal);
            float weight = psi * theta;

            projectDistSum[j] += projectDiff * weight;
            normalSum[j] += t.m_normal * weight;
            weightSum[j] += weight;
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

CPointSet* CPointSet::simplify(const float epsilon) const {
    float minX, minY, minZ;
    minX = minY = minZ = FLT_MAX;

    for (const CPoint& point : m_points) {
        minX = std::min(minX, point.m_position(0));
        minY = std::min(minY, point.m_position(1));
        minZ = std::min(minZ, point.m_position(2));
    }

    std::map<std::tuple<int, int, int>, std::vector<int>> map;
    for (int i = 0; i < m_points.size(); i++) {
        int x = (int)((m_points[i].m_position(0) - minX) / epsilon);
        int y = (int)((m_points[i].m_position(1) - minY) / epsilon);
        int z = (int)((m_points[i].m_position(2) - minZ) / epsilon);
        map[std::make_tuple(x, y, z)].push_back(i);
    }

    std::vector<CPoint> points;
    for (const std::pair<std::tuple<int, int, int>, std::vector<int>>& pair : map)
        points.push_back(m_points[pair.second[randomUniform(pair.second.size())]]);

    return new CPointSet(points);
}

CPointSet* CPointSet::resample(float sharpnessAngle, float edgeSensitivity, float neighborRadius, int size) const {
    edgeSensitivity *= 10.0f;
    float spacing = averageSpacing();
    if (neighborRadius < spacing)
        neighborRadius = spacing * 3.0f;
    float cosSigma = std::cos(sharpnessAngle / 180.0f * std::acos(-1.0f));
    sharpnessAngle = std::pow(std::max(1e-8f, 1.0f - cosSigma), 2.0f);

    std::vector<CPoint> points(m_points);
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

CPointSet* CPointSet::smooth(const int k) const {
    //std::vector<CPoint> m_points = toPoint(vertices);
    //CGAL::jet_smooth_point_set<CGAL::Sequential_tag>(m_points, k);
    std::vector<PointNormal> points = toPointNormal(m_points);
    CGAL::bilateral_smooth_point_set<CGAL::Sequential_tag>(points, k, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointNormal>()).normal_map(CGAL::Second_of_pair_property_map<PointNormal>()));
    std::vector<CPoint> newPoints = fromPointNormal(points);

    //std::vector<Vertex> vertices = fromPoint(m_points);
    return new CPointSet(newPoints);
}

CMesh* CPointSet::reconstruct(const double maximumFacetLength) const {
    /*std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    if (type == 0) {
        std::vector<CPoint> m_points = toPoint(this->vertices);
        std::vector<std::array<std::size_t, 3>> facets;
        CGAL::advancing_front_surface_reconstruction(m_points.begin(), m_points.end(), std::back_inserter(facets));

        vertices = this->vertices;
        for (std::array<std::size_t, 3>& facet : facets)
            for (size_t index : facet)
                indices.push_back(index);
    }
    else if (type == 1) {
        std::vector<CPoint> m_points = toPoint(this->vertices);
        CGAL::Scale_space_surface_reconstruction_3<Kernel> reconstruct(m_points.begin(), m_points.end());
        reconstruct.increase_scale(4, CGAL::Scale_space_reconstruction_3::Jet_smoother<Kernel>());
        reconstruct.reconstruct_surface(CGAL::Scale_space_reconstruction_3::Advancing_front_mesher<Kernel>(1.0));

        vertices = this->vertices;
        for (auto iter = reconstruct.facets_begin(); iter != reconstruct.facets_end(); iter++)
            for (unsigned int index : *iter)
                indices.push_back(index);
    }
    else if (type == 2) {
        std::vector<PointNormal> m_points = toPointNormal(this->vertices);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (Vertex& vertex : this->vertices)
            cloud->push_back(pcl::PointXYZ(vertex.m_position.x, vertex.m_position.y, vertex.m_position.z));
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        SurfaceMesh mesh;
        CGAL::poisson_surface_reconstruction_delaunay(m_points.begin(), m_points.end(), CGAL::First_of_pair_property_map<PointNormal>(), CGAL::Second_of_pair_property_map<PointNormal>(), mesh, 1.0);

        std::map<VertexIndex, unsigned int> mapping;
        for (VertexIndex vertex : mesh.vertices()) {
            CPoint point = mesh.point(vertex);
            std::vector<int> indices;
            std::vector<float> distances;
            tree->nearestKSearch(pcl::PointXYZ(point.x(), point.y(), point.z()), 1, indices, distances);
            if (distances[0] < 0.5f) {
                mapping[vertex] = vertices.size();
                vertices.push_back(Vertex(point.x(), point.y(), point.z()));
            }
        }

        for (FaceIndex face : mesh.faces()) {
            HalfedgeIndex halfedge, end;
            bool flag = true;

            halfedge = end = mesh.halfedge(face);
            do {
                flag &= (mapping.find(mesh.source(halfedge)) != mapping.end());
                halfedge = mesh.next(halfedge);
            } while (halfedge != end);

            if (flag) {
                halfedge = end = mesh.halfedge(face);
                do {
                    indices.push_back(mapping[mesh.source(halfedge)]);
                    halfedge = mesh.next(halfedge);
                } while (halfedge != end);
            }
        }
    }
    else {
        pcl::PointCloud<pcl::PointNormal>::Ptr m_points(new pcl::PointCloud<pcl::PointNormal>);
        for (Vertex& vertex : this->vertices)
            m_points->push_back(pcl::PointNormal(vertex.m_position.x, vertex.m_position.y, vertex.m_position.z, vertex.m_normal.x, vertex.m_normal.y, vertex.m_normal.z));
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
        tree->setInputCloud(m_points);
        pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

        if (type == 3) {
            pcl::GreedyProjectionTriangulation<pcl::PointNormal> gpt;
            gpt.setSearchRadius(5.0);
            gpt.setMu(5.0);
            gpt.setMaximumNearestNeighbors(100);
            gpt.setMaximumSurfaceAngle(M_PI / 2);
            gpt.setMinimumAngle(M_PI / 18);
            gpt.setMaximumAngle(M_PI * 2 / 3);
            gpt.setNormalConsistency(false);
            gpt.setInputCloud(m_points);
            gpt.setSearchMethod(tree);
            gpt.reconstruct(*mesh);
        }
        else if (type == 4) {
            pcl::MarchingCubesHoppe<pcl::PointNormal> mc;
            mc.setGridResolution(64, 64, 64);
            mc.setInputCloud(m_points);
            mc.setSearchMethod(tree);
            mc.reconstruct(*mesh);
        }
        else if (type == 5) {
            pcl::MarchingCubesRBF<pcl::PointNormal> mc;
            mc.setGridResolution(64, 64, 64);
            mc.setInputCloud(m_points);
            mc.setSearchMethod(tree);
            mc.reconstruct(*mesh);
        }

        fromPCLPointCloud2(mesh->cloud, *m_points);
        for (pcl::PointNormal& point : *m_points)
            vertices.push_back(Vertex(point.x, point.y, point.z));
        for (pcl::Vertices& polygon : mesh->polygons)
            for (unsigned int vertex : polygon.vertices)
                indices.push_back(vertex);
    }*/

    std::vector<Point> points = toPoint(m_points);
    CGAL::Scale_space_surface_reconstruction_3<Kernel> reconstruct(points.begin(), points.end());
    reconstruct.increase_scale(4, CGAL::Scale_space_reconstruction_3::Jet_smoother<Kernel>());
    reconstruct.reconstruct_surface(CGAL::Scale_space_reconstruction_3::Advancing_front_mesher<Kernel>(maximumFacetLength));

    std::vector<unsigned int> indices;
    for (auto iter = reconstruct.facets_begin(); iter != reconstruct.facets_end(); iter++)
        for (unsigned int index : *iter)
            indices.push_back(index);
    std::cout << indices.size() / 3 << " facet(s) generated by reconstruction." << std::endl;

    return new CMesh(m_points, indices);
}

void CPointSet::render() const {
    glPointSize(5);
    glBindVertexArray(m_vao);
    glDrawArrays(GL_POINTS, 0, m_points.size());
    glBindVertexArray(0);
}