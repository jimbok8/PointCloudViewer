#include "CPointSet.h"

CPointSet::CPointSet(const std::string& path) {
    std::fstream::sync_with_stdio(false);
    std::ifstream fin(path);
    std::string s;

    for (int i = 0; i < 11; i++)
        std::getline(fin, s);

    float x, y, z, weight, r, g, b;
    while (fin >> x >> y >> z >> weight >> r >> g >> b) {
        if (weight >= 500.0f)
            m_points.push_back(CPoint(Eigen::Vector3f(x, y, z)));
        std::getline(fin, s);
    }

    initialize(true);
}

CPointSet::CPointSet(const std::vector<CPoint>& points, bool normalFlag) :
    m_points(points) {
    initialize(normalFlag);
}

CPointSet::~CPointSet() {
    if (m_points.size() > 0) {
        annDeallocPts(m_pointArray);
        delete m_tree;
    }
}

void CPointSet::bind() {
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

void CPointSet::initialize(bool normalFlag) {
    m_pointArray = annAllocPts(m_points.size(), 3);
    for (int i = 0; i < m_points.size(); i++)
        for (int j = 0; j < 3; j++)
            m_pointArray[i][j] = m_points[i].m_position(j);
    m_tree = new ANNkd_tree(m_pointArray, m_points.size(), 3);

    if (normalFlag)
        calculateNormals();

    bind();
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
    float spacing = averageSpacing();
    spacing = (spacing * 3.0f) * (spacing * 3.0f);
    for (int i = 0; i < m_points.size(); i++) {
        Eigen::Vector3f avg(0.0f, 0.0f, 0.0f);
        int num = 0;
        float sum = 0.0f;
        for (const int neighbor : neighbors[i]) {
            float dist2 = (m_points[i].m_position - m_points[neighbor].m_position).squaredNorm();
            if (dist2 < spacing) {
                avg += m_points[neighbor].m_position;
                num++;
                sum += std::sqrt(dist2);
                graph[i].push_back(std::make_pair(neighbor, dist2));
                graph[neighbor].push_back(std::make_pair(i, dist2));
            }
        }
        avg /= (float)num;
        sum /= (float)num;

        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (const int neighbor : neighbors[i]) {
            float dist2 = (m_points[i].m_position - m_points[neighbor].m_position).squaredNorm();
            if (dist2 < spacing) {
                Eigen::Vector3f x = m_points[neighbor].m_position - avg;
                cov += x * x.transpose();
            }
        }
        cov /= (float)num;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver;
        solver.compute(cov);
        m_points[i].m_normal = solver.eigenvectors().col(0);
        m_points[i].m_u = solver.eigenvectors().col(2).normalized();
        m_points[i].m_v = m_points[i].m_normal.cross(m_points[i].m_u);
        m_points[i].m_u *= sum;
        m_points[i].m_v *= sum;
    }

    /*std::vector<float> dist(m_points.size(), FLT_MAX);
    std::vector<bool> flag(m_points.size(), false);
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<>> heap;
    for (int i = 0; i < m_points.size(); i++)
        if (!flag[i]) {
            dist[i] = 0.0f;
            heap.push(std::make_pair(dist[i], i));
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
                        if (m_points[next].m_normal.dot(m_points[now].m_normal) < 0.0f) {
                            m_points[next].m_normal = -m_points[next].m_normal;
                            m_points[next].m_u = -m_points[next].m_u;
                        }
                    }
                }
            }
        }*/

    for (CPoint& point : m_points)
        if (point.m_normal.dot(point.m_position) < 0.0f) {
            point.m_normal = -point.m_normal;
            point.m_u = -point.m_u;
        }
}

Eigen::Vector3f CPointSet::centroid() const {
    Eigen::Vector3f ans(0.0f, 0.0f, 0.0f);
    for (const CPoint& point : m_points)
        ans += point.m_position;

    return ans / (float)m_points.size();
}

Eigen::Vector3f CPointSet::nearest(const Eigen::Vector3f& point) const {
    ANNpoint pointTemp = annAllocPt(3);
    for (int i = 0; i < 3; i++)
        pointTemp[i] = point(i);

    ANNidxArray indices = new ANNidx[1];
    ANNdistArray distances = new ANNdist[1];
    m_tree->annkSearch(pointTemp, 1, indices, distances);

    int index = indices[0];

    delete[] indices;
    delete[] distances;

    return m_points[index].m_position;
}

std::vector<CPoint> CPointSet::getPoints() const {
    return m_points;
}

int CPointSet::size() const {
    return m_points.size();
}

float CPointSet::scale() const {
    float minX, maxX, minY, maxY, minZ, maxZ;
    minX = minY = minZ = FLT_MAX;
    maxX = maxY = maxZ = -FLT_MAX;

    for (const CPoint& point : m_points) {
        minX = std::min(minX, point.m_position.x());
        maxX = std::max(maxX, point.m_position.x());
        minY = std::min(minY, point.m_position.y());
        maxY = std::max(maxY, point.m_position.y());
        minZ = std::min(minZ, point.m_position.z());
        maxZ = std::max(maxZ, point.m_position.z());
    }

    return std::max(maxX - minX, std::max(maxY - minY, maxZ - minZ));
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

CPointSet* CPointSet::simplify(const CSimplifyParameter& parameter, bool normalFlag) const {
    float epsilon = parameter.m_epsilon;

    //std::map<std::tuple<int, int, int>, int> num;
    //std::map<std::tuple<int, int, int>, Eigen::Vector3f> sumPosition, sumNormal, sumU, sumV;
    std::map<std::tuple<int, int, int>, std::vector<CPoint>> indices;
    for (const CPoint& point : m_points) {
        int x = (int)std::floor(point.m_position.x() / epsilon);
        int y = (int)std::floor(point.m_position.y() / epsilon);
        int z = (int)std::floor(point.m_position.z() / epsilon);
        std::tuple<int, int, int> tuple = std::make_tuple(x, y, z);
        
        indices[tuple].push_back(point);
        //if (num.find(tuple) == num.end())
        //    num[tuple] = 0;    
        //num[tuple]++;
        //if (sumPosition.find(tuple) == sumPosition.end())
        //    sumPosition[tuple] = Eigen::Vector3f::Zero();
        //sumPosition[tuple] += point.m_position;
        //if (sumNormal.find(tuple) == sumNormal.end())
        //    sumNormal[tuple] = Eigen::Vector3f::Zero();
        //sumNormal[tuple] += point.m_normal;
        //if (sumU.find(tuple) == sumU.end())
        //    sumU[tuple] = Eigen::Vector3f::Zero();
        //sumU[tuple] += point.m_u;
        //if (sumV.find(tuple) == sumV.end())
        //    sumV[tuple] = Eigen::Vector3f::Zero();
        //sumV[tuple] += point.m_v;
    }

    std::vector<CPoint> points;
    //for (const std::pair<std::tuple<int, int, int>, int>& pair : num)
        //points.push_back(CPoint(sumPosition[pair.first] / (float)pair.second, sumNormal[pair.first] / (float)pair.second, sumU[pair.first] / (float)pair.second, sumV[pair.first] / (float)pair.second));
    for (const std::pair<std::tuple<int, int, int>, std::vector<CPoint>>& pair : indices)
        points.push_back(*pair.second.rbegin());

    return new CPointSet(points, normalFlag);
}

CPointSet* CPointSet::smooth(const CSmoothParameter& parameter, bool normalFlag) const {
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
        points.push_back(CPoint(position, normal, m_points[i].m_u, m_points[i].m_v));
    }

    return new CPointSet(points, normalFlag);
}

void CPointSet::registrate(const CPointSet* target, Eigen::Matrix3f& rotate, Eigen::Vector3f& translate) const {
    const int MAX_ITERATION = 100;
    const float EPSILON = 1e-6f;
    const float DISTANCE_THRESHOLD = 0.05f;

    rotate = Eigen::Matrix3f::Identity();
    translate = Eigen::Vector3f::Zero();
    float loss = 1e10;

    Eigen::Vector3f centroidSource = centroid();
    Eigen::Vector3f centroidTarget = target->centroid();

    for (int i = 0; i < MAX_ITERATION; i++) {
        std::vector<CPoint> current;
        for (const CPoint& point : m_points)
            current.push_back(CPoint(rotate * point.m_position + translate));

        Eigen::Vector3f centroidCurrent = rotate * centroidSource + translate;

        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        float lossTemp = 0.0f;

        for (const CPoint& point : current) {
            Eigen::Vector3f nearest = target->nearest(point.m_position);
            float dist2 = (point.m_position - nearest).squaredNorm();
            if (dist2 < DISTANCE_THRESHOLD) {
                H += (point.m_position - centroidCurrent) * (nearest - centroidTarget).transpose();
                lossTemp += dist2;
            }
        }

        if (std::fabs(loss - lossTemp) < EPSILON)
            break;
        else
            loss = lossTemp;

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
        rotate = svd.matrixV() * svd.matrixU().transpose();
        translate = centroidTarget - rotate * centroidCurrent;
    }
}

CPointSet* CPointSet::combine(const CPointSet* set, bool normalFlag) const {
    std::vector<CPoint> points = m_points;
    for (const CPoint& point : set->m_points)
        points.push_back(point);

    return new CPointSet(points, normalFlag);
}

void CPointSet::combine(const CPointSet* set, const Eigen::Matrix3f& rotate, const Eigen::Vector3f& translate) {
    for (const CPoint& point : set->m_points)
        m_points.push_back(CPoint(rotate * point.m_position + translate));
}

void CPointSet::render() const {
    glPointSize(5);
    glBindVertexArray(m_vao);
    glDrawArrays(GL_POINTS, 0, m_points.size());
    glBindVertexArray(0);
}

void CPointSet::save(const std::string & path) const {
    std::fstream::sync_with_stdio(false);
    std::ofstream fout(path);

    for (const CPoint& point : m_points) {
        Eigen::Vector3f position = point.m_position * 0.01f;
        Eigen::Vector3f u = point.m_u * 0.01f;
        Eigen::Vector3f v = point.m_v * 0.01f;

        if (isnormal(position.x()) && isnormal(position.y()) && isnormal(position.z()) && isnormal(u.x()) && isnormal(u.y()) && isnormal(u.z()) && isnormal(v.x()) && isnormal(v.y()) && isnormal(v.z())) {
            fout << position.x() << ' ' << position.y() << ' ' << position.z() << ' ';
            fout << u.x() << ' ' << u.y() << ' ' << u.z() << ' ';
            fout << v.x() << ' ' << v.y() << ' ' << v.z() << std::endl;
        }
    }
}