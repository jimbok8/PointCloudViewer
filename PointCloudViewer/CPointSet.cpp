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
    int seed = rand() % m_points.size();
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

Eigen::Vector3f CPointSet::interpolate(const CFunction* function, Eigen::Vector3f& p1, float f1, Eigen::Vector3f& p2, float f2) const {
    while ((p1 - p2).squaredNorm() > 0.01f) {
        Eigen::Vector3f p = (p1 + p2) * 0.5f;
        float f = function->f(p);
        if (f * f1 >= 0.0f) {
            p1 = p;
            f1 = f;
        }
        else {
            p2 = p;
            f2 = f;
        }
    }

    return f1 < 0.0f ? p1 : p2;
}

CMesh* CPointSet::marchingCubes(const int resolutionX, const int resolutionY, const int resolutionZ, const float radius, const float epsilon) const {
    const unsigned long long triangles[256] = {
        0ULL, 33793ULL, 36945ULL, 159668546ULL,
        18961ULL, 144771090ULL, 5851666ULL, 595283255635ULL,
        20913ULL, 67640146ULL, 193993474ULL, 655980856339ULL,
        88782242ULL, 736732689667ULL, 797430812739ULL, 194554754ULL,
        26657ULL, 104867330ULL, 136709522ULL, 298069416227ULL,
        109224258ULL, 8877909667ULL, 318136408323ULL, 1567994331701604ULL,
        189884450ULL, 350847647843ULL, 559958167731ULL, 3256298596865604ULL,
        447393122899ULL, 651646838401572ULL, 2538311371089956ULL, 737032694307ULL,
        29329ULL, 43484162ULL, 91358498ULL, 374810899075ULL,
        158485010ULL, 178117478419ULL, 88675058979ULL, 433581536604804ULL,
        158486962ULL, 649105605635ULL, 4866906995ULL, 3220959471609924ULL,
        649165714851ULL, 3184943915608436ULL, 570691368417972ULL, 595804498035ULL,
        124295042ULL, 431498018963ULL, 508238522371ULL, 91518530ULL,
        318240155763ULL, 291789778348404ULL, 1830001131721892ULL, 375363605923ULL,
        777781811075ULL, 1136111028516116ULL, 3097834205243396ULL, 508001629971ULL,
        2663607373704004ULL, 680242583802939237ULL, 333380770766129845ULL, 179746658ULL,
        42545ULL, 138437538ULL, 93365810ULL, 713842853011ULL,
        73602098ULL, 69575510115ULL, 23964357683ULL, 868078761575828ULL,
        28681778ULL, 713778574611ULL, 250912709379ULL, 2323825233181284ULL,
        302080811955ULL, 3184439127991172ULL, 1694042660682596ULL, 796909779811ULL,
        176306722ULL, 150327278147ULL, 619854856867ULL, 1005252473234484ULL,
        211025400963ULL, 36712706ULL, 360743481544788ULL, 150627258963ULL,
        117482600995ULL, 1024968212107700ULL, 2535169275963444ULL, 4734473194086550421ULL,
        628107696687956ULL, 9399128243ULL, 5198438490361643573ULL, 194220594ULL,
        104474994ULL, 566996932387ULL, 427920028243ULL, 2014821863433780ULL,
        492093858627ULL, 147361150235284ULL, 2005882975110676ULL, 9671606099636618005ULL,
        777701008947ULL, 3185463219618820ULL, 482784926917540ULL, 2900953068249785909ULL,
        1754182023747364ULL, 4274848857537943333ULL, 13198752741767688709ULL, 2015093490989156ULL,
        591272318771ULL, 2659758091419812ULL, 1531044293118596ULL, 298306479155ULL,
        408509245114388ULL, 210504348563ULL, 9248164405801223541ULL, 91321106ULL,
        2660352816454484ULL, 680170263324308757ULL, 8333659837799955077ULL, 482966828984116ULL,
        4274926723105633605ULL, 3184439197724820ULL, 192104450ULL, 15217ULL,
        45937ULL, 129205250ULL, 129208402ULL, 529245952323ULL,
        169097138ULL, 770695537027ULL, 382310500883ULL, 2838550742137652ULL,
        122763026ULL, 277045793139ULL, 81608128403ULL, 1991870397907988ULL,
        362778151475ULL, 2059003085103236ULL, 2132572377842852ULL, 655681091891ULL,
        58419234ULL, 239280858627ULL, 529092143139ULL, 1568257451898804ULL,
        447235128115ULL, 679678845236084ULL, 2167161349491220ULL, 1554184567314086709ULL,
        165479003923ULL, 1428768988226596ULL, 977710670185060ULL, 10550024711307499077ULL,
        1305410032576132ULL, 11779770265620358997ULL, 333446212255967269ULL, 978168444447012ULL,
        162736434ULL, 35596216627ULL, 138295313843ULL, 891861543990356ULL,
        692616541075ULL, 3151866750863876ULL, 100103641866564ULL, 6572336607016932133ULL,
        215036012883ULL, 726936420696196ULL, 52433666ULL, 82160664963ULL,
        2588613720361524ULL, 5802089162353039525ULL, 214799000387ULL, 144876322ULL,
        668013605731ULL, 110616894681956ULL, 1601657732871812ULL, 430945547955ULL,
        3156382366321172ULL, 7644494644932993285ULL, 3928124806469601813ULL, 3155990846772900ULL,
        339991010498708ULL, 10743689387941597493ULL, 5103845475ULL, 105070898ULL,
        3928064910068824213ULL, 156265010ULL, 1305138421793636ULL, 27185ULL,
        195459938ULL, 567044449971ULL, 382447549283ULL, 2175279159592324ULL,
        443529919251ULL, 195059004769796ULL, 2165424908404116ULL, 1554158691063110021ULL,
        504228368803ULL, 1436350466655236ULL, 27584723588724ULL, 1900945754488837749ULL,
        122971970ULL, 443829749251ULL, 302601798803ULL, 108558722ULL,
        724700725875ULL, 43570095105972ULL, 2295263717447940ULL, 2860446751369014181ULL,
        2165106202149444ULL, 69275726195ULL, 2860543885641537797ULL, 2165106320445780ULL,
        2280890014640004ULL, 11820349930268368933ULL, 8721082628082003989ULL, 127050770ULL,
        503707084675ULL, 122834978ULL, 2538193642857604ULL, 10129ULL,
        801441490467ULL, 2923200302876740ULL, 1443359556281892ULL, 2901063790822564949ULL,
        2728339631923524ULL, 7103874718248233397ULL, 12775311047932294245ULL, 95520290ULL,
        2623783208098404ULL, 1900908618382410757ULL, 137742672547ULL, 2323440239468964ULL,
        362478212387ULL, 727199575803140ULL, 73425410ULL, 34337ULL,
        163101314ULL, 668566030659ULL, 801204361987ULL, 73030562ULL,
        591509145619ULL, 162574594ULL, 100608342969108ULL, 5553ULL,
        724147968595ULL, 1436604830452292ULL, 176259090ULL, 42001ULL,
        143955266ULL, 2385ULL, 18433ULL, 0ULL,
    };
    const int edges[12][2] = {
        {0, 1},
        {2, 3},
        {4, 5},
        {6, 7},
        {0, 2},
        {1, 3},
        {4, 6},
        {5, 7},
        {0, 4},
        {1, 5},
        {2, 6},
        {3, 7}
    };

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

    //minX = minY = minZ = -1.05f;
    //maxX = maxY = maxZ = 1.05f;
    float dx = (maxX - minX) / (float)resolutionX;
    float dy = (maxY - minY) / (float)resolutionY;
    float dz = (maxZ - minZ) / (float)resolutionZ;
    //CFunction* function = new CSphereFunction(Eigen::Vector3f(0.0f, 0.0f, 0.0f), 1.0f);
    CFunction* function = new CIMLSFunction(m_points, radius, epsilon);
    std::vector<CPoint> points;
    std::vector<unsigned int> indices;
    
    float x = minX;
    for (int i = 0; i < resolutionX; i++) {
        float y = minY;
        for (int j = 0; j < resolutionY; j++) {
            float z = minZ;
            for (int k = 0; k < resolutionZ; k++) {
                Eigen::Vector3f vertices[8] = {
                    Eigen::Vector3f(x, y, z),
                    Eigen::Vector3f(x + dx, y, z),
                    Eigen::Vector3f(x, y + dy, z),
                    Eigen::Vector3f(x + dx, y + dy, z),
                    Eigen::Vector3f(x, y, z + dz),
                    Eigen::Vector3f(x + dx, y, z + dz),
                    Eigen::Vector3f(x, y + dy, z + dz),
                    Eigen::Vector3f(x + dx, y + dy, z + dz)
                };
                z += dz;

                int t = 0;
                std::vector<float> f(8);
                bool flag = true;
                for (int h = 7; h >= 0; h--) {
                    f[h] = function->f(vertices[h]);
                    if (!std::isnormal(f[h])) {
                        flag = false;
                        break;
                    }
                    t = (t << 1) + (f[h] < 0.0f);
                }
                if (!flag || t == 0 || t == 255)
                    continue;

                unsigned long long tmp = triangles[t];
                int triangleNum = tmp & 0xF;
                tmp >>= 4;
                for (int h = 0; h < triangleNum * 3; h++) {
                    int edgeIndex = tmp & 0xF;
                    tmp >>= 4;

                    int v1 = edges[edgeIndex][0];
                    int v2 = edges[edgeIndex][1];

                    float t1 = std::fabs(f[v1]);
                    float t2 = std::fabs(f[v2]);

                    points.push_back(CPoint((t1 * vertices[v1] + t2 * vertices[v2]) / (t1 + t2)));
                    //points.push_back(CPoint(interpolate(function, vertices[v1], f[v1], vertices[v2], f[v2])));
                    indices.push_back(indices.size());
                }
            }
            y += dy;
        }
        x += dx;
    }

    delete function;

    return new CMesh(points, indices);
}

CMesh* CPointSet::reconstruct(const CReconstructParameter& parameter) const {
    float radius_ = parameter.m_radius;
    float epsilon = parameter.m_epsilon;
    
    return marchingCubes(50, 50, 50, radius_, epsilon);

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
    //glPointSize(5);
    glBindVertexArray(m_vao);
    glDrawArrays(GL_POINTS, 0, m_points.size());
    glBindVertexArray(0);
}