#include "PointSet.h"

PointSet::PointSet(std::vector<Vertex>& vertices) {
    this->vertices = vertices;

    calculateNormals();

    unsigned int vbo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(Vertex), this->vertices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    glBindVertexArray(0);
}

PointSet::~PointSet() {};

std::vector<PointNormal> PointSet::toPointNormal(std::vector<Vertex>& vertices) {
    std::vector<PointNormal> ans;
    for (Vertex& vertex : vertices)
        ans.push_back(std::make_pair(Point(vertex.position.x, vertex.position.y, vertex.position.z), Vector(vertex.normal.x, vertex.normal.y, vertex.normal.z)));

    return ans;
}

std::vector<Vertex> PointSet::fromPointNormal(std::vector<PointNormal>& points) {
    std::vector<Vertex> ans;
    for (PointNormal& point : points)
        ans.push_back(Vertex(point.first.x(), point.first.y(), point.first.z(), point.second.x(), point.second.y(), point.second.z()));

    return ans;
}

void PointSet::calculateNormals() {
    std::vector<PointNormal> points = toPointNormal(vertices);
    CGAL::jet_estimate_normals<CGAL::Sequential_tag>(points, 50, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointNormal>()).normal_map(CGAL::Second_of_pair_property_map<PointNormal>()));
    CGAL::mst_orient_normals(points, 50, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointNormal>()).normal_map(CGAL::Second_of_pair_property_map<PointNormal>()));
    vertices = fromPointNormal(points);
}

int PointSet::size() {
    return vertices.size();
}

std::vector<Vertex> PointSet::getVertices() {
    return vertices;
}

std::vector<PointSet> PointSet::divide(double epsilon, float threshold) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
    for (unsigned int i = 0; i < this->vertices.size(); i++)
        points->push_back(pcl::PointXYZ(this->vertices[i].position.x, this->vertices[i].position.y, this->vertices[i].position.z));
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(points);

    bool* flag = new bool[this->vertices.size()];
    memset(flag, false, sizeof(bool) * this->vertices.size());
    std::vector<PointSet> ans;
    for (int i = 0; i < this->vertices.size(); i++)
        if (!flag[i]) {
            std::queue<int> q;
            std::vector<Vertex> vertices;
            flag[i] = true;
            q.push(i);
            vertices.push_back(this->vertices[i]);
            while (!q.empty()) {
                int now = q.front();
                q.pop();
                std::vector<int> indices;
                std::vector<float> distances;
                tree->radiusSearch(pcl::PointXYZ(this->vertices[now].position.x, this->vertices[now].position.y, this->vertices[now].position.z), epsilon, indices, distances);
                for (int j : indices)
                    if (!flag[j]) {
                        flag[j] = true;
                        q.push(j);
                        vertices.push_back(this->vertices[j]);
                    }
            }
            ans.push_back(PointSet(vertices));
        }

    delete[] flag;
    return ans;
}

void PointSet::render() {
    glPointSize(2);
    glBindVertexArray(vao);
    glDrawArrays(GL_POINTS, 0, vertices.size());
    glBindVertexArray(0);
}
