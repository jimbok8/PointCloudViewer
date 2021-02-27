#include "Mesh.h"

Mesh::Mesh(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
    for (Vertex& vertex : vertices)
        points->push_back(pcl::PointXYZ(vertex.position.x, vertex.position.y, vertex.position.z));

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(points);
    
    unsigned int num = 0;
    std::vector<unsigned int> mapping(vertices.size(), UINT_MAX);
    for (int i = 0; i < vertices.size(); i++)
        if (mapping[i] == UINT_MAX) {
            std::vector<int> indices;
            std::vector<float> distances;
            tree->radiusSearch(pcl::PointXYZ(vertices[i].position.x, vertices[i].position.y, vertices[i].position.z), 1e-6, indices, distances);
            for (int index : indices)
                mapping[index] = num;
            num++;
            this->vertices.push_back(vertices[i]);
        }
    for (int index : indices)
        this->indices.push_back(mapping[index]);
    std::cout << vertices.size() - this->vertices.size() << " duplicate point(s) reduced" << std::endl;

    calculateNormals();

    unsigned vbo, ebo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(Vertex), &(this->vertices[0]), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->indices.size() * sizeof(unsigned int), &(this->indices[0]), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    glBindVertexArray(0);
}

Mesh::~Mesh() {}

void Mesh::calculateNormals() {
    std::vector<std::vector<glm::vec3>> normals(vertices.size());
    std::vector<std::vector<float>> weights(vertices.size());
    for (auto iter = indices.begin(); iter != indices.end(); ) {
        int x = *(iter++);
        int y = *(iter++);
        int z = *(iter++);
        glm::vec3 a = vertices[y].position - vertices[x].position;
        glm::vec3 b = vertices[z].position - vertices[y].position;
        glm::vec3 normal = glm::normalize(glm::cross(a, b));
        normals[x].push_back(normal);
        normals[y].push_back(normal);
        normals[z].push_back(normal);
    }

    for (int i = 0; i < vertices.size(); i++)
        if (!normals[i].empty()) {
            vertices[i].normal = glm::normalize(std::accumulate(normals[i].begin(), normals[i].end(), glm::vec3(0.0f, 0.0f, 0.0f)) / (float) normals[i].size());
            //if (glm::dot(vertices[i].normal, glm::vec3(0.0f, 0.0f, 100.0f) - vertices[i].position) < 0)
                //vertices[i].normal = -vertices[i].normal;
        }
}

void Mesh::render() {
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}