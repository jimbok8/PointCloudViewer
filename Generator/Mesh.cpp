#include "Mesh.h"

Mesh::Mesh(const std::string& path) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_GenSmoothNormals);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        std::cerr << "Failed to load model:" << std::endl << importer.GetErrorString() << std::endl;
        return;
    }

    processNode(scene->mRootNode, scene);

    float xMin, xMax, yMin, yMax, zMin, zMax;
    xMin = yMin = zMin = FLT_MAX;
    xMax = yMax = zMax = -FLT_MAX;
    for (const Vertex& vertex : vertices) {
        xMin = std::min(xMin, vertex.position.x);
        xMax = std::max(xMax, vertex.position.x);
        yMin = std::min(yMin, vertex.position.y);
        yMax = std::max(yMax, vertex.position.y);
        zMin = std::min(zMin, vertex.position.z);
        zMax = std::max(zMax, vertex.position.z);
    }

    glm::vec3 center((xMin + xMax) / 2.0f, (yMin + yMax) / 2.0f, (zMin + zMax) / 2.0f);
    for (Vertex& vertex : vertices)
        vertex.position -= center;

    unsigned vbo, ebo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    glBindVertexArray(0);
}

Mesh::~Mesh() {}

void Mesh::processNode(const aiNode* node, const aiScene* scene) {
    for (unsigned int i = 0; i < node->mNumMeshes; i++)
        processMesh(scene->mMeshes[node->mMeshes[i]]);
    for (unsigned int i = 0; i < node->mNumChildren; i++)
        processNode(node->mChildren[i], scene);
}

void Mesh::processMesh(const aiMesh* mesh) {
    unsigned int size = vertices.size();
    for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
        glm::vec3 position(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
        glm::vec3 normal(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z);
        vertices.push_back(Vertex(position, normal));
    }

    for (unsigned int i = 0; i < mesh->mNumFaces; i++)
        for (unsigned int j = 0; j < mesh->mFaces[i].mNumIndices; j++)
            indices.push_back(mesh->mFaces[i].mIndices[j] + size);
}

std::vector<Vertex> Mesh::getVertices() const {
    return vertices;
}

std::vector<unsigned int> Mesh::getIndices() const {
    return indices;
}

void Mesh::render() const {
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}