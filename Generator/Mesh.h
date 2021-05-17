#ifndef MESH_H
#define MESH_H

#include <algorithm>
#include <cfloat>
#include <string>
#include <vector>
#include <iostream>

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "Vertex.h"

class Mesh {
private:
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    unsigned int vao;
    void processNode(const aiNode* node, const aiScene* scene);
    void processMesh(const aiMesh* mesh);

public:
    Mesh(const std::string& path);
    ~Mesh();
    std::vector<Vertex> getVertices() const;
    std::vector<unsigned int> getIndices() const;
    void render() const;
};

#endif