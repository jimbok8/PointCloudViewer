#include "PointSet.h"

PointSet::PointSet() {}

PointSet::PointSet(std::vector<Vertex> &vertices) {
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

PointSet::~PointSet() {}

std::vector<Point> PointSet::toPoint(std::vector<Vertex>& vertices) {
    std::vector<Point> ans;
    for (Vertex& vertex : vertices)
        ans.push_back(Point(vertex.position.x, vertex.position.y, vertex.position.z));

    return ans;
}

std::vector<Vertex> PointSet::fromPoint(std::vector<Point>& points) {
    std::vector<Vertex> ans;
    for (Point& point : points)
        ans.push_back(Vertex(point.x(), point.y(), point.z()));

    return ans;
}

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
    CGAL::pca_estimate_normals<CGAL::Sequential_tag>(points, 50, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointNormal>()).normal_map(CGAL::Second_of_pair_property_map<PointNormal>()));
    CGAL::mst_orient_normals(points, 50, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointNormal>()).normal_map(CGAL::Second_of_pair_property_map<PointNormal>()));
    vertices = fromPointNormal(points);
}

std::vector<Vertex> PointSet::getVertices() const {
    return vertices;
}

PointSet PointSet::simplify(double epsilon) {
    std::vector<Point> points = toPoint(vertices);
    int t = points.size();
    points.erase(CGAL::grid_simplify_point_set(points, epsilon), points.end());
    std::vector<Point>(points).swap(points);
    std::cout << t - points.size() << " point(s) removed after simplification." << std::endl;

    std::vector<Vertex> vertices = fromPoint(points);
    return PointSet(vertices);
}

PointSet PointSet::upsample(double sharpnessAngle, double edgeSensitivity, double neighborRadius, int size) {
    std::vector<PointNormal> points = toPointNormal(this->vertices);
    CGAL::edge_aware_upsample_point_set<CGAL::Parallel_if_available_tag>(points, std::back_inserter(points),
        CGAL::parameters::
        point_map(CGAL::First_of_pair_property_map<PointNormal>()).
        normal_map(CGAL::Second_of_pair_property_map<PointNormal>()).
        sharpness_angle(sharpnessAngle).
        edge_sensitivity(edgeSensitivity).
        neighbor_radius(neighborRadius).
        number_of_output_points(size));
    std::vector<Vertex> vertices = fromPointNormal(points);

    return PointSet(vertices);
}

PointSet PointSet::smooth(int k) {
    //std::vector<Point> points = toPoint(vertices);
    //CGAL::jet_smooth_point_set<CGAL::Sequential_tag>(points, k);
    std::vector<PointNormal> points = toPointNormal(vertices);
    CGAL::bilateral_smooth_point_set<CGAL::Sequential_tag>(points, k, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointNormal>()).normal_map(CGAL::Second_of_pair_property_map<PointNormal>()));
    std::vector<Vertex> vertices = fromPointNormal(points);

    //std::vector<Vertex> vertices = fromPoint(points);
    return PointSet(vertices);
}

Mesh PointSet::reconstruct(double maximumFacetLength) {
    /*std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    if (type == 0) {
        std::vector<Point> points = toPoint(this->vertices);
        std::vector<std::array<std::size_t, 3>> facets;
        CGAL::advancing_front_surface_reconstruction(points.begin(), points.end(), std::back_inserter(facets));

        vertices = this->vertices;
        for (std::array<std::size_t, 3>& facet : facets)
            for (size_t index : facet)
                indices.push_back(index);
    }
    else if (type == 1) {
        std::vector<Point> points = toPoint(this->vertices);
        CGAL::Scale_space_surface_reconstruction_3<Kernel> reconstruct(points.begin(), points.end());
        reconstruct.increase_scale(4, CGAL::Scale_space_reconstruction_3::Jet_smoother<Kernel>());
        reconstruct.reconstruct_surface(CGAL::Scale_space_reconstruction_3::Advancing_front_mesher<Kernel>(1.0));

        vertices = this->vertices;
        for (auto iter = reconstruct.facets_begin(); iter != reconstruct.facets_end(); iter++)
            for (unsigned int index : *iter)
                indices.push_back(index);
    }
    else if (type == 2) {
        std::vector<PointNormal> points = toPointNormal(this->vertices);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (Vertex& vertex : this->vertices)
            cloud->push_back(pcl::PointXYZ(vertex.position.x, vertex.position.y, vertex.position.z));
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        SurfaceMesh mesh;
        CGAL::poisson_surface_reconstruction_delaunay(points.begin(), points.end(), CGAL::First_of_pair_property_map<PointNormal>(), CGAL::Second_of_pair_property_map<PointNormal>(), mesh, 1.0);

        std::map<VertexIndex, unsigned int> mapping;
        for (VertexIndex vertex : mesh.vertices()) {
            Point point = mesh.point(vertex);
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
        pcl::PointCloud<pcl::PointNormal>::Ptr points(new pcl::PointCloud<pcl::PointNormal>);
        for (Vertex& vertex : this->vertices)
            points->push_back(pcl::PointNormal(vertex.position.x, vertex.position.y, vertex.position.z, vertex.normal.x, vertex.normal.y, vertex.normal.z));
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
        tree->setInputCloud(points);
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
            gpt.setInputCloud(points);
            gpt.setSearchMethod(tree);
            gpt.reconstruct(*mesh);
        }
        else if (type == 4) {
            pcl::MarchingCubesHoppe<pcl::PointNormal> mc;
            mc.setGridResolution(64, 64, 64);
            mc.setInputCloud(points);
            mc.setSearchMethod(tree);
            mc.reconstruct(*mesh);
        }
        else if (type == 5) {
            pcl::MarchingCubesRBF<pcl::PointNormal> mc;
            mc.setGridResolution(64, 64, 64);
            mc.setInputCloud(points);
            mc.setSearchMethod(tree);
            mc.reconstruct(*mesh);
        }

        fromPCLPointCloud2(mesh->cloud, *points);
        for (pcl::PointNormal& point : *points)
            vertices.push_back(Vertex(point.x, point.y, point.z));
        for (pcl::Vertices& polygon : mesh->polygons)
            for (unsigned int vertex : polygon.vertices)
                indices.push_back(vertex);
    }*/

    std::vector<Point> points = toPoint(this->vertices);
    CGAL::Scale_space_surface_reconstruction_3<Kernel> reconstruct(points.begin(), points.end());
    reconstruct.increase_scale(4, CGAL::Scale_space_reconstruction_3::Jet_smoother<Kernel>());
    reconstruct.reconstruct_surface(CGAL::Scale_space_reconstruction_3::Advancing_front_mesher<Kernel>(maximumFacetLength));

    std::vector<Vertex> vertices = this->vertices;
    std::vector<unsigned int> indices;
    for (auto iter = reconstruct.facets_begin(); iter != reconstruct.facets_end(); iter++)
        for (unsigned int index : *iter)
            indices.push_back(index);
    std::cout << indices.size() / 3 << " facet(s) generated by reconstruction." << std::endl;

    return Mesh(vertices, indices);
}

void PointSet::render() {
    glPointSize(5);
    glBindVertexArray(vao);
    glDrawArrays(GL_POINTS, 0, vertices.size());
    glBindVertexArray(0);
}