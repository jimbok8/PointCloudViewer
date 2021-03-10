#include "PointSet.h"

PointSet::PointSet(std::vector<Vertex>& vertices) {
    this->vertices = vertices;

    calculateSpacing();
    calculateNormals();

    unsigned int vbo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(Vertex), &(this->vertices[0]), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    glBindVertexArray(0);
}

PointSet::~PointSet() {};

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

void PointSet::calculateSpacing() {
    std::vector<Point> points = toPoint(vertices);
    spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, 24);
}

void PointSet::calculateNormals() {
    std::vector<PointNormal> points = toPointNormal(vertices);
    CGAL::jet_estimate_normals<CGAL::Sequential_tag>(points, 24, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointNormal>()).normal_map(CGAL::Second_of_pair_property_map<PointNormal>()));
    vertices = fromPointNormal(points);
}

int PointSet::size() {
    return vertices.size();
}

std::vector<PointSet> PointSet::divide(double scale, float threshold) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (unsigned int i = 0; i < this->vertices.size(); i++)
        cloud->push_back(pcl::PointXYZ(this->vertices[i].position.x, this->vertices[i].position.y, this->vertices[i].position.z));

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    bool* flag = new bool[this->vertices.size()];
    memset(flag, false, sizeof(bool) * this->vertices.size());
    std::vector<PointSet> ans;
    for (unsigned int i = 0; i < this->vertices.size(); i++)
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
                tree->radiusSearch(pcl::PointXYZ(this->vertices[now].position.x, this->vertices[now].position.y, this->vertices[now].position.z), scale * spacing, indices, distances);
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

PointSet PointSet::removeOutliers(int k, double scale) {
    std::vector<Point> points = toPoint(vertices);
    int t = points.size();
    points.erase(CGAL::remove_outliers<CGAL::Sequential_tag>(points, k, CGAL::parameters::threshold_percent(100.0).threshold_distance(scale * spacing)), points.end());
    std::vector<Point>(points).swap(points);
    std::cout << t - points.size() << " point(s) are outliers." << std::endl;

    std::vector<Vertex> vertices = fromPoint(points);
    return PointSet(vertices);
}

PointSet PointSet::simplify(double scale) {
    std::vector<Point> points = toPoint(vertices);
    int t = points.size();
    points.erase(CGAL::grid_simplify_point_set(points, scale * spacing), points.end());
    std::vector<Point>(points).swap(points);
    std::cout << t - points.size() << " point(s) removed after simplification." << std::endl;

    std::vector<Vertex> vertices = fromPoint(points);
    return PointSet(vertices);
}

PointSet PointSet::smooth(int k) {
    std::vector<Point> points = toPoint(vertices);
    CGAL::jet_smooth_point_set<CGAL::Sequential_tag>(points, k);

    std::vector<Vertex> vertices = fromPoint(points);
    return PointSet(vertices);
}

Mesh PointSet::reconstruct(int type, int resolution) {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    if (type == 0) {
        std::vector<Point> points = toPoint(this->vertices);
        std::vector<std::array<std::size_t, 3>> facets;
        CGAL::advancing_front_surface_reconstruction(points.begin(), points.end(), std::back_inserter(facets));
    
        vertices = this->vertices;
        for (std::array<std::size_t, 3>&facet : facets)
            for (size_t index : facet)
                indices.push_back(index);
    }
    else if (type == 1) {
        std::vector<Point> points = toPoint(this->vertices);
        CGAL::Scale_space_surface_reconstruction_3<Kernel> reconstruct(points.begin(), points.end());
        reconstruct.increase_scale(4, CGAL::Scale_space_reconstruction_3::Jet_smoother<Kernel>());
        reconstruct.reconstruct_surface(CGAL::Scale_space_reconstruction_3::Advancing_front_mesher<Kernel>(5.0 * spacing));
        
        vertices = this->vertices;
        for (auto iter = reconstruct.facets_begin(); iter != reconstruct.facets_end(); iter++)
            for (unsigned int index : *iter)
                indices.push_back(index);
    } else if (type == 2) {
        std::vector<PointNormal> points = toPointNormal(this->vertices);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (Vertex& vertex : this->vertices)
            cloud->push_back(pcl::PointXYZ(vertex.position.x, vertex.position.y, vertex.position.z));
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        SurfaceMesh mesh;
        CGAL::poisson_surface_reconstruction_delaunay(points.begin(), points.end(), CGAL::First_of_pair_property_map<PointNormal>(), CGAL::Second_of_pair_property_map<PointNormal>(), mesh, 0.1 * spacing);

        std::map<VertexIndex, unsigned int> mapping;
        for (VertexIndex vertex : mesh.vertices()) {
            Point point = mesh.point(vertex);
            std::vector<int> indices;
            std::vector<float> distances;
            tree->nearestKSearch(pcl::PointXYZ(point.x(), point.y(), point.z()), 1, indices, distances);
            if (distances[0] < 0.5) {
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
    } else if (type == 3) {
        pcl::PointCloud<pcl::PointNormal>::Ptr points(new pcl::PointCloud<pcl::PointNormal>);
        for (Vertex& vertex : this->vertices)
            points->push_back(pcl::PointNormal(vertex.position.x, vertex.position.y, vertex.position.z, vertex.normal.x, vertex.normal.y, vertex.normal.z));

        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
        tree->setInputCloud(points);

        pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gpt;
        gpt.setSearchRadius(10.0 * spacing);
        gpt.setMu(5.0);
        gpt.setMaximumNearestNeighbors(200);
        gpt.setMaximumSurfaceAngle(M_PI / 2);
        gpt.setMinimumAngle(M_PI / 18);
        gpt.setMaximumAngle(M_PI * 2 / 3);
        gpt.setNormalConsistency(false);
        gpt.setInputCloud(points);
        gpt.setSearchMethod(tree);
        gpt.reconstruct(*mesh);

        fromPCLPointCloud2(mesh->cloud, *points);
        for (pcl::PointNormal& point : *points)
            vertices.push_back(Vertex(point.x, point.y, point.z));
        for (pcl::Vertices& polygon : mesh->polygons)
            for (unsigned int vertex : polygon.vertices)
                indices.push_back(vertex);
    } else if (type == 4) {
        pcl::PointCloud<pcl::PointNormal>::Ptr points(new pcl::PointCloud<pcl::PointNormal>);
        for (Vertex& vertex : this->vertices)
            points->push_back(pcl::PointNormal(vertex.position.x, vertex.position.y, vertex.position.z, vertex.normal.x, vertex.normal.y, vertex.normal.z));

        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
        tree->setInputCloud(points);

        pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

        pcl::MarchingCubesHoppe<pcl::PointNormal> mc;
        mc.setGridResolution(resolution, resolution, resolution);
        mc.setInputCloud(points);
        mc.setSearchMethod(tree);
        mc.reconstruct(*mesh);

        fromPCLPointCloud2(mesh->cloud, *points);
        for (pcl::PointNormal& point : *points)
            vertices.push_back(Vertex(point.x, point.y, point.z));
        for (pcl::Vertices& polygon : mesh->polygons)
            for (unsigned int vertex : polygon.vertices)
                indices.push_back(vertex);
    } else if (type == 5) {
        pcl::PointCloud<pcl::PointNormal>::Ptr points(new pcl::PointCloud<pcl::PointNormal>);
        for (Vertex& vertex : this->vertices)
            points->push_back(pcl::PointNormal(vertex.position.x, vertex.position.y, vertex.position.z, vertex.normal.x, vertex.normal.y, vertex.normal.z));

        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
        tree->setInputCloud(points);

        pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

        pcl::MarchingCubesRBF<pcl::PointNormal> mc;
        mc.setGridResolution(resolution, resolution, resolution);
        mc.setInputCloud(points);
        mc.setSearchMethod(tree);
        mc.reconstruct(*mesh);

        fromPCLPointCloud2(mesh->cloud, *points);
        for (pcl::PointNormal& point : *points)
            vertices.push_back(Vertex(point.x, point.y, point.z));
        for (pcl::Vertices& polygon : mesh->polygons)
            for (unsigned int vertex : polygon.vertices)
                indices.push_back(vertex);
    }
    std::cout << indices.size() / 3 << " facet(s) generated by reconstruction." << std::endl;
    
    return Mesh(vertices, indices);
}

void PointSet::render() {
    glPointSize(2);
    glBindVertexArray(vao);
    glDrawArrays(GL_POINTS, 0, vertices.size());
    glBindVertexArray(0);
}

void PointSet::save(const std::string& path) {
    std::ofstream fout(path);
    for (Vertex& vertex : vertices)
        fout << "v " << vertex.position.x << ' ' << vertex.position.y << ' ' << vertex.position.z << " 0 0" << std::endl;
}