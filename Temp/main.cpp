#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>

static std::vector<Eigen::Vector3f> a, b;

static void read(const std::string& path, std::vector<Eigen::Vector3f>& points) {
    std::ifstream fin(path);
    float x, y, z;
    while (fin >> x >> y >> z)
        points.emplace_back(x, y, z);
}

int main() {
    read("../Test/temp.txt", a);
    read("../Test/tmp.txt", b);
    for (int i = 0; i < a.size(); i++)
        if ((a[i] - b[i]).squaredNorm() > 1e-6)
            std::cout << i << ':' << std::endl << a[i] << std::endl << b[i] << std::endl << std::endl;
    return 0;
}