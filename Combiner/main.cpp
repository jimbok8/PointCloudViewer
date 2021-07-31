#include <string>
#include <vector>
#include <fstream>

#include <Eigen/Dense>

#include "CPoint.h"

int main() {
    std::vector<CPoint> points;
    for (int i = 49; i < 50; i++) {
        std::ifstream fin("../data/qjhdl/hd/" + std::to_string(i) + "thFrame.pcd");

        std::string s;
        for (int j = 0; j < 11; j++)
            getline(fin, s);

        float x, y, z, weight, r, g, b;
        while (fin >> x >> y >> z >> weight >> r >> g >> b) {
            points.push_back(CPoint(Eigen::Vector3f(x, y, z)));

            getline(fin, s);
        }
    }

    std::ofstream fout("../data/qjhdl/hd49.dat");
    for (const CPoint& point : points)
        fout << "v " << point.m_position.x() << ' ' << point.m_position.y() << ' ' << point.m_position.z() << " 0 0" << std::endl;

    return 0;
}