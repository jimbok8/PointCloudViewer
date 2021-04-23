#ifndef CANDIDATE_H
#define CANDIDATE_H

#include <cmath>

#include <Eigen/Dense>

class CCandidate {
private:
    const float BETA = acos(-1) / 6.0f;
    int m_source, m_target, m_opposite, m_index;
    float m_plausibility;
    Eigen::Vector3f m_normal;

public:
    CCandidate();
    CCandidate(const int source, const int target, const int opposite, const int index, const Eigen::Vector3f& normal, const float beta, const float radius);
    ~CCandidate();
    int getSource() const;
    int getTarget() const;
    int getOpposite() const;
    int getIndex() const;
    Eigen::Vector3f getNormal() const;
    bool empty() const;
    bool operator<(const CCandidate& candidate) const;
    void operator=(const CCandidate& candidate);
};

#endif