#include "CCandidate.h"

CCandidate::CCandidate() :
    m_index(-1) {}

CCandidate::CCandidate(const int source, const int target, const int opposite, const int index, const Eigen::Vector3f& normal, const float beta, const float radius) :
    m_source(source),
    m_target(target),
    m_opposite(opposite),
    m_index(index),
    m_normal(normal) {
    m_plausibility = beta < BETA ? 1 / radius : -beta;
}

CCandidate::~CCandidate() {}

int CCandidate::getSource() const {
    return m_source;
}

int CCandidate::getTarget() const {
    return m_target;
}

int CCandidate::getOpposite() const {
    return m_opposite;
}

int CCandidate::getIndex() const {
    return m_index;
}

Eigen::Vector3f CCandidate::getNormal() const {
    return m_normal;
}

bool CCandidate::empty() const {
    return m_index < 0;
}

bool CCandidate::operator<(const CCandidate& candidate) const {
    return m_plausibility < candidate.m_plausibility;
}

void CCandidate::operator=(const CCandidate& candidate) {
    m_source = candidate.m_source;
    m_target = candidate.m_target;
    m_opposite = candidate.m_opposite;
    m_index = candidate.m_index;
    m_plausibility = candidate.m_plausibility;
    m_normal = candidate.m_normal;
}