#include "CPoint.h"

CPoint::CPoint(const Eigen::Vector3f& position, const Eigen::Vector3f& u, const Eigen::Vector3f& v) :
    m_position(position),
    m_u(u),
    m_v(v) {}

CPoint::~CPoint() {}
