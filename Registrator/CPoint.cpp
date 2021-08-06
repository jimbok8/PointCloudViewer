#include "CPoint.h"

CPoint::CPoint() :
    m_position(0.0f, 0.0f, 0.0f),
    m_normal(0.0f, 0.0f, 0.0f) {}

CPoint::CPoint(const Eigen::Vector3f& position) :
    m_position(position),
    m_normal(0.0f, 0.0f, 0.0f) {}

CPoint::CPoint(const Eigen::Vector3f& position, const Eigen::Vector3f& normal) :
    m_position(position),
    m_normal(normal) {}

CPoint::CPoint(const Eigen::Vector3f& position, const Eigen::Vector3f& normal, const Eigen::Vector3f& u, const Eigen::Vector3f& v) :
    m_position(position),
    m_normal(normal),
    m_u(u),
    m_v(v) {}

CPoint::~CPoint() {}
