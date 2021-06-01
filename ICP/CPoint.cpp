#include "CPoint.h"

CPoint::CPoint(const Eigen::Vector3f& position) :
    m_position(position),
    m_normal(0.0f, 0.0f, 0.0f),
    m_color(0.0f, 0.0f, 0.0f) {}

CPoint::CPoint(const Eigen::Vector3f& position, const Eigen::Vector3f& normal) :
    m_position(position),
    m_normal(normal),
    m_color(0.0f, 0.0f, 0.0f) {}

CPoint::CPoint(const Eigen::Vector3f& position, const Eigen::Vector3f& normal, const Eigen::Vector3f& color) :
    m_position(position),
    m_normal(normal),
    m_color(color) {}

CPoint::~CPoint() {}
