#include "CSphereFunction.h"

CSphereFunction::CSphereFunction(const Eigen::Vector3f& center, const float radius) :
    CFunction(),
    m_center(center),
    m_radius2(radius * radius) {}

CSphereFunction::~CSphereFunction() {}

float CSphereFunction::f(const Eigen::Vector3f& x) const {
    return (x - m_center).squaredNorm() - m_radius2;
}