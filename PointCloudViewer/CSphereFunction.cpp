#include "CSphereFunction.h"

CSphereFunction::CSphereFunction(const Eigen::Vector3f& center, const float radius) :
    CFunction(),
    center(center),
    radius2(radius * radius) {}

CSphereFunction::~CSphereFunction() {}

float CSphereFunction::f(const Eigen::Vector3f& x) const {
    return (x - center).squaredNorm() - radius2;
}