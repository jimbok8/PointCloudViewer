#include "CSurfel.h"

CSurfel::CSurfel(const Vector3D& position, const Vector3D& normal, const float radius, const COLORREF diffuseColor, const COLORREF specularColor) :
    m_position(position),
    m_normal(normal),
    m_radius(radius),
    m_diffuseColor(diffuseColor),
    m_specularColor(specularColor) {}

CSurfel::~CSurfel() {}

Vector3D CSurfel::getPosition() const {
    return m_position;
}

Vector3D CSurfel::getNormal() const {
    return m_normal;
}

float CSurfel::getRadius() const {
    return m_radius;
}

COLORREF CSurfel::getDiffuseColor() const {
    return m_diffuseColor;
}

COLORREF CSurfel::getSpecularColor() const {
    return m_specularColor;
}