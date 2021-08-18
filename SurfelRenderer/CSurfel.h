#ifndef SURFEL_H
#define SURFEL_H

#include <Windows.h>

#include "Vector3D.h"

class CSurfel {
private:
    Vector3D m_position, m_normal;
    float m_radius;
    COLORREF m_diffuseColor, m_specularColor;

public:
    CSurfel(const Vector3D& position, const Vector3D& normal, const float radius, const COLORREF diffuseColor, const COLORREF specularColor);
    ~CSurfel();
    Vector3D getPosition() const;
    Vector3D getNormal() const;
    float getRadius() const;
    COLORREF getDiffuseColor() const;
    COLORREF getSpecularColor() const;
};

#endif