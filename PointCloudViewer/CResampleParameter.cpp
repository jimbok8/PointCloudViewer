#include "CResampleParameter.h"

CResampleParameter::CResampleParameter(const float sharpnessAngle, const float edgeSensitivity, const float neighborRadius, const int size) :
    m_sharpnessAngle(sharpnessAngle),
    m_edgeSensitivity(edgeSensitivity),
    m_neighborRadius(neighborRadius),
    m_size(size) {}

CResampleParameter::~CResampleParameter() {}