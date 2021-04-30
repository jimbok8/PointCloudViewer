#include "CSmoothParameter.h"

CSmoothParameter::CSmoothParameter(const int k, const float sharpnessAngle) :
    m_k(k),
    m_sharpnessAngle(sharpnessAngle) {}

CSmoothParameter::~CSmoothParameter() {}