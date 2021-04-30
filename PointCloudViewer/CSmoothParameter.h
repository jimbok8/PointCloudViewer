#ifndef SMOOTH_PARAMETER_H
#define SMOOTH_PARAMETER_H

class CSmoothParameter {
public:
    int m_k;
    float m_sharpnessAngle;
    CSmoothParameter(const int k, const float sharpnessAngle);
    ~CSmoothParameter();
};

#endif