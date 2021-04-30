#ifndef RESAMPLE_PARAMETER_H
#define RESAMPLE_PARAMETER_H

class CResampleParameter {
public:
    float m_sharpnessAngle, m_edgeSensitivity, m_neighborRadius;
    int m_size;
    CResampleParameter(const float sharpnessAngle, const float edgeSensitivity, const float neighborRadius, const int size);
    ~CResampleParameter();
};

#endif