#ifndef RECONSTRUCT_PARAMETER_H
#define RECONSTRUCT_PARAMETER_H

class CReconstructParameter {
public:
    int m_iterationNumber;
    float m_maximumRadius;
    CReconstructParameter(const int iterationNumber, const float maximumRadius);
    ~CReconstructParameter();
};

#endif