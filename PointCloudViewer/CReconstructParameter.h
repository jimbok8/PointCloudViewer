#ifndef RECONSTRUCT_PARAMETER_H
#define RECONSTRUCT_PARAMETER_H

class CReconstructParameter {
public:
    int m_iterationNumber;
    float m_maximumFacetLength, m_radius, m_epsilon;
    CReconstructParameter(const int iterationNumber, const float maximumFacetLength, const float radius, const float epsilon);
    ~CReconstructParameter();
};

#endif