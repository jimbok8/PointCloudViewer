#ifndef RECONSTRUCT_PARAMETER_H
#define RECONSTRUCT_PARAMETER_H

class CReconstructParameter {
public:
    int m_iterationNumber;
    float m_maximumFacetLength;
    CReconstructParameter(const int iterationNumber, const float maximumFacetLength);
    ~CReconstructParameter();
};

#endif