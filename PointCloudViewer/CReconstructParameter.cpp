#include "CReconstructParameter.h"

CReconstructParameter::CReconstructParameter(const int iterationNumber, const float maximumFacetLength, const float radius, const float epsilon) :
    m_iterationNumber(iterationNumber),
    m_maximumFacetLength(maximumFacetLength),
    m_radius(radius),
    m_epsilon(epsilon) {}

CReconstructParameter::~CReconstructParameter() {}