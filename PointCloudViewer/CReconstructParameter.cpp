#include "CReconstructParameter.h"

CReconstructParameter::CReconstructParameter(const int iterationNumber, const float maximumFacetLength) :
    m_iterationNumber(iterationNumber),
    m_maximumFacetLength(maximumFacetLength) {}

CReconstructParameter::~CReconstructParameter() {}