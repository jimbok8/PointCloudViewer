#ifndef SIMPLIFY_PARAMETER_H
#define SIMPLIFY_PARAMETER_H

class CSimplifyParameter {
public:
    float m_epsilon;
    CSimplifyParameter(const float epsilon);
    ~CSimplifyParameter();
};

#endif