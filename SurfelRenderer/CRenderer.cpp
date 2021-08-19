#include "CRenderer.h"

CRenderer::CRenderer(const std::vector<CSurfel>& surfels, const int width, const int height, const COLORREF backgroundColor) :
    m_surfels(surfels),
    m_width(width),
    m_height(height),
    m_backgroundColor(backgroundColor) {
    m_image = new unsigned char[width * height * 3];
    MtrUtil::MtrUnity4x4f(m_cameraPosition.scaleTranslationMatrix);
    m_cameraPosition.scaleTranslationMatrix[14] = -1000.0f;
    MtrUtil::MtrUnity4x4f(m_cameraPosition.rotationMatrix);

    zBuffer = (ZBuffer*)malloc(sizeof(ZBuffer));
    zBuffer->xsize = width;
    zBuffer->ysize = height;
    zBuffer->bufsize = width * height * sizeof(SrfZBufItem);
    zBuffer->buf = (SrfZBufItem*)malloc(zBuffer->bufsize);
    zBuffer->cleardata = (SrfZBufItem*)calloc(zBuffer->bufsize, 1);

    // default values for surface splatting
    zBuffer->cutoffRadius = 1.f;
    zBuffer->constThreshold = 0.4f;
    zBuffer->distThreshold = 2.0f;
    zBuffer->angleTrheshold = 8.f;

    zBuffer->LUTsize = 1024;
    zBuffer->filterLUT = (float*)malloc(sizeof(float) * zBuffer->LUTsize);
    float d = 1.f / ((float)zBuffer->LUTsize - 1.f);
    // table will be indexed by squared distance
    for (int i = 0; i < zBuffer->LUTsize; i++)
    {
        zBuffer->filterLUT[i] = (float)exp(-(float)i * d * zBuffer->cutoffRadius * zBuffer->cutoffRadius);
    }

    // init clear data
    for (int i = 0; i < width * height; i++)
    {
        // initialize with a 'large' value. to improve...
        zBuffer->cleardata[i].z = 100000.f;
        zBuffer->cleardata[i].zMin = 1000000.0f;
        zBuffer->cleardata[i].zMax = 1000000.0f;
    }

    // allocate and initialize the warper
    warper = (Warper*)malloc(sizeof(Warper));
    warper->zbf = zBuffer;
    // set the default view frustum (similar to gluPerspective): angle of field of view, 
    // aspect ratio, near and far clipping planes
    wrpSetFrustum(warper, 30.f, 1.f, 10.f, 100000.f);
    wrpSetViewport(warper, width, height);

    shader = (Shader*)malloc(sizeof(Shader));
    shader->wrp = warper;
    shader->frameBuffer = m_image;
}

CRenderer::~CRenderer() {
    delete[] m_image;
    delete warper;
    delete zBuffer;
    delete shader;
}

void CRenderer::init() {
    unsigned char r = GetRValue(m_backgroundColor);
    unsigned char g = GetGValue(m_backgroundColor);
    unsigned char b = GetBValue(m_backgroundColor);
    int index = 0;
    for (int j = 0; j < m_height; j++)
        for (int i = 0; i < m_width; i++) {
            unsigned char* p = getPixelPtr(m_image, m_width, m_height, i, j);
            *p = r;
            *(p + 1) = g;
            *(p + 2) = b;
        }

    memmove(zBuffer->buf, zBuffer->cleardata, zBuffer->bufsize);
}

int CRenderer::getWidth() const {
    return m_width;
}

int CRenderer::getHeight() const {
    return m_height;
}

const unsigned char* CRenderer::getImage() const {
    return m_image;
}

void CRenderer::scale(const float dScaleX, const float dScaleY, const float dScaleZ) {
    m_cameraPosition.scaleTranslationMatrix[0] *= dScaleX;
    m_cameraPosition.scaleTranslationMatrix[5] *= dScaleY;
    m_cameraPosition.scaleTranslationMatrix[10] *= dScaleZ;
}

void CRenderer::translate(const float dx, const float dy, const float dz) {
    m_cameraPosition.scaleTranslationMatrix[12] += dx;
    m_cameraPosition.scaleTranslationMatrix[13] += dy;
    m_cameraPosition.scaleTranslationMatrix[14] += dz;
}

void CRenderer::rotate(const float dAngle, const float x, const float y, const float z) {
    MyDataTypes::TransformationMatrix16f userRotation, finalRotation;

    MtrUtil::MtrCreateRotation4x4fc(dAngle, x, y, z, userRotation);
    MtrUtil::MtrMult4x4f(userRotation, m_cameraPosition.rotationMatrix, finalRotation);
    MtrUtil::MtrCopy4x4f(finalRotation, m_cameraPosition.rotationMatrix);
}

void CRenderer::render() {
    init();

    MyDataTypes::TransformationMatrix16f transformation, convertedTransformation;
    MtrUtil::MtrMult4x4f(m_cameraPosition.scaleTranslationMatrix, m_cameraPosition.rotationMatrix, transformation);
    MtrUtil::MtrTranspose4x4f(transformation, convertedTransformation);
    convertedTransformation[8] *= -1.0f;
    convertedTransformation[9] *= -1.0f;
    convertedTransformation[10] *= -1.0f;
    convertedTransformation[11] *= -1.0f;

    wrpSetTrafo(warper, convertedTransformation);

	zbfPrepareForWriting(zBuffer);
    
    wrpSetTrafoVars(warper);

    int bbox[4];
    for (CSurfel& surfel : m_surfels)
        wrpProjectSample(&surfel, bbox);

    bbox[0] = bbox[1] = 0;
    bbox[2] = m_width - 1;
    bbox[3] = m_height - 1;
    shdShadeZBuffer(shader, 1, bbox, m_backgroundColor);
}