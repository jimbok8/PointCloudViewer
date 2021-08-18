#include "CRenderer.h"

CRenderer::CRenderer(const int width, const int height, const COLORREF backgroundColor) :
    m_width(width),
    m_height(height),
    m_backgroundColor(backgroundColor) {
    m_image = new unsigned char[width * height * 3];

    zBuffer = (ZBuffer*)malloc(sizeof(ZBuffer));
    zBuffer->xsize = width;
    zBuffer->ysize = height;
    zBuffer->bufsize = width * height * sizeof(SrfZBufItem);
    zBuffer->buf = (SrfZBufItem*)malloc(zBuffer->bufsize);
    zBuffer->cleardata = (SrfZBufItem*)calloc(zBuffer->bufsize, 1);
    zBuffer->options = 0;

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
    warper->options = 0;
    // set the default view frustum (similar to gluPerspective): angle of field of view, 
    // aspect ratio, near and far clipping planes
    wrpSetFrustum(warper, 45.f, 1.f, 3.f, 14000.f);
    // WrpSetViewport(sct->warper, sct->fWidth, sct->fHeight);
    wrpSetViewport(warper, width, height);

    shader = (Shader*)malloc(sizeof(Shader));
    shader->wrp = warper;
    shader->frameBuffer = frameBuffer;
    //shader->lsl = lsl;
    shader->options = 0;
    shader->diffColor = SHD_DIFFCOL_DEFAULT;
    //shader->shdLightSample = shdLightSamplePhong_R;

    shader->shdUserShadingData = NULL;
    //shader->shdUserShadingFunction = NULL;

    // color sets for debugging
    shader->dbgcolors[0] = 0x000000ff;
    shader->dbgcolors[1] = 0x0000ff00;
    shader->dbgcolors[2] = 0x00ff0000;

    sqrt_3 = (float)sqrt(3.0f);
    sqrt_2 = (float)sqrt(2.0f);
}

CRenderer::~CRenderer() {
    delete[] m_image;
}

unsigned char* CRenderer::getPixelPtr(const int x, const int y) const {
    return m_image + ((m_height - y - 1) * m_width + x) * 3;
}

void CRenderer::init() {
    unsigned char r = GetRValue(m_backgroundColor);
    unsigned char g = GetGValue(m_backgroundColor);
    unsigned char b = GetBValue(m_backgroundColor);
    int index = 0;
    for (int i = 0; i < m_height; i++)
        for (int j = 0; j < m_width; j++) {
            unsigned char* p = getPixelPtr(j, i);
            *p = r;
            *(p + 1) = g;
            *(p + 2) = b;
        }
}

const unsigned char* CRenderer::getImage() const {
    return m_image;
}

void CRenderer::render() {
    init();

    MyDataTypes::TransformationMatrix16f convertedTransformation;
    MtrUtil::MtrTranspose4x4f(m_sceneViewMatrix, convertedTransformation);

    wrpSetTrafo(warper, convertedTransformation);

	zbfPrepareForWriting(zBuffer);
    
    wrpSetTrafoVars(warper);

    int bbox[4];
    for (CSurfel& surfel : m_surfels)
        wrpProjectSample(&surfel, bbox);

    bbox[0] = bbox[1] = 0;
    bbox[2] = iWidth - 1;
    bbox[3] = iHeight - 1;
    int magfactor = (iWidth > pWidth ? iWidth / pWidth : 1);

    // shade and copy to display image
    shdShadeZBuffer(shader, magfactor, bbox, m_backgroundColor);
}