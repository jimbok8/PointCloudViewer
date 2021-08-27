#include "CRenderer.h"

CRenderer::CRenderer(const int numSurfels, const Surfel* surfels, const int width, const int height, const unsigned char backgroundR, const unsigned char backgroundG, const unsigned char backgroundB, const bool useGpu) :
	m_numSurfels(numSurfels),
    m_width(width),
    m_height(height),
    m_backgroundR(backgroundR),
	m_backgroundG(backgroundG),
	m_backgroundB(backgroundB),
	m_useGpu(useGpu) {
    MtrUnity4x4f(m_cameraPosition.scaleTranslationMatrix);
    m_cameraPosition.scaleTranslationMatrix[14] = -1000.0f;
    MtrUnity4x4f(m_cameraPosition.rotationMatrix);

	m_image = new unsigned char[width * height * 3];

	// allocate and initialize the warper
	m_warper = new Warper;
	// set the default view frustum (similar to gluPerspective): angle of field of view, 
	// aspect ratio, near and far clipping planes
	setFrustum(30.f, 1.f, 10.f, 100000.f);

	m_zBufferProperty = new ZBufferProperty;
	m_zBufferProperty->bufsize = width * height;
	m_zBufferProperty->LUTsize = 1024;

	// default values for surface splatting
	m_zBufferProperty->cutoffRadius = 1.0f;
	m_zBufferProperty->constThreshold = 0.4f;
	m_zBufferProperty->distThreshold = 2.0f;
	m_zBufferProperty->angleTrheshold = 8.0f;

	m_zBuffer = new ZBufferItem[width * height];

	// init clear data
	m_clearData = new ZBufferItem[width * height];
	for (int i = 0; i < width * height; i++) {
		// initialize with a 'large' value. to improve...
		m_clearData[i].zMin = FLT_MAX;
		m_clearData[i].zMax = -FLT_MAX;
		m_clearData[i].w = 0.0f;
		m_clearData[i].c[0] = 0.0f;
		m_clearData[i].c[1] = 0.0f;
		m_clearData[i].c[2] = 0.0f;
		m_clearData[i].n[0] = 0.0f;
		m_clearData[i].n[1] = 0.0f;
		m_clearData[i].n[2] = 0.0f;
	}

	m_filterLUT = new float[m_zBufferProperty->LUTsize];
	float d = 1.f / ((float)m_zBufferProperty->LUTsize - 1.f);
	// table will be indexed by squared distance
	for (int i = 0; i < m_zBufferProperty->LUTsize; i++)
		m_filterLUT[i] = (float)exp(-(float)i * d * m_zBufferProperty->cutoffRadius * m_zBufferProperty->cutoffRadius);

	m_surfels = new Surfel[numSurfels];
	memcpy(m_surfels, surfels, sizeof(Surfel) * numSurfels);

	if (useGpu) {
		cudaMalloc(&m_zBufferPropertyGpu, sizeof(ZBufferProperty));
		cudaMemcpy(m_zBufferPropertyGpu, m_zBufferProperty, sizeof(ZBufferProperty), cudaMemcpyHostToDevice);

		cudaMalloc(&m_zBufferGpu, sizeof(ZBufferItem) * m_zBufferProperty->bufsize);
		cudaMemcpy(m_zBufferGpu, m_zBuffer, sizeof(ZBufferItem) * m_zBufferProperty->bufsize, cudaMemcpyHostToDevice);

		cudaMalloc(&m_filterLUTGpu, sizeof(float) * m_zBufferProperty->LUTsize);
		cudaMemcpy(m_filterLUTGpu, m_filterLUT, sizeof(float) * m_zBufferProperty->LUTsize, cudaMemcpyHostToDevice);

		cudaMalloc(&m_surfelsGpu, sizeof(Surfel) * numSurfels);
		cudaMemcpy(m_surfelsGpu, m_surfels, sizeof(Surfel) * numSurfels, cudaMemcpyHostToDevice);

		cudaMalloc(&m_imageGpu, sizeof(unsigned char) * width * height * 3);
	}
}

CRenderer::~CRenderer() {
    delete[] m_image;
	delete[] m_surfels;
    delete m_warper;
	delete m_zBufferProperty;
	delete[] m_zBuffer;
	delete[] m_clearData;
	delete[] m_filterLUT;
	if (m_useGpu) {
		cudaFree(m_zBufferPropertyGpu);
		cudaFree(m_zBufferGpu);
		cudaFree(m_filterLUTGpu);
		cudaFree(m_surfelsGpu);
		cudaFree(m_imageGpu);
	}
}

void CRenderer::init() {
	if (m_useGpu)
		cudaMemcpy(m_zBufferGpu, m_clearData, sizeof(ZBufferItem) * m_zBufferProperty->bufsize, cudaMemcpyHostToDevice);
	else
		memcpy(m_zBuffer, m_clearData, sizeof(ZBufferItem) * m_zBufferProperty->bufsize);
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
    TransformationMatrix16f userRotation, finalRotation;

    MtrCreateRotation4x4fc(dAngle, x, y, z, userRotation);
    MtrMult4x4f(userRotation, m_cameraPosition.rotationMatrix, finalRotation);
    MtrCopy4x4f(finalRotation, m_cameraPosition.rotationMatrix);
}

void CRenderer::render() {

    init();

    TransformationMatrix16f transformation, convertedTransformation;
    MtrMult4x4f(m_cameraPosition.scaleTranslationMatrix, m_cameraPosition.rotationMatrix, transformation);
    MtrTranspose4x4f(transformation, convertedTransformation);
    convertedTransformation[8] *= -1.0f;
    convertedTransformation[9] *= -1.0f;
    convertedTransformation[10] *= -1.0f;
    convertedTransformation[11] *= -1.0f;

    setTrafo(convertedTransformation);


	if (m_useGpu) {
		projectGpu(m_width, m_height, m_warper, m_zBufferPropertyGpu, m_zBufferGpu, m_filterLUTGpu, m_numSurfels, m_surfelsGpu);
		shadeZBufferGpu(m_width, m_height, m_warper, m_zBufferGpu, m_imageGpu, m_backgroundR, m_backgroundG, m_backgroundB);
		cudaMemcpy(m_image, m_imageGpu, sizeof(unsigned char) * m_width * m_height * 3, cudaMemcpyDeviceToHost);
	}
	else {
		project(m_width, m_height, m_warper, m_zBufferProperty, m_zBuffer, m_filterLUT, m_numSurfels, m_surfels);
		shadeZBuffer(m_width, m_height, m_warper, m_zBuffer, m_image, m_backgroundR, m_backgroundG, m_backgroundB);
	}
}

void CRenderer::setFrustum(float fofv, float aspect, float nearplane, float farplane)
{
	float t, b, l, r;
	float s;
	float* fv;

	// note: xP and yP are the half x,y-extent of the view frustum at z=1!
	m_warper->frustum.yP = (float)tan(fofv / 360.0 * M_PI);
	m_warper->frustum.xP = m_warper->frustum.yP * aspect;
	m_warper->frustum.nearplane = nearplane;
	m_warper->frustum.farplane = farplane;
	m_warper->frustum.xC = 0.f;
	m_warper->frustum.yC = 0.f;

	// top, bottom, left right coordinates for view frustum
	t = m_warper->frustum.yP * nearplane;
	b = -t;
	r = m_warper->frustum.xP * nearplane;
	l = -r;

	// camera coordinates of view frustum
	fv = m_warper->frustum.v;
	fv[0] = l; fv[1] = b; fv[2] = nearplane;
	fv[3] = r; fv[4] = b; fv[5] = nearplane;
	fv[6] = l; fv[7] = t; fv[8] = nearplane;
	fv[9] = r; fv[10] = t; fv[11] = nearplane;
	s = farplane / nearplane;
	fv[12] = l * s; fv[13] = b * s; fv[14] = farplane;
	fv[15] = r * s; fv[16] = b * s; fv[17] = farplane;
	fv[18] = l * s; fv[19] = t * s; fv[20] = farplane;
	fv[21] = r * s; fv[22] = t * s; fv[23] = farplane;
}

//-------------------------------------------------------------------------
// Init transformation variables
// note: the matrix trafo[] is expected to be row major ordered, i.e.
// row 1 has indices 0..3, row 2 has indices 4..7, etc.
//-------------------------------------------------------------------------
void CRenderer::setTrafo(const float trafo[16])
{
	float s;
	float r[9], t[3];
	float ir[9];
	float tm[9], tm2[9];
	int i, j, k;

	// get rotation part
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			r[i * 3 + j] = trafo[i * 4 + j];
		}
	}
	// get translation part
	k = 3;
	for (i = 0; i < 3; i++)
	{
		t[i] = trafo[k];
		k += 4;
	}

	// store the uniform scaling value
	// note: surfel rendering only works correctly with uniform scaling, thus uniform scaling
	// is assumed here.
	s = (float)sqrt(trafo[0] * trafo[0] + trafo[4] * trafo[4] + trafo[8] * trafo[8]);
	m_warper->transformation.scaling = s;

	// calculate inverse rotation and translation (needed for view frustum culling)
	MtrInverse3x3f(r, ir);

	// calculate the matrix for transforming the normals, which is the transpose 
	// inverse of the model-view transformation matrix.
	// note: eliminate a possible uniform scaling from the rotation matrix, this 
	// guarantees that transformed unit normals are still of unit length!
	MtrMultScal3x3f(r, 1 / s, tm);
	MtrInverse3x3f(tm, tm2);
	MtrTranspose3x3f(tm2, m_warper->transformation.normalsRotation);

	// set the member variables of 'wrp'
	MtrCopy3x3f(r, m_warper->transformation.rotation);
	for (i = 0; i < 3; i++) m_warper->transformation.translation[i] = t[i];
}