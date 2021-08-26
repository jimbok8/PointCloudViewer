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
	}
}

void CRenderer::init() {
    int index = 0;
    for (int j = 0; j < m_height; j++)
        for (int i = 0; i < m_width; i++)
			setColor(i, j, m_backgroundR, m_backgroundG, m_backgroundB);

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
	double t0 = glfwGetTime();

    init();

    TransformationMatrix16f transformation, convertedTransformation;
    MtrMult4x4f(m_cameraPosition.scaleTranslationMatrix, m_cameraPosition.rotationMatrix, transformation);
    MtrTranspose4x4f(transformation, convertedTransformation);
    convertedTransformation[8] *= -1.0f;
    convertedTransformation[9] *= -1.0f;
    convertedTransformation[10] *= -1.0f;
    convertedTransformation[11] *= -1.0f;

    setTrafo(convertedTransformation);

	double t1 = glfwGetTime();

    int bbox[4];
	bbox[2] = bbox[3] = 0;
	bbox[0] = m_width - 1;
	bbox[1] = m_height - 1;
	if (m_useGpu) {
		projectGPU(m_width, m_height, m_warper, m_zBufferPropertyGpu, m_zBufferGpu, m_filterLUTGpu, m_numSurfels, m_surfelsGpu, bbox);
		cudaMemcpy(m_zBuffer, m_zBufferGpu, sizeof(ZBufferItem) * m_zBufferProperty->bufsize, cudaMemcpyDeviceToHost);
	}
	else
		project(m_width, m_height, m_warper, m_zBufferProperty, m_zBuffer, m_filterLUT, m_numSurfels, m_surfels, bbox);

	double t2 = glfwGetTime();

    shadeZBuffer(1, bbox);

	double t3 = glfwGetTime();
	std::cout << t1 - t0 << ' ' << t2 - t1 << ' ' << t3 - t2 << std::endl;
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
	float ir[9], it[3];
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
	it[0] = -ir[0] * t[0] - ir[1] * t[1] - ir[2] * t[2];
	it[1] = -ir[3] * t[0] - ir[4] * t[1] - ir[5] * t[2];
	it[2] = -ir[6] * t[0] - ir[7] * t[1] - ir[8] * t[2];

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

unsigned char* CRenderer::getPixelPtr(const int x, const int y) {
	return m_image + (y * m_width + x) * 3;
}

void CRenderer::setColor(const int x, const int y, const unsigned char r, const unsigned char g, const unsigned char b) {
	unsigned char* p = getPixelPtr(x, y);
	*p = r;
	*(p + 1) = g;
	*(p + 2) = b;
}

//-------------------------------------------------------------------------
// Calculate phong shading for a sample
// note: this function uses the formulation with the reflection vector. for
// directional light sources, it produces exactly the same results as cube
// maps.
//-------------------------------------------------------------------------
void CRenderer::lightSamplePhongR(const float r, const float g, const float b, const float nx, const float ny, const float nz, const float vx, const float vy, const float vz, float& resultR, float& resultG, float& resultB)
{
	float Ir, Ig, Ib;
	float Ar, Ag, Ab;
	float Lx, Ly, Lz;
	float Rx, Ry, Rz;
	float t, power, ndotl, rdotv;
	int j;

	float kA = 0.5f;
	float kD = 0.75f;
	float kS = 0.25f;
	int shininess = 0;
	float specularR = 205;
	float specularG = 205;
	float specularB = 205;

	Ir = Ig = Ib = 1.0f;
	Ar = Ag = Ab = 0.5f;

	// ambient contribution
	t = kA;
	resultR = t * Ar * r;
	resultG = t * Ag * g;
	resultB = t * Ab * b;

	Lx = Ly = 0.0f;
	Lz = -1.0f;

	// calculate the N*L dot product
	ndotl = nx * Lx + ny * Ly + nz * Lz;
	ndotl = (ndotl < 0 ? -ndotl : ndotl);

	// calculate normalized reflection vector
	Rx = 2 * nx * ndotl - Lx;
	Ry = 2 * ny * ndotl - Ly;
	Rz = 2 * nz * ndotl - Lz;

	// calculate R*V dot product
	rdotv = vx * Rx + vy * Ry + vz * Rz;
	rdotv = (rdotv < 0 ? -rdotv : rdotv);

	// calculate the phong shininess power
	power = rdotv;
	j = shininess;
	while (j > 0)
	{
		power *= rdotv;
		j--;
	}

	// increment intensities
	t = kD * ndotl;
	power *= kS;
	resultR += Ir * (t * r + specularR * power);
	resultG += Ig * (t * g + specularG * power);
	resultB += Ib * (t * b + specularB * power);
}

//-------------------------------------------------------------------------
// Shade the samples in the z-buffer of a warper
//-------------------------------------------------------------------------
void CRenderer::shadeZBuffer(int magfactor, int bbox[4])
{
	int zbf_rows, zbf_cols;
	float zbf_x_cur, zbf_y_cur;
	int zbf_i_cur;
	ZBufferItem* zbf_buf = m_zBuffer;

	Frustum* frst;

	//int xsize, ysize;				// x and y screen coordinates size			

	ZBufferItem* zbufitem;				// current zbuffer item

	float vp_sx, vp_sy;				// variables for inverse viewport mapping
	float vp_tx, vp_ty;

	float w_, vec_len_;
	int c;

	int start_dim_x, start_dim_y;	// writing the color values to the display image
	int end_dim_x, end_dim_y;		// (supports progressive rendering)
	int dim_y, dim_x;

	// initialise local variables for more efficient access
	frst = &(m_warper->frustum);

	// set variables for inverse viewport mapping 
	vp_sx = 2 * frst->xP / m_width;
	vp_sy = 2 * frst->yP / m_height;
	vp_tx = frst->xC - frst->xP;
	vp_ty = frst->yC - frst->yP;

	// shade z-buffer
	// bbox[] specifies a bounding box
	zbf_x_cur = bbox[0] + 0.5f;
	zbf_y_cur = bbox[1] + 0.5f;
	zbf_rows = bbox[1];
	zbf_cols = bbox[0];
	zbf_i_cur = bbox[1] * m_width + bbox[0];
	while (zbf_rows <= bbox[3])
	{
		while (zbf_cols <= bbox[2])

		{
			zbufitem = &(zbf_buf[zbf_i_cur]);
			// avoid division by zero!
			if (zbufitem->w != 0.0f)
			{
				// NOTE: we do per surfel shading, hence all there is left to do for the shader is
				// to normalize the depth, colors and normals, and write them to the display image

				w_ = 1.f / zbufitem->w;

				// normalize colors
				float r = zbufitem->c[0] * w_;
				float g = zbufitem->c[1] * w_;
				float b = zbufitem->c[2] * w_;

				// re-normalize normal
				float nx = zbufitem->n[0];
				float ny = zbufitem->n[1];
				float nz = zbufitem->n[2];
				w_ = 1.f / (float)sqrt(nx * nx + ny * ny + nz * nz);
				nx *= w_;
				ny *= w_;
				nz *= w_;

				// compute viewing vector
				float vx = -(zbf_x_cur * vp_sx + vp_tx);
				float vy = -(zbf_y_cur * vp_sy + vp_ty);
				float vz = -1.f;
				vec_len_ = 1.f / (float)sqrt(vx * vx + vy * vy + 1.f);
				vx *= vec_len_;
				vy *= vec_len_;
				vz *= vec_len_;

				float resultR, resultG, resultB;
				lightSamplePhongR(r, g, b, nx, ny, nz, vx, vy, vz, resultR, resultG, resultB);

				// clamp color intensities
				if (resultR > 255.0) resultR = 255.0;
				if (resultG > 255.0) resultG = 255.0;
				if (resultB > 255.0) resultB = 255.0;

				// write to display image, blow up pixels for progressive rendering
				start_dim_x = zbf_cols * magfactor;
				end_dim_x = start_dim_x + magfactor;
				start_dim_y = zbf_rows * magfactor;
				end_dim_y = start_dim_y + magfactor;

				// write output color to frame buffer
				for (dim_y = start_dim_y; dim_y < end_dim_y; dim_y++)
					for (dim_x = start_dim_x; dim_x < end_dim_x; dim_x++)
						setColor(dim_x, dim_y, (unsigned char)resultR, (unsigned char)resultG, (unsigned char)resultB);
			}
			else {
				// write to display image, blow up pixels for progressive rendering
				start_dim_x = zbf_cols * magfactor;
				end_dim_x = start_dim_x + magfactor;
				start_dim_y = zbf_rows * magfactor;
				end_dim_y = start_dim_y + magfactor;
				for (dim_y = start_dim_y; dim_y < end_dim_y; dim_y++)
					for (dim_x = start_dim_x; dim_x < end_dim_x; dim_x++)
						setColor(dim_x, dim_y, m_backgroundR, m_backgroundG, m_backgroundB);
			}

			zbf_x_cur += 1.0f;
			zbf_i_cur++;
			zbf_cols++;
		}
		bbox[1]++;
		zbf_x_cur = bbox[0] + 0.5f;
		zbf_y_cur = bbox[1] + 0.5f;
		zbf_rows = bbox[1];
		zbf_cols = bbox[0];
		zbf_i_cur = bbox[1] * m_width + bbox[0];
	}
}