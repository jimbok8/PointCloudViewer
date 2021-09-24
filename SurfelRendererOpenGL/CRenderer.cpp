#include "CRenderer.h"

CRenderer::CRenderer(const int numSurfels, const Surfel* surfels, const int width, const int height, const unsigned char backgroundR, const unsigned char backgroundG, const unsigned char backgroundB) :
	m_width(width),
	m_height(height),
	m_numSurfels(numSurfels),
	m_backgroundR(backgroundR),
	m_backgroundG(backgroundG),
	m_backgroundB(backgroundB)
	{
	MtrUnity4x4f(m_cameraPosition.scaleTranslationMatrix);
	m_cameraPosition.scaleTranslationMatrix[14] = -1000.0f;
	MtrUnity4x4f(m_cameraPosition.rotationMatrix);

	m_warper = new Warper;
	setFrustum(30.f, (float)width / (float)height, 10.f, 100000.f);

	m_zBufferProperty = new ZBufferProperty;
	m_zBufferProperty->bufsize = width * height;
	m_zBufferProperty->LUTsize = 1024;
	m_zBufferProperty->cutoffRadius = 1.0f;
	m_zBufferProperty->angleTrheshold = 8.0f;

	m_zBuffer = new ZBufferItem[width * height];

	m_clearData = new ZBufferItem[width * height];
	for (int i = 0; i < width * height; i++) {
		m_clearData[i].color = Eigen::Vector4f::Zero();
		m_clearData[i].normal = Eigen::Vector4f::Zero();
		m_clearData[i].zMin = FLT_MAX;
		m_clearData[i].zMax = -FLT_MAX;
		m_clearData[i].w = 0.0f;
	}

	m_filterLUT = new float[m_zBufferProperty->LUTsize];
	float d = 1.f / ((float)m_zBufferProperty->LUTsize - 1.f);
	for (int i = 0; i < m_zBufferProperty->LUTsize; i++)
		m_filterLUT[i] = (float)exp(-(float)i * d * m_zBufferProperty->cutoffRadius * m_zBufferProperty->cutoffRadius);

	m_surfels = new Surfel[numSurfels];
	memcpy(m_surfels, surfels, sizeof(Surfel) * numSurfels);

	m_vertices = new Vertex[numSurfels * 4];
	m_indices = new unsigned int[numSurfels * 6];
	for (int i = 0; i < m_numSurfels; i++) {
		int vertexIndex = i * 4;
		m_vertices[vertexIndex].color = m_surfels[i].color;
		m_vertices[vertexIndex + 1].color = m_surfels[i].color;
		m_vertices[vertexIndex + 2].color = m_surfels[i].color;
		m_vertices[vertexIndex + 3].color = m_surfels[i].color;

		int indexIndex = i * 6;
		m_indices[indexIndex] = vertexIndex;
		m_indices[indexIndex + 1] = vertexIndex + 1;
		m_indices[indexIndex + 2] = vertexIndex + 2;
		m_indices[indexIndex + 3] = vertexIndex + 3;
		m_indices[indexIndex + 4] = vertexIndex + 2;
		m_indices[indexIndex + 5] = vertexIndex + 1;
	}

	m_image = new unsigned char[width * height * 3];
	m_floatImage = new float[width * height * 3];

	unsigned vbo, ebo;
	glGenVertexArrays(1, &m_vao);
	glGenBuffers(1, &vbo);
	glGenBuffers(1, &ebo);

	glBindVertexArray(m_vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * m_numSurfels * 4, m_vertices, GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * m_numSurfels * 6, m_indices, GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, position));
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, color));
	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, x0));
	glEnableVertexAttribArray(4);
	glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, y0));
	glEnableVertexAttribArray(5);
	glVertexAttribPointer(5, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, zMin));
	glEnableVertexAttribArray(6);
	glVertexAttribPointer(6, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, zMax));
	glEnableVertexAttribArray(7);
	glVertexAttribPointer(7, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, a));
	glEnableVertexAttribArray(8);
	glVertexAttribPointer(8, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, b));
	glEnableVertexAttribArray(9);
	glVertexAttribPointer(9, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, c));
	glEnableVertexAttribArray(10);
	glVertexAttribPointer(10, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, det_));
	glBindVertexArray(0);

	glGenBuffers(1, &m_ssbo0);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_ssbo0);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(ZBufferProperty), m_zBufferProperty, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_ssbo0);

	glGenBuffers(1, &m_ssbo1);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_ssbo1);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(Warper), m_warper, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_ssbo1);

	glGenBuffers(1, &m_ssbo2);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_ssbo2);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(Surfel) * m_numSurfels, m_surfels, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_ssbo2);

	glGenBuffers(1, &m_ssbo3);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_ssbo3);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(ZBufferItem) * m_zBufferProperty->bufsize, m_zBuffer, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, m_ssbo3);

	glGenBuffers(1, &m_ssbo4);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_ssbo4);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(float) * m_zBufferProperty->LUTsize, m_filterLUT, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, m_ssbo4);

	glGenBuffers(1, &m_ssbo5);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_ssbo5);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(float) * m_zBufferProperty->bufsize * 3, m_floatImage, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, m_ssbo5);

	glGenBuffers(1, &m_ssbo6);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_ssbo6);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(Vertex) * numSurfels * 4, m_vertices, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, m_ssbo6);
}

CRenderer::~CRenderer() {
	delete[] m_image;
	delete[] m_surfels;
	delete m_warper;
	delete m_zBufferProperty;
	delete[] m_zBuffer;
	delete[] m_clearData;
	delete[] m_filterLUT;
}

void CRenderer::init() {
	//memcpy(m_zBuffer, m_clearData, sizeof(ZBufferItem) * m_zBufferProperty->bufsize);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_ssbo3);
	void* p = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_WRITE);
	memcpy(p, m_clearData, sizeof(ZBufferItem) * m_zBufferProperty->bufsize);
	glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
}

void CRenderer::setFrustum(float fofv, float aspect, float nearplane, float farplane) {
	float t, b, l, r;
	float s;
	float* fv;

	// note: xP and yP are the half x,y-extent of the view frustum at z=1!
	m_warper->yP = (float)tan(fofv / 360.0 * M_PI);
	m_warper->xP = m_warper->yP * aspect;
	m_warper->nearplane = nearplane;
	m_warper->farplane = farplane;
	m_warper->xC = 0.f;
	m_warper->yC = 0.f;

	// top, bottom, left right coordinates for view frustum
	t = m_warper->yP * nearplane;
	b = -t;
	r = m_warper->xP * nearplane;
	l = -r;

	// camera coordinates of view frustum
	fv = m_warper->v;
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

void CRenderer::setTrafo(const float trafo[16]) {
	float s;
	float r[9], t[3];
	float ir[9];
	float tm[9], tm2[9];
	int i, j, k;

	// get rotation part
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			r[i * 3 + j] = trafo[i * 4 + j];
	// get translation part
	k = 3;
	for (i = 0; i < 3; i++) {
		t[i] = trafo[k];
		k += 4;
	}

	// store the uniform scaling value
	// note: surfel rendering only works correctly with uniform scaling, thus uniform scaling
	// is assumed here.
	s = (float)sqrt(trafo[0] * trafo[0] + trafo[4] * trafo[4] + trafo[8] * trafo[8]);
	m_warper->scaling = s;

	// calculate inverse rotation and translation (needed for view frustum culling)
	MtrInverse3x3f(r, ir);

	// calculate the matrix for transforming the normals, which is the transpose 
	// inverse of the model-view transformation matrix.
	// note: eliminate a possible uniform scaling from the rotation matrix, this 
	// guarantees that transformed unit normals are still of unit length!
	MtrMultScal3x3f(r, 1 / s, tm);
	MtrInverse3x3f(tm, tm2);
	MtrTranspose3x3f(tm2, m_warper->normalsRotation);

	// set the member variables of 'wrp'
	MtrCopy3x3f(r, m_warper->rotation);
	for (i = 0; i < 3; i++)
		m_warper->translation[i] = t[i];

	glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_ssbo1);
	void* p = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_WRITE);
	memcpy(p, m_warper, sizeof(Warper));
	glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
}

void CRenderer::project() {
	CComputeShader computeShader("shader/Compute.glsl");
	computeShader.use();
	computeShader.setInt("width", m_width);
	computeShader.setInt("height", m_height);
	glDispatchCompute((m_numSurfels + 1023) / 1024, 1, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void CRenderer::splat() {
	double t0 = glfwGetTime();
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_ssbo6);
	void* p = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_WRITE);
	glBindVertexArray(m_vao);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * m_numSurfels * 4, p, GL_STATIC_DRAW);
	glBindVertexArray(0);
	glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
	double t1 = glfwGetTime();

	CRenderShader renderShader1("shader/Vertex1.glsl", "shader/Fragment1.glsl");
	renderShader1.use();
	renderShader1.setInt("width", m_width);
	renderShader1.setInt("height", m_height);
	glBindVertexArray(m_vao);
	glDrawElements(GL_TRIANGLES, m_numSurfels * 6, GL_UNSIGNED_INT, 0);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	glBindVertexArray(0);

	/*CRenderShader renderShader2("shader/Vertex1.glsl", "shader/Fragment2.glsl");
	renderShader2.use();
	renderShader2.setInt("width", m_width);
	renderShader2.setInt("height", m_height);
	glBindVertexArray(m_vao);
	glDrawElements(GL_TRIANGLES, m_numSurfels * 6, GL_UNSIGNED_INT, 0);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	glBindVertexArray(0);*/

	CRenderShader renderShader3("shader/Vertex1.glsl", "shader/Fragment3.glsl");
	renderShader3.use();
	renderShader3.setInt("width", m_width);
	renderShader3.setInt("height", m_height);
	glBindVertexArray(m_vao);
	glDrawElements(GL_TRIANGLES, m_numSurfels * 6, GL_UNSIGNED_INT, 0);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	glBindVertexArray(0);

	double t2 = glfwGetTime();
	std::cout << t1 - t0 << ' ' << t2 - t1 << std::endl;
}

void CRenderer::shade() {
	CComputeShader computeShader("shader/ComputeShade.glsl");
	computeShader.use();
	computeShader.setInt("width", m_width);
	computeShader.setInt("height", m_height);
	glDispatchCompute((m_width * m_height + 1023) / 1024, 1, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_ssbo5);
	void* p = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_WRITE);
	memcpy(m_floatImage, p, sizeof(float) * m_zBufferProperty->bufsize * 3);
	glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

	int sum = 0;
	for (int i = 0; i < m_zBufferProperty->bufsize * 3; i++)
		m_image[i] = (unsigned char)m_floatImage[i];
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

void CRenderer::resize(const int width, const int height) {
	m_width = width;
	m_height = height;

	delete[] m_zBuffer;
	delete[] m_clearData;
	delete[] m_image;

	m_zBufferProperty->bufsize = width * height;

	m_zBuffer = new ZBufferItem[width * height];

	m_clearData = new ZBufferItem[width * height];
	for (int i = 0; i < width * height; i++) {
		m_clearData[i].color = Eigen::Vector4f::Zero();
		m_clearData[i].normal = Eigen::Vector4f::Zero();
		m_clearData[i].zMin = FLT_MAX;
		m_clearData[i].zMax = -FLT_MAX;
		m_clearData[i].w = 0.0f;
	}

	m_image = new unsigned char[width * height * 3];
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
	project();
	double t2 = glfwGetTime();
	splat();
	double t3 = glfwGetTime();
	shade();
	double t4 = glfwGetTime();
	//std::cout << t1 - t0 << ' ' << t2 - t1 << ' ' << t3 - t2 << ' ' << t4 - t3 << std::endl;
}