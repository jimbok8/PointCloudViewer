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
		m_clearData[i].transformedNormal = Eigen::Vector4f::Zero();
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

	m_image = new unsigned char[width * height * 3];

	m_factor = 1.0f;
	m_translate = m_rotate = Eigen::Matrix4f::Identity();
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
	memcpy(m_zBuffer, m_clearData, sizeof(ZBufferItem) * m_zBufferProperty->bufsize);
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
}

void CRenderer::project() {
	unsigned int ssbo0;
	glGenBuffers(1, &ssbo0);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo0);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(ZBufferProperty), m_zBufferProperty, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo0);

	unsigned int ssbo1;
	glGenBuffers(1, &ssbo1);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo1);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(Warper), m_warper, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, ssbo1);

	unsigned int ssbo2;
	glGenBuffers(1, &ssbo2);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo2);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(Surfel) * m_numSurfels, m_surfels, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, ssbo2);
	
	CComputeShader computeShader("shader/Compute.glsl");
	computeShader.use();
	computeShader.setInt("width", m_width);
	computeShader.setInt("height", m_height);
	glDispatchCompute((m_numSurfels + 1023) / 1024, 1, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	void* p = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_WRITE);
	memcpy(m_surfels, p, sizeof(Surfel) * m_numSurfels);

	/*for (int i = 0; i < 10; i++) {
		std::cout << (int)((m_surfels[i].xMin + 1.0f) / 2.0f * m_width) << ' ' << (int)((m_surfels[i].xMax + 1.0f) / 2.0f * m_width) << ' ' << (int)((m_surfels[i].yMin + 1.0f) / 2.0f * m_height) << ' ' << (int)((m_surfels[i].yMax + 1.0f) / 2.0f * m_height) << ' ';
		std::cout << m_surfels[i].radius << ' ' << m_surfels[i].zMin << ' ' << m_surfels[i].zMax << std::endl;
	}
	for (int i = m_numSurfels - 10; i < m_numSurfels; i++) {
		std::cout << (int)((m_surfels[i].xMin + 1.0f) / 2.0f * m_width) << ' ' << (int)((m_surfels[i].xMax + 1.0f) / 2.0f * m_width) << ' ' << (int)((m_surfels[i].yMin + 1.0f) / 2.0f * m_height) << ' ' << (int)((m_surfels[i].yMax + 1.0f) / 2.0f * m_height) << ' ';
		std::cout << m_surfels[i].radius << ' ' << m_surfels[i].zMin << ' ' << m_surfels[i].zMax << std::endl;
	}*/
}

void CRenderer::splat() {
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;

	for (int i = 0; i < m_numSurfels; i++) {
		Vertex v0, v1, v2, v3;
		v0.position = Eigen::Vector4f(m_surfels[i].xMin, m_surfels[i].yMin, 0.0f, 1.0f);
		v0.color = m_surfels[i].color;
		v0.transformedNormal = m_surfels[i].transformedNormal;
		v0.x0 = m_surfels[i].x0;
		v0.y0 = m_surfels[i].y0;
		v0.zMin = m_surfels[i].zMin;
		v0.zMax = m_surfels[i].zMax;
		v0.a = m_surfels[i].a;
		v0.b = m_surfels[i].b;
		v0.c = m_surfels[i].c;
		v0.det_ = m_surfels[i].det_;

		v1.position = Eigen::Vector4f(m_surfels[i].xMin, m_surfels[i].yMax, 0.0f, 1.0f);
		v1.color = m_surfels[i].color;
		v1.transformedNormal = m_surfels[i].transformedNormal;
		v1.x0 = m_surfels[i].x0;
		v1.y0 = m_surfels[i].y0;
		v1.zMin = m_surfels[i].zMin;
		v1.zMax = m_surfels[i].zMax;
		v1.a = m_surfels[i].a;
		v1.b = m_surfels[i].b;
		v1.c = m_surfels[i].c;
		v1.det_ = m_surfels[i].det_;

		v2.position = Eigen::Vector4f(m_surfels[i].xMax, m_surfels[i].yMin, 0.0f, 1.0f);
		v2.color = m_surfels[i].color;
		v2.transformedNormal = m_surfels[i].transformedNormal;
		v2.x0 = m_surfels[i].x0;
		v2.y0 = m_surfels[i].y0;
		v2.zMin = m_surfels[i].zMin;
		v2.zMax = m_surfels[i].zMax;
		v2.a = m_surfels[i].a;
		v2.b = m_surfels[i].b;
		v2.c = m_surfels[i].c;
		v2.det_ = m_surfels[i].det_;

		v3.position = Eigen::Vector4f(m_surfels[i].xMax, m_surfels[i].yMax, 0.0f, 1.0f);
		v3.color = m_surfels[i].color;
		v3.transformedNormal = m_surfels[i].transformedNormal;
		v3.x0 = m_surfels[i].x0;
		v3.y0 = m_surfels[i].y0;
		v3.zMin = m_surfels[i].zMin;
		v3.zMax = m_surfels[i].zMax;
		v3.a = m_surfels[i].a;
		v3.b = m_surfels[i].b;
		v3.c = m_surfels[i].c;
		v3.det_ = m_surfels[i].det_;

		int i0, i1, i2, i3;
		i0 = vertices.size();
		i1 = vertices.size() + 1;
		i2 = vertices.size() + 2;
		i3 = vertices.size() + 3;

		vertices.push_back(v0);
		vertices.push_back(v1);
		vertices.push_back(v2);
		vertices.push_back(v3);

		indices.push_back(i0);
		indices.push_back(i1);
		indices.push_back(i2);
		indices.push_back(i3);
		indices.push_back(i2);
		indices.push_back(i1);
	}

	unsigned vao, vbo, ebo;
	glGenVertexArrays(1, &vao);
	glGenBuffers(1, &vbo);
	glGenBuffers(1, &ebo);

	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, position));
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, color));
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, transformedNormal));
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

	unsigned int ssbo0;
	glGenBuffers(1, &ssbo0);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo0);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(ZBufferProperty), m_zBufferProperty, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo0);

	unsigned int ssbo3;
	glGenBuffers(1, &ssbo3);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo3);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(ZBufferItem) * m_zBufferProperty->bufsize, m_zBuffer, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, ssbo3);

	unsigned int ssbo4;
	glGenBuffers(1, &ssbo4);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo4);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(float) * m_zBufferProperty->LUTsize, m_filterLUT, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, ssbo4);

	CRenderShader renderShader1("shader/Vertex1.glsl", "shader/Fragment1.glsl");
	renderShader1.use();
	renderShader1.setInt("width", m_width);
	renderShader1.setInt("height", m_height);
	glBindVertexArray(vao);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, ssbo3);
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	CRenderShader renderShader2("shader/Vertex1.glsl", "shader/Fragment2.glsl");
	renderShader2.use();
	renderShader2.setInt("width", m_width);
	renderShader2.setInt("height", m_height);
	glBindVertexArray(vao);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, ssbo3);
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	CRenderShader renderShader3("shader/Vertex1.glsl", "shader/Fragment3.glsl");
	renderShader3.use();
	renderShader3.setInt("width", m_width);
	renderShader3.setInt("height", m_height);
	glBindVertexArray(vao);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, ssbo4);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, ssbo3);
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	void* p = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_WRITE);
	memcpy(m_zBuffer, p, sizeof(ZBufferItem)* m_zBufferProperty->bufsize);

	int sum = 0;
	for (int i = 0; i < m_zBufferProperty->bufsize; i++)
		if (m_zBuffer[i].zMin < FLT_MAX) {
			std::cout << m_zBuffer[i].color.transpose() << ' ' << m_zBuffer[i].transformedNormal.transpose() << ' ' << m_zBuffer[i].zMin << ' ' << m_zBuffer[i].zMax << ' ' << m_zBuffer[i].w << std::endl;
			if ((++sum) == 10)
				break;
		}
	std::cout << sum << std::endl;
}

void CRenderer::shade() {
	unsigned int ssbo1;
	glGenBuffers(1, &ssbo1);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo1);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(Warper), m_warper, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, ssbo1);

	unsigned int ssbo3;
	glGenBuffers(1, &ssbo3);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo3);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(ZBufferItem) * m_zBufferProperty->bufsize, m_zBuffer, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, ssbo3);

	float* image = new float[m_zBufferProperty->bufsize * 3];
	unsigned int ssbo5;
	glGenBuffers(1, &ssbo5);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo5);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(float) * m_zBufferProperty->bufsize * 3, image, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, ssbo5);

	CComputeShader computeShader("shader/ComputeShade.glsl");
	computeShader.use();
	computeShader.setInt("width", m_width);
	computeShader.setInt("height", m_height);
	glDispatchCompute((m_width * m_height + 1023) / 1024, 1, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	void* p = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_WRITE);
	memcpy(image, p, sizeof(float) * m_zBufferProperty->bufsize * 3);

	int sum = 0;
	for (int i = 0; i < m_zBufferProperty->bufsize * 3; i++) {
		m_image[i] = (unsigned char)image[i];
		//if (m_image[i] > 30 || m_image[i] < 20) {
		//	std::cout << (int)m_image[i] << ' ';
		//	sum++;
		//}
	}
	//std::cout << sum << std::endl;
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
		m_clearData[i].transformedNormal = Eigen::Vector4f::Zero();
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

	m_factor *= dScaleX;
}

void CRenderer::translate(const float dx, const float dy, const float dz) {
	m_cameraPosition.scaleTranslationMatrix[12] += dx;
	m_cameraPosition.scaleTranslationMatrix[13] += dy;
	m_cameraPosition.scaleTranslationMatrix[14] += dz;

	m_translate = TransformHelper::translate(Eigen::Vector3f(dx, dy, dz)) * m_translate;
}

void CRenderer::rotate(const float dAngle, const float x, const float y, const float z) {
	TransformationMatrix16f userRotation, finalRotation;

	MtrCreateRotation4x4fc(dAngle, x, y, z, userRotation);
	MtrMult4x4f(userRotation, m_cameraPosition.rotationMatrix, finalRotation);
	MtrCopy4x4f(finalRotation, m_cameraPosition.rotationMatrix);

	m_rotate = TransformHelper::rotate(Eigen::Vector3f(x, y, z), dAngle) * m_rotate;
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

	project();
	splat();
	shade();
}