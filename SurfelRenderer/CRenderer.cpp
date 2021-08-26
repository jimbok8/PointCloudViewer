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
    // set the default view frustum (similar to gluPerspective): angle of field of view, 
    // aspect ratio, near and far clipping planes
    setFrustum(30.f, 1.f, 10.f, 100000.f);
}

CRenderer::~CRenderer() {
    delete[] m_image;
    delete warper;
    delete zBuffer;
}

void CRenderer::init() {
    unsigned char r = GetRValue(m_backgroundColor);
    unsigned char g = GetGValue(m_backgroundColor);
    unsigned char b = GetBValue(m_backgroundColor);
    int index = 0;
    for (int j = 0; j < m_height; j++)
        for (int i = 0; i < m_width; i++) {
            unsigned char* p = getPixelPtr(i, j);
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
	double t0 = glfwGetTime();

    init();

    MyDataTypes::TransformationMatrix16f transformation, convertedTransformation;
    MtrUtil::MtrMult4x4f(m_cameraPosition.scaleTranslationMatrix, m_cameraPosition.rotationMatrix, transformation);
    MtrUtil::MtrTranspose4x4f(transformation, convertedTransformation);
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
    for (CSurfel& surfel : m_surfels)
        wrpProjectSample(m_width, m_height, warper, zBuffer, &surfel, bbox);

	double t2 = glfwGetTime();

    shadeZBuffer(1, bbox);

	double t3 = glfwGetTime();
	std::cout << t1 - t0 << ' ' << t2 - t1 << ' ' << t3 - t2 << std::endl;
}

void CRenderer::unpackRGB(float& r, float& g, float& b, COLORREF color) {
	r = (float)((color & 0x00ff0000) >> 16);
	g = (float)((color & 0x0000ff00) >> 8);
	b = (float)(color & 0x000000ff);
}

//-------------------------------------------------------------------------
// Splat a surfel into the z-buffer
//-------------------------------------------------------------------------
int CRenderer::zbfSurfaceSplat(int width, int height, ZBuffer* zBuffer, float x0, float y0, float z, float n[3], CSurfel* surfel, int l, float scale_stoo, float scale_otoc, float vp_sx, float vp_sy, float vp_tx, float vp_ty, int bbox[4])
{
	float* zbf_filterLUT = zBuffer->filterLUT;
	float zbf_LUTsize = zBuffer->LUTsize;
	float zbf_cutoffRadius_2 = zBuffer->cutoffRadius * zBuffer->cutoffRadius;
	float _zbf_cutoffRadius_2 = 1 / (zbf_cutoffRadius_2);
	float zbf_constThreshold = zBuffer->constThreshold;
	float zbf_distThreshold = zBuffer->distThreshold;
	float zbf_angleThreshold = zBuffer->angleTrheshold;

	float V_x, V_y, V_z;		// viewing direction
	float S_x, S_y, S_z;		// S parameter direction on ST plane
	float T_x, T_y, T_z;		// T parameter direction on ST plane
	float Ix_x, Ix_y, Ix_z;		// direction of projection of screen x vector onto ST plane
	float Iy_x, Iy_y, Iy_z;		// direction of projection of screen y vector onto ST plane
	float r, r_, f;
	float ndotv;				// N*V (normal dot viewing direction) dotproduct
	float sx, sy, tx, ty;		// derivatives of the screen to ST plane mapping
	float m11, m12, m22;
	float a, b, b_2, c;			// the EWA ellipse coefficients

	int Xmin, Xmax, Ymin, Ymax;	// bounding box of the ellipse to be rasterized
	float lx, ly;
	int X, Y;
	float x, y;
	float q, dq, ddq;

	float dzc_dxs, dzc_dys;			// dzc/dxs, dzc/dys derivatives
	float z_start, z_cur;			// z values to be rasterized

	int i;
	COLORREF color, specularColor;
	float r_comp, g_comp, b_comp;

	float threshold_c0, threshold_c1, threshold_c3;
	float e;
	float w;

	float l_2, _l_2;
	float det_;

	Vector3D normal;
	float _radius;

	bool clip;

	//scale z buffer according to surfel radius
	float scale_otoc_radius;

	// init bounding box return value
	bbox[0] = width;
	bbox[1] = height;
	bbox[2] = bbox[3] = 0;

	l_2 = (float)(1 << l);
	_l_2 = 1 / l_2;

	// thresholds for the 'merge/separate' decision
	// note: 
	// - these thresholds are dependent on the level of the warped samples, since the
	// distance on the surface between samples increases with the warped level.
	// - these offsets should be constant in units 'dz per unit in object space'. but since 
	// z coordinates in the z-buffer are in camera coordinates, we have to transform the offsets
	// to camera space by applying the 'scale_otoc' (object to camera space) scaling
	threshold_c0 = zbf_constThreshold * l_2 * scale_otoc;		// the basic constant offset
	threshold_c1 = zbf_distThreshold * l_2 * scale_otoc;		// offset dependent on the square distance to the ellipse center q=r^2
	threshold_c3 = zbf_angleThreshold;							// cut off for 'ndotv' used for calculating the derivatives dzc_dxs,dzc_dys

	l_2 *= l_2;

	// step 1: calculate the EWA ellipse coefficients

	// compute normalized viewing vector V
	// restore camera coordinates of projected point (on z=1 plane)
	V_x = -(x0 / vp_sx - vp_tx);
	V_y = -(y0 / vp_sy - vp_ty);
	V_z = -1.f;
	r_ = 1.f / (float)sqrt(V_x * V_x + V_y * V_y + 1.f);
	V_x *= r_; V_y *= r_; V_z *= r_;
	ndotv = n[0] * V_x + n[1] * V_y + n[2] * V_z;

	if (ndotv < 0) {
		n[0] = -n[0];
		n[1] = -n[1];
		n[2] = -n[2];
		ndotv = -ndotv;
	}

	// project screen x,y unit vectors along V onto ST plane
	// determine direction Ix,Iy of projection of x,y
	Ix_x = -V_z * n[2] - V_y * n[1];		// (X x V) x N
	Ix_y = V_y * n[0];
	Ix_z = V_z * n[0];
	Iy_x = V_x * n[1];				// (Y x V) x N
	Iy_y = -V_x * n[0] - V_z * n[2];
	Iy_z = V_z * n[1];

	// f given by the vector equation Y+g*V=f*Iy resp X+g*V=f*Ix
	// Iy*f resp. Ix*f is the intersection point of Iy,Ix with Y+g*V resp. X+g*V
	// (see SurfaceSplatting.mws)
	f = -1.f / ndotv;

	// dzdx, dzdy derivatives for rasterizing z values on the ellipse
	if (f < -threshold_c3) e = -threshold_c3; else e = f;
	// note: since z coordinates in the z buffer are camera coordinate z-values, we have
	// to transform from screen to camera coordinates (by concatenating the screen-to-object
	// and object-to-camera scalings).
	dzc_dxs = Ix_z * e * scale_stoo * scale_otoc * z;
	dzc_dys = Iy_z * e * scale_stoo * scale_otoc * z;
	e = -e;

	// normalize Ix
	r = (float)sqrt(Ix_x * Ix_x + Ix_y * Ix_y + Ix_z * Ix_z);
	r_ = 1 / r;
	S_x = Ix_x * r_;
	S_y = Ix_y * r_;
	S_z = Ix_z * r_;

	// determine T parameter direction on ST plane
	// note: S parameter direction is chosen as Ix
	// T is automatically normalized,  since n and S have unit length
	// and are orthogonal by construction of S
	T_x = n[1] * S_z - n[2] * S_y;		// N x S
	T_y = n[2] * S_x - n[0] * S_z;
	T_z = n[0] * S_y - n[1] * S_x;

	// compute sx, sy, tx, ty derivatives
	// these derivatives build the inverse jacobian inv(J)=[sx,sy | tx,ty]
	// of the mapping J from object surface to screen

	_radius = surfel->getRadius();
	scale_otoc_radius = scale_otoc * _radius;
	_radius = 1.0f / _radius;

	Iy_x *= f; Iy_y *= f; Iy_z *= f;		// f*Iy is the intersection point with Y+g*V
	sx = r * f * scale_stoo * z * _radius * _l_2;		// note f given by the vector equation X+g*V=f*Ix takes the same value as above
	tx = 0;
	sy = (Iy_x * S_x + Iy_y * S_y + Iy_z * S_z) * scale_stoo * z * _radius * _l_2;		// Iy*S projects Iy onto S
	ty = (Iy_x * T_x + Iy_y * T_y + Iy_z * T_z) * scale_stoo * z * _radius * _l_2;		// Iy*T projects Iy onto T

	// compute z-range of the reconstruction kernel
	// see ellipseboundingbox_general.mws, idea: compute the point on the ellipse, where the ellipse tangent
	// is perpendicular to the depth gradient, i.e. [dzc_dxs, dzc_dys]. this is the point with maximum depth
	// on the ellipse
	// NOTE: to avoid overshoots in z, we use the z-range of the reconstruction kernel for blending!
	// NOTE: the variable "d" in the maple sheet corresponds to "-zbf_cutoffRadius_2"!
	// first, compute the conic matrix of the reconstruction kernel, which is [a b/2 | b/2 c] = inv(J)^T*inv(J)
	a = sx * sx;
	b_2 = sx * sy;
	b = 2.f * b_2;
	c = sy * sy + ty * ty;
	float discr;
	discr = -4 * a * dzc_dxs * b * dzc_dys * c - a * dzc_dys * dzc_dys * b * b + 4 * c * dzc_dys * dzc_dys * a * a +
		4 * a * dzc_dxs * dzc_dxs * c * c + b * b * b * dzc_dxs * dzc_dys - b * b * dzc_dxs * dzc_dxs * c;

	float zExtremum_x, zExtremum_y, tmp;
	tmp = (float)sqrt(discr * zbf_cutoffRadius_2);
	zExtremum_x = tmp * (-dzc_dys * b + 2 * dzc_dxs * c) / discr;
	zExtremum_y = tmp * (-2 * dzc_dys * a + dzc_dxs * b) / discr;

	float zRange_x, zRange_y;
	tmp = zExtremum_x * dzc_dxs;
	zRange_x = (tmp < 0) ? -tmp : tmp;
	tmp = zExtremum_y * dzc_dys;
	zRange_y = (tmp < 0) ? -tmp : tmp;

	float zMax, zMin;
	zMax = z + zRange_x + zRange_y;
	zMin = z - zRange_x - zRange_y;

	// guarantee a certain minimum z-range, otherwise blending fails for splats exactly parallel to the
	// image plane (the minimum z-range is 1 in object space, and then scaled to camera space).
	if (zMax - zMin < l_2 * scale_otoc_radius) {
		zMax += 0.5f * l_2 * scale_otoc_radius;
		zMin -= 0.5f * l_2 * scale_otoc_radius;
	}


	// calculate the matrix e[]=inv(J*transpose(J)+I), which describes the
	// EWA ellipse on the screen
	// note: column vectors are used, therefore the calculations are slightly
	// different to Heckbert's notation (cf. Maple file SurfaceSplatting2.mws)

	// the filtered variance matrix m[]
	// m[] = J*transpose(J)+I (details see SurfaceSplatting2.mws)
	r_ = 1.f / (sx * ty);
	r_ *= r_;							// r_ = 1/(sx*ty)^2, note that always tx=0, therefore some terms are missing
	m11 = (ty * ty + sy * sy) * r_ + 1.0f;	// in these formulas compared to the maple sheet
	m12 = -sy * sx * r_;
	m22 = sx * sx * r_ + 1.0f;

	// the filtered conic matric m[]^(-1)
	// matrix(A,B/2,B/2,C) = inv(m[])
	r_ = 1.f / (m11 * m22 - m12 * m12);
	a = m22 * r_;
	b_2 = -m12 * r_;
	b = b_2 * 2.f;
	c = m11 * r_;

	// calculate the normalization factor for the gaussian filter, which corresponds to the 
	// area of the reconstruction filter in source space, thus it is 
	// 1 / sqrt( det(inv(J)*transpose(inv(J))+I) )
	// note: the factor l_2 compensate for the level of detail of the LDC tree
	det_ = l_2 / (float)sqrt(sx * sx * ty * ty + sx * sx + sy * sy + ty * ty + 1.f);

	// bounding box of the ellipse
	// see ellipseboundingbox.mws, an exact axis aligned bounding box is computed by finding the points on
	// the ellipse where the tangent of the ellipse is parallel to x- and y-axis respectively.
	// NOTE: the variable "d" in the maple sheet corresponds to "-zbf_cutoffRadius_2"!
	discr = (float)sqrt((-b * b + 4 * c * a) * zbf_cutoffRadius_2 * a);
	ly = 2.f / (-b * b + 4 * c * a) * discr;

	discr = (float)sqrt(c * (-b * b + 4 * c * a) * zbf_cutoffRadius_2);
	lx = 2.f / (-b * b + 4 * c * a) * discr;

	lx = (lx < 0) ? -lx : lx;
	ly = (ly < 0) ? -ly : ly;
	Xmax = (int)(x0 + lx) + 1;
	Xmin = (int)(x0 - lx);
	Ymax = (int)(y0 + ly) + 1;
	Ymin = (int)(y0 - ly);

	// get surfel color
	color = surfel->getDiffuseColor();
	unpackRGB(r_comp, g_comp, b_comp, color);

	// get material attributes. in case we do per pixel shading, these attributes are written to 
	// the zBuffer, too.
	float _shd_kA = 0.5f;
	float _shd_kD = 0.75f;
	float _shd_kS = 0.25f;
	unsigned char _shd_shininess = (unsigned char)0;
	specularColor = surfel->getSpecularColor();

	// step 2: rasterize the EWA ellipse

	// padding
	clip = false;
	if (Xmin < 0) {
		Xmin = 0;
		clip = true;
		if (Xmax < 0)
			return 0;
	}
	if (Xmax >= width) {
		Xmax = width - 1;
		clip = true;
		if (Xmin >= width)
			return 0;
	}
	if (Ymin < 0) {
		Ymin = 0;
		clip = true;
		if (Ymax < 0)
			return 0;
	}
	if (Ymax >= height) {
		Ymax = height - 1;
		clip = true;
		if (Ymin >= height)
			return 0;
	}

	// set bounding box (the bounding box is a return value)
	bbox[0] = Xmin;
	bbox[1] = Ymin;
	bbox[2] = Xmax;
	bbox[3] = Ymax;

	// z value in the lower left corner of the rasterized area
	z_start = z - dzc_dxs * (x0 - ((float)Xmin + 0.5f)) - dzc_dys * (y0 - ((float)Ymin + 0.5f));

	x = ((float)Xmin + 0.5f) - x0;
	ddq = 2 * a;

	// *********************
	// ellipse rasterization
	// *********************
	for (Y = Ymin; Y <= Ymax; Y++)
	{
		// finite differences for ellipse rasterization
		y = ((float)Y + 0.5f) - y0;
		dq = a * (2 * x + 1) + b * y;
		q = (c * y + b * x) * y + a * x * x;

		// init z value
		z_cur = z_start;

		for (X = Xmin; X <= Xmax; X++)
		{
			i = X + width * Y;

			if (q < zbf_cutoffRadius_2)
			{

				// compare z-ranges
				if (!(zBuffer->buf[i].zMax < zMin || zMax < zBuffer->buf[i].zMin))
				{
					// z-ranges overlap
					// increase z-range if necessary
					zBuffer->buf[i].zMin = (zMin < zBuffer->buf[i].zMin) ? zMin : zBuffer->buf[i].zMin;
					zBuffer->buf[i].zMax = (zMax > zBuffer->buf[i].zMax) ? zMax : zBuffer->buf[i].zMax;

					// merge contributions
					w = zbf_filterLUT[(int)(q * _zbf_cutoffRadius_2 * zbf_LUTsize)] * det_;

					zBuffer->buf[i].w += w;

					// add color contribution
					zBuffer->buf[i].c[0] += r_comp * w;
					zBuffer->buf[i].c[1] += g_comp * w;
					zBuffer->buf[i].c[2] += b_comp * w;

					// normals
					zBuffer->buf[i].n[0] += n[0] * w;
					zBuffer->buf[i].n[1] += n[1] * w;
					zBuffer->buf[i].n[2] += n[2] * w;

					// z 
					zBuffer->buf[i].z += z_cur * w;

					// per pixel shading
					zBuffer->buf[i].kA = _shd_kA;
					zBuffer->buf[i].kD = _shd_kD;
					zBuffer->buf[i].kS = _shd_kS;
					zBuffer->buf[i].shininess = _shd_shininess;
					zBuffer->buf[i].specularColor = specularColor;
				}
				else if (zMin < zBuffer->buf[i].zMin) {
					// new z-range does not overlap previous one, but is closer to viewer
					// update z-range
					zBuffer->buf[i].zMin = zMin;
					zBuffer->buf[i].zMax = zMax;

					// new frontmost contribution
					w = zbf_filterLUT[(int)(q * _zbf_cutoffRadius_2 * zbf_LUTsize)] * det_;
					zBuffer->buf[i].w = w;

					// add color contribution
					zBuffer->buf[i].c[0] = r_comp * w;
					zBuffer->buf[i].c[1] = g_comp * w;
					zBuffer->buf[i].c[2] = b_comp * w;

					zBuffer->buf[i].n[0] = n[0] * w;
					zBuffer->buf[i].n[1] = n[1] * w;
					zBuffer->buf[i].n[2] = n[2] * w;

					// update z value
					zBuffer->buf[i].z = z_cur * w;

					// per pixel shading
					zBuffer->buf[i].kA = _shd_kA;
					zBuffer->buf[i].kD = _shd_kD;
					zBuffer->buf[i].kS = _shd_kS;
					zBuffer->buf[i].shininess = _shd_shininess;
					zBuffer->buf[i].specularColor = specularColor;
				}
			}
			q += dq;
			dq += ddq;

			z_cur += dzc_dxs;
		}
		z_start += dzc_dys;
	}

	return 0;
}

void CRenderer::setFrustum(float fofv, float aspect, float nearplane, float farplane)
{
	float t, b, l, r;
	float s;
	float* fv;

	// note: xP and yP are the half x,y-extent of the view frustum at z=1!
	warper->frustum.yP = (float)tan(fofv / 360.0 * M_PI);
	warper->frustum.xP = warper->frustum.yP * aspect;
	warper->frustum.nearplane = nearplane;
	warper->frustum.farplane = farplane;
	warper->frustum.xC = 0.f;
	warper->frustum.yC = 0.f;

	// top, bottom, left right coordinates for view frustum
	t = warper->frustum.yP * nearplane;
	b = -t;
	r = warper->frustum.xP * nearplane;
	l = -r;

	// camera coordinates of view frustum
	fv = warper->frustum.v;
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
	warper->transformation.scaling = s;

	// calculate inverse rotation and translation (needed for view frustum culling)
	MtrUtil::MtrInverse3x3f(r, ir);
	it[0] = -ir[0] * t[0] - ir[1] * t[1] - ir[2] * t[2];
	it[1] = -ir[3] * t[0] - ir[4] * t[1] - ir[5] * t[2];
	it[2] = -ir[6] * t[0] - ir[7] * t[1] - ir[8] * t[2];

	// calculate the matrix for transforming the normals, which is the transpose 
	// inverse of the model-view transformation matrix.
	// note: eliminate a possible uniform scaling from the rotation matrix, this 
	// guarantees that transformed unit normals are still of unit length!
	MtrUtil::MtrMultScal3x3f(r, 1 / s, tm);
	MtrUtil::MtrInverse3x3f(tm, tm2);
	MtrUtil::MtrTranspose3x3f(tm2, warper->transformation.normalsRotation);

	// set the member variables of 'wrp'
	MtrUtil::MtrCopy3x3f(r, warper->transformation.rotation);
	for (i = 0; i < 3; i++) warper->transformation.translation[i] = t[i];
}

//-------------------------------------------------------------------------
// Project a sample to the z-buffer
//-------------------------------------------------------------------------
// caution: this function relies on global variables!
void CRenderer::wrpProjectSample(int width, int height, Warper* warper, ZBuffer* zBuffer, CSurfel* surfel, int bbox[4])
{
	int i, j;

	float wrp_frustum_nearplane;
	float wrp_frustum_farplane;
	float vp_sx, vp_sy;				// scaling for viewport mapping
	float vp_tx[3];							// translation for viewport mapping, x direction
	float vp_ty[3];							// translation for viewport mapping, y direction
	float A[3][9];					// the transformation matrix (in 3 variants, one for each base plane)
	float v[3];						// the translation vector
	float normalsA[9];				// the transposed inverse of A for transforming normals from camera to object space
	float stoo_scaling;				// screen to camera scaling due to viewport mapping and uniform scaling in the modelview transform
	float otoc_scaling;				// object to camera space scaling, due to scaling in transformation matrix

	// static variables used for warping, which are independent of current block
	wrp_frustum_nearplane = warper->frustum.nearplane;
	wrp_frustum_farplane = warper->frustum.farplane;
	stoo_scaling = warper->frustum.xP * 2 / (width * warper->transformation.scaling);
	otoc_scaling = warper->transformation.scaling;

	// set transformation variables
	MtrUtil::MtrCopy3x3f(warper->transformation.rotation, A[0]);
	for (i = 0; i < 3; i++) v[i] = warper->transformation.translation[i];
	MtrUtil::MtrCopy3x3f(warper->transformation.normalsRotation, normalsA);

	// rotation matrices for yz- and zx-baseplanes
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			A[1][i * 3 + j] = A[0][i * 3 + (j + 1) % 3];
			A[2][i * 3 + j] = A[0][i * 3 + (j + 2) % 3];
		}
	}

	// set viewport mapping variables
	vp_sx = width / (2 * warper->frustum.xP);
	vp_sy = height / (2 * warper->frustum.yP);

	// set variables for warping from all 3 baseplanes
	for (i = 0; i < 3; i++)
	{
		vp_tx[i] = warper->frustum.xP - warper->frustum.xC;
		vp_ty[i] = warper->frustum.yP - warper->frustum.yC;
	}

	float x_c, y_c;             // camera-space x and y values
	float z_c, r_z_c;			// camera-space z-value (and its reciprocal) of sample being warped

	float xImg, yImg;			// x- and y-screen-coordinates of warped sample
	float xPad, yPad;			// pads in x and y direction for correct clipping

	Vector3D pos;				// object space sample position
	Vector3D nrm;				// object space normal
	Vector3D tan1,              // object space tangent axes
		tan2;
	float r;					// surfel radius
	float n[3];					// camera space normal
	int splatBBox[4];			// bounding box of rasterized splat

	// get sample position
	pos = surfel->getPosition();

	// apply transformation matrix
	z_c = A[0][6] * pos[0] + A[0][7] * pos[1] + A[0][8] * pos[2] + v[2];
	// apply near and far clipping planes
	if (z_c > wrp_frustum_nearplane && z_c < wrp_frustum_farplane) {

		x_c = A[0][0] * pos[0] + A[0][1] * pos[1] + A[0][2] * pos[2] + v[0];
		y_c = A[0][3] * pos[0] + A[0][4] * pos[1] + A[0][5] * pos[2] + v[1];

		// perspective divide and viewport transformation
		r_z_c = 1 / z_c;
		xImg = (x_c * r_z_c + vp_tx[0]) * vp_sx;
		yImg = (y_c * r_z_c + vp_ty[0]) * vp_sy;

		// for correct clipping: project surfel radius to screen space
		r = surfel->getRadius();
		r *= 1 / z_c;
		xPad = r * vp_sx;
		yPad = r * vp_sy;

		// put it into the z-buffer
		if ((xImg >= -xPad) && (xImg < width + xPad) &&
			(yImg >= -yPad) && (yImg < height + yPad))
		{
			// transform normal to camera coordinates
			nrm = surfel->getNormal();

			n[0] = normalsA[0] * nrm[0] + normalsA[1] * nrm[1] + normalsA[2] * nrm[2];
			n[1] = normalsA[3] * nrm[0] + normalsA[4] * nrm[1] + normalsA[5] * nrm[2];
			n[2] = normalsA[6] * nrm[0] + normalsA[7] * nrm[1] + normalsA[8] * nrm[2];

			// caution: this function (or macro) relies on global variables!
			// note: 'warped level' is set to 0
			zbfSurfaceSplat(width, height, zBuffer, xImg, yImg, z_c, n, surfel, 0, stoo_scaling, otoc_scaling, vp_sx, vp_sy, vp_tx[0], vp_ty[0], splatBBox);

			// enlarge bounding box (containing the modified frame buffer area) if necessary
			if (splatBBox[0] < bbox[0]) bbox[0] = splatBBox[0];
			if (splatBBox[1] < bbox[1]) bbox[1] = splatBBox[1];
			if (splatBBox[2] > bbox[2]) bbox[2] = splatBBox[2];
			if (splatBBox[3] > bbox[3]) bbox[3] = splatBBox[3];
		}
	}
}

unsigned char* CRenderer::getPixelPtr(const int x, const int y) {
	return m_image + (y * m_width + x) * 3;
}

void CRenderer::setColor(const int x, const int y, const COLORREF newPixelColor) {
	unsigned char* p = getPixelPtr(x, y);
	*p = GetRValue(newPixelColor);
	*(p + 1) = GetGValue(newPixelColor);
	*(p + 2) = GetBValue(newPixelColor);
}

//-------------------------------------------------------------------------
// Calculate phong shading for a sample
// note: this function uses the formulation with the reflection vector. for
// directional light sources, it produces exactly the same results as cube
// maps.
//-------------------------------------------------------------------------
void CRenderer::lightSamplePhongR(float _shd_kA, float _shd_kD, float _shd_kS, unsigned char _shd_shininess, MyDataTypes::RGBTriple _shd_specularColor)
{
	float Ir, Ig, Ib;
	float Ar, Ag, Ab;
	float Lx, Ly, Lz;
	float Rx, Ry, Rz;
	float t, r, ndotl, rdotv;
	int j;

	Ir = Ig = Ib = 1.0f;
	Ar = Ag = Ab = 0.5f;

	// ambient contribution
	t = _shd_kA;
	_shd_Ir.r += t * Ar * _shd_Id.r;
	_shd_Ir.g += t * Ag * _shd_Id.g;
	_shd_Ir.b += t * Ab * _shd_Id.b;

	Lx = Ly = 0.0f;
	Lz = -1.0f;

	// calculate the N*L dot product
	ndotl = _shd_nx_c * Lx + _shd_ny_c * Ly + _shd_nz_c * Lz;
	ndotl = (ndotl < 0 ? -ndotl : ndotl);

	// calculate normalized reflection vector
	Rx = 2 * _shd_nx_c * ndotl - Lx;
	Ry = 2 * _shd_ny_c * ndotl - Ly;
	Rz = 2 * _shd_nz_c * ndotl - Lz;

	// calculate R*V dot product
	rdotv = _shd_vx * Rx + _shd_vy * Ry + _shd_vz * Rz;
	rdotv = (rdotv < 0 ? -rdotv : rdotv);

	// calculate the phong shininess power
	r = rdotv;
	j = _shd_shininess;
	while (j > 0)
	{
		r *= rdotv;
		j--;
	}

	// increment intensities
	t = _shd_kD * ndotl;
	r *= _shd_kS;
	_shd_Ir.r += Ir * (t * _shd_Id.r + _shd_specularColor.r * r);
	_shd_Ir.g += Ig * (t * _shd_Id.g + _shd_specularColor.g * r);
	_shd_Ir.b += Ib * (t * _shd_Id.b + _shd_specularColor.b * r);
}

//-------------------------------------------------------------------------
// Shade the samples in the z-buffer of a warper
//-------------------------------------------------------------------------
void CRenderer::shadeZBuffer(int magfactor, int bbox[4])
{
	int zbf_rows, zbf_cols;
	float zbf_x_cur, zbf_y_cur;
	int zbf_i_cur;
	SrfZBufItem* zbf_buf = zBuffer->buf;

	Frustum* frst;

	//int xsize, ysize;				// x and y screen coordinates size			

	SrfZBufItem* zbufitem;				// current zbuffer item

	float vp_sx, vp_sy;				// variables for inverse viewport mapping
	float vp_tx, vp_ty;

	float w_, vec_len_;
	int c, specularColor;

	int start_dim_x, start_dim_y;	// writing the color values to the display image
	int end_dim_x, end_dim_y;		// (supports progressive rendering)
	int dim_y, dim_x;

	// initialise local variables for more efficient access
	frst = &(warper->frustum);

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
				_shd_Id.r = _shd_Ir.r = zbufitem->c[0] * w_;
				_shd_Id.g = _shd_Ir.g = zbufitem->c[1] * w_;
				_shd_Id.b = _shd_Ir.b = zbufitem->c[2] * w_;

				// re-normalize normal
				_shd_nx_c = zbufitem->n[0];
				_shd_ny_c = zbufitem->n[1];
				_shd_nz_c = zbufitem->n[2];
				w_ = 1.f / (float)sqrt(_shd_nx_c * _shd_nx_c + _shd_ny_c * _shd_ny_c + _shd_nz_c * _shd_nz_c);
				_shd_nx_c *= w_;
				_shd_ny_c *= w_;
				_shd_nz_c *= w_;

				// compute viewing vector
				_shd_vx = -(zbf_x_cur * vp_sx + vp_tx);
				_shd_vy = -(zbf_y_cur * vp_sy + vp_ty);
				_shd_vz = -1.f;
				vec_len_ = 1.f / (float)sqrt(_shd_vx * _shd_vx + _shd_vy * _shd_vy + 1.f);
				_shd_vx *= vec_len_; _shd_vy *= vec_len_; _shd_vz *= vec_len_;

				float _shd_kA = zbufitem->kA;
				float _shd_kD = zbufitem->kD;
				float _shd_kS = zbufitem->kS;
				unsigned char _shd_shininess = zbufitem->shininess;
				specularColor = zbufitem->specularColor;
				MyDataTypes::RGBTriple _shd_specularColor;
				unpackRGB(_shd_specularColor.r, _shd_specularColor.g, _shd_specularColor.b, specularColor);

				_shd_Ir.r = 0.f;
				_shd_Ir.g = 0.f;
				_shd_Ir.b = 0.f;
				lightSamplePhongR(_shd_kA, _shd_kD, _shd_kS, _shd_shininess, _shd_specularColor);

				// clamp color intensities
				if (_shd_Ir.r > 255.0) _shd_Ir.r = 255.0;
				if (_shd_Ir.g > 255.0) _shd_Ir.g = 255.0;
				if (_shd_Ir.b > 255.0) _shd_Ir.b = 255.0;

				// pack color to format for packed RGB arithmetic
				c = (((int)_shd_Ir.r << 16) & 0x00ff0000) | (((int)_shd_Ir.g << 8) & 0x0000ff00) | ((int)_shd_Ir.b & 0x000000ff);

				// write to display image, blow up pixels for progressive rendering
				start_dim_x = zbf_cols * magfactor;
				end_dim_x = start_dim_x + magfactor;
				start_dim_y = zbf_rows * magfactor;
				end_dim_y = start_dim_y + magfactor;

				// write output color to frame buffer
				for (dim_y = start_dim_y; dim_y < end_dim_y; dim_y++)
				{
					for (dim_x = start_dim_x; dim_x < end_dim_x; dim_x++)
					{
						setColor(dim_x, dim_y, c);
					}
				}
			}
			else {
				// write to display image, blow up pixels for progressive rendering
				start_dim_x = zbf_cols * magfactor;
				end_dim_x = start_dim_x + magfactor;
				start_dim_y = zbf_rows * magfactor;
				end_dim_y = start_dim_y + magfactor;
				for (dim_y = start_dim_y; dim_y < end_dim_y; dim_y++)
				{
					for (dim_x = start_dim_x; dim_x < end_dim_x; dim_x++)
					{
						setColor(dim_x, dim_y, m_backgroundColor);
					}
				}
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