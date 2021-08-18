#ifndef Z_BUFFER_H
#define Z_BUFFER_H

// unpack R,G,B components
#define UNPACK_FLOAT_RGB(_r,_g,_b,_c) \
	_r = (float)((_c & 0x00ff0000) >> 16); \
	_g = (float)((_c & 0x0000ff00) >> 8); \
	_b = (float)(_c & 0x000000ff)

#include "CSurfel.h"

// export the z-buffer item structure for fast external access to 
// data stored in the z-buffer
typedef struct _SrfZBufItem
{
	float z;				// the reconstructed camera space z-value of a surfel
	float zMin;				// z-range used for blending
	float zMax;
	float w;				// accumulated weights
	float n[3];				// accumulated normal
	float c[3];				// accumulated color values
	unsigned char alpha;	// composite alpha value

	float kA;
	float kD;
	float kS;
	unsigned char shininess;
	unsigned int specularColor;

	void* userData;			// pointer to user data
} SrfZBufItem;

typedef struct _ZBuffer
{
	int xsize;
	int ysize;
	SrfZBufItem* buf;
	SrfZBufItem* cleardata;
	long bufsize;

	int options;

	//ZBfSplatSurfel splatFunction;

	// surface splatting parameters
	float cutoffRadius;
	float covthreshold;			// hack for compatibility with srf.c/.h, this variable is not used here.
	int viscontrib;				// dito
	float recscaling;			// dito

	// blending thresholds used for surface splatting
	float constThreshold;
	float distThreshold;
	float angleTrheshold;

	// reconstruction filter table used for surface splatting
	int LUTsize;
	float* filterLUT;

	// the extended frame buffer which holds data for
	// interactive manipulation
	//FrameBufferInterface* frameBuffer;
} ZBuffer;

// Global variables
int zbf_rows, zbf_cols;
float zbf_x_cur, zbf_y_cur;
int zbf_i_cur;
int zbf_xsize, zbf_ysize;
int zbf_material;
SrfZBufItem* zbf_buf;

// Static variables
static float zbf_constThreshold;
static float zbf_distThreshold;
static float zbf_angleThreshold;
static float* zbf_filterLUT;
static int zbf_LUTsize;
static float zbf_cutoffRadius_2;
static float _zbf_cutoffRadius_2;


// Extern variables
extern float _shd_nx_c, _shd_ny_c, _shd_nz_c;
extern float _shd_vx, _shd_vy, _shd_vz;
extern float _shd_ndotv;
extern float _shd_kA, _shd_kD, _shd_kS;
extern unsigned char _shd_shininess;
extern MyDataTypes::RGBTriple _shd_specularColor;
extern MyDataTypes::RGBTriple _shd_Id;
extern MyDataTypes::RGBTriple _shd_Ir;

//-------------------------------------------------------------------------
// Prepare for writing
//-------------------------------------------------------------------------
static void zbfPrepareForWriting(ZBuffer* zbf)
{
	zbf_buf = zbf->buf;
	zbf_xsize = zbf->xsize;
	zbf_ysize = zbf->ysize;
	zbf_filterLUT = zbf->filterLUT;
	zbf_LUTsize = zbf->LUTsize;
	zbf_cutoffRadius_2 = zbf->cutoffRadius * zbf->cutoffRadius;
	_zbf_cutoffRadius_2 = 1 / (zbf_cutoffRadius_2);
	_zbf_cutoffRadius_2 = zbf->constThreshold;
	zbf_distThreshold = zbf->distThreshold;
	zbf_angleThreshold = zbf->angleTrheshold;
}

//-------------------------------------------------------------------------
// Prepare for reading
//-------------------------------------------------------------------------
void zbfPrepareForReading(ZBuffer* zbf)
{
	zbf_buf = zbf->buf;
	zbf_xsize = zbf->xsize;
	zbf_ysize = zbf->ysize;
}

//-------------------------------------------------------------------------
// Splat a surfel into the z-buffer
//-------------------------------------------------------------------------
static int zbfSurfaceSplat(ZBuffer* zbf, float x0, float y0, float z, float n[3], CSurfel* surfel, int l,
	float scale_stoo, float scale_otoc, float vp_sx, float vp_sy, float vp_tx, float vp_ty, int bbox[4]/*, Object* object*/)
{
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

	int attributes;
	int adjustedBufferHeight;  // = buffer height - 1            

	bool clip;

	//scale z buffer according to surfel radius
	float scale_otoc_radius;

	// init bounding box return value
	bbox[0] = zbf_xsize;
	bbox[1] = zbf_ysize;
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
		//if (zbf->options & ZBF_BIDINORMALS) {
		//	// two-sided lighting, flip normals
		//	n[0] = -n[0];
		//	n[1] = -n[1];
		//	n[2] = -n[2];
			ndotv = -ndotv;
		//}
		//else {
		//	// backface culling
		//	return 0;
		//}
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
	UNPACK_FLOAT_RGB(r_comp, g_comp, b_comp, color);

	// modify color if surfel is selected and visualization of the selection is enabled
	/*if (surfel->isFlagOn(SurfelInterface::EMPHASISE) == true &&
		(zbf->options & ZBF_VISUALIZESELECTION)) {
		if (surfel->isFlagOn(SurfelInterface::SELECTED1) == true) {
			r_comp *= 0.5f;
			g_comp *= 0.5f;
			b_comp = b_comp * 0.5f + 122.0f;
		}
		if (surfel->isFlagOn(SurfelInterface::SELECTED2) == true) {
			r_comp = r_comp * 0.5f + 122.0f;
			g_comp *= 0.5f;
			b_comp *= 0.5f;
		}
		if (surfel->isFlagOn(SurfelInterface::SELECTED3) == true) {
			r_comp *= 0.5f;
			g_comp = g_comp * 0.5f + 122.0f;
			b_comp *= 0.5f;
		}
	}*/

	// if in shadow adjust color values
	/*if ((zbf->options & ZBF_SHADOWS) && (surfel->isFlagOn(SurfelInterface::COVERED) == true)) {
		r_comp = 0.0f;
		g_comp = 0.0f;
		b_comp = 0.0f;
	}*/

	// get material attributes. in case we do per pixel shading, these attributes are written to 
	// the zBuffer, too.
	_shd_kA = 0.5f;
	_shd_kD = 0.75f;
	_shd_kS = 0.25f;
	_shd_shininess = (unsigned char)0;
	specularColor = surfel->getSpecularColor();
	UNPACK_FLOAT_RGB(_shd_specularColor.r, _shd_specularColor.g, _shd_specularColor.b, specularColor);

	//if (zbf->options & ZBF_PERSURFELSHADING) {
		// perform per surfel shading
		// setup variables for shading
		_shd_nx_c = n[0];
		_shd_ny_c = n[1];
		_shd_nz_c = n[2];

		_shd_vx = V_x;
		_shd_vy = V_y;
		_shd_vz = V_z;
		_shd_ndotv = ndotv;

		_shd_Id.r = _shd_Ir.r = r_comp;
		_shd_Id.g = _shd_Ir.g = g_comp;
		_shd_Id.b = _shd_Ir.b = b_comp;

		// perform shading
#ifdef _USE_NORMAL_
		if (_shd_options & SHD_LIGHTING)
		{
			_shd_Ir.r = 0.f;
			_shd_Ir.g = 0.f;
			_shd_Ir.b = 0.f;
			_shd_lightsample();
		}
#endif

		// re-assign colors for further processing
		r_comp = _shd_Ir.r;
		g_comp = _shd_Ir.g;
		b_comp = _shd_Ir.b;
	//}

	// step 2: rasterize the EWA ellipse

	// padding
	clip = false;
	if (Xmin < 0) {
		Xmin = 0;
		clip = true;
		if (Xmax < 0)
			return 0;
	}
	if (Xmax >= zbf_xsize) {
		Xmax = zbf_xsize - 1;
		clip = true;
		if (Xmin >= zbf_xsize)
			return 0;
	}
	if (Ymin < 0) {
		Ymin = 0;
		clip = true;
		if (Ymax < 0)
			return 0;
	}
	if (Ymax >= zbf_ysize) {
		Ymax = zbf_ysize - 1;
		clip = true;
		if (Ymin >= zbf_ysize)
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

	// write splat data to framebuffer if required
	//attributes = zbf->frameBuffer->getAttributes();

	/*if (attributes & FrameBufferInterface::ALL_SPLATS && clip == false) {
		zbf->frameBuffer->setSplatInfo(surfel, x0, y0, z, a, b_2, c, bbox);
	}*/

	// *********************
	// ellipse rasterization
	// *********************
	//if ((attributes & FrameBufferInterface::PERPIXEL_C_Z_N_W) || (attributes & FrameBufferInterface::PERPIXEL_SURFELLISTS))
	//{

		adjustedBufferHeight = zbf->ysize - 1;

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
				i = X + zbf_xsize * Y;

				if (q < zbf_cutoffRadius_2)
				{

					// compare z-ranges
					if (!(zbf->buf[i].zMax < zMin || zMax < zbf->buf[i].zMin))
					{
						// z-ranges overlap
						// increase z-range if necessary
						zbf->buf[i].zMin = (zMin < zbf->buf[i].zMin) ? zMin : zbf->buf[i].zMin;
						zbf->buf[i].zMax = (zMax > zbf->buf[i].zMax) ? zMax : zbf->buf[i].zMax;

						// merge contributions
						w = zbf_filterLUT[(int)(q * _zbf_cutoffRadius_2 * zbf_LUTsize)] * det_;

						zbf->buf[i].w += w;

						// add color contribution
						zbf->buf[i].c[0] += r_comp * w;
						zbf->buf[i].c[1] += g_comp * w;
						zbf->buf[i].c[2] += b_comp * w;

						// normals
						zbf->buf[i].n[0] += n[0] * w;
						zbf->buf[i].n[1] += n[1] * w;
						zbf->buf[i].n[2] += n[2] * w;

						// z 
						zbf->buf[i].z += z_cur * w;

						// per pixel shading
						//if (!(zbf->options & ZBF_PERSURFELSHADING)) {
							zbf->buf[i].kA = _shd_kA;
							zbf->buf[i].kD = _shd_kD;
							zbf->buf[i].kS = _shd_kS;
							zbf->buf[i].shininess = _shd_shininess;
							zbf->buf[i].specularColor = specularColor;
						//}

						// *******************************
						// write additional per pixel data
						// *******************************

						/*if (attributes & FrameBufferInterface::PERPIXEL_SURFELLISTS) {
							zbf->frameBuffer->addVisibleSurfel(X, adjustedBufferHeight - Y, surfel);
						}
						if (attributes & FrameBufferInterface::PERPIXEL_SURFEL) {
							zbf->buf[i].userData = surfel;
						}
						if (attributes & FrameBufferInterface::PERPIXEL_OBJECTPOINTER) {
							zbf->frameBuffer->setObjectAtPixel(X, adjustedBufferHeight - Y, object);
						}*/

					}
					else if (zMin < zbf->buf[i].zMin) {
						// new z-range does not overlap previous one, but is closer to viewer
						// update z-range
						zbf->buf[i].zMin = zMin;
						zbf->buf[i].zMax = zMax;

						// new frontmost contribution
						w = zbf_filterLUT[(int)(q * _zbf_cutoffRadius_2 * zbf_LUTsize)] * det_;
						zbf->buf[i].w = w;

						// add color contribution
						zbf->buf[i].c[0] = r_comp * w;
						zbf->buf[i].c[1] = g_comp * w;
						zbf->buf[i].c[2] = b_comp * w;

						zbf->buf[i].n[0] = n[0] * w;
						zbf->buf[i].n[1] = n[1] * w;
						zbf->buf[i].n[2] = n[2] * w;

						// update z value
						zbf->buf[i].z = z_cur * w;

						// per pixel shading
						//if (!(zbf->options & ZBF_PERSURFELSHADING)) {
							zbf->buf[i].kA = _shd_kA;
							zbf->buf[i].kD = _shd_kD;
							zbf->buf[i].kS = _shd_kS;
							zbf->buf[i].shininess = _shd_shininess;
							zbf->buf[i].specularColor = specularColor;
						//}

						// **************************************
						// reset per pixel data, write new values
						// **************************************

						/*if (attributes & FrameBufferInterface::PERPIXEL_SURFELLISTS) {
							zbf->frameBuffer->resetPosition(X, adjustedBufferHeight - Y);
							zbf->frameBuffer->addVisibleSurfel(X, adjustedBufferHeight - Y, surfel);
						}
						if (attributes & FrameBufferInterface::PERPIXEL_SURFEL) {
							zbf->buf[i].userData = surfel;
						}
						if (attributes & FrameBufferInterface::PERPIXEL_OBJECTPOINTER) {
							zbf->frameBuffer->setObjectAtPixel(X, adjustedBufferHeight - Y, object);
						}*/

					}
				}

				q += dq;
				dq += ddq;

				z_cur += dzc_dxs;
			}
			z_start += dzc_dys;
		}
	//}

	return 0;
}

#endif