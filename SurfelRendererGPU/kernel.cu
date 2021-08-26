#include <cassert>
#include <iostream>

#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <GLFW/glfw3.h>

#include "TypeHelper.h"
#include "Vector3D.h"

void surfaceSplatStep1(int width, int height, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, float x0, float y0, float z, float n[3], Surfel* surfel, int l, float scale_stoo, float scale_otoc, float vp_sx, float vp_sy, float vp_tx, float vp_ty, int bbox[4])
{
	float zbf_LUTsize = zBufferProperty->LUTsize;
	float zbf_cutoffRadius_2 = zBufferProperty->cutoffRadius * zBufferProperty->cutoffRadius;
	float _zbf_cutoffRadius_2 = 1 / (zbf_cutoffRadius_2);
	float zbf_angleThreshold = zBufferProperty->angleTrheshold;

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
	float r_comp, g_comp, b_comp;

	float threshold_c3;
	float e;
	float w;

	float l_2, _l_2;
	float det_;

	Vector3D normal;
	float _radius;

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

	_radius = surfel->radius;
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
	r_comp = surfel->r;
	g_comp = surfel->g;
	b_comp = surfel->b;

	// step 2: rasterize the EWA ellipse

	// padding
	if (Xmin < 0) {
		Xmin = 0;
		if (Xmax < 0)
			return;
	}
	if (Xmax >= width) {
		Xmax = width - 1;
		if (Xmin >= width)
			return;
	}
	if (Ymin < 0) {
		Ymin = 0;
		if (Ymax < 0)
			return;
	}
	if (Ymax >= height) {
		Ymax = height - 1;
		if (Ymin >= height)
			return;
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
				if (zMin < zBuffer[i].zMin) {
					// new z-range does not overlap previous one, but is closer to viewer
					// update z-range
					zBuffer[i].zMin = zMin;
					zBuffer[i].zMax = zMax;
				}
			}
			q += dq;
			dq += ddq;

			z_cur += dzc_dxs;
		}
		z_start += dzc_dys;
	}
}

void surfaceSplatStep2(int width, int height, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, float x0, float y0, float z, float n[3], Surfel* surfel, int l, float scale_stoo, float scale_otoc, float vp_sx, float vp_sy, float vp_tx, float vp_ty, int bbox[4])
{
	float zbf_LUTsize = zBufferProperty->LUTsize;
	float zbf_cutoffRadius_2 = zBufferProperty->cutoffRadius * zBufferProperty->cutoffRadius;
	float _zbf_cutoffRadius_2 = 1 / (zbf_cutoffRadius_2);
	float zbf_angleThreshold = zBufferProperty->angleTrheshold;

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
	float r_comp, g_comp, b_comp;

	float threshold_c3;
	float e;
	float w;

	float l_2, _l_2;
	float det_;

	Vector3D normal;
	float _radius;

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

	_radius = surfel->radius;
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
	r_comp = surfel->r;
	g_comp = surfel->g;
	b_comp = surfel->b;

	// step 2: rasterize the EWA ellipse

	// padding
	if (Xmin < 0) {
		Xmin = 0;
		if (Xmax < 0)
			return;
	}
	if (Xmax >= width) {
		Xmax = width - 1;
		if (Xmin >= width)
			return;
	}
	if (Ymin < 0) {
		Ymin = 0;
		if (Ymax < 0)
			return;
	}
	if (Ymax >= height) {
		Ymax = height - 1;
		if (Ymin >= height)
			return;
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
				if (zMin <= zBuffer[i].zMax)
				{
					// merge contributions
					w = filterLUT[(int)(q * _zbf_cutoffRadius_2 * zbf_LUTsize)] * det_;

					zBuffer[i].w += w;

					// add color contribution
					zBuffer[i].c[0] += r_comp * w;
					zBuffer[i].c[1] += g_comp * w;
					zBuffer[i].c[2] += b_comp * w;

					// normals
					zBuffer[i].n[0] += n[0] * w;
					zBuffer[i].n[1] += n[1] * w;
					zBuffer[i].n[2] += n[2] * w;
				}
			}	
			q += dq;
			dq += ddq;

			z_cur += dzc_dxs;
		}
		z_start += dzc_dys;
	}
}

void projectSampleStep1(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, Surfel* surfel, int bbox[4])
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
	memcpy(A[0], warper->transformation.rotation, sizeof(float) * 9);
	//MtrCopy3x3f(warper->transformation.rotation, A[0]);
	for (i = 0; i < 3; i++) v[i] = warper->transformation.translation[i];
	memcpy(normalsA, warper->transformation.normalsRotation, sizeof(float) * 9);
	//MtrCopy3x3f(warper->transformation.normalsRotation, normalsA);

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
	pos = surfel->position;

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
		r = surfel->radius;
		r *= 1 / z_c;
		xPad = r * vp_sx;
		yPad = r * vp_sy;

		// put it into the z-buffer
		if ((xImg >= -xPad) && (xImg < width + xPad) &&
			(yImg >= -yPad) && (yImg < height + yPad))
		{
			// transform normal to camera coordinates
			nrm = surfel->normal;

			n[0] = normalsA[0] * nrm[0] + normalsA[1] * nrm[1] + normalsA[2] * nrm[2];
			n[1] = normalsA[3] * nrm[0] + normalsA[4] * nrm[1] + normalsA[5] * nrm[2];
			n[2] = normalsA[6] * nrm[0] + normalsA[7] * nrm[1] + normalsA[8] * nrm[2];

			// caution: this function (or macro) relies on global variables!
			// note: 'warped level' is set to 0
			surfaceSplatStep1(width, height, zBufferProperty, zBuffer, filterLUT, xImg, yImg, z_c, n, surfel, 0, stoo_scaling, otoc_scaling, vp_sx, vp_sy, vp_tx[0], vp_ty[0], splatBBox);

			// enlarge bounding box (containing the modified frame buffer area) if necessary
			if (splatBBox[0] < bbox[0]) bbox[0] = splatBBox[0];
			if (splatBBox[1] < bbox[1]) bbox[1] = splatBBox[1];
			if (splatBBox[2] > bbox[2]) bbox[2] = splatBBox[2];
			if (splatBBox[3] > bbox[3]) bbox[3] = splatBBox[3];
		}
	}
}

void projectSampleStep2(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, Surfel* surfel, int bbox[4])
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
	memcpy(A[0], warper->transformation.rotation, sizeof(float) * 9);
	//MtrCopy3x3f(warper->transformation.rotation, A[0]);
	for (i = 0; i < 3; i++) v[i] = warper->transformation.translation[i];
	memcpy(normalsA, warper->transformation.normalsRotation, sizeof(float) * 9);
	//MtrCopy3x3f(warper->transformation.normalsRotation, normalsA);

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
	pos = surfel->position;

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
		r = surfel->radius;
		r *= 1 / z_c;
		xPad = r * vp_sx;
		yPad = r * vp_sy;

		// put it into the z-buffer
		if ((xImg >= -xPad) && (xImg < width + xPad) &&
			(yImg >= -yPad) && (yImg < height + yPad))
		{
			// transform normal to camera coordinates
			nrm = surfel->normal;

			n[0] = normalsA[0] * nrm[0] + normalsA[1] * nrm[1] + normalsA[2] * nrm[2];
			n[1] = normalsA[3] * nrm[0] + normalsA[4] * nrm[1] + normalsA[5] * nrm[2];
			n[2] = normalsA[6] * nrm[0] + normalsA[7] * nrm[1] + normalsA[8] * nrm[2];

			// caution: this function (or macro) relies on global variables!
			// note: 'warped level' is set to 0
			surfaceSplatStep2(width, height, zBufferProperty, zBuffer, filterLUT, xImg, yImg, z_c, n, surfel, 0, stoo_scaling, otoc_scaling, vp_sx, vp_sy, vp_tx[0], vp_ty[0], splatBBox);

			// enlarge bounding box (containing the modified frame buffer area) if necessary
			if (splatBBox[0] < bbox[0]) bbox[0] = splatBBox[0];
			if (splatBBox[1] < bbox[1]) bbox[1] = splatBBox[1];
			if (splatBBox[2] > bbox[2]) bbox[2] = splatBBox[2];
			if (splatBBox[3] > bbox[3]) bbox[3] = splatBBox[3];
		}
	}
}

void project(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, int numSurfels, Surfel* surfels, int bbox[4]) {
	for (int i = 0; i < numSurfels; i++)
		projectSampleStep1(width, height, warper, zBufferProperty, zBuffer, filterLUT, &surfels[i], bbox);
	for (int i = 0; i < numSurfels; i++)
		projectSampleStep2(width, height, warper, zBufferProperty, zBuffer, filterLUT, &surfels[i], bbox);
}

__device__ static float atomicMin(float* address, float val)
{
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
			__float_as_int(::fminf(val, __int_as_float(assumed))));
	} while (assumed != old);
	return __int_as_float(old);
}

__device__ static float atomicMax(float* address, float val)
{
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
			__float_as_int(::fmaxf(val, __int_as_float(assumed))));
	} while (assumed != old);
	return __int_as_float(old);
}

__device__ void surfaceSplatStep1GPU(int width, int height, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, float x0, float y0, float z, float n[3], Surfel* surfel, int l, float scale_stoo, float scale_otoc, float vp_sx, float vp_sy, float vp_tx, float vp_ty, int* bbox)
{
	float zbf_LUTsize = zBufferProperty->LUTsize;
	float zbf_cutoffRadius_2 = zBufferProperty->cutoffRadius * zBufferProperty->cutoffRadius;
	float _zbf_cutoffRadius_2 = 1 / zbf_cutoffRadius_2;
	float zbf_angleThreshold = zBufferProperty->angleTrheshold;

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
	float r_comp, g_comp, b_comp;

	float threshold_c3;
	float e;
	float w;

	float l_2, _l_2;
	float det_;

	Vector3D normal;
	float _radius;

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

	_radius = surfel->radius;
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
	r_comp = surfel->r;
	g_comp = surfel->g;
	b_comp = surfel->b;

	// step 2: rasterize the EWA ellipse

	// padding
	if (Xmin < 0) {
		Xmin = 0;
		if (Xmax < 0)
			return;
	}
	if (Xmax >= width) {
		Xmax = width - 1;
		if (Xmin >= width)
			return;
	}
	if (Ymin < 0) {
		Ymin = 0;
		if (Ymax < 0)
			return;
	}
	if (Ymax >= height) {
		Ymax = height - 1;
		if (Ymin >= height)
			return;
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
				atomicMin(&zBuffer[i].zMin, zMin);
			q += dq;
			dq += ddq;

			z_cur += dzc_dxs;
		}
		z_start += dzc_dys;
	}
}

__device__ void surfaceSplatStep2GPU(int width, int height, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, float x0, float y0, float z, float n[3], Surfel* surfel, int l, float scale_stoo, float scale_otoc, float vp_sx, float vp_sy, float vp_tx, float vp_ty, int* bbox)
{
	float zbf_LUTsize = zBufferProperty->LUTsize;
	float zbf_cutoffRadius_2 = zBufferProperty->cutoffRadius * zBufferProperty->cutoffRadius;
	float _zbf_cutoffRadius_2 = 1 / zbf_cutoffRadius_2;
	float zbf_angleThreshold = zBufferProperty->angleTrheshold;

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
	float r_comp, g_comp, b_comp;

	float threshold_c3;
	float e;
	float w;

	float l_2, _l_2;
	float det_;

	Vector3D normal;
	float _radius;

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

	_radius = surfel->radius;
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
	r_comp = surfel->r;
	g_comp = surfel->g;
	b_comp = surfel->b;

	// step 2: rasterize the EWA ellipse

	// padding
	if (Xmin < 0) {
		Xmin = 0;
		if (Xmax < 0)
			return;
	}
	if (Xmax >= width) {
		Xmax = width - 1;
		if (Xmin >= width)
			return;
	}
	if (Ymin < 0) {
		Ymin = 0;
		if (Ymax < 0)
			return;
	}
	if (Ymax >= height) {
		Ymax = height - 1;
		if (Ymin >= height)
			return;
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
				if (zMin == zBuffer[i].zMin)
					atomicMax(&zBuffer[i].zMax, zMax);
			q += dq;
			dq += ddq;

			z_cur += dzc_dxs;
		}
		z_start += dzc_dys;
	}
}

__device__ void surfaceSplatStep3GPU(int width, int height, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, float x0, float y0, float z, float n[3], Surfel* surfel, int l, float scale_stoo, float scale_otoc, float vp_sx, float vp_sy, float vp_tx, float vp_ty, int* bbox)
{
	float zbf_LUTsize = zBufferProperty->LUTsize;
	float zbf_cutoffRadius_2 = zBufferProperty->cutoffRadius * zBufferProperty->cutoffRadius;
	float _zbf_cutoffRadius_2 = 1 / zbf_cutoffRadius_2;
	float zbf_angleThreshold = zBufferProperty->angleTrheshold;

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
	float r_comp, g_comp, b_comp;

	float threshold_c3;
	float e;
	float w;

	float l_2, _l_2;
	float det_;

	Vector3D normal;
	float _radius;

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

	_radius = surfel->radius;
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
	r_comp = surfel->r;
	g_comp = surfel->g;
	b_comp = surfel->b;

	// step 2: rasterize the EWA ellipse

	// padding
	if (Xmin < 0) {
		Xmin = 0;
		if (Xmax < 0)
			return;
	}
	if (Xmax >= width) {
		Xmax = width - 1;
		if (Xmin >= width)
			return;
	}
	if (Ymin < 0) {
		Ymin = 0;
		if (Ymax < 0)
			return;
	}
	if (Ymax >= height) {
		Ymax = height - 1;
		if (Ymin >= height)
			return;
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

			if (q < zbf_cutoffRadius_2) {
				if (zMin <= zBuffer[i].zMax)
				{
					// merge contributions
					w = filterLUT[(int)(q * _zbf_cutoffRadius_2 * zbf_LUTsize)] * det_;

					atomicAdd(&zBuffer[i].w, w);

					// add color contribution
					atomicAdd(&zBuffer[i].c[0], r_comp * w);
					atomicAdd(&zBuffer[i].c[1], g_comp * w);
					atomicAdd(&zBuffer[i].c[2], b_comp * w);

					// normals
					atomicAdd(&zBuffer[i].n[0], n[0] * w);
					atomicAdd(&zBuffer[i].n[1], n[1] * w);
					atomicAdd(&zBuffer[i].n[2], n[2] * w);
				}
			}
			q += dq;
			dq += ddq;

			z_cur += dzc_dxs;
		}
		z_start += dzc_dys;
	}
}

__device__ void projectSampleStep1GPU(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, Surfel* surfel, int* bbox)
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
	for (i = 0; i < 9; i++)
		A[0][i] = normalsA[i] = warper->transformation.rotation[i];
	for (i = 0; i < 3; i++) v[i] = warper->transformation.translation[i];

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
	pos = surfel->position;

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
		r = surfel->radius;
		r *= 1 / z_c;
		xPad = r * vp_sx;
		yPad = r * vp_sy;

		// put it into the z-buffer
		if ((xImg >= -xPad) && (xImg < width + xPad) &&
			(yImg >= -yPad) && (yImg < height + yPad))
		{
			// transform normal to camera coordinates
			nrm = surfel->normal;

			n[0] = normalsA[0] * nrm[0] + normalsA[1] * nrm[1] + normalsA[2] * nrm[2];
			n[1] = normalsA[3] * nrm[0] + normalsA[4] * nrm[1] + normalsA[5] * nrm[2];
			n[2] = normalsA[6] * nrm[0] + normalsA[7] * nrm[1] + normalsA[8] * nrm[2];

			// caution: this function (or macro) relies on global variables!
			// note: 'warped level' is set to 0
			surfaceSplatStep1GPU(width, height, zBufferProperty, zBuffer, filterLUT, xImg, yImg, z_c, n, surfel, 0, stoo_scaling, otoc_scaling, vp_sx, vp_sy, vp_tx[0], vp_ty[0], splatBBox);

			// enlarge bounding box (containing the modified frame buffer area) if necessary
			atomicMin(&bbox[0], splatBBox[0]);
			atomicMin(&bbox[1], splatBBox[1]);
			atomicMax(&bbox[2], splatBBox[2]);
			atomicMax(&bbox[3], splatBBox[3]);
		}
	}
}

__device__ void projectSampleStep2GPU(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, Surfel* surfel, int* bbox)
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
	for (i = 0; i < 9; i++)
		A[0][i] = normalsA[i] = warper->transformation.rotation[i];
	for (i = 0; i < 3; i++) v[i] = warper->transformation.translation[i];

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
	pos = surfel->position;

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
		r = surfel->radius;
		r *= 1 / z_c;
		xPad = r * vp_sx;
		yPad = r * vp_sy;

		// put it into the z-buffer
		if ((xImg >= -xPad) && (xImg < width + xPad) &&
			(yImg >= -yPad) && (yImg < height + yPad))
		{
			// transform normal to camera coordinates
			nrm = surfel->normal;

			n[0] = normalsA[0] * nrm[0] + normalsA[1] * nrm[1] + normalsA[2] * nrm[2];
			n[1] = normalsA[3] * nrm[0] + normalsA[4] * nrm[1] + normalsA[5] * nrm[2];
			n[2] = normalsA[6] * nrm[0] + normalsA[7] * nrm[1] + normalsA[8] * nrm[2];

			// caution: this function (or macro) relies on global variables!
			// note: 'warped level' is set to 0
			surfaceSplatStep2GPU(width, height, zBufferProperty, zBuffer, filterLUT, xImg, yImg, z_c, n, surfel, 0, stoo_scaling, otoc_scaling, vp_sx, vp_sy, vp_tx[0], vp_ty[0], splatBBox);

			// enlarge bounding box (containing the modified frame buffer area) if necessary
			atomicMin(&bbox[0], splatBBox[0]);
			atomicMin(&bbox[1], splatBBox[1]);
			atomicMax(&bbox[2], splatBBox[2]);
			atomicMax(&bbox[3], splatBBox[3]);
		}
	}
}

__device__ void projectSampleStep3GPU(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, Surfel* surfel, int* bbox)
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
	for (i = 0; i < 9; i++)
		A[0][i] = normalsA[i] = warper->transformation.rotation[i];
	for (i = 0; i < 3; i++) v[i] = warper->transformation.translation[i];

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
	pos = surfel->position;

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
		r = surfel->radius;
		r *= 1 / z_c;
		xPad = r * vp_sx;
		yPad = r * vp_sy;

		// put it into the z-buffer
		if ((xImg >= -xPad) && (xImg < width + xPad) &&
			(yImg >= -yPad) && (yImg < height + yPad))
		{
			// transform normal to camera coordinates
			nrm = surfel->normal;

			n[0] = normalsA[0] * nrm[0] + normalsA[1] * nrm[1] + normalsA[2] * nrm[2];
			n[1] = normalsA[3] * nrm[0] + normalsA[4] * nrm[1] + normalsA[5] * nrm[2];
			n[2] = normalsA[6] * nrm[0] + normalsA[7] * nrm[1] + normalsA[8] * nrm[2];

			// caution: this function (or macro) relies on global variables!
			// note: 'warped level' is set to 0
			surfaceSplatStep3GPU(width, height, zBufferProperty, zBuffer, filterLUT, xImg, yImg, z_c, n, surfel, 0, stoo_scaling, otoc_scaling, vp_sx, vp_sy, vp_tx[0], vp_ty[0], splatBBox);

			// enlarge bounding box (containing the modified frame buffer area) if necessary
			atomicMin(&bbox[0], splatBBox[0]);
			atomicMin(&bbox[1], splatBBox[1]);
			atomicMax(&bbox[2], splatBBox[2]);
			atomicMax(&bbox[3], splatBBox[3]);
		}
	}
}

__global__ void kernelStep1(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, int numSurfels, Surfel* surfels, int* bbox) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	while (i < numSurfels) {
		projectSampleStep1GPU(width, height, warper, zBufferProperty, zBuffer, filterLUT, &surfels[i], bbox);
		i += gridDim.x * blockDim.x;
	}
}

__global__ void kernelStep2(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, int numSurfels, Surfel* surfels, int* bbox) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	while (i < numSurfels) {
		projectSampleStep2GPU(width, height, warper, zBufferProperty, zBuffer, filterLUT, &surfels[i], bbox);
		i += gridDim.x * blockDim.x;
	}
}

__global__ void kernelStep3(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, int numSurfels, Surfel* surfels, int* bbox) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	while (i < numSurfels) {
		projectSampleStep3GPU(width, height, warper, zBufferProperty, zBuffer, filterLUT, &surfels[i], bbox);
		i += gridDim.x * blockDim.x;
	}
}

void projectGPU(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, int numSurfels, Surfel* surfels, int bbox[4]) {
	double t0 = glfwGetTime();

	Warper* warperGpu;
	cudaMalloc(&warperGpu, sizeof(Warper));
	cudaMemcpy(warperGpu, warper, sizeof(Warper), cudaMemcpyHostToDevice);

	int* bboxGpu;
	cudaMalloc(&bboxGpu, sizeof(int) * 4);
	cudaMemcpy(bboxGpu, bbox, sizeof(int) * 4, cudaMemcpyHostToDevice);

	double t1 = glfwGetTime();

	kernelStep1<<<512, 512>>>(width, height, warperGpu, zBufferProperty, zBuffer, filterLUT, numSurfels, surfels, bboxGpu);
	kernelStep2<<<512, 512>>>(width, height, warperGpu, zBufferProperty, zBuffer, filterLUT, numSurfels, surfels, bboxGpu);
	kernelStep3<<<512, 512>>>(width, height, warperGpu, zBufferProperty, zBuffer, filterLUT, numSurfels, surfels, bboxGpu);

	cudaError_t cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess)
		fprintf(stderr, "Kernel launch failed: %s\n", cudaGetErrorString(cudaStatus));

	double t2 = glfwGetTime();

	//cudaMemcpy(zBuffer, zBufferGpu, sizeof(ZBufferItem) * zBufferProperty->bufsize, cudaMemcpyDeviceToHost);
	cudaMemcpy(bbox, bboxGpu, sizeof(int) * 4, cudaMemcpyDeviceToHost);

	double t3 = glfwGetTime();
	//std::cout << t1 - t0 << ' ' << t2 - t1 << ' ' << t3 - t2 << std::endl;

	cudaFree(warperGpu);
	cudaFree(bboxGpu);
}