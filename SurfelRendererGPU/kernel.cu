#include <iostream>

#include <GLFW/glfw3.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include "TypeHelper.h"
#include "Vector3D.h"

void calculateAttributes(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, Surfel* surfel) {
	float wrp_frustum_nearplane, wrp_frustum_farplane;
	float zbf_cutoffRadius_2, zbf_angleThreshold;
	float vp_sx, vp_sy;				// scaling for viewport mapping
	float vp_tx, vp_ty;				// translation for viewport mapping
	float A[3][9];					// the transformation matrix (in 3 variants, one for each base plane)
	float v[3];						// the translation vector
	float normalsA[9];				// the transposed inverse of A for transforming normals from camera to object space
	float stoo_scaling;				// screen to camera scaling due to viewport mapping and uniform scaling in the modelview transform
	float otoc_scaling;				// object to camera space scaling, due to scaling in transformation matrix
	
	int i, j;

	float x_c, y_c;             // camera-space x and y values
	float z_c, r_z_c;			// camera-space z-value (and its reciprocal) of sample being warped

	float xImg, yImg;			// x- and y-screen-coordinates of warped sample
	float xPad, yPad;			// pads in x and y direction for correct clipping

	Vector3D pos;				// object space sample position
	Vector3D nrm;				// object space normal

	float radius, _radius;		// surfel radius
	float n[3];					// camera space normal

	float V_x, V_y, V_z;		// viewing direction
	float S_x, S_y, S_z;		// S parameter direction on ST plane
	float T_x, T_y, T_z;		// T parameter direction on ST plane
	float Ix_x, Ix_y, Ix_z;		// direction of projection of screen x vector onto ST plane
	float Iy_x, Iy_y, Iy_z;		// direction of projection of screen y vector onto ST plane
	float r, r_, f;
	float ndotv;				// N*V (normal dot viewing direction) dotproduct
	float sx, sy, ty;		// derivatives of the screen to ST plane mapping
	float m11, m12, m22;
	float a, b, b_2, c;			// the EWA ellipse coefficients

	int xMin, xMax, yMin, yMax;	// bounding box of the ellipse to be rasterized
	float lx, ly;

	float dzc_dxs, dzc_dys;			// dzc/dxs, dzc/dys derivatives

	float e;

	float det_;

	bool flag;

	//scale z buffer according to surfel radius
	float scale_otoc_radius;
	float discr, zExtremum_x, zExtremum_y, tmp, zRange_x, zRange_y, zMin, zMax;

	// static variables used for warping, which are independent of current block
	wrp_frustum_nearplane = warper->frustum.nearplane;
	wrp_frustum_farplane = warper->frustum.farplane;
	zbf_cutoffRadius_2 = zBufferProperty->cutoffRadius * zBufferProperty->cutoffRadius;
	zbf_angleThreshold = zBufferProperty->angleTrheshold;
	stoo_scaling = warper->frustum.xP * 2 / (width * warper->transformation.scaling);
	otoc_scaling = warper->transformation.scaling;

	// set transformation variables
	for (i = 0; i < 9; i++)
		A[0][i] = warper->transformation.rotation[i];
	for (i = 0; i < 3; i++)
		v[i] = warper->transformation.translation[i];
	for (i = 0; i < 9; i++)
		normalsA[i] = warper->transformation.normalsRotation[i];

	// rotation matrices for yz- and zx-baseplanes
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++) {
			A[1][i * 3 + j] = A[0][i * 3 + (j + 1) % 3];
			A[2][i * 3 + j] = A[0][i * 3 + (j + 2) % 3];
		}

	// set viewport mapping variables
	vp_sx = width / (2 * warper->frustum.xP);
	vp_sy = height / (2 * warper->frustum.yP);

	// set variables for warping from all 3 baseplanes
	vp_tx = warper->frustum.xP - warper->frustum.xC;
	vp_ty = warper->frustum.yP - warper->frustum.yC;

	// get sample position
	pos = surfel->position;

	// apply transformation matrix
	z_c = A[0][6] * pos[0] + A[0][7] * pos[1] + A[0][8] * pos[2] + v[2];
	// apply near and far clipping planes
	flag = false;
	if (z_c > wrp_frustum_nearplane && z_c < wrp_frustum_farplane) {
		x_c = A[0][0] * pos[0] + A[0][1] * pos[1] + A[0][2] * pos[2] + v[0];
		y_c = A[0][3] * pos[0] + A[0][4] * pos[1] + A[0][5] * pos[2] + v[1];

		// perspective divide and viewport transformation
		r_z_c = 1 / z_c;
		xImg = (x_c * r_z_c + vp_tx) * vp_sx;
		yImg = (y_c * r_z_c + vp_ty) * vp_sy;

		// for correct clipping: project surfel radius to screen space
		radius = surfel->radius;
		radius *= r_z_c;
		xPad = radius * vp_sx;
		yPad = radius * vp_sy;

		// put it into the z-buffer
		if ((xImg >= -xPad) && (xImg < width + xPad) && (yImg >= -yPad) && (yImg < height + yPad)) {
			// transform normal to camera coordinates
			nrm = surfel->normal;

			n[0] = normalsA[0] * nrm[0] + normalsA[1] * nrm[1] + normalsA[2] * nrm[2];
			n[1] = normalsA[3] * nrm[0] + normalsA[4] * nrm[1] + normalsA[5] * nrm[2];
			n[2] = normalsA[6] * nrm[0] + normalsA[7] * nrm[1] + normalsA[8] * nrm[2];

			// caution: this function (or macro) relies on global variables!
			// note: 'warped level' is set to 0

			// thresholds for the 'merge/separate' decision
			// note: 
			// - these thresholds are dependent on the level of the warped samples, since the
			// distance on the surface between samples increases with the warped level.
			// - these offsets should be constant in units 'dz per unit in object space'. but since 
			// z coordinates in the z-buffer are in camera coordinates, we have to transform the offsets
			// to camera space by applying the 'scale_otoc' (object to camera space) scaling

			// step 1: calculate the EWA ellipse coefficients

			// compute normalized viewing vector V
			// restore camera coordinates of projected point (on z=1 plane)
			V_x = -(xImg / vp_sx - vp_tx);
			V_y = -(yImg / vp_sy - vp_ty);
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
			e = f < -zbf_angleThreshold ? -zbf_angleThreshold : f;
			// note: since z coordinates in the z buffer are camera coordinate z-values, we have
			// to transform from screen to camera coordinates (by concatenating the screen-to-object
			// and object-to-camera scalings).
			dzc_dxs = Ix_z * e * stoo_scaling * otoc_scaling * z_c;
			dzc_dys = Iy_z * e * stoo_scaling * otoc_scaling * z_c;

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
			scale_otoc_radius = otoc_scaling * _radius;
			_radius = 1.0f / _radius;

			Iy_x *= f; Iy_y *= f; Iy_z *= f;		// f*Iy is the intersection point with Y+g*V
			sx = r * f * stoo_scaling * z_c * _radius;		// note f given by the vector equation X+g*V=f*Ix takes the same value as above
			sy = (Iy_x * S_x + Iy_y * S_y + Iy_z * S_z) * stoo_scaling * z_c * _radius;		// Iy*S projects Iy onto S
			ty = (Iy_x * T_x + Iy_y * T_y + Iy_z * T_z) * stoo_scaling * z_c * _radius;		// Iy*T projects Iy onto T

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
			discr = -4 * a * dzc_dxs * b * dzc_dys * c - a * dzc_dys * dzc_dys * b * b + 4 * c * dzc_dys * dzc_dys * a * a +
				4 * a * dzc_dxs * dzc_dxs * c * c + b * b * b * dzc_dxs * dzc_dys - b * b * dzc_dxs * dzc_dxs * c;

			tmp = (float)sqrt(discr * zbf_cutoffRadius_2);
			zExtremum_x = tmp * (-dzc_dys * b + 2 * dzc_dxs * c) / discr;
			zExtremum_y = tmp * (-2 * dzc_dys * a + dzc_dxs * b) / discr;

			tmp = zExtremum_x * dzc_dxs;
			zRange_x = (tmp < 0) ? -tmp : tmp;
			tmp = zExtremum_y * dzc_dys;
			zRange_y = (tmp < 0) ? -tmp : tmp;

			zMax = z_c + zRange_x + zRange_y;
			zMin = z_c - zRange_x - zRange_y;

			// guarantee a certain minimum z-range, otherwise blending fails for splats exactly parallel to the
			// image plane (the minimum z-range is 1 in object space, and then scaled to camera space).
			if (zMax - zMin < scale_otoc_radius) {
				zMax += 0.5f * scale_otoc_radius;
				zMin -= 0.5f * scale_otoc_radius;
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
			det_ = 1.0f / (float)sqrt(sx * sx * ty * ty + sx * sx + sy * sy + ty * ty + 1.f);

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
			xMax = (int)(xImg + lx) + 1;
			xMin = (int)(xImg - lx);
			yMax = (int)(yImg + ly) + 1;
			yMin = (int)(yImg - ly);

			// step 2: rasterize the EWA ellipse

			// padding
			if (xMin < 0) {
				xMin = 0;
				if (xMax < 0)
					xMax = -1;
			}
			if (xMax >= width) {
				xMax = width - 1;
				if (xMin >= width)
					xMin = width;
			}
			if (yMin < 0) {
				yMin = 0;
				if (yMax < 0)
					yMax = -1;
			}
			if (yMax >= height) {
				yMax = height - 1;
				if (yMin >= height)
					yMin = height;
			}

			surfel->xMin = xMin;
			surfel->xMax = xMax;
			surfel->yMin = yMin;
			surfel->yMax = yMax;
			surfel->area = (xMax - xMin + 1) * (yMax - yMin + 1);
			surfel->zMin = zMin;
			surfel->zMax = zMax;
			surfel->x0 = xImg;
			surfel->y0 = yImg;
			surfel->a = a;
			surfel->b = b;
			surfel->c = c;
			surfel->det_ = det_;
			surfel->n[0] = n[0];
			surfel->n[1] = n[1];
			surfel->n[2] = n[2];

			flag = true;
		}
	}
	if (!flag) {
		surfel->xMin = 0;
		surfel->xMax = -1;
		surfel->yMin = 0;
		surfel->yMax = -1;
		surfel->area = 0;
	}
}

void surfaceSplatStep1(int width, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, Surfel* surfel) {
	float zbf_cutoffRadius_2;

	int xMin, xMax, yMin, yMax;	// bounding box of the ellipse to be rasterized
	float zMin, zMax, x0, y0, a, b, c;
	int X, Y;
	float x, y, q;

	ZBufferItem* zbufitem;

	zbf_cutoffRadius_2 = zBufferProperty->cutoffRadius * zBufferProperty->cutoffRadius;

	xMin = surfel->xMin;
	xMax = surfel->xMax;
	yMin = surfel->yMin;
	yMax = surfel->yMax;
	zMin = surfel->zMin;
	zMax = surfel->zMax;
	x0 = surfel->x0;
	y0 = surfel->y0;
	a = surfel->a;
	b = surfel->b;
	c = surfel->c;

	// *********************
	// ellipse rasterization
	// *********************
	for (Y = yMin; Y <= yMax; Y++)
		for (X = xMin; X <= xMax; X++) {
			y = (float)Y + 0.5f - y0;
			x = (float)X + 0.5f - x0;
			q = a * x * x + b * x * y + c * y * y;
			zbufitem = &zBuffer[X + width * Y];

			if (q < zbf_cutoffRadius_2)
				if (zMin < zbufitem->zMin) {
					zbufitem->zMin = zMin;
					zbufitem->zMax = zMax;
				}
		}
}

void surfaceSplatStep2(int width, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, Surfel* surfel) {
	float zbf_LUTsize, zbf_cutoffRadius_2, _zbf_cutoffRadius_2;
	
	int xMin, xMax, yMin, yMax;	// bounding box of the ellipse to be rasterized
	float zMin, x0, y0, a, b, c, det_, n[3], red, green, blue;
	int X, Y;
	float x, y, q, w;

	ZBufferItem* zbufitem;

	zbf_LUTsize = (float)zBufferProperty->LUTsize;
	zbf_cutoffRadius_2 = zBufferProperty->cutoffRadius * zBufferProperty->cutoffRadius;
	_zbf_cutoffRadius_2 = 1 / (zbf_cutoffRadius_2);

	xMin = surfel->xMin;
	xMax = surfel->xMax;
	yMin = surfel->yMin;
	yMax = surfel->yMax;
	zMin = surfel->zMin;
	x0 = surfel->x0;
	y0 = surfel->y0;
	a = surfel->a;
	b = surfel->b;
	c = surfel->c;
	det_ = surfel->det_;
	n[0] = surfel->n[0];
	n[1] = surfel->n[1];
	n[2] = surfel->n[2];
	red = surfel->red;
	green = surfel->green;
	blue = surfel->blue;

	// *********************
	// ellipse rasterization
	// *********************
	for (Y = yMin; Y <= yMax; Y++)
		for (X = xMin; X <= xMax; X++) {
			y = (float)Y + 0.5f - y0;
			x = (float)X + 0.5f - x0;
			q = a * x * x + b * x * y + c * y * y;
			zbufitem = &zBuffer[X + width * Y];

			if (q < zbf_cutoffRadius_2)
				if (zMin <= zbufitem->zMax) {
					// merge contributions
					w = filterLUT[(int)(q * _zbf_cutoffRadius_2 * zbf_LUTsize)] * det_;

					zbufitem->w += w;

					// add color contribution
					zbufitem->c[0] += red * w;
					zbufitem->c[1] += green * w;
					zbufitem->c[2] += blue * w;

					// normals
					zbufitem->n[0] += n[0] * w;
					zbufitem->n[1] += n[1] * w;
					zbufitem->n[2] += n[2] * w;
				}
		}
}

void project(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, int numSurfels, Surfel* surfels) {
	for (int i = 0; i < numSurfels; i++)
		calculateAttributes(width, height, warper, zBufferProperty, &surfels[i]);
	for (int i = 0; i < numSurfels; i++)
		surfaceSplatStep1(width, zBufferProperty, zBuffer, &surfels[i]);
	for (int i = 0; i < numSurfels; i++)
		surfaceSplatStep2(width, zBufferProperty, zBuffer, filterLUT, &surfels[i]);
}

__device__ void calculateAttributesGpu(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, Surfel* surfel, int* sum) {
	__shared__ float wrp_frustum_nearplane, wrp_frustum_farplane;
	__shared__ float zbf_cutoffRadius_2, zbf_angleThreshold;
	__shared__ float vp_sx, vp_sy;				// scaling for viewport mapping
	__shared__ float vp_tx, vp_ty;				// translation for viewport mapping
	__shared__ float A[3][9];					// the transformation matrix (in 3 variants, one for each base plane)
	__shared__ float v[3];						// the translation vector
	__shared__ float normalsA[9];				// the transposed inverse of A for transforming normals from camera to object space
	__shared__ float stoo_scaling;				// screen to camera scaling due to viewport mapping and uniform scaling in the modelview transform
	__shared__ float otoc_scaling;				// object to camera space scaling, due to scaling in transformation matrix

	int i, j;

	float x_c, y_c;             // camera-space x and y values
	float z_c, r_z_c;			// camera-space z-value (and its reciprocal) of sample being warped

	float xImg, yImg;			// x- and y-screen-coordinates of warped sample
	float xPad, yPad;			// pads in x and y direction for correct clipping

	Vector3D pos;				// object space sample position
	Vector3D nrm;				// object space normal

	float radius, _radius;		// surfel radius
	float n[3];					// camera space normal

	float V_x, V_y, V_z;		// viewing direction
	float S_x, S_y, S_z;		// S parameter direction on ST plane
	float T_x, T_y, T_z;		// T parameter direction on ST plane
	float Ix_x, Ix_y, Ix_z;		// direction of projection of screen x vector onto ST plane
	float Iy_x, Iy_y, Iy_z;		// direction of projection of screen y vector onto ST plane
	float r, r_, f;
	float ndotv;				// N*V (normal dot viewing direction) dotproduct
	float sx, sy, ty;		// derivatives of the screen to ST plane mapping
	float m11, m12, m22;
	float a, b, b_2, c;			// the EWA ellipse coefficients

	int xMin, xMax, yMin, yMax;	// bounding box of the ellipse to be rasterized
	float lx, ly;

	float dzc_dxs, dzc_dys;			// dzc/dxs, dzc/dys derivatives

	float e;

	float det_;

	bool flag;

	//scale z buffer according to surfel radius
	float scale_otoc_radius;
	float discr, zExtremum_x, zExtremum_y, tmp, zRange_x, zRange_y, zMin, zMax;

	// static variables used for warping, which are independent of current block
	wrp_frustum_nearplane = warper->frustum.nearplane;
	wrp_frustum_farplane = warper->frustum.farplane;
	zbf_cutoffRadius_2 = zBufferProperty->cutoffRadius * zBufferProperty->cutoffRadius;
	zbf_angleThreshold = zBufferProperty->angleTrheshold;
	stoo_scaling = warper->frustum.xP * 2 / (width * warper->transformation.scaling);
	otoc_scaling = warper->transformation.scaling;

	// set transformation variables
	for (i = 0; i < 9; i++)
		A[0][i] = warper->transformation.rotation[i];
	for (i = 0; i < 3; i++)
		v[i] = warper->transformation.translation[i];
	for (i = 0; i < 9; i++)
		normalsA[i] = warper->transformation.normalsRotation[i];

	// rotation matrices for yz- and zx-baseplanes
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++) {
			A[1][i * 3 + j] = A[0][i * 3 + (j + 1) % 3];
			A[2][i * 3 + j] = A[0][i * 3 + (j + 2) % 3];
		}

	// set viewport mapping variables
	vp_sx = width / (2 * warper->frustum.xP);
	vp_sy = height / (2 * warper->frustum.yP);

	// set variables for warping from all 3 baseplanes
	vp_tx = warper->frustum.xP - warper->frustum.xC;
	vp_ty = warper->frustum.yP - warper->frustum.yC;

	// get sample position
	pos = surfel->position;

	// apply transformation matrix
	z_c = A[0][6] * pos[0] + A[0][7] * pos[1] + A[0][8] * pos[2] + v[2];
	// apply near and far clipping planes
	flag = false;
	if (z_c > wrp_frustum_nearplane && z_c < wrp_frustum_farplane) {
		x_c = A[0][0] * pos[0] + A[0][1] * pos[1] + A[0][2] * pos[2] + v[0];
		y_c = A[0][3] * pos[0] + A[0][4] * pos[1] + A[0][5] * pos[2] + v[1];

		// perspective divide and viewport transformation
		r_z_c = 1 / z_c;
		xImg = (x_c * r_z_c + vp_tx) * vp_sx;
		yImg = (y_c * r_z_c + vp_ty) * vp_sy;

		// for correct clipping: project surfel radius to screen space
		radius = surfel->radius;
		radius *= r_z_c;
		xPad = radius * vp_sx;
		yPad = radius * vp_sy;

		// put it into the z-buffer
		if ((xImg >= -xPad) && (xImg < width + xPad) && (yImg >= -yPad) && (yImg < height + yPad)) {
			// transform normal to camera coordinates
			nrm = surfel->normal;

			n[0] = normalsA[0] * nrm[0] + normalsA[1] * nrm[1] + normalsA[2] * nrm[2];
			n[1] = normalsA[3] * nrm[0] + normalsA[4] * nrm[1] + normalsA[5] * nrm[2];
			n[2] = normalsA[6] * nrm[0] + normalsA[7] * nrm[1] + normalsA[8] * nrm[2];

			// caution: this function (or macro) relies on global variables!
			// note: 'warped level' is set to 0

			// thresholds for the 'merge/separate' decision
			// note: 
			// - these thresholds are dependent on the level of the warped samples, since the
			// distance on the surface between samples increases with the warped level.
			// - these offsets should be constant in units 'dz per unit in object space'. but since 
			// z coordinates in the z-buffer are in camera coordinates, we have to transform the offsets
			// to camera space by applying the 'scale_otoc' (object to camera space) scaling

			// step 1: calculate the EWA ellipse coefficients

			// compute normalized viewing vector V
			// restore camera coordinates of projected point (on z=1 plane)
			V_x = -(xImg / vp_sx - vp_tx);
			V_y = -(yImg / vp_sy - vp_ty);
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
			e = f < -zbf_angleThreshold ? -zbf_angleThreshold : f;
			// note: since z coordinates in the z buffer are camera coordinate z-values, we have
			// to transform from screen to camera coordinates (by concatenating the screen-to-object
			// and object-to-camera scalings).
			dzc_dxs = Ix_z * e * stoo_scaling * otoc_scaling * z_c;
			dzc_dys = Iy_z * e * stoo_scaling * otoc_scaling * z_c;

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
			scale_otoc_radius = otoc_scaling * _radius;
			_radius = 1.0f / _radius;

			Iy_x *= f; Iy_y *= f; Iy_z *= f;		// f*Iy is the intersection point with Y+g*V
			sx = r * f * stoo_scaling * z_c * _radius;		// note f given by the vector equation X+g*V=f*Ix takes the same value as above
			sy = (Iy_x * S_x + Iy_y * S_y + Iy_z * S_z) * stoo_scaling * z_c * _radius;		// Iy*S projects Iy onto S
			ty = (Iy_x * T_x + Iy_y * T_y + Iy_z * T_z) * stoo_scaling * z_c * _radius;		// Iy*T projects Iy onto T

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
			discr = -4 * a * dzc_dxs * b * dzc_dys * c - a * dzc_dys * dzc_dys * b * b + 4 * c * dzc_dys * dzc_dys * a * a +
				4 * a * dzc_dxs * dzc_dxs * c * c + b * b * b * dzc_dxs * dzc_dys - b * b * dzc_dxs * dzc_dxs * c;

			tmp = (float)sqrt(discr * zbf_cutoffRadius_2);
			zExtremum_x = tmp * (-dzc_dys * b + 2 * dzc_dxs * c) / discr;
			zExtremum_y = tmp * (-2 * dzc_dys * a + dzc_dxs * b) / discr;

			tmp = zExtremum_x * dzc_dxs;
			zRange_x = (tmp < 0) ? -tmp : tmp;
			tmp = zExtremum_y * dzc_dys;
			zRange_y = (tmp < 0) ? -tmp : tmp;

			zMax = z_c + zRange_x + zRange_y;
			zMin = z_c - zRange_x - zRange_y;

			// guarantee a certain minimum z-range, otherwise blending fails for splats exactly parallel to the
			// image plane (the minimum z-range is 1 in object space, and then scaled to camera space).
			if (zMax - zMin < scale_otoc_radius) {
				zMax += 0.5f * scale_otoc_radius;
				zMin -= 0.5f * scale_otoc_radius;
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
			det_ = 1.0f / (float)sqrt(sx * sx * ty * ty + sx * sx + sy * sy + ty * ty + 1.f);

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
			xMax = (int)(xImg + lx) + 1;
			xMin = (int)(xImg - lx);
			yMax = (int)(yImg + ly) + 1;
			yMin = (int)(yImg - ly);

			// step 2: rasterize the EWA ellipse

			// padding
			if (xMin < 0) {
				xMin = 0;
				if (xMax < 0)
					xMax = -1;
			}
			if (xMax >= width) {
				xMax = width - 1;
				if (xMin >= width)
					xMin = width;
			}
			if (yMin < 0) {
				yMin = 0;
				if (yMax < 0)
					yMax = -1;
			}
			if (yMax >= height) {
				yMax = height - 1;
				if (yMin >= height)
					yMin = height;
			}

			surfel->xMin = xMin;
			surfel->xMax = xMax;
			surfel->yMin = yMin;
			surfel->yMax = yMax;
			surfel->area = (xMax - xMin + 1) * (yMax - yMin + 1);
			surfel->zMin = zMin;
			surfel->zMax = zMax;
			surfel->x0 = xImg;
			surfel->y0 = yImg;
			surfel->a = a;
			surfel->b = b;
			surfel->c = c;
			surfel->det_ = det_;
			surfel->n[0] = n[0];
			surfel->n[1] = n[1];
			surfel->n[2] = n[2];

			flag = true;
		}
	}
	if (!flag) {
		surfel->xMin = 0;
		surfel->xMax = -1;
		surfel->yMin = 0;
		surfel->yMax = -1;
		surfel->area = 0;
	}
	*sum = surfel->area;
}

__device__ static float atomicMin(float* address, float val) {
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed, __float_as_int(::fminf(val, __int_as_float(assumed))));
	} while (assumed != old);
	return __int_as_float(old);
}

__device__ static float atomicMax(float* address, float val) {
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed, __float_as_int(::fmaxf(val, __int_as_float(assumed))));
	} while (assumed != old);
	return __int_as_float(old);
}

__device__ void surfaceSplatStep1Gpu(int width, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, Surfel* surfel, int index) {
	__shared__ float zbf_cutoffRadius_2;

	int yDiff, X, Y;
	float x, y, q;

	ZBufferItem* zbufitem;

	zbf_cutoffRadius_2 = zBufferProperty->cutoffRadius * zBufferProperty->cutoffRadius;

	yDiff = surfel->yMax - surfel->yMin + 1;
	Y = surfel->yMin + index % yDiff;
	X = surfel->xMin + index / yDiff;

	y = (float)Y + 0.5f - surfel->y0;
	x = (float)X + 0.5f - surfel->x0;
	q = surfel->a * x * x + surfel->b * x * y + surfel->c * y * y;
	zbufitem = &zBuffer[X + width * Y];

	if (q < zbf_cutoffRadius_2)
		atomicMin(&zbufitem->zMin, surfel->zMin);
}

__device__ void surfaceSplatStep2Gpu(int width, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, Surfel* surfel, int index) {
	__shared__ float zbf_cutoffRadius_2;

	int yDiff, X, Y;
	float x, y, q;

	ZBufferItem* zbufitem;

	zbf_cutoffRadius_2 = zBufferProperty->cutoffRadius * zBufferProperty->cutoffRadius;

	yDiff = surfel->yMax - surfel->yMin + 1;
	Y = surfel->yMin + index % yDiff;
	X = surfel->xMin + index / yDiff;

	y = (float)Y + 0.5f - surfel->y0;
	x = (float)X + 0.5f - surfel->x0;
	q = surfel->a * x * x + surfel->b * x * y + surfel->c * y * y;
	zbufitem = &zBuffer[X + width * Y];

	if (q < zbf_cutoffRadius_2 && surfel->zMin == zbufitem->zMin)
		atomicMax(&zbufitem->zMax, surfel->zMax);
}

__device__ void surfaceSplatStep3Gpu(int width, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, Surfel* surfel, int index) {
	__shared__ float zbf_LUTsize, zbf_cutoffRadius_2, _zbf_cutoffRadius_2;

	int yDiff, X, Y;
	float x, y, q, w;

	ZBufferItem* zbufitem;

	zbf_LUTsize = zBufferProperty->LUTsize;
	zbf_cutoffRadius_2 = zBufferProperty->cutoffRadius * zBufferProperty->cutoffRadius;
	_zbf_cutoffRadius_2 = 1 / zbf_cutoffRadius_2;

	yDiff = surfel->yMax - surfel->yMin + 1;
	Y = surfel->yMin + index % yDiff;
	X = surfel->xMin + index / yDiff;

	y = (float)Y + 0.5f - surfel->y0;
	x = (float)X + 0.5f - surfel->x0;
	q = surfel->a * x * x + surfel->b * x * y + surfel->c * y * y;
	zbufitem = &zBuffer[X + width * Y];

	if (q < zbf_cutoffRadius_2 && surfel->zMin <= zbufitem->zMax) {
		// merge contributions
		w = filterLUT[(int)(q * _zbf_cutoffRadius_2 * zbf_LUTsize)] * surfel->det_;

		atomicAdd(&zbufitem->w, w);

		// add color contribution
		atomicAdd(&zbufitem->c[0], surfel->red * w);
		atomicAdd(&zbufitem->c[1], surfel->green * w);
		atomicAdd(&zbufitem->c[2], surfel->blue * w);

		// normals
		atomicAdd(&zbufitem->n[0], surfel->n[0] * w);
		atomicAdd(&zbufitem->n[1], surfel->n[1] * w);
		atomicAdd(&zbufitem->n[2], surfel->n[2] * w);
	}
}

__global__ void calculateAttributesKernel(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, int numSurfels, Surfel* surfels, int* sum) {
	int i;
	
	i = blockIdx.x * blockDim.x + threadIdx.x;
	while (i < numSurfels) {
		calculateAttributesGpu(width, height, warper, zBufferProperty, &surfels[i], &sum[i + 1]);
		i += gridDim.x * blockDim.x;
	}
}

/*__global__ void surfaceSplatStep1Kernel(int width, int height, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, int numSurfels, Surfel* surfels) {
	int i, j;
	
	i = blockIdx.x;
	while (i < numSurfels) {
		j = threadIdx.x;
		__syncthreads();
		while (j < surfels[i].area) {
			surfaceSplatStep1Gpu(width, zBufferProperty, zBuffer, &surfels[i], j);
			j += blockDim.x;
		}
		i += gridDim.x;
	}
}

__global__ void surfaceSplatStep2Kernel(int width, int height, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, int numSurfels, Surfel* surfels) {
	int i, j;
	
	i = blockIdx.x;
	while (i < numSurfels) {
		j = threadIdx.x;
		__syncthreads();
		while (j < surfels[i].area) {
			surfaceSplatStep2Gpu(width, zBufferProperty, zBuffer, &surfels[i], j);
			j += blockDim.x;
		}
		i += gridDim.x;
	}
}

__global__ void surfaceSplatStep3Kernel(int width, int height, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, int numSurfels, Surfel* surfels) {
	int i, j;

	i = blockIdx.x;
	while (i < numSurfels) {
		j = threadIdx.x;
		__syncthreads();
		while (j < surfels[i].area) {
			surfaceSplatStep3Gpu(width, zBufferProperty, zBuffer, filterLUT, &surfels[i], j);
			j += blockDim.x;
		}
		i += gridDim.x;
	}
}*/

__global__ void surfaceSplatStep1Kernel(int width, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, int numSurfels, Surfel* surfels, int* sum) {
	__shared__ int sumArea;
	int i, l, r, mid;

	sumArea = sum[numSurfels - 1];
	i = blockIdx.x * blockDim.x + threadIdx.x;
	while (i < sumArea) {
		l = 0;
		r = numSurfels + 1;
		while (r - l > 1) {
			mid = (l + r) >> 1;
			sum[mid] <= i ? l = mid : r = mid;
		}
		if (r <= numSurfels)
			surfaceSplatStep1Gpu(width, zBufferProperty, zBuffer, &surfels[l], i - sum[l]);

		i += gridDim.x * blockDim.x;
	}
}

__global__ void surfaceSplatStep2Kernel(int width, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, int numSurfels, Surfel* surfels, int* sum) {
	__shared__ int sumArea;
	int i, l, r, mid;

	sumArea = sum[numSurfels - 1];
	i = blockIdx.x * blockDim.x + threadIdx.x;
	while (i < sumArea) {
		l = 0;
		r = numSurfels + 1;
		while (r - l > 1) {
			mid = (l + r) >> 1;
			sum[mid] <= i ? l = mid : r = mid;
		}
		if (r <= numSurfels)
			surfaceSplatStep2Gpu(width, zBufferProperty, zBuffer, &surfels[l], i - sum[l]);

		i += gridDim.x * blockDim.x;
	}
}

__global__ void surfaceSplatStep3Kernel(int width, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, int numSurfels, Surfel* surfels, int* sum) {
	__shared__ int sumArea;
	int i, l, r, mid;

	sumArea = sum[numSurfels - 1];
	i = blockIdx.x * blockDim.x + threadIdx.x;
	while (i < sumArea) {
		l = 0;
		r = numSurfels + 1;
		while (r - l > 1) {
			mid = (l + r) >> 1;
			sum[mid] <= i ? l = mid : r = mid;
		}
		if (r <= numSurfels)
			surfaceSplatStep3Gpu(width, zBufferProperty, zBuffer, filterLUT, &surfels[l], i - sum[l]);

		i += gridDim.x * blockDim.x;
	}
}

void projectGpu(int width, int height, Warper* warper, ZBufferProperty* zBufferProperty, ZBufferItem* zBuffer, float* filterLUT, int numSurfels, Surfel* surfels) {
	int* sum, * sumGpu;
	cudaMalloc(&sumGpu, sizeof(int) * (numSurfels + 1));

	calculateAttributesKernel<<<512, 512>>>(width, height, warper, zBufferProperty, zBuffer, filterLUT, numSurfels, surfels, sumGpu);
	sum = new int[numSurfels + 1];
	cudaMemcpy(sum, sumGpu, sizeof(int) * (numSurfels + 1), cudaMemcpyDeviceToHost);
	for (int i = 1; i <= numSurfels; i++)
		sum[i] += sum[i - 1];
	cudaMemcpy(sumGpu, sum, sizeof(int) * (numSurfels + 1), cudaMemcpyHostToDevice);
	surfaceSplatStep1Kernel<<<512, 1024>>>(width, zBufferProperty, zBuffer, numSurfels, surfels, sumGpu);
	surfaceSplatStep2Kernel<<<512, 1024>>>(width, zBufferProperty, zBuffer, numSurfels, surfels, sumGpu);
	surfaceSplatStep3Kernel<<<512, 1024>>>(width, zBufferProperty, zBuffer, filterLUT, numSurfels, surfels, sumGpu);

	cudaError_t cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess)
		fprintf(stderr, "Kernel launch failed: %s\n", cudaGetErrorString(cudaStatus));

	cudaFree(sumGpu);
	delete[] sum;
}

__host__ __device__ void lightSamplePhongR(const float r, const float g, const float b, const float nx, const float ny, const float nz, const float vx, const float vy, const float vz, float& resultR, float& resultG, float& resultB) {
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
	while (j > 0) {
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

void shade(int width, int height, Warper* warper, ZBufferItem* zBuffer, unsigned char* image, unsigned char backgroundR, unsigned char backgroundG, unsigned char backgroundB) {
	int zbf_rows, zbf_cols;
	float zbf_x_cur, zbf_y_cur;
	int zbf_i_cur, index;

	float vp_sx, vp_sy;				// variables for inverse viewport mapping
	float vp_tx, vp_ty;

	float w_, vec_len_;

	Frustum* frst;

	ZBufferItem* zbufitem;				// current zbuffer item

	// initialise local variables for more efficient access
	frst = &(warper->frustum);

	// set variables for inverse viewport mapping 
	vp_sx = 2 * frst->xP / width;
	vp_sy = 2 * frst->yP / height;
	vp_tx = frst->xC - frst->xP;
	vp_ty = frst->yC - frst->yP;

	// shade z-buffer
	// bbox[] specifies a bounding box
	zbf_i_cur = 0;
	index = 0;
	for (zbf_rows = 0; zbf_rows < height; zbf_rows++)
		for (zbf_cols = 0; zbf_cols < width; zbf_cols++) {
			zbf_x_cur = zbf_cols + 0.5f;
			zbf_y_cur = zbf_rows + 0.5f;
			zbufitem = &zBuffer[zbf_i_cur];
			// avoid division by zero!
			if (zbufitem->w != 0.0f) {
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
				image[index] = (unsigned char)resultR;
				image[index + 1] = (unsigned char)resultG;
				image[index + 2] = (unsigned char)resultB;
			}
			else {
				image[index] = (unsigned char)backgroundR;
				image[index + 1] = (unsigned char)backgroundG;
				image[index + 2] = (unsigned char)backgroundB;
			}

			zbf_i_cur++;
			index += 3;
		}
}

__global__ void shadeKernel(int width, int height, Warper* warper, ZBufferItem* zBuffer, unsigned char* image, unsigned char backgroundR, unsigned char backgroundG, unsigned char backgroundB) {
	__shared__ Frustum* frst;
	__shared__ float vp_sx, vp_sy;				// variables for inverse viewport mapping
	__shared__ float vp_tx, vp_ty;

	int i, index;

	// initialise local variables for more efficient access
	frst = &(warper->frustum);

	// set variables for inverse viewport mapping 
	vp_sx = 2 * frst->xP / width;
	vp_sy = 2 * frst->yP / height;
	vp_tx = frst->xC - frst->xP;
	vp_ty = frst->yC - frst->yP;

	i = blockIdx.x * blockDim.x + threadIdx.x;
	index = i * 3;
	while (i < width * height) {
		int zbf_rows = i / width;
		int zbf_cols = i % width;
		float zbf_x_cur = zbf_cols + 0.5f;
		float zbf_y_cur = zbf_rows + 0.5f;
		ZBufferItem* zbufitem = &zBuffer[i];
		// avoid division by zero!
		if (zbufitem->w != 0.0f) {
			// NOTE: we do per surfel shading, hence all there is left to do for the shader is
			// to normalize the depth, colors and normals, and write them to the display image

			float w_ = 1.f / zbufitem->w;

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
			float vec_len_ = 1.f / (float)sqrt(vx * vx + vy * vy + 1.f);
			vx *= vec_len_;
			vy *= vec_len_;
			vz *= vec_len_;

			float resultR, resultG, resultB;
			lightSamplePhongR(r, g, b, nx, ny, nz, vx, vy, vz, resultR, resultG, resultB);

			// clamp color intensities
			if (resultR > 255.0) resultR = 255.0;
			if (resultG > 255.0) resultG = 255.0;
			if (resultB > 255.0) resultB = 255.0;
			image[index] = (unsigned char)resultR;
			image[index + 1] = (unsigned char)resultG;
			image[index + 2] = (unsigned char)resultB;
		}
		else {
			image[index] = (unsigned char)backgroundR;
			image[index + 1] = (unsigned char)backgroundG;
			image[index + 2] = (unsigned char)backgroundB;
		}

		i += gridDim.x * blockDim.x;
		index = i * 3;
	}
}

void shadeGpu(int width, int height, Warper* warper, ZBufferItem* zBuffer, unsigned char* image, unsigned char backgroundR, unsigned char backgroundG, unsigned char backgroundB) {
	shadeKernel<<<512, 512>>>(width, height, warper, zBuffer, image, backgroundR, backgroundG, backgroundB);

	cudaError_t cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess)
		fprintf(stderr, "Kernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
}