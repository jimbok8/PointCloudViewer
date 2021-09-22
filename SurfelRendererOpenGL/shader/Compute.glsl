#version 430 core

struct ZBufferProperty {
	int bufsize;
	int LUTsize;
	float cutoffRadius;
	float angleTrheshold;
};

struct Warper {
    float rotation[9];
    float translation[3];
    float normalsRotation[9];
    float scaling;
    float xP, yP;
    float xC, yC;
    float nearplane, farplane;
    float v[24];
};

struct Surfel {
    vec4 position, normal, color, transformedNormal;
    int xMin, xMax, yMin, yMax;
    float radius, zMin, zMax, x0, y0, a, b, c, det_;
};

layout(local_size_x = 1024) in;

uniform int width;
uniform int height;

layout(std430, binding = 0) buffer SSBO0 {
	ZBufferProperty zBufferProperty;
};

layout(std430, binding = 1) buffer SSBO1 {
    Warper warper;
};

layout(std430, binding = 2) buffer SSBO2 {
    Surfel surfels[];
};

void main() {
    int index = int(gl_GlobalInvocationID.x);
    if (index >= surfels.length())
        return;

	float wrp_frustum_nearplane, wrp_frustum_farplane;
	float zbf_cutoffRadius_2, zbf_angleThreshold;
	float vp_sx, vp_sy;				// scaling for viewport mapping
	float vp_tx, vp_ty;				// translation for viewport mapping
	float A0[9], A1[9], A2[9];		// the transformation matrix (in 3 variants, one for each base plane)
	float v[3];						// the translation vector
	float normalsA[9];				// the transposed inverse of A for transforming normals from camera to object space
	float stoo_scaling;				// screen to camera scaling due to viewport mapping and uniform scaling in the modelview transform
	float otoc_scaling;				// object to camera space scaling, due to scaling in transformation matrix

	int i, j;

	float x_c, y_c;             // camera-space x and y values
	float z_c, r_z_c;			// camera-space z-value (and its reciprocal) of sample being warped

	float xImg, yImg;			// x- and y-screen-coordinates of warped sample
	float xPad, yPad;			// pads in x and y direction for correct clipping

	vec4 pos;				// object space sample position
	vec4 nrm;				// object space normal

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
	wrp_frustum_nearplane = warper.nearplane;
	wrp_frustum_farplane = warper.farplane;
	zbf_cutoffRadius_2 = zBufferProperty.cutoffRadius * zBufferProperty.cutoffRadius;
	zbf_angleThreshold = zBufferProperty.angleTrheshold;
	stoo_scaling = warper.xP * 2 / (width * warper.scaling);
	otoc_scaling = warper.scaling;

	// set transformation variables
	for (i = 0; i < 9; i++)
		A0[i] = warper.rotation[i];
	for (i = 0; i < 3; i++)
		v[i] = warper.translation[i];
	for (i = 0; i < 9; i++)
		normalsA[i] = warper.normalsRotation[i];

	// rotation matrices for yz- and zx-baseplanes
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++) {
			A1[i * 3 + j] = A0[i * 3 + (j + 1) % 3];
			A2[i * 3 + j] = A0[i * 3 + (j + 2) % 3];
		}

	// set viewport mapping variables
	vp_sx = width / (2 * warper.xP);
	vp_sy = height / (2 * warper.yP);

	// set variables for warping from all 3 baseplanes
	vp_tx = warper.xP - warper.xC;
	vp_ty = warper.yP - warper.yC;

	// get sample position
	pos = surfels[index].position;

	// apply transformation matrix
	z_c = A0[6] * pos.x + A0[7] * pos.y + A0[8] * pos.z + v[2];
	// apply near and far clipping planes
	flag = false;
	if (z_c > wrp_frustum_nearplane && z_c < wrp_frustum_farplane) {
		x_c = A0[0] * pos.x + A0[1] * pos.y + A0[2] * pos.z + v[0];
		y_c = A0[3] * pos.x + A0[4] * pos.y + A0[5] * pos.z + v[1];

		// perspective divide and viewport transformation
		r_z_c = 1 / z_c;
		xImg = (x_c * r_z_c + vp_tx) * vp_sx;
		yImg = (y_c * r_z_c + vp_ty) * vp_sy;

		// for correct clipping: project surfel radius to screen space
		radius = surfels[index].radius;
		radius *= r_z_c;
		xPad = radius * vp_sx;
		yPad = radius * vp_sy;

		// put it into the z-buffer
		if ((xImg >= -xPad) && (xImg < width + xPad) && (yImg >= -yPad) && (yImg < height + yPad)) {
			// transform normal to camera coordinates
			nrm = surfels[index].normal;

			n[0] = normalsA[0] * nrm.x + normalsA[1] * nrm.y + normalsA[2] * nrm.z;
			n[1] = normalsA[3] * nrm.x + normalsA[4] * nrm.y + normalsA[5] * nrm.z;
			n[2] = normalsA[6] * nrm.x + normalsA[7] * nrm.y + normalsA[8] * nrm.z;

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
			r_ = 1.f / sqrt(V_x * V_x + V_y * V_y + 1.f);
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
			r = sqrt(Ix_x * Ix_x + Ix_y * Ix_y + Ix_z * Ix_z);
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

			_radius = surfels[index].radius;
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

			tmp = sqrt(discr * zbf_cutoffRadius_2);
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
			det_ = 1.0f / sqrt(sx * sx * ty * ty + sx * sx + sy * sy + ty * ty + 1.f);

			// bounding box of the ellipse
			// see ellipseboundingbox.mws, an exact axis aligned bounding box is computed by finding the points on
			// the ellipse where the tangent of the ellipse is parallel to x- and y-axis respectively.
			// NOTE: the variable "d" in the maple sheet corresponds to "-zbf_cutoffRadius_2"!
			discr = sqrt((-b * b + 4 * c * a) * zbf_cutoffRadius_2 * a);
			ly = 2.f / (-b * b + 4 * c * a) * discr;

			discr = sqrt(c * (-b * b + 4 * c * a) * zbf_cutoffRadius_2);
			lx = 2.f / (-b * b + 4 * c * a) * discr;

			lx = (lx < 0) ? -lx : lx;
			ly = (ly < 0) ? -ly : ly;
			xMax = int(xImg + lx) + 1;
			xMin = int(xImg - lx);
			yMax = int(yImg + ly) + 1;
			yMin = int(yImg - ly);

			//if (index == 0) {
			//	x = (x / width * 2.0f - 1.0f) * z_c * warper.xP;
			//	y = (y / height * 2.0f - 1.0f) * z_c * warper.yP;
			//	float w = 1.0f;
			//	//z = -(z * (wrp_frustum_nearplane + wrp_frustum_farplane) + 2.0f * wrp_frustum_nearplane * wrp_frustum_farplane) / (wrp_frustum_farplane - wrp_frustum_nearplane);
			//	//float w = (z * (wrp_frustum_farplane - wrp_frustum_nearplane) + 2.0f * wrp_frustum_nearplane * wrp_frustum_farplane) / (wrp_frustum_nearplane + wrp_frustum_farplane);
			//	Eigen::Vector4f v(x, y, z, w);
			//	Eigen::Matrix4f modelMat, viewMat, projectionMat;
			//	modelMat = translate * rotate * TransformHelper::scale(factor);
			//	viewMat = TransformHelper::lookAt(Eigen::Vector3f(0.0f, 0.0f, 1000.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
			//	projectionMat = TransformHelper::perspective(acos(-1.0f) / 6.0f, (float)width / (float)height, 10.0f, 100000.0f);
			//	Eigen::Vector4f p = (viewMat * modelMat).inverse() * v;
			//	Eigen::Vector3f d(p(0) - pos.x, p(1) - pos.y, p(2) - pos.z);
			//	std::cout << d.dot(Eigen::Vector3f(nrm.x, nrm.y, nrm.z)) << std::endl;
			//	//std::cout << p(0) << ' ' << p(1) << ' ' << p(2) << ' ' << p(3) << std::endl;
			//}

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

			surfels[index].xMin = xMin;
			surfels[index].xMax = xMax;
			surfels[index].yMin = yMin;
			surfels[index].yMax = yMax;
			surfels[index].zMin = zMin;
			surfels[index].zMax = zMax;
			surfels[index].x0 = xImg;
			surfels[index].y0 = yImg;
			surfels[index].a = a;
			surfels[index].b = b;
			surfels[index].c = c;
			surfels[index].det_ = det_;
			surfels[index].transformedNormal[0] = n[0];
			surfels[index].transformedNormal[1] = n[1];
			surfels[index].transformedNormal[2] = n[2];

			flag = true;
		}
	}
	if (!flag) {
		surfels[index].xMin = 0;
		surfels[index].xMax = -1;
		surfels[index].yMin = 0;
		surfels[index].yMax = -1;
	}
}