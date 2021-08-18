#ifndef WARPER_H
#define WARPER_H

#include "Matrix.h"
#include "ZBuffer.h"
#include "CSurfel.h"

typedef struct _Transformation
{
	float rotation[9];			// model-view rotation
	float translation[3];		// model-view translation
	float invRotation[9];		// inverse model-view rotation (used for block clipping)
	float invTranslation[3];	// inverse model-view rotation (used for block clipping)
	float normalsRotation[9];	// rotation matrix to transform normals
	float scaling;				// uniform model-view scaling
} Transformation;

// view frustum
typedef struct _Frustum
{
	float xP, yP;				// x,y-extent of the view frustum at z=1
	float xC, yC;				// x,y-center point of the view frustum at z=1
	float nearplane, farplane;	// camera space z-values of near and far clipping planes
	float v[24];				// vertices of the view frustum in camera space
	float M[9];					// view frustum normalization transformation (used for clipping)
} Frustum;

// viewport
typedef struct _Viewport
{
	// note: the viewport has the origin at its lower left corner
	int xS, yS;					// size of the viewport in screen pixels
} Viewport;

// warper
typedef struct _Warper
{
	Transformation transformation;
	Frustum frustum;
	Viewport viewport;
	ZBuffer* zbf;

	int options;
} Warper;

// Static variables

// variables set in wrpSetTrafoVars(), valid for all blocks with the same transformation matrix
static int wrp_viewport_xsize;
static int wrp_viewport_ysize;
static float wrp_frustum_nearplane;
static float wrp_frustum_farplane;
static float vp_sx, vp_sy;				// scaling for viewport mapping
float vp_tx[3];							// translation for viewport mapping, x direction
float vp_ty[3];							// translation for viewport mapping, y direction

static float A[3][9];					// the transformation matrix (in 3 variants, one for each base plane)
static float v[3];						// the translation vector
static float normalsA[9];				// the transposed inverse of A for transforming normals from camera to object space
static float pnormA[9];					// concatenated perpective normalization & modelview transformations
static float pnormV[3];
static float stoo_scaling;				// screen to camera scaling due to viewport mapping and uniform scaling in the modelview transform
static float otoc_scaling;				// object to camera space scaling, due to scaling in transformation matrix

static const Vector3D* wrp_nrm_table;
static ZBuffer* wrp_zbf;
//static ZBfSplatSurfel wrp_zbf_splatsurfel;
static int wrp_options;

//-------------------------------------------------------------------------
// Set the viewing frustum
//-------------------------------------------------------------------------
static void wrpSetFrustum(Warper* wrp, float fofv, float aspect, float nearplane, float farplane)
{
	float t, b, l, r;
	float s;
	float* fv;

	// note: xP and yP are the half x,y-extent of the view frustum at z=1!
	wrp->frustum.yP = (float)tan(fofv / 360.0 * M_PI);
	wrp->frustum.xP = wrp->frustum.yP * aspect;
	wrp->frustum.nearplane = nearplane;
	wrp->frustum.farplane = farplane;
	wrp->frustum.xC = 0.f;
	wrp->frustum.yC = 0.f;

	// top, bottom, left right coordinates for view frustum
	t = wrp->frustum.yP * nearplane;
	b = -t;
	r = wrp->frustum.xP * nearplane;
	l = -r;

	// scale and shear factors for the view frustum normalization transform
	wrp->frustum.M[0] = 2 * nearplane / (r - l);	// x scale
	wrp->frustum.M[4] = 2 * nearplane / (t - b);	// y scale
	wrp->frustum.M[8] = 1.0f;					// z scale
	wrp->frustum.M[2] = -(r + l) / (r - l);			// x shear
	wrp->frustum.M[5] = -(t + b) / (t - b);			// y shear
	wrp->frustum.M[1] = wrp->frustum.M[3] = wrp->frustum.M[6] = wrp->frustum.M[7] = 0.0f;

	// camera coordinates of view frustum
	fv = wrp->frustum.v;
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
// Set viewport
//-------------------------------------------------------------------------
static void wrpSetViewport(Warper* wrp, int xsize, int ysize)
{
	wrp->viewport.xS = xsize;
	wrp->viewport.yS = ysize;
}

//-------------------------------------------------------------------------
// Init transformation variables
// note: the matrix trafo[] is expected to be row major ordered, i.e.
// row 1 has indices 0..3, row 2 has indices 4..7, etc.
//-------------------------------------------------------------------------
static void wrpSetTrafo(Warper* wrp, const float trafo[16])
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
	wrp->transformation.scaling = s;

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
	MtrUtil::MtrTranspose3x3f(tm2, wrp->transformation.normalsRotation);

	// set the member variables of 'wrp'
	MtrUtil::MtrCopy3x3f(r, wrp->transformation.rotation);
	for (i = 0; i < 3; i++) wrp->transformation.translation[i] = t[i];
	MtrUtil::MtrCopy3x3f(ir, wrp->transformation.invRotation);
	for (i = 0; i < 3; i++) wrp->transformation.invTranslation[i] = it[i];

#ifdef DEBUG_CDE_TRAFO
	printf("Warper::Rotation matrix:\n");
	for (i = 0; i < 9; i++)
	{
		printf("%f ", r->transformation.rotation[i]);
	}
	printf("\n");
	printf("Warper::Translation vector:\n");
	for (i = 0; i < 3; i++)
	{
		printf("%f ", r->transformation.translation[i]);
	}
#endif
}

//-------------------------------------------------------------------------
// Set up static variables valid for all blocks with the same 
// transformation matrix
//-------------------------------------------------------------------------
static void wrpSetTrafoVars(Warper* wrp)
{
	int i;
	int r, c;

	// static variables used for warping, which are independent of current block
	wrp_options = wrp->options;
	wrp_viewport_xsize = wrp->viewport.xS;
	wrp_viewport_ysize = wrp->viewport.yS;
	wrp_frustum_nearplane = wrp->frustum.nearplane;
	wrp_frustum_farplane = wrp->frustum.farplane;
	stoo_scaling = wrp->frustum.xP * 2 / (wrp_viewport_xsize * wrp->transformation.scaling);
	otoc_scaling = wrp->transformation.scaling;
	wrp_zbf = wrp->zbf;
	//wrp_zbf_splatsurfel = wrp->zbf->splatFunction;

	// set transformation variables
	MtrUtil::MtrCopy3x3f(wrp->transformation.rotation, A[0]);
	for (i = 0; i < 3; i++) v[i] = wrp->transformation.translation[i];
	MtrUtil::MtrCopy3x3f(wrp->transformation.normalsRotation, normalsA);

	// rotation matrices for yz- and zx-baseplanes
	for (r = 0; r < 3; r++)
	{
		for (c = 0; c < 3; c++)
		{
			A[1][r * 3 + c] = A[0][r * 3 + (c + 1) % 3];
			A[2][r * 3 + c] = A[0][r * 3 + (c + 2) % 3];
		}
	}

	// concatenate perspective normalization and modelview trafo
	MtrUtil::MtrMult3x3fr(wrp->frustum.M, A[0], pnormA);
	pnormV[0] = wrp->frustum.M[0] * v[0] + wrp->frustum.M[2] * v[2];
	pnormV[1] = wrp->frustum.M[4] * v[1] + wrp->frustum.M[5] * v[2];
	pnormV[2] = wrp->frustum.M[8] * v[2];

	// set viewport mapping variables
	vp_sx = wrp_viewport_xsize / (2 * wrp->frustum.xP);
	vp_sy = wrp_viewport_ysize / (2 * wrp->frustum.yP);

	// set variables for warping from all 3 baseplanes
	for (i = 0; i < 3; i++)
	{
		vp_tx[i] = wrp->frustum.xP - wrp->frustum.xC;
		vp_ty[i] = wrp->frustum.yP - wrp->frustum.yC;
	}
}

//-------------------------------------------------------------------------
// Project a sample to the z-buffer
//-------------------------------------------------------------------------
// caution: this function relies on global variables!
static void wrpProjectSample(CSurfel* surfel, int bbox[4]/*, Object* object, bool useEllipticalSurfels*/)
{
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
	float camtan1[3];			// camera space tangent axes
	float camtan2[3];
	int splatBBox[4];			// bounding box of rasterized splat

	float clipAxes[4 * 10];      // maximum 10 clip planes
	int i;

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
		if ((xImg >= -xPad) && (xImg < wrp_viewport_xsize + xPad) &&
			(yImg >= -yPad) && (yImg < wrp_viewport_ysize + yPad))
		{
			// transform normal to camera coordinates
			nrm = surfel->getNormal();

			n[0] = normalsA[0] * nrm[0] + normalsA[1] * nrm[1] + normalsA[2] * nrm[2];
			n[1] = normalsA[3] * nrm[0] + normalsA[4] * nrm[1] + normalsA[5] * nrm[2];
			n[2] = normalsA[6] * nrm[0] + normalsA[7] * nrm[1] + normalsA[8] * nrm[2];

			/*if (useEllipticalSurfels) {

				tan1 = surfel->getAxisOne();

				camtan1[0] = normalsA[0] * tan1[0] + normalsA[1] * tan1[1] + normalsA[2] * tan1[2];
				camtan1[1] = normalsA[3] * tan1[0] + normalsA[4] * tan1[1] + normalsA[5] * tan1[2];
				camtan1[2] = normalsA[6] * tan1[0] + normalsA[7] * tan1[1] + normalsA[8] * tan1[2];

				tan2 = surfel->getAxisTwo();

				camtan2[0] = normalsA[0] * tan2[0] + normalsA[1] * tan2[1] + normalsA[2] * tan2[2];
				camtan2[1] = normalsA[3] * tan2[0] + normalsA[4] * tan2[1] + normalsA[5] * tan2[2];
				camtan2[2] = normalsA[6] * tan2[0] + normalsA[7] * tan2[1] + normalsA[8] * tan2[2];

				int numClipAxes = surfel->getNofAssociatedSurfels();
				SurfelInterface* assSurfel;

				Vector3D normal1, normal2, normal3;
				Vector3D pos1, pos2;  // two points on intersection line
				Vector3D pos1cam, pos2cam;
				float pos1Imgx, pos1Imgy, pos2Imgx, pos2Imgy;
				float clipNormx, clipNormy;

				// compute clip axes
				for (i = 0; i < numClipAxes; i++) {

					assSurfel = surfel->getAssociatedSurfel(i);

					// find point on intersection line as intersection of three planes

					// plane1 = tangent plane of surfel
					// plane2 = tangent plane of assSurfel
					// plane3 = plane through (surfel->position + assSurfel->position) / 2 that is orthogonal
					//          to both plane1 and plane2

					normal1 = surfel->getNormal();
					normal2 = assSurfel->getNormal();

					float d1 = Vector3D::dotProduct(surfel->getPosition(), normal1);
					float d2 = Vector3D::dotProduct(assSurfel->getPosition(), normal2);

					normal3 = Vector3D::crossProduct(normal1, normal2);
					float d3 = Vector3D::dotProduct(0.5 * (surfel->getPosition() + assSurfel->getPosition()), normal3);

					float temp1, temp2, temp3, temp4, temp5, temp6, temp7;


					temp1 = normal1[0] * normal3[1];
					temp2 = normal1[0] * normal2[1];
					temp3 = normal3[0] * normal1[1];
					temp4 = normal2[1] * normal3[0];
					temp5 = normal3[1] * normal2[0];
					temp6 = normal2[0] * normal1[1];
					temp7 = 1 / (-normal2[2] * temp1 + normal3[2] * temp2 + normal2[2] * temp3 - normal1[2] * temp4 + normal1[2] * temp5 - normal3[2] * temp6);
					pos1[0] = temp7 * (-normal1[1] * normal3[2] * d2 + normal1[1] * d3 * normal2[2] - normal1[2] * normal2[1] * d3 + normal1[2] * normal3[1] * d2 - d1 * normal3[1] * normal2[2] + d1 * normal2[1] * normal3[2]);
					pos1[1] = -temp7 * (-normal1[0] * normal3[2] * d2 + normal1[0] * d3 * normal2[2] + normal3[0] * normal1[2] * d2 + normal3[2] * normal2[0] * d1 - d3 * normal2[0] * normal1[2] - normal3[0] * d1 * normal2[2]);
					pos1[2] = temp7 * (-d1 * temp4 + d3 * temp2 - d2 * temp1 + d1 * temp5 + d2 * temp3 - d3 * temp6);

					pos2 = pos1 + normal3;


					pos1cam[0] = A[0][0] * pos1[0] + A[0][1] * pos1[1] + A[0][2] * pos1[2] + v[0];
					pos1cam[1] = A[0][3] * pos1[0] + A[0][4] * pos1[1] + A[0][5] * pos1[2] + v[1];
					pos1cam[2] = A[0][6] * pos1[0] + A[0][7] * pos1[1] + A[0][8] * pos1[2] + v[2];

					// perspective divide and viewport transformation
					pos1cam[2] = 1.0f / pos1cam[2];
					pos1Imgx = (pos1cam[0] * pos1cam[2] + vp_tx[0]) * vp_sx;
					pos1Imgy = (pos1cam[1] * pos1cam[2] + vp_ty[0]) * vp_sy;

					pos2cam[0] = A[0][0] * pos2[0] + A[0][1] * pos2[1] + A[0][2] * pos2[2] + v[0];
					pos2cam[1] = A[0][3] * pos2[0] + A[0][4] * pos2[1] + A[0][5] * pos2[2] + v[1];
					pos2cam[2] = A[0][6] * pos2[0] + A[0][7] * pos2[1] + A[0][8] * pos2[2] + v[2];

					pos2cam[2] = 1 / pos2cam[2];
					pos2Imgx = (pos2cam[0] * pos2cam[2] + vp_tx[0]) * vp_sx;
					pos2Imgy = (pos2cam[1] * pos2cam[2] + vp_ty[0]) * vp_sy;

					// flip clip plane (distinguish union with difference & intersection)
					if (surfel->isFlagOn(SurfelInterface::CLIP_ORIENTATION)) {
						clipNormx = pos2Imgy - pos1Imgy;
						clipNormy = pos1Imgx - pos2Imgx;
					}
					else {
						clipNormx = pos1Imgy - pos2Imgy;
						clipNormy = pos2Imgx - pos1Imgx;
					}

					clipAxes[i * 4] = pos1Imgx;
					clipAxes[i * 4 + 1] = pos1Imgy;
					clipAxes[i * 4 + 2] = clipNormx;
					clipAxes[i * 4 + 3] = clipNormy;
				}


				ZBfSurfaceSplatEllipse(wrp_zbf, xImg, yImg, x_c, y_c, z_c, n, camtan1, camtan2, surfel, 0, stoo_scaling, otoc_scaling,
					vp_sx, vp_sy, vp_tx[0], vp_ty[0], splatBBox, object, clipAxes, numClipAxes);
			}
			else {*/


				// caution: this function (or macro) relies on global variables!
				// note: 'warped level' is set to 0
				zbfSurfaceSplat(wrp_zbf, xImg, yImg, z_c, n, surfel, 0, stoo_scaling, otoc_scaling,
					vp_sx, vp_sy, vp_tx[0], vp_ty[0], splatBBox/*, object*/);

			//}

			// enlarge bounding box (containing the modified frame buffer area) if necessary
			if (splatBBox[0] < bbox[0]) bbox[0] = splatBBox[0];
			if (splatBBox[1] < bbox[1]) bbox[1] = splatBBox[1];
			if (splatBBox[2] > bbox[2]) bbox[2] = splatBBox[2];
			if (splatBBox[3] > bbox[3]) bbox[3] = splatBBox[3];
		}
	}
}

#endif
