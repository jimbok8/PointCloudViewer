#ifndef TYPE_HELPER_H
#define TYPE_HELPER_H

#include "Vector3D.h"

typedef struct _RGBTriple {
	float r, g, b;
} RGBTriple;

typedef float TransformationMatrix16f[16];

typedef struct _CameraPosition {
	TransformationMatrix16f scaleTranslationMatrix, rotationMatrix;
} CameraPosition;

typedef struct _Surfel {
    Vector3D position, normal;
    float radius, r, g, b;
} Surfel;

typedef struct _Transformation {
	float rotation[9];			// model-view rotation
	float translation[3];		// model-view translation
	float normalsRotation[9];	// rotation matrix to transform normals
	float scaling;				// uniform model-view scaling
} Transformation;

typedef struct _Frustum {
	float xP, yP;				// x,y-extent of the view frustum at z=1
	float xC, yC;				// x,y-center point of the view frustum at z=1
	float nearplane, farplane;	// camera space z-values of near and far clipping planes
	float v[24];				// vertices of the view frustum in camera space
} Frustum;

typedef struct _Warper {
	Transformation transformation;
	Frustum frustum;
} Warper;

typedef struct _ZBufferItem {
	float zMin;				// z-range used for blending
	float zMax;
	float w;				// accumulated weights
	float n[3];				// accumulated normal
	float c[3];				// accumulated color values
} ZBufferItem;

typedef struct _ZBufferProperty {
	int bufsize;

	// surface splatting parameters
	float cutoffRadius;

	// blending thresholds used for surface splatting
	float constThreshold;
	float distThreshold;
	float angleTrheshold;

	// reconstruction filter table used for surface splatting
	int LUTsize;
} ZBufferProperty;

#endif