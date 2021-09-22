#ifndef TYPE_HELPER_H
#define TYPE_HELPER_H

#include <Eigen/Dense>

typedef float TransformationMatrix16f[16];

typedef struct _CameraPosition {
	TransformationMatrix16f scaleTranslationMatrix, rotationMatrix;
} CameraPosition;

typedef struct _Surfel {
	Eigen::Vector4f position, normal, color, transformedNormal;
	int xMin, xMax, yMin, yMax;
	float radius, zMin, zMax, x0, y0, a, b, c, det_;
} Surfel;

typedef struct _Warper {
	float rotation[9];			// model-view rotation
	float translation[3];		// model-view translation
	float normalsRotation[9];	// rotation matrix to transform normals
	float scaling;				// uniform model-view scaling
	float xP, yP;				// x,y-extent of the view frustum at z=1
	float xC, yC;				// x,y-center point of the view frustum at z=1
	float nearplane, farplane;	// camera space z-values of near and far clipping planes
	float v[24];				// vertices of the view frustum in camera space
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
	int LUTsize;
	float cutoffRadius;		// surface splatting parameters
	float angleTrheshold;	// blending thresholds used for surface splatting
} ZBufferProperty;

#endif