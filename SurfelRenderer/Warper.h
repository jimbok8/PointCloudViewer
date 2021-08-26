#ifndef WARPER_H
#define WARPER_H

typedef struct _Transformation
{
	float rotation[9];			// model-view rotation
	float translation[3];		// model-view translation
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
} Frustum;

// warper
typedef struct _Warper
{
	Transformation transformation;
	Frustum frustum;
} Warper;

#endif
