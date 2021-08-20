#ifndef Z_BUFFER_H
#define Z_BUFFER_H

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

	float kA;
	float kD;
	float kS;
	unsigned char shininess;
	unsigned int specularColor;
} SrfZBufItem;

typedef struct _ZBuffer
{
	SrfZBufItem* buf;
	SrfZBufItem* cleardata;
	long bufsize;

	// surface splatting parameters
	float cutoffRadius;

	// blending thresholds used for surface splatting
	float constThreshold;
	float distThreshold;
	float angleTrheshold;

	// reconstruction filter table used for surface splatting
	int LUTsize;
	float* filterLUT;
} ZBuffer;

#endif