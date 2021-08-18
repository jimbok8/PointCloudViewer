#ifndef SHADER_H
#define SHADER_H

#define PACK_DIM_RGB(_c,_r,_g,_b) \
	_c = ((_r << 16) & 0x00ff0000) | \
		((_g << 8) & 0x0000ff00) | \
		(_b & 0x000000ff)

#define NOF_DBGCOLORS 8

#define ZBFGETITEM(_zbufitem) \
{ \
	/* zbf_i_cur, zbf_cols, zbf_rows, zbf_x_cur, zbf_y_cur contain */ \
	/* the current position in the z-buffer */ \
	_zbufitem = &(zbf_buf[zbf_i_cur]); \
}

#define ZBFNEXTCOL() \
{ \
	zbf_x_cur += 1.0f; \
	zbf_i_cur++; \
	zbf_cols++; \
}

#define ZBFSETPOS(_x,_y) \
{ \
	zbf_x_cur = _x+0.5f; \
	zbf_y_cur = _y+0.5f; \
	zbf_rows = _y; \
	zbf_cols = _x; \
	zbf_i_cur = _y*zbf_xsize + _x; \
}

#define ZBFCOLS zbf_cols
#define ZBFROWS zbf_rows

#include "Warper.h"

// Variable and type declarations
typedef enum _ShdProperties
{
	SHD_BIDINORMALS = 1,
	SHD_DBGCOLORS = 2,
	SHD_ENVMAPPING = 4,
	SHD_LIGHTING = 8,
	SHD_PRESHADESPLAT = 16,
	SHD_PERPIXELSHADING = 32
} ShdProperties;

typedef enum _ShdShadingModel
{
	SHD_PHONG_H = 1,
	SHD_PHONG_R = 2,
	SHD_TORRANCESPARROW = 3,
	SHD_USERFNCT = 4
} ShdShadingModel;

typedef enum _ShdTextureFiltering
{
	SHD_TEXTUREFILTERING_NOINTERP = 1,
	SHD_TEXTUREFILTERING_LINEAR = 2
} ShdTextureFiltering;

typedef enum _ShdDiffColor
{
	SHD_DIFFCOL_DEFAULT = 0,
	SHD_DIFFCOL_MIPMAPLEVEL = 1,
	SHD_DIFFCOL_LODLEVEL = 2
} ShdDiffColor;

//typedef void (*ShdLightSampleFnct)();
//typedef void (*ShdTextureFilteringFnct)();

//extern ShdLightSampleFnct _shd_lightsample;

typedef struct _Shader
{
	Warper* wrp;
	//FrameBufferInterface* frameBuffer;

	//LSrList* lsl;

	//ShdLightSampleFnct shdLightSample;

	//ShdLightSampleFnct shdUserShadingFunction;
	void* shdUserShadingData;

	int options;
	ShdDiffColor diffColor;
	int dbgcolors[NOF_DBGCOLORS];
} Shader;

// Static variables for efficiency reasons
// dereferenced member variables
static const Vector3D* shd_normalmap_table;
static int shd_diffcolor;
static int* shd_dbgcolors;
//static LSrList* shd_lsl;
//static ShdTextureFilteringFnct shd_gettexture;

// additional variables used over function scope
static float _shd_objScaling;
static unsigned char _shd_nroflevels;
static int _shd_nrofuserbytes;

// global variables (potentially accessed by external shading functions)
float _shd_nx_c, _shd_ny_c, _shd_nz_c;
float _shd_x_c, _shd_y_c, _shd_z_c;
float _shd_vx, _shd_vy, _shd_vz;
float _shd_ndotv;
float _shd_kA, _shd_kD, _shd_kS;
unsigned char _shd_shininess;
MyDataTypes::RGBTriple _shd_specularColor;
MyDataTypes::RGBTriple _shd_Id;
MyDataTypes::RGBTriple _shd_Ir;
void* _shd_userdata;
int _shd_options;
//ShdLightSampleFnct _shd_lightsample;

static float sqrt_3, sqrt_2;

//-------------------------------------------------------------------------
// Shade the samples in the z-buffer of a warper
//-------------------------------------------------------------------------
static void shdShadeZBuffer(Shader* shd, int magfactor, int bbox[4], COLORREF backgroundColor)
{
	Frustum* frst;
	Viewport* vprt;

	int xsize, ysize;				// x and y screen coordinates size			

	SrfZBufItem* zbufitem;				// current zbuffer item

	float vp_sx, vp_sy;				// variables for inverse viewport mapping
	float vp_tx, vp_ty;

	float w_, vec_len_;
	int c, specularColor;

	int start_dim_x, start_dim_y;	// writing the color values to the display image
	int end_dim_x, end_dim_y;		// (supports progressive rendering)
	int dim_y, dim_x;
	unsigned int adjustedBufferHeight;      // buffer height - 1

	int attributes;

#ifdef STATISTIC_LOD_SHADER
	float dbg_z;
#endif

#ifdef STATISTIC_TME_SHADER
	start = clock();
#endif

	zbfPrepareForReading(shd->wrp->zbf);
	//DImPrepareForAccess(shd->dim);

	// initialise local variables for more efficient access
	xsize = shd->wrp->viewport.xS;
	ysize = shd->wrp->viewport.yS;
	frst = &(shd->wrp->frustum);
	vprt = &(shd->wrp->viewport);

	// initialise static variables for efficient access over function scope
	shd_diffcolor = shd->diffColor;
	shd_dbgcolors = shd->dbgcolors;

	// set variables for inverse viewport mapping 
	vp_sx = 2 * frst->xP / vprt->xS;
	vp_sy = 2 * frst->yP / vprt->yS;
	vp_tx = frst->xC - frst->xP;
	vp_ty = frst->yC - frst->yP;

	// shade z-buffer
	// bbox[] specifies a bounding box
	ZBFSETPOS(bbox[0], bbox[1]);
	//attributes = shd->frameBuffer->getAttributes();
	adjustedBufferHeight = shd->frameBuffer->getSize().cy - 1;
	while (ZBFROWS <= bbox[3])
	{
		while (ZBFCOLS <= bbox[2])

		{
			ZBFGETITEM(zbufitem);

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

				/*if ((_shd_options & SHD_PERPIXELSHADING) && (_shd_options & SHD_LIGHTING)) {

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
					_shd_ndotv = _shd_nx_c * _shd_vx + _shd_ny_c * _shd_vy + _shd_nz_c * _shd_vz;

					_shd_kA = zbufitem->kA;
					_shd_kD = zbufitem->kD;
					_shd_kS = zbufitem->kS;
					_shd_shininess = zbufitem->shininess;
					specularColor = zbufitem->specularColor;
					UNPACK_FLOAT_RGB(_shd_specularColor.r, _shd_specularColor.g, _shd_specularColor.b, specularColor);

					_shd_Ir.r = 0.f;
					_shd_Ir.g = 0.f;
					_shd_Ir.b = 0.f;
					_shd_lightsample();
				}*/

				// clamp color intensities
				if (_shd_Ir.r > 255.0) _shd_Ir.r = 255.0;
				if (_shd_Ir.g > 255.0) _shd_Ir.g = 255.0;
				if (_shd_Ir.b > 255.0) _shd_Ir.b = 255.0;

				// pack color to format for packed RGB arithmetic
				PACK_DIM_RGB(c, (int)_shd_Ir.r, (int)_shd_Ir.g, (int)_shd_Ir.b);

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
						shd->frameBuffer->setColor(dim_x, adjustedBufferHeight - dim_y, c);
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
						shd->frameBuffer->setColor(dim_x, adjustedBufferHeight - dim_y, backgroundColor);
					}
				}
			}

			ZBFNEXTCOL();
		}
		bbox[1]++;
		ZBFSETPOS(bbox[0], bbox[1]);
	}

#ifdef STATISTIC_TME_SHADER
	end = clock();
	sec = ((float)end - (float)start) / CLOCKS_PER_SEC;
	printf("Shader::Timing: shaded z-buffer in %f sec.\n", sec);
#endif
}

#endif