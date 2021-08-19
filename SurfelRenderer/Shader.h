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

#include <iostream>

#include "Warper.h"

typedef struct _Shader
{
	Warper* wrp;
	unsigned char* frameBuffer;
} Shader;

// Global variables
extern int zbf_rows, zbf_cols;
extern float zbf_x_cur, zbf_y_cur;
extern int zbf_i_cur;
extern int zbf_xsize, zbf_ysize;
extern int zbf_material;
extern SrfZBufItem* zbf_buf;

static unsigned char* getPixelPtr(unsigned char* image, const int width, const int height, const int x, const int y) {
	return image + (y * width + x) * 3;
}

static void setColor(unsigned char* image, const int width, const int height, const int x, const int y, const COLORREF newPixelColor) {
	unsigned char* p = getPixelPtr(image, width, height, x, y);
	*p = GetRValue(newPixelColor);
	*(p + 1) = GetGValue(newPixelColor);
	*(p + 2) = GetBValue(newPixelColor);
}

//-------------------------------------------------------------------------
// Calculate phong shading for a sample
// note: this function uses the formulation with the reflection vector. for
// directional light sources, it produces exactly the same results as cube
// maps.
//-------------------------------------------------------------------------
static void shdLightSamplePhong_R()
{
	float Ir, Ig, Ib;
	float Ar, Ag, Ab;
	float Lx, Ly, Lz;
	float Rx, Ry, Rz;
	float t, r, ndotl, rdotv;
	int j;

	Ir = Ig = Ib = 1.0f;
	Ar = Ag = Ab = 0.5f;

	// ambient contribution
	t = _shd_kA;
	_shd_Ir.r += t * Ar * _shd_Id.r;
	_shd_Ir.g += t * Ag * _shd_Id.g;
	_shd_Ir.b += t * Ab * _shd_Id.b;

	Lx = Ly = 0.0f;
	Lz = -1.0f;

	// calculate the N*L dot product
	ndotl = _shd_nx_c * Lx + _shd_ny_c * Ly + _shd_nz_c * Lz;
	ndotl = (ndotl < 0 ? -ndotl : ndotl);

	// calculate normalized reflection vector
	Rx = 2 * _shd_nx_c * ndotl - Lx;
	Ry = 2 * _shd_ny_c * ndotl - Ly;
	Rz = 2 * _shd_nz_c * ndotl - Lz;

	// calculate R*V dot product
	rdotv = _shd_vx * Rx + _shd_vy * Ry + _shd_vz * Rz;
	rdotv = (rdotv < 0 ? -rdotv : rdotv);

	// calculate the phong shininess power
	r = rdotv;
	j = _shd_shininess;
	while (j > 0)
	{
		r *= rdotv;
		j--;
	}

	// increment intensities
	t = _shd_kD * ndotl;
	r *= _shd_kS;
	_shd_Ir.r += Ir * (t * _shd_Id.r + _shd_specularColor.r * r);
	_shd_Ir.g += Ig * (t * _shd_Id.g + _shd_specularColor.g * r);
	_shd_Ir.b += Ib * (t * _shd_Id.b + _shd_specularColor.b * r);
}

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

	// initialise local variables for more efficient access
	xsize = shd->wrp->viewport.xS;
	ysize = shd->wrp->viewport.yS;
	frst = &(shd->wrp->frustum);
	vprt = &(shd->wrp->viewport);

	// set variables for inverse viewport mapping 
	vp_sx = 2 * frst->xP / vprt->xS;
	vp_sy = 2 * frst->yP / vprt->yS;
	vp_tx = frst->xC - frst->xP;
	vp_ty = frst->yC - frst->yP;

	// shade z-buffer
	// bbox[] specifies a bounding box
	ZBFSETPOS(bbox[0], bbox[1]);
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
				shdLightSamplePhong_R();

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
						setColor(shd->frameBuffer, xsize, ysize, dim_x, dim_y, c);
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
						setColor(shd->frameBuffer, xsize, ysize, dim_x, dim_y, backgroundColor);
					}
				}
			}

			ZBFNEXTCOL();
		}
		bbox[1]++;
		ZBFSETPOS(bbox[0], bbox[1]);
	}
}

#endif