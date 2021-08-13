// Title:   SurfelGPRenderer.h
// Created: Thu Sep 25 14:51:32 2003
// Authors: Richard Keiser, Oliver Knoll, Mark Pauly, Matthias Zwicker
//
// Copyright (c) 2001, 2002, 2003 Computer Graphics Lab, ETH Zurich
//
// This file is part of the Pointshop3D system.
// See http://www.pointshop3d.com/ for more information.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public
// License along with this program; if not, write to the Free
// Software Foundation, Inc., 59 Temple Place - Suite 330, Boston,
// MA 02111-1307, USA.
//
// Contact info@pointshop3d.com if any conditions of this
// licensing are not clear to you.
//

#ifndef __SURFELGPRENDERER_H_
#define __SURFELGPRENDERER_H_

#include "../Scene.h"
#include "../MyDataTypes.h"
#include "../../Surfel/RendererInterface.h"
//#include "../../../Core/Interfaces/src/RendererConfigurationInterface.h"
#include "../../Surfel/FrameBufferInterface.h"
#include "SurfelPipeline_SurfSplat/Srf.h"
//#include "SurfelGPRendererWidget.h"
//#include "SurfelGPRendererConfiguration.h"
#include "SurfelGPFrameBuffer.h"

#define NIL		0
#define LEFT	1
#define	RIGHT	2
#define BOTTOM	4
#define TOP		8

/**
 * This class implements the <code>RendererInterface</code> and acts as a wrapper class for the
 * <em>SurfelGP Renderer</em> as written by Matthias Zwicker.
 *
 * @author Oliver Knoll
 * @version 1.2
 */
class SurfelGPRenderer : public RendererInterface 
{
public:

	SurfelGPRenderer();
	virtual ~SurfelGPRenderer();	

	// ***************************
	// interface RendererInterface
	// ***************************

	bool isInteractive() const;
	bool providesPointPicking() const;
	float getDepth (unsigned int x, unsigned int y) const ;
	Vector3D getCameraSpaceNormal (unsigned int x, unsigned int y) const;

	void initialize (const bool isVisible/*, QWidget *parent*/);

	void setVisible (const bool enable);
	bool isVisible() const;

	//QWidget *getRendererWidget() const;
	CImage* getRenderedImage();
	//RendererConfigurationInterface *getRendererConfiguration();

	void setSceneView (const MyDataTypes::TransformationMatrix16f newSceneViewMatrix);
	void getSceneView (MyDataTypes::TransformationMatrix16f sceneViewMatrix) const;
	void setViewFrustum (const MyDataTypes::ViewFrustum newViewFrustum);
	MyDataTypes::ViewFrustum getViewFrustum() const;
	void setViewPortSize (const CSize newViewPortSize);
	CSize getViewPortSize() const;
	void setShadingEnabled (const bool enable);
	bool isShadingEnabled() const;
	void setLightDirection (const Vector3D newLightDirection);
	Vector3D getLightDirection() const;
	void setBackgroundColor(COLORREF newBackgroundColor);
	COLORREF getBackgroundColor() const;

	void setTwoSidedNormalsEnabled (const bool enable);
	bool isTwoSidedNormalsEnabled() const;
	void setShadowsEnabled (const bool enable);
	bool isShadowsEnabled() const;
	void toggleSelectionVisualization();
	void setSelectionVisualizationEnabled(const bool enable);
	bool isSelectionVisualizationEnabled() const;
	void setCutoffRadius (const float radius);
	float getCutoffRadius() const;
	void setBlendingThresholds (const float constThreshold, const float distThreshold, const float angleThreshold) ;
	void getBlendingThresholds (float* constThreshold, float* distThreshold, float* angleThreshold);
	void renderFrame (const int attributes = (FrameBufferInterface::PERPIXEL_C_Z_N_W | FrameBufferInterface::PERPIXEL_OBJECTPOINTER), const bool allScene = true);

	void addToFrame (const std::vector<SurfelInterface*>* surfels, int nSurfels, const int attributes = FrameBufferInterface::PERPIXEL_C_Z_N_W, const bool updateRendererWidget = true);
	void subtractFromFrame (const std::vector<SurfelInterface*>* surfels, int nSurfels, const int attributes = FrameBufferInterface::PERPIXEL_C_Z_N_W, const bool updateRendererWidget = true);

	FrameBufferInterface *getFrameBuffer() const;

	/**
	 * draws a line with z-buffer test
	 * The Bresenham algorithm is used for drawing. If a line point is visible, its color is set to the
	 * frame buffer but without changing the depth. So only the color changes, not the depth.
	 *
	 * @param start
	 *		start point
	 * @param end
	 *		end point
	 * @param color
	 *		line color
	 */
	void drawLine (const Vector3D start, const Vector3D end, COLORREF color);

	/**
	 * Cohen-Sutherland clipping algorithm
	 * 
	 * @param start
	 *			start point
	 * @param end
	 *			end point
	 * @param Ax
	 *			clipped x coordinate of start
	 * @param Ay
	 *			clipped y coordinate of start
	 * @param Bx
	 *			clipped x coordinate of end
	 * @param By
	 *			clipped y coordinate of end
	 * @return true if the line is inside the viewport, false otherwise
	 */
	bool clipLine(Vector2D start, Vector2D end, int *Ax, int *Ay, int *Bx, int *By);

private:   

	int                                  iWidth, iHeight;
	int                                  superSampling;
	SrfContext                           *surfelContext;

	SrfHandle                            lightSourceHandle;
	Vector3D                             lightDirection;

	SurfelGPFrameBuffer                  *frameBuffer;

	float                                constThreshold,
	                                     distThreshold,
										 angleThreshold;

	bool								 shadingEnabled,
		                                 selectionVisualizationEnabled;

	//SurfelGPRendererWidget               *rendererWidget;
	//QWidget                              *parentWidget;          // the parent widget of the rendererWidget
	//SurfelGPRendererConfiguration        rendererConfiguration;

	MyDataTypes::TransformationMatrix16f sceneViewMatrix;
	MyDataTypes::ViewFrustum             viewFrustum;

	COLORREF								 backgroundColor;

	// instantiates the surfel context
	void instantiateSurfelContext();

	void renderShadowBuffer();

	// frees any used memory
	void cleanUp();

	// normalizes the 'lightDirection' and updates the light direction in the SurfelGPRenderer 
	// according to 'lightDirection' - does not re-render the image
	// PRE: the 'surfelContext' and the 'lightSourceHandle' must already be initialized
	// 
	// updateScene: true, if scene should be re-rendered
	void updateLightDirection (const bool updateScene);

	
	// help method for the lineClip method
	inline short CompOutCode(float x, float y, float xmin, float xmax, float ymin, float ymax);

private ://slots:

	void handleObjectModified();
	void handleActiveObjectChanged();
	void handleObjectAdded();
	void handleObjectRemoved();
	void handleObjectMoved();

	// simply re-emits the signal 'widgetRepainted'
	void handleWidgetRepainted();

	void handleCutOffRadiusChanged (const float newCutOffRadius);

	void handleTwoSidedNormalsChanged (const bool enable);

	void handleConfigurationApplied();

};

// ****************
// access functions
// ****************

// extern "C" is needed so those functions can be accessed
// with GetProcAddress() (WIN32) and dlsym() (UNIX) respective
extern "C" {
	RendererInterface *createRenderer();
	const char *getRendererType();
}

#endif  // __SURFELGPRENDERER_H_

// Some Emacs-Hints -- please don't remove:
//
//  Local Variables:
//  mode:C++
//  tab-width:4
//  End:
