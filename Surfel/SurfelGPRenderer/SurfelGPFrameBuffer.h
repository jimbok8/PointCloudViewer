#ifndef __SURFELGPFRAMEBUFFER_H_
#define __SURFELGPFRAMEBUFFER_H_

#include "../FrameBufferInterface.h"
#include "../RendererInterface.h"
#include "../SurfelCollection.h"
#include "../Scene.h"
#include "../Vector3D.h"
#include "../Vector2D.h"
#include "../CImage.h"

#include "SurfelPipeline_SurfSplat/Srf.h"
#include <vector>

/**
 * This class implements the <code>FrameBufferInterface</code> for the <em>SurfelGP Renderer</em>.
 *
 * @author Oliver Knoll
 * @version 1.2
 */
class SurfelGPFrameBuffer : public FrameBufferInterface {

public:

	/**
	 * Creates this <code>SurfelGPFrameBuffer</code> with initial dimension <code>size</code> and
	 * associates it with the <code>model</code>, so the internal buffer sizes can be adjusted
	 * according to the number of <code>Surfel</code>s in the <code>Model</code>.
	 *
	 * @param size
	 *        the initial <code>CSize</code> of this <code>SurfelGPFrameBuffer</code>	 
	 */
	SurfelGPFrameBuffer (const CSize size = CSize (0, 0));
	virtual ~SurfelGPFrameBuffer();

	/**
	 * Sets the surfel context <code>SrfContext</code> for this <code>SurfelGPFrameBuffer</code>.
	 *
	 * @param srf
	 *        a pointer to the surfel context <code>SrfContext</code>
	 */
	void setSurfelContext(SrfContext *srf);

	// ******************************
	// interface FrameBufferInterface
	// ******************************

	void setAttributes (const int attriubtes);
	int getAttributes() const;
	void setValid (const bool valid);
	bool isValid() const;
	void setSize (const CSize newBufferSize);
	CSize getSize() const;

	void setColor (const int x, const int y, const COLORREF newPixelColor);
	COLORREF getColor (const int x, const int y) const;
	CImage *getImage() const;
	void clearImage (const COLORREF clearColor);

	void addWeight (const int x, const int y, const float pixelWeightIncrement);
	void setWeight (const int x, const int y, const float newPixelweight);
	float getWeight (const int x, const int y) const;
	void setDepth (const int x, const int y, const float z);
	float getDepth (const int x, const int y) const;
	void setNormal (const int x, const int y, const Vector3D newNormal);
	void setNormal (const int x, const int y, const float nx, const float ny, const float nz);
	Vector3D getNormal (const int x, const int y) const;
	SurfelInterface* getSurfel(const int x, const int y);
	void addVisibleSurfel (const int x, const int y, SurfelInterface *visibleSurfel);
	void getVisibleSurfels (const int x, const int y,  const std::vector<SurfelInterface *> **visibleSurfels, unsigned int *nofVisible) const;

	void resetPosition (const int x, const int y);
	bool isPixelCovered (const int x, const int y) const;
	bool allPixelsCovered ();
	
	Object* getObjectAtPixel (const int x, const int y) const;
	void setObjectAtPixel (const int x, const int y, Object *object);

	int getNofSplatInfoEntries() const;
	std::vector<SurfelInterface *>* getSurfelsInViewFrustum();
	std::vector<MyDataTypes::TextureCoordinate>* getTextureCoordinatesInViewFrustum();
	std::vector<MyDataTypes::Splat>* getSplatsInViewFrustum();
	void setSplatInfo (SurfelInterface *surfel, const float x, const float y, const float z, const float a, const float b_2, const float c, const int bbox[4]);

	CImage* getZImage() const;
	float getMaxZValue() const;	
	float getMinZValue() const;	

	std::list<SurfelInterface*> *getAllVisibleSurfels ();
	void markAllSurfels ();

private:

	typedef struct bufferEntry {
		unsigned int                      nofVisible;	   // number of visible surfels
		std::vector<SurfelInterface *> *visibleSurfels; // the visible surfels
		Object                    *object;         // the object belonging to the last surfel rendered to this pixel
	} BufferEntry;

	BufferEntry             *buffer;            // the per pixel data which holds pointers to visible surfels

	CSize                   size;               // the size of the buffer in pixels
	int						height_1;			// size.height()-1
	int						height;				// size.height()
	int						width;				// size.width()

	SrfZBufItem*			srfFrameBuffer;		// direct access to internal frame buffer of the surfel renderer
												// this buffer stores pixel weights, z values, and normals

	int                     attributes;			// attributes that are written to the frame buffer during rendering
	CImage					*image;             // contains the rendered image	
	
	bool                    valid;              // true, if the buffer is filled with valid data
	                        

	std::vector<SurfelInterface*> surfels;           // array storing the rendered surfels
	std::vector<MyDataTypes::TextureCoordinate>  texCoords;
	std::vector<MyDataTypes::Splat>              splats;	          // array storing splats rendered to the frame buffer

	unsigned int					nSplats;			// number of splats currently stored in the splats array

	// initializes all buffer entries
	void initBuffer();

	// resets all buffer entries - buffer must already be initialized
	// with 'initBuffer'
	void resetBuffer();

	// removes the current 'buffer', releasing all resources
	void removeBuffer();



};

#endif  // __SURFELGPFRAMEBUFFER_H_

// Some Emacs-Hints -- please don't remove:
//
//  Local Variables:
//  mode:C++
//  tab-width:4
//  End:
