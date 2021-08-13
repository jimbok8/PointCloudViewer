// Title:   Transformer.cpp
// Created: Thu Sep 25 14:09:38 2003
// Authors: Oliver Knoll, Mark Pauly, Matthias Zwicker
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
//#include "stdafx.h"
#include <math.h>

#include "Matrix.h"
#include "Common.h"
//#include "../../Configuration/src/Configuration.h"
#include "Scene.h"
#include "Transformer.h"

// ******
// global
// ******

// the single Transformer instance, only
// accesible by getInstance() and
// destroyInstance()
static Transformer *instance = 0;

// **************
// public methods
// **************

Transformer::Transformer (const CSize viewPortSize, const MyDataTypes::ViewFrustum viewFrustum,
	const MyDataTypes::TransformationMatrix16f sceneTransformationMatrix,
	const MyDataTypes::TransformationMatrix16f objectTransformationMatrix) {

		this->viewPortSize = viewPortSize;
		this->viewFrustum  = viewFrustum;

		memcpy (this->sceneTransformationMatrix, sceneTransformationMatrix, sizeof (MyDataTypes::TransformationMatrix16f));
		memcpy (this->objectTransformationMatrix, objectTransformationMatrix, sizeof (MyDataTypes::TransformationMatrix16f));
		MtrUtil::MtrTranspose4x4f (sceneTransformationMatrix, transposedSceneTransformationMatrix);
		MtrUtil::MtrTranspose4x4f (objectTransformationMatrix, transposedObjectTransformationMatrix);

		MtrUtil::MtrInverse4x4f (sceneTransformationMatrix, inverseSceneTransformationMatrix);
		MtrUtil::MtrInverse4x4f (objectTransformationMatrix, inverseObjectTransformationMatrix);
		MtrUtil::MtrTranspose4x4f (inverseSceneTransformationMatrix, transposedInverseSceneTransformationMatrix);
		MtrUtil::MtrTranspose4x4f (inverseObjectTransformationMatrix, transposedInverseObjectTransformationMatrix);

		this->updateViewPortTrafo();
}

Transformer::~Transformer() {}

Transformer *Transformer::getSceneTransformer() {

	//Configuration                        *configuration;
	Scene                                *scene;
	MyDataTypes::TransformationMatrix16f sceneTransformationMatrix,
		objectTransformationMatrix;
	Object                               *object;

	if (instance == 0) {
		//configuration = Configuration::getInstance();
		scene = Scene::getInstance();
		scene->getTransformationMatrix (sceneTransformationMatrix);
		object = scene->getActiveObject();
		if (object != 0) {
			object->getTransformationMatrix (objectTransformationMatrix);
		}
		else {
			MtrUtil::MtrUnity4x4f (objectTransformationMatrix);
		}
		//instance = new Transformer (configuration->getViewPortSize(),
		//	configuration->getViewFrustum(),
		//	sceneTransformationMatrix,
		//	objectTransformationMatrix);

	}
	return instance;

}

void Transformer::destroySceneTransformer() {

	if (instance != 0) {
		delete instance;
	}
	instance = 0;

}

CSize Transformer::getViewPortSize() const {
	return viewPortSize;
}

MyDataTypes::ViewFrustum Transformer::getViewFrustum() const {
	return viewFrustum;
}


void Transformer::getSceneTransformationMatrix (MyDataTypes::TransformationMatrix16f sceneTransformationMatrix) const {
	memcpy (sceneTransformationMatrix, this->sceneTransformationMatrix, sizeof (MyDataTypes::TransformationMatrix16f));
}

void Transformer::getObjectTransformationMatrix (MyDataTypes::TransformationMatrix16f objectTransformationMatrix) const {
	memcpy (objectTransformationMatrix, this->objectTransformationMatrix, sizeof (MyDataTypes::TransformationMatrix16f));
}


Vector2D Transformer::sceneToViewPort(const Vector3D& osPoint) {

	Vector2D vpPoint;

	vpPoint = cameraToViewPort (sceneToCamera (osPoint));
	return vpPoint;

}

Vector3D Transformer::viewPortToScene (const Vector2D& point, const float depth) {

	Vector3D osPoint;

	osPoint = cameraToScene (viewPortToCamera (point, depth));
	return osPoint;

}

Vector3D Transformer::viewPortToObject(const Vector2D& point, const float depth) {

	Vector3D osPoint;

	osPoint = sceneToObject (cameraToScene (viewPortToCamera (point, depth)));

	return osPoint;
}

Vector2D Transformer::objectToViewPort (const Vector3D &point) {

	Vector2D vpPoint;

	vpPoint = cameraToViewPort (objectToCamera (point));
	return vpPoint;	

}

Vector3D Transformer::sceneToCamera(const Vector3D& osPoint) {

	Vector3D csPoint;
	float    v[4],
		r[4];
	int      i;

	for (i = 0; i < 3; i++) {
		v[i] = osPoint[i];
	}
	v[3] = 1.0f;

	MtrUtil::MtrMultVector4f (sceneTransformationMatrix, v, r);

	for(i = 0; i < 3; i++) {
		csPoint[i] = r[i];
	}

	return csPoint;

}

Vector3D Transformer::cameraToScene (const Vector3D& point) {

	Vector3D osPoint;

	float v[4], r[4];
	int i;

	for(i=0; i<3; i++)
		v[i] = point[i];
	v[3] = 1.f;

	MtrUtil::MtrMultVector4f (inverseSceneTransformationMatrix, v, r);

	for(i=0; i<3; i++) {
		osPoint[i] = r[i];
	}

	return osPoint;

}

Vector3D Transformer::objectToCamera (const Vector3D &point) {

	Vector3D csPoint;
	float    v[4],
		r[4];
	int      i;
	MyDataTypes::TransformationMatrix16f finalTransformationMatrix;

	for (i = 0; i < 3; i++) {
		v[i] = point[i];
	}
	v[3] = 1.0f;

	MtrUtil::MtrMult4x4f (sceneTransformationMatrix, objectTransformationMatrix, finalTransformationMatrix);
	MtrUtil::MtrMultVector4f (finalTransformationMatrix, v, r);

	for (i = 0; i < 3; i++) {
		csPoint[i] = r[i];
	}

	return csPoint;

}

Vector3D Transformer::cameraToObject (const Vector3D &point) {

	Vector3D osPoint;

	MyDataTypes::TransformationMatrix16f finalTransformationMatrix;
	float                                v[4],
		r[4];
	int                                  i;

	for(i=0; i<3; i++) {
		v[i] = point[i];
	}
	v[3] = 1.0f;

	MtrUtil::MtrMult4x4f (inverseObjectTransformationMatrix, inverseSceneTransformationMatrix, finalTransformationMatrix);
	MtrUtil::MtrMultVector4f (finalTransformationMatrix, v, r);

	for(i=0; i<3; i++) {
		osPoint[i] = r[i];
	}

	return osPoint;

}

Vector3D Transformer::sceneToObject (const Vector3D &point) {

	int i;
	float v[4], r[4];
	Vector3D osPoint;

	for(i=0; i<3; i++) {
		v[i] = point[i];
	}
	v[3] = 1.0f;

	MtrUtil::MtrMultVector4f (inverseObjectTransformationMatrix, v, r);

	for(i=0; i<3; i++) {
		osPoint[i] = r[i];
	}

	return osPoint;

}

Vector3D Transformer::objectToScene (const Vector3D &point) {

	int i;
	float v[4], r[4];
	Vector3D ssPoint;

	for(i=0; i<3; i++) {
		v[i] = point[i];
	}
	v[3] = 1.0f;

	MtrUtil::MtrMultVector4f (objectTransformationMatrix, v, r);

	for(i=0; i<3; i++) {
		ssPoint[i] = r[i];
	}

	return ssPoint;

}

Vector2D Transformer::cameraToViewPort (const Vector3D& point) {

	Vector2D vpPoint;
	float    r_z_c;

	r_z_c = -1.0f / point[2];
	vpPoint[0] = (point[0] * r_z_c + viewPortTranslation[0]) * viewPortScaling[0]; 
	vpPoint[1] = viewPortSize.cy - (point[1] * r_z_c + viewPortTranslation[1]) * viewPortScaling[1];

	return vpPoint;

}

Vector3D Transformer::viewPortToCamera (const Vector2D& point, const float depth) {

	Vector3D csPoint;

	csPoint[0] = (point[0] / viewPortScaling[0] - viewPortTranslation[0]) * -depth;
	csPoint[1] = ((viewPortSize.cy - point[1]) / viewPortScaling[1] - viewPortTranslation[1]) * -depth;
	csPoint[2] = depth;

	return csPoint;

}

Vector3D Transformer::sceneToCameraVector(const Vector3D& ssVector) {

	Vector3D csVector;
	float    v[4],
		r[4];
	int      i;

	for (i = 0; i < 3; i++) {
		v[i] = ssVector[i];
	}
	v[3] = 1.0f;

	MtrUtil::MtrMultVector4f (transposedInverseSceneTransformationMatrix, v, r);

	for(i = 0; i < 3; i++) {
		csVector[i] = r[i];
	}

	return csVector;

}

Vector3D Transformer::cameraToSceneVector(const Vector3D& csVector) {
	Vector3D ssVector;

	float    v[4],
		r[4];
	int      i;

	for (i = 0; i < 3; i++) {
		v[i] = csVector[i];
	}
	v[3] = 1.0f;

	MtrUtil::MtrMultVector4f (transposedSceneTransformationMatrix, v, r);

	for(i = 0; i < 3; i++) {
		ssVector[i] = r[i];
	}

	return ssVector;
}

Vector3D Transformer::cameraToObjectVector(const Vector3D& ssVector) {

	MyDataTypes::TransformationMatrix16f cameraToObjectVector;
	Vector3D osVector;
	float    v[4],
		r[4];
	int      i;

	for (i = 0; i < 3; i++) {
		v[i] = ssVector[i];
	}
	v[3] = 1.0f;

	MtrUtil::MtrMult4x4f(transposedObjectTransformationMatrix, transposedSceneTransformationMatrix, cameraToObjectVector);
	MtrUtil::MtrMultVector4f (cameraToObjectVector, v, r);

	for(i = 0; i < 3; i++) {
		osVector[i] = r[i];
	}

	return osVector;

}

Vector3D Transformer::sceneToObjectVector(const Vector3D& ssVector) {

	Vector3D osVector;
	float    v[4],
		r[4];
	int      i;

	for (i = 0; i < 3; i++) {
		v[i] = ssVector[i];
	}
	v[3] = 1.0f;

	MtrUtil::MtrMultVector4f (transposedObjectTransformationMatrix, v, r);

	for(i = 0; i < 3; i++) {
		osVector[i] = r[i];
	}

	return osVector;

}

Vector3D Transformer::objectToCameraVector(const Vector3D& osVector) {

	MyDataTypes::TransformationMatrix16f objectToCameraVector;
	Vector3D csVector;

	float    v[4],
		r[4];
	int      i;

	for (i = 0; i < 3; i++) {
		v[i] = osVector[i];
	}
	v[3] = 1.0f;

	MtrUtil::MtrMult4x4f(transposedInverseSceneTransformationMatrix, transposedInverseObjectTransformationMatrix, objectToCameraVector);
	MtrUtil::MtrMultVector4f (objectToCameraVector, v, r);

	for(i = 0; i < 3; i++) {
		csVector[i] = r[i];
	}

	return csVector;
}

Vector3D Transformer::objectToSceneVector(const Vector3D& osVector) {

	Vector3D ssVector;

	float    v[4],
		r[4];
	int      i;

	for (i = 0; i < 3; i++) {
		v[i] = osVector[i];
	}
	v[3] = 1.0f;

	MtrUtil::MtrMultVector4f (transposedInverseObjectTransformationMatrix, v, r);

	float uniformScaling = MtrUtil::MtrGetUniformScalingFactorSquared4x4f (objectTransformationMatrix);

	for(i = 0; i < 3; i++) {
		ssVector[i] = uniformScaling * r[i];
	}

	return ssVector;
}


// ************
// public slots
// ************

void Transformer::setViewPortSize (const CSize newViewPortSize) {

	viewPortSize = newViewPortSize;
	this->updateViewPortTrafo();
	printf ("Transformer: setViewPortSize: set to %d, %d", viewPortSize.cx, viewPortSize.cy);

}

void Transformer::setViewFrustum (const MyDataTypes::ViewFrustum newViewFrustum) {

	viewFrustum = newViewFrustum;
	this->updateViewPortTrafo();

}

void Transformer::setSceneTransformationMatrix (const float* newSceneTransformationMatrix) {

	memcpy (sceneTransformationMatrix, newSceneTransformationMatrix, sizeof (MyDataTypes::TransformationMatrix16f));
	MtrUtil::MtrTranspose4x4f (sceneTransformationMatrix, transposedSceneTransformationMatrix);

	MtrUtil::MtrInverse4x4f (sceneTransformationMatrix, inverseSceneTransformationMatrix);
	MtrUtil::MtrTranspose4x4f (inverseSceneTransformationMatrix, transposedInverseSceneTransformationMatrix);
}

void Transformer::setObjectTransformationMatrix (const float* newObjectTransformationMatrix) {

	memcpy (objectTransformationMatrix, newObjectTransformationMatrix, sizeof (MyDataTypes::TransformationMatrix16f));
	MtrUtil::MtrTranspose4x4f (objectTransformationMatrix, transposedObjectTransformationMatrix);

	MtrUtil::MtrInverse4x4f (objectTransformationMatrix, inverseObjectTransformationMatrix);
	MtrUtil::MtrTranspose4x4f (inverseObjectTransformationMatrix, transposedInverseObjectTransformationMatrix);
}

// ***************
// private methods
// ***************

void Transformer::updateViewPortTrafo() {

	float width,
		height;

	height = 2.0f * ((float)tan (viewFrustum.fieldOfView / 360.0f * M_PI));
	width  = height * viewFrustum.aspectRatio;
	printf ("Transformer: updateViewPortTrafo: view port size = (%d, %d), image width = %f, height = %f",
		viewPortSize.cx, viewPortSize.cy, width, height);

	viewPortScaling[0] = (float)viewPortSize.cx  / width;
	viewPortScaling[1] = (float)viewPortSize.cy / height;

	viewPortTranslation[0] = width  / 2.0f;
	viewPortTranslation[1] = height / 2.0f;

	printf ("Transformer: updateViewPortTrafo: viewPortScaling [%f / %f] - viewPortTranslation [%f / %f]", 
		viewPortScaling[0], viewPortScaling[1], viewPortTranslation[0], viewPortTranslation[1]);

}

// Some Emacs-Hints -- please don't remove:
//
//  Local Variables:
//  mode:C++
//  tab-width:4
//  End:
