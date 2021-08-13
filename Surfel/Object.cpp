//#include "stdafx.h"
#include "matrix.h"
#include "Object.h"


// **************
// public methods
// **************

Object::Object (const char *name, const SurfelInterface::PropertyDescriptor propertyDescriptor) 
{
	this->name = name;
	this->surfelCollection = new SurfelCollection(propertyDescriptor);

	// get informed whenever the SurfelCollection has changed
	//this->connect (surfelCollection, SIGNAL (surfelCollectionChanged()),
	//	this, SLOT (handleSurfelCollectionChanged()));

	MtrUtil::MtrUnity4x4f (scaleTranslationMatrix);
	MtrUtil::MtrUnity4x4f (rotationMatrix);

	this->setFlag (Object::VISIBLE, true);
	this->setFlag (Object::ENABLED, true);
}

Object::~Object() 
{
	delete surfelCollection;
}

void Object::setName (const char* newName, const bool emitSignal) 
{
	name = newName;
	if (emitSignal == true) {
		//emit objectRenamed();
	}

}

const char* Object::getName() const 
{
	return name.c_str();
}

void Object::setSurfelCollection (std::vector<SurfelInterface*> *surfels, const bool emitSignal) {

	// first remove the existing surfels
	surfelCollection->clear (false);

	surfelCollection->addSurfels (surfels, false);

	if (emitSignal == true) {
		//emit objectModified();
	}

}


SurfelCollection* Object::extractSurfelCollection (const bool emitSignal) {

	SurfelCollection *extracted = surfelCollection;
	surfelCollection = new SurfelCollection();

	if (emitSignal == true) {
		//emit objectModified();
	}

	return extracted;
}



SurfelCollection *Object::getSurfelCollection() {
	return surfelCollection;
}



void Object::setFlag (const Object::Flags flags, const bool on) 
{
	if (on == true) {
		this->flags |= flags;
	}
	else {
		this->flags &= ~flags;
	}
}



bool Object::isFlagOn (const Object::Flags flags) {
	return ((this->flags & flags) == flags);
}



void Object::copy (Object *sourceObject, const bool emitSignal) 
{
	this->name = sourceObject->getName();
	surfelCollection->copy (sourceObject->surfelCollection);
	memcpy (rotationMatrix, sourceObject->rotationMatrix, sizeof (MyDataTypes::TransformationMatrix16f));
	memcpy (scaleTranslationMatrix, sourceObject->scaleTranslationMatrix, sizeof (MyDataTypes::TransformationMatrix16f));

	if (emitSignal == true) {
//		emit objectModified();
	}
}

Object *Object::copy() 
{
	Object *deepCopy;
	deepCopy = new Object(this->name.c_str());
	deepCopy->surfelCollection->copy (surfelCollection);
	memcpy (deepCopy->rotationMatrix, rotationMatrix, sizeof (MyDataTypes::TransformationMatrix16f));
	memcpy (deepCopy->scaleTranslationMatrix, scaleTranslationMatrix, sizeof (MyDataTypes::TransformationMatrix16f));
	return deepCopy;
}

void Object::setRotation (const float angle, const float x, const float y, const float z, const bool emitSignal) 
{
	MtrUtil::MtrCreateRotation4x4fc (angle, x, y, z, rotationMatrix);

	if (emitSignal == true) {
		//emit objectMoved();
	}

}

void Object::rotate (const float dAngle, const float x, const float y, const float z, const bool emitSignal) 
{
	MyDataTypes::TransformationMatrix16f userRotation,
		finalRotation;

	MtrUtil::MtrCreateRotation4x4fc (dAngle, x, y, z, userRotation);
	MtrUtil::MtrMult4x4f (userRotation, rotationMatrix, finalRotation);
	MtrUtil::MtrCopy4x4f (finalRotation, rotationMatrix);

	if (emitSignal == true) {
		//emit objectMoved();
	}
}

void Object::setPosition (const float x, const float y, const float z, const bool emitSignal) 
{
	scaleTranslationMatrix[12] = x;
	scaleTranslationMatrix[13] = y;
	scaleTranslationMatrix[14] = z;

	if (emitSignal == true) {
		//emit objectMoved();
	}
}

void Object::setPosition (const Vector3D newPosition, const bool emitSignal) 
{
	scaleTranslationMatrix[12] = newPosition[0];
	scaleTranslationMatrix[13] = newPosition[1];
	scaleTranslationMatrix[14] = newPosition[2];

	if (emitSignal == true) {
		//emit objectMoved();
	}
}

void Object::getPosition (float *x, float *y, float *z) const 
{
	*x = scaleTranslationMatrix[12];
	*y = scaleTranslationMatrix[13];
	*z = scaleTranslationMatrix[14];
}

Vector3D Object::getPosition() const 
{
	return Vector3D (scaleTranslationMatrix[12], scaleTranslationMatrix[13], scaleTranslationMatrix[14]);
}

void Object::translate (const float dx, const float dy, const float dz, const bool emitSignal) 
{
	scaleTranslationMatrix[12] += dx;
	scaleTranslationMatrix[13] += dy;
	scaleTranslationMatrix[14] += dz;

	if (emitSignal == true) {
		//emit objectMoved();
	}
}

void Object::translate (Vector3D translation, const bool emitSignal) 
{
	scaleTranslationMatrix[12] += translation[0];
	scaleTranslationMatrix[13] += translation[1];
	scaleTranslationMatrix[14] += translation[2];

	if (emitSignal == true) {
		//emit objectMoved();
	}
}

void Object::setScale (const float scaleX, const float scaleY, const float scaleZ, const bool emitSignal) 
{
	scaleTranslationMatrix[0]  = scaleX;
	scaleTranslationMatrix[5]  = scaleY;
	scaleTranslationMatrix[10] = scaleZ;

	if (emitSignal == true) {
		//objectMoved();
	}
}

void Object::setScale (const Vector3D newScale, const bool emitSignal) 
{
	scaleTranslationMatrix[0]  = newScale[0];
	scaleTranslationMatrix[5]  = newScale[1];
	scaleTranslationMatrix[10] = newScale[2];

	if (emitSignal == true) {
		//objectMoved();
	}
}

void Object::getScale (float *scaleX, float *scaleY, float *scaleZ) const 
{
	*scaleX = scaleTranslationMatrix[0];
	*scaleY = scaleTranslationMatrix[5];
	*scaleZ = scaleTranslationMatrix[10];
}

Vector3D Object::getScale() const 
{
	return Vector3D (scaleTranslationMatrix[0], scaleTranslationMatrix[5], scaleTranslationMatrix[10]);
}

void Object::scale (const float dScaleX, const float dScaleY, const float dScaleZ, const bool emitSignal) 
{
	scaleTranslationMatrix[0]  *= dScaleX;
	scaleTranslationMatrix[5]  *= dScaleY;
	scaleTranslationMatrix[10] *= dScaleZ;

	if (emitSignal == true) {
		//emit objectMoved();
	}

}

void Object::scale (const Vector3D scale, const bool emitSignal) 
{
	scaleTranslationMatrix[0]  *= scale[0];
	scaleTranslationMatrix[5]  *= scale[1];
	scaleTranslationMatrix[10] *= scale[2];

	if (emitSignal == true) {
		//emit objectMoved();
	}
}

void Object::getScaleMatrix (MyDataTypes::TransformationMatrix16f scaleMatrix) const 
{
	MtrUtil::MtrUnity4x4f (scaleMatrix);
	scaleMatrix[0] = scaleTranslationMatrix[0];
	scaleMatrix[5] = scaleTranslationMatrix[5];
	scaleMatrix[10] = scaleTranslationMatrix[10];
}

void Object::getTranslationMatrix (MyDataTypes::TransformationMatrix16f translationMatrix) const {

	MtrUtil::MtrUnity4x4f (translationMatrix);
	translationMatrix[12] = scaleTranslationMatrix[12];
	translationMatrix[13] = scaleTranslationMatrix[13];
	translationMatrix[14] = scaleTranslationMatrix[14];
}

void Object::setRotationMatrix (MyDataTypes::TransformationMatrix16f newRotationMatrix, const bool emitSignal) {

	MtrUtil::MtrCopy4x4f (newRotationMatrix, this->rotationMatrix);

	if (emitSignal == true) {
		//emit objectMoved();
	}

}

void Object::getRotationMatrix (MyDataTypes::TransformationMatrix16f rotationMatrix) const {
	MtrUtil::MtrCopy4x4f (this->rotationMatrix, rotationMatrix);
}

void Object::getTransformationMatrix (MyDataTypes::TransformationMatrix16f transformationMatrix) const {
	MtrUtil::MtrMult4x4f (scaleTranslationMatrix, rotationMatrix, transformationMatrix);
}

// *************
// private slots
// *************

void Object::handleSurfelCollectionChanged() {
	//emit objectModified ();
}

// Some Emacs-Hints -- please don't remove:
//
//  Local Variables:
//  mode:C++
//  tab-width:4
//  End:
