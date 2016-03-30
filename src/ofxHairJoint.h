#ifndef _OFX_HAIR_JOINT_
#define _OFX_HAIR_JOINT_

#include "ofMain.h"

class ofxHairJoint
{
public:
	ofxHairJoint();
	
	bool bParent;
	bool bSite;
	bool isParent() { return bParent; }
	bool isSite() { return bSite; }

	void setParentJoint(ofxHairJoint *joint) { m_parent = joint; } 
	void setChildJoint(ofxHairJoint *joint) { m_child = joint; }
	inline ofxHairJoint* getParentJoint() { return m_parent; };
	inline ofxHairJoint* getChildJoint() { return m_child; };

	void setOffset(const ofVec3f offset) { m_offset = offset; }
	void setLocalMatrix(const ofMatrix4x4 &mat) { m_localMatrix = mat;}
	void setGrobalMatrix(const ofMatrix4x4 &mat) { m_globalMatrix = mat;}
	inline ofVec3f& getOffset() { return m_offset; }
	inline ofMatrix4x4& getLocalMatrix() { return m_localMatrix; }
	inline ofMatrix4x4& getGlobalMatrix() { return m_globalMatrix; }
	inline ofPoint getPosition() const { return m_globalMatrix.getTranslation(); }

	void setLocalTranslate(const ofVec3f &trans) { m_localTranslate = trans; }
	void setGlobalTranslate(const ofVec3f &trans) { m_globalTranslate = trans; }
	void setLocalRotation(const ofQuaternion &rotate) { m_localRotation = rotate; }
	void setGlobalRotation(const ofQuaternion &rotate) { m_globalRotation = rotate; }
	inline ofVec3f getLocalTranslate() { return m_localTranslate; }
	inline ofVec3f getGlobalTranslate() { return m_globalTranslate; }
	inline ofQuaternion getLocalRotation() { return m_localRotation; }
	inline ofQuaternion getGlobalRotation() { return m_globalRotation; }


	// new feature
	ofVec3f m_localTranslate;
	ofVec3f m_globalTranslate;
	ofQuaternion m_localRotation;
	ofQuaternion m_globalRotation;
private:
	ofxHairJoint *m_parent;
	ofxHairJoint *m_child;
	ofVec3f m_offset;
	ofMatrix4x4 m_localMatrix;
	ofMatrix4x4 m_globalMatrix;


};

#endif