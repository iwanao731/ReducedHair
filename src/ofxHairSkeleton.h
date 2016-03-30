#ifndef _OFX_HAIR_SKELETON_
#define _OFX_HAIR_SKELETON_

#include "ofMain.h"
#include "ofxHairJoint.h"
#include "ofxHairParticle.h"
#include "ofxHairStrand.h"

class ofxHairSkeleton
{
public:
	
	void loadHairStrand(ofxHairStrand strand);
	void buildJointHierarchy();
	void buildJointMatrix(ofxHairStrand &strand);
	void buildJointMatrix2(ofxHairStrand &strand);
	void buildLocalMatrix();
	void updateJoint(ofxHairJoint &joint); // doesn't work
	void init() { m_joints0 = m_joints; }
	void drawAxis();

	// util
	inline ofxHairJoint getJoint(int index) { return m_joints[index]; }
	inline ofxHairJoint getJoint0(int index) { return m_joints0[index]; }
	int getNumJoints() { return m_numJoints; }
	vector<ofxHairJoint> m_joints;
	vector<ofxHairJoint> m_joints0;

private:
	ofPoint m_root_pos;
	int m_numJoints;
};

#endif