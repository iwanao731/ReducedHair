#ifndef _OFX_HAIR_MODEL_
#define _OFX_HAIR_MODEL_

#include "ofMain.h"
#include "ofxHairStrand.h"
#include "ofxHairParticle.h"

class ofxHairModel
{
public:
	std::vector<ofxHairStrand> strands;

	void addHairStrand(const ofVec3f position, const ofVec3f normal, const float length, const int resolution);
	int getNumParticles() { return m_numParticles; };
	int getNumStrand() { return m_numStrands; }
	bool loadHairModel(string filename);
	bool exportHairModel(string filename);
	bool loadHairModelAsText(string filename);
	bool loadGuideHair(string filename);
	bool exportHairModelAsText(string filename);
	void buildJointHierarchy();
	void buildJointMatrix();
	void updateJointMatrix();
	//void buildJointMatrix2();
	//void updateJointMatrix(ofxHairParticle &particle);

private:
	int m_numStrands;
	int m_numParticles;
};

#endif