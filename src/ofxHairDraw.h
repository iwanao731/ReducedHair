#ifndef _OFX_HAIR_DRAW_
#define _OFX_HAIR_DRAW_

#include "ofMain.h"
#include "ofxHairModel.h"

class ofxHairDraw
{
public:
	ofxHairDraw(ofxHairModel &model);
	void draw();
	ofxHairDraw& setDrawHairColor(bool v);
	ofxHairDraw& setDrawHairParticles(bool v);
	ofxHairDraw& setDrawHairEdges(bool v);
	ofxHairDraw& setDrawHairNormal(bool v);
	ofxHairDraw& setDrawHairGuide(bool v);


private:
	ofxHairModel& m_model;
	bool bColor;
	bool bNode;
	bool bEdge;
	bool bNormalHair;
	bool bGuideHair;
};

#endif