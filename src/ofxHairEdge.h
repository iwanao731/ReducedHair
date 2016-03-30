#ifndef _OFX_HAIR_EDGE_
#define _OFX_HAIR_EDGE_

#include "ofMain.h"

class ofxHairEdge
{
public:
	ofxHairEdge();
	int index1, index2;
	int groupIndex;
	float getWeight(){ return m_weight; };
	void setWeight(const float weight) { m_weight = weight; }

private:
	float m_weight;
	//ofxHairNode m_nodes; // cannot include

};

#endif