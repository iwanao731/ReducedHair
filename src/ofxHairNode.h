#ifndef _OFX_HAIR_NODE_
#define _OFX_HAIR_NODE_

#include "ofMain.h"
#include "ofxHairEdge.h"
//#include "ofxHairJoint.h"

class ofxHairNode
{
public:
	void setIndex(const int index) { m_index = index; }
	void setGroupIndex(const int index) { m_groupIndex = index; }
	void setBoolGuideHair(const bool b) { m_bGuideHair = b; };
	void addEdge(int i, int j);
	void addGuideLink(int index) { m_guideParticleIndices.push_back(index); }
	void clearGuideLink() { m_guideParticleIndices.clear(); }
	void setEdgeWeight(int index, float weight);
	void setStrandIndex(int index) { m_strandIndex = index; }

	int getNumEdges() { return m_edge.size(); };
	inline vector<ofxHairEdge> getEdge() { return m_edge; }
	inline vector<int> getGuideLink() { return m_guideParticleIndices; }
	int getGroupIndex() { return m_groupIndex; }
	int getIndex() { return m_index; }
	bool getBoolGuideHair() { return m_bGuideHair; }
	int getStrandIndex() { return m_strandIndex; }

private:
	int m_index;
	int m_groupIndex;
	bool m_bGuideHair;
	int m_strandIndex;
	vector<ofxHairEdge> m_edge;
	vector<int> m_guideParticleIndices;
//	ofxHairJoint *m_joint;
};

#endif