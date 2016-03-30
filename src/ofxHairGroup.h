#ifndef _OFX_HAIR_GROUP_
#define _OFX_HAIR_GROUP_

#include "ofMain.h"
#include "ofxHairNode.h"

class ofxHairGroup
{
public:
	ofxHairGroup();

	// set
	void addNode(ofxHairNode &node);
	void addLinkGroupIndex(int index) { m_linkGroupIndex.push_back(index); }
	void setWeightestNodeIndex(int index) { m_weightestNode = index; }
	void setMaximumEnergyValue(float value) { maximumEnergyValue = value; }
	void setGuideHairIndex(int index) { m_strandIndex = index; }

	// get
	int getNumNode(){ m_nodes.size(); }
	int getWeightestNodeIndex() { return m_weightestNode; }
	int getGuideHairIndex() { return m_strandIndex; }
	inline vector<ofxHairNode> getNode(){ return m_nodes; }
	inline vector<int> getLinkGroupIndex() { return m_linkGroupIndex; }

	// calc
	void calcMaximumWeight();

private:
	int m_weightestNode;
	int m_strandIndex;
	float maximumEnergyValue;
	vector<ofxHairNode> m_nodes;
	vector<int> m_linkGroupIndex;
};

#endif