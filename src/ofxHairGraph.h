#ifndef _OFX_HAIR_GRAPH_
#define _OFX_HAIR_GRAPH_

#include "ofMain.h"
#include "ofxHairNode.h"
#include "ofxHairGroup.h"
#include "ofxHairModel.h"
#include "ofxHairUtil.h"

class ofxHairGraph
{
public:
	ofxHairGraph();

	// load
	bool loadGraph(const string filename);
	bool loadWeight(const string filename);
	bool loadGuide(const string filename, int resolution);
	bool loadGroup(const string filename);

	// set
	void setNumNodes(const int numNodes) { m_numNodes = numNodes; }
	void setNumEdges(const int numEdges) { m_numEdges = numEdges; }

	// get
	int getNumNodes() { return m_numNodes; }
	int getNumEdges() { return m_numEdges; }
	int getNumGroups() { return m_groups.size(); }
	int getNumNormalNodes() { return m_numNormalNodes; }
	inline ofxHairNode getHairNode(int index) { return m_nodes[index]; } 
	inline vector<ofxHairGroup> getHairGroup() { return m_groups; }
	//inline ofxHairNode getNode(const int index) { return m_nodes[index]; }

	void init();
	void setupGroupLink();
	void setupGuideLink();
	void setupNormalNum();
	void calculateMaximumWeight();
	void calculateEnergyFunction();
	bool exportGuideHair(string filename);
	void debugSetGroupColor(ofxHairModel &model);
	void draw(ofxHairModel &model);
	void drawLink(ofxHairModel &model, int &index);

	// adaptive guide hair
	vector<pair<int,int> > adaptive_strands;
	void sortStrandByEnergyValue();
	vector<int> getSortedGuideStrandIndex(int numAdaptiveHair);
	void debugGuideHairColor(ofxHairModel &model, int numAdaptiveHair);

private:
	vector<ofxHairNode> m_nodes;
	vector<ofxHairGroup> m_groups;

	int m_resolution;
	int m_numNodes;
	int m_numEdges;
	int m_numNormalNodes;
};

#endif