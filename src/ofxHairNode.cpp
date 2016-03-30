#include "ofxHairNode.h"

void ofxHairNode::addEdge(int i, int j)
{
	ofxHairEdge e;
	e.index1 = i;
	e.index2 = j;
	m_edge.push_back(e);
}

void ofxHairNode::setEdgeWeight(int index, float weight)
{
	for(int i=0; i<m_edge.size(); i++){
		if(m_edge[i].index2 == index){
			m_edge[i].setWeight(weight);
		}
	}
}
