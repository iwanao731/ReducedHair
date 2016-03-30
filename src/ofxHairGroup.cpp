#include "ofxHairGroup.h"

ofxHairGroup::ofxHairGroup()
{

}

void ofxHairGroup::addNode(ofxHairNode &node)
{
	m_nodes.push_back(node);
}

void ofxHairGroup::calcMaximumWeight()
{
	int index = 0;
	float maxWeight = 0.0;
	for(auto p : getNode()){
		for(auto e : p.getEdge()){
			cout << index << ":" << e.getWeight() << endl;
			index++;
			if(maxWeight < e.getWeight()) {
				maxWeight = e.getWeight();
				cout << maxWeight << endl;
				index = p.getIndex();
			}
		}
	}
}
