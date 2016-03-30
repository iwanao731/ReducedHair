#pragma once

#include "ofxHEMesh.h"

class ofxHEMeshAdaptive : public ofxHEMesh {
public:
	ofxHEMeshAdaptive(Scalar detail);

	void initializeMesh();
	void adapt();
	
	void splitLongEdges();
	inline bool halfedgeShouldBeSplit(ofxHEMeshHalfedge h);
	void splitHalfedgeAndTriangulate(ofxHEMeshHalfedge h);
	void getLongEdges(vector<ofxHEMeshHalfedge>& edges);
	
	void collapseShortEdges();
	inline bool halfedgeShouldBeCollapsed(ofxHEMeshHalfedge h);
	void getShortEdges(vector<ofxHEMeshHalfedge>& edges);

protected:
	
	Scalar detail;
	Scalar detail2;
	Scalar edgeLength;
	Scalar edgeLength2;
	Scalar thickness;
	Scalar thickness2;
	Scalar maxMove;
};