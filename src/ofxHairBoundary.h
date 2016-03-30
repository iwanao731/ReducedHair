#ifndef _OFX_HAIR_BOUNDARY_
#define _OFX_HAIR_BOUNDARY_

#include "ofMain.h"
#include "ofxHEMesh.h"

struct Triangle{
	int p1, p2, p3;
};

class ofxHairBoundary
{
public:
	vector<ofPoint>  m_points;
	vector<ofVec3f>  m_normal;
	vector<float>    m_boundaryPsi;
	Triangle         **m_neighborTriangle;
	unsigned int     *m_numNeighborTriangle;
	vector<Triangle> m_triangles;

	bool load(string filename);
	void setPosition(vector<ofPoint>  &points);
	int getNumVertices() { return m_points.size(); }
	void calcNeighborVertices();
	void calcNormal();
};

#endif