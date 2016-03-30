#ifndef _OFX_HAIR_COLLISION_
#define _OFX_HAIR_COLLISION_

#include "ofMain.h"
#include "ofxHairParticle.h"
#include "ofxHairBoundary.h"
#include "ofxMeshUtil.h"
#include "ofxNearestNeighbour.h"

class ofxHairCollision
{
public:
	ofxHairCollision();

	static void collisionClosestPointOnTriangle(vector<ofxHairParticle*> position, vector<ofPoint> boundaryX, vector<ofVec3f> boundaryNormal, vector<Triangle> triangles );
	static void collisionClosestPointOnTriangleFromNearestVertex(vector<ofxHairParticle*> position, vector<ofPoint> boundaryX, vector<ofVec3f> boundaryNormal, unsigned int *numNeighborTriangles, Triangle **neighborTriangles);
	static void collisionClosestPointOnTriangle_ke_tree(vector<ofxHairParticle*> position, vector<ofPoint> boundaryX, vector<ofVec3f> boundaryNormal, unsigned int *numNeighborTriangles, Triangle **neighborTriangles, float radius );
	float getRadius() {return m_supportRadius; }
	void setRadius(float radius) {m_supportRadius = radius; }

private:
	float m_supportRadius;

	// Nearest Neighbor Using kd-tree
    vector<pair<NNIndex, float> > indices;
};

#endif