#pragma once

#include "ofxHEMesh.h"

namespace hemesh {

	ofxHEMeshFace selectFace(const ofCamera& camera, const ofVec2f& mousePos, const ofxHEMesh& hemesh, ofVec3f &loc);
	ofxHEMeshVertex selectVertex(const ofCamera& camera, const ofVec2f& mousePos, const ofxHEMesh& hemesh, ofVec3f &loc);

} // hemesh::