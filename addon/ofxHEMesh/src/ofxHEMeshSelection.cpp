#include "ofxHEMeshSelection.h"
#include "ofxRay.h"

namespace hemesh {

ofxHEMeshFace selectFace(const ofCamera& camera, const ofVec2f& mousePos, const ofxHEMesh& hemesh, ofVec3f &loc) {
	ofVec3f rayStart = camera.screenToWorld(ofVec3f(mousePos.x, mousePos.y, 0));
	ofVec3f rayEnd = camera.screenToWorld(ofVec3f(mousePos.x, mousePos.y, 1));
	ofxRay::Ray ray(rayStart, rayEnd-rayStart);
	
	ofxHEMeshFaceIterator fit = hemesh.facesBegin();
	ofxHEMeshFaceIterator fite = hemesh.facesEnd();
	float minDistance = 100000000;
	ofxHEMeshFace minFace;
	for(; fit != fite; ++fit) {
		ofxHEMeshPolygonSplitter pit = hemesh.splitPolygon(*fit);
		ofxHEMeshPolygonSplitter pite = pit;
		do {
			ofxHEMeshTriangle& tri = *pit;
			ofxRay::Plane plane(hemesh.vertexPoint(tri.v1), hemesh.triangleNormal(tri));
			if(ray.t.dot(plane.getNormal()) < 0) {
				if(plane.intersect(ray, loc)) {
					if(hemesh.withinTriangle(tri, loc)) {
						float distance = ray.distanceTo(loc);
						if(distance < minDistance) {
							minDistance = distance;
							minFace = *fit;
						}
					}
				}
			}
			++pit;
		} while(pit != pite);
	}
	return minFace;
}

ofxHEMeshVertex selectVertex(const ofCamera& camera, const ofVec2f& mousePos, const ofxHEMesh& hemesh, ofVec3f &loc) {
	ofxHEMeshFace face = selectFace(camera, mousePos, hemesh, loc);
	if(!face.isValid()) return ofxHEMeshVertex();
	
	ofxHEMeshFaceCirculator fc = hemesh.faceCirculate(face);
	ofxHEMeshFaceCirculator fce = fc;
	float minDistance = 100000000;
	ofxHEMeshVertex minVertex;
	do {
		ofxHEMeshVertex v = hemesh.halfedgeVertex(*fc);
		float distance = hemesh.vertexPoint(v).distanceSquared(loc);
		if(distance < minDistance) {
			minDistance = distance;
			minVertex = v;
		}
		++fc;
	} while(fc != fce);
	return minVertex;
}

}