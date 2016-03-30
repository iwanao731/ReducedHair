#pragma once
#include "ofMain.h"
#include "ofxHEMesh.h"

class ofxHEMeshOBJMesh;


class ofxHEMeshOBJLoader{
public:
	ofxHEMeshOBJLoader();
	~ofxHEMeshOBJLoader();
	
	bool loadModel(string modelName);
	void addToHemesh(ofxHEMesh& hemesh, int idx);
	int getNumMeshes() const { return objMeshes.size(); }
	
protected:
	void parseFace(const string& line, ofxHEMeshOBJMesh *mesh);

	vector<ofxHEMeshOBJMesh *> objMeshes;
};