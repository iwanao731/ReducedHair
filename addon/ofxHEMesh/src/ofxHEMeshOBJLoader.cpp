#include "ofxHEMeshOBJLoader.h"
#include <iostream>
#include <fstream>

using std::ifstream;


class ofxHEMeshOBJMesh{
public:
	vector<ofxHEMesh::Point> vertices;
	vector<ofxHEMesh::ExplicitFace> faces;
};


ofxHEMeshOBJLoader::ofxHEMeshOBJLoader()
{}

ofxHEMeshOBJLoader::~ofxHEMeshOBJLoader() {
	for(int i=0; i < objMeshes.size(); ++i) {
		delete objMeshes[i];
	}
}

bool ofxHEMeshOBJLoader::loadModel(string modelName) {
	ofFile file;
	file.open(modelName, ofFile::ReadOnly, true); // Since it may be a binary file we should read it in binary -Ed
    if(!file.exists()) {
        ofLogVerbose("ofxAssimpModelLoader") << "loadModel(): model does not exist: \"" << modelName << "\"";
        return false;
    }
	
	ifstream fileStream(file.getAbsolutePath().c_str());
	if(fileStream.is_open()) {
		string line;
		ofxHEMeshOBJMesh *mesh = new ofxHEMeshOBJMesh();
		objMeshes.push_back(mesh);
		
		while(std::getline(file, line)) {
			switch(line[0]) {
				// v, vt, vn
				case 'v':
					switch(line[1]) {
						case 't':
						case 'n':
							// ignore texcoord and normals for now
							break;
							
						case ' ': {
							float x, y, z;
							sscanf(line.c_str(), "v %f %f %f", &x, &y, &z);
							mesh->vertices.push_back(ofxHEMesh::Point(x, y, z));
						}
						break;
					}
					break;
					
					
				case 'f':
					parseFace(line, mesh);
					break;
			}
		}
	}
	fileStream.close();
}

void ofxHEMeshOBJLoader::addToHemesh(ofxHEMesh& hemesh, int idx) {
	if(idx >= 0 && idx < objMeshes.size()) {
		ofxHEMeshOBJMesh *mesh = objMeshes[idx];
		ofxHEMeshVertex vstart(hemesh.getNumVertices());
		
		for(int i=0; i < mesh->vertices.size(); ++i) {
			hemesh.addVertex(mesh->vertices[i]);
		}

		vector<ofxHEMesh::ExplicitFace> faces;
		faces.reserve(mesh->faces.size());
		for(int i=0; i < mesh->faces.size(); ++i) {
			faces.push_back(mesh->faces[i]);
		}
		hemesh.addFaces(faces);
	}
}


void ofxHEMeshOBJLoader::parseFace(const string& line, ofxHEMeshOBJMesh *mesh) {
	ofxHEMesh::ExplicitFace face;

	int v, t, n, read, res;
	int pos = 0;
	
	if(line.find("//") != string::npos) {
		res = sscanf(line.c_str(), "f %d//%d%n", &v, &n, &read);
		while(res == 2) {
			face.push_back(ofxHEMeshVertex(v-1));
			pos += read;
			res = sscanf(line.c_str()+pos, " %d//%d%n", &v, &n, &read);
		}
	}
	else if(line.find('/') != string::npos) {
		if(sscanf(line.c_str(), "f %d/%d/%d", &v, &t, &n) == 3) {
			res = sscanf(line.c_str(), "f %d/%d/%d%n", &v, &t, &n, &read);
			while(res == 3) {
				face.push_back(ofxHEMeshVertex(v-1));
				pos += read;
				res = sscanf(line.c_str()+pos, " %d/%d/%d%n", &v, &t, &n, &read);
			}
		}
		else {
			res = sscanf(line.c_str(), "f %d/%d%n", &v, &n, &read);
			while(res == 2) {
				face.push_back(ofxHEMeshVertex(v-1));
				pos += read;
				res = sscanf(line.c_str()+pos, " %d/%d%n", &v, &n, &read);
			}
		}
	}
	else {
		res = sscanf(line.c_str(), "f %d%n", &v, &read);
		while(res == 1) {
			face.push_back(ofxHEMeshVertex(v-1));
			pos += read;
			res = sscanf(line.c_str()+pos, " %d%n", &v, &read);
		}
	}

	mesh->faces.push_back(face);
}