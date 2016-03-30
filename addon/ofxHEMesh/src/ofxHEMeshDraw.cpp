#include "ofxHEMeshDraw.h"

ofxHEMeshDraw::ofxHEMeshDraw(ofxHEMesh& hemesh, NormalType normalType)
:	hemesh(hemesh),
	normalType(normalType),
	meshVertexNormals(NULL),
	drawVertices(false),
	drawVertexLabels(false),
	drawEdges(false),
	drawBoundaryEdges(false),
	drawFaces(true),
	drawVertexNormals(false),
	calculateVertexNormals(false),
	material(NULL),
	ownsMaterial(false),
	normalScale(0.05)
{
	//setMaterial(RedMaterial);
}


ofxHEMeshDraw::~ofxHEMeshDraw() {
	if(material) delete material;
}

void ofxHEMeshDraw::draw(const ofCamera& camera) {

	if(hemesh.getTopologyDirty()) {
		if(normalType == VertexNormals) {
			calculateVertexNormals = true;
		}
		if(drawEdges.enabled) {
			updateEdges();
		}
		if(drawBoundaryEdges.enabled) {
			updateBoundaryEdges();
		}
		// Do this before updating faces
		if(normalType != NoNormals || drawVertexNormals || drawVertexLabels.enabled) {
			updateNormals();
			if(drawVertexNormals) {
				updateVertexNormalVectors();
			}
		}
		if(drawFaces.enabled) {
			updateFaces();
		}
		
		hemesh.setTopologyDirty(false);
		hemesh.setGeometryDirty(false);
	}
	else if(hemesh.getGeometryDirty()) {
		if(normalType == VertexNormals) {
			calculateVertexNormals = true;
		}
		if(drawEdges.enabled) {
			edges.setVertexData(hemesh.getPoints().ptr(), hemesh.getPoints().size(), GL_DYNAMIC_DRAW);
		}
		if(drawBoundaryEdges.enabled) {
			boundaryEdges.setVertexData(hemesh.getPoints().ptr(), hemesh.getPoints().size(), GL_DYNAMIC_DRAW);
		}
		if(drawFaces.enabled) {
			faces.setVertexData(hemesh.getPoints().ptr(), hemesh.getPoints().size(), GL_DYNAMIC_DRAW);
		}
		if(drawVertexNormals && calculateVertexNormals) {
			updateVertexNormals();
		}
		hemesh.setGeometryDirty(false);
	}
	
	if(drawFaces.enabled) {
		glEnable(GL_LIGHTING);
			glEnable(GL_LIGHT0);		
			glEnable(GL_POLYGON_OFFSET_FILL);
				glPolygonOffset(1, 1);
				if(material) {
					if(ownsMaterial) img.getTextureReference().bind();
					material->begin();
				}
				faces.drawElements(GL_TRIANGLES, faces.getNumIndices());
				if(material) {
					material->end();
					if(ownsMaterial) img.getTextureReference().unbind();
				}
			glDisable(GL_POLYGON_OFFSET_FILL);
		glDisable(GL_LIGHTING);
	}
	if(drawBoundaryEdges.enabled && boundaryEdges.getNumIndices() > 0) {
		glLineWidth(3);
		ofSetColor(ofColor::blue);
		boundaryEdges.drawElements(GL_LINES, boundaryEdges.getNumIndices());
		glLineWidth(1);
	}
	if(drawEdges.enabled) {
		glEnable(GL_BLEND);
		//glBlendFunc(GL_ONE_MINUS_SRC_ALPHA, GL_SRC_ALPHA);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		ofSetColor(ofColor::white, 30);
		edges.drawElements(GL_LINES, edges.getNumIndices());
		glDisable(GL_BLEND);
	}
	if(drawVertices.enabled) {
		//ofSetColor(251, 198, 87);
		ofSetColor(93, 144, 192);
		glPointSize(3);
		faces.draw(GL_POINTS, 0, faces.getNumVertices());
		glPointSize(1);
	}
	if(drawVertexLabels.enabled) {
		//ofSetColor(251, 198, 87);
		ofSetColor(93, 144, 192);
		ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
		ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
		for(; vit != vite; ++vit) {
			ofVec3f pt = hemesh.vertexPoint(*vit);
			//if(camera.getLookAtDir().dot(meshVertexNormals->get((*vit).idx)) <= 0) {
				stringstream ss;
				ss << (*vit).idx;
				ofDrawBitmapString(ss.str(), pt.x, pt.y, pt.z);
			//}
		}
	}
	if(drawVertexNormals) {
		ofSetColor(ofColor::red);
		vertexNormals.draw(GL_LINES, 0, vertexNormals.getNumVertices());
	}
}

bool ofxHEMeshDraw::getDrawVertices() const {
	return drawVertices.enabled;
}

ofxHEMeshDraw& ofxHEMeshDraw::setDrawVertices(bool v) {
	drawVertices.enable(v);
	return *this;
}

bool ofxHEMeshDraw::getDrawVertexLabels() const {
	return drawVertexLabels.enabled;
}

ofxHEMeshDraw& ofxHEMeshDraw::setDrawVertexLabels(bool v) {
	drawVertexLabels.enable(v);
	return *this;
}

bool ofxHEMeshDraw::getDrawEdges() const {
	return drawEdges.enabled;
}

ofxHEMeshDraw& ofxHEMeshDraw::setDrawEdges(bool v) {
	drawEdges.enable(v);
	return *this;
}

bool ofxHEMeshDraw::getDrawBoundaryEdges() const {
	return drawBoundaryEdges.enabled;
}

ofxHEMeshDraw& ofxHEMeshDraw::setDrawBoundaryEdges(bool v) {
	drawBoundaryEdges.enable(v);
	return *this;
}

bool ofxHEMeshDraw::getDrawFaces() const {
	return drawFaces.enabled;
}

ofxHEMeshDraw& ofxHEMeshDraw::setDrawFaces(bool v) {
	drawFaces.enable(v);
	return *this;
}

bool ofxHEMeshDraw::getDrawVertexNormals() const {
	return drawVertexNormals;
}

ofxHEMeshDraw& ofxHEMeshDraw::setDrawVertexNormals(bool v) {
	drawVertexNormals = v;
	if(drawVertexNormals) {
		calculateVertexNormals = true;
	}
	return *this;
}

void ofxHEMeshDraw::setMaterial(MaterialType matType) {
	bool usingArbTex = ofGetUsingArbTex();
	ofDisableArbTex();
	setMaterial(NULL);
	
	switch(matType) {
		case BlackMaterial:
			img.loadImage("matcap2.jpg");
			break;
			
		case RedMaterial:
			img.loadImage("matcap.jpg");
			break;
			
		case ClayMaterial:
			img.loadImage("matcap3.jpg");
			break;
		
		default:
			break;
	}
	
	material = new ofShader();
	material->load("SEMShader");
	ownsMaterial = true;
	
	material->begin();
	material->setUniformTexture("tMatCap", img.getTextureReference(), 0);
	material->end();
	
	if(usingArbTex) {
		ofEnableArbTex();
	}
	else {
		ofDisableArbTex();
	}
}

void ofxHEMeshDraw::setMaterial(ofShader *v) {
	if(ownsMaterial && material) {
		delete material;
	}
	material = v;
}

void ofxHEMeshDraw::faceIndices(vector<ofIndexType>& indices) {
	ofxHEMeshFaceIterator fit = hemesh.facesBegin();
	ofxHEMeshFaceIterator fite = hemesh.facesEnd();
	for(; fit != fite; ++fit) {
		ofxHEMeshPolygonSplitter pit = hemesh.splitPolygon(*fit);
		ofxHEMeshPolygonSplitter pite = pit;
		do {
			ofxHEMeshTriangle& tri = *pit;
			indices.push_back(tri.v1.idx);
			indices.push_back(tri.v2.idx);
			indices.push_back(tri.v3.idx);
			++pit;
		} while (pit != pite);
	}
}

void ofxHEMeshDraw::edgeIndices(vector<ofIndexType>& indices) {
	ofxHEMeshEdgeIterator eit = hemesh.edgesBegin();
	ofxHEMeshEdgeIterator eite = hemesh.edgesEnd();
	for(; eit != eite; ++eit) {
		indices.push_back(hemesh.halfedgeSource(*eit).idx);
		indices.push_back(hemesh.halfedgeSink(*eit).idx);
	}
}

void ofxHEMeshDraw::vertexNormalVectors(vector<ofVec3f> &points, ofxHEMesh::Scalar scale) {
	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		ofxHEMesh::Point pt = hemesh.vertexPoint(*vit);
		ofxHEMesh::Point pt2 = pt + meshVertexNormals->get((*vit).idx);
		points.push_back(pt);
		points.push_back(pt2);
	}
}

void ofxHEMeshDraw::boundaryEdgeIndices(vector<ofIndexType>& indices) {
	ofxHEMeshEdgeIterator eit = hemesh.edgesBegin();
	ofxHEMeshEdgeIterator eite = hemesh.edgesEnd();
	for(; eit != eite; ++eit) {
		ofxHEMeshHalfedge h = *eit;
		ofxHEMeshHalfedge ho = hemesh.halfedgeOpposite(h);
		if(!hemesh.halfedgeFace(h).isValid()) {
			indices.push_back(hemesh.halfedgeSource(h).idx);
			indices.push_back(hemesh.halfedgeSink(h).idx);
		}
		if(!hemesh.halfedgeFace(ho).isValid()) {
			indices.push_back(hemesh.halfedgeSource(ho).idx);
			indices.push_back(hemesh.halfedgeSink(ho).idx);
		}
	}
}

void ofxHEMeshDraw::updateEdges() {
	vector<ofIndexType> indices;
	indices.reserve(hemesh.getNumEdges()*2);
	edgeIndices(indices);
	edges.setVertexData(hemesh.getPoints().ptr(), hemesh.getPoints().size(), GL_DYNAMIC_DRAW);
	edges.setIndexData(&indices[0], indices.size(), GL_DYNAMIC_DRAW);
	drawEdges.needsUpdate = false;
}

void ofxHEMeshDraw::updateBoundaryEdges() {
	vector<ofIndexType> indices;
	boundaryEdgeIndices(indices);
	boundaryEdges.setVertexData(hemesh.getPoints().ptr(), hemesh.getPoints().size(), GL_DYNAMIC_DRAW);
	boundaryEdges.setIndexData(&indices[0], indices.size(), GL_DYNAMIC_DRAW);
	drawEdges.needsUpdate = false;
}

void ofxHEMeshDraw::updateFaces() {
	vector<ofIndexType> indices;
	indices.reserve(hemesh.getNumFaces()*3);	// minimally have triangle faces
	faceIndices(indices);
	faces.setVertexData(hemesh.getPoints().ptr(), hemesh.getPoints().size(), GL_DYNAMIC_DRAW);
	if(normalType == VertexNormals) {
		faces.setNormalData(meshVertexNormals->ptr(), meshVertexNormals->size(), GL_DYNAMIC_DRAW);
	}
	faces.setIndexData(&indices[0], indices.size(), GL_DYNAMIC_DRAW);
	drawFaces.needsUpdate = false;
}

void ofxHEMeshDraw::updateNormals() {
	switch(normalType) {
		case VertexNormals:
			updateVertexNormals();
			break;
			
		case FaceNormals:
			break;
			
		default:
			break;
	}
	
	if(drawVertexNormals && calculateVertexNormals) {
		updateVertexNormals();
	}
}

void ofxHEMeshDraw::updateVertexNormals() {
	if(!meshVertexNormals) {
		meshVertexNormals = hemesh.addVertexProperty<ofxHEMesh::Direction>("vertex-normals");
	}
	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		meshVertexNormals->set((*vit).idx, hemesh.angleWeightedVertexNormal(*vit));
	}
	calculateVertexNormals = false;
}

void ofxHEMeshDraw::updateVertexNormalVectors() {
	vector<ofVec3f> points;
	points.reserve(hemesh.getNumVertices()*2);
	vertexNormalVectors(points, normalScale);
	vertexNormals.setVertexData(&points[0], points.size(), GL_DYNAMIC_DRAW);
}