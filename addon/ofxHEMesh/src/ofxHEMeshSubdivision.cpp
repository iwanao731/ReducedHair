#include "ofxHEMeshSubdivision.h"

ofxHEMeshCornerCutSubdivision::ofxHEMeshCornerCutSubdivision(ofxHEMesh& hemesh)
: hemesh(hemesh)
{
	faces.reserve(hemesh.getNumVertices()+hemesh.getNumEdges()+hemesh.getNumFaces());
}

ofxHEMeshCornerCutSubdivision::~ofxHEMeshCornerCutSubdivision() {}

void ofxHEMeshCornerCutSubdivision::apply() {
	processFaces();
	createVertexFaces();
	createHalfedgeFaces();
	createNewVertices();
	createNewFaces();
}

void ofxHEMeshCornerCutSubdivision::processFaces() {
	ofxHEMeshFaceIterator fit = hemesh.facesBegin();
	ofxHEMeshFaceIterator fite = hemesh.facesEnd();
	int i=0;
	for(; fit != fite; ++fit) {
		// Points on the current face
		vector<ofxHEMesh::Point> points;
		hemesh.facePoints(*fit, points);
		int k = points.size();
		
		// New indices for face
		ofxHEMesh::ExplicitFace face;
		face.reserve(k);
		
		cornerVertices.insert(std::pair<ofxHEMeshFace, VertexMap>(*fit, VertexMap()));
		
		vector<ofxHEMesh::Scalar> weights(k);
		vertexWeights(weights, *fit);
		
		// Calculate the new face vertex positions
		map<ofxHEMeshVertex, ofxHEMeshVertex>& newVertices = cornerVertices[*fit];
		ofxHEMeshFaceCirculator fc = hemesh.faceCirculate(*fit);
		ofxHEMeshFaceCirculator fce = fc;
		int n = 0;
		bool nextBoundary = false;
		bool boundary = hemesh.halfedgeIsOnBoundary(hemesh.halfedgeOpposite(*fc));
		do {
			// Calculate the new position of the vertex
			// based on wighted sum of all face points
			ofxHEMeshVertex v = hemesh.halfedgeVertex(*fc);
			ofxHEMeshHalfedge hnext = hemesh.halfedgeNext(*fc);
			nextBoundary = hemesh.halfedgeIsOnBoundary(hemesh.halfedgeOpposite(hnext));
			if(boundary && nextBoundary) {
				// insert an extra vertex
				ofxHEMesh::Point pt1 = hemesh.halfedgeLerp(*fc, 0.75);
				ofxHEMesh::Point pt2 = hemesh.halfedgeLerp(hnext, 0.25);
				cornerPoints.push_back(pt1);
				cornerPoints.push_back(pt2);
				boundaryVertices.insert(v);
				
				ofxHEMeshVertex vn1(i);
				i++;
				ofxHEMeshVertex vn2(i);
				face.push_back(vn1);
				face.push_back(vn2);
			}
			else {
				// move the existing vertex
				ofxHEMeshVertex vn(i);
				ofxHEMesh::Point pt;
				if(boundary) {
					boundaryVertices.insert(v);
					pt = hemesh.halfedgeLerp(*fc, 0.75);
				}
				else if(nextBoundary) {
					boundaryVertices.insert(v);
					pt = hemesh.halfedgeLerp(hnext, 0.25);
				}
				else {
					pt = points[n]*weights[0];
					for(int _m=1; _m < k; ++_m) {
						int m = (_m+n)%k;
						pt += points[m]*weights[_m];
					}
				}
				cornerPoints.push_back(pt);
				newVertices.insert(std::pair<ofxHEMeshVertex, ofxHEMeshVertex>(v, vn));
				face.push_back(vn);
			}
			
			boundary = nextBoundary;
			++n;
			++i;
			++fc;
		} while(fc != fce);
		
		faces.push_back(face);
	}
}
	
void ofxHEMeshCornerCutSubdivision::createVertexFaces() {
	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		if(boundaryVertices.find(*vit) == boundaryVertices.end()) {
			ofxHEMesh::ExplicitFace face;
			ofxHEMeshVertexCirculator vc = hemesh.vertexCirculate(*vit);
			ofxHEMeshVertexCirculator vce = vc;
			do {
				ofxHEMeshFace f = hemesh.halfedgeFace(*vc);
				face.push_back(cornerVertices[f][*vit]);
				++vc;
			} while(vc != vce);
			faces.push_back(face);
		}
	}
}

void ofxHEMeshCornerCutSubdivision::createHalfedgeFaces() {
	ofxHEMeshEdgeIterator eit = hemesh.edgesBegin();
	ofxHEMeshEdgeIterator eite = hemesh.edgesEnd();
	for(; eit != eite; ++eit) {
		ofxHEMesh::ExplicitFace face(4);
		ofxHEMeshHalfedge h = *eit;
		ofxHEMeshHalfedge ho = hemesh.halfedgeOpposite(h);
		if(!(hemesh.halfedgeIsOnBoundary(h) || hemesh.halfedgeIsOnBoundary(ho))) {
			ofxHEMeshFace f = hemesh.halfedgeFace(h);
			ofxHEMeshFace fo = hemesh.halfedgeFace(ho);
			ofxHEMeshVertex v = hemesh.halfedgeVertex(h);
			ofxHEMeshVertex vo = hemesh.halfedgeVertex(ho);
			face[0] = cornerVertices[fo][vo];
			face[1] = cornerVertices[fo][v];
			face[2] = cornerVertices[f][v];
			face[3] = cornerVertices[f][vo];
			faces.push_back(face);
		}
	}
}

void ofxHEMeshCornerCutSubdivision::createNewVertices() {
	hemesh.clearVertices();
	
	ofxHEMeshFaceIterator fit = hemesh.facesBegin();
	ofxHEMeshFaceIterator fite = hemesh.facesEnd();
	int i = 0;
	for(; fit != fite; ++fit) {
		ofxHEMeshFaceCirculator fc = hemesh.faceCirculate(*fit);
		ofxHEMeshFaceCirculator fce = fc;
		bool nextBoundary = false;
		bool boundary = hemesh.halfedgeIsOnBoundary(hemesh.halfedgeOpposite(*fc));
		do {
			ofxHEMeshHalfedge hnext = hemesh.halfedgeNext(*fc);
			nextBoundary = hemesh.halfedgeIsOnBoundary(hemesh.halfedgeOpposite(hnext));
			if(boundary && nextBoundary) {
				hemesh.addVertex(cornerPoints[i]);
				++i;
				hemesh.addVertex(cornerPoints[i]);
			}
			else {
				hemesh.addVertex(cornerPoints[i]);
			}
			boundary = nextBoundary;
			++i;
			++fc;
		} while(fc != fce);
	}
}

void ofxHEMeshCornerCutSubdivision::createNewFaces() {
	hemesh.clearHalfedges();
	hemesh.clearFaces();
	hemesh.addFaces(faces);
}


ofxHEMeshDooSabinSubdivision::ofxHEMeshDooSabinSubdivision(ofxHEMesh& hemesh)
: ofxHEMeshCornerCutSubdivision(hemesh)
{}

void ofxHEMeshDooSabinSubdivision::vertexWeights(vector<ofxHEMesh::Scalar>& weights, ofxHEMeshFace f) {
	int faceSize = weights.size();
	
	weights[0] = 1./4. + 5./(4.*faceSize);
	for(int i=1; i < faceSize; ++i) {
		weights[i] = (3.+2.*cos(2.*i*PI/faceSize))/(4.*faceSize);
	}
}


ofxHEMeshModifiedCornerCutSubdivision::ofxHEMeshModifiedCornerCutSubdivision(ofxHEMesh& hemesh, ofxHEMesh::Scalar tension)
: ofxHEMeshCornerCutSubdivision(hemesh), tension(tension)
{}

void ofxHEMeshModifiedCornerCutSubdivision::vertexWeights(vector<ofxHEMesh::Scalar>& weights, ofxHEMeshFace f) {
	int faceSize = weights.size();
	
	weights[0] = tension;
	for(int i=1; i < faceSize; ++i) {
		ofxHEMesh::Scalar M = 3.+2.*cos(2.*i*PI/faceSize);
		weights[i] = M*(1.-tension)/(3.*faceSize-5.);
	}
}


ofxHEMeshFacePeel::ofxHEMeshFacePeel(ofxHEMesh& hemesh, ofxHEMesh::Scalar thickness)
: ofxHEMeshModifiedCornerCutSubdivision(hemesh, 1.-thickness)
{}

void ofxHEMeshFacePeel::apply() {
	subdivide();
	createCrust();
}

void ofxHEMeshFacePeel::subdivide() {
	processFaces();
	numFaces = faces.size();
	
	createVertexFaces();
	createHalfedgeFaces();
	createNewVertices();
	createNewFaces();
}

void ofxHEMeshFacePeel::createCrust() {
	
	// duplicate hemesh
	// move vertices
	// reverse faces (ensure that the starting vertices are the same still)
	// add to existing hemesh
	// track corresponding face faces

	innerHemesh = hemesh;
	
	// can assume there are no holes in the faceProperties arrays
	// since the subdivide() operation creates consecutive faces
	// starting at index 0
	map<ofxHEMeshVertex, ofxHEMesh::Direction> movements;
	for(int i=0; i < numFaces; ++i) {
		ofxHEMeshFace f(i);
		ofxHEMeshFaceCirculator fc = innerHemesh.faceCirculate(f);
		ofxHEMeshFaceCirculator fce = fc;
		do {
			// Find offset direction by:
			//	1) Getting normals to just created edge faces
			//	2) Finding normal to plane defined by normals calculated in 1) and edge connected to v
			//	3) Finding the unique direction orthogonal to the normals calculated in 2)
			ofxHEMeshHalfedge h = *fc;
			ofxHEMeshHalfedge h2 = innerHemesh.halfedgeNext(h);
			ofxHEMeshVertex v = innerHemesh.halfedgeVertex(h);
			ofxHEMesh::Point p0 = innerHemesh.vertexPoint(v);
			ofxHEMesh::Point p1 = innerHemesh.vertexPoint(innerHemesh.halfedgeVertex(h2));
			ofxHEMesh::Point p2 = innerHemesh.vertexPoint(innerHemesh.halfedgeVertex(innerHemesh.halfedgePrev(h)));
			ofxHEMesh::Point p3 = innerHemesh.vertexPoint(innerHemesh.halfedgeVertex(innerHemesh.halfedgeNext(innerHemesh.halfedgeOpposite(h2))));
			ofxHEMesh::Point p4 = innerHemesh.vertexPoint(innerHemesh.halfedgeVertex(innerHemesh.halfedgePrev(innerHemesh.halfedgePrev(innerHemesh.halfedgeOpposite(h)))));
			
			ofxHEMesh::Direction dir1 = p1-p0;
			ofxHEMesh::Direction dir2 = p2-p0;
			ofxHEMesh::Direction efn1 = dir1.crossed(p3-p0);
			ofxHEMesh::Direction efn2 = dir2.crossed(p4-p0);
			ofxHEMesh::Direction basis1 = efn1.crossed(dir1).normalize();
			ofxHEMesh::Direction basis2 = dir2.crossed(efn2).normalize();
			ofxHEMesh::Direction dir = basis1.crossed(basis2).normalize();
			movements.insert(std::pair<ofxHEMeshVertex, ofxHEMesh::Direction>(v, dir*-(1-tension)));
			
			++fc;
		} while(fc != fce);
	}
	
	map<ofxHEMeshVertex, ofxHEMesh::Direction>::const_iterator it = movements.begin();
	map<ofxHEMeshVertex, ofxHEMesh::Direction>::const_iterator ite = movements.end();
	for(; it != ite; ++it) {
		innerHemesh.vertexMove(it->first, it->second);
	}
	
	
	int totalFaces = hemesh.getNumFaces();
	innerHemesh.reverseFaces();
	hemesh.addMesh(innerHemesh);
	for(int i=0; i < numFaces; ++i) {
		ofxHEMeshHalfedge h1 = hemesh.faceHalfedge(ofxHEMeshFace(i));
		ofxHEMeshHalfedge h2 = hemesh.nearestVertexInFaceToPoint(hemesh.vertexPoint(hemesh.halfedgeVertex(h1)), ofxHEMeshFace(totalFaces+i));
		h2 = hemesh.halfedgeNext(h2);
		hemesh.connectFacesSimple(h1, h2);
	}
}