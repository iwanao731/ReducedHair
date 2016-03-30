#include "ofxHEMesh.h"
#include "ofxHEMeshOBJLoader.h"
#include "ofxHEMeshSubdivision.h"
#include <sstream>


#define WRAP_NEXT(idx, n) (((idx)+1)%(n))
#define WRAP_PREV(idx, n) (((idx)+(n)-1)%(n))

using std::ifstream;
using std::ofstream;
using std::queue;
using std::set;

ofxHEMesh::ofxHEMesh()
:	points(0),
	topologyDirty(false),
	geometryDirty(false)
{
	vertexAdjacency = addVertexProperty<ofxHEMeshVertexAdjacency>("vertex-adjacency", ofxHEMeshVertexAdjacency());
	halfedgeAdjacency = addHalfedgeProperty<ofxHEMeshHalfedgeAdjacency>("halfedge-adjacency", ofxHEMeshHalfedgeAdjacency());
	faceAdjacency = addFaceProperty<ofxHEMeshFaceAdjacency>("face-adjacency", ofxHEMeshFaceAdjacency());
	points = addVertexProperty<Point>("points", Point());
}

ofxHEMesh& ofxHEMesh::operator=(const ofxHEMesh& src) {
	src.vertexProperties.duplicate(vertexProperties);
	src.halfedgeProperties.duplicate(halfedgeProperties);
	src.edgeProperties.duplicate(edgeProperties);
	src.faceProperties.duplicate(faceProperties);
	
	vertexAdjacency = (ofxHEMeshProperty<ofxHEMeshVertexAdjacency> *)vertexProperties.get("vertex-adjacency");
	halfedgeAdjacency = (ofxHEMeshProperty<ofxHEMeshHalfedgeAdjacency> *)halfedgeProperties.get("halfedge-adjacency");
	faceAdjacency = (ofxHEMeshProperty<ofxHEMeshFaceAdjacency> *)faceProperties.get("face-adjacency");
	points = (ofxHEMeshProperty<Point> *)vertexProperties.get("points");
	
	topologyDirty = true;
	geometryDirty = true;
	return *this;
}

void ofxHEMesh::remeshLoop() {
	// Get the location of new (odd) vertices dividing edges in the subd mesh
	map<ofxHEMeshHalfedge, ofxHEMeshVertex> edgeVertices;
	ofxHEMeshEdgeIterator eit = edgesBegin();
	ofxHEMeshEdgeIterator eite = edgesEnd();
	for(; eit != eite; ++eit) {
		ofxHEMeshHalfedge h = *eit;
		ofxHEMeshHalfedge ho = halfedgeOpposite(h);
		
		Point pt = halfedgeMidpoint(h);
		ofxHEMeshVertex v = addVertex(pt);
		edgeVertices.insert(std::pair<ofxHEMeshHalfedge, ofxHEMeshVertex>(h, v));
		edgeVertices.insert(std::pair<ofxHEMeshHalfedge, ofxHEMeshVertex>(halfedgeOpposite(h), v));
	}
	
	// Calculate the subd mesh faces
	vector<ExplicitFace> faces;
	faces.reserve(faceAdjacency->size()*4);
	ofxHEMeshFaceIterator fit = facesBegin();
	ofxHEMeshFaceIterator fite = facesEnd();
	for(; fit != fite; ++fit) {
		ofxHEMeshFaceCirculator fc = faceCirculate(*fit);
		ofxHEMeshFaceCirculator fce = fc;
		
		ExplicitFace innerFace(3);
		int i=0;
		do {
			ExplicitFace cornerFace(3);
			ofxHEMeshHalfedge h = *fc;
			cornerFace[0] = halfedgeSink(h);
			cornerFace[1] = edgeVertices[halfedgeNext(h)];
			cornerFace[2] = edgeVertices[h];
			faces.push_back(cornerFace);
			innerFace[i] = edgeVertices[h];
			++i;
			++fc;
		} while(fc != fce);
		faces.push_back(innerFace);
	}
	
	halfedgeProperties.clear();
	faceProperties.clear();
	addFaces(faces);
}

// Assumes the mesh is a triangulation
void ofxHEMesh::subdivideLoop() {

	// Get the new location of existing (even) vertices in the subd mesh
	ofxHEMeshVertexIterator vit = verticesBegin();
	ofxHEMeshVertexIterator vite = verticesEnd();
	map<ofxHEMeshVertex, Point> newPositions;
	for(; vit != vite; ++vit) {
		ofxHEMeshVertexCirculator vc = vertexCirculate(*vit);
		ofxHEMeshVertexCirculator vce = vc;
		int valence = vertexValence(*vit);
		Scalar beta;
		if(valence == 3) {
			beta = 3./16.;
		}
		else {
			beta = 3./(8.*valence);
		}
		
		Point nPt = vertexPoint(*vit)*(1.-valence*beta);
		bool onBoundary = false;
		do {
			if(halfedgeIsOnBoundary(*vc)) {
				onBoundary = true;
				break;
			}
			nPt += vertexPoint(halfedgeSource(*vc))*beta;
			++vc;
		} while(vc != vce);
		
		if(onBoundary) {
			ofxHEMeshHalfedge h1 = *vc;
			ofxHEMeshHalfedge h2 = halfedgeSinkCW(h1);
			nPt =
				3./4.*vertexPoint(*vit) +
				1./8.*(vertexPoint(halfedgeSource(h1)) + vertexPoint(halfedgeSource(h2)));
		}
		
		newPositions.insert(std::pair<ofxHEMeshVertex, Point>(*vit, nPt));
	}

	// Get the location of new (odd) vertices dividing edges in the subd mesh
	map<ofxHEMeshHalfedge, ofxHEMeshVertex> edgeVertices;
	ofxHEMeshEdgeIterator eit = edgesBegin();
	ofxHEMeshEdgeIterator eite = edgesEnd();
	for(; eit != eite; ++eit) {
		ofxHEMeshHalfedge h = *eit;
		ofxHEMeshHalfedge ho = halfedgeOpposite(h);
		
		Point pt = halfedgeMidpoint(h);
		if(halfedgeIsOnBoundary(h) || halfedgeIsOnBoundary(ho)) {
			pt = halfedgeMidpoint(h);
		}
		else {
			ofxHEMeshHalfedge hn = halfedgeNext(h);
			ofxHEMeshHalfedge hon = halfedgeNext(ho);
			pt =
				(vertexPoint(halfedgeSource(h)) + vertexPoint(halfedgeSink(h)))*0.375 +
				(vertexPoint(halfedgeSink(hn)) + vertexPoint(halfedgeSink(hon)))*0.125;
		}
		ofxHEMeshVertex v = addVertex(pt);
		edgeVertices.insert(std::pair<ofxHEMeshHalfedge, ofxHEMeshVertex>(h, v));
		edgeVertices.insert(std::pair<ofxHEMeshHalfedge, ofxHEMeshVertex>(halfedgeOpposite(h), v));
	}
	
	// Move the existing vertices to their subd mesh positions
	map<ofxHEMeshVertex, Point>::const_iterator npt_it = newPositions.begin();
	map<ofxHEMeshVertex, Point>::const_iterator npt_ite = newPositions.end();
	for(; npt_it != npt_ite; ++npt_it) {
		vertexMoveTo(npt_it->first, npt_it->second);
	}
	
	// Calculate the subd mesh faces
	vector<ExplicitFace> faces;
	faces.reserve(faceAdjacency->size()*4);
	ofxHEMeshFaceIterator fit = facesBegin();
	ofxHEMeshFaceIterator fite = facesEnd();
	for(; fit != fite; ++fit) {
		ofxHEMeshFaceCirculator fc = faceCirculate(*fit);
		ofxHEMeshFaceCirculator fce = fc;
		
		ExplicitFace innerFace(3);
		int i=0;
		do {
			ExplicitFace cornerFace(3);
			ofxHEMeshHalfedge h = *fc;
			cornerFace[0] = halfedgeSink(h);
			cornerFace[1] = edgeVertices[halfedgeNext(h)];
			cornerFace[2] = edgeVertices[h];
			faces.push_back(cornerFace);
			innerFace[i] = edgeVertices[h];
			++i;
			++fc;
		} while(fc != fce);
		faces.push_back(innerFace);
	}
	
	halfedgeProperties.clear();
	faceProperties.clear();
	addFaces(faces);
}

// Assumes no boundaries
void ofxHEMesh::subdivideCatmullClark() {
	map<ofxHEMeshFace, Point> facePoints;
	map<ofxHEMeshFace, ofxHEMeshVertex> faceVertices;
	ofxHEMeshFaceIterator fit = facesBegin();
	ofxHEMeshFaceIterator fite = facesEnd();
	for(; fit != fite; ++fit) {
		Point centroid = faceCentroid(*fit);
		facePoints.insert(std::pair<ofxHEMeshFace, Point>(*fit, centroid));
		
		ofxHEMeshVertex v = addVertex(centroid);
		faceVertices.insert(std::pair<ofxHEMeshFace, ofxHEMeshVertex>(*fit, v));
	}
	
	map<ofxHEMeshHalfedge, Point> edgePoints;
	map<ofxHEMeshHalfedge, ofxHEMeshVertex> edgeVertices;
	ofxHEMeshEdgeIterator eit = edgesBegin();
	ofxHEMeshEdgeIterator eite = edgesEnd();
	for(; eit != eite; ++eit) {
		ofxHEMeshHalfedge h = *eit;
		ofxHEMeshHalfedge ho = halfedgeOpposite(h);
		
		Point pt;
		if(halfedgeIsOnBoundary(h) || halfedgeIsOnBoundary(ho)) {
			pt = halfedgeMidpoint(h);
		}
		else {
			pt = 0.25*(
				vertexPoint(halfedgeSink(h)) + vertexPoint(halfedgeSink(ho)) +
				facePoints[halfedgeFace(h)] + facePoints[halfedgeFace(ho)]
			);
		}
		edgePoints.insert(std::pair<ofxHEMeshHalfedge, Point>(h, pt));
		edgePoints.insert(std::pair<ofxHEMeshHalfedge, Point>(ho, pt));
		
		ofxHEMeshVertex v = addVertex(pt);
		edgeVertices.insert(std::pair<ofxHEMeshHalfedge, ofxHEMeshVertex>(h, v));
		edgeVertices.insert(std::pair<ofxHEMeshHalfedge, ofxHEMeshVertex>(ho, v));
	}
	
	ofxHEMeshVertexIterator vit = verticesBegin();
	ofxHEMeshVertexIterator vite = verticesEnd();
	for(; vit != vite; ++vit) {
		Point Q(0, 0, 0);
		Point R(0, 0, 0);
		Scalar valence = 0;
		ofxHEMeshVertexCirculator vc = vertexCirculate(*vit);
		ofxHEMeshVertexCirculator vce = vc;
		bool onBoundary = false;
		do {
			ofxHEMeshHalfedge h = *vc;
			if(halfedgeIsOnBoundary(h)) {
				onBoundary = true;
				break;
			}
			Q += facePoints[halfedgeFace(h)];
			R += edgePoints[h];
			++valence;
			++vc;
		} while(vc != vce);
		
		Point npt;
		if(onBoundary) {
			ofxHEMeshHalfedge h1 = *vc;
			ofxHEMeshHalfedge h2 = halfedgeSinkCW(h1);
			npt =
				3./4.*vertexPoint(*vit) +
				1./8.*(vertexPoint(halfedgeSource(h1)) + vertexPoint(halfedgeSource(h2)));
		}
		else {	
			Q /= valence;
			R /= valence;
			Point S = vertexPoint(*vit);
			npt = (Q + R*2 + S*(valence-3))/valence;
		}
		vertexMoveTo(*vit, npt);
	}
	
	vector<ExplicitFace> faces;
	faces.reserve(faceAdjacency->size()*4);
	fit = facesBegin();
	fite = facesEnd();
	for(; fit != fite; ++fit) {
		ofxHEMeshFace f = *fit;
		ofxHEMeshFaceCirculator fc = faceCirculate(f);
		ofxHEMeshFaceCirculator fce = fc;
		do {
			ExplicitFace face(4);
			ofxHEMeshHalfedge h = *fc;
			face[0] = edgeVertices[h];
			face[1] = halfedgeVertex(h);
			face[2] = edgeVertices[halfedgeNext(h)];
			face[3] = faceVertices[f];
			faces.push_back(face);
			++fc;
		} while(fc != fce);
	}
	
	halfedgeProperties.clear();
	faceProperties.clear();
	addFaces(faces);
}

void ofxHEMesh::subdivideDooSabin() {
	ofxHEMeshDooSabinSubdivision subd(*this);
	subd.apply();
}

void ofxHEMesh::subdivideModifiedCornerCut(Scalar tension) {
	ofxHEMeshModifiedCornerCutSubdivision subd(*this, tension);
	subd.apply();
}

void ofxHEMesh::facePeel(Scalar thickness) {
	ofxHEMeshFacePeel peel(*this, thickness);
	peel.apply();
}

void ofxHEMesh::dual() {
	// New vertices will be created in order starting form 0 since
	// the old vertices will be cleared out
	int i=0;
	map<ofxHEMeshFace, ofxHEMeshVertex> faceVertices;
	vector<Point> faceCentroids;
	faceCentroids.reserve(faceAdjacency->size());
	ofxHEMeshFaceIterator fit = facesBegin();
	ofxHEMeshFaceIterator fite = facesEnd();
	for(; fit != fite; ++fit) {
		faceVertices.insert(std::pair<ofxHEMeshFace, ofxHEMeshVertex>(*fit, ofxHEMeshVertex(i)));
		faceCentroids.push_back(faceCentroid(*fit));
		++i;
	}
	
	// New faces are the cells around the existing vertices
	vector<ExplicitFace> faces;
	faces.reserve(getNumVertices());
	ofxHEMeshVertexIterator vit = verticesBegin();
	ofxHEMeshVertexIterator vite = verticesEnd();
	for(; vit != vite; ++vit) {
		ExplicitFace face;
		ofxHEMeshVertexCirculator vc = vertexCirculate(*vit);
		ofxHEMeshVertexCirculator vce = vc;
		bool onBoundary = false;
		do {
			if(halfedgeIsOnBoundary(*vc)) {
				onBoundary = true;
				break;
			}
			ofxHEMeshFace f = halfedgeFace(*vc);
			face.push_back(faceVertices[f]);
			++vc;
		} while(vc != vce);
		
		if(!onBoundary) {
			faces.push_back(face);
		}
	}
	
	vertexProperties.clear();
	// Iterates in same order as at top of function so that
	// vertex indices and centroids match
	fit = facesBegin();
	fite = facesEnd();
	i=0;
	for(; fit != fite; ++fit) {
		ofxHEMeshVertex v = addVertex(faceCentroids[i]);
		++i;
	}
	
	halfedgeProperties.clear();
	faceProperties.clear();
	addFaces(faces);
}

void ofxHEMesh::triangulate() {
	int nfaces = getNumFaces();
	for(int i=0; i < nfaces; ++i) {
		ofxHEMeshFace f(i);
		ofxHEMeshHalfedge h = faceHalfedge(f);
		if(h.isValid()) {
			int fsize = faceSize(f);
			if(fsize > 3) {
				ofxHEMeshHalfedge hprev = halfedgePrev(h);
				h = halfedgeNext(h);
				for(int j=3; j < fsize; ++j) {
					ofxHEMeshHalfedge hnext = halfedgeNext(h);
					connectHalfedgesCofacial(h, hprev);
					h = hnext;
				}
			}
		}
	}
}

void ofxHEMesh::centroidTriangulation() {
	vector<ExplicitFace> faces;
	faces.reserve(getNumFaces());
	
	ofxHEMeshFaceIterator fit = facesBegin();
	ofxHEMeshFaceIterator fite = facesEnd();
	for(; fit != fite; ++fit) {
		ofxHEMeshVertex v = addVertex(faceCentroid(*fit));
		ofxHEMeshFaceCirculator fc = faceCirculate(*fit);
		ofxHEMeshFaceCirculator fce = fc;
		do {
			ExplicitFace face(3);
			face[0] = v;
			face[1] = halfedgeSource(*fc);
			face[2] = halfedgeSink(*fc);
			faces.push_back(face);
			++fc;
		} while(fc != fce);
	}
	
	halfedgeProperties.clear();
	faceProperties.clear();
	addFaces(faces);
}

void ofxHEMesh::reverseFaces() {
	ofxHEMeshFaceIterator fit = facesBegin();
	ofxHEMeshFaceIterator fite = facesEnd();
	for(; fit != fite; ++fit) {
		ofxHEMeshFaceCirculator fc = faceCirculate(*fit);
		ofxHEMeshFaceCirculator fce = fc;
		vector<ofxHEMeshHalfedge> halfedges;
		vector<ofxHEMeshVertex> vertices;
		do {
			halfedges.push_back(*fc);
			vertices.push_back(halfedgeVertex(*fc));
			++fc;
		} while(fc != fce);
		
		int n = halfedges.size();
		for(int i=0; i < n; ++i) {
			ofxHEMeshHalfedge& h = halfedges[i];
			ofxHEMeshHalfedge& hn = halfedges[WRAP_NEXT(i, n)];
			ofxHEMeshHalfedge& hp = halfedges[WRAP_PREV(i, n)];
			ofxHEMeshVertex &v = vertices[WRAP_PREV(i, n)];
			setHalfedgeVertex(h, v);
			setVertexHalfedge(v, h);
			setHalfedgeNext(h, hp);
			setHalfedgePrev(h, hn);
		}
	}
	topologyDirty = true;
}

void ofxHEMesh::translate(Direction dir) {
	ofxHEMeshVertexIterator vit = verticesBegin();
	ofxHEMeshVertexIterator vite = verticesEnd();
	for(; vit != vite; ++vit) {
		vertexMove(*vit, dir);
	}
	geometryDirty = true;
}

// Assumes h1 and h2 are two edges to be joined in a face and that
// their respective faces have the same number of vertices
void ofxHEMesh::connectFacesSimple(ofxHEMeshHalfedge h1, ofxHEMeshHalfedge h2) {
	vector<ExplicitFace> faces;
	ofxHEMeshFaceCirculator fc1(this, h1);
	ofxHEMeshFaceCirculator fce1 = fc1;
	
	ofxHEMeshFaceCirculator fc2(this, h2);
	ofxHEMeshFaceCirculator fce2 = fc2;
	
	removeFace(halfedgeFace(h1));
	removeFace(halfedgeFace(h2));
	
	do {
		ExplicitFace face(4);
		face[0] = halfedgeSource(*fc1);
		face[1] = halfedgeSink(*fc1);
		face[2] = halfedgeSource(*fc2);
		face[3] = halfedgeSink(*fc2);
		faces.push_back(face);
		
		--fc2;
		++fc1;
	} while(fc1 != fce1);
	
	for(int i=0; i < faces.size(); ++i) {
		addFace(faces[i]);
	}
}

void ofxHEMesh::connectHalfedgesCofacial(ofxHEMeshHalfedge h1, ofxHEMeshHalfedge h2) {
	ofxHEMeshFace fn(faceProperties.size());
	faceAdjacency->extend();
	
	ofxHEMeshHalfedge hn = addEdge();
	ofxHEMeshHalfedge hno(hn.idx+1);
	
	ofxHEMeshFace f = halfedgeFace(h1);
	
	// associate halfedge with face
	setFaceHalfedge(fn, hno);
	setFaceHalfedge(f, h1);
	
	// set vertex/face adjacency info for new halfedges
	setHalfedgeFace(hn, f);
	setHalfedgeFace(hno, fn);
	setHalfedgeVertex(hn, halfedgeVertex(h2));
	setHalfedgeVertex(hno, halfedgeVertex(h1));
	
	// get halfedges connected to newly created ones
	ofxHEMeshHalfedge h1n = halfedgeNext(h2);
	ofxHEMeshHalfedge h2n = halfedgeNext(h1);
	
	// move halfedges to new face
	ofxHEMeshHalfedge hfn = h2n;
	do {
		setHalfedgeFace(hfn, fn);
		hfn = halfedgeNext(hfn);
	} while(hfn != h2);
	setHalfedgeFace(h2, fn);
	
	// link into existing face
	linkHalfedges(h1, hn);
	linkHalfedges(hn, h1n);
	
	// link into new face
	linkHalfedges(h2, hno);
	linkHalfedges(hno, h2n);
	
	topologyDirty = true;
}

ofxHEMeshVertex ofxHEMesh::splitHalfedgeQuadraticFit(ofxHEMeshHalfedge h) {
	return splitHalfedge(h, halfedgeQuadraticFit(h));
}

ofxHEMeshVertex ofxHEMesh::splitHalfedge(ofxHEMeshHalfedge h, Scalar t) {
	return splitHalfedge(h, halfedgeLerp(h, t));
}

ofxHEMeshVertex ofxHEMesh::splitHalfedge(ofxHEMeshHalfedge h, Point pt) {
	ofxHEMeshVertex vn = addVertex(pt);
	ofxHEMeshHalfedge hn = addEdge();
	ofxHEMeshHalfedge hno(hn.idx+1);
	ofxHEMeshHalfedge ho = halfedgeOpposite(h);
	
	setHalfedgeFace(hn, halfedgeFace(h));
	setHalfedgeFace(hno, halfedgeFace(ho));
	
	ofxHEMeshVertex v = halfedgeVertex(h);
	setVertexHalfedge(vn, h);
	setVertexHalfedge(v, hn);
	setHalfedgeVertex(hn, v);
	setHalfedgeVertex(hno, vn);
	setHalfedgeVertex(h, vn);
	
	ofxHEMeshHalfedge hnn = halfedgeNext(h);
	linkHalfedges(hn, hnn);
	
	ofxHEMeshHalfedge hnop = halfedgePrev(ho);
	linkHalfedges(hnop, hno);
	
	linkHalfedges(h, hn);
	linkHalfedges(hno, ho);
	
	topologyDirty = true;
	geometryDirty = true;
	return vn;
}

ofxHEMeshVertex ofxHEMesh::collapseHalfedgeQuadraticFit(ofxHEMeshHalfedge h) {
	return collapseHalfedge(h, halfedgeQuadraticFit(h));
}

ofxHEMeshVertex ofxHEMesh::collapseHalfedge(ofxHEMeshHalfedge h, Scalar t) {
	return collapseHalfedge(h, halfedgeLerp(h, t));
}

ofxHEMeshVertex ofxHEMesh::collapseHalfedge(ofxHEMeshHalfedge h, Point pt) {
	if(halfedgeEndPointsShareOneRing(h)) {
		return ofxHEMeshVertex();
	}
	
	ofxHEMeshHalfedge ho = halfedgeOpposite(h);
	ofxHEMeshHalfedge hprev = halfedgePrev(h);
	ofxHEMeshHalfedge hoprev = halfedgePrev(ho);
	ofxHEMeshVertex v1 = halfedgeVertex(ho);
	ofxHEMeshVertex v2 = halfedgeVertex(h);
	
	// Move halfedges on v2 to v1
	ofxHEMeshHalfedge from = halfedgeSinkCCW(h);
	do {
		setHalfedgeVertex(from, v1);
		from = halfedgeSinkCCW(from);
	} while(from != h);
	
	// Ensure remaining vertex links to a valid halfedge
	if(vertexHalfedge(v1) == ho) {
		setVertexHalfedge(v1, hoprev);
	}

	// Remove links to h and its opposite
	linkHalfedges(hprev, halfedgeNext(h));
	linkHalfedges(hoprev, halfedgeNext(ho));
	
	// Check for degenerate faces
	ofxHEMeshFace f1 = halfedgeFace(h);
	if(faceHalfedge(f1) == h) {
		setFaceHalfedge(f1, hprev);
	}
	removeFaceIfDegenerate(f1);
	
	ofxHEMeshFace f2 = halfedgeFace(ho);
	if(faceHalfedge(f2) == ho) {
		setFaceHalfedge(f2, hoprev);
	}
	removeFaceIfDegenerate(f2);
	
	eraseHalfedge(h);
	setVertexHalfedge(v2, ofxHEMeshHalfedge());
	removeVertex(v2);
	vertexMoveTo(v1, pt);
	
	return v1;
}

ofxHEMeshHalfedge ofxHEMesh::nearestVertexInFaceToPoint(const Point& pt, ofxHEMeshFace f) const {
	ofxHEMeshFaceCirculator fc = faceCirculate(f);
	ofxHEMeshFaceCirculator fce = fc;

	Scalar minDistance = pt.distanceSquared(vertexPoint(halfedgeVertex(*fc)));
	ofxHEMeshHalfedge minHalfedge = *fc;
	++fc;
	while(fc != fce) {
		Scalar distance = pt.distanceSquared(vertexPoint(halfedgeVertex(*fc)));
		if(distance < minDistance) {
			minDistance = distance;
			minHalfedge = *fc;
		}
		++fc;
	}
	return minHalfedge;
}

bool ofxHEMesh::loadOBJModel(string modelName) {
	ofxHEMeshOBJLoader loader;
	bool res = loader.loadModel(modelName);
	if(res) {
		loader.addToHemesh(*this, 0);
	}
	return res;
}

void ofxHEMesh::addMesh(const ofMesh& mesh) {
	ofxHEMeshVertex vstart(vertexAdjacency->size());
	
	const vector<ofVec3f>& vertices = mesh.getVertices();
	vector<ofVec3f>::const_iterator vit = vertices.begin();
	vector<ofVec3f>::const_iterator vite = vertices.end();

	for(; vit != vite; ++vit) {
		addVertex(*vit);
	}
	
	
	// Assume triangles for now
	// TOOD: use mesh.getMode()
	vector< vector<ofxHEMeshVertex> > faces;
	const vector<ofIndexType>& indices = mesh.getIndices();
	vector<ofIndexType>::const_iterator iit = indices.begin();
	vector<ofIndexType>::const_iterator iite = indices.end();
	
	faces.reserve(indices.size()/3);
	while(iit != iite) {
		vector<ofxHEMeshVertex> face(3);
		for(int i=0; i < 3; ++i) {
			ofxHEMeshVertex v = ofxHEMeshVertex((*iit)+vstart.idx);
			face[i] = v;
			++iit;
		}
		faces.push_back(face);
	}
	addFaces(faces);
}

void ofxHEMesh::addMesh(const ofxHEMesh& hemesh) {
	int vertexOffset = vertexProperties.size();
	int halfedgeOffset = halfedgeProperties.size();
	int faceOffset = faceProperties.size();
	
	vertexProperties.reserve(vertexOffset+hemesh.vertexProperties.size());
	halfedgeProperties.reserve(halfedgeOffset+hemesh.halfedgeProperties.size());
	faceProperties.reserve(faceOffset+hemesh.faceProperties.size());
	
	map<ofxHEMeshVertex, ofxHEMeshVertex> vertexMap;
	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		ofxHEMeshVertex vn = addVertex(hemesh.vertexPoint(*vit));
		vertexMap.insert(std::pair<ofxHEMeshVertex, ofxHEMeshVertex>(*vit, vn));
	}
	
	vector<ExplicitFace> faces;
	ofxHEMeshFaceIterator fit = hemesh.facesBegin();
	ofxHEMeshFaceIterator fite = hemesh.facesEnd();
	for(; fit != fite; ++fit) {
		ExplicitFace face;
		ofxHEMeshFaceCirculator fc = hemesh.faceCirculate(*fit);
		ofxHEMeshFaceCirculator fce = fc;
		do {
			ofxHEMeshVertex v = hemesh.halfedgeVertex(*fc);
			face.push_back(vertexMap[v]);
			++fc;
		} while(fc != fce);
		
		faces.push_back(face);
	}
	addFaces(faces);
}

ofxHEMeshVertex ofxHEMesh::addVertex(const Point& p) {
	int idx = vertexProperties.size();
	vertexProperties.extend();
	points->set(idx, p);
	ofxHEMeshVertex v(idx);
	topologyDirty = true;
	return v;
}

ofxHEMeshHalfedge ofxHEMesh::addEdge() {
	halfedgeProperties.extend();
	halfedgeProperties.extend();
	return ofxHEMeshHalfedge(halfedgeProperties.size()-2);
}

static bool orderVertices(ofxHEMeshVertex& v1, ofxHEMeshVertex& v2) {
	bool swap = v1 > v2;
	if(swap) {
		ofxHEMeshVertex v = v2;
		v2 = v1;
		v1 = v;
	}
	return swap;
}

void printExplicitFace(const ofxHEMesh::ExplicitFace& face) {
	std::cout << "f: ";
	for(int i=0; i < face.size(); ++i) {
		std::cout << face[i].idx << "-";
	}
	std::cout << "\n";
}

void printExplicitFaces(const vector<ofxHEMesh::ExplicitFace>& faces) {
	for(int i=0; i < faces.size(); ++i) {
		printExplicitFace(faces[i]);
	}
}


void ofxHEMesh::addFaces(const vector<ExplicitFace>& faces) {
	map<ExplicitEdge, ofxHEMeshHalfedge> explicitEdgeMap;
	int i, j;
	
	
	// Create any edges that don't yet exist
	set<ofxHEMeshHalfedge> allHalfedges;
	for(i=0; i < faces.size(); ++i) {
		const vector<ofxHEMeshVertex>& face = faces[i];
		int nv = int(face.size());
		for(j=0; j < nv; ++j) {
			ofxHEMeshVertex v1 = face[j];
			ofxHEMeshVertex v2 = face[WRAP_NEXT(j, nv)];
			bool swap = orderVertices(v1, v2);
			if(explicitEdgeMap.find(ExplicitEdge(v1, v2)) == explicitEdgeMap.end()) {
				ofxHEMeshHalfedge h1 = addEdge();
				ofxHEMeshHalfedge h1o = halfedgeOpposite(h1);
				setHalfedgeVertex(h1, v2);
				setHalfedgeVertex(h1o, v1);
				setVertexHalfedge(v2, h1);
				setVertexHalfedge(v1, h1o);

				explicitEdgeMap.insert(std::pair<ExplicitEdge, ofxHEMeshHalfedge>(ExplicitEdge(v1, v2), h1));
				allHalfedges.insert(h1);
				allHalfedges.insert(h1o);
			}
		}
	}
	
	// Link Halfedges around faces
	for(i=0; i < faces.size(); ++i) {

		ofxHEMeshFace f(faceProperties.size());
		faceAdjacency->extend();
	
		const vector<ofxHEMeshVertex>& face = faces[i];
		int nv = int(face.size());
		vector<ofxHEMeshHalfedge> halfedges;
		for(j=0; j < nv; ++j) {
			ofxHEMeshVertex v1 = face[j];
			ofxHEMeshVertex v2 = face[WRAP_NEXT(j, nv)];
			bool didSwap = orderVertices(v1, v2);
			auto iter = explicitEdgeMap.find(ExplicitEdge(v1, v2));
			if(didSwap) halfedges.push_back(halfedgeOpposite(iter->second));
			else halfedges.push_back(iter->second);
		}
		
		setFaceHalfedge(f, halfedges[0]);
		for(j=0; j < nv; ++j) {
			ofxHEMeshHalfedge h1 = halfedges[j];
			ofxHEMeshHalfedge h2 = halfedges[WRAP_NEXT(j, nv)];			
			allHalfedges.erase(allHalfedges.find(h1));
			setHalfedgeNext(h1, h2);
			setHalfedgePrev(h2, h1);
			setHalfedgeFace(h1, f);
		}
	}

	set<ofxHEMeshHalfedge>::const_iterator ahit = allHalfedges.begin();
	set<ofxHEMeshHalfedge>::const_iterator ahite = allHalfedges.end();
	map<ofxHEMeshVertex, ofxHEMeshHalfedge> sinks;
	map<ofxHEMeshVertex, ofxHEMeshHalfedge> sources;
	for(; ahit != ahite; ++ahit) {
		ofxHEMeshHalfedge h = *ahit;
		sinks.insert(std::pair<ofxHEMeshVertex, ofxHEMeshHalfedge>(halfedgeSink(h), h));
		sources.insert(std::pair<ofxHEMeshVertex, ofxHEMeshHalfedge>(halfedgeSource(h), h));
	}
	
	map<ofxHEMeshVertex, ofxHEMeshHalfedge>::const_iterator sink_it = sinks.begin();
	map<ofxHEMeshVertex, ofxHEMeshHalfedge>::const_iterator sink_ite = sinks.end();
	map<ofxHEMeshVertex, ofxHEMeshHalfedge>::const_iterator source_ite = sources.end();
	for(; sink_it != sink_ite; ++sink_it) {
		map<ofxHEMeshVertex, ofxHEMeshHalfedge>::const_iterator source_it = sources.find(sink_it->first);
		if(source_it == source_ite) {
			std::cout << halfedgeString(sink_it->second) << " doesn't have a source\n";
			continue;
		}
		setHalfedgeNext(sink_it->second, source_it->second);
		setHalfedgePrev(source_it->second, sink_it->second);
	}
	topologyDirty = true;
}


ofxHEMeshFace ofxHEMesh::addFace(const ExplicitFace& vertices) {
	vector<ofxHEMeshHalfedge> halfedges;
	vector<bool> exists;
	
	size_t nv = vertices.size();
	size_t i;
	for(i=0; i < nv; ++i) {
		ofxHEMeshVertex v1 = vertices[i];
		ofxHEMeshVertex v2 = vertices[WRAP_NEXT(i, nv)];
		
		ofxHEMeshHalfedge h = findHalfedge(v1, v2);
		bool hExists = h.isValid();

		if(hExists && !halfedgeIsOnBoundary(h)) {
			std::stringstream ss;
			ss << "attempt to add Face where Halfedge " << v1.idx << " " << v2.idx << " isn't on a boundary\n";
			ss << "Face: ";
			for(int j=0; j < nv; ++j) {
				ss << vertices[j].idx << "-";
			}
			std::cout << ss.str();
			throw std::invalid_argument(ss.str());
			return ofxHEMeshFace();
		}
		
		halfedges.push_back(h);
		exists.push_back(hExists);
	}
	
	ofxHEMeshFace f(faceProperties.size());
	faceAdjacency->extend();
	topologyDirty = true;

	// set the face of halfedges and create any new ones necessary
	vector<ofxHEMeshHalfedge> halfedgesPrev(nv);
	vector<ofxHEMeshHalfedge> halfedgesNext(nv);
	for(i=0; i < nv; ++i) {
		if(exists[i]) {
			setHalfedgeFace(halfedges[i], f);
			
			// cache halfedge adjacency information
			halfedgesPrev[i] = halfedgePrev(halfedges[i]);
			halfedgesNext[i] = halfedgeNext(halfedges[i]);
		}
		else {
			ofxHEMeshHalfedge h1 = addEdge();
			ofxHEMeshHalfedge h2 = ofxHEMeshHalfedge(h1.idx+1);
			
			ofxHEMeshVertex v1 = vertices[i];
			ofxHEMeshVertex v2 = vertices[WRAP_NEXT(i, nv)];
			setHalfedgeVertex(h1, v2);
			setHalfedgeVertex(h2, v1);
			setHalfedgeFace(h1, f);
			setVertexHalfedge(v2, h1);
			halfedges[i] = h1;
		}
	}
	
	// link halfedges together
	for(i=0; i < nv; ++i) {
		ofxHEMeshHalfedge h = halfedges[i];
		
		// inner loop
		auto pidx = WRAP_PREV(i, nv);
		auto nidx = WRAP_NEXT(i, nv);
		ofxHEMeshHalfedge p = halfedges[pidx];
		ofxHEMeshHalfedge n = halfedges[nidx];
		setHalfedgePrev(h, p);
		setHalfedgeNext(h, n);
		
		if(!exists[i]) {
			// outer loop
			ofxHEMeshHalfedge ho = halfedgeOpposite(h);
			if(exists[nidx]) {
				ofxHEMeshHalfedge np = halfedgesPrev[nidx];
				setHalfedgePrev(ho, np);
				setHalfedgeNext(np, ho);
			}
			else {
				setHalfedgePrev(ho, halfedgeOpposite(n));
			}
			
			if(exists[pidx]) {
				ofxHEMeshHalfedge nn = halfedgesNext[pidx];
				setHalfedgeNext(ho, nn);
				setHalfedgePrev(nn, ho);
			}
			else {
				setHalfedgeNext(ho, halfedgeOpposite(p));
			}
		}
	}
	
	setFaceHalfedge(f, halfedges[0]);
	return f;
}


void ofxHEMesh::removeVertex(ofxHEMeshVertex v) {
	if(!v.isValid()) throw std::invalid_argument("attempting to remove invalid vertex");
	
	ofxHEMeshHalfedge h = vertexHalfedge(v);
	if(h.isValid()) {
		do {
			ofxHEMeshHalfedge hh = halfedgeSinkCCW(h);
			if(removeHalfedge(h)) {
				break;
			}
			h = hh;
		}
		while(h.isValid());
	}
	setVertexHalfedge(v, ofxHEMeshHalfedge());
	topologyDirty = true;
}


bool ofxHEMesh::removeHalfedge(ofxHEMeshHalfedge h) {
	ofxHEMeshFace f = halfedgeFace(h);
	ofxHEMeshHalfedge ho = halfedgeOpposite(h);
	ofxHEMeshFace f2 = halfedgeFace(ho);

	ofxHEMeshVertex v = halfedgeVertex(h);
	ofxHEMeshVertex v2 = halfedgeVertex(ho);

	ofxHEMeshHalfedge h1p = halfedgePrev(h);
	ofxHEMeshHalfedge h1n = halfedgeSourceCW(h);
	ofxHEMeshHalfedge h2p = halfedgePrev(ho);
	ofxHEMeshHalfedge h2n = halfedgeSourceCW(ho);
	
	ofxHEMeshVertex vrem;
	if(h1p == ho) vrem = v2;
	else vrem = v;
	
	linkHalfedges(h1p, h1n);
	linkHalfedges(h2p, h2n);
	eraseHalfedge(h);

	if(f.idx == f2.idx) {
		// belong to the same face, last edge linking vertex to mesh
		setVertexHalfedge(vrem, ofxHEMeshHalfedge());
		if(vrem != v2) {
			setVertexHalfedge(v2, h1p);
		}
		if(f.isValid()) {
			setFaceHalfedge(f, h1p);
		}
		return true;
	}
	else {
		setVertexHalfedge(v, h2p);
		setVertexHalfedge(v2, h1p);
		setFaceHalfedge(f, ofxHEMeshHalfedge());
		setFaceHalfedge(f2, h1p);
	
		// set all halfedges to the common face
		ofxHEMeshHalfedge hh = h1p;
		do {
			setHalfedgeFace(hh, f2);
			hh = halfedgeNext(hh);
		} while(hh != h1p);
		return false;
	}
}

void ofxHEMesh::eraseHalfedge(ofxHEMeshHalfedge h) {
	ofxHEMeshHalfedge ho = halfedgeOpposite(h);
	halfedgeAdjacency->set(h.idx, ofxHEMeshHalfedgeAdjacency());
	halfedgeAdjacency->set(ho.idx, ofxHEMeshHalfedgeAdjacency());
	topologyDirty = true;
}

void ofxHEMesh::removeFace(ofxHEMeshFace f) {
	ofxHEMeshHalfedge h = faceHalfedge(f);
	ofxHEMeshHalfedge hstart = h;
	do {
		setHalfedgeFace(h, ofxHEMeshFace());
		h = halfedgeNext(h);
	}
	while(h != hstart);
	setFaceHalfedge(f, ofxHEMeshHalfedge());
	topologyDirty = true;
}

bool ofxHEMesh::removeFaceIfDegenerate(ofxHEMeshFace f) {
	bool degenerate = faceIsDegenerate(f);
	if(degenerate) {
		ofxHEMeshHalfedge hprev = faceHalfedge(f);
		ofxHEMeshHalfedge hnext = halfedgeNext(hprev);
		removeFace(f);
		swapHalfedgeAdjacency(halfedgeOpposite(hprev), hnext);
		eraseHalfedge(hprev);
	}
	return degenerate;
}

int ofxHEMesh::getNumVertices() const {
	return (int)vertexAdjacency->size();
}

int ofxHEMesh::getNumEdges() const {
	return ((int)halfedgeAdjacency->size())/2;
}

int ofxHEMesh::getNumHalfedges() const {
	return (int)halfedgeAdjacency->size();
}

int ofxHEMesh::getNumFaces() const {
	return (int)faceAdjacency->size();
}

ofxHEMeshFaceIterator ofxHEMesh::facesBegin() const {
	ofxHEMeshFace f(0);
	if(faceAdjacency->size() > 0) {
		while(!faceHalfedge(f).isValid()) {
			++f.idx;
		}
	}
	return ofxHEMeshFaceIterator(this, f);
}

ofxHEMeshFaceIterator ofxHEMesh::facesEnd() const {
	return ofxHEMeshFaceIterator(this, ofxHEMeshFace(faceAdjacency->size()));
}

ofxHEMeshEdgeIterator ofxHEMesh::edgesBegin() const {
	ofxHEMeshHalfedge h(0);
	if(halfedgeAdjacency->size() > 0) {
		while(!halfedgeFace(h).isValid()) {
			++h.idx;
		}
	}
	return ofxHEMeshEdgeIterator(this, h);
}

ofxHEMeshEdgeIterator ofxHEMesh::edgesEnd() const {
	return ofxHEMeshEdgeIterator(this, ofxHEMeshHalfedge(halfedgeAdjacency->size()));
}

ofxHEMeshVertexIterator ofxHEMesh::verticesBegin() const {
	ofxHEMeshVertex v(0);
	if(vertexAdjacency->size() > 0) {
		while(!vertexHalfedge(v).isValid()) {
			++v.idx;
		}
	}
	return ofxHEMeshVertexIterator(this, v);
}

ofxHEMeshVertexIterator ofxHEMesh::verticesEnd() const {
	return ofxHEMeshVertexIterator(this, ofxHEMeshVertex(vertexAdjacency->size()));
}

ofxHEMeshFaceCirculator ofxHEMesh::faceCirculate(const ofxHEMeshFace& f) const {
	return ofxHEMeshFaceCirculator(this, faceHalfedge(f));
}

ofxHEMeshPolygonSplitter ofxHEMesh::splitPolygon(const ofxHEMeshFace& f) const {
	return ofxHEMeshPolygonSplitter(this, faceHalfedge(f));
}

ofxHEMeshVertexCirculator ofxHEMesh::vertexCirculate(const ofxHEMeshVertex& v) const {
	return ofxHEMeshVertexCirculator(this, vertexHalfedge(v));
}

ofxHEMeshHalfedge ofxHEMesh::vertexHalfedge(ofxHEMeshVertex v) const {
	return vertexAdjacency->get(v.idx).he;
}


void ofxHEMesh::setVertexHalfedge(ofxHEMeshVertex v, ofxHEMeshHalfedge h) {
	vertexAdjacency->get(v.idx).he = h;
}


ofxHEMeshHalfedge ofxHEMesh::faceHalfedge(ofxHEMeshFace f) const {
	return faceAdjacency->get(f.idx).he;
}


void ofxHEMesh::setFaceHalfedge(ofxHEMeshFace f, ofxHEMeshHalfedge h) {
	faceAdjacency->get(f.idx).he = h;
}


ofxHEMeshHalfedge ofxHEMesh::halfedgeOpposite(ofxHEMeshHalfedge h) const {
	if((h.idx & 1) == 1) return ofxHEMeshHalfedge(h.idx-1);
	else return ofxHEMeshHalfedge(h.idx+1);
}


ofxHEMeshVertex ofxHEMesh::halfedgeVertex(ofxHEMeshHalfedge h) const {
	return halfedgeAdjacency->get(h.idx).v;
}


void ofxHEMesh::setHalfedgeVertex(ofxHEMeshHalfedge h, ofxHEMeshVertex v) {
	halfedgeAdjacency->get(h.idx).v = v;
}


ofxHEMeshFace ofxHEMesh::halfedgeFace(ofxHEMeshHalfedge h) const {
	return halfedgeAdjacency->get(h.idx).f;
}


void ofxHEMesh::setHalfedgeFace(ofxHEMeshHalfedge h, ofxHEMeshFace f) {
	halfedgeAdjacency->get(h.idx).f = f;
}


ofxHEMeshVertex ofxHEMesh::halfedgeSource(ofxHEMeshHalfedge h) const {
	return halfedgeVertex(halfedgeOpposite(h));
}


ofxHEMeshVertex ofxHEMesh::halfedgeSink(ofxHEMeshHalfedge h) const {
	return halfedgeVertex(h);
}


ofxHEMeshHalfedge ofxHEMesh::halfedgeNext(ofxHEMeshHalfedge h) const {
	return halfedgeAdjacency->get(h.idx).next;
}


void ofxHEMesh::setHalfedgeNext(ofxHEMeshHalfedge h, ofxHEMeshHalfedge next) {
	halfedgeAdjacency->get(h.idx).next = next;
}


ofxHEMeshHalfedge ofxHEMesh::halfedgePrev(ofxHEMeshHalfedge h) const {
	return halfedgeAdjacency->get(h.idx).prev;
}


void ofxHEMesh::setHalfedgePrev(ofxHEMeshHalfedge h, ofxHEMeshHalfedge prev) {
	halfedgeAdjacency->get(h.idx).prev = prev;
}


ofxHEMeshHalfedge ofxHEMesh::halfedgeSourceCW(ofxHEMeshHalfedge h) const {
	return halfedgeNext(halfedgeOpposite(h));
}


ofxHEMeshHalfedge ofxHEMesh::halfedgeSourceCCW(ofxHEMeshHalfedge h) const {
	return halfedgeOpposite(halfedgePrev(h));
}


ofxHEMeshHalfedge ofxHEMesh::halfedgeSinkCW(ofxHEMeshHalfedge h) const {
	return halfedgeOpposite(halfedgeNext(h));
}


ofxHEMeshHalfedge ofxHEMesh::halfedgeSinkCCW(ofxHEMeshHalfedge h) const {
	return halfedgePrev(halfedgeOpposite(h));
}



ofxHEMeshHalfedge ofxHEMesh::findHalfedge(ofxHEMeshVertex v1, ofxHEMeshVertex v2) const {
	ofxHEMeshHalfedge h1 = vertexHalfedge(v1);
	ofxHEMeshHalfedge h2 = vertexHalfedge(v2);
	if(h1.isValid() && h2.isValid()) {
		ofxHEMeshHalfedge hStart = h2;
		do {
			if(halfedgeSource(h2) == v1) {
				return h2;
			}
			h2 = halfedgeSinkCCW(h2);
		} while(h2 != hStart);
	}
	return ofxHEMeshHalfedge();
}

bool ofxHEMesh::halfedgeIsOnBoundary(ofxHEMeshHalfedge h) const {
	return !halfedgeFace(h).isValid();
}

void ofxHEMesh::linkHalfedges(ofxHEMeshHalfedge prev, ofxHEMeshHalfedge next) {
	setHalfedgeNext(prev, next);
	setHalfedgePrev(next, prev);
}

void ofxHEMesh::swapHalfedgeAdjacency(ofxHEMeshHalfedge src, ofxHEMeshHalfedge dst) {
	ofxHEMeshHalfedgeAdjacency tmp = halfedgeAdjacency->get(src.idx);
	halfedgeAdjacency->set(src.idx, halfedgeAdjacency->get(dst.idx));
	halfedgeAdjacency->set(dst.idx, tmp);
	linkHalfedges(dst, halfedgeNext(dst));
	linkHalfedges(halfedgePrev(dst), dst);
	
	ofxHEMeshFace f = halfedgeFace(dst);
	if(faceHalfedge(f) == src) {
		setFaceHalfedge(f, dst);
	}
	
	ofxHEMeshVertex v = halfedgeVertex(dst);
	if(vertexHalfedge(v) == src) {
		setVertexHalfedge(v, dst);
	}
	
	ofxHEMeshHalfedge dsto = halfedgeOpposite(dst);
	ofxHEMeshVertex vo = halfedgeVertex(dsto);
	if(vertexHalfedge(vo) == halfedgeOpposite(src)) {
		setVertexHalfedge(vo, dsto);
	}
}

int ofxHEMesh::faceSize(ofxHEMeshFace f) const {
	ofxHEMeshFaceCirculator fc = faceCirculate(f);
	ofxHEMeshFaceCirculator fce = fc;
	int n = 0;
	do {
		++n;
		++fc;
	} while(fc != fce);
	return n;
}

bool ofxHEMesh::faceIsDegenerate(ofxHEMeshFace f) const {
	ofxHEMeshHalfedge h = faceHalfedge(f);
	return h == halfedgeNext(halfedgeNext(h));
}

bool ofxHEMesh::halfedgeEndPointsShareOneRing(ofxHEMeshHalfedge h) const {
	ofxHEMeshVertex v1 = halfedgeSource(h);
	ofxHEMeshVertex v2 = halfedgeSink(h);

	set<ofxHEMeshVertex> oneRing1;
	vertexOneRing(v1, oneRing1);
	
	oneRing1.erase(halfedgeVertex(halfedgeSourceCW(h)));
	oneRing1.erase(halfedgeVertex(halfedgeSourceCCW(h)));
	
	ofxHEMeshVertexCirculator vc2 = vertexCirculate(v2);
	ofxHEMeshVertexCirculator vce2 = vc2;
	do {
		if(oneRing1.find(halfedgeSource(*vc2)) != oneRing1.end()) {
			return true;
		}
		++vc2;
	} while(vc2 != vce2);
	return false;
}

int ofxHEMesh::vertexValence(ofxHEMeshVertex v) const {
	int n=0;
	ofxHEMeshVertexCirculator vc = vertexCirculate(v);
	ofxHEMeshVertexCirculator vce = vc;
	do {
		++n;
		++vc;
	} while(vc != vce);
	return n;
}

void ofxHEMesh::vertexOneRing(ofxHEMeshVertex v, set<ofxHEMeshVertex>& oneRing) const {
	ofxHEMeshVertexCirculator vc = vertexCirculate(v);
	ofxHEMeshVertexCirculator vce = vc;
	do {
		oneRing.insert(halfedgeSource(*vc));
		++vc;
	} while(vc != vce);
}

bool ofxHEMesh::verticesShareOneRing(ofxHEMeshVertex v1, ofxHEMeshVertex v2) const {
	set<ofxHEMeshVertex> oneRing1;
	vertexOneRing(v1, oneRing1);
	
	ofxHEMeshVertexCirculator vc2 = vertexCirculate(v2);
	ofxHEMeshVertexCirculator vce2 = vc2;
	do {
		if(oneRing1.find(halfedgeSource(*vc2)) != oneRing1.end()) {
			return true;
		}
		++vc2;
	} while(vc2 != vce2);
	return false;
}

bool ofxHEMesh::halfedgeIsInFace(ofxHEMeshFace f, ofxHEMeshHalfedge h) const {
	ofxHEMeshFaceCirculator fc = faceCirculate(f);
	ofxHEMeshFaceCirculator fce = fc;
	do {
		if(h == *fc) return true;
		++fc;
	} while(fc != fce);
	return false;
}

bool ofxHEMesh::halfedgeLinksToVertex(ofxHEMeshVertex v, ofxHEMeshHalfedge h) const {
	ofxHEMeshVertexCirculator vc = vertexCirculate(v);
	ofxHEMeshVertexCirculator vce = vc;
	std::cout << "halfedgeLinksToVertex: " << v.idx << ":\n";
	int i=0;
	do {
		if(h == *vc) return true;
		++i;
		//if(i > 6) {
		//	std::cout << "\tchecking: " << (*vc).idx << ":" << halfedgeString(*vc) << " " << h.idx << ":" << halfedgeString(h) << "\n";
		//}
		++vc;
	} while(vc != vce);
	return false;
}

void ofxHEMesh::vertexMove(ofxHEMeshVertex v, const Direction& dir) {
	vertexMoveTo(v, vertexPoint(v)+dir);
}

void ofxHEMesh::vertexMoveTo(ofxHEMeshVertex v, const Point& p) {
	points->set(v.idx, p);
	geometryDirty = true;
}

ofxHEMesh::Point ofxHEMesh::centroid() const {
	Point c(0, 0, 0);
	Scalar n = 0;
	ofxHEMeshVertexIterator vit = verticesBegin();
	ofxHEMeshVertexIterator vite = verticesEnd();
	for(; vit != vite; ++vit) {
		c += vertexPoint(*vit);
		++n;
	}
	c *= 1./n;
	return c;
}

ofxHEMesh::Scalar ofxHEMesh::meanEdgeLength() const {
	ofxHEMeshEdgeIterator eit = edgesBegin();
	ofxHEMeshEdgeIterator eite = edgesEnd();
	Scalar res = 0;
	Scalar n = 0;
	for(; eit != eite; ++eit) {
		res += halfedgeLength(*eit);
		++n;
	}
	return res/n;
}

ofxHEMesh::Scalar ofxHEMesh::edgeLengthVariance(Scalar expected) const {
	ofxHEMeshEdgeIterator eit = edgesBegin();
	ofxHEMeshEdgeIterator eite = edgesEnd();
	Scalar res = 0;
	Scalar n = 0;
	for(; eit != eite; ++eit) {
		Scalar v = halfedgeLength(*eit) - expected;
		res += v*v;
		++n;
	}
	return res/n;
}

ofxHEMesh::Point ofxHEMesh::vertexPoint(ofxHEMeshVertex v) const {
	return points->get(v.idx);
}

void ofxHEMesh::vertexOneHoodFaces(ofxHEMeshVertex v, vector<ofxHEMeshFace>& faces) const {

	ofxHEMeshVertexCirculator vc = vertexCirculate(v);
	ofxHEMeshVertexCirculator vce = vc;
	do {
		faces.push_back(halfedgeFace(*vc));
		++vc;
	} while(vc != vce);
}

ofxHEMesh::Direction ofxHEMesh::angleWeightedVertexNormal(ofxHEMeshVertex v) const {
	ofxHEMeshVertexCirculator vc = vertexCirculate(v);
	ofxHEMeshVertexCirculator vce = vc;
	
	Direction n(0, 0, 0);
	do {
		ofxHEMeshHalfedge& h = *vc;
		
		Scalar theta = angleAtVertex(h);
		ofxHEMeshFace f = halfedgeFace(h);
		if(f.isValid()) {
			n += faceNormal(f)*theta;
		}
		++vc;
	} while(vc != vce);
	// TODO: check for NaNs
	n.normalize();
	return n;
}

ofxHEMesh::Scalar ofxHEMesh::vertexArea(ofxHEMeshVertex v) const {
	Scalar A = 0;
	ofxHEMeshVertexCirculator vc = vertexCirculate(v);
	ofxHEMeshVertexCirculator vce = vc;
	do {
		ofxHEMeshHalfedge h = *vc;
		ofxHEMeshFace f = halfedgeFace(h);
		A += faceArea(f);
		h = halfedgeSinkCCW(h);
		++vc;
	} while(vc != vce);
	// TODO: check for NaN
	return A*0.333333333333333;
}

void ofxHEMesh::vertexQuadric(ofxHEMeshVertex v, ofMatrix4x4& Q) const {
	ofxHEMeshVertexCirculator vc = vertexCirculate(v);
	ofxHEMeshVertexCirculator vce = vc;
	do {
		ofMatrix4x4 faceQ;
		faceQuadric(halfedgeFace(*vc), faceQ);
		float *ptrQ = Q.getPtr();
		float *ptrFace = faceQ.getPtr();
		for(int i=0; i < 16; ++i) {
			ptrQ[i] += ptrFace[i];
		}
		++vc;
	} while(vc != vce);
}

void ofxHEMesh::facePointsIndices(ofxHEMeshFace f, vector<int>& indices) const {
	ofxHEMeshFaceCirculator fc = faceCirculate(f);
	ofxHEMeshFaceCirculator fce = fc;
	do {
		indices.push_back(halfedgeVertex(*fc).idx);
		++fc;
	} while(fc != fce);
}

void ofxHEMesh::facePoints(ofxHEMeshFace f, vector<Point>& points) const {
	ofxHEMeshFaceCirculator fc = faceCirculate(f);
	ofxHEMeshFaceCirculator fce = fc;
	do {
		points.push_back(vertexPoint(halfedgeVertex(*fc)));
		++fc;
	} while(fc != fce);
}

ofxHEMeshTriangle ofxHEMesh::face2Triangle(ofxHEMeshFace f) const {
	ofxHEMeshPolygonSplitter pit = splitPolygon(f);
	return *pit;
}

void ofxHEMesh::faceOneHoodFaces(ofxHEMeshFace f, vector<ofxHEMeshFace>& faces) const {	
	
	ofxHEMeshHalfedge h_start = faceHalfedge(f);
	ofxHEMeshHalfedge h_end = h_start;

	do {
		ofxHEMeshHalfedge h = h_start; // start
		ofxHEMeshHalfedge ht = h;	// temp
		ofxHEMeshHalfedge he = ht;	// end
		int i=0;
		do {
			h = halfedgeNext(h);
			if(ht == h){
				if(i!=0){
					faces.push_back(halfedgeFace(h));
				}
				h = halfedgePrev(h);
				h = halfedgeOpposite(h);
				ht = h;
				i++;
			}
		} while (h != he);
		h_start = halfedgeNext(h_start);
	} while (h_start != h_end);
}

ofxHEMesh::Scalar ofxHEMesh::faceArea(ofxHEMeshFace f) const {
	// TODO: general case vs. Triangle case
	ofxHEMeshHalfedge h1 = faceHalfedge(f);
	ofxHEMeshHalfedge h2 = halfedgeNext(h1);
	Direction v1 = halfedgeDirection(h1);
	Direction v2 = halfedgeDirection(h2);
	Direction n = v1.cross(v2);
	return 0.5*n.length();
}

ofxHEMesh::Point ofxHEMesh::faceCentroid(ofxHEMeshFace f) const {
	Point centroid(0, 0, 0);
	float n = 0;
	ofxHEMeshFaceCirculator fc = faceCirculate(f);
	ofxHEMeshFaceCirculator fce = fc;
	do {
		centroid += vertexPoint(halfedgeVertex(*fc));
		++n;
		++fc;
	} while(fc != fce);
	
	return centroid*(1/n);
}

ofxHEMesh::Direction ofxHEMesh::faceNormal(ofxHEMeshFace f) const {
	ofxHEMeshHalfedge h1 = faceHalfedge(f);
	ofxHEMeshHalfedge h2 = halfedgeNext(h1);
	Direction v1 = halfedgeDirection(h1);
	Direction v2 = halfedgeDirection(h2);
	Direction n = v1.cross(v2);
	n.normalize();
	return n;
}

void ofxHEMesh::faceQuadric(ofxHEMeshFace f, ofMatrix4x4& Q) const {
	Direction n = faceNormal(f);
	Point p = vertexPoint(halfedgeVertex(faceHalfedge(f)));
	double d = -n.dot(p);
	
	Q(0, 0) = n[0]*n[0];
	Q(1, 0) = n[0]*n[1];
	Q(2, 0) = n[0]*n[2];
	Q(3, 0) = n[0]*d;
	
	Q(0, 1) = n[1]*n[0];
	Q(1, 1) = n[1]*n[1];
	Q(2, 1) = n[1]*n[2];
	Q(3, 1) = n[1]*d;
	
	Q(0, 2) = n[2]*n[0];
	Q(1, 2) = n[2]*n[1];
	Q(2, 2) = n[2]*n[2];
	Q(3, 2) = n[2]*d;
	
	Q(0, 3) = d*n[0];
	Q(1, 3) = d*n[1];
	Q(2, 3) = d*n[2];
	Q(3, 3) = d*d;
}

ofxHEMesh::Point ofxHEMesh::halfedgeLerp(ofxHEMeshHalfedge h, Scalar t) const {
	Point p1 = vertexPoint(halfedgeSource(h));
	Point p2 = vertexPoint(halfedgeSink(h));
	return p1+(p2-p1)*t;
}

ofxHEMesh::Point ofxHEMesh::halfedgeMidpoint(ofxHEMeshHalfedge h) const {
	return halfedgeLerp(h, 0.5);
}

ofxHEMesh::Scalar ofxHEMesh::halfedgeCotan(ofxHEMeshHalfedge h) const {
	ofxHEMeshHalfedge h2 = halfedgeNext(h);
	ofxHEMeshHalfedge h3 = halfedgeNext(h2);
	ofxHEMeshVertex v1 = halfedgeVertex(h2);
	ofxHEMeshVertex v2 = halfedgeVertex(h3);
	ofxHEMeshVertex v3 = halfedgeVertex(h);
	Point p0 = vertexPoint(v1);
	Point p1 = vertexPoint(v2);
	Point p2 = vertexPoint(v3 );
	
	Direction u = p1-p0;
	Direction v = p2-p0;

	Scalar res = u.dot(v)/u.cross(v).length();
	// TODO: check for NaNs
	return res;
}

ofxHEMesh::Scalar ofxHEMesh::halfedgeLengthSquared(ofxHEMeshHalfedge h) const {
	return halfedgeDirection(h).lengthSquared();
}

ofxHEMesh::Scalar ofxHEMesh::halfedgeLength(ofxHEMeshHalfedge h) const {
	return halfedgeDirection(h).length();
}

ofxHEMesh::Direction ofxHEMesh::halfedgeDirection(ofxHEMeshHalfedge h) const {
	return vertexPoint(halfedgeSink(h)) - vertexPoint(halfedgeSource(h));
}

ofxHEMesh::Scalar ofxHEMesh::angleAtVertex(ofxHEMeshHalfedge h) const {
	Direction v1 = -halfedgeDirection(h);
	Direction v2 = halfedgeDirection(halfedgeNext(h));
	v1.normalize();
	v2.normalize();
	Scalar dot = MAX(MIN(1, v1.dot(v2)), -1);
	Scalar res = acos(dot);
	// TODO: check for NaNs
	return res;
}

ofxHEMesh::Scalar ofxHEMesh::halfedgeAngle(ofxHEMeshHalfedge h) const {
	ofxHEMesh::Scalar s;
	return s;
}

ofxHEMesh::Direction ofxHEMesh::halfedgeRotated(ofxHEMeshHalfedge h) const {
	Direction n = faceNormal(halfedgeFace(h));
	Direction dir = halfedgeDirection(h);
	return n.cross(dir);
}

// assume symmetrix matrix with (3, 3) equal to 1
ofxHEMesh::Scalar quadraticError(const ofMatrix4x4& Q, const ofxHEMesh::Point& pos) {
	ofxHEMesh::Scalar x = pos.x;
	ofxHEMesh::Scalar y = pos.y;
	ofxHEMesh::Scalar z = pos.z;
	return
		Q(0, 0)*x*x + 2*Q(0, 1)*x*y + 2*Q(0, 2)*x*z + 2*Q(0, 3)*x +
						Q(1, 1)*y*y + 2*Q(1, 2)*y*z + 2*Q(1, 3)*y +
										Q(2, 2)*z*z + 2*Q(2, 3)*z +
														Q(3, 3);
}

ofxHEMesh::Point ofxHEMesh::halfedgeQuadraticFit(ofxHEMeshHalfedge h) const {
	ofMatrix4x4 Q1(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
	ofMatrix4x4 Q2(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
	ofxHEMeshVertex v1 = halfedgeSource(h);
	ofxHEMeshVertex v2 = halfedgeVertex(h);
	vertexQuadric(v1, Q1);
	vertexQuadric(v2, Q2);
	ofMatrix4x4 Q(
		Q1(0, 0)+Q2(0, 0), Q1(0, 1)+Q2(0, 1), Q1(0, 2)+Q2(0, 2), Q1(0, 3)+Q2(0, 3),
		Q1(1, 0)+Q2(1, 0), Q1(1, 1)+Q2(1, 1), Q1(1, 2)+Q2(1, 2), Q1(1, 3)+Q2(1, 3),
		Q1(2, 0)+Q2(2, 0), Q1(2, 1)+Q2(2, 1), Q1(2, 2)+Q2(2, 2), Q1(2, 3)+Q2(2, 3),
		Q1(3, 0)+Q2(3, 0), Q1(3, 1)+Q2(3, 1), Q1(3, 2)+Q2(3, 2), Q1(3, 3)+Q2(3, 3)
	);
	
	ofMatrix3x3 Qsub(
		Q(0, 0), Q(0, 1), Q(0, 2),
		Q(1, 0), Q(1, 1), Q(1, 2),
		Q(2, 0), Q(2, 1), Q(2, 2)
	);
	Scalar det = Qsub.determinant();
	
	if(ABS(det) <= 1e-8) {
		Point pos1 = vertexPoint(v1);
		Point pos2 = vertexPoint(v2);
		Point mid = halfedgeMidpoint(h);
		Scalar err1 = quadraticError(Q, pos1);
		Scalar err2 = quadraticError(Q, pos2);
		Scalar errmid = quadraticError(Q, mid);
		
		if(err1 < err2) {
			if(err1 < errmid) return pos1;
			else return mid;
		}
		else {
			if(err2 < errmid) return pos2;
			else return mid;
		}
	}
	
	ofMatrix3x3 Qa(
		Q(0, 1), Q(0, 2), Q(0, 3),
		Q(1, 1), Q(1, 2), Q(1, 3),
		Q(2, 1), Q(2, 2), Q(2, 3)
	);
	ofMatrix3x3 Qb(
		Q(0, 0), Q(0, 2), Q(0, 3),
		Q(1, 0), Q(1, 2), Q(1, 3),
		Q(2, 0), Q(2, 2), Q(2, 3)
	);
	ofMatrix3x3 Qc(
		Q(0, 0), Q(0, 1), Q(0, 3),
		Q(1, 0), Q(1, 1), Q(1, 3),
		Q(2, 0), Q(2, 1), Q(2, 3)
	);
	Point res = Point(
		Qa.determinant(),
		-Qb.determinant(),
		Qc.determinant()
	)*(-1./det);
	
	Point pos1 = vertexPoint(v1);
	Point pos2 = vertexPoint(v2);
	Point mid = halfedgeMidpoint(h);
	Scalar l1 = (pos1-pos2).length();
	Scalar l2 = (res-pos1).length();
	Scalar l3 = (res-pos2).length();
	if(l2 > l1 || l3 > l1) {
		Scalar err1 = quadraticError(Q, pos1);
		Scalar err2 = quadraticError(Q, pos2);
		Scalar errmid = quadraticError(Q, mid);
		
		if(err1 < err2) {
			if(err1 < errmid) return pos1;
			else return mid;
		}
		else {
			if(err2 < errmid) return pos2;
			else return mid;
		}
	}
	
	return res;
}

ofxHEMesh::Direction ofxHEMesh::triangleNormal(const ofxHEMeshTriangle& tri) const {
	Direction dir1 = vertexPoint(tri.v2)-vertexPoint(tri.v1);
	Direction dir2 = vertexPoint(tri.v3)-vertexPoint(tri.v1);
	return dir1.crossed(dir2).getNormalized();
}

bool ofxHEMesh::withinTriangle(const ofxHEMeshTriangle& tri, const Point& pt) const {
	Point p1 = vertexPoint(tri.v1);
	Point p2 = vertexPoint(tri.v2);
	Point p3 = vertexPoint(tri.v3);
	Direction n = triangleNormal(tri);
	
	Direction A1 = p1-p3;
	Direction A2 = pt-p1;
	Direction A = A1.crossed(A2);
	if(A.dot(n) < 0) return false;
	
	Direction B1 = p2-p1;
	Direction B2 = pt-p2;
	Direction B = B1.crossed(B2);
	if(B.dot(n) < 0) return false;
	
	Direction C1 = p3-p2;
	Direction C2 = pt-p3;
	Direction C = C1.crossed(C2);
	return C.dot(n) >= 0;
}

void ofxHEMesh::clearVertices() {
	vertexProperties.clear();
}

void ofxHEMesh::clearHalfedges() {
	halfedgeProperties.clear();
}

void ofxHEMesh::clearFaces() {
	faceProperties.clear();
}

string ofxHEMesh::halfedgeString(ofxHEMeshHalfedge h) const {
	std::stringstream ss;
	ss << halfedgeSource(h).idx << "-" << halfedgeSink(h).idx;
	return ss.str();
}

void ofxHEMesh::printFace(ofxHEMeshFace f) const {
	ofxHEMeshFaceCirculator fc = faceCirculate(f);
	ofxHEMeshFaceCirculator fce = fc;
	do {
		std::cout << halfedgeVertex(*fc).idx << "-";
		++fc;
	} while(fc != fce);
	std::cout << "\n";
}

void ofxHEMesh::printVertexOneHood(ofxHEMeshVertex v) const {
	ofxHEMeshVertexCirculator vc = vertexCirculate(v);
	ofxHEMeshVertexCirculator vce = vc;
	do {
		std::cout << halfedgeSource(*vc).idx << "-";
		++vc;
	} while(vc != vce);
	std::cout << "\n";
}

void ofxHEMesh::printVertexOneHoodFaces(ofxHEMeshVertex v) const {
	ofxHEMeshVertexCirculator vc = vertexCirculate(v);
	ofxHEMeshVertexCirculator vce = vc;
	std::cout << "v: " << v.idx << "\n";
	do {
		std::cout << "\t";
		printFace(halfedgeFace(*vc));
		++vc;
	} while(vc != vce);
	std::cout << "\n";
}

void ofxHEMesh::print() const {
	ofxHEMeshFaceIterator fit = facesBegin();
	ofxHEMeshFaceIterator fite = facesEnd();
	for(; fit != fite; ++fit) {
		std::cout << (*fit).idx << " ";
		printFace(*fit);
	}
}

bool ofxHEMesh::verifyConnectivity() const {
	bool res = true;
	for(int i=0; i < halfedgeAdjacency->size(); ++i) {
		ofxHEMeshHalfedge h(i);
		if(halfedgeVertex(h).isValid()) {
			if(halfedgeNext(halfedgePrev(h)) != h) {
				std::cout << h.idx << "(" << halfedgeString(h) << ") is not the next of its prev " <<
						halfedgePrev(h).idx << "(" << halfedgeString(halfedgePrev(h)) << ")\n";
				res = false;
			}
			if(halfedgePrev(halfedgeNext(h)) != h) {
				std::cout << h.idx << "(" << halfedgeString(h) << ") is not the prev of its next " <<
						halfedgeNext(h).idx << "(" << halfedgeString(halfedgeNext(h)) << ")\n";
				res = false;
			}
			if(!halfedgeIsInFace(halfedgeFace(h), h)) {
				std::cout << h.idx << "(" << halfedgeString(h) << ") is not in face " << halfedgeFace(h).idx << "\n";
				res = false;
			}
			if(!halfedgeLinksToVertex(halfedgeVertex(h), h)) {
				std::cout << h.idx << "(" << halfedgeString(h) << ") is not connected to vertex " << halfedgeVertex(h).idx << "\n";
				res = false;
			}
		}
	}
	return res;
}