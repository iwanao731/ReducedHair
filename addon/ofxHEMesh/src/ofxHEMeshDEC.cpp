#include "ofxHEMeshDEC.h"

namespace hemesh {

void hodgeStar0Form(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& star0) {
	int n = hemesh.getNumVertices();
	star0.resize(n, n);
	star0.reserve(n);
	
	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		ofxHEMeshVertex v = *vit;
		star0.insert(v.idx, v.idx) = hemesh.vertexArea(v);
	}
}

void hodgeStar1Form(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& star1) {
	int n = hemesh.getNumEdges();
	star1.resize(n, n);
	star1.reserve(n);
	
	ofxHEMeshEdgeIterator eit = hemesh.edgesBegin();
	ofxHEMeshEdgeIterator eite = hemesh.edgesEnd();
	for(; eit != eite; ++eit) {
		ofxHEMeshHalfedge h = *eit;
		ofxHEMeshHalfedge ho = hemesh.halfedgeOpposite(h);
		ofxHEMesh::Scalar cotAlpha = hemesh.halfedgeCotan(h);
		ofxHEMesh::Scalar cotBeta  = hemesh.halfedgeCotan(ho);
		int eidx = h.idx/2;
		star1.insert(eidx, eidx) = (cotAlpha + cotBeta)*0.5;
	}
}

typedef Eigen::Triplet<double> Tripletd;

void exteriorDerivative0Form(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& d0) {
	int nV = hemesh.getNumVertices();
	int nE = hemesh.getNumEdges();
	d0.resize(nE, nV);
	
	//d0.reserve(nE*2);
	//d0.reserve(Eigen::VectorXi::Constant(nV, 4));
	vector<Tripletd> entries;
	entries.reserve(nE*2);
	
	ofxHEMeshEdgeIterator eit = hemesh.edgesBegin();
	ofxHEMeshEdgeIterator eite = hemesh.edgesEnd();
	for(; eit != eite; ++eit) {
		ofxHEMeshHalfedge h = *eit;
		ofxHEMeshVertex v1 = hemesh.halfedgeSink(h);
		ofxHEMeshVertex v2 = hemesh.halfedgeSource(h);
		int eidx = h.idx/2;
		entries.push_back(Tripletd(eidx, v1.idx, 1));
		entries.push_back(Tripletd(eidx, v2.idx, -1));
	}
	d0.setFromTriplets(entries.begin(), entries.end());
}

void laplacian(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& L) {
	Eigen::SparseMatrix<double> d0;
	Eigen::SparseMatrix<double> star1;
	exteriorDerivative0Form(hemesh, d0);
	hodgeStar1Form(hemesh, star1);
	L = d0.transpose()*star1*d0;
}



MeanCurvatureNormals::MeanCurvatureNormals(ofxHEMesh& hemesh)
:	hemesh(hemesh)
{}

void MeanCurvatureNormals::build() {
	laplacian(hemesh, L);
	getPositions();
	normals = L*positions;
}

void MeanCurvatureNormals::getNormals(vector<ofxHEMesh::Direction>& normals) {
	normals.resize(hemesh.getNumVertices());
	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		normals[(*vit).idx] = getNormal(*vit);
	}
}

ofxHEMesh::Direction MeanCurvatureNormals::getNormal(ofxHEMeshVertex v) {
	return ofxHEMesh::Direction(normals(v.idx, 0), normals(v.idx, 1), normals(v.idx, 2));
}

void MeanCurvatureNormals::getPositions() {
	positions.resize(L.rows(), 3);
	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		ofxHEMeshVertex v = *vit;
		ofxHEMesh::Point pos = hemesh.vertexPoint(v);
		positions(v.idx, 0) = (double)pos[0];
		positions(v.idx, 1) = (double)pos[1];
		positions(v.idx, 2) = (double)pos[2];
	}
}


MeanCurvatureFlow::MeanCurvatureFlow(ofxHEMesh& hemesh)
: hemesh(hemesh)
{}
		
void MeanCurvatureFlow::step(double amt) {
	laplacian(hemesh, L);
	hodgeStar0Form(hemesh, star0);
	getPositions();
	
	Eigen::SparseMatrix<double> A = star0 + amt*L;
	Eigen::Matrix<double, Eigen::Dynamic, 3> rhs = star0 * positions;
	
	Eigen::SimplicialLDLT< Eigen::SparseMatrix<double> > solver;
	solver.compute(A);
	if(solver.info()!=Eigen::Success) {
		// decomposition failed
		//return false;
	}
	
	Eigen::Matrix<double, Eigen::Dynamic, 3> newPositions = solver.solve(rhs);
	if(solver.info()!=Eigen::Success) {
		// solving failed
		//return false;
	}
	
	setPositions(newPositions);
}

void MeanCurvatureFlow::getPositions() {
	positions.resize(L.rows(), 3);
	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		ofxHEMeshVertex v = *vit;
		ofxHEMesh::Point pos = hemesh.vertexPoint(v);
		positions(v.idx, 0) = (double)pos[0];
		positions(v.idx, 1) = (double)pos[1];
		positions(v.idx, 2) = (double)pos[2];
	}
}

void MeanCurvatureFlow::setPositions(Eigen::Matrix<double, Eigen::Dynamic, 3> &newPositions) {
	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		ofxHEMeshVertex v = *vit;
		hemesh.vertexMoveTo(v, ofxHEMesh::Point(
			newPositions(v.idx, 0),
			newPositions(v.idx, 1),
			newPositions(v.idx, 2)
		));
	}
}

Geodesics::Geodesics(ofxHEMesh& hemesh)
: hemesh(hemesh)
{}

void Geodesics::build(vector<ofxHEMeshVertex>& impulseLocations, double dt, vector<ofxHEMesh::Scalar>& distances) {
	hodgeStar0Form(hemesh, star0);
	hodgeStar1Form(hemesh, star1);
	exteriorDerivative0Form(hemesh, d0);
	L = d0.transpose()*star1*d0;
	L += (1.0e-8)*star0;
	
	 // heat flow for short interval
	ofxHEMesh::Scalar meanEdgeLength = hemesh.meanEdgeLength();
	dt *= (meanEdgeLength*meanEdgeLength);
	
	Eigen::SparseMatrix<double> A = star0 + dt*L;
	buildImpulseSignal(impulseLocations);
	
	Eigen::SimplicialLDLT< Eigen::SparseMatrix<double> > solver;
	solver.compute(A);
	if(solver.info()!=Eigen::Success) {
		// decomposition failed
		//return false;
	}
	Eigen::Matrix<double, Eigen::Dynamic, 1> u = solver.solve(u0);
	if(solver.info()!=Eigen::Success) {
		// solving failed
		//return false;
	}
	
	
	
	// extract geodesic
	vector<ofxHEMesh::Direction> vectorField(hemesh.getNumFaces());
	computeVectorField(u, vectorField);
	
	Eigen::Matrix<double, Eigen::Dynamic, 1> div;
	computeDivergence(div, vectorField);
	
	
	
	Eigen::SimplicialLDLT< Eigen::SparseMatrix<double> > solver2;
	solver2.compute(L);
	if(solver2.info()!=Eigen::Success) {
		// decomposition failed
		//return false;
	}
	Eigen::Matrix<double, Eigen::Dynamic, 1> phi = solver2.solve(div);
	if(solver2.info()!=Eigen::Success) {
		// solving failed
		//return false;
	}
	
	double minPhi = phi.minCoeff();
	distances.resize(hemesh.getNumVertices());
	for(int i=0; i < phi.rows(); ++i) {
		distances[i] = phi(i)-minPhi;
	}
}

int Geodesics::buildImpulseSignal(vector<ofxHEMeshVertex>& impulseLocations) {
	u0.resize(hemesh.getNumVertices(), 1);
	u0.setZero();
	for(int i=0; i < impulseLocations.size(); ++i) {
		u0(impulseLocations[i].idx) = 1;
	}
	return impulseLocations.size();
}

void Geodesics::computeVectorField(Eigen::Matrix<double, Eigen::Dynamic, 1>& u, vector<ofxHEMesh::Direction>& vectorField) {
	ofxHEMeshFaceIterator fit = hemesh.facesBegin();
	ofxHEMeshFaceIterator fite = hemesh.facesEnd();
	for(; fit != fite; ++fit) {
		// if( f->isBoundary() ) continue;
		
		ofxHEMeshHalfedge hij = hemesh.faceHalfedge(*fit);
		ofxHEMeshHalfedge hjk = hemesh.halfedgeNext(hij);
		ofxHEMeshHalfedge hki = hemesh.halfedgeNext(hjk);
		
		ofxHEMeshVertex vi = hemesh.halfedgeVertex(hki);
		ofxHEMeshVertex vj = hemesh.halfedgeVertex(hij);
		ofxHEMeshVertex vk = hemesh.halfedgeVertex(hjk);
		
		double ui = u(vi.idx);
		double uj = u(vj.idx);
		double uk = u(vk.idx);
		
		ofxHEMesh::Direction eij90 = hemesh.halfedgeRotated(hij);
		ofxHEMesh::Direction ejk90 = hemesh.halfedgeRotated(hjk);
		ofxHEMesh::Direction eki90 = hemesh.halfedgeRotated(hki);
		
		// Precision issues with floats
		double fA = hemesh.faceArea(*fit);
		double X = 0.5*(ui*double(ejk90.x) + uj*double(eki90.x) + uk*double(eij90.x))/fA;
		double Y = 0.5*(ui*double(ejk90.y) + uj*double(eki90.y) + uk*double(eij90.y))/fA;
		double Z = 0.5*(ui*double(ejk90.z) + uj*double(eki90.z) + uk*double(eij90.z))/fA;

		double len = sqrt(X*X + Y*Y + Z*Z);
		X /= -len;
		Y /= -len;
		Z /= -len;
		vectorField[(*fit).idx] = ofVec3f(X, Y, Z);
	}
}

void Geodesics::computeDivergence(Eigen::Matrix<double, Eigen::Dynamic, 1>& div, vector<ofxHEMesh::Direction>& vectorField) {
	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	div.resize(hemesh.getNumVertices());
	for(; vit != vite; ++vit) {
		double sum = 0;
		ofxHEMeshVertexCirculator vc = hemesh.vertexCirculate(*vit);
		ofxHEMeshVertexCirculator vce = vc;
		do {
			// TODO: check boundary
			ofxHEMesh::Direction n = hemesh.halfedgeRotated(hemesh.halfedgePrev(*vc));
			ofxHEMesh::Direction vv = vectorField[hemesh.halfedgeFace(*vc).idx];
			sum += n.dot(vv);			
			++vc;
		} while(vc != vce);
		
		div((*vit).idx) = sum;
	}
}

} // hemesh::