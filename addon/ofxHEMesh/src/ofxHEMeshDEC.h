#pragma once
#include "ofxHEMesh.h"
#include "Eigen/Sparse"

namespace hemesh {

	void hodgeStar0Form(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& star0);
	void hodgeStar1Form(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& star1);
	void exteriorDerivative0Form(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& d0);
	void laplacian(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& L);

	class MeanCurvatureNormals{
	public:
		MeanCurvatureNormals(ofxHEMesh& hemesh);
		void getNormals(vector<ofxHEMesh::Direction>& normals);
		ofxHEMesh::Direction getNormal(ofxHEMeshVertex v);
		//ofxHEMesh::Scalar getMeanCurvature(ofxHEMeshVertex v);
		
		void build();
		
	protected:
		void getPositions();

		const ofxHEMesh& hemesh;
		Eigen::SparseMatrix<double> L;
		Eigen::Matrix<double, Eigen::Dynamic, 3> positions;
		Eigen::Matrix<double, Eigen::Dynamic, 3> normals;
	};
	
	class MeanCurvatureFlow{
	public:
		MeanCurvatureFlow(ofxHEMesh& hemesh);
		
		void step(double amt);
		
	protected:
		void getPositions();
		void setPositions(Eigen::Matrix<double, Eigen::Dynamic, 3> &newPositions);
	
		ofxHEMesh& hemesh;
		Eigen::SparseMatrix<double> L;
		Eigen::SparseMatrix<double> star0;
		Eigen::Matrix<double, Eigen::Dynamic, 3> positions;
	};
	
	class Geodesics{
	public:
		Geodesics(ofxHEMesh& hemesh);
		
		void build(vector<ofxHEMeshVertex>& impulseLocations, double dt, vector<ofxHEMesh::Scalar>& distances);
		
	protected:
		int buildImpulseSignal(vector<ofxHEMeshVertex>& impulseLocations);
		void computeVectorField(Eigen::Matrix<double, Eigen::Dynamic, 1>& u, vector<ofxHEMesh::Direction>& vectorField);
		void computeDivergence(Eigen::Matrix<double, Eigen::Dynamic, 1>& div, vector<ofxHEMesh::Direction>& vectorField);
	
		ofxHEMesh& hemesh;
		Eigen::Matrix<double, Eigen::Dynamic, 1> u0;
		Eigen::SparseMatrix<double> star0;
		Eigen::SparseMatrix<double> star1;
		Eigen::SparseMatrix<double> d0;
		Eigen::SparseMatrix<double> L;
	};

} // hemesh::