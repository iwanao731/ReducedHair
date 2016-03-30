#ifndef _OFX_HAIR_GUIDE_SELECTION_
#define _OFX_HAIR_GUIDE_SELECTION_

#include "ofMain.h"
#include "ofxHairModel.h"
#include "ofxHairSkeleton.h"
#include "ofxNearestNeighbour.h"
#include "ofxMeshUtil.h"
#include <Eigen/Sparse>

class ofxHairGuideSelection
{
public:

	// construct graph
	void initkNNGraph(vector<ofPoint*> points);
	void updatekNNGraph(vector<ofPoint*> points, float radius);
	void buildkNNGraph(int num_frames, float epsiron);
	bool exportGraphFile(string filename);
	bool exportWeightFile(string filename);

	// construct group
	void buildKCutGroup(int numCut);
	void importGroupFile(const string filename);

	// compute weight
	void accumulateWeight(vector<ofxHairSkeleton> skeleton, bool debug=false);
	void calcWeight();
	float calculateSimilarity(ofxHairSkeleton s1, ofxHairSkeleton s2);

	// debug draw function
	void debugDraw_kNN(int particle_index, vector<ofxHairSkeleton> skeleton, int strand_resolution);
	void debugDraw_Group(vector<ofxHairSkeleton> skeleton, int strand_resolution);
	void debugSetGroupColor(ofxHairModel &model, int groupNum);

private:
	Eigen::SparseMatrix<int> m_counts;	// 1. choosing edge <- init and update
	Eigen::SparseMatrix<int> m_weight;	// 2. calculate weight <- build

	typedef Eigen::Triplet<double> T;
	std::vector<T> m_edgeGraph;
	std::vector<T> m_edgeWeight;
	std::vector<int> groupIndex;

	int edgeNum;

	float calculateSimilarityJoint(ofxHairJoint init_j1, ofxHairJoint curent_ji, ofxHairJoint init_j2, ofxHairJoint current_j2);
	float frobeniusNorm(ofMatrix4x4 mat);
};


#endif