#ifndef _OFX_HAIR_WEIGHT_OPTIMIZATION_
#define _OFX_HAIR_WEIGHT_OPTIMIZATION_

#include "ofMain.h"
#include "ofxHairGraph.h"
#include "ofxHairUtil.h"
#include "Eigen/Dense"
#include "QuadProg++.hh"

// C++11
#include <thread>
#include <atomic>

using namespace QuadProgPP;

struct trackGuideParticle{
	int m_index;
	std::vector<ofMatrix4x4> q_frame;
};

struct normalParticle{
	int index;
	ofVec3f m_initPos;
	vector<ofVec3f> m_currPos;
	vector<trackGuideParticle> guideParticles;
};

class ofxHairWeightOptimization {

public:
	ofxHairWeightOptimization();

	bool loadGuideMatrix(const string filename);
	bool loadNormalParticleInfo(const string filename);
	void setGraph(ofxHairGraph graph) { m_graph = graph; }
	void calculateWeights();
	int getNumNormalParticles(){ m_normalParticles.size(); }
	void setExampleNum(int frameNum) { exampleNum = frameNum; }
	bool checkOptimization();

private:
	ofxHairGraph m_graph;
	vector<trackGuideParticle> m_guideParticle;
	vector<normalParticle> m_normalParticles;
	void optimizeEachParticles(const int index);
	void exportTempFile(const int index, Eigen::MatrixXd am, Eigen::MatrixXd bv);
	bool loadTempFile(const int index, Eigen::MatrixXd& m_am, Eigen::MatrixXd& m_bv, vector<int>& linkIndex);
	void exportSkinWeight(int j, string address, vector<pair<float, int> > weight);
	void reOptimization(int index, std::vector<std::pair<float, int> > &weight);

	int exampleNum;
};

#endif