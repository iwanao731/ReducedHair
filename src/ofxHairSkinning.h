#ifndef _OFX_HAIR_SKINNING_
#define _OFX_HAIR_SKINNING_

#include "ofMain.h"
#include "ofxHairModel.h"
#include "ofxHairJoint.h"
#include "ofxHairSkeleton.h"
#include "ofxHairGraph.h"
#include "ofxHairUtil.h"
#include "Eigen/Dense"

// C++11
#include <thread>
#include <atomic>

struct weightInfo {
	int index;
	vector<int> guideIndices;
	vector<float> weight;
};

class ofxHairSkinning {

public:
	ofxHairSkinning();

	void setSkinPoints2(ofxHairModel model);
	void calcSkinningWeight2(ofxHairModel model);
	void deformation2(ofxHairModel model);

	// pre-process
	void initializeTransform(); // new feature
	void buildSkeletonModel(ofxHairModel &model);
	void setSkinPoints(vector<ofPoint> &points);
	void init();

	// real-time process
	void updateTransform(ofxHairModel model); // new featrue
	void transformation(ofxHairModel &model, ofMatrix4x4 transform); // new feature
	void updateSkeletonModel(ofxHairModel &model);
	void updatePosition(ofxHairModel& model);
	void deformation();
	void deformation(ofxHairModel& model);

	void draw(int skeltonIdx) { m_skeltons[skeltonIdx].drawAxis(); }
	void draw();
	void drawSkinningWeight(int indexJoint, ofxHairModel model);

	// util
	float getWeight(const int skin_idx, int joint_idx);
	ofColor getWeightColor(float value, float errMin, float errMax);
	int getNumSkeletons() { return m_numSkeletons; }

	// skinning weight
	void setGraphInfo(const ofxHairGraph graph) { m_graph = graph; }
	void calcSkinningWeight();

	void openGuideMatrixInfo(int frameNum);
	void writeGuideMatrixInfo();
	void closeGuideMatrixInfo();

	void openNormalParticleInfo(ofxHairModel model, int frameNum);
	void writeNormalParticleInfo(ofxHairModel model);
	void closeNormalParticleInfo();

	bool loadSkinningWeight();
	void debugSetWeightColor(ofxHairModel& model);

	ofstream ofsMat;
	ofstream ofsNormal;

private:
	int m_numSkeletons;
	int m_numJointPoints;
	int m_numSkinPoints;
	float **m_weight; // [skin][joint]

	vector<ofPoint> m_skin_initial_points;
	vector<ofPoint> m_skin_points;
	vector<ofxHairSkeleton> m_skeltons;
	map<int, int> mapIdxNode2Joint;  // between particle index and joint index

	// skinning weight
	ofxHairGraph m_graph;
	vector<weightInfo> m_weightInfo;

	// function
	float calcWeightEuclidianDistance(const ofPoint input, const ofPoint p0, const ofPoint p1, const int c = -16);
	void normalizeWeight();
};

#endif