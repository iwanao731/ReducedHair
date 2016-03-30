#pragma once

#include "ofMain.h"
#include "ofxUI.h"
#include "ofxHEMesh.h"
#include "ofxHEMeshDraw.h"
#include "ofxHairModel.h"
#include "ofxHairSim.h"
#include "ofxHairSkeleton.h"
#include "ofxHairSkinning.h"
#include "ofxHairGuideSelection.h"
#include "ofxHairWeightOptimization.h"
#include "ofxHairGraph.h"
#include "ofxMeshUtil.h"

// C++11
#include <thread>
#include <atomic>

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void reset();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
		ofxUISuperCanvas *gui1, *gui2, *gui3, *gui4;
		void setGUI1();
		void setGUI2();
		void setGUI3();
		void setGUI4();
		void guiEvent(ofxUIEventArgs &e);
		void processOpenFileSelection(ofFileDialogResult openFileResult);
		string originalFileExtension;

		ofxHairModel hairModel, initModel;
		ofxHairSim sim;
		ofxHairSkinning skinning;
		vector<ofxHairSkeleton> skeleton;
		ofxHairGuideSelection guide;
		ofxHairGraph graph;

		// adaptive guide hair
		vector<int> adaptive_guide_indices;

		ofxHEMesh hem, initHem;
		ofEasyCam cam;

		// load graph
		void loadGraph(ofxHairGraph& graph);
		void hairFitting();
};
