#include "ofApp.h"
#include "ofxHairDraw.h"

// important factor
float searchRadius = 6.0f; // 4.5
int resolution = 25;
int frameNum = 500;
int cutGroupNum = 600; // 800

// simulation parameter
float supportRadius = 7.0f;
float timestep = 0.3f;
float damping = 0.7;
float powergage = 0.0f;

// count
int frameCount = 0;
int skinningCount = 0;
int numAdapHair = 20;

// bool
bool SkinningMethod = false;
bool bDrawMesh = true;
bool bRotation = false;
bool bPlay = false;
bool bSimilarity = false;
bool bHairColor = true;
bool bConstGroup = false;
bool bTrackLinkPosition = false;
bool bLoadGraph = false;
bool bFitHeadAndHair = false;
bool bDeformation = false;
bool bSelectGuideHair = false;
bool bDrawGuide = true;
bool bDrawSkinWeight = false;
bool bDrawNormal = true;
bool bSkinning = false;
bool bTrackingLinkPosition = false;

//--------------------------------------------------------------
void ofApp::setup(){

	string hairname = "../../../../apps/sharedData/hair/reducedhair_models/exported_text_straight.hair";
	string filename = "../../../../../apps/sharedData/model/Infinite_Scan_Ver0.1/Infinite-Level_02_.obj";

	hairModel.loadHairModelAsText(hairname);
	hem.loadOBJModel(filename);
	initHem= hem;
	sim.loadBoundaryOBJ(filename);

	setGUI1();
	setGUI2();
	setGUI3();
	setGUI4();
	sim.setTimeStep(timestep);
	sim.setDamping(damping);
	sim.setSupportRadius(supportRadius);

	// calculation of similarity
	for(auto s : hairModel.strands){
		ofxHairSkeleton skel;
		skel.loadHairStrand(s);
		skel.init();
		skeleton.push_back(skel);
	}
}

//--------------------------------------------------------------
void ofApp::update(){

	sim.setTimeStep(timestep);
	sim.setDamping(damping);
	sim.setSupportRadius(supportRadius);

	if(bPlay){
		ofMatrix4x4 mat; mat.isIdentity();
		
		if(bRotation){
			cam.getGlobalTransformMatrix();
			//mat.rotateRad(sin(frameCount*7.5f), 0, 1, 0);
			mat.rotateRad(sin(ofGetElapsedTimef())*0.75f, 1, 0, 0);
		}

		ofxHEMeshVertexIterator vit = hem.verticesBegin();
		ofxHEMeshVertexIterator vite = hem.verticesEnd();

		int i=0;
		vector<ofPoint> points;
		for( ; vit != vite; vit++, i++ ){
			ofVec3f pos = mat * initHem.vertexPoint(vit.v);
			points.push_back(pos);
			hem.vertexMoveTo(vit.v, pos);
		}

		sim.updateBoundary(points);
		sim.update(mat);

		// calculate nearest neighbor particle in radius and build edge relationship
		if(bSimilarity)
		{
			guide.updatekNNGraph(sim.getParticlePosition(), searchRadius);
			frameCount++;

			if(frameCount == frameNum){
				guide.buildkNNGraph(frameNum, 0.2);
				//reset();
				initModel = hairModel;
				bSimilarity = false;
				bPlay = false;
				frameCount = 0;
				//bHairColor = true;
			}
		}else if(bConstGroup){
			for(int i=0; i<hairModel.getNumStrand(); i++){
				skeleton[i].buildJointHierarchy();
				skeleton[i].buildJointMatrix2(hairModel.strands[i]);
			}

			bool printValue = false;
			guide.accumulateWeight(skeleton, printValue);
			frameCount++;

			if(frameCount == frameNum){
				guide.calcWeight();
				guide.exportGraphFile("data/test.graph");
				guide.exportGraphFile("data/test.group");
				guide.exportWeightFile("data/test.weight");
				guide.buildKCutGroup(cutGroupNum);
				guide.importGroupFile("data/test.group");
				//reset();
				guide.debugSetGroupColor(hairModel, cutGroupNum);
				bConstGroup = false;
				bPlay = false;
				frameCount = 0;
			}
		}else if(bTrackLinkPosition){
			skinning.updateSkeletonModel(hairModel);

			// write matrix information at each guide particle
			skinning.writeGuideMatrixInfo();

			// write normal particle infomation
			skinning.writeNormalParticleInfo(hairModel);

			frameCount++;

			if(frameCount == frameNum){
				skinning.closeGuideMatrixInfo();
				skinning.closeNormalParticleInfo();
				frameCount = 0;
				bPlay = false;
				bTrackLinkPosition = false;
				//reset();
			}
		}else if(bDeformation)
		{
			if(SkinningMethod){
				skinning.updateSkeletonModel(hairModel);
				skinning.deformation();
				skinning.updatePosition(hairModel);
			}else{
				//skinning.deformation(hairModel);	
				//skinning.updateTransform(hairModel);

				skinning.updateSkeletonModel(hairModel);
				skinning.transformation(hairModel, mat);
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){

	ofBackground(0);
	ofEnableDepthTest();

	cam.begin();

	if(bDrawMesh){
		ofSetColor(255);
		hem.setTopologyDirty(true);
		ofxHEMeshDraw hedraw(hem);
		hedraw.setDrawEdges(true);
		hedraw.setDrawFaces(true);
		hedraw.draw(cam);
	}
	
	ofxHairDraw viewer(hairModel);
	viewer.setDrawHairColor(bHairColor);
	viewer.setDrawHairParticles(false);
	viewer.setDrawHairEdges(true);
	viewer.setDrawHairNormal(bDrawNormal);
	viewer.setDrawHairGuide(bDrawGuide);
	viewer.draw();

	if(bDrawSkinWeight){
		//skinning.draw();
		//graph.drawLink(hairModel, skinningCount);
		skinning.drawSkinningWeight(skinningCount, hairModel);
	}

	//graph.draw(hairModel);

	cam.end();
	
	if(bPlay){
		powergage = sin(ofGetElapsedTimef())*ofGetWidth();
	}

	ofSetColor(25.0, 25.0, 200.0, 255);
	ofRect(0.0, ofGetHeight()-30, 0.0, (float)ofGetWidth() * (float)frameCount/(float)frameNum, 30);

	ofSetColor(255);

	ofDisableDepthTest();

	ofSetWindowTitle(ofToString(ofGetFrameRate()));

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch (key)
	{
		case 'f':
			ofToggleFullscreen();
			break;
		case 'r':
		{
			reset();
			break;
		}
		case ' ':
			bPlay = !bPlay;
			break;
		case 'g':
		{
			gui1->toggleVisible();
			gui2->toggleVisible();
			gui3->toggleVisible();
			gui4->toggleVisible();
			break;
		}
		case '+':
			skinningCount++;
			break;
		case '-':
			skinningCount--;
			break;
	default:
		break;
	}
}

//--------------------------------------------------------------
void ofApp::reset()
{
	hairModel = initModel;
	hem = initHem;
	vector<ofPoint> points;
	ofxHEMeshVertexIterator vit = hem.verticesBegin();
	ofxHEMeshVertexIterator vite = hem.verticesEnd();
	for( ; vit != vite; vit++ ){
		ofVec3f pos = hem.vertexPoint(vit.v);
		points.push_back(pos);
	}
	sim.updateBoundary(points);

	if(bSkinning){
		skinning.updateSkeletonModel(hairModel);
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

void ofApp::processOpenFileSelection(ofFileDialogResult openFileResult){
	
	ofLogVerbose("getName(): "  + openFileResult.getName());
	ofLogVerbose("getPath(): "  + openFileResult.getPath());
	
	ofFile file (openFileResult.getPath()); 
	
	if (file.exists()){
		//Limiting this example to one image so we delete previous ones
	
		ofLogVerbose("The file exists - now checking the type via file extension");
		string fileExtension = ofToUpper(file.getExtension());
		
		//We only want images
		if (fileExtension == "OBJ") {
			
			hem.clearFaces();
			hem.clearVertices();
			hem.clearHalfedges();

			//Save the file extension to use when we save out
			originalFileExtension = fileExtension;
			
			//Load the selected image
			if(!hem.loadOBJModel(openFileResult.getPath())){
				std::cout << "couldn't load file..." << std::endl;	
			}

			sim.loadBoundaryOBJ(openFileResult.getPath());

		}else if(fileExtension == "HAIR") {

			ofxHairModel hair;
			if(hair.loadHairModelAsText(openFileResult.getPath())){
				hairModel = hair;
				initModel = hairModel;
				cout << "done" << endl;
			}
		}
	}	
}


void ofApp::setGUI1()
{
	gui1 = new ofxUISuperCanvas("1: BASICS");
   	gui1->setWidgetFontSize(OFX_UI_FONT_MEDIUM);

    gui1->addSpacer();
    gui1->addLabelToggle("Load OBJ (.obj)", false);

	gui1->addSpacer();
    gui1->addLabelToggle("Load Hair (.hair)", false);

    gui1->addSpacer();
	gui1->addLabelToggle("Fit obj and hair", false);

    gui1->addSpacer();
	gui1->addSlider("supportRadius", 6.0f, 15.0f, supportRadius);
	gui1->addSlider("timestep", 0.0f, 1.4f, timestep);
	gui1->addSlider("damping", 0.0f, 1.0f, damping);

    gui1->addSpacer();
	gui1->addLabelToggle("Simulation", false);

    gui1->addSpacer();
	gui1->addToggle("Rotation", bRotation);
	gui1->addToggle("Draw Guide Hair", bDrawGuide);
	gui1->addToggle("Draw Normal Hair", bDrawNormal);
	gui1->addToggle("Draw Hair Color", bHairColor);
	gui1->addToggle("Draw OBJ Model", bDrawMesh);

    gui1->setPosition(0, 0);
	gui1->autoSizeToFitWidgets();
	ofAddListener(gui1->newGUIEvent, this, &ofApp::guiEvent);
}

void ofApp::setGUI2()
{
    gui2 = new ofxUISuperCanvas("2: SIMILARITY");
    
	gui2->setWidgetFontSize(OFX_UI_FONT_MEDIUM);

    gui2->addLabel("K NEAREST NEIGHTBOR", OFX_UI_FONT_SMALL);
	gui2->addSlider("RADIUS", 0.0f, 10.0f, searchRadius);

	gui2->addSpacer();
    gui2->addLabel("INPUT NUM OF FRAMES", OFX_UI_FONT_SMALL);
	gui2->addTextInput("INPUT NUM OF FRAMES", ofToString(frameNum))->setAutoClear(false);

    gui2->addSpacer();
    gui2->addLabel("CALCULATE SIMILARITY", OFX_UI_FONT_SMALL);
    gui2->addLabelToggle("Construct Graph", bSimilarity);

    gui2->addSpacer();
    gui2->addLabel("CALCULATE ERROR", OFX_UI_FONT_SMALL);
	gui2->addLabelToggle("Construct Group", false);
	gui2->addSlider("Num of Cut Group", 0, 1000, cutGroupNum);
    
    gui2->addSpacer();
    gui2->addLabel("OPTIMIZE ERROR", OFX_UI_FONT_SMALL);
	gui2->addLabelToggle("Select Guide Hair", false);

    gui2->addSpacer();
    gui2->addLabel("SORTED GUIDE HAIR", OFX_UI_FONT_SMALL);
	gui2->addSlider("Adaptive Guide Hair", 0, hairModel.getNumStrand(), numAdapHair);
 
    gui2->setPosition(212, 0);
	gui2->autoSizeToFitWidgets();
	gui2->setWidth(212);
	ofAddListener(gui2->newGUIEvent, this, &ofApp::guiEvent);
}

void ofApp::setGUI3()
{
    gui3 = new ofxUISuperCanvas("3: WEIGHT");
    
	gui3->setWidgetFontSize(OFX_UI_FONT_MEDIUM);

    gui3->addSpacer();
    gui3->addLabel("3.1 PRE PROCESS", OFX_UI_FONT_SMALL);
	gui3->addLabelToggle("Tracking Link Position", bTrackLinkPosition);

	gui3->addSpacer();
    gui3->addLabel("3.2 PRE PROCESS", OFX_UI_FONT_SMALL);
	gui3->addLabelToggle("Optimize Weight", false);

	gui3->addSpacer();
    gui3->addLabel("3.3.1 PRE PROCESS", OFX_UI_FONT_SMALL);
	gui3->addLabelToggle("Load Weight", false);
	

    vector<string> names;
	names.push_back("CALC");
	names.push_back("LOAD");
	gui3->addSpacer();
	gui3->addRadio("RADIO HORIZONTAL", names, OFX_UI_ORIENTATION_HORIZONTAL);

	gui3->addSpacer();
    gui3->addLabel("3.3.2 PRE PROCESS", OFX_UI_FONT_SMALL);
	gui3->addLabelToggle("Calc Weight", false);

	gui3->addSpacer();
	gui3->addToggle("Draw Skin Weight", bDrawSkinWeight);
	gui3->setPosition(212*2, 0);
	gui3->setWidth(212);
    gui3->autoSizeToFitWidgets();
	ofAddListener(gui3->newGUIEvent, this, &ofApp::guiEvent);
}

void ofApp::setGUI4()
{
    gui4 = new ofxUISuperCanvas("4: INTERPOLATE");
    
	gui4->setWidgetFontSize(OFX_UI_FONT_MEDIUM);

    gui4->addSpacer();
    gui4->addLabel("PREPARE", OFX_UI_FONT_SMALL);
	gui4->addLabelToggle("Set Interpolation", bTrackLinkPosition);

    gui4->addSpacer();
    gui4->addLabel("SIMULATE", OFX_UI_FONT_SMALL);
	gui4->addLabelToggle("Deformation", bTrackLinkPosition);

	gui4->setPosition(212*3, 0);
	gui4->setWidth(212);
    gui4->autoSizeToFitWidgets();
	ofAddListener(gui4->newGUIEvent, this, &ofApp::guiEvent);
}

void ofApp::guiEvent(ofxUIEventArgs &e)
{
	string name = e.getName();
	int kind = e.getKind();
	cout << "got event from: " << name << endl;

    if(name == "Load OBJ (.obj)")
    {
		//Open the Open File Dialog
		ofFileDialogResult openFileResult= ofSystemLoadDialog("Select a obj"); 
		
		//Check if the user opened a file
		if (openFileResult.bSuccess){
			
			ofLogVerbose("User selected a file");
			
			//We have a file, check it and process it
			processOpenFileSelection(openFileResult);
			
		}else {
			ofLogVerbose("User hit cancel");
		}
		ofxUIToggle *toggle = (ofxUIToggle *) e.getToggle();
		toggle->setValue(false);
    }
	else if(name == "Load Hair (.hair)")
	{
		//Open the Open File Dialog
		ofFileDialogResult openFileResult= ofSystemLoadDialog("Select a hair"); 
		
		//Check if the user opened a file
		if (openFileResult.bSuccess){
			
			ofLogVerbose("User selected a file");
			
			//We have a file, check it and process it
			processOpenFileSelection(openFileResult);
			
		}else {
			ofLogVerbose("User hit cancel");
		}

		ofxUIToggle *toggle = (ofxUIToggle *) e.getToggle();
		toggle->setValue(false);
	}else if(name == "Fit obj and hair")
	{
		hairFitting();

	}else if(name == "Simulation")
	{
		ofxUIToggle *toggle = (ofxUIToggle *) e.getToggle();
		bPlay = toggle->getValue();
	}else if(name == "Draw Normal Hair")
	{
		bDrawNormal = !bDrawNormal;
	}
	else if(name == "Draw Guide Hair")
	{
		bDrawGuide = !bDrawGuide;
	}else if(name == "Draw Hair Color")
	{
		bHairColor = !bHairColor;
	}else if(name == "Draw OBJ Model")
	{
		bDrawMesh = !bDrawMesh;
	}else if(name == "timestep")
	{
		ofxUISlider *slider = (ofxUISlider *) e.getSlider();
		timestep = slider->getValue();
		sim.setTimeStep(timestep);
	}else if(name == "Rotation")
	{
		bRotation = !bRotation;
	}else if(name == "RADIUS"){
		ofxUISlider *slider = (ofxUISlider *) e.getSlider();
		searchRadius = slider->getValue();
	}else if(name == "Construct Graph")
	{
		ofxUIButton *button = (ofxUIButton *) e.getButton();
		bSimilarity = button->getValue();
		//reset();
		bPlay = true;
	}else if(name == "Similarity frame num")
	{

	}else if(name == "INPUT NUM OF FRAMES")
	{
		cout << "ToDo" << endl;
		cout << "frameNum : " << frameNum << endl;
	}else if(name == "Construct Group")
	{
		bPlay = true;
		bConstGroup = true;
	}else if(name == "Num of Cut Group")
	{
		ofxUISlider *slider = (ofxUISlider *) e.getSlider();
		cutGroupNum = slider->getValue();	
		cout << "Num of Cut Group : " << cutGroupNum << endl;
	}else if(name == "Select Guide Hair")
	{
		if(!bFitHeadAndHair) hairFitting();

		string graphname = "data/test.graph";
		graph.loadGraph(graphname);

		string weightname = "data/test.weight";
		graph.loadWeight(weightname);

		string groupname = "data/test.group";
		graph.loadGroup(groupname);

		if(!bSelectGuideHair){
			graph.calculateMaximumWeight();
			graph.calculateEnergyFunction();

			// export guide hair set
			string guideHairName = "data/test.guide";
			graph.exportGuideHair(guideHairName);
			graph.sortStrandByEnergyValue();
		}else{
			string guideHairName = "data/test.guide";
			graph.loadGuide(guideHairName, resolution);
		}

		// reset();
		graph.debugSetGroupColor(hairModel);
		initModel = hairModel;

		bSelectGuideHair = true;
	}else if(name == "Adaptive Guide Hair")
	{
		ofxUISlider *slider = (ofxUISlider *) e.getSlider();
		numAdapHair = slider->getValue();
		graph.debugGuideHairColor(hairModel, numAdapHair);

	}else if(name == "Tracking Link Position")
	{
		ofxUIButton *button = (ofxUIButton *) e.getButton();
		bTrackLinkPosition = button->getValue();

		if(!bLoadGraph){
			loadGraph(graph);
			bLoadGraph = true;;
		}

		skinning.buildSkeletonModel(hairModel);
		skinning.init();
		skinning.setGraphInfo(graph);		
		skinning.openGuideMatrixInfo(frameNum);
		skinning.openNormalParticleInfo(hairModel, frameNum);
		bPlay = true;
		bTrackingLinkPosition = true;

	}else if(name == "Optimize Weight")
	{
		cout << "calculate skinning weight ..." << endl;

		ofxHairWeightOptimization optimize;

		// load link guide particle indices
		if(!bLoadGraph){
			loadGraph(graph);
			bLoadGraph = true;;
		}
		optimize.setGraph(graph);
		optimize.setExampleNum(frameNum);

		if(optimize.checkOptimization())
		{
			// load normal hair index, initial position, current position 
			string normalfile = "data/test.normalPos";
			optimize.loadNormalParticleInfo(normalfile);

			// load guide hair index and quaternion 
			string filename = "data/test.guidemat";
			optimize.loadGuideMatrix(filename);

			// optimization
			optimize.calculateWeights();

			// set color by loading weight
			cout << "finished" << endl;
		}
	}else if(name == "Load Weight")
	{
		if(!bLoadGraph){
			loadGraph(graph);
			skinning.setGraphInfo(graph);		
			bLoadGraph = true;
		}

		reset();
		skinning.loadSkinningWeight();
		skinning.debugSetWeightColor(hairModel);
		initModel = hairModel;
	}else if(name == "CALC"){
		SkinningMethod = true;
	}else if(name == "LOAD"){
		SkinningMethod = false;
	}else if(name == "Calc Weight")
	{
		if(!bLoadGraph){
			loadGraph(graph);
			skinning.setGraphInfo(graph);		
			bLoadGraph = true;
		}

		//reset();
		if(bTrackingLinkPosition){
			skinning.buildSkeletonModel(hairModel);
			skinning.init();
		}

		// old vertion
		vector<ofVec3f> points;
		int k=0; 
		for(int i=0; i<hairModel.strands.size(); i++){
			for(int j=0; j<hairModel.strands[i].m_particles.size(); j++){
				if(!graph.getHairNode(k).getBoolGuideHair()){
					hairModel.strands[i].m_particles[j].enabled = false;
					points.push_back(hairModel.strands[i].m_particles[j].position);
				}
				k++;
			}
		}

		//sim.init(hairModel);
		initModel = hairModel;

		skinning.buildSkeletonModel(hairModel);
		skinning.setSkinPoints(points);
		skinning.calcSkinningWeight();
		skinning.init();

		//skinning.calcSkinningWeight2(hairModel);

		//skinning.debugSetWeightColor(hairModel);
		initModel = hairModel;

		SkinningMethod = true;

	}else if(name == "Set Interpolation")
	{
		if(!bFitHeadAndHair) hairFitting();

		if(!bLoadGraph){
			loadGraph(graph);
			skinning.setGraphInfo(graph);		
			bLoadGraph = true;;
		}

		//reset();

		skinning.buildSkeletonModel(hairModel);
		//skinning.updateSkeletonModel(hairModel);

		skinning.init();
		skinning.initializeTransform();
		bSkinning = true;
	}else if(name == "Deformation")
	{
		bDeformation = true;
		bPlay = true;
	}else if(name == "Draw Skin Weight"){
		bDrawSkinWeight = !bDrawSkinWeight;

		if(!bLoadGraph){
			loadGraph(graph);
			bLoadGraph = true;;
		}
	}
}

void ofApp::loadGraph(ofxHairGraph& graph)
{
	cout << "graph loading ...";

	ofxHairGraph m_graph;
	string address = "data/";
	string graphname = "test.graph";
	m_graph.loadGraph(address + graphname);

	string weightname = "test.weight";
	m_graph.loadWeight(address + weightname);

	string guidename = "test.guide";
	m_graph.loadGuide(address + guidename, resolution);
	hairModel.loadGuideHair(address + guidename);

	string groupname = "test.group";
	m_graph.loadGroup(address + groupname);
	m_graph.init();
	m_graph.setupGroupLink(); // group
	m_graph.setupGuideLink(); // guide
	m_graph.setupNormalNum();

	graph.debugSetGroupColor(hairModel);

	graph = m_graph;

	cout << "done" << endl;
}

void ofApp::hairFitting()
{
	cout << "Fitting Hair and Model...";
	sim.setCollisionType(CollisionType::ClosestPointOnTriangleKdTree);
	sim.setSupportRadius(supportRadius);
	float radius = 10.0f;
	sim.modelHairFitting(hairModel, radius);
	initModel = hairModel;
	sim.init(hairModel);
	guide.initkNNGraph(sim.getParticlePosition());
	bFitHeadAndHair = true;
	cout << "done..." << endl;
}
