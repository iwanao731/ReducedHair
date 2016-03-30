#include "ofxHairSkinning.h"
#include "ofxMeshUtil.h"

//--------------------------------------------------------------
ofxHairSkinning::ofxHairSkinning()
{
}

//--------------------------------------------------------------
void ofxHairSkinning::init()
{
	for(int i=0; i<m_skeltons.size(); i++){
		for(int j=0; j<m_skeltons[i].getNumJoints(); j++){

			// no effect
			//m_skeltons[i].getJoint0(j).setGrobalMatrix(m_skeltons[i].getJoint(j).getGlobalMatrix());
			//m_skeltons[i].getJoint0(j) = m_skeltons[i].getJoint(j);

			// this is working
			m_skeltons[i].m_joints0 = m_skeltons[i].m_joints;
		}
	}
}

//--------------------------------------------------------------
void ofxHairSkinning::initializeTransform()
{
	for (size_t k = 0; k < m_skeltons.size(); k++) {
		for (size_t i = 0; i < m_skeltons[k].getNumJoints(); i++) {
			if (i == 0) { // root particle.
				ofVec3f v0 = m_skeltons[k].m_joints0[i+1].getPosition() - m_skeltons[k].m_joints0[i].getPosition();
				ofVec3f v1(1.0f, 0.0f, 0.0f);
				ofQuaternion q;
				q.makeRotate(v0, v1);

				// local and global are same
				m_skeltons[k].m_joints0[i].setLocalRotation(q);
				m_skeltons[k].m_joints0[i].setLocalTranslate(m_skeltons[k].m_joints0[i].getPosition());

				m_skeltons[k].m_joints0[i].setGlobalRotation(m_skeltons[k].m_joints0[i].getLocalRotation());
				m_skeltons[k].m_joints0[i].setGlobalTranslate(m_skeltons[k].m_joints0[i].getLocalTranslate());
			}else{
				ofVec3f v_i_minus_1 = m_skeltons[k].m_joints0[i-1].getPosition();
				ofVec3f v_i = m_skeltons[k].m_joints0[i].getPosition();

				ofVec3f vec = v_i - v_i_minus_1;
				vec = m_skeltons[k].m_joints0[i-1].getGlobalRotation() * vec;
				ofVec3f vecX = vec.normalized();
				ofVec3f v1(1.0f, 0.0f, 0.0f);
				ofQuaternion q;
				q.makeRotate(vec, v1);

				m_skeltons[k].m_joints0[i].setLocalTranslate(vec);

				m_skeltons[k].m_joints0[i].setGlobalRotation(m_skeltons[k].getJoint0(i-1).getGlobalRotation() * m_skeltons[k].m_joints0[i].getLocalRotation());
				m_skeltons[k].m_joints0[i].setGlobalTranslate(m_skeltons[k].getJoint0(i-1).getGlobalTranslate() + m_skeltons[k].m_joints0[i].getLocalTranslate());
			}
		}
	}
}

//--------------------------------------------------------------
void ofxHairSkinning::updateTransform(ofxHairModel model)
{
	for (size_t k = 0; k < m_skeltons.size(); k++) {
		for (size_t i = 0; i < m_skeltons[k].getNumJoints(); i++) {
			if (i == 0) { // root particle.
				//m_skeltons[k].m_joints[i+1].getPosition() = model.strands[k].m_particles[i+1].position;
				//m_skeltons[k].m_joints[i].getPosition() = model.strands[k].m_particles[i].position;
				
				ofVec3f v0 = model.strands[k].m_particles[i+1].position - model.strands[k].m_particles[i].position;
				ofVec3f v1(1.0f, 0.0f, 0.0f);
				ofQuaternion q;
				q.makeRotate(v0, v1);

				// local and global are same
				m_skeltons[k].m_joints[i].setLocalRotation(q);
				m_skeltons[k].m_joints[i].setLocalTranslate(model.strands[k].m_particles[i].position);

				m_skeltons[k].m_joints[i].setGlobalRotation(m_skeltons[k].m_joints[i].getLocalRotation());
				m_skeltons[k].m_joints[i].setGlobalTranslate(m_skeltons[k].m_joints[i].getLocalTranslate());
			}else{
				//m_skeltons[k].m_joints[i-1].getPosition() = model.strands[k].m_particles[i-1].position;
				//m_skeltons[k].m_joints[i].getPosition() = model.strands[k].m_particles[i].position;

				ofVec3f v_i_minus_1 = model.strands[k].m_particles[i-1].position;
				ofVec3f v_i = model.strands[k].m_particles[i].position;

				ofVec3f vec = v_i - v_i_minus_1;
				vec = m_skeltons[k].m_joints[i-1].getGlobalRotation() * vec;
				ofVec3f vecX = vec.normalized();
				ofVec3f v1(1.0f, 0.0f, 0.0f);
				ofQuaternion q;
				q.makeRotate(vec, v1);

				m_skeltons[k].m_joints[i].setLocalTranslate(vec);
				m_skeltons[k].m_joints[i].setLocalRotation(q);
				m_skeltons[k].m_joints[i].setGlobalRotation(m_skeltons[k].m_joints[i-1].getGlobalRotation() * m_skeltons[k].m_joints[i].getLocalRotation());
				m_skeltons[k].m_joints[i].setGlobalTranslate(m_skeltons[k].m_joints[i-1].getGlobalTranslate() + m_skeltons[k].m_joints[i].getLocalTranslate());
			}
		}
	}
}

//--------------------------------------------------------------
void ofxHairSkinning::transformation(ofxHairModel &model, ofMatrix4x4 transform)
{
	for(int i=0; i<m_weightInfo.size(); i++)
	{
		ofVec3f pos = ofVec3f::zero();
		int sIdx = m_weightInfo[i].index/25;
		int pIdx = m_weightInfo[i].index%25;

		for(int j=0; j<m_weightInfo[i].guideIndices.size(); j++){
			int s = mapIdxNode2Joint[m_weightInfo[i].guideIndices[j]]/25;
			int p = mapIdxNode2Joint[m_weightInfo[i].guideIndices[j]]%25;

			ofMatrix4x4 mat  = m_skeltons[s].getJoint(p).getGlobalMatrix();
			ofMatrix4x4 mat0 = m_skeltons[s].getJoint0(p).getGlobalMatrix();

			pos  += model.strands[sIdx].m_particles[pIdx].position0 * mat0.getInverse() * mat * m_weightInfo[i].weight[j];
		}
		if(pIdx > 0)
			model.strands[sIdx].m_particles[pIdx].position = pos;
	}
}

//--------------------------------------------------------------
void ofxHairSkinning::deformation()
{
	for(int i=0; i<m_numSkinPoints; i++){
		ofVec3f p = ofVec3f::zero();
		int count = 0;
		for(int k=0; k<m_skeltons.size(); k++){
			for(int j=0; j<m_skeltons[k].getNumJoints(); j++) {
				if(m_weight[i][count]>0.01){
					ofMatrix4x4 mat  = m_skeltons[k].getJoint(j).getGlobalMatrix();
					ofMatrix4x4 mat0 = m_skeltons[k].getJoint0(j).getGlobalMatrix();
					p += m_skin_initial_points[i] * mat0.getInverse() * mat * m_weight[i][count];
				}
				count++;
			}
		}
		m_skin_points[i] = p;
	}
}

//--------------------------------------------------------------
void ofxHairSkinning::deformation(ofxHairModel& model)
{
	for(int i=0; i<m_weightInfo.size(); i++)
	{
		ofVec3f pos = ofVec3f::zero();
		int sIdx = m_weightInfo[i].index/25;
		int pIdx = m_weightInfo[i].index%25;

		if(pIdx > 1){
			for(int j=0; j<m_weightInfo[i].guideIndices.size(); j++){

				int s = mapIdxNode2Joint[m_weightInfo[i].guideIndices[j]]/25;
				int p = mapIdxNode2Joint[m_weightInfo[i].guideIndices[j]]%25;
				ofMatrix4x4 mat  = m_skeltons[s].getJoint(p).getGlobalMatrix();
				ofMatrix4x4 mat0 = m_skeltons[s].getJoint0(p).getGlobalMatrix();
				pos += model.strands[sIdx].m_particles[pIdx].position0 * mat0.getInverse() * mat * m_weightInfo[i].weight[j];
			}
			model.strands[sIdx].m_particles[pIdx].position = pos;
		}
	}
}

//--------------------------------------------------------------
void ofxHairSkinning::buildSkeletonModel(ofxHairModel &model)
{
	std::cout << "build skeleton structure from hair model...";

	m_skeltons.clear();
	m_numJointPoints = 0;

	int count = 0;
	int num = 0;
	for(auto s : model.strands)
	{
		if(s.bGuideHair)
		{
			ofxHairSkeleton skeleton;
			skeleton.loadHairStrand(s);
			m_skeltons.push_back(skeleton);
			m_numJointPoints += s.getResolution();

			// create map from (particle index) to (joint index). -> we also can confirm (strand index) to (skeleton index)
			int res = s.getResolution();
			for(int i=0; i<res; i++){
				mapIdxNode2Joint.insert( map<int,int>::value_type(count*res+i, num*res+i) );
			}
			num++;
		}
		count++;
	}

	m_numSkeletons = m_skeltons.size();

	cout << "done..." << endl;
}

//--------------------------------------------------------------
void ofxHairSkinning::updateSkeletonModel(ofxHairModel &model)
{
	int k=0;
	for(int i=0; i<model.strands.size(); i++){
		if(model.strands[i].bGuideHair){
			m_skeltons[k].buildJointHierarchy();
			m_skeltons[k].buildJointMatrix2(model.strands[i]);
			k++;
		}
	}
}

//--------------------------------------------------------------
void ofxHairSkinning::updatePosition(ofxHairModel& model)
{
	int k = 0;
	int kk=0;
	for(int i=0; i<model.strands.size(); i++){
		for(int j=0; j<model.strands[i].m_particles.size(); j++){
			if(!m_graph.getHairNode(k).getBoolGuideHair()){
				model.strands[i].m_particles[j].position = m_skin_points[kk];
				kk++;
			}
			k++;
		}
	}
}

//--------------------------------------------------------------
void ofxHairSkinning::setSkinPoints(vector<ofPoint> &points)
{
	m_skin_points = points;
	m_skin_initial_points = points;
	m_numSkinPoints = m_skin_points.size();

	m_weight = new float *[m_numSkinPoints];
	for(int i=0; i<m_numSkinPoints; i++){
		m_weight[i] = new float [m_numJointPoints];
	}
}

//--------------------------------------------------------------
void ofxHairSkinning::setSkinPoints2(ofxHairModel model)
{
	m_weightInfo.clear();
	m_weightInfo.resize(m_graph.getNumNormalNodes());
	m_numSkinPoints = (m_graph.getNumNormalNodes());

	int count = 0;
	for(int i=0; i<model.strands.size(); i++){
		if(model.strands[i].bGuideHair){
			int numParticles = model.strands[i].m_particles.size();
			for(int j=0; j<numParticles; j++){
				m_weightInfo[count].index = i*numParticles+j;
				m_weightInfo[count].guideIndices.resize(m_skeltons.size()*numParticles);
				m_weightInfo[count].weight.resize(m_skeltons.size()*numParticles);
				count++;
			}
		}
	}
}

//--------------------------------------------------------------
void ofxHairSkinning::calcSkinningWeight()
{
	for(int i=0; i<m_numSkinPoints; i++){
		ofPoint p = m_skin_initial_points[i];
		int count = 0;
		for(int j=0; j<m_numSkeletons; j++){
			for(int k=0; k<m_skeltons[j].getNumJoints(); k++){
				ofPoint p0 = m_skeltons[j].getJoint(k).getPosition();
				ofPoint p1 = m_skeltons[j].getJoint(k+1).getPosition();
				m_weight[i][count] = calcWeightEuclidianDistance(p, p0, p1, -8);
				count++;
			}
		}
	}
	normalizeWeight();
}

//--------------------------------------------------------------
void ofxHairSkinning::calcSkinningWeight2(ofxHairModel model)
{
	cout << "calc skin weight" << endl;

	//// normal particles
	//for(int i=0; i<model.strands.size(); i++){
	//	if(!model.strands[i].bGuideHair){
	//		for(int j=0; j<model.strands[i].m_particles.size(); j++){

	//			weightInfo w;
	//			w.index = i*25+j;
	//			ofPoint p = model.strands[i].m_particles[j].position0;

	//			// guide particles
	//			for(int k=0; k<model.strands.size(); k++){
	//				if(model.strands[k].bGuideHair){
	//					for(int l=0; l<model.strands[k].m_particles.size()-1; l++){
	//						ofPoint p0 = model.strands[k].m_particles[l].position0;
	//						ofPoint p1 = model.strands[k].m_particles[l+1].position0;

	//						w.guideIndices.push_back(k*25+l);
	//						w.weight.push_back(calcWeightEuclidianDistance(p, p0, p1, -8));
	//					}
	//				}
	//			}
	//			
	//			// normalize
	//			float sum = 0.0f;
	//			for(int k=0; k<w.guideIndices.size(); k++){
	//				sum += w.weight[k];
	//			}
	//			for(int k=0; k<w.guideIndices.size(); k++){
	//				w.weight[k] /= sum;
	//			}

	//			// remove
	//			for(int k=0; k<w.guideIndices.size(); k++){
	//				if(w.weight[k] < 0.1){
	//					w.weight.erase(w.weight.begin() + k);
	//					w.guideIndices.erase(w.guideIndices.begin() + k);
	//				}
	//			}

	//			// normalize
	//			sum = 0.0f;
	//			for(int k=0; k<w.guideIndices.size(); k++){
	//				sum += w.weight[k];
	//			}
	//			for(int k=0; k<w.guideIndices.size(); k++){
	//				w.weight[k] /= sum;
	//				if(w.weight[k] > 0.1)
	//					cout << w.weight[k] << endl;
	//			}

	//			cout << "-------------------" << endl;

	//			m_weightInfo.push_back(w);

	//		}
	//	}
	//}

	// normal particles

	int count = 0;
	for(int i=0; i<model.strands.size(); i++){
		for(int j=0; j<model.strands[i].m_particles.size(); j++){

			weightInfo w;
			w.index = i*25+j;
			ofPoint p = model.strands[i].m_particles[j].position0;

			// guide particles
			for(int k=0; k<model.strands.size(); k++){
				for(int l=0; l<model.strands[k].m_particles.size()-1; l++){
					if(i*25+j != k*25+l){
						ofPoint p0 = model.strands[k].m_particles[l].position0;
						ofPoint p1 = model.strands[k].m_particles[l+1].position0;

						w.guideIndices.push_back(k*25+l);
						w.weight.push_back(calcWeightEuclidianDistance(p, p0, p1, -8));
					}
				}
			}
				
			// normalize
			float sum = 0.0f;
			for(int k=0; k<w.guideIndices.size(); k++){
				sum += w.weight[k];
			}
			for(int k=0; k<w.guideIndices.size(); k++){
				w.weight[k] /= sum;
			}

			// remove
			//for(int k=0; k<w.guideIndices.size(); k++){
			//	if(w.weight[k] < 0.1){
			//		w.weight.erase(w.weight.begin() + k);
			//		w.guideIndices.erase(w.guideIndices.begin() + k);
			//	}
			//}

			// normalize
			sum = 0.0f;
			for(int k=0; k<w.guideIndices.size(); k++){
				sum += w.weight[k];
			}

			for(int k=0; k<w.guideIndices.size(); k++){
				w.weight[k] /= sum;
				if(w.weight[k] > 0.0001){
					count++;
					//cout << w.weight[k] << endl;
				}
			}


			cout << "-------------------" << endl;

			m_weightInfo.push_back(w);
		}
	}

	string filename = "data/test.graph";
	ofstream ofs(filename);
	
	ofs << m_weightInfo.size() << " " << count << endl;

	for(int k=0; k<m_weightInfo.size(); k++){
		for(int l=0; l<m_weightInfo[k].guideIndices.size(); l++){
			if(m_weightInfo[k].weight[l] > 0.0001){
				ofs << m_weightInfo[k].guideIndices[l] << " ";
			}
		}
		ofs << endl;
	}

	ofs.close();

	//for(int i=0; i<
	//m_weightInfo[0]

	//cout << "done" << endl;

	//// error
	//for(int i=0; i<m_weightInfo.size(); i++){
	//	ofPoint p = model.strands[m_weightInfo[i].index/25].m_particles[m_weightInfo[i].index%25].position;
	//	int count = 0;
	//	for(int j=0; j<model.strands.size(); j++){
	//		if(model.strands[j].bGuideHair){
	//			for(int k=0; k<model.strands[j].m_particles.size()-1; k++){
	//				m_weightInfo[i].guideIndices[count] = j * model.strands[j].getResolution() + k;

	//				ofPoint p0 = model.strands[m_weightInfo[i].guideIndices[count]/25].m_particles[m_weightInfo[i].guideIndices[count]%25].position;
	//				ofPoint p1 = model.strands[m_weightInfo[i].guideIndices[count]/25].m_particles[m_weightInfo[i].guideIndices[count]%25+1].position;
	//				m_weightInfo[i].weight[count] = calcWeightEuclidianDistance(p, p0, p1, -8);
	//				count++;
	//			}
	//		}
	//	}
	//}
}

//--------------------------------------------------------------
void ofxHairSkinning::deformation2(ofxHairModel model)
{
	for(int j=0; j<model.strands.size(); j++){
		if(model.strands[j].bGuideHair){
			for(int k=0; k<model.strands[j].m_particles.size()-1; k++){

			}
		}
	}
}

//--------------------------------------------------------------
void ofxHairSkinning::openGuideMatrixInfo(int frameNum)
{
#if 1
	string filename = "data/test.guidemat";
	ofsMat.open(filename, ios::out|ios::binary|ios::trunc);

	// count num
	int count = 0;
	for(int i=0; i<m_graph.getNumNodes(); i++){
		if(m_graph.getHairNode(i).getBoolGuideHair())
		{
			count++;
		}
	}

	// write guide particle num
	ofsMat.write((const char*)&count, sizeof(int));

	for(int i=0; i<m_graph.getNumNodes(); i++){
		if(m_graph.getHairNode(i).getBoolGuideHair())
		{
			ofsMat.write((const char*)&i, sizeof(int));
		}
	}

	// write frame num
	ofsMat.write((const char*)&frameNum, sizeof(int));

#else
	string filename = "data/test.guidemat";
	ofsMat.open(filename);

	// count num
	int count = 0;
	for(int i=0; i<m_graph.getNumNodes(); i++){
		if(m_graph.getHairNode(i).getBoolGuideHair()){
			count++;
		}
	}

	ofsMat << count << endl;

	for(int i=0; i<m_graph.getNumNodes(); i++){
		if(m_graph.getHairNode(i).getBoolGuideHair())
		{
			ofsMat << i << " "; 
		}
	}
	ofsMat << endl;

	// write frame num
	ofsMat << frameNum << endl;

#endif
}

//--------------------------------------------------------------
void ofxHairSkinning::writeGuideMatrixInfo()
{
#if 1
	for(int i=0; i<m_graph.getNumNodes(); i++){
		if(m_graph.getHairNode(i).getBoolGuideHair())
		{
			int sIndex = mapIdxNode2Joint[i]/m_skeltons[0].getNumJoints();
			int jIndex = mapIdxNode2Joint[i]%m_skeltons[0].getNumJoints();
			ofMatrix4x4 mat = m_skeltons[sIndex].getJoint(jIndex).getGlobalMatrix();
			for(int i=0; i<16; i++){
				float f = mat(i/4, i%4);
				ofsMat.write((const char*)&f, sizeof(float));
			}
			//ofsMat.write((const char*)&mat, sizeof(mat));
		}
	}

#else
	for(int i=0; i<m_graph.getNumNodes(); i++){
		if(m_graph.getHairNode(i).getBoolGuideHair())
		{
			int sIndex = mapIdxNode2Joint[i]/m_skeltons[0].getNumJoints();
			int jIndex = mapIdxNode2Joint[i]%m_skeltons[0].getNumJoints();
			ofMatrix4x4 mat = m_skeltons[sIndex].getJoint(jIndex).getGlobalMatrix();
			for(int k=0; k<16; k++){
				ofsMat << mat(k/4,k%4) << " ";
			}
		}
	}
	ofsMat << endl;
#endif
}

//--------------------------------------------------------------
void ofxHairSkinning::closeGuideMatrixInfo()
{
	ofsMat.close();
}

//--------------------------------------------------------------
void ofxHairSkinning::openNormalParticleInfo(ofxHairModel model, int frameNum)
{
#if 1
	string filename = "data/test.normalPos";
	ofsNormal.open(filename, ios::out|ios::binary|ios::trunc);

	// vertices num
	int count = 0;
	for(int i=0; i<m_graph.getNumNodes(); i++){
		if(!m_graph.getHairNode(i).getBoolGuideHair())
		{
			count++;
		}
	}
	ofsNormal.write((const char*)&count, sizeof(int));

	// indices
	for(int i=0; i<m_graph.getNumNodes(); i++){
		if(!m_graph.getHairNode(i).getBoolGuideHair())
		{
			ofsNormal.write((const char*)&i, sizeof(int));
		}
	}

	// initial position
	for(int i=0; i<m_graph.getNumNodes(); i++){
		if(!m_graph.getHairNode(i).getBoolGuideHair())
		{
			int sIndex = i/m_skeltons[0].getNumJoints();
			int jIndex = i&m_skeltons[0].getNumJoints();
			ofPoint p0 = model.strands[sIndex].m_particles[jIndex].position;
			ofsNormal.write((const char*)&p0, sizeof(p0));
		}
	}

	ofsNormal.write((const char*)&frameNum, sizeof(int));

#else
	// indices
	string filename = "data/test.normalPos";
	ofsNormal.open(filename);
	for(int i=0; i<m_graph.getNumNodes(); i++){
		if(!m_graph.getHairNode(i).getBoolGuideHair())
		{
			ofsNormal << i << " "; 
		}
	}

	ofsNormal << endl;

	// initial position
	for(int i=0; i<m_graph.getNumNodes(); i++){
		if(!m_graph.getHairNode(i).getBoolGuideHair())
		{
			int sIndex = i/m_skeltons[0].getNumJoints();
			int jIndex = i&m_skeltons[0].getNumJoints();
			ofPoint p0 = model.strands[sIndex].m_particles[jIndex].position;
			ofsNormal << p0.x << " " << p0.y << " " << p0.z << " ";
		}
	}
	ofsNormal << endl;

#endif
}

//--------------------------------------------------------------
void ofxHairSkinning::writeNormalParticleInfo(ofxHairModel model)
{
#if 1
	// current position
	for(int i=0; i<m_graph.getNumNodes(); i++){
		if(!m_graph.getHairNode(i).getBoolGuideHair())
		{
			int sIndex = i/m_skeltons[0].getNumJoints();
			int jIndex = i%m_skeltons[0].getNumJoints();
			ofPoint p1 = model.strands[sIndex].m_particles[jIndex].position;
			ofsNormal.write((const char*)&p1, sizeof(p1));
		}
	}

#else
	// current position
	for(int i=0; i<m_graph.getNumNodes(); i++){
		if(!m_graph.getHairNode(i).getBoolGuideHair())
		{
			int sIndex = i/m_skeltons[0].getNumJoints();
			int jIndex = i%m_skeltons[0].getNumJoints();
			ofPoint p1 = model.strands[sIndex].m_particles[jIndex].position;
			ofsNormal << p1.x << " " << p1.y << " " << p1.z << " ";
		}
	}
	ofsNormal << endl;

#endif;
}

//--------------------------------------------------------------
void ofxHairSkinning::closeNormalParticleInfo()
{
	ofsNormal.close();
}

//--------------------------------------------------------------
bool ofxHairSkinning::loadSkinningWeight()
{
	cout << "load skinning weight...";

	m_weightInfo.resize(m_graph.getNumNormalNodes());

	for(int i=0; i<m_graph.getNumNormalNodes(); i++)
	{
		string filename = "data/weight/sample_" + ofToString(i) + ".skinweight";
		ifstream ofs(filename);
		if (ofs.fail()) {
			std::cerr << "failed..." << filename << std::endl;
			//return false;
		}
	
		std::string str;

		// header
		getline(ofs, str);
		m_weightInfo[i].index = ofToInt(str);

		getline(ofs, str);
		m_weightInfo[i].guideIndices.resize(ofToInt(str));
		m_weightInfo[i].weight.resize(ofToInt(str));

		for(int j=0; j<m_weightInfo[i].guideIndices.size(); j++){
			getline(ofs, str);
			vector<string> indices = ofxHairUtil::split(str, ' ');
			m_weightInfo[i].guideIndices[j] = ofToInt(indices[0]);
			m_weightInfo[i].weight[j] = ofToFloat(indices[1]);
		}
	}

	// normalize
	//for(int i=0; i<m_weightInfo.size(); i++){
	//	float sum = 0.0f;
	//	for(int j=0; j<m_weightInfo[i].guideIndices.size(); j++){
	//		sum += m_weightInfo[i].weight[j];
	//	}
	//	for(int j=0; j<m_weightInfo[i].guideIndices.size(); j++){
	//		m_weightInfo[i].weight[j] /= sum;
	//	}
	//}

	cout << "done..." << endl;
	return true;
}

//--------------------------------------------------------------
void ofxHairSkinning::debugSetWeightColor(ofxHairModel& model)
{
	cout << "set weight color on hair...";

	for(int i=0; i<m_weightInfo.size(); i++){
		ofVec3f color(0.0f);
		for(int j=0; j<m_weightInfo[i].guideIndices.size(); j++){
			int sIdx = m_weightInfo[i].guideIndices[j]/25;
			int pIdx = m_weightInfo[i].guideIndices[j]%25;

			ofVec3f col;
			col.x = model.strands[sIdx].m_particles[pIdx].color.r;
			col.y = model.strands[sIdx].m_particles[pIdx].color.g;
			col.z = model.strands[sIdx].m_particles[pIdx].color.b;
			color += col * m_weightInfo[i].weight[j];
		}
		int sIdx = m_weightInfo[i].index/25;
		int pIdx = m_weightInfo[i].index%25;
		model.strands[sIdx].m_particles[pIdx].color = ofColor(color.x, color.y, color.z);
	}
	cout << "done..." << endl;
}

//--------------------------------------------------------------
void ofxHairSkinning::normalizeWeight()
{
	for(int i=0; i<m_numSkinPoints; i++){
		float sum = 0.0;
		for(int j=0; j<m_numJointPoints; j++){
			sum += m_weight[i][j];
		}

		for(int j=0; j<m_numJointPoints; j++){
			m_weight[i][j] = m_weight[i][j]/sum;
		}
	}
}

//--------------------------------------------------------------
void ofxHairSkinning::drawSkinningWeight(int indexJoint, ofxHairModel model)
{
	glPointSize(10.0);
	glBegin(GL_POINTS);
	int i=0;

	ofSetColor(255, 0, 0);	
	int joint = m_weightInfo[indexJoint].index;
	ofPoint p = model.strands[joint/25].m_particles[joint%25].position;
	glVertex3f(p.x, p.y, p.z);

	for(auto g : m_weightInfo[indexJoint].guideIndices)
	{
		ofColor c = getWeightColor(m_weightInfo[indexJoint].weight[i], 0.0, 1.0);
		ofSetColor(c);
		ofVec3f v = model.strands[g/25].m_particles[g%25].position;
		glVertex3f(v.x, v.y, v.z);
		i++;
	}
	glEnd();

	ofSetColor(255);
	cout << ofToString(joint) + " : " + ofToString(m_weightInfo[indexJoint].guideIndices.size()) << endl;
}

//--------------------------------------------------------------
float ofxHairSkinning::calcWeightEuclidianDistance(const ofPoint input, const ofPoint p0, const ofPoint p1, const int c)
{
	float d = ofxMeshUtil::distance_point_to_edge(input, p0, p1);
	return pow(d+1,c);
}

//--------------------------------------------------------------
float ofxHairSkinning::getWeight(const int skin_idx, int joint_idx)
{
	return m_weight[skin_idx][joint_idx];
}

//--------------------------------------------------------------
void ofxHairSkinning::draw()
{
	for(auto s : m_skeltons)
		s.drawAxis();
}

//--------------------------------------------------------------
ofColor ofxHairSkinning::getWeightColor(float value, float errMin, float errMax)
{
	int r, g, b;
	float norm_err = (value - errMin) / (errMax - errMin);
	float H, Hi, f, p, q, t, S = 1.0f, V = 1.0f;

	H = 360.0f - (240.0f * norm_err + 120.0f);

	if(H < 0.0f) 
	{ 
		H = 0.0f; 
	}

	Hi = (float)floor(H / 60.0f);

	f = H / 60.0f - Hi;

	p = V * (1.0f - S);
	q = V * (1.0f - f * S);
	t = V * (1.0f - (1.0f - f) * S);

	r = g = b = 0;

	if(Hi == 0) 
	{
		r = (int)(255.0f * V); 
		g = (int)(255.0f * t);
		b = (int)(255.0f * p);
	}
	if(Hi == 1)
	{
		r = (int)(255.0f * q); 
		g = (int)(255.0f * V);
		b = (int)(255.0f * p);
	}
	if(Hi == 2)
	{
		r = (int)(255.0f * p); 
		g = (int)(255.0f * V);
		b = (int)(255.0f * t);
	}
	if(Hi == 3)
	{
		r = (int)(255.0f * p); 
		g = (int)(255.0f * q);
		b = (int)(255.0f * V);
	}
	if(Hi == 4)
	{
		r = (int)(255.0f * t); 
		g = (int)(255.0f * p);
		b = (int)(255.0f * V);
	}
	if(Hi == 5)
	{
		r = (int)(255.0f * V); 
		g = (int)(255.0f * p);
		b = (int)(255.0f * q);
	}

	ofColor error(r,g,b);
	return error;
}