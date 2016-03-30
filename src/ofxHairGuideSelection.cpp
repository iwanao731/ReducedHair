#include "ofxHairGuideSelection.h"

float ofxHairGuideSelection::calculateSimilarity(ofxHairSkeleton s1, ofxHairSkeleton s2)
{
	// we assume s1 and s2 have same number of particles
	int jointNum = s1.getNumJoints();

	float sumError = 0.0;
	float maxError =0.0;

	for(int i=0; i<jointNum; i++) {
		ofPoint init_pos_1 = s1.getJoint0(i).getPosition();
		ofPoint currnt_pos_1 = s1.getJoint(i).getPosition();
		ofPoint init_pos_2 = s2.getJoint0(i).getPosition();
		ofPoint currnt_pos_2 = s2.getJoint(i).getPosition();
		ofMatrix4x4 mat_1 = s1.getJoint(i).getLocalMatrix();
		ofMatrix4x4 mat_2 = s2.getJoint(i).getLocalMatrix();

		float error = (currnt_pos_1 - mat_2 * init_pos_1).lengthSquared() + (currnt_pos_2 - mat_1 * init_pos_2).lengthSquared();
		if(maxError < error) { maxError = error; }
		sumError += error;
	}
	return sumError;
}

float ofxHairGuideSelection::frobeniusNorm(ofMatrix4x4 mat)
{
	float error = 0;
	for(int i=0; i<4; i++){
		for(int j=0; j<4; j++){
			error += mat(i, j) * mat(i, j);
		}
	}
	return error;
}

void ofxHairGuideSelection::initkNNGraph(vector<ofPoint*> points)
{
	m_counts.resize(points.size(), points.size());
	m_counts.setZero();

	m_weight.resize(points.size(), points.size());
	m_weight.setZero();
}

void ofxHairGuideSelection::updatekNNGraph(vector<ofPoint*> points, float radius)
{
	vector<ofPoint> tmp_p;
	tmp_p.resize(points.size());

	// TODO 
	// I'd like to remove this part and I'd like to use points directly
	// It's means I have to try to change to vector<ofPoint> points from vector<ofPoint*> points
	int count=0;
	for(auto& p : points){
		tmp_p[count] = p[0];
		count++;
	}

	// build Nearest Neighbor structure on boundary
	ofxNearestNeighbour3D nn;
	nn.buildIndex(tmp_p);

	//m_edgeGraph.clear();
	// calculate every frame
	for(int i=0; i<points.size(); i++){

		vector<pair<NNIndex, float> > indices;
		nn.findPointsWithinRadius(tmp_p[i], radius, indices);

		for (int j=0; j<indices.size(); ++j) {

			if(i!=indices[j].first && i<indices[j].first) {
				m_counts.coeffRef(i, indices[j].first) += 1; // <- so slow but no limit

				//m_edgeGraph.push_back(T(i, indices[j].first, 1)); // <- That's so fast, but they have a limit around 40 milion
			}
		}
	}

	//cout << m_edgeGraph.size()  << "," << m_edgeGraph.max_size() << endl;

	// clean
	tmp_p.clear();
}

void ofxHairGuideSelection::buildkNNGraph(int num_frames, float epsiron)
{
	//m_counts.setFromTriplets(m_edgeGraph.begin(), m_edgeGraph.end());
	std::vector<T> w;

	// we further remove the graph edges by removing the edges whose counts are smaller than a threshold
	for(int i=0; i<m_counts.outerSize(); ++i){
		for(Eigen::SparseMatrix<int>::InnerIterator it(m_counts,i); it; ++it){
			if(it.value() > epsiron * num_frames){

				// set new connection, and initialize as 0
				w.push_back(T(it.row(), it.col(), 0));
				w.push_back(T(it.col(), it.row(), 0));
			}
		}
	}

	// set weight edge
	m_weight.setFromTriplets(w.begin(), w.end());

	// clean
	w.clear();
}

void ofxHairGuideSelection::accumulateWeight(vector<ofxHairSkeleton> skeleton, bool debug)
{
	int numJoint = skeleton[0].getNumJoints();

	for(int i=0; i<m_weight.outerSize(); ++i){
		for(Eigen::SparseMatrix<int>::InnerIterator it(m_weight,i); it; ++it){

			// calculate skeleton number from particle number 
			int skeletonIndex1 = it.col() / numJoint;
			int jointIndex1 = it.col() % numJoint;
			int skeletonIndex2 = it.row() / numJoint;
			int jointIndex2 = it.row() % numJoint;

			float error = calculateSimilarityJoint(
				skeleton[skeletonIndex1].getJoint0(jointIndex1),
				skeleton[skeletonIndex1].getJoint(jointIndex1),
				skeleton[skeletonIndex2].getJoint0(jointIndex2),
				skeleton[skeletonIndex2].getJoint(jointIndex2)
				);

			m_weight.coeffRef(it.col(), it.row()) += error;

			if(debug){
				cout << it.col() << "," << it.row() << "," << error << endl;
			}

			// still have error if use this style
			//m_edgeWeight.push_back(T(it.col(), it.row(), error));
		}
	}
}

void ofxHairGuideSelection::calcWeight()
{
	float maxError = 0.0f;
	edgeNum = 0;

	// calculate maximum value
	for(int i=0; i<m_weight.outerSize(); ++i){
		for(Eigen::SparseMatrix<int>::InnerIterator it(m_weight,i); it; ++it){
			if(it.value() > maxError) { maxError = it.value(); }
		}
	}

	// calculate final weight
	for(int i=0; i<m_weight.outerSize(); ++i){
		for(Eigen::SparseMatrix<int>::InnerIterator it(m_weight,i); it; ++it){
			m_weight.coeffRef(it.col(), it.row()) = -m_weight.coeffRef(it.col(), it.row()) + maxError;
			edgeNum++;
			//cout << m_weight.coeffRef(it.col(), it.row()) << ",";
			//cout << it.row() << ",";
			//cout << it.col() << ",";
			//cout << it.index() <<endl;
		}
	}
}

bool ofxHairGuideSelection::exportGraphFile(string filename)
{
	ofstream ofs(filename);
	ofs << m_counts.rows() << " " << edgeNum/2 << endl;

	for(int i=0; i<m_weight.outerSize(); ++i){
		for(Eigen::SparseMatrix<int>::InnerIterator it(m_weight,i);it; ++it){			
			ofs << it.row()+1 << " ";
			//ofs << it.row()+1 << " " << m_weight.coeffRef(it.col(), it.row()) << " ";
		}
		ofs << endl;
	}
	ofs.close();

	return true;
}

bool ofxHairGuideSelection::exportWeightFile(string filename)
{
	ofstream ofs(filename);

	for(int i=0; i<m_weight.outerSize(); ++i){
		for(Eigen::SparseMatrix<int>::InnerIterator it(m_weight,i); it; ++it){
			ofs << it.col() << " " << it.row() << " " << m_weight.coeffRef(it.col(), it.row()) << std::endl;
		}
	}
	ofs.close();
	return true;
}

float ofxHairGuideSelection::calculateSimilarityJoint(ofxHairJoint init_j1, ofxHairJoint curent_ji, ofxHairJoint init_j2, ofxHairJoint current_j2)
{
	ofPoint init_pos_1 = init_j1.getPosition();
	ofPoint currnt_pos_1 = curent_ji.getPosition();
	ofPoint init_pos_2 = init_j2.getPosition();
	ofPoint currnt_pos_2 = current_j2.getPosition();

	ofMatrix4x4 mat_1 = curent_ji.getLocalMatrix();
	ofMatrix4x4 mat_2 = current_j2.getLocalMatrix();
	mat_1.setTranslation(ofVec3f::zero());
	mat_2.setTranslation(ofVec3f::zero());

	float error = (currnt_pos_1 - mat_2 * init_pos_1).lengthSquared() + (currnt_pos_2 - mat_1 * init_pos_2).lengthSquared();

	//if(error > 5000){
	//	cout << mat_1 << endl << endl;
	//	cout << mat_2 << endl << endl;
	//	cout << error << endl << endl;
	//	cout <<  (currnt_pos_1 - mat_2 * init_pos_1).lengthSquared() << "\t" << currnt_pos_1 << "\t" << mat_2 * init_pos_1 << endl << endl;
	//	cout <<  (currnt_pos_2 - mat_1 * init_pos_2).lengthSquared() << "\t" << currnt_pos_2 << "\t" << mat_1 * init_pos_2 << endl << endl;
	//	cout << "----------------------" << endl;
	//}
	return error;
}

void ofxHairGuideSelection::debugDraw_kNN(int particle_index, vector<ofxHairSkeleton> skeleton, int strand_resolution)
{
	for(Eigen::SparseMatrix<int>::InnerIterator it(m_weight, particle_index); it; ++it){
		int index1 = it.row()/strand_resolution;
		int index2 = it.row()%strand_resolution;

		ofSetColor(255, 0, 0);
		ofDrawSphere(skeleton[index1].getJoint(index2).getPosition(), 4.0);
	}
}

void ofxHairGuideSelection::debugSetGroupColor(ofxHairModel &model, int groupNum)
{
	int resolution = model.strands[0].getResolution();

	for(int i=0; i<m_weight.outerSize(); ++i){
		for(Eigen::SparseMatrix<int>::InnerIterator it(m_weight, i); it; ++it){
			int index1 = it.row()/resolution;
			int index2 = it.row()%resolution;
			
			ofColor c;
			c.setHsb(ofMap(groupIndex[it.row()], 0, groupNum, 0, 255), 200, 255);
			model.strands[index1].m_particles[index2].color = c;
		}
	}
}

void ofxHairGuideSelection::debugDraw_Group(vector<ofxHairSkeleton> skeleton, int strand_resolution)
{
	for(int i=0; i<m_weight.outerSize(); ++i){
		for(Eigen::SparseMatrix<int>::InnerIterator it(m_weight, i); it; ++it){
			int index1 = it.row()/strand_resolution;
			int index2 = it.row()%strand_resolution;

			ofDrawSphere(skeleton[index1].getJoint(index2).getPosition(), 4.0);
		}
	}
}

void ofxHairGuideSelection::buildKCutGroup(int numCut)
{
	std::cout << "build k-cut group... partition to " << numCut << endl;

	int ret;
	string command = "gpmetis.exe data/test.group " + ofToString(numCut);
	ret = system(command.c_str());
	if(ret){
		std::cout << "failed" << endl;
	}else{
		std::cout << "done" << endl;
	}
}

void ofxHairGuideSelection::importGroupFile(const string filename)
{
	// import group file
	ifstream ofs(filename);
	if (ofs.fail()) {
        std::cerr << "failed" << std::endl;
    }else{
		cout << "succeeded" << endl;
	}
	
	std::string str;
	while (getline(ofs, str)) {
		groupIndex.push_back(ofToInt(str));
    }
}