#include "ofxHairWeightOptimization.h"

//--------------------------------------------------------------
ofxHairWeightOptimization::ofxHairWeightOptimization()
{
}

bool ofxHairWeightOptimization::loadNormalParticleInfo(const string filename)
{
#if 1
	cout << filename << " loading ...";

	ifstream ofs(filename, ios::in | ios::binary );
	if(ofs.fail()){
		cout << "failed" << endl;
		return false;
	}

	// resize
	int numNParticles;
	ofs.read( ( char * ) &numNParticles, sizeof( int ) );
	m_normalParticles.resize(numNParticles);

	// index
	for(int i=0; i<numNParticles; i++){
		int index;
		ofs.read( ( char * ) &index, sizeof( int ) );
		m_normalParticles[i].index = index;
	}

	// init position
	for(int i=0; i<numNParticles; i++){
		ofPoint p0;
		ofs.read( ( char * ) &p0, sizeof( p0 ) );
		m_normalParticles[i].m_initPos = p0;		
	}

	int frameNum;
	ofs.read( ( char * ) &frameNum, sizeof( int ) );

	// current position
	for(int i=0; i<frameNum; i++){
		for(int j=0; j<numNParticles; j++){
			ofPoint p1;
			ofs.read( ( char * ) &p1, sizeof( p1 ) );
			m_normalParticles[j].m_currPos.push_back(p1);
		}
	}
	
	// link group
	for(int i=0; i<m_normalParticles.size(); i++){
		int index = m_normalParticles[i].index;
		m_normalParticles[i].guideParticles.resize(m_graph.getHairNode(index).getGuideLink().size());
		for(int j=0; j<m_graph.getHairNode(index).getGuideLink().size(); j++){
			m_normalParticles[i].guideParticles[j].m_index = m_graph.getHairNode(index).getGuideLink().at(j);
		}
	}

	cout << "done" << endl;

	return true;
#else
	std::cout << filename << " loading ...";

	ifstream ofs(filename);
	if(ofs.fail()){
		std::cout << "failed" << endl;
		return false;
	}

	// index
	string str;
	std::getline(ofs, str);
	vector<string> indices = ofxHairUtil::split(str, ' ');
	m_normalParticles.resize(indices.size());

	int i=0;
	for(auto s : indices){
		m_normalParticles[i].index = ofToInt(s);
		i++;
	}

	// init position
	{
		std::getline(ofs, str);
		vector<string> indices = ofxHairUtil::split(str, ' ');
		int k=0;
		for(int i=0; i<indices.size(); i+=3){
			ofVec3f p;
			p.x = ofToFloat(indices[i+0]);
			p.y = ofToFloat(indices[i+1]);
			p.z = ofToFloat(indices[i+2]);
			m_normalParticles[k].m_initPos = p;
			k++;
		}
	}

	// current position
	while(std::getline(ofs, str)) {
		vector<string> indices = ofxHairUtil::split(str, ' ');

		int k=0;
		for(int i=0; i<indices.size(); i+=3){
			ofVec3f p;
			p.x = ofToFloat(indices[i+0]);
			p.y = ofToFloat(indices[i+1]);
			p.z = ofToFloat(indices[i+2]);
			m_normalParticles[k].m_currPos.push_back(p);
			k++;
	  	}
	}

	// link group
	for(int i=0; i<m_normalParticles.size(); i++){
		int index = m_normalParticles[i].index;
		m_normalParticles[i].guideParticles.resize(m_graph.getHairNode(index).getGuideLink().size());
		for(int j=0; j<m_graph.getHairNode(index).getGuideLink().size(); j++){
			m_normalParticles[i].guideParticles[j].m_index = m_graph.getHairNode(index).getGuideLink().at(j);
		}
	}

	std::cout << "done" << endl;

	return true;
#endif
	
}

bool ofxHairWeightOptimization::loadGuideMatrix(const string filename)
{
#if 1
	cout << filename << " loading ...";

	ifstream ofs(filename, ios::in | ios::binary );
	if(ofs.fail()){
		cout << "failed" << endl;
		return false;
	}

	int numGuideParticles;
	ofs.read( ( char * ) &numGuideParticles, sizeof( int ) );
	m_guideParticle.resize(numGuideParticles);

	// guide hair indices
	for(int i=0; i<numGuideParticles; i++){
		int index;
		ofs.read( ( char * ) &index, sizeof( int ) );
		m_guideParticle[i].m_index = index;
	}

	// frame num
	int frameNum;
	ofs.read( ( char * ) &frameNum, sizeof( int ) );

	for(int i=0; i<frameNum; i++){
		for(int j=0; j<numGuideParticles; j++){
			ofMatrix4x4 mat;
			for(int k=0; k<16; k++){
				float f;
				ofs.read( ( char * ) &f, sizeof( float ) );
				mat(k/4, k%4) = f;
			}
			//ofs.read( ( char * ) &mat, sizeof( ofMatrix4x4 ) );
			m_guideParticle[j].q_frame.push_back(mat);
		}
	}
	
	cout << "done" << endl;

	return true;

#else
	cout << filename << " loading ...";

	ifstream ofs(filename);
	if(ofs.fail()){
		cout << "failed" << endl;
		return false;
	}

	string str;
	getline(ofs, str);
	int numGuideParticles = ofToInt(str);
	m_guideParticle.resize(numGuideParticles);

	getline(ofs, str);
	vector<string> indices = ofxHairUtil::split(str, ' ');

	// guide hair indices
	for(int i=0; i<numGuideParticles; i++){
		m_guideParticle[i].m_index = ofToInt(indices[i]);
	}

	int frameNum;
	getline(ofs, str);
	frameNum = ofToInt(str);

	for(int i=0; i<frameNum; i++){
		getline(ofs, str);
		vector<string> indices = ofxHairUtil::split(str, ' ');
		for(int j=0; j<numGuideParticles; j++){
			ofMatrix4x4 mat;
			for(int k=0; k<16; k++){
				mat(k/4,k%4) = ofToFloat(indices[j*16+k]);
			}
			m_guideParticle[j].q_frame.push_back(mat);
		}		
	}
	cout << "done" << endl;

	return true;
#endif
}

void ofxHairWeightOptimization::calculateWeights()
{
#if 0
	for(int i=0; i<m_normalParticles.size(); i++){
		if(i>318)
			optimizeEachParticles(i); // optimization
	}

#else
	// paralel computing using C++11
	std::vector<std::thread> workers;
	std::atomic<int> i(0);

	int numThreads = std::thread::hardware_concurrency();
	numThreads = (numThreads < 1) ? 1 : numThreads;

	for (auto t = 0; t < numThreads; t++) {
		workers.push_back(std::thread([&, t]() {
		int index = 0;
		
		while ((index = i++) < m_normalParticles.size()) { // m_normalParticles.size()-1 ??
			optimizeEachParticles(i-1); // optimization
		}
		}));
	}

	for (auto &t : workers) {
		t.join();
	}
#endif
}

bool pairCompare(const std::pair<float, int>firstElem, const std::pair<float, int> secondElem) {
	return firstElem.first > secondElem.first;
}

bool ofxHairWeightOptimization::checkOptimization()
{	
	cout << "check start...";
	int count = exampleNum;
	for(int i=0; i<m_graph.getNumNodes(); i++){
		if(!m_graph.getHairNode(i).getBoolGuideHair()){
			int boneNum = m_graph.getHairNode(i).getGuideLink().size();
			if(boneNum > count){
				count = boneNum;
			}
			if(boneNum == 0){
				cout << "Number of Link particles is 0" << endl;
				return false;
			}
		}
	}

	if(count > exampleNum){
		cout << "you can't optimize because linkNum is over than exampleNum" << endl;
		cout << "index : " << count << ">" << exampleNum << endl;
		return false;
	}


	cout << "OK..." << endl;
	return true;
}

void ofxHairWeightOptimization::optimizeEachParticles(const int index)
{
	cout << "---------" << endl;
	cout << "index :" << index << endl;

	Matrix<double> G, CE, CI;
	Vector<double> g0, ce0, ci0, x;
	int n, m, p;
	double sum = 0.0;
	char ch;
	vector<int> linkIndex;

	int boneNum = m_normalParticles[index].guideParticles.size();

	cout << "link num : " << boneNum << endl;

	if(boneNum > 0)
	{
		Eigen::MatrixXd am;
		Eigen::MatrixXd bv;

		bv.resize(1, 3*exampleNum);
		//am.resize(exampleNum, 3*boneNum);
		am.resize(boneNum, 3*exampleNum);

		// set bv
		for(int i=0; i<exampleNum; i++)
		{
			//bv(0, i*3+0) = m_normalParticles[index].m_currPos[i].x;
			//bv(0, i*3+1) = m_normalParticles[index].m_currPos[i].y;
			//bv(0, i*3+2) = m_normalParticles[index].m_currPos[i].z;
			bv(0, i*3+0) = floor(m_normalParticles[index].m_currPos[i].x*10000) / 1000000.f;
			bv(0, i*3+1) = floor(m_normalParticles[index].m_currPos[i].y*10000) / 1000000.f;
			bv(0, i*3+2) = floor(m_normalParticles[index].m_currPos[i].z*10000) / 1000000.f;
		}

		// set am
		for(int j=0; j<boneNum; j++){
			for(int m=0; m<m_guideParticle.size(); m++){
				if(m_normalParticles[index].guideParticles[j].m_index == m_guideParticle[m].m_index){
					ofMatrix4x4 mat0 = m_guideParticle[m].q_frame[0];
					for(int k=0; k<exampleNum; k++){
						ofMatrix4x4 mat = m_guideParticle[m].q_frame[k];
						ofVec3f p = m_normalParticles[index].m_initPos * mat0.getInverse() * mat;
						//am(j, k*3+0) = p.x;
						//am(j, k*3+1) = p.y;
						//am(j, k*3+2) = p.z;
						am(j, k*3+0) = floor(p.x*10000) / 1000000.f;
						am(j, k*3+1) = floor(p.y*10000) / 1000000.f;
						am(j, k*3+2) = floor(p.z*10000) / 1000000.f;
						//am(k, j*3+0) = floor(p.x*10000) / 1000000.f;
						//am(k, j*3+1) = floor(p.y*10000) / 1000000.f;
						//am(k, j*3+2) = floor(p.z*10000) / 1000000.f;
					}
				}
			}
		}

		// set link
		linkIndex.resize(boneNum);
		for(int i=0; i<boneNum; i++){
			linkIndex[i] = m_normalParticles[index].guideParticles[i].m_index;
		}

		//Eigen::MatrixXd am1;
		//am1.resize(exampleNum, 3*boneNum);

		//// change
		//// ng : bonenum > exampleNum
		//for(int i=0; i<exampleNum; i++){
		//	for(int j=0; j<boneNum; j++){
		//		am1(i, j*3+0) = (float) am(j, i*3+0);
		//		am1(i, j*3+1) = (float) am(j, i*3+1);
		//		am1(i, j*3+2) = (float) am(j, i*3+2);
		//	}
		//}

		//am = am1;

		//for(int i=0; i<exampleNum; i++){
		//	for(int j=0; j<boneNum; j++){
		//		am(i, j*3+0) = floor(am1(i, j*3+0)*1000000) / 1000000.f;
		//		am(i, j*3+1) = floor(am1(i, j*3+1)*1000000) / 1000000.f;
		//		am(i, j*3+2) = floor(am1(i, j*3+2)*1000000) / 1000000.f;
		//	}
		//}

		//string filename = "data/weight/sample_" + ofToString(index) + ".skinweight";
		//ofstream ofs(filename);

		//ofs << exampleNum << endl;
		//ofs << linkIndex.size() << endl;
		//ofs << index << " ";
		//for(int i=0; i<linkIndex.size(); i++){
		//	ofs << linkIndex[i] << " ";
		//}
		//ofs << endl;

		//for(int i=0; i<exampleNum; i++){
		//	ofs << bv(0, i*3+0) << " " << bv(0, i*3+1) << " " << bv(0, i*3+2) << " ";
		//	for(int j=0; j<boneNum; j++){
		//		ofs << am(i, j*3+0) << " " << am(i, j*3+1) << " " << am(i, j*3+2) << " ";
		//	}
		//	ofs << endl;
		//}

		//exportTempFile(index, am, bv);
		//loadTempFile(index, am, bv, linkIndex);

		//{
			//for(int i=0; i<exampleNum; i++){
			//	for(int j=0; j<boneNum; j++){
			//		am(i, j*3+0) = floor(am1(i, j*3+0)*100000) / 100000.f;
			//		am(i, j*3+1) = floor(am1(i, j*3+1)*100000) / 100000.f;
			//		am(i, j*3+2) = floor(am1(i, j*3+2)*100000) / 100000.f;

				//	ofPoint p0(am(i, j*3+0), am(i, j*3+1), am(i, j*3+2));
				//	ofPoint p1(am1(i, j*3+0), am1(i, j*3+1), am1(i, j*3+2));
				//	if(p0 != p1){
				//		cout << setprecision(10); // ¸“x‚ð 4Œ…‚ÉŽw’è‚µ‚Ü‚·B
				//		cout << "error : " << (p0-p1).length() << endl;
				//		cout << p0 << endl;
				//		cout << p1 << endl;
				//		cout << "------------------" << endl;
				//	}
			//	}
			//}

			Eigen::MatrixXd gMat = am * am.transpose();
			Eigen::VectorXd gVec = - am * bv.transpose();

			n = boneNum;
			G.resize(n, n);
			{
				for (int i = 0; i < n; i++)	
					for (int j = 0; j < n; j++)
						G[i][j] = gMat(i,j);
			}
			
			g0.resize(n);
			{
				for (int i = 0; i < n; i++)
					g0[i] = gVec(i);
			}
		  
			m = 1;
			CE.resize(n, m);
			{
				for (int i = 0; i < n; i++)
					for (int j = 0; j < m; j++)
						CE[i][j] = 1.0;
			} 
		  
			ce0.resize(m);
			{
				for (int j = 0; j < m; j++)
					ce0[j] = -1.0;
			}
			
			p = n;
			CI.resize(n, p);
			{ 
				for (int i = 0; i < n; i++)
					for (int j = 0; j < p; j++)
						if(i==j){
							CI[i][j] = 1.0;
						}else{
							CI[i][j] = 0.0;	
						}
			}

			ci0.resize(p);
			{
				for (int j = 0; j < p; j++)
					ci0[j] = 0.0;
			}

			x.resize(n);

			std::cout << "f: " << solve_quadprog(G, g0, CE, ce0, CI, ci0, x) << std::endl;

			//cout <<  "x" << endl;

			for(int k=0; k<n; k++){
				if(x[k] > 0.0){
					sum += x[k];
				}
			}

			cout << "sum: " << sum << endl;

			// sort
			std::vector<std::pair<float, int> > weight;
			for(int k=0; k<n; k++){
				std::pair<float, int> w;
				w.first = x[k];
				w.second = linkIndex[k];
				weight.push_back(w);
			}

			std::sort(weight.begin(),weight.end(), pairCompare);

			// disp
			std::vector<std::pair<float, int> > reduce_weight;
			for(int j=0; j<n; j++){
				if(/*weight[j].first > 0.00001 &&*/ j<10){
					//cout << weight[j].second << "	" << weight[j].first << endl;
					reduce_weight.push_back(weight[j]);
				}
			}

			reOptimization(index, reduce_weight);

			string address = "data/weight/sample_";
			exportSkinWeight(index, address, reduce_weight);
		}
	//}
}

void ofxHairWeightOptimization::reOptimization(int index, std::vector<std::pair<float, int> > &r_weight)
{
	Eigen::MatrixXd am;
	Eigen::MatrixXd bv;	
	vector<int> linkIndex;
	Matrix<double> G, CE, CI;
	Vector<double> g0, ce0, ci0, x;
	int n, m, p;
	double sum = 0.0;
	char ch;

	int boneNum = r_weight.size();
	bv.resize(1, 3*exampleNum);
	am.resize(boneNum, 3*exampleNum);

	// bv
	for(int i=0; i<exampleNum; i++)
	{		
		bv(0, i*3+0) = m_normalParticles[index].m_currPos[i].x/100;
	  	bv(0, i*3+1) = m_normalParticles[index].m_currPos[i].y/100;
	  	bv(0, i*3+2) = m_normalParticles[index].m_currPos[i].z/100;
	}

	// am
	for(int j=0; j<boneNum; j++){
		for(int m=0; m<m_guideParticle.size(); m++){
			if(r_weight[j].second == m_guideParticle[m].m_index){
				ofMatrix4x4 mat0 = m_guideParticle[m].q_frame[0];
				for(int k=0; k<exampleNum; k++){
					ofMatrix4x4 mat = m_guideParticle[m].q_frame[k];
					ofVec3f p = m_normalParticles[index].m_initPos * mat0.getInverse() * mat;
					am(j, k*3+0) = p.x/100;
					am(j, k*3+1) = p.y/100;
					am(j, k*3+2) = p.z/100;
				}
			}
		}
	}

	Eigen::MatrixXd am1;
	am1.resize(exampleNum, 3*boneNum);

	// change
	// ng : bonenum > exampleNum
	for(int i=0; i<boneNum; i++){
		for(int j=0; j<exampleNum; j++){
			am1(j, i*3+0) = am(i, j*3+0);
			am1(j, i*3+1) = am(i, j*3+1);
			am1(j, i*3+2) = am(i, j*3+2);
	  	}
	}

	am = am1;

	linkIndex.resize(boneNum);
	for(int i=0; i<boneNum; i++){
		linkIndex[i] = r_weight[i].second;
		//linkIndex[i] = m_normalParticles[index].guideParticles[i].m_index;
	}


	Eigen::MatrixXd gMat = am * am.transpose();
	Eigen::VectorXd gVec = - am * bv.transpose();

	n = boneNum;
	G.resize(n, n);
	{
		for (int i = 0; i < n; i++)	
			for (int j = 0; j < n; j++)
				G[i][j] = gMat(i,j); //am(i,j);
	}
			
	g0.resize(n);
	{
		for (int i = 0; i < n; i++)
			g0[i] = gVec(i);
	}
		  
	m = 1;
	CE.resize(n, m);
	{
		for (int i = 0; i < n; i++)
			for (int j = 0; j < m; j++)
				CE[i][j] = 1.0;
	} 
		  
	ce0.resize(m);
	{
		for (int j = 0; j < m; j++)
			ce0[j] = -1.0;
	}
			
	p = n;
	CI.resize(n, p);
	{ 
		for (int i = 0; i < n; i++)
			for (int j = 0; j < p; j++)
				if(i==j){
					CI[i][j] = 1.0;
				}else{
					CI[i][j] = 0.0;	
				}
	}

	ci0.resize(p);
	{
		for (int j = 0; j < p; j++)
			ci0[j] = 0.0;
	}

	x.resize(n);

	std::cout << "f: " << solve_quadprog(G, g0, CE, ce0, CI, ci0, x) << std::endl;

	//cout <<  "x" << endl;

	for(int k=0; k<n; k++){
		if(x[k] > 0.0){
			sum += x[k];
		}
	}

	cout << "sum: " << sum << endl;

	// sort
	std::vector<std::pair<float, int> > weight;
	for(int k=0; k<n; k++){
		std::pair<float, int> w;
		w.first = x[k];
		w.second = linkIndex[k];
		weight.push_back(w);
	}

	std::sort(weight.begin(),weight.end(), pairCompare);

	// disp
	r_weight.clear();
	for(int j=0; j<n; j++){
		//if(weight[j].first > 0.00001){
			r_weight.push_back(weight[j]);
		//}
	}
}

void ofxHairWeightOptimization::exportSkinWeight(int j, string address, vector<pair<float, int> > weight)
{
	// output function
	ofstream ofs2;
	string exportfile = ofToString(j) + ".skinweight";
	ofs2.open(address + exportfile);

	ofs2 << m_normalParticles[j].index << endl;
	ofs2 << weight.size() << endl;

	for(auto w : weight){
		ofs2 << w.second << " " << w.first << endl;
	}

	// calc sum
	float sum = 0.0f;
	for(auto w : weight){
		sum += w.first;
	}
	ofs2 << "sum " << sum << endl;
}

bool ofxHairWeightOptimization::loadTempFile(const int index, Eigen::MatrixXd& m_am, Eigen::MatrixXd& m_bv, vector<int>& linkIndex)
{
	Eigen::MatrixXd am;
	Eigen::MatrixXd bv;
	int boneNum;

	string filename = "data/weight/sample_" + ofToString(index) + ".skinweight";
	ifstream ofs(filename);
	if(ofs.fail()){
		cout << "failed" << endl;
		return false;
	}
	string str;
	getline(ofs, str);
	exampleNum = ofToFloat(str);

	getline(ofs, str);
	boneNum = ofToInt(str);

	if(boneNum==0){
		return false;
	}

	getline(ofs, str);
	vector<string> indices = ofxHairUtil::split(str, ' ');

	linkIndex.resize(indices.size()-1);
	int i=0;
	for(auto s : indices){
		if(i==0){
			int index = ofToInt(s);
		}else{
			linkIndex[i-1] = ofToInt(s);
		}
		i++;
	}

	// header
	bv.resize(1, 3*exampleNum);
	am.resize(boneNum, 3*exampleNum);

	int j = 0;
	while(getline(ofs, str)) {
		vector<string> indices = ofxHairUtil::split(str, ' ');

		bv(0, j*3+0) = ofToFloat(indices[0]);
	  	bv(0, j*3+1) = ofToFloat(indices[1]);
	  	bv(0, j*3+2) = ofToFloat(indices[2]);

	  	int k=0;
		for(int i=3; i<indices.size(); i+=3){
			am(j, k*3+0) = ofToFloat(indices[i+0]);
			am(j, k*3+1) = ofToFloat(indices[i+1]);
			am(j, k*3+2) = ofToFloat(indices[i+2]);
			k++;
	  	}
		j++;
	}

	m_am = am;
	m_bv = bv;
	return true;
}

void ofxHairWeightOptimization::exportTempFile(const int index, Eigen::MatrixXd am, Eigen::MatrixXd bv)
{
	int boneNum = m_normalParticles[index].guideParticles.size();

	string filename = "data/weight/sample_" + ofToString(index) + ".skinweight";
	ofstream ofs(filename);
	if (ofs.fail()) {
        std::cerr << "failed" << std::endl;
    }

	ofs << exampleNum << endl;
	ofs << boneNum << endl;

	ofs << m_normalParticles[index].index << " ";
	for(int i=0; i<boneNum; i++){
		ofs << m_normalParticles[index].guideParticles[i].m_index << " ";
	}
	ofs << endl;

	for(int i=0; i<exampleNum; i++){
		ofs << bv(0, i*3+0) << " " << bv(0, i*3+1) << " " << bv(0, i*3+2) << " ";
		for(int j=0; j<boneNum; j++){
			ofs << am(i, j*3+0) << " " << am(i, j*3+1) << " " << am(i, j*3+2) << " ";
		}
		ofs << endl;
	}


	//for(int i=0; i<boneNum; i++){
	//	ofs << bv(0, i*3+0) << " " << bv(0, i*3+1) << " " << bv(0, i*3+2) << " ";
	//	for(int j=0; j<exampleNum; j++){
	//		ofs << am(i, j*3+0) << " " << am(i, j*3+1) << " " << am(i, j*3+2) << " ";
	//	}
	//	ofs << endl;
	//}

	ofs.close();
}
