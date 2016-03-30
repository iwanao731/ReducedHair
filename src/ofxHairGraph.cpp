#include "ofxHairGraph.h"

ofxHairGraph::ofxHairGraph()
{
	m_resolution = 25;
}

bool ofxHairGraph::loadGraph(const string filename)
{
	ifstream ofs(filename);
	if (ofs.fail()) {
        std::cerr << "failed" << std::endl;
    }
	
	std::string str;

	// header
	getline(ofs, str);
	vector<string> header = ofxHairUtil::split(str, ' ');
	setNumNodes(ofToInt(header[0]));
	setNumEdges(ofToInt(header[1]));
	m_nodes.resize(getNumNodes());

	// contents
	int i=0;
	while (getline(ofs, str)) {

		// initialize
		ofxHairNode n;
		n.setIndex(i);
		n.setBoolGuideHair(false);

		vector<string> indices = ofxHairUtil::split(str, ' ');
		for(auto s : indices){
			n.addEdge(i, ofToInt(s)-1);	// -1 is important
		}
		m_nodes[i] = n;
		i++;
    }
	return true;
}

bool ofxHairGraph::loadWeight(const string filename)
{
	ifstream ofs(filename);
	if (ofs.fail()) {
        std::cerr << "failed" << std::endl;
    }
	
	std::string str;

	int i=0;
	while (getline(ofs, str)) {
		vector<string> indices = ofxHairUtil::split(str, ' ');
		int i = ofToInt(indices[0]);
		int j = ofToInt(indices[1]);
		float weight = ofToFloat(indices[2]);
		m_nodes[i].setEdgeWeight(j, weight);
		i++;
    }
	return true;
}

bool ofxHairGraph::loadGuide(const string filename, int resolution)
{
	// import group file
	ifstream ofs(filename);
	if (ofs.fail()) {
        std::cerr << "failed" << std::endl;
		return false;
    }

	std::string str;
	getline(ofs, str);
	int num = ofToInt(str);

	while (getline(ofs, str)) {
		int guideIndex = ofToInt(str);

		for(int i=resolution*guideIndex; i<resolution*guideIndex+resolution; i++){
			m_nodes[i].setBoolGuideHair(true);
		}
    }
	return true;
}
bool ofxHairGraph::loadGroup(const string filename)
{
	// import group file
	ifstream ofs(filename);
	if (ofs.fail()) {
        std::cerr << "failed" << std::endl;
		return false;
    }
	
	std::string str;
	getline(ofs, str);
	m_groups.resize(ofToInt(str));

	int i = 0;
	while (getline(ofs, str)) {
		m_nodes[i].setIndex(i);
		m_nodes[i].setStrandIndex(i/m_resolution);
		m_nodes[i].setGroupIndex(ofToInt(str));
		m_groups[ofToInt(str)].addNode(m_nodes[i]);
		i++;
    }
	return true;
}

void ofxHairGraph::init()
{

}

void ofxHairGraph::setupGroupLink()
{
	vector<vector<int> > group;
	group.resize(m_groups.size());

	for(auto &n : m_nodes){
		int index = n.getGroupIndex();
		for(auto e : n.getEdge()){
			int neighborIndex = m_nodes[e.index2].getGroupIndex();
			if(index != neighborIndex){
				group[index].push_back(neighborIndex);
			}
		}
	}

	int i=0;
	for(auto &g : group){
		sort(g.begin(), g.end());
		g.erase(std::unique(g.begin(), g.end()), g.end());
		for(auto l : g){
			m_groups[i].addLinkGroupIndex(l);
		}
		i++;
	}

	//i = 0;
	//for(auto g : m_groups){
	//	cout << i << " : ";
	//	for(auto l : g.getLinkGroupIndex()){
	//		cout << l << ",";
	//	}
	//	cout << endl;
	//	i++;
	//}
}

void ofxHairGraph::setupGuideLink()
{
	for(auto &n : m_nodes){

		// in the group
		int gIndex = n.getGroupIndex();
		for(auto p : m_groups[gIndex].getNode()){
			if(p.getBoolGuideHair())
				n.addGuideLink(p.getIndex());
		}

		// neighbor group
		for(auto& g : m_groups[gIndex].getLinkGroupIndex()){
			for(auto& nn : m_groups[g].getNode()){
				if(nn.getBoolGuideHair()){
					n.addGuideLink(nn.getIndex());
				}
			}
		}
	}
}

void ofxHairGraph::setupNormalNum()
{
	int count = 0;
	for(auto &n : m_nodes){
		if(!n.getBoolGuideHair()){
			count++;
		}
	}
	m_numNormalNodes = count;

}
void ofxHairGraph::calculateMaximumWeight()
{
	int i=0;
	for(auto g : m_groups){
		int max=0;
		int index=0;
		for(auto p : g.getNode()){
			for(auto e : p.getEdge()){
				if(max < e.getWeight()){
					max = e.getWeight();
					index = p.getIndex();
				}
			}
		}
		m_groups[i].setWeightestNodeIndex(index);
		g.setWeightestNodeIndex(index);
		i++;
	}

	int j=0;
	for(auto g : m_groups){
		int index = g.getWeightestNodeIndex();
		int strandIndex = index/m_resolution;
		for(int i=strandIndex*m_resolution; i<strandIndex*m_resolution+m_resolution; i++){
			//m_nodes[i].setBoolGuideHair(true);
			m_nodes[i].setStrandIndex(strandIndex);
		}
		m_groups[j].setGuideHairIndex(strandIndex);
		j++;
	}
}

void ofxHairGraph::calculateEnergyFunction()
{
	for(auto &g : m_groups){
		int index = g.getWeightestNodeIndex();
		int strandIndex = m_nodes[index].getStrandIndex();
		float sumWeight = 0;
		for(int i=strandIndex*m_resolution; i<strandIndex*m_resolution+m_resolution; i++){
			for(auto e : m_nodes[i].getEdge()){
				sumWeight += e.getWeight();
			}
		}
		g.setMaximumEnergyValue(sumWeight);
	}

	int count;
	int sumCount;
	int finish = 0;
	do{
		count = 0;
		for(auto &g : m_groups){
			float weight = 0.0f;
			int nodeIndex = 0;
			int strandIndex = 0;
			for(auto n : g.getNode()){
				float w = 0.0f;
				int sIndex = n.getStrandIndex();
				for(int i=sIndex*m_resolution; i<sIndex*m_resolution+m_resolution; i++){
					for(auto e : m_nodes[i].getEdge()){
							w += e.getWeight();
					}
				}
				
				if(weight < w){
					weight = w;
					strandIndex = sIndex;
					nodeIndex = n.getIndex();
				}
			}
			g.setWeightestNodeIndex(nodeIndex);
			if(g.getGuideHairIndex() == strandIndex){
				count++;
			}else{
				g.setGuideHairIndex(strandIndex);
				for(int i=strandIndex*m_resolution; i<strandIndex*m_resolution+m_resolution; i++){
					m_groups[m_nodes[i].getGroupIndex()].setGuideHairIndex(strandIndex);
				}
			}
		}
		std::cout << "iterate: " << count << std::endl;
		if(sumCount == count){
			finish++;
		}else{
			finish = 0;
		}

		sumCount = count;
	} while (finish < 10);
}

void ofxHairGraph::sortStrandByEnergyValue()
{
	for(auto &g : m_groups){
		int index = g.getGuideHairIndex();
		float w = 0.0f;
		for(int i=index*m_resolution; i<index*m_resolution+m_resolution; i++){
			for(auto e : m_nodes[i].getEdge()){
				w += e.getWeight();
			}
		}
		pair<int,int> p;
		p.first = w;
		p.second = index;
		adaptive_strands.push_back(p);
	}

	////int count = 0;
	//for(auto &g : m_groups){
	//	float weight = 0.0f;
	//	int nodeIndex = 0;
	//	int strandIndex = 0;
	//	for(auto n : g.getNode()){
	//		if(n.getBoolGuideHair()){
	//			float w = 0.0f;
	//			int sIndex = n.getStrandIndex();
	//			for(int i=sIndex*m_resolution; i<sIndex*m_resolution+m_resolution; i++){
	//				for(auto e : m_nodes[i].getEdge()){
	//						w += e.getWeight();
	//				}
	//			}
	//			adaptive_strands[sIndex].first = w;
	//			adaptive_strands[sIndex].second = sIndex;				
	//		}
	//	}
	//}

	std::sort(adaptive_strands.begin(), adaptive_strands.end());
	//std::sort(adaptive_strands.begin(), adaptive_strands.end(), greater<pair<int,int> >());

	int i=0;
	for(auto s : adaptive_strands)
	{
		if(s.first > 0){
			cout << s.second << " : " << s.first << endl;
		}
		i++;
	}
}

vector<int> ofxHairGraph::getSortedGuideStrandIndex(int numAdaptiveHair)
{
	vector<int> indices;
	int count = 0;
	for(auto s : adaptive_strands) {
		if(count < numAdaptiveHair){
			indices.push_back(s.second);
			count++;
		}
	}
	return indices;
}

void ofxHairGraph::debugGuideHairColor(ofxHairModel &model, int numAdaptiveHair)
{
	vector<int> guideHairIndex;
	guideHairIndex = getSortedGuideStrandIndex(numAdaptiveHair);

	for(int i=0; i<model.strands.size(); i++){
		model.strands[i].bGuideHair = false;
		for(int j=0; j<model.strands[i].m_particles.size(); j++){
			model.strands[i].m_particles[j].color = ofColor(60,60,60,20);
		}
	}
	int count = 0;
	for(auto i : guideHairIndex){
		ofColor c;
		c = ofColor(255, 0, 0);
		//c.setHsb(ofMap(count, 0, guideHairIndex.size(), 0, 255), 200, 255);
		model.strands[i].bGuideHair = true;
		for(int j=0; j<model.strands[i].m_particles.size(); j++){
			model.strands[i].m_particles[j].color = c;
		}
		count++;
	}
}

bool ofxHairGraph::exportGuideHair(string filename)
{
	cout << "export guide hair index : " << filename << endl;
	ofstream ofs(filename);
	vector<int> guideHairIndex;
	for(auto g : m_groups){
		guideHairIndex.push_back(g.getGuideHairIndex());
	}

	std::sort(guideHairIndex.begin(), guideHairIndex.end());
	guideHairIndex.erase(std::unique(guideHairIndex.begin(), guideHairIndex.end()), guideHairIndex.end());

	ofs << guideHairIndex.size() << endl;

	for(auto i : guideHairIndex){
		ofs << i << std::endl;
	}

	ofs.close();
	cout << "finished export guide hair index" << endl;
	return true;
}

void ofxHairGraph::debugSetGroupColor(ofxHairModel &model)
{
	vector<int> guideHairIndex;
	for(auto g : m_groups){
		guideHairIndex.push_back(g.getGuideHairIndex());
	}

	std::sort(guideHairIndex.begin(), guideHairIndex.end());
	guideHairIndex.erase(std::unique(guideHairIndex.begin(), guideHairIndex.end()), guideHairIndex.end());

	for(int i=0; i<model.strands.size(); i++){
		model.strands[i].bGuideHair = false;
		for(int j=0; j<model.strands[i].m_particles.size(); j++){
			model.strands[i].m_particles[j].color = ofColor(60,60,60,20);
		}
	}
	int count = 0;
	for(auto i : guideHairIndex){
		ofColor c;
		c.setHsb(ofMap(count, 0, guideHairIndex.size(), 0, 255), 200, 255);
		model.strands[i].bGuideHair = true;
		for(int j=0; j<model.strands[i].m_particles.size(); j++){
			model.strands[i].m_particles[j].color = c;
		}
		count++;
	}
}

void ofxHairGraph::draw(ofxHairModel &model)
{
	ofSetColor(255, 255, 0);
	for(auto &n : m_nodes){
		for(auto e : n.getEdge()){
//			if(m_nodes[e.index1].getBoolGuideHair() + m_nodes[e.index2].getBoolGuideHair() == 1){
				glBegin(GL_LINES);
				ofPoint p0 = model.strands[e.index1/25].m_particles[e.index1%25].position;
				ofPoint p1 = model.strands[e.index2/25].m_particles[e.index2%25].position;
				glVertex3d(p0.x, p0.y, p0.z);
				glVertex3d(p1.x, p1.y, p1.z);
				glEnd();
			//}
		}
	}
}

void ofxHairGraph::drawLink(ofxHairModel &model, int &index)
{
	glPointSize(7.0);
	glBegin(GL_POINTS);
	ofPoint p0 = model.strands[index/25].m_particles[index%25].position;
	ofSetColor(255, 0, 0);
	glVertex3d(p0.x, p0.y, p0.z);
	ofSetColor(255, 255, 0);
	cout << index << ": ";
	for(auto &link : m_nodes[index].getGuideLink()){
		ofPoint p = model.strands[link/25].m_particles[link%25].position;
		glVertex3d(p.x, p.y, p.z);
		cout << link << " ";
	}
	cout << endl;
	glEnd();
}
