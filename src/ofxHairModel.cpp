#include "ofxHairModel.h"

void ofxHairModel::addHairStrand(const ofVec3f position, const ofVec3f normal, const float length, const int resolution)
{
	ofxHairStrand s;
	s.setup(resolution, length, position, normal);
	s.setDiableParticle(0); // stick on the hairline

	strands.push_back(s);
	m_numStrands = strands.size();

	int count = 0;
	for(int i=0; i<m_numStrands; i++){
		count += strands[i].m_resolution;
	}
	m_numParticles = count;
}

bool ofxHairModel::loadHairModel(string filename)
{
	// This format is created by 
	// "A Reduced Model for Interactive Hairs", SIGGRAPH 2014
	strands.clear();

    std::fstream file;
    char buf[16];

	file.open(filename, std::ios::in | std::ios::binary);
    if ( !file.is_open() ) {
        cout <<  "file open error" << endl;
		return false;
    }

	// int : total particle size
    file.read((char*)&m_numParticles,sizeof(m_numParticles));
	std::cout << "Number of particles : " << m_numParticles << std::endl;

	ofxHairParticle *particles;
	particles = new ofxHairParticle [m_numParticles];

	// total particle size * float * 3: all particle positions
	for(int i=0; i<m_numParticles; i++){
		float pos[3];
		file.read( (char*)&pos, sizeof(pos));
		particles[i].position = ofVec3f(pos[0], -pos[1], -pos[2]);
		particles[i].position0 = ofVec3f(pos[0], -pos[1], -pos[2]);

		//cout << i << '\t' << pos[0] << "," << pos[1] << "," << pos[2] << endl; 
	}

	// int : total strand size
    file.read((char*)&m_numStrands, sizeof(m_numStrands));
	std::cout << "Number of strands : " << m_numStrands << std::endl;
	strands.resize(m_numStrands);
    
	// total strand size * int : particle size for each strand (use this to get the offset in all particle positions)
	for(int i=0; i<m_numStrands; i++){
		int resolution;
		file.read( (char*)&resolution, sizeof(resolution) );
		strands[i].m_resolution = resolution;
		strands[i].m_length.resize(strands[i].m_resolution);
		strands[i].m_particles.resize(resolution);

		//cout << i << '\t' << resolution << endl; 
	}

    file.close();

	int count = 0;
	for(int i=0; i<strands.size(); i++){

		// set random color
		ofColor c;
		c.setHsb(ofRandom(0, 255), 200, 255, 255);
		
		for(int j=0; j<strands[i].m_resolution; j++){
			ofxHairParticle p(particles[count].position, 1.0);
			strands[i].m_particles[j] = p;
			strands[i].m_particles[j].color = c;
			
			if(j==0){
				strands[i].setDiableParticle(0);
			}else{
				strands[i].m_length[j-1] = (strands[i].m_particles[j-1].position - strands[i].m_particles[j].position).length();
			}

			count++;
		}
		//buildJointHierarchy();
		//buildJointMatrix();
	}

	return true;
}


bool ofxHairModel::exportHairModel(string filename)
{
	std::cout << "exporting... " << std::endl;

    std::fstream fout;
    char buf[16];

	fout.open(filename, std::ios::out | std::ios::binary | ios::trunc);
    if ( !fout.is_open() ) {
        cout <<  "file open error" << endl;
		return false;
    }

	// int : total particle size
	fout.write( (char*) &m_numParticles, sizeof(int) );

	// total particle size * float * 3: all particle positions
	for(int i=0; i<m_numStrands; i++){
		for(int j=0; j<strands[i].getResolution(); j++){
			float pos[3];
			pos[0] = strands[i].m_particles[j].position[0];
			pos[1] = strands[i].m_particles[j].position[1];
			pos[2] = strands[i].m_particles[j].position[2];
			fout.write( (char*) &pos, sizeof(pos) );
		}
	}
	
	// int : total strand size
	fout.write( (char*) &m_numStrands, sizeof(int) );

	// total strand size * int : particle size for each strand (use this to get the offset in all particle positions)
	for(int i=0; i<m_numStrands; i++){
		int resolution = strands[i].getResolution();
		fout.write( (char*) &resolution, sizeof(int) );
	}

	fout.close();  //ファイルを閉じる
	std::cout << "saved as... '" << filename  << "'" << std::endl;
	return true;
}


bool ofxHairModel::loadHairModelAsText(string filename)
{
    std::fstream file;
    char buf[16];

	file.open(filename, std::ios::in);
    if ( !file.is_open() ) {
        cout <<  "file open error" << endl;
		return false;
    }

	// int : total particle size
	file >> m_numParticles;
	std::cout << "Number of particles : " << m_numParticles << std::endl;

	ofxHairParticle *particles;
	particles = new ofxHairParticle [m_numParticles];

	// total particle size * float * 3: all particle positions
	for(int i=0; i<m_numParticles; i++){
		float pos[3];
		file >> pos[0] >> pos[1] >> pos[2];
		particles[i].position = ofVec3f(pos[0], pos[1], pos[2]);
		particles[i].tmp_position = ofVec3f(pos[0], pos[1], pos[2]);
		//std::cout << pos[0] << "," << pos[1] << "," << pos[2] << endl;
	}

	// int : total strand size
	file >> m_numStrands;
	std::cout << "Number of strands : " << m_numStrands << std::endl;
	strands.resize(m_numStrands);
    
	// total strand size * int : particle size for each strand (use this to get the offset in all particle positions)
	for(int i=0; i<m_numStrands; i++){
		int resolution;
		file >> resolution;
		strands[i].m_resolution = resolution;
		strands[i].m_length.resize(strands[i].m_resolution);
		strands[i].m_particles.resize(resolution);
	}

    file.close();

	int count = 0;
	for(int i=0; i<strands.size(); i++){

		// set random color
		ofColor c;
		c.setHsb(ofRandom(0, 255), 200, 255, 255);
		//c = ofColor((float)c.r/255, (float)c.g/255, (float)c.b/255);
		
		for(int j=0; j<strands[i].m_resolution; j++){
			strands[i].m_particles[j].position = particles[count].position;
			strands[i].m_particles[j].position0 = ofVec3f(particles[count].position.x, particles[count].position.y, particles[count].position.z);
			strands[i].m_particles[j].tmp_position = particles[count].tmp_position;
			strands[i].m_particles[j].color = c;
			
			if(j==0){
				strands[i].setDiableParticle(0);
			}else{
				strands[i].m_length[j-1] = (strands[i].m_particles[j-1].position - strands[i].m_particles[j].position).length();
			}

			count++;
		}

		buildJointHierarchy();
		buildJointMatrix();
	}

	return true;
}

bool ofxHairModel::loadGuideHair(string filename)
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
		strands[guideIndex].bGuideHair = true;
    }
	return true;
}

bool ofxHairModel::exportHairModelAsText(string filename)
{
	std::cout << "exporting... " << std::endl;

    std::fstream fout;
    char buf[16];

	fout.open(filename, std::ios::out | ios::trunc);
    if ( !fout.is_open() ) {
        cout <<  "file open error" << endl;
		return false;
    }

	// int : total particle size
	fout << m_numParticles << endl;
	
	// total particle size * float * 3: all particle positions
	for(int i=0; i<m_numStrands; i++){
		for(int j=0; j<strands[i].getResolution(); j++){
			float pos[3];
			pos[0] = strands[i].m_particles[j].position[0];
			pos[1] = strands[i].m_particles[j].position[1];
			pos[2] = strands[i].m_particles[j].position[2];
			fout << pos[0] << "\t" << pos[1] << "\t" << pos[2] << endl;
		}
	}
	
	// int : total strand size
	fout << m_numStrands << endl;

	// total strand size * int : particle size for each strand (use this to get the offset in all particle positions)
	for(int i=0; i<m_numStrands; i++){
		int resolution = strands[i].getResolution();
		fout << resolution << endl;
	}

	fout.close();  //ファイルを閉じる
	std::cout << "saved as... '" << filename  << "'" << std::endl;
	return true;
}

void ofxHairModel::buildJointHierarchy()
{
	// set root point of strand
	for(int sidx=0; sidx<strands.size(); sidx++)
	{
		// calculate offset and local matrix
		for(int i=0; i<strands[sidx].getResolution(); i++){
			if(i==0){
				strands[sidx].m_particles[i+1].isParent = true;
				strands[sidx].m_particles[i+1].isSite = false;
				strands[sidx].m_particles[i].parent = &strands[sidx].m_particles[i];
				strands[sidx].m_particles[i].child = &strands[sidx].m_particles[i+1];
			}else if(i<strands[sidx].getResolution()-1)	{
				strands[sidx].m_particles[i+1].isParent = false;
				strands[sidx].m_particles[i+1].isSite = false;
				strands[sidx].m_particles[i].parent = &strands[sidx].m_particles[i-1];
				strands[sidx].m_particles[i].child = &strands[sidx].m_particles[i+1];
			}else{
				strands[sidx].m_particles[i+1].isParent = false;
				strands[sidx].m_particles[i+1].isSite = true;
				strands[sidx].m_particles[i].parent = &strands[sidx].m_particles[i-1];
				strands[sidx].m_particles[i].child = &strands[sidx].m_particles[i];
			}
		}
	}
}

void ofxHairModel::buildJointMatrix()
{
	for(int sidx=0; sidx<strands.size(); sidx++)
	{
		// calculate offset and local matrix
		for(int i=0; i<strands[sidx].getResolution(); i++){

			if(i==0){
				ofVec3f v0(1.0, 0.0, 0.0);
				ofVec3f v1 = strands[sidx].m_particles[i+1].position0 - strands[sidx].m_particles[i].position0;

				ofQuaternion q;
				q.makeRotate(v0, v1);

				strands[sidx].m_particles[i].local_rotation0 = q;
				strands[sidx].m_particles[i].local_trans0 = ofVec3f::zero();
				strands[sidx].m_particles[i].global_rotation0 = q;
				strands[sidx].m_particles[i].global_trans0 = strands[sidx].m_particles[i].position;

			}else{
				ofVec3f p0 = strands[sidx].m_particles[i-1].position0;
				ofVec3f p1 = strands[sidx].m_particles[i+0].position0;
				ofVec3f p2 = strands[sidx].m_particles[i+1].position0;
				strands[sidx].m_particles[i].local_rotation0.makeRotate((p1-p0).normalized(), (p2-p1).normalized());
				strands[sidx].m_particles[i].local_trans0 = p2-p1;
				strands[sidx].m_particles[i].global_rotation0 = strands[sidx].m_particles[i-1].global_rotation0 * strands[sidx].m_particles[i].global_rotation0;
				strands[sidx].m_particles[i].global_trans0 = strands[sidx].m_particles[i-1].global_trans0 + strands[sidx].m_particles[i].global_trans0;
			}
		}
	}
}

void ofxHairModel::updateJointMatrix()
{
}

//
//void ofxHairModel::buildJointMatrix2()
//{
//	// set root point of strand
//	for(int sidx=0; sidx<strands.size(); sidx++)
//	{
//		ofPoint m_root_pos = strands[sidx].m_particles[0].position; // <- this particle is always movement each frame if the character is movement
//
//		// calculate offset and local matrix
//		for(int i=0; i<strands[sidx].getResolution()-1; i++){
//
//			ofMatrix4x4 mat;
//			mat.makeIdentityMatrix();
//			
//			// translate
//			ofVec3f v0(0.0, 1.0, 0.0);
//			ofVec3f v1 = strands[sidx].m_particles[i+1].position - strands[sidx].m_particles[i].position;
//			strands[sidx].m_particles[i].offset = ofVec3f::zero();
//
//			// rotate
//			ofQuaternion q;
//			q.makeRotate(v0, v1);
//
//			mat.glTranslate(strands[sidx].m_particles[i].position);
//			mat.glRotate(q);
//			strands[sidx].m_particles[i].global_matrix = mat;
//
//		}
//
//		for(int i=0; i<strands[sidx].getResolution(); i++){
//			if(!strands[sidx].m_particles[i].isSite){
//				strands[sidx].m_particles[i].local_matrix = strands[sidx].m_particles[i+1].global_matrix * strands[sidx].m_particles[i].global_matrix.getInverse();
//				strands[sidx].m_particles[i].local_matrix.setTranslation(strands[sidx].m_particles[i].offset);
//			}
//		}
//	}
//}
//
//void ofxHairModel::updateJointMatrix(ofxHairParticle &particle)
//{
//	ofVec3f translate;
//	ofMatrix4x4 rotate;
//
//	if(particle.isParent){
//		translate = particle.position;
//	}
//
//	translate += particle.offset;	// collect global position
//	rotate = particle.local_matrix;	// collect global rotation matrix
//	
//	ofMatrix4x4 mat;
//	mat.makeIdentityMatrix();
//
//	if(particle.isParent){
//		mat.glTranslate(translate);
//		mat.glRotate(rotate.getRotate());
//	}else{
//		mat.glTranslate(0.0, translate.length(), 0.0);
//		mat.glRotate(rotate.getRotate());
//		mat = mat * particle.parent->global_matrix;
//	}
//
//	particle.global_matrix = mat;
//
//	if(!particle.isSite){
//		updateJointMatrix(particle.child[0]);
//	}
//}