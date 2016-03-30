#include "ofxHairSkeleton.h"

void ofxHairSkeleton::loadHairStrand(ofxHairStrand strand)
{
	// clear
	m_joints.clear();
	m_joints0.clear();

	// resize
	m_numJoints = strand.m_resolution;
	m_joints.resize(m_numJoints);
	m_joints0.resize(m_numJoints);

	buildJointHierarchy();
	buildJointMatrix2(strand);
}

void ofxHairSkeleton::buildJointHierarchy()
{
	for(int i=0; i<m_numJoints; i++){
		if(i==0){
			m_joints[i].bParent = true;
			m_joints[i].bSite = false;
			m_joints[i].setChildJoint(&m_joints[i+1]);
		}else if(i < m_numJoints-1) {
			m_joints[i].bParent = false;
			m_joints[i].bSite = false;
			m_joints[i].setParentJoint(&m_joints[i-1]);
			m_joints[i].setChildJoint(&m_joints[i+1]);
		}else{
			m_joints[i].bParent = false;
			m_joints[i].bSite = true;
			m_joints[i].setParentJoint(&m_joints[i-1]);
		}
	}
}

void ofxHairSkeleton::buildJointMatrix(ofxHairStrand &strand)
{
	// set root point of strand 
	m_root_pos = strand.m_particles[0].position; //<- this particle is always movement each frame if the character is movement

	// calculate offset and local matrix
	for(int i=0; i<m_numJoints; i++){

		if(i==0){
			ofMatrix4x4 mat;
			mat.makeIdentityMatrix();
			
			// translate
			ofVec3f v0(0.0, 1.0, 0.0);
			ofVec3f v1 = strand.m_particles[i+1].position - strand.m_particles[i].position;
			m_joints[i].setOffset(ofVec3f::zero());

			// rotate
			ofQuaternion q;
			q.makeRotate(v0, v1);

			//mat.glTranslate(m_joints[i].getOffset());
			mat.glRotate(q);
			m_joints[i].setLocalMatrix(mat);

		}else if(i < m_numJoints-1) {
			ofMatrix4x4 mat;
			mat.makeIdentityMatrix();

			// translate
			ofVec3f v0 = strand.m_particles[i].position - strand.m_particles[i-1].position;
			ofVec3f v1 = strand.m_particles[i+1].position - strand.m_particles[i].position;
			m_joints[i].setOffset(v0);

			// rotate
			ofQuaternion q;
			q.makeRotate(v0, v1);

			//mat.glTranslate(m_joints[i].getOffset());
			mat.glRotate(q);
			m_joints[i].setLocalMatrix(mat);

		}else{
			ofMatrix4x4 mat;
			mat.makeIdentityMatrix();

			// translate
			ofVec3f v0 = strand.m_particles[i].position - strand.m_particles[i-1].position;
			m_joints[i].setOffset(v0);

			//mat.glTranslate(m_joints[i].getOffset());			
			m_joints[i].setLocalMatrix(mat);
		}
	}

	updateJoint(m_joints[0]);
}

void ofxHairSkeleton::buildJointMatrix2(ofxHairStrand &strand)
{
	// set root point of strand
	m_root_pos = strand.m_particles[0].position; // <- this particle is always movement each frame if the character is movement

	// calculate offset and local matrix
	for(int i=0; i<m_numJoints; i++){

		ofMatrix4x4 mat;
		mat.makeIdentityMatrix();
			
		// translate
		ofVec3f v0(0.0, 1.0, 0.0);
		ofVec3f v1 = strand.m_particles[i+1].position - strand.m_particles[i].position;
		m_joints[i].setOffset(ofVec3f::zero());

		// rotate
		ofQuaternion q;
		q.makeRotate(v0, v1);

		mat.glTranslate(strand.m_particles[i].position);
		mat.glRotate(q);
		m_joints[i].setGrobalMatrix(mat);
		
		//cout << "------------------------" << endl;
		//cout << m_joints0[i].getGlobalMatrix() << endl;
		//cout << m_joints[i].getGlobalMatrix() << endl;
		//cout << m_joints[i].getGlobalMatrix() * m_joints0[i].getGlobalMatrix().getInverse() << endl;
		

	}

	buildLocalMatrix();

	//updateJoint(m_joints[0]);
}

void ofxHairSkeleton::buildLocalMatrix()
{	
	m_joints[0].setLocalMatrix(m_joints[0].getGlobalMatrix());

	// calculate local matrix
	for(int i=1; i<m_numJoints; i++){
		ofVec3f vec0 = m_joints0[i].getPosition() - m_joints0[i-1].getPosition();
		ofVec3f dir0 = vec0.normalized();

		ofVec3f vec = m_joints[i].getPosition() - m_joints[i-1].getPosition();
		ofVec3f dir = vec.normalized();

		ofMatrix4x4 local_matrix;
		ofQuaternion q;
		q.makeRotate(dir0, dir);
		ofVec3f pos = vec - vec0;

		local_matrix.setRotate(q);
		local_matrix.setTranslation(vec0);
		m_joints[i].setLocalMatrix(local_matrix);

		//ofMatrix4x4 mat;
		//mat = m_joints[i].getGlobalMatrix() * m_joints[i-1].getGlobalMatrix().getInverse();
		////mat.setTranslation(ofVec3f(0.0)); // remove translate features
		//m_joints[i].setLocalMatrix(mat);
		//m_joints[i].setOffset(m_joints[i].getPosition() - m_joints[i-1].getPosition());
	}

	//m_joints[0].setLocalMatrix(m_joints[0].getGlobalMatrix());

	//// calculate local matrix
	//for(int i=1; i<m_numJoints; i++){
	//	ofMatrix4x4 mat;
	//	mat = m_joints[i].getGlobalMatrix() * m_joints[i-1].getGlobalMatrix().getInverse();
	//	mat.setTranslation(ofVec3f(0.0)); // remove translate features
	//	m_joints[i].setLocalMatrix(mat);
	//	m_joints[i].setOffset(m_joints[i].getPosition() - m_joints[i-1].getPosition());
	//}
}

void ofxHairSkeleton::updateJoint(ofxHairJoint &joint)
{
	ofVec3f translate;
	ofMatrix4x4 rotate;

	if(joint.isParent()){
		translate = m_root_pos;
	}

	translate += joint.getOffset();	// collect global position
	rotate = joint.getLocalMatrix();	// collect global rotation matrix
	
	ofMatrix4x4 mat;
	mat.makeIdentityMatrix();

	if(joint.isParent()){
		mat.glTranslate(translate);
		mat.glRotate(rotate.getRotate());
	}else{
		mat.glTranslate(0.0, translate.length(), 0.0);
		mat.glRotate(rotate.getRotate());
		mat = mat * joint.getParentJoint()->getGlobalMatrix();
	}

	joint.setGrobalMatrix(mat);

	if(!joint.isSite()){
		updateJoint(joint.getChildJoint()[0]);
	}
}

void ofxHairSkeleton::drawAxis()
{
	for(int i=0; i<m_numJoints; i++){
		ofPushMatrix();	
		glMultMatrixf(m_joints[i].getGlobalMatrix().getPtr());
		ofDrawAxis(2.0);
		ofPopMatrix();
	}
}