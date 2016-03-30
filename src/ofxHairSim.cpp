#include "ofxHairSim.h"

ofxHairSim::ofxHairSim()
{
	m_gravity.set(0.0, -9.8f, 0.0);
	m_damping = 0.9;
	m_timestep = 0.05;
	m_collisionType = ClosestPointOnTriangleKdTree;
	bFloorContact = false;
	m_floor_Y = -1000;
}

ofxHairSim::~ofxHairSim()
{
	//m_particles.clear();
	//m_particlesPos.clear();
	//delete hair;
}

void ofxHairSim::init(ofxHairModel &model)
{
	hair = &model; // link
	initModel = model;

	m_particles.clear();
	int num = hair->getNumParticles();
	m_particles.resize(num);
	m_particlesPos.resize(num);
	m_particlesPosition.resize(num);

	for(int i=0; i<hair->getNumStrand(); i++){
		for(int j=0; j<hair->strands[i].getResolution(); j++){
			m_particles[i*hair->strands[i].m_resolution+j] = &hair->strands[i].m_particles[j];
			m_particlesPos[i*hair->strands[i].m_resolution+j] = &m_particles[i*hair->strands[i].m_resolution+j]->tmp_position;
			m_particlesPosition[i*hair->strands[i].m_resolution+j] = m_particles[i*hair->strands[i].m_resolution+j]->tmp_position;
		}
	}
}

bool ofxHairSim::loadBoundaryOBJ(string filename)
{
	return boundary.load(filename);
}

void ofxHairSim::updateBoundary(vector<ofPoint> &points)
{
	boundary.setPosition(points);

#if 0
	for(int i=0; i<hair->getNumStrand(); i++){
		hair->strands[i].m_particles[0].position = points[i];
	}
#else
	// changing root position
	for(int i=0; i<hair->getNumStrand(); i++){
		hair->strands[i].m_particles[0].position = 
			  points[hair->strands[i].root_ratio.idx[0]] * hair->strands[i].root_ratio.s
			+ points[hair->strands[i].root_ratio.idx[1]] * hair->strands[i].root_ratio.t
			+ points[hair->strands[i].root_ratio.idx[2]] * hair->strands[i].root_ratio.u;
	}
#endif
}

void ofxHairSim::modelHairFitting(ofxHairModel& _hair, float radius)
{
	hair = &_hair;

	ofxNearestNeighbour3D nn;
	nn.buildIndex(boundary.m_points);

	for(auto& s : _hair.strands)
	{	
		ofPoint root_pos = s.m_particles[0].position;
	    vector<pair<NNIndex, float> > indices;
	    nn.findPointsWithinRadius(root_pos, radius, indices);	// the result is depended by radius parameter

		// calculate closest point
		float closest_dist = 1000000;
		Triangle closest_triangle;
		ofPoint closest_point;
		float weight[3];
		for(auto idx : indices){
			for(int i = 0; i < boundary.m_numNeighborTriangle[idx.first]; i++){
				Triangle t = boundary.m_neighborTriangle[idx.first][i];
				ofPoint collision_point;
				float w[3];
				float dist = ofxMeshUtil::point_triangle_distance2(root_pos, boundary.m_points[t.p1], boundary.m_points[t.p2], boundary.m_points[t.p3], collision_point, w[0], w[1], w[2]);
				if(dist < closest_dist){
					closest_dist = dist;
					closest_point = collision_point;
					closest_triangle = t;
					weight[0] = w[0];
					weight[1] = w[1];
					weight[2] = w[2];
				}
			}
		}

		// set
		s.root_ratio.s = weight[0];
		s.root_ratio.t = weight[1];
		s.root_ratio.u = weight[2];
		s.root_ratio.idx[0] = closest_triangle.p1;
		s.root_ratio.idx[1] = closest_triangle.p2;
		s.root_ratio.idx[2] = closest_triangle.p3;

		// translate
		ofVec3f trans(closest_point - root_pos);
		for(auto& p : s.m_particles)
		{
			p.position += trans;
			p.position0 = p.position;
		}
	}
}

void ofxHairSim::reset()
{
}

void ofxHairSim::update(ofMatrix4x4 mRotate)
{
	updateForce();
	updateVelocity();
	solveConstraint(mRotate);
	solveCollision();
	updatePositionAndVelocity();
}

void ofxHairSim::updateForce()
{
	for(auto& p : m_particles) {
		p->forces += m_gravity;
	}
}

void ofxHairSim::updateVelocity()
{
	float dt = m_timestep;

	for(auto p : m_particles) {

		if(!p->enabled) {
			p->tmp_position = p->position;
			continue;
		}

		p->velocity = p->velocity + dt * (p->forces * p->inv_mass);
		p->tmp_position += (p->velocity * dt);
		p->forces = ofVec3f(0.0);
		p->velocity *= 0.99;
	}
}

void ofxHairSim::solveCollision()
{
	switch (m_collisionType)
	{
		case ClosestPointOnTriangle:
			ofxHairCollision::collisionClosestPointOnTriangle(m_particles, boundary.m_points, boundary.m_normal, boundary.m_triangles);
			break;
		case ClosestPointOnTriangleFromVertex:
			ofxHairCollision::collisionClosestPointOnTriangleFromNearestVertex(m_particles, boundary.m_points, boundary.m_normal, boundary.m_numNeighborTriangle, boundary.m_neighborTriangle);
			break;
		case ClosestPointOnTriangleKdTree:
			ofxHairCollision::collisionClosestPointOnTriangle_ke_tree(m_particles, boundary.m_points, boundary.m_normal, boundary.m_numNeighborTriangle, boundary.m_neighborTriangle, m_collision.getRadius() );
			break;
		default:
			break;
	}
}

void ofxHairSim::solveConstraint(ofMatrix4x4 mRotate)
{
	// Han and Harada 2012
	for(int i=0; i<hair->getNumStrand(); i++) {
		for(int j=0; j<hair->strands[i].getResolution(); j++) {
			if(j==0){
				ofxHairParticle* pc = &hair->strands[i].m_particles[0];
				ofVec3f delta_x_i_global = mRotate * pc->position0 - pc->tmp_position;
				pc->tmp_position += delta_x_i_global;
			}else{
				float t = j / (float)hair->strands[i].getResolution();
				ofxHairParticle* pc = &hair->strands[i].m_particles[j];

				// global shape constraint
				float SgRoot = 0.3;	
				float SgTip = 0.1;
				float Sg = (1.0f - t) * SgRoot + t * SgTip; // Linear interpolation
				ofVec3f delta_x_i_global = Sg * (mRotate * pc->position0 - pc->tmp_position);
				pc->tmp_position += delta_x_i_global;
			}
		}
	}

	//applyLocalShapeConstrain(mRotate);

	for(int i=0; i<hair->getNumStrand(); i++) {
		for(int j=1; j<hair->strands[i].getResolution(); j++) {
			// distance constraint
			ofVec3f dir;
			ofVec3f curr_pos;
			ofxHairParticle* pa = &hair->strands[i].m_particles[j-1];
			ofxHairParticle* pb = &hair->strands[i].m_particles[j];

			curr_pos = pb->tmp_position;
			dir = pb->tmp_position - pa->tmp_position;
			dir.normalize();

			pb->tmp_position = pa->tmp_position + dir * hair->strands[i].getLength(j);
						
			// floor contact
			if(bFloorContact){
				if(pb->tmp_position.y < m_floor_Y)
					pb->tmp_position.y = m_floor_Y;
			}

			pb->d = curr_pos - pb->tmp_position;
		}
	}
}

void ofxHairSim::applyLocalShapeConstrain(ofMatrix4x4 headRot)
{
	const int numIteration =  4;
	float SlRoot = 1.0;
	float SlTip = 0.6;
	
	for(int sidx=0; sidx<hair->strands.size(); sidx++)
	{
		for(int iter = 0; iter < numIteration; iter++)
		{
			ofQuaternion rotGlobal = hair->strands[sidx].m_particles[0].local_rotation0;

			for(int i=0; i<hair->strands[sidx].getResolution()-1; i++)
			{
				ofVec3f pos = hair->strands[sidx].m_particles[i].tmp_position;
				float t = i / (float)hair->strands[sidx].getResolution();
				float Sl = (1.0f - t) * SlRoot + t * SlTip; // Linear interpolation
				ofVec3f pos_plus_one = hair->strands[sidx].m_particles[i+1].tmp_position;
				ofQuaternion rotGlobalWorld = headRot.getRotate() * rotGlobal;
				ofVec3f pos0_i_plus_1_local = hair->strands[sidx].m_particles[i+1].local_trans0;
				ofVec3f pos0_i_plus_1_world = rotGlobal * pos0_i_plus_1_local + pos;
				ofVec3f delta = 0.5f * Sl * (pos0_i_plus_1_world - pos_plus_one);

				if (hair->strands[sidx].m_particles[i].mass > 0.0f) {
					pos -= delta;
					hair->strands[sidx].m_particles[i].d += delta;
				}

				if (hair->strands[sidx].m_particles[i+1].mass > 0.0f) {
					pos_plus_one += delta;
					hair->strands[sidx].m_particles[i+1].d += delta;
				}

				// Update local/global frame
				//ofQuaternion invRotGlobalWorld = ofMatrix4x4(rotGlobalWorld).getInverse().getRotate();
				//ofVec3f vec = (pos_plus_one - pos).normalized();
				//ofVec3f x_i_plus_1_frame_i = (invRotGlobalWorld * vec).normalized();
				//ofVec3f e = ofVec3f(1.0f, 0.0f, 0.0f);
				//ofVec3f rotAxis = e.crossed(x_i_plus_1_frame_i);

				//if (rotAxis.length() > 0.001f) {
				//	float angleInRadian = acos(e.dot(x_i_plus_1_frame_i));
				//	rotAxis = rotAxis.normalized();
				//	ofQuaternion localRot = ofQuaternion(angleInRadian, rotAxis);
				//	rotGlobal = rotGlobal * localRot;
				//}

				hair->strands[sidx].m_particles[i].tmp_position = pos;
				hair->strands[sidx].m_particles[i+1].tmp_position = pos_plus_one;

				pos = pos_plus_one;
			}
		}
	}
}


void ofxHairSim::applyLocalShapeConstrain2(ofMatrix4x4 headRot)
{
	const int numIteration = 4;
	float SlRoot = 1.0f;
	float SlTip = 0.4f;
	
	for(int iter = 0; iter < numIteration; iter++)
	{
		for(int sidx=0; sidx<hair->strands.size(); sidx++)
		{
			ofQuaternion rotGlobal = hair->strands[sidx].m_particles[0].local_rotation0;
			ofVec3f pos = hair->strands[sidx].m_particles[1].tmp_position;

			for(int i=1; i<hair->strands[sidx].getResolution()-1; i++)
			{
				float t = i / (float)hair->strands[sidx].getResolution();
				float Sl = (1.0f - t) * SlRoot + t * SlTip; // Linear interpolation

				ofVec3f delta = hair->strands[sidx].m_particles[i].position0 - hair->strands[sidx].m_particles[i].position;
				hair->strands[sidx].m_particles[i].tmp_position -= 0.5f * Sl * delta;
				hair->strands[sidx].m_particles[i+1].tmp_position += 0.5 * Sl * delta;
			}
		}
	}
}
void ofxHairSim::updatePositionAndVelocity()
{
	float dt = m_timestep;

	for(int i=0; i<hair->getNumStrand(); i++) {
		for(int j=1; j<hair->strands[i].getResolution(); j++) {
			ofxHairParticle* pa = &hair->strands[i].m_particles[j-1];
			ofxHairParticle* pb = &hair->strands[i].m_particles[j];
			if(!pa->enabled) {
				continue;
			}
			pa->velocity = ((pa->tmp_position - pa->position) / dt) + m_damping *  (pb->d / dt);
			pa->position = pa->tmp_position;
		}
		ofxHairParticle* last = &hair->strands[i].m_particles.back();
		last->position = last->tmp_position;
	}
}