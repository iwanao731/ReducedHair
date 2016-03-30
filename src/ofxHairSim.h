#ifndef _OFX_HAIR_SIM_
#define _OFX_HAIR_SIM_

#include "ofMain.h"
#include "ofxHairModel.h"
#include "ofxHairBoundary.h"
#include "ofxHairCollision.h"
#include "ofxNearestNeighbour.h"

enum CollisionType {
	ClosestPointOnTriangle = 1,
	ClosestPointOnTriangleFromVertex = 2,
	ClosestPointOnTriangleKdTree = 3,
};

class ofxHairSim
{
public:
	ofxHairSim();
	~ofxHairSim();
	void init(ofxHairModel &model);
	void updateControlPoint(ofPoint *points);
	void update(ofMatrix4x4 mRotate);
	void reset();

	void setTimeStep(const float timestep) { m_timestep = timestep; }
	void setDamping(const float damping) { m_damping = damping; }
	float getTimeStep() { return m_timestep; }
	float getDamping() { return m_damping; }

	bool loadBoundaryOBJ(string filename);
	void modelHairFitting(ofxHairModel& hair, float radius);
	void updateBoundary(vector<ofPoint>  &points);
	void setCollisionType(CollisionType type) { m_collisionType = type; }
	void setSupportRadius(float radius) { m_collision.setRadius(radius); }
	void setFloorHeight(float Ypos) { bFloorContact = true; m_floor_Y = Ypos; }
	std::vector<ofPoint*> getParticlePosition() { return m_particlesPos; }

	CollisionType getCollisionType() { return m_collisionType; }

	ofxHairModel *hair;

private:
	float m_timestep;
	float m_damping;
	float m_floor_Y;
	bool bFloorContact;
	ofVec3f m_gravity;
	ofxHairModel initModel;
	ofxHairBoundary boundary;
	ofxHairCollision m_collision;
	CollisionType m_collisionType;

	std::vector<ofxHairParticle*> m_particles;
	std::vector<ofVec3f*> m_particlesPos;
	std::vector<ofVec3f> m_particlesPosition;

	void updateForce();
	void updateVelocity();
	void updateCollision(ofxHairBoundary &colModel);
	void solveCollision();
	void solveConstraint(ofMatrix4x4 mRotate);
		void applyLocalShapeConstrain(ofMatrix4x4 headRot);
		void applyLocalShapeConstrain2(ofMatrix4x4 headRot);
	void updatePositionAndVelocity();
};
#endif