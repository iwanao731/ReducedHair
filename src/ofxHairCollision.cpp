#include "ofxHairCollision.h"

ofxHairCollision::ofxHairCollision()
{
	//m_supportRadius = 7.0f;	// 50.0f
}

void ofxHairCollision::collisionClosestPointOnTriangle(
	vector<ofxHairParticle*> position,
	vector<ofPoint> boundaryX,
	vector<ofVec3f> boundaryNormal,
	vector<Triangle> triangles )
{
	// calculate each particle
	for(auto p : position) {

		float distance = 100000000000;
		ofVec3f collision_point;
		ofVec3f direction;
		Triangle triangle;

		// calculate closest triangle
		for(auto f : triangles) {
			ofVec3f temp_col_point;
			float d = ofxMeshUtil::point_triangle_distance(p->position, boundaryX[f.p1], boundaryX[f.p2], boundaryX[f.p3], temp_col_point);
			ofVec3f dir = p->position - temp_col_point;
			if(distance > d){
				collision_point = temp_col_point;
				distance = d;
				direction = dir;
				triangle = f;
			}
		}

		// move to out side or not
		ofVec3f triNormal = ofxMeshUtil::triangleNormal(boundaryX[triangle.p1], boundaryX[triangle.p2], boundaryX[triangle.p3]);
		if(direction.dot(triNormal)<0){
			p->tmp_position -= (distance) * (p->position - collision_point).normalized();
			p->color = ofColor(255,0,0);
		}else{
			p->color = ofColor(255,255,255);
		}
	}
}

void ofxHairCollision::collisionClosestPointOnTriangleFromNearestVertex(
	vector<ofxHairParticle*> position,
	vector<ofPoint> boundaryX,
	vector<ofVec3f> boundaryNormal,
	unsigned int *numNeighborTriangles,
	Triangle **neighborTriangles )
{
	int index = 0;
	for(auto p : position) {

		// calculate closest surface vertex from each particle
		float dist = 100000000000;
		int index;

		for(int j=0; j<boundaryX.size(); j++)
		{
			float temp = (p->position - boundaryX[j]).length();
			if(dist > temp){
				dist = temp;
				index = j;
			}
		}
		
		ofVec3f dir = p->position - boundaryX[index];

		if(dir.dot(boundaryNormal[index])<0){

			float distance = 100000000;
			ofVec3f collision;

			for(int k=0; k<numNeighborTriangles[index]; k++){
				int t1 = neighborTriangles[index][k].p1;
				int t2 = neighborTriangles[index][k].p2;
				int t3 = neighborTriangles[index][k].p3;

				if(t1<1000){
					ofVec3f colPoint;
					float d = ofxMeshUtil::point_triangle_distance(p->position, boundaryX[t1], boundaryX[t2], boundaryX[t3], colPoint);
					if(distance > d){
						collision = colPoint;
						distance = d;
					}
				}
			}
			p->tmp_position -= distance * (p->position - collision).normalized();
			p->color = ofColor(255,0,0);
		}else{
			p->color = ofColor(255,255,255);
		}
		index++;
	}
}

void ofxHairCollision::collisionClosestPointOnTriangle_ke_tree(
	vector<ofxHairParticle*> position,
	vector<ofPoint> boundaryX,
	vector<ofVec3f> boundaryNormal,
	unsigned int *numNeighborTriangles,
	Triangle **neighborTriangles,
	float radius )
{
	// set vertices
	std::vector<ofVec3f> points;
	for(int i=0; i<position.size(); i++){
		points.push_back(position[i]->tmp_position);
		position[i]->collision_color = ofColor(255,255,255);
	}

	// build Nearest Neighbor structure on boundary
	ofxNearestNeighbour3D nn;
	nn.buildIndex(boundaryX);

	// we input each particles of hair
	for(int i=0; i<points.size(); i++) {

	    vector<pair<NNIndex, float> > indices;
	    nn.findPointsWithinRadius(points[i], radius, indices);	// the result is depended by radius parameter
		
		float distance = 100000000;
		int index;			// closest index
		ofVec3f direction;	// direction from particle to closest surface vertex

		// calculate closest vertex between each particle and boundary
		for (unsigned j = 0; j < indices.size(); ++j) {
			ofVec3f dir = position[i]->tmp_position - boundaryX[indices[j].first];

				// calculate closest vertex
				float dist = dir.length();
				if(dist < distance){
					distance = dist;
					index = indices[j].first;
					direction = dir;
				}
		}

		// inside or not
		if(	direction.dot(boundaryNormal[index]) < 0 ) {

			// closest triangle
			ofVec3f colPoint;	// closest(collision) point on Triangle
			distance = 10000000;
			for(int k=0; k<numNeighborTriangles[index]; k++) {
				Triangle t = neighborTriangles[index][k];	
				ofVec3f colp;
				float dist = ofxMeshUtil::point_triangle_distance(position[i]->tmp_position, boundaryX[t.p1], boundaryX[t.p2], boundaryX[t.p3], colp);
				if(distance > dist){
					distance = dist;
					colPoint = colp;
				}
			}
			position[i]->tmp_position -= (distance) * (position[i]->tmp_position - colPoint).normalized();
			position[i]->collision_color = ofColor(255,0,0);
		}
	}
}