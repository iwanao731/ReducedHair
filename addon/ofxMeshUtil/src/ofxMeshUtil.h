#pragma once

#include "ofMain.h"

class ofxMeshUtil
{
public:
	// mathematics
	static bool b_positive_value(const float value) { return (value>=0 ? true : false); }
	static float cos_alpha(const ofVec3f v1, const ofVec3f v2) { return v1.dot(v2) / (v1.length() * v2.length()); }

	// geometric
	static bool vertex_on_triangle(const ofVec3f p, const ofVec3f p1, const ofVec3f p2, const ofVec3f p3); // if the vertex is inside on the triangle
	static float distance_point_to_plane(const ofVec3f p, const ofVec3f p1, const ofVec3f p2, const ofVec3f p3, ofVec3f &pc);	
	static float distance_point_to_edge(const ofVec3f p, const ofVec3f p1, const ofVec3f p2, ofVec3f &pc = ofVec3f::zero());

	static unsigned int closest_point_index(const ofVec3f input_point, const unsigned int pointNum, const ofVec3f *points);
	static float closest_point_distance(const ofVec3f input_point, const unsigned int pointNum, const ofVec3f *points, unsigned int &index);
	static float point_triangle_distance(const ofVec3f p0, const ofVec3f p1, const ofVec3f p2, const ofVec3f p3, ofVec3f &pc = ofVec3f::zero());
	static float point_triangle_distance2(const ofVec3f p0, const ofVec3f p1, const ofVec3f p2, const ofVec3f p3, ofVec3f &pc, float& w12, float& w23, float& w31);
	static ofVec3f center_of_triangle(const ofVec3f p1, const ofVec3f p2, const ofVec3f p3);
	static ofVec3f midpoint_of_edge(const ofVec3f p1, const ofVec3f p2) { return ofVec3f(p1+p2)/2; }
	static ofVec3f triangleNormal(const ofVec3f p1, const ofVec3f p2, const ofVec3f p3);
	static void normal_of_triangle_edge(const ofVec3f p1, const ofVec3f p2, const ofVec3f p3, ofVec3f &n12, ofVec3f &n23, ofVec3f &n31);
	static float angle_two_edge(const ofPoint p1, const ofPoint p2, const ofPoint p3);
	static ofMatrix4x4 rotation_matrix_between_two_vectors(const ofVec3f vFrom, const ofVec3f vTo);
	static ofMatrix4x4 getRotationOrthoAxis(ofVec3f v0, ofVec3f v1);
	static ofMatrix4x4 getRotationOrthoAxis2(ofVec3f v0, ofVec3f v1);
	static ofMatrix4x4 getRotationOrthoAxis3(ofVec3f v0, ofVec3f v1);
	static ofColor getErrorColor(float value, float errMin, float errMax);
	// will remove
	static float point_edge_distance(const ofVec3f p0, const ofVec3f p1, const ofVec3f p2, ofVec3f &pc = ofVec3f::zero());

};
