#include "ofxMeshUtil.h"
#include <iostream>
#include <cmath>

bool ofxMeshUtil::vertex_on_triangle(const ofVec3f p, const ofVec3f p1, const ofVec3f p2, const ofVec3f p3)
{
	float t1, t2, t3;
	t1 = (p2-p1).cross(p-p1).z;
	t2 = (p3-p2).cross(p-p2).z;
	t3 = (p1-p3).cross(p-p3).z;

	if( b_positive_value(t1) && b_positive_value(t2) && b_positive_value(t3) )
		return true;
	else{
		//std::cout << "point is not on triangle" << std::endl;
		return false;
	}
}

// UNDER CONSTRUCTION
float ofxMeshUtil::distance_point_to_plane(const ofVec3f p, const ofVec3f p1, const ofVec3f p2, const ofVec3f p3, ofVec3f &pc)
{
	// http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.479.8237&rep=rep1&type=pdf

	ofVec3f normal = triangleNormal(p1, p2, p3);
	pc = p - (p1-p).length() * cos_alpha(p1-p, normal) * normal.normalized();

	// inside on triangle or not
	if(vertex_on_triangle(p, p1, p2, p3))
	{
		ofVec3f v1 = (p1-p2).normalized() + (p1-p3).normalized();
		ofVec3f v2 = (p2-p3).normalized() + (p2-p1).normalized();
		ofVec3f v3 = (p3-p1).normalized() + (p3-p2).normalized();
		return (p - pc).length();
	}else{
		// To do implementation of distance_point_from_edge
		return 	point_triangle_distance(p, p1, p2, p3, pc);

		//ofVec3f R = ( (p2- pc).cross(p1 - pc) ).cross(p2 - p1);
		//float gamma = cos_alpha(p1-pc, R);
		//ofVec3f pp2 = pc * (p1 - pc).length() * gamma * R.normalized();
		//
		////float t = (pp2 - p1) / (p2 - p1);
		//float t = 0;
		//if(t < 0.0f){
		//	return (p1-p).length();
		//}else if(t > 1.0f){
		//	return (p2- p).length();
		//}else{
		//	return sqrt((pp2 - pc).lengthSquared() + (pc - p).lengthSquared());
		//}
	}
}

ofVec3f ofxMeshUtil::triangleNormal(const ofVec3f p1, const ofVec3f p2, const ofVec3f p3)
{
	return (p2-p1).cross(p3-p1);
}

float ofxMeshUtil::closest_point_distance(const ofVec3f input_point, const unsigned int pointNum, const ofVec3f *points, unsigned int &index)
{
	float distance = 1e+8;
	for(int i=0; i<pointNum; i++)
	{
		if(distance > (points[i] - input_point).length())
		{
			distance = (points[i] - input_point).length();
			index = i;
		}
	}
	return distance;
}

unsigned int ofxMeshUtil::closest_point_index(const ofVec3f input_point, const unsigned int pointNum, const ofVec3f *points)
{
	unsigned int index;
	closest_point_distance(input_point, pointNum, points, index);
	return index;
}

// find distance x0 is from segment x1-x2
float ofxMeshUtil::point_edge_distance(const ofVec3f p0, const ofVec3f p1, const ofVec3f p2, ofVec3f &pc)
{
	ofVec3f dx(p2-p1);
	double m2=dx.lengthSquared();

	// find parameter value of closest point on segment
	float s12=(float)(p2-p0).dot(dx)/m2;
	if(s12<0){
		s12=0;
	}else if(s12>1){
		s12=1;
	}

	// and find the distance
	pc = s12*p1+(1-s12)*p2;
	return p0.distance(pc);
}

float ofxMeshUtil::point_triangle_distance(const ofVec3f p0, const ofVec3f p1, const ofVec3f p2, const ofVec3f p3, ofVec3f &pc)
{
   // first find barycentric coordinates of closest point on infinite plane
   ofVec3f p13(p1-p3), p23(p2-p3), p03(p0-p3);
   float m13=p13.lengthSquared(), m23=p23.lengthSquared(), d=p13.dot(p23);
   float invdet=1.f/max(m13*m23-d*d,1e-30f);
   float a=p13.dot(p03), b=p23.dot(p03);

   // the barycentric coordinates themselves
   float w23=invdet*(m23*a-d*b);
   float w31=invdet*(m13*b-d*a);
   float w12=1-w23-w31;

   // if we're inside the triangle
   if(w23>=0 && w31>=0 && w12>=0){ 
	   pc = w23*p1+w31*p2+w12*p3;
	   return p0.distance(pc); 
   }else{ // we have to clamp to one of the edges
		ofVec3f pp[3];
		float dist[3];
		dist[0] = ofxMeshUtil::point_edge_distance(p0, p1, p2, pp[0]);
		dist[1] = ofxMeshUtil::point_edge_distance(p0, p1, p3, pp[1]);
		dist[2] = ofxMeshUtil::point_edge_distance(p0, p2, p3, pp[2]);

		float closest = min(min(dist[0], dist[1]), dist[2]);

		for(int i=0; i<3; i++){
			if(closest == dist[i]){
				pc = pp[i];
			}
		}
		return closest;

		// MORE COOL APPROACH

   //   if(w23>0){ // this rules out edge 2-3 for us
   //      return min(point_edge_distance(p0,p1,p2), point_edge_distance(p0,p1,p3));
	  //}else if(w31>0){ // this rules out edge 1-3
   //      return min(point_edge_distance(p0,p1,p2), point_edge_distance(p0,p2,p3));
	  //}else{ // w12 must be >0, ruling out edge 1-2
   //      return min(point_edge_distance(p0,p1,p3), point_edge_distance(p0,p2,p3));
	  //}
   }

}

float ofxMeshUtil::point_triangle_distance2(ofVec3f p0, const ofVec3f p1, const ofVec3f p2, const ofVec3f p3, ofVec3f &pc, float& w12, float& w23, float& w31)
{
   // first find barycentric coordinates of closest point on infinite plane
   ofVec3f p13(p1-p3), p23(p2-p3), p03(p0-p3);
   float m13=p13.lengthSquared(), m23=p23.lengthSquared(), d=p13.dot(p23);
   float invdet=1.f/max(m13*m23-d*d,1e-30f);
   float a=p13.dot(p03), b=p23.dot(p03);

   // the barycentric coordinates themselves
   w23=invdet*(m23*a-d*b);
   w31=invdet*(m13*b-d*a);
   w12=1-w23-w31;

   // if we're inside the triangle
   if(w23>=0 && w31>=0 && w12>=0){ 
	   pc = w23*p1+w31*p2+w12*p3;
	   return p0.distance(pc); 
   }else{ // we have to clamp to one of the edges
		ofVec3f pp[3];
		float dist[3];
		dist[0] = ofxMeshUtil::point_edge_distance(p0, p1, p2, pp[0]);
		dist[1] = ofxMeshUtil::point_edge_distance(p0, p1, p3, pp[1]);
		dist[2] = ofxMeshUtil::point_edge_distance(p0, p2, p3, pp[2]);

		float closest = min(min(dist[0], dist[1]), dist[2]);

		for(int i=0; i<3; i++){
			if(closest == dist[i]){
				pc = pp[i];
			}
		}
		return closest;
   }
}


ofVec3f ofxMeshUtil::center_of_triangle(const ofVec3f p1, const ofVec3f p2, const ofVec3f p3)
{
	return (p1+p2+p3)/3;
}

void ofxMeshUtil::normal_of_triangle_edge(const ofVec3f p1, const ofVec3f p2, const ofVec3f p3, ofVec3f &n12, ofVec3f &n23, ofVec3f &n31)
{
//	ofVec3f normal = triangleNormal(p1, p2, p3);
	ofVec3f normal = (p2-p1).cross(p3-p2);
	n12 = (p2-p1).cross(normal).normalized();
	n23 = (p3-p2).cross(normal).normalized();
	n31 = (p1-p3).cross(normal).normalized();
}

//p1-p2-p3
float ofxMeshUtil::angle_two_edge(const ofPoint p1, const ofPoint p2, const ofPoint p3)
{
	ofVec3f p1p2 = p1-p2;
	ofVec3f p2p3 = p3-p2;
	return p1p2.angle(p2p3);

	//ofVec3f p1p3 = p3-p1;
	//float cosa = (p1p2.lengthSquared() + p2p3.lengthSquared() - p1p3.lengthSquared()) / (2* p1p2.length() * p2p3.length());
	//return acos(cosa);
}

float ofxMeshUtil::distance_point_to_edge(const ofVec3f p, const ofVec3f p1, const ofVec3f p2, ofVec3f &pc)
{
	ofVec3f dx(p2-p1);
	double m2=dx.lengthSquared();

	// find parameter value of closest point on segment
	if(!m2 == 0.0){
		float s12=(float)(p2-p).dot(dx)/m2;
		if(s12<0){
			s12=0.0;
		}else if(s12>1){
			s12=1.0;
		}

		// and find the distance
		pc = s12*p1+(1-s12)*p2;
		return p.distance(pc);
	}else{
		return (p1-p).length();
	}
}


//--------------------------------------------------------------
ofColor ofxMeshUtil::getErrorColor(float value, float errMin, float errMax)
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

ofMatrix4x4 ofxMeshUtil::rotation_matrix_between_two_vectors(const ofVec3f v1, const ofVec3f v2)
{
    ofVec3f m1[3], m2[3];

	ofVec3f axis = v1.crossed(v2);
    axis.normalize();

    /* construct 2 matrices */
    m1[0] = v1;
    m2[0] = v2;

    m1[1] = axis;
    m2[1] = axis;

    m1[2] = m1[1].cross(m1[0]);
    m2[2] = m2[1].cross(m2[0]);

    /* calculate the difference between m1 and m2 */
	ofMatrix3x3 m11, m22;
	m11.set(m1[0].x, m1[0].y, m1[0].z, m1[1].x, m1[1].y, m1[1].z, m1[2].x, m1[2].y, m1[2].z);
	m22.set(m2[0].x, m2[0].y, m2[0].z, m2[1].x, m2[1].y, m2[1].z, m2[2].x, m2[2].y, m2[2].z);
	m11.transpose();

    ofMatrix3x3 m = m22 * m11;

	ofMatrix4x4 mmm;
	mmm.set(m.a, m.b, m.c, 0.0, m.d, m.e, m.f, 0.0, m.g, m.h, m.i, 0.0, 0.0, 0.0, 0.0, 1.0);

    return mmm;
}

// make orthonormal vertor triple for a frame of coordinate
ofMatrix4x4 ofxMeshUtil::getRotationOrthoAxis(ofVec3f vFrom, ofVec3f vTo)
{
    ofVec3f v0 = vTo - vFrom;
    ofVec3f v1 = ofVec3f(0.0f, 1.0f, 0.0f);

	v0.normalized();
	v1.normalized();
	ofVec3f nv1 = v1 - v0 * v1.dot(v0);
	nv1.normalized();
	ofVec3f nv0 = v0;
	ofVec3f nv2 = nv1.crossed(nv0);

	ofMatrix4x4 mat;
	mat.set(nv2.x, nv2.y, nv2.z, 0.0, nv1.x, nv1.y, nv1.z, 0.0, nv0.x, nv0.y, nv0.z, 0.0, 0.0, 0.0, 0.0, 1.0);
	return mat;
}

ofMatrix4x4 ofxMeshUtil::getRotationOrthoAxis2(ofVec3f v0, ofVec3f v1)
{
	//ofVec3f v = v0.crossed(v1);
	ofVec3f v = v0.getPerpendicular(v1);
	v = v.normalized();
	float angle = v0.angleRad(v1);
	float _sin = sin(angle);
	float _cos = cos(angle);
	
	ofMatrix4x4 mat;
	mat(0,0) = v.x * v.x * (1-_cos) + _cos;
	mat(0,1) = v.x * v.y * (1-_cos) - v.z * _sin;
	mat(0,2) = v.z * v.x * (1-_cos) + v.y * _sin;
	mat(0,3) = 0.0;
	mat(1,0) = v.x * v.y * (1-_cos) + v.z * _sin;
	mat(1,1) = v.y * v.y * (1-_cos) + _cos;
	mat(1,2) = v.y * v.z * (1-_cos) - v.x * _sin;
	mat(1,3) = 0.0;
	mat(2,0) = v.z * v.x * (1-_cos) - v.y * _sin;
	mat(2,1) = v.y * v.z * (1-_cos) + v.x * _sin;
	mat(2,2) = v.z * v.z * (1-_cos) + _cos;
	mat(2,3) = 0.0;
	mat(3,0) = 0.0;
	mat(3,1) = 0.0;
	mat(3,2) = 0.0;
	mat(3,3) = 1.0;

	return mat;
}

ofMatrix4x4 ofxMeshUtil::getRotationOrthoAxis3(ofVec3f v0, ofVec3f v1)
{
	ofMatrix4x4 mat;
	mat(0,0) = v0.normalized().x;  
	mat(0,1) = v0.normalized().y;  
	mat(0,2) = v0.normalized().z;  
	mat(0,3) = 0.0;
	mat(1,0) = ((v0.cross(v1)).crossed(v1)).normalized().x;  
	mat(1,1) = ((v0.cross(v1)).crossed(v1)).normalized().y;  
	mat(1,2) = ((v0.cross(v1)).crossed(v1)).normalized().z;  
	mat(1,3) = 0.0;
	mat(2,0) = (v0.cross(v1)).normalized().x;  
	mat(2,1) = (v0.cross(v1)).normalized().y;  
	mat(2,2) = (v0.cross(v1)).normalized().z;  
	mat(2,3) = 0.0;
	mat(3,0) = 0.0;
	mat(3,1) = 0.0;
	mat(3,2) = 0.0;
	mat(3,3) = 1.0;

	return mat;
}