#include "ofxHairDraw.h"

ofxHairDraw::ofxHairDraw(ofxHairModel &model)
:	m_model(model)
{
}

void ofxHairDraw::draw()
{
	if(bEdge){
		glLineWidth(0.1f);
		for(auto s : m_model.strands) {
			if(s.bGuideHair && bGuideHair){
				glBegin(GL_LINE_STRIP);
				for(auto p : s.m_particles) {
					if(bColor){
						ofSetColor(p.color);
					}else{
						ofSetColor(p.collision_color);
					}
					glVertex3f(p.position.x, p.position.y, p.position.z);
				}
				glEnd();
			}
			
			if(!s.bGuideHair && bNormalHair){
				glBegin(GL_LINE_STRIP);
				for(auto p : s.m_particles) {
					if(bColor){
						ofSetColor(p.color);
					}else{
						ofSetColor(p.collision_color);
					}
					glVertex3f(p.position.x, p.position.y, p.position.z);
				}
				glEnd();
			}
		}
	}

	if(bNode){
		ofSetColor(255);
		glPointSize(3.0);
		glBegin(GL_POINTS);
		for(auto s : m_model.strands) {
			for(auto p : s.m_particles) {
				ofPoint pos = p.position;
				ofSetColor(p.color);
				if(bColor){
					ofSetColor(p.collision_color);
				}
				glVertex3f(pos.x, pos.y, pos.z);
			}
		}
		glEnd();
	}
}

ofxHairDraw& ofxHairDraw::setDrawHairParticles(bool v)
{
	bNode = v;
	return *this;
}

ofxHairDraw& ofxHairDraw::setDrawHairEdges(bool v)
{
	bEdge = v;
	return *this;
}

ofxHairDraw& ofxHairDraw::setDrawHairColor(bool v)
{
	bColor = v;
	return *this;
}

ofxHairDraw& ofxHairDraw::setDrawHairNormal(bool v)
{
	bNormalHair = v;
	return *this;
}


ofxHairDraw& ofxHairDraw::setDrawHairGuide(bool v)
{
	bGuideHair = v;
	return *this;
}

