#include "ofxHairBoundary.h"
#include "ofxMeshUtil.h"

bool ofxHairBoundary::load(string filename)
{
	ofxHEMesh hem;
	if(!hem.loadOBJModel(filename))
	{
		std::cout << "couldn't load boundary model" << std::endl;
		return false;
	}else{

		// resize
		m_neighborTriangle = new Triangle *[hem.getNumVertices()];
		m_numNeighborTriangle = new unsigned int [hem.getNumVertices()];
		m_boundaryPsi.resize(hem.getNumVertices());

		// calculate triangle relationship
		ofxHEMeshFaceIterator fit = hem.facesBegin();
		ofxHEMeshFaceIterator fite = hem.facesEnd();

		for(; fit != fite; fit++){
			ofxHEMeshPolygonSplitter pit = hem.splitPolygon(fit.f);
			ofxHEMeshTriangle& tri = *pit;

			Triangle triangle;
			triangle.p1 = tri.v1.idx;
			triangle.p2 = tri.v2.idx;
			triangle.p3 = tri.v3.idx;
			m_triangles.push_back(triangle);
		}
		
		ofxHEMeshVertexIterator vit = hem.verticesBegin();
		ofxHEMeshVertexIterator vite = hem.verticesEnd();
	
		for(int i=0; vit!=vite; vit++, i++){

			// vertex and normal
			m_points.push_back(hem.vertexPoint(vit.v));
			m_normal.push_back(hem.angleWeightedVertexNormal(vit.v));
			
			std::vector<ofxHEMeshFace> faces; 
			hem.vertexOneHoodFaces(vit.v, faces);

			// we assume that triangle has six particles in one ring
			m_neighborTriangle[i] = new Triangle [faces.size()];
			m_numNeighborTriangle[i] = faces.size();

			int count = 0;
			for(auto f : faces){
				ofxHEMeshTriangle t =  hem.face2Triangle(f);
				m_neighborTriangle[i][count].p1 = t.v1.idx;;
				m_neighborTriangle[i][count].p2 = t.v2.idx;
				m_neighborTriangle[i][count].p3 = t.v3.idx;
				count++;
			}
		}
	}
	return true;
}

void ofxHairBoundary::setPosition(vector<ofPoint>  &points)
{
	// update vertices
	m_points = points;

	// update normal
	calcNormal();
}

void ofxHairBoundary::calcNormal()
{
	for(int i=0; i<m_points.size(); i++){
		ofVec3f normal(0.0);
		for(int j=0; j<m_numNeighborTriangle[i]; j++){
			Triangle t = m_neighborTriangle[i][j];
			normal += ofxMeshUtil::triangleNormal(m_points[t.p1], m_points[t.p2], m_points[t.p3]);
		}
		m_normal[i] = normal.normalize();
	}
}