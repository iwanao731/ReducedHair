#pragma once

struct ofxHEMeshNode {
	enum Indices{
		Invalid = -1
	};

	explicit ofxHEMeshNode(int idx=Invalid)
	:	idx(idx)
	{}
	
	bool isValid() const {
		return idx != Invalid;
	}
	
	bool operator==(const ofxHEMeshNode &rhs) const {
		return idx == rhs.idx;
	}
	bool operator!=(const ofxHEMeshNode &rhs) const {
		return idx != rhs.idx;
	}
	bool operator<(const ofxHEMeshNode &rhs) const {
		return idx < rhs.idx;
	}
	bool operator>(const ofxHEMeshNode &rhs) const {
		return idx > rhs.idx;
	}

	int idx;
};

struct ofxHEMeshVertex : public ofxHEMeshNode {
	explicit ofxHEMeshVertex(int idx=Invalid) : ofxHEMeshNode(idx)
	{}
};
struct ofxHEMeshHalfedge : public ofxHEMeshNode {
	explicit ofxHEMeshHalfedge(int idx=Invalid) : ofxHEMeshNode(idx)
	{}
};
struct ofxHEMeshFace : public ofxHEMeshNode {
	enum Winding{
		CW=0,
		CCW
	};

	explicit ofxHEMeshFace(int idx=Invalid) : ofxHEMeshNode(idx)
	{}
};

/*
std::ostream& operator<<(std::ostream& os, const Vertex& v) {
    os << std::string("Vertex:");
	os << v.idx;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Halfedge& h) {
    os << std::string("Halfedge:") << h.idx;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Face& f) {
    os << std::string("Face:") << f.idx;
    return os;
}
*/