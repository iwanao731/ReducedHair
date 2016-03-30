#pragma once
#include <string>
#include <vector>
#include <map>

using std::string;
using std::vector;
using std::map;


/*
In order to define generic properties agnostic of type, there has to be a virtual base class
that is inherited by a templated sub-class such that all instantiations of the template can be 
stored in a single collection as pointers.
*/
// interface that doesn't require knowing the array element type
class ofxHEMeshPropertyBase {
public:
	ofxHEMeshPropertyBase() {}
	virtual ~ofxHEMeshPropertyBase() {}

	virtual int capacity() = 0;
	virtual void clear() = 0;
	virtual void extend() = 0;
	virtual void resize(int n) = 0;
	virtual void reserve(int n) = 0;
	virtual void swapItems(int idx1, int idx2) = 0;
	virtual int size() const = 0;
	virtual ofxHEMeshPropertyBase * duplicate() = 0;

protected:
};


// wrapper around std::vector
template <typename T>
class ofxHEMeshProperty : public ofxHEMeshPropertyBase {
public:
	ofxHEMeshProperty(const string &name, T def=T())
	:	name(name), def(def)
	{}
	
	ofxHEMeshProperty(const ofxHEMeshProperty& src) {
		name = src.name;
		values = src.values;
		def = src.def;
	}
	
	int capacity() { return int(values.capacity()); }
	void clear() { values.clear(); }
	void extend() { values.push_back(def); }
	T& get(int idx) { return values[idx]; }
	const T& get(int idx) const { return values[idx]; }
	T& last() { return get(size()-1); }
	const T& last() const { return get(size()-1); }
	void resize(int n) { values.resize(n, def); }
	void reserve(int n) { values.reserve(n); }
	void set(int idx, const T &v) {
		values[idx] = v;
	}
	int size() const { return int(values.size()); }
	void swapItems(int idx1, int idx2) {
		T tmp = values[idx1];
		values[idx1] = values[idx2];
		values[idx2] = tmp;
	}
	
	T* ptr() { return &values[0]; }
	const T* ptr() const { return &values[0]; }
	
	vector<T>& getValues() { return values; }
	const vector<T>& getValues() const { return values; }
	
	ofxHEMeshPropertyBase * duplicate() {
		ofxHEMeshProperty *property = new ofxHEMeshProperty(name, def);
		property->values = values;
		return property;
	}

protected:
	string name;
	vector<T> values;
	T def;
};

class ofxHEMeshPropertySet {
public:
	typedef map<string, void*> PropertyMap;
	typedef map<string, void*>::iterator PropertyMapIterator;
	typedef map<string, void*>::const_iterator PropertyMapConstIterator;

	ofxHEMeshPropertySet() {}
	~ofxHEMeshPropertySet() {
		clear();
	}
	
	void clear() {
		PropertyMapIterator it = properties.begin();
		PropertyMapIterator ite = properties.end();
		for(; it != ite; ++it) {
			((ofxHEMeshPropertyBase *)it->second)->clear();
		}
	}
	
	template <typename T>
	ofxHEMeshProperty<T> * add(const string &name, T def) {
		ofxHEMeshProperty<T>* prop = new ofxHEMeshProperty<T>(name);
		properties.insert(std::pair<string, void*>(name, prop));
		prop->resize(size());
		return prop;
	}
	
	ofxHEMeshPropertyBase * get(const string& name) {
		ofxHEMeshPropertyBase *prop = NULL;
		PropertyMap::iterator it = properties.find(name);
		if(it != properties.end()) {
			prop = (ofxHEMeshPropertyBase *)it->second;
		}
		return prop;
	}
	
	void remove(const string &name) {
		PropertyMapIterator it = properties.find(name);
		if(it != properties.end()) {
			ofxHEMeshPropertyBase *prop = (ofxHEMeshPropertyBase *)it->second;
			properties.erase(it);
			delete prop;
		}
	}
	
	void extend() {
		PropertyMapIterator it = properties.begin();
		PropertyMapIterator ite = properties.end();
		for(; it != ite; ++it) {
			((ofxHEMeshPropertyBase *)it->second)->extend();
		}
	}
	
	void resize(int n) {
		PropertyMapIterator it = properties.begin();
		PropertyMapIterator ite = properties.end();
		for(; it != ite; ++it) {
			((ofxHEMeshPropertyBase *)it->second)->resize(n);
		}
	}
	
	void reserve(int n) {
		PropertyMapIterator it = properties.begin();
		PropertyMapIterator ite = properties.end();
		for(; it != ite; ++it) {
			((ofxHEMeshPropertyBase *)it->second)->reserve(n);
		}
	}
	
	// assumes arrays are always synchonized after an add operation
	int size() const {
		PropertyMapConstIterator it = properties.begin();
		if(it == properties.end()) return 0;
		else return ((ofxHEMeshPropertyBase *)it->second)->size();
	}
	
	void swapItems(int idx1, int idx2) {
		PropertyMapIterator it = properties.begin();
		PropertyMapIterator ite = properties.end();
		for(; it != ite; ++it) {
			((ofxHEMeshPropertyBase *)it->second)->swapItems(idx1, idx2);
		}
	}
	
	void duplicate(ofxHEMeshPropertySet &dst) const {
		dst.properties = properties;
		
		PropertyMapConstIterator it = properties.begin();
		PropertyMapConstIterator ite = properties.end();
		for(; it != ite; ++it) {
			ofxHEMeshPropertyBase *prop = ((ofxHEMeshPropertyBase *)it->second)->duplicate();
			dst.properties[it->first] = prop;
			
		}
	}

protected:
	PropertyMap properties;
};