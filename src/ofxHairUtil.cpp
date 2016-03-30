#include "ofxHairUtil.h"

vector<string> ofxHairUtil::split(const string &str, char delim)
{
  istringstream iss(str);
  string tmp;
  vector<string> res;

  while(getline(iss, tmp, delim))
	  res.push_back(tmp);

  return res;
}