#include "IOUtils.h"

#include <algorithm>
#include <cctype>

using namespace std;


/*------------------------------- Vizmo Debug --------------------------------*/

ofstream* vdo = nullptr;

void
VDInit(string _filename) {
  VDClose();
  if(!_filename.empty())
    vdo = new ofstream(_filename.c_str());
}

void
VDClose() {
  if(vdo) {
    vdo->close();
    delete vdo;
    vdo = nullptr;
  }
}


void
VDComment(string _s) {
  if(vdo)
    (*vdo) << "Comment " << _s << endl;
}


void
VDClearAll() {
  if(vdo)
    (*vdo) << "ClearAll " << endl;
}


void
VDClearLastTemp() {
  if(vdo)
    (*vdo) << "ClearLastTemp " << endl;
}


void
VDClearComments() {
  if(vdo)
    (*vdo) << "ClearComments " << endl;
}

/*----------------------------- Other IO Utils -------------------------------*/

bool
FileExists(const string& _filename) {
  ifstream ifs(_filename.c_str());
  return ifs.good();
}


void
GoToNext(istream& _is) {
  string line;
  while(!_is.eof()) {
    char c;
    while(isspace(_is.peek()))
      _is.get(c);

    c = _is.peek();
    if(!IsCommentLine(c))
      return;
    else
      getline(_is, line);
  }
}


bool
IsCommentLine(const char _c) {
  return _c == '#';
}


string
GetPathName(const string& _filename) {
  size_t pos = _filename.rfind('/');
  return pos == string::npos ? "" : _filename.substr(0, pos+1);
}


string
ReadFieldString(istream& _is, CountingStreamBuffer& _cbs,
    const string& _desc, bool _toUpper) {
  string s = ReadField<string>(_is, _cbs, _desc);
  if(_toUpper)
    transform(s.begin(), s.end(), s.begin(), ::toupper);
  return s;
}


vector<string>
GetTokens(string _s, string _delimiters) {
  vector<string> tokens;
  int cutAt;
  while( (cutAt = _s.find_first_of(_delimiters)) != (int)_s.npos ) {
    if(cutAt > 0)
      tokens.push_back(_s.substr(0, cutAt));
    _s = _s.substr(cutAt + 1);
  }

  if(_s.length() > 0)
    tokens.push_back(_s);

  return tokens;
}

/*----------------------------------------------------------------------------*/
