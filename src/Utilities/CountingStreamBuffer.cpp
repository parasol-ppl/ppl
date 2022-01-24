#include "CountingStreamBuffer.h"

#include <sstream>

using namespace std;


/*------------------------------ Construction --------------------------------*/

CountingStreamBuffer::
CountingStreamBuffer(const string& _filename) : m_filename(_filename),
    m_fileStream(_filename), m_streamBuffer(m_fileStream.rdbuf()),
    m_filePos(0), m_line(1), m_prevLine(1), m_column(0),
    m_prevColumn(static_cast<size_t>(-1)) { }

/*------------------------------ I/O Interface -------------------------------*/

size_t
CountingStreamBuffer::
LineNumber() const {
  return m_line;
}


size_t
CountingStreamBuffer::
PrevLineNumber() const {
  return m_prevLine;
}


size_t
CountingStreamBuffer::
Column() const {
  return m_column;
}


streamsize
CountingStreamBuffer::
FilePos() const {
  return m_filePos;
}


string
CountingStreamBuffer::
Where() const {
  ostringstream oss;
  oss << "File: " << m_filename
      << "\n\tLine: " << m_line
      << "\n\tColumn: " << m_column;
  return oss.str();
}

/*--------------------------- streambuf Overrides ----------------------------*/

streambuf::int_type
CountingStreamBuffer::
underflow() {
  return m_streamBuffer->sgetc();
}


streambuf::int_type
CountingStreamBuffer::
uflow() {
  int_type rc = m_streamBuffer->sbumpc();

  m_prevLine = m_line;
  if(traits_type::eq_int_type(rc, traits_type::to_int_type('\n'))) {
    ++m_line;
    m_prevColumn = m_column + 1;
    m_column = static_cast<size_t>(-1);
  }

  ++m_column;
  ++m_filePos;
  return rc;
}


streambuf::int_type
CountingStreamBuffer::
pbackfail(streambuf::int_type _c) {
  if(traits_type::eq_int_type(_c, traits_type::to_int_type('\n'))) {
    --m_line;
    m_prevLine = m_line;
    m_column = m_prevColumn;
    m_prevColumn = 0;
  }

  --m_column;
  --m_filePos;

  if(_c != traits_type::eof())
    return m_streamBuffer->sputbackc(traits_type::to_char_type(_c));
  else
    return m_streamBuffer->sungetc();
}


ios::pos_type
CountingStreamBuffer::
seekoff(ios::off_type _pos, ios_base::seekdir _dir, ios_base::openmode _mode) {
  if(_dir == ios_base::beg && _pos == static_cast<ios::off_type>(0)) {
    m_prevLine = 1;
    m_line = 1;
    m_column = 0;
    m_prevColumn = static_cast<size_t>(-1);
    m_filePos = 0;

    return m_streamBuffer->pubseekoff(_pos, _dir, _mode);
  }
  else
    return streambuf::seekoff(_pos, _dir, _mode);
}


ios::pos_type
CountingStreamBuffer::
seekpos(ios::pos_type _pos, ios_base::openmode _mode) {
  if(_pos == static_cast<ios::pos_type>(0)) {
    m_prevLine = 1;
    m_line = 1;
    m_column = 0;
    m_prevColumn = static_cast<size_t>(-1);
    m_filePos = 0;

    return m_streamBuffer->pubseekpos(_pos, _mode);
  }
  else
    return streambuf::seekpos(_pos, _mode);
}

/*----------------------------------------------------------------------------*/
