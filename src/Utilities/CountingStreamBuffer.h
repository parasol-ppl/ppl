#ifndef COUNTING_STREAM_BUFFER_H_
#define COUNTING_STREAM_BUFFER_H_

#include <cstddef>
#include <iostream>
#include <fstream>
#include <string>
#include <streambuf>


////////////////////////////////////////////////////////////////////////////////
/// Specialization of std::streambuf that gives us detailed information about
/// where parsing errors occur within input files.
///
/// @TODO This class appears to be implemented incorrectly - it is used as a
///       stand-alone object rather than a streambuf for an input stream. We
///       should rectify this by removing m_fileStream and m_streamBuffer so that
///       it can be used properly.
////////////////////////////////////////////////////////////////////////////////
class CountingStreamBuffer final : public std::streambuf {

  public:

    ///@name Construction
    ///@{

    /// @param _filename Filename
    CountingStreamBuffer(const std::string& _filename);

    // Disallow copy and assignment
    CountingStreamBuffer(const CountingStreamBuffer&) = delete;
    CountingStreamBuffer& operator=(const CountingStreamBuffer&) = delete;

    ///@}
    ///@name I/O Interface
    ///@{

    /// @return Current line number
    size_t LineNumber() const;

    /// @return Line number of previously read character
    size_t PrevLineNumber() const;

    /// @return Current column
    size_t Column() const;

    /// @return Current file position
    std::streamsize FilePos() const;

    /// @return String describing current file position
    std::string Where() const;

    ///@}

  private:

    ///@}
    ///@name streambuf Overrides
    ///@{

    /// Extract next character from stream without advancing read position.
    /// @return Next character or EOF
    virtual std::streambuf::int_type underflow() override;

    /// Extract next character from stream
    /// @return Next character of EOF
    virtual std::streambuf::int_type uflow() override;

    /// Put back last character
    /// @param _c Character
    /// @return Value of character put back or EOF
    virtual std::streambuf::int_type pbackfail(std::streambuf::int_type _c)
        override;

    /// Change position by offset according to dir and mode
    /// @param _pos Position
    /// @param _dir Direction
    /// @param _mode Mode
    /// @return Position
    virtual std::ios::pos_type seekoff(std::ios::off_type _pos,
        std::ios_base::seekdir _dir, std::ios_base::openmode _mode) override;

    /// Change to specified position according to mode
    /// @param _pos Position
    /// @param _mode Mode
    /// @return Position
    virtual std::ios::pos_type seekpos(std::ios::pos_type _pos,
        std::ios_base::openmode _mode) override;

    ///@}
    ///@name Internal State
    ///@{

    std::string m_filename;                  ///< Filename
    std::ifstream m_fileStream;              ///< Hosted file stream
    std::streambuf* m_streamBuffer{nullptr}; ///< Hosted streambuffer
    std::streamsize m_filePos;               ///< File position

    size_t m_line;                      ///< Current line number
    size_t m_prevLine;                  ///< Line number of last read character.
    size_t m_column;                    ///< Current column
    size_t m_prevColumn;                ///< Previous column

    ///@}

};

#endif
