#include "Conversions.h"

#include "Utilities/PMPLExceptions.h"

// @NOTE: This should only be called once, when
// the drawable is being added to the Simulation.
// If this needs to be more efficient, this could
// be an unordered map.
glutils::color
StringToColor(const std::string& _color) {
  if(_color == "black")
      return glutils::color::black;
  else if(_color == "dark_grey")
      return glutils::color::dark_grey;
  else if(_color == "medium_grey")
      return glutils::color::medium_grey;
  else if(_color == "grey")
      return glutils::color::grey;
  else if(_color == "light_grey")
      return glutils::color::light_grey;
  else if(_color == "white")
      return glutils::color::white;
  else if(_color == "brown")
      return glutils::color::brown;
  else if(_color == "maroon")
      return glutils::color::maroon;
  else if(_color == "dark_red")
      return glutils::color::dark_red;
  else if(_color == "red")
      return glutils::color::red;
  else if(_color == "orange")
      return glutils::color::orange;
  else if(_color == "yellow")
      return glutils::color::yellow;
  else if(_color == "light_yellow")
      return glutils::color::light_yellow;
  else if(_color == "dark_green")
      return glutils::color::dark_green;
  else if(_color == "green")
      return glutils::color::green;
  else if(_color == "goblin_green")
      return glutils::color::goblin_green;
  else if(_color == "midnight_blue")
      return glutils::color::midnight_blue;
  else if(_color == "blue_grey")
      return glutils::color::blue_grey;
    else if(_color == "blue")
      return glutils::color::blue;
  else if(_color == "light_blue")
      return glutils::color::light_blue;
  else if(_color == "cyan")
      return glutils::color::cyan;
  else if(_color == "magenta")
      return glutils::color::magenta;
  else if(_color == "violet")
      return glutils::color::violet;
  else
    throw RunTimeException(WHERE)
      << "Unrecognized color '"
      << _color <<
      "', see Simulation.cpp for list of color strings";
}
