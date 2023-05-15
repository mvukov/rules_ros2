#include "{{header}}"

#include <cstdlib>
#include <string>

namespace {{namespace}} {
namespace {

constexpr const char* AMENT_PREFIX_PATH_ENV = "AMENT_PREFIX_PATH";
constexpr const char* AMENT_PREFIX_PATH = "{{ament_prefix_path}}";

struct AmentPrefixPathSetup
{
  AmentPrefixPathSetup()
  {
    const char* ament_prefix_path = std::getenv(AMENT_PREFIX_PATH_ENV);
    if (ament_prefix_path)
    {
      std::string ament_prefix_path_str = ament_prefix_path;
      ament_prefix_path_str += ":";
      ament_prefix_path_str += AMENT_PREFIX_PATH;
      setenv(AMENT_PREFIX_PATH_ENV, ament_prefix_path_str.c_str(), 1);
    }
    else
    {
      setenv(AMENT_PREFIX_PATH_ENV, AMENT_PREFIX_PATH, 1);
    }
  }
};

}

void setup_ament_prefix_path()
{
  static AmentPrefixPathSetup setup{};
}


} // namespace {{namespace}}
