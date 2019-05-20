#include <cstdlib>
#include <ompl/tools/lightning/Lightning.h>
#include <string>

namespace ob = ompl::base;
namespace ot = ompl::tools;
namespace og = ompl::geometric;

typedef struct PathPose {
    double x;
    double y;
    double theta;

    std::string asString()
    {
        std::stringstream ss;
        ss << "(" << x << ", " << y << ", " << theta << ")";
        return ss.str();
    }
} PathPose;

extern "C" bool
comparePaths(PathPose **path1, int *pathLength1, PathPose **path2, int *pathLength2)
{
    return false;
}
