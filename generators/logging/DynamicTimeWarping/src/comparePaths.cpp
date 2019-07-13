#include <cstdlib>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/tools/lightning/Lightning.h>
#include <ompl/tools/lightning/DynamicTimeWarp.h>
#include <string>
#include <iostream>
#include <fstream>

namespace ob = ompl::base;
namespace ot = ompl::tools;
namespace og = ompl::geometric;

class NullBuffer : public std::streambuf
{
public:
  int overflow(int c) { return c; }
};

typedef struct PathPose {
    double x;
    double y;
    double theta;

    std::string asString() const
    {
        std::stringstream ss;
        ss << "(" << x << ", " << y << ", " << theta << ")";
        return ss.str();
    }
} PathPose;

extern "C" bool
comparePaths(const PathPose *path1, int pathLength1, const PathPose *path2, int pathLength2,
             bool isHolonomicRobot, double turningRadius, double similarityThreshold)
{
    // std::cout << pathLength1 << " " << pathLength2 << std::endl;
    // std::cout << isHolonomicRobot << " " << turningRadius << std::endl;
    // std::cout << path1[0].asString() << "\t" << path2[0].asString() << std::endl;

    ob::StateSpacePtr space =
        isHolonomicRobot
            ? std::make_shared<ob::SE2StateSpace>()
            : std::make_shared<ob::ReedsSheppStateSpace>(turningRadius);

    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);

    og::PathGeometric pg1 = og::PathGeometric(si);
    for (int i = 0; i < pathLength1; i++)
    {
        ob::State* state = space->allocState();
        std::vector<double> reals(3);
        reals[0] = path1[i].x;
        reals[1] = path1[i].y;
        reals[2] = path1[i].theta;
        space->deserialize(state, reals.data());
        pg1.append(state);
        si->freeState(state);
    }

    og::PathGeometric pg2 = og::PathGeometric(si);
    for (int i = 0; i < pathLength2; i++)
    {
        ob::State* state = space->allocState();
        std::vector<double> reals(3);
        reals[0] = path2[i].x;
        reals[1] = path2[i].y;
        reals[2] = path2[i].theta;
        space->deserialize(state, reals.data());
        pg2.append(state);
        si->freeState(state);
    }

    NullBuffer null_buffer;
    std::ostream null_stream(&null_buffer);
    // TODO: This call is essential. Why? We get incorrect results without this call!
    si->printSettings(null_stream); // Use NULL buffer to avoid priniting to the console

    ot::DynamicTimeWarpPtr dwt = std::make_shared<ot::DynamicTimeWarp>(si);
    double score1 = dwt->calcDTWDistance(pg1, pg2); // TODO: Again, why is this needed? Doesnt work without this!
    double score2 = dwt->getPathsScore(pg1, pg2);

    // std::cout << "Path Score: " << score1 << " " << score2 << std::endl;

    dwt = NULL;
    si = NULL;
    space = NULL;

    return score2 < similarityThreshold;
}
