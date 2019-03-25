#include "MultipleCircleStateValidityChecker.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
//#include <ompl/geometric/planners/sst/SST.h>
#include <boost/algorithm/string.hpp>
#include <boost/math/constants/constants.hpp>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>

using namespace mrpt::maps;
using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;
namespace ot = ompl::tools;

typedef struct PathPose {
    double x;
    double y;
    double theta;
} PathPose;

enum PLANNER_TYPE {
    SIMPLE_SETUP = 0,
    EXPERIENCE_LIGHTNING,
    EXPERIENCE_THUNDER
};

og::SimpleSetup *getPlanningSetup(PLANNER_TYPE type, ob::StateSpacePtr space,
                                  std::string mapFilename)
{
    og::SimpleSetup *ssPtr = NULL;

    std::string dbPath = std::string("generated/experienceDBs/") + mapFilename;

    switch (type) {
    case PLANNER_TYPE::SIMPLE_SETUP: {
        ssPtr = new og::SimpleSetup(space);
    } break;
    case PLANNER_TYPE::EXPERIENCE_LIGHTNING: {
        ot::Lightning *ptr = new ot::Lightning(space);
        ptr->setFilePath(dbPath + std::string("_lightning.db"));
        ptr->clear();
        ssPtr = ptr;
    } break;
    case PLANNER_TYPE::EXPERIENCE_THUNDER: {
        ot::Thunder *ptr = new ot::Thunder(space);
        ptr->setFilePath(dbPath + std::string("_thunder.db"));
        ptr->clear();
        ssPtr = ptr;
    } break;

    default: {
        std::cout << "INVALID PLANNER TYPE SPECIFIED. Fallback to Simple setup."
                  << std::endl;
        ssPtr = new og::SimpleSetup(space);
    } break;
    }

    return ssPtr;
}

extern "C" bool
plan_multiple_circles(const char *mapFilename, double mapResolution,
                      double robotRadius, double *xCoords, double *yCoords,
                      int numCoords, double startX, double startY,
                      double startTheta, double goalX, double goalY,
                      double goalTheta, PathPose **path, int *pathLength,
                      double distanceBetweenPathPoints, double turningRadius,
                      PLANNER_TYPE plannerType, const char *experienceDBName)
{

    double pLen = 0.0;
    int numInterpolationPoints = 0;
    ob::StateSpacePtr space(new ob::ReedsSheppStateSpace(turningRadius));

    COccupancyGridMap2D gridmap;
    gridmap.loadFromBitmapFile(mapFilename, (float)mapResolution, 0.0f, 0.0f);
    std::cout << "Loaded map (1) " << mapFilename << std::endl;

    ob::ScopedState<> start(space), goal(space);
    ob::RealVectorBounds bounds(2);
    bounds.low[0] = gridmap.getXMin();
    bounds.low[1] = gridmap.getYMin();
    bounds.high[0] = gridmap.getXMax();
    bounds.high[1] = gridmap.getYMax();

    space->as< ob::SE2StateSpace >()->setBounds(bounds);
    std::cout << "Bounds are [(" << bounds.low[0] << "," << bounds.low[1]
              << "),(" << bounds.high[0] << "," << bounds.high[1] << ")]"
              << std::endl;

    og::SimpleSetup *ssPtr =
        getPlanningSetup(plannerType, space, experienceDBName);

    // set state validity checking for this space
    ob::SpaceInformationPtr si(ssPtr->getSpaceInformation());
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(
        new MultipleCircleStateValidityChecker(si, mapFilename, mapResolution,
                                               robotRadius, xCoords, yCoords,
                                               numCoords)));

     ob::PlannerPtr planner(new og::RRTConnect(si));
    //ob::PlannerPtr planner(new og::RRTstar(si));
    // ob::PlannerPtr planner(new og::TRRT(si));
    // ob::PlannerPtr planner(new og::SST(si));
    // ob::PlannerPtr planner(new og::LBTRRT(si));
    // ob::PlannerPtr planner(new og::PRMstar(si));
    // ob::PlannerPtr planner(new og::SPARS(si));
    // ob::PlannerPtr planner(new og::pRRT(si));
    // ob::PlannerPtr planner(new og::LazyRRT(si));
    ssPtr->setPlanner(planner);

    ompl::tools::ExperienceSetup *ePtr =
        dynamic_cast< ot::ExperienceSetup * >(ssPtr);
    if (ePtr != NULL) {
        ompl::base::PlannerPtr repairPlanner(new og::RRTConnect(si));
        ePtr->setRepairPlanner(repairPlanner);
    }

    // set the start and goal states
    start[0] = startX;
    start[1] = startY;
    start[2] = startTheta;
    goal[0] = goalX;
    goal[1] = goalY;
    goal[2] = goalTheta;
    ssPtr->setStartAndGoalStates(start, goal);

    // this call is optional, but we put it in to get more output information
    ssPtr->getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    ssPtr->setup();
    ssPtr->print();

    // attempt to solve the problem within 30 seconds of planning time
    ob::PlannerStatus solved = ssPtr->solve(30.0);

    if (solved) {
        std::cout << "Found solution:" << std::endl;
        ssPtr->simplifySolution();
        og::PathGeometric pth = ssPtr->getSolutionPath();
        pLen = pth.length();
        numInterpolationPoints = ((double)pLen) / distanceBetweenPathPoints;
        if (numInterpolationPoints > 0)
            pth.interpolate(numInterpolationPoints);

        std::vector< ob::State * > states = pth.getStates();
        std::vector< double > reals;

        *pathLength = states.size();
        *path = (PathPose *)malloc(sizeof(PathPose) * states.size());
        memset(*path, 0, sizeof(PathPose) * states.size());

        for (unsigned i = 0; i < states.size(); i++) {
            space->copyToReals(reals, states[i]);
            (*path)[i].x = reals[0];
            (*path)[i].y = reals[1];
            (*path)[i].theta = reals[2];
        }

        if (ePtr != NULL) {
            ePtr->doPostProcessing();
            ePtr->save();
        }
    }
    else {
        std::cout << "No solution found" << std::endl;
    }

    if (ssPtr != NULL) {
        delete ssPtr;
        ssPtr = NULL;
    }

    return solved ? 1 : 0;
}

extern "C" bool
plan_multiple_circles_nomap(double *xCoords, double *yCoords, int numCoords,
                            double startX, double startY, double startTheta,
                            double goalX, double goalY, double goalTheta,
                            PathPose **path, int *pathLength,
                            double distanceBetweenPathPoints,
                            double turningRadius, PLANNER_TYPE plannerType)
{

    double pLen = 0.0;
    int numInterpolationPoints = 0;
    ob::StateSpacePtr space(new ob::ReedsSheppStateSpace(turningRadius));

    std::cout << "Using empty map" << std::endl;

    ob::ScopedState<> start(space), goal(space);
    ob::RealVectorBounds bounds(2);
    bounds.low[0] = -10000;
    bounds.low[1] = -10000;
    bounds.high[0] = 10000;
    bounds.high[1] = 10000;

    space->as< ob::SE2StateSpace >()->setBounds(bounds);
    std::cout << "Bounds are [(" << bounds.low[0] << "," << bounds.low[1]
              << "),(" << bounds.high[0] << "," << bounds.high[1] << ")]"
              << std::endl;

    og::SimpleSetup *ssPtr = getPlanningSetup(plannerType, space, "NoMap");

    // set state validity checking for this space
    ob::SpaceInformationPtr si(ssPtr->getSpaceInformation());
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(
        new MultipleCircleStateValidityChecker(si)));

     ob::PlannerPtr planner(new og::RRTConnect(si));
    //ob::PlannerPtr planner(new og::RRTstar(si));
    // ob::PlannerPtr planner(new og::TRRT(si));
    // ob::PlannerPtr planner(new og::SST(si));
    // ob::PlannerPtr planner(new og::LBTRRT(si));
    // ob::PlannerPtr planner(new og::PRMstar(si));
    // ob::PlannerPtr planner(new og::SPARS(si));
    // ob::PlannerPtr planner(new og::pRRT(si));
    // ob::PlannerPtr planner(new og::LazyRRT(si));
    ssPtr->setPlanner(planner);

    ompl::tools::ExperienceSetup *ePtr =
        dynamic_cast< ot::ExperienceSetup * >(ssPtr);
    if (ePtr != NULL) {
        ompl::base::PlannerPtr repairPlanner(new og::RRTConnect(si));
        ePtr->setRepairPlanner(repairPlanner);
    }

    // set the start and goal states
    start[0] = startX;
    start[1] = startY;
    start[2] = startTheta;
    goal[0] = goalX;
    goal[1] = goalY;
    goal[2] = goalTheta;
    ssPtr->setStartAndGoalStates(start, goal);

    // this call is optional, but we put it in to get more output information
    ssPtr->getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    ssPtr->setup();
    ssPtr->print();

    // attempt to solve the problem within 30 seconds of planning time
    ob::PlannerStatus solved = ssPtr->solve(30.0);

    if (solved) {
        std::cout << "Found solution:" << std::endl;
        ssPtr->simplifySolution();
        og::PathGeometric pth = ssPtr->getSolutionPath();
        pLen = pth.length();
        numInterpolationPoints = ((double)pLen) / distanceBetweenPathPoints;
        if (numInterpolationPoints > 0)
            pth.interpolate(numInterpolationPoints);

        std::vector< ob::State * > states = pth.getStates();
        std::vector< double > reals;

        *pathLength = states.size();
        *path = (PathPose *)malloc(sizeof(PathPose) * states.size());
        memset(*path, 0, sizeof(PathPose) * states.size());

        for (unsigned i = 0; i < states.size(); i++) {
            space->copyToReals(reals, states[i]);
            (*path)[i].x = reals[0];
            (*path)[i].y = reals[1];
            (*path)[i].theta = reals[2];
        }

        if (ePtr != NULL) {
            ePtr->doPostProcessing();
            ePtr->save();
        }
    }
    else {
        std::cout << "No solution found" << std::endl;
    }

    if (ssPtr != NULL) {
        delete ssPtr;
        ssPtr = NULL;
    }

    return solved ? 1 : 0;
}
