#include "StateValidityChecker.h"
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
#include <ompl/util/Console.h>

#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>

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

    std::string asString()
    {
        std::stringstream ss;
        ss << "(" << x << ", " << y << ", " << theta << ")";
        return ss.str();
    }
} PathPose;

enum PLANNER_TYPE {
    SIMPLE_SETUP = 0,
    EXPERIENCE_LIGHTNING,
    EXPERIENCE_THUNDER,
    PLANNER_TYPE_COUNT
};

enum MODE { NORMAL = 0, REPLANNING, EXPERIENCE_GENERATION, MODE_COUNT };

bool LOGGING_ACTIVE = true;

extern "C" void cleanupPath(PathPose *path)
{
    std::cout << "Cleaning up memory.." << std::endl;
    free(path);
}

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

std::string getLogFileName(const char *experienceDBName,
                           PLANNER_TYPE plannerType)
{
    std::stringstream filename;
    filename << "generated/experienceLogs/";
    filename << experienceDBName;
    switch (plannerType) {
    case PLANNER_TYPE::EXPERIENCE_LIGHTNING:
        filename << "_lightning";
        break;
    case PLANNER_TYPE::EXPERIENCE_THUNDER:
        filename << "_thunder";
        break;
    default:
        filename << "_simple";
        break;
    }
    filename << ".log";

    return filename.str();
}

void log(const std::string &logFilename, const std::string &log)
{
    if (log.empty() || !LOGGING_ACTIVE)
        return;

    // Open the log file
    std::ofstream logfile;
    logfile.open(logFilename.c_str(), std::ios_base::app);
    if (!logfile) {
        std::cout << "File " << logFilename
                  << " does not exist. Creating a new one..." << std::endl;
        logfile.open(logFilename.c_str());
        if (!logfile) {
            std::cout << "Could not create a new logfile " << logFilename
                      << std::endl;
            return;
        }
    }

    // Log the data
    logfile << log.c_str();

    // Close the log file
    logfile.close();
}

std::string getProblemInfo(const char *mapFilename, double mapResolution,
                           double robotRadius, double *xCoords, double *yCoords,
                           int numCoords, double startX, double startY,
                           double startTheta, double goalX, double goalY,
                           double goalTheta, PathPose **path, int *pathLength,
                           double distanceBetweenPathPoints,
                           double turningRadius, PLANNER_TYPE plannerType,
                           const char *experienceDBName, MODE mode,
                           bool isHolonomicRobot)
{
    if (plannerType < PLANNER_TYPE::SIMPLE_SETUP ||
        plannerType >= PLANNER_TYPE::PLANNER_TYPE_COUNT ||
        mode != MODE::NORMAL) {
        return "";
    }

    // Construct the log message
    std::stringstream log;

    log << "\n\n====== Start of planning instance ======\n";
    log << "Map Filename: " << mapFilename << "\n";
    log << "Map Resolution: " << mapResolution << "\n";
    log << "Robot Radius: " << robotRadius << "\n";
    log << "Collision Centers: ";
    for (int i = 0; i < numCoords; i++) {
        log << "(" << xCoords[i] << ", " << yCoords[i] << ") ";
    }
    log << "\n";
    log << "Start Pose: (" << startX << ", " << startY << ", " << startTheta
        << ")"
        << "\n";
    log << "Goal Pose: (" << goalX << ", " << goalY << ", " << goalTheta << ")"
        << "\n";
    log << "Distance between points: " << distanceBetweenPathPoints << "\n";
    log << "Turning Radius: " << turningRadius << "\n";
    if (plannerType == PLANNER_TYPE::EXPERIENCE_LIGHTNING) {
        log << "Planner Type: LIGHTNING"
            << "\n";
    }
    else if (plannerType == PLANNER_TYPE::EXPERIENCE_THUNDER) {
        log << "Planner Type: THUNDER"
            << "\n";
    }
    else {
        log << "Planner Type: SIMPLE (RRT*)"
            << "\n";
    }
    log << "Is Holonomic Robot: " << (isHolonomicRobot ? "True" : "False")
        << "\n";
    log << "-------------------------------------------------" << std::endl;

    return log.str();
}

std::string
getLogTime(const std::string &tag,
           std::chrono::time_point< std::chrono::system_clock > &now)
{
    std::stringstream log;
    log << tag << ": ";
    now = std::chrono::system_clock::now();
    std::time_t time_now = std::chrono::system_clock::to_time_t(now);
    log << std::put_time(std::localtime(&time_now), "%d-%m-%Y %T") << std::endl;
    return log.str();
}

std::string getPathToLog(PathPose **path, int pathLength)
{
    std::stringstream log;
    log << "\nSolution contains " << pathLength << " nodes" << std::endl;
    log << "Generated path: \n";
    for (int i = 0; i < pathLength; i++) {
        log << (*path)[i].asString() << " ";
    }
    log << "\n" << std::endl;
    return log.str();
}

extern "C" bool
plan_multiple_circles(const char *mapFilename, double mapResolution,
                      double robotRadius, double *xCoords, double *yCoords,
                      int numCoords, double startX, double startY,
                      double startTheta, double goalX, double goalY,
                      double goalTheta, PathPose **path, int *pathLength,
                      double distanceBetweenPathPoints, double turningRadius,
                      PLANNER_TYPE plannerType, const char *experienceDBName,
                      MODE mode, bool isHolonomicRobot)
{
    std::string logFilename = getLogFileName(experienceDBName, plannerType);

    if (plannerType >= PLANNER_TYPE::SIMPLE_SETUP &&
        plannerType < PLANNER_TYPE::PLANNER_TYPE_COUNT &&
        mode == MODE::NORMAL) {
        // Setup OMPL logging stream to the log file
        ompl::msg::useOutputHandler(new ompl::msg::OutputHandlerFile(
            getLogFileName(experienceDBName, plannerType).c_str()));
        LOGGING_ACTIVE = true;
    }
    else {
        ompl::msg::noOutputHandler();
        LOGGING_ACTIVE = false;
    }

    std::cout << "Logging status: " << LOGGING_ACTIVE << logFilename << std::endl;

    std::string probInfo = getProblemInfo(
        mapFilename, mapResolution, robotRadius, xCoords, yCoords, numCoords,
        startX, startY, startTheta, goalX, goalY, goalTheta, path, pathLength,
        distanceBetweenPathPoints, turningRadius, plannerType, experienceDBName,
        mode, isHolonomicRobot);
    log(logFilename, probInfo);

    double pLen = 0.0;
    int numInterpolationPoints = 0;
    bool isReplan = (mode == MODE::REPLANNING);

    std::chrono::time_point< std::chrono::system_clock > startTime;
    log(logFilename, getLogTime("Start time", startTime));

    ob::StateSpacePtr space =
        isHolonomicRobot
            ? ob::StateSpacePtr(new ob::SE2StateSpace())
            : ob::StateSpacePtr(new ob::ReedsSheppStateSpace(turningRadius));

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

    plannerType = isReplan ? PLANNER_TYPE::SIMPLE_SETUP : plannerType;
    og::SimpleSetup *ssPtr =
        getPlanningSetup(plannerType, space, experienceDBName);

    // set state validity checking for this space
    ob::SpaceInformationPtr si(ssPtr->getSpaceInformation());
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(
        new MultipleCircleStateValidityChecker(si, mapFilename, mapResolution,
                                               robotRadius, xCoords, yCoords,
                                               numCoords)));

    ob::PlannerPtr planner;
    if (isReplan) {
        planner = ob::PlannerPtr(new og::RRTConnect(si));
    }
    else {
        // planner = ob::PlannerPtr(new og::RRTConnect(si));
        planner = ob::PlannerPtr(new og::RRTstar(si));
        // planner = ob::PlannerPtr(new og::TRRT(si));
        // planner = ob::PlannerPtr(new og::SST(si));
        // planner = ob::PlannerPtr(new og::LBTRRT(si));
        // planner = ob::PlannerPtr(new og::PRMstar(si));
        // planner = ob::PlannerPtr(new og::SPARS(si));
        // planner = ob::PlannerPtr(new og::pRRT(si));
        // planner = ob::PlannerPtr(new og::LazyRRT(si));
    }

    ssPtr->setPlanner(planner);

    ompl::tools::ExperienceSetup *ePtr =
        dynamic_cast< ot::ExperienceSetup * >(ssPtr);
    if (ePtr != NULL) {
        ompl::base::PlannerPtr repairPlanner(new og::RRTConnect(si));
        ePtr->setRepairPlanner(repairPlanner);

        // Disable planning from recall if we are generating experiences
        if (mode == MODE::EXPERIENCE_GENERATION) {
            ePtr->enablePlanningFromRecall(false);
        }
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
    // ssPtr->print();

    // attempt to solve the problem within 30 seconds of planning time
    ob::PlannerStatus solved = ssPtr->solve(30.0);

    if (solved) {
        std::cout << "Found solution:" << std::endl;
        if (plannerType == PLANNER_TYPE::SIMPLE_SETUP)
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
            ePtr->saveIfChanged();

            // Log the planning logs to the log file
            std::ostringstream stream;
            stream << "\n";
            ePtr->printLogs(stream);
            log(logFilename, stream.str());
        }
    }
    else {
        std::cout << "No solution found" << std::endl;
    }

    if (ssPtr != NULL) {
        delete ssPtr;
        ssPtr = NULL;
    }

    std::chrono::time_point< std::chrono::system_clock > endTime;
    std::string endTimeLog = getLogTime("End time", endTime);
    log(logFilename, getPathToLog(path, *pathLength));
    log(logFilename, endTimeLog);

    std::chrono::duration< double > elapsed_seconds = endTime - startTime;
    std::stringstream ss;
    ss << "Planning took ";
    ss << elapsed_seconds.count() << " seconds" << std::endl;
    log(logFilename, ss.str());
    log(logFilename, "========================================\n\n");

    return solved ? 1 : 0;
}

extern "C" bool plan_multiple_circles_nomap(
    double *xCoords, double *yCoords, int numCoords, double startX,
    double startY, double startTheta, double goalX, double goalY,
    double goalTheta, PathPose **path, int *pathLength,
    double distanceBetweenPathPoints, double turningRadius,
    PLANNER_TYPE plannerType, MODE mode, bool isHolonomicRobot)
{

    double pLen = 0.0;
    int numInterpolationPoints = 0;
    bool isReplan = (mode == MODE::REPLANNING);

    ob::StateSpacePtr space =
        isHolonomicRobot
            ? ob::StateSpacePtr(new ob::SE2StateSpace())
            : ob::StateSpacePtr(new ob::ReedsSheppStateSpace(turningRadius));

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

    plannerType = isReplan ? PLANNER_TYPE::SIMPLE_SETUP : plannerType;
    og::SimpleSetup *ssPtr = getPlanningSetup(plannerType, space, "NoMap");

    // set state validity checking for this space
    ob::SpaceInformationPtr si(ssPtr->getSpaceInformation());
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(
        new MultipleCircleStateValidityChecker(si)));

    ob::PlannerPtr planner;
    if (isReplan) {
        planner = ob::PlannerPtr(new og::RRTConnect(si));
    }
    else {
        // planner = ob::PlannerPtr(new og::RRTConnect(si));
        planner = ob::PlannerPtr(new og::RRTstar(si));
        // planner = ob::PlannerPtr(new og::TRRT(si));
        // planner = ob::PlannerPtr(new og::SST(si));
        // planner = ob::PlannerPtr(new og::LBTRRT(si));
        // planner = ob::PlannerPtr(new og::PRMstar(si));
        // planner = ob::PlannerPtr(new og::SPARS(si));
        // planner = ob::PlannerPtr(new og::pRRT(si));
        // planner = ob::PlannerPtr(new og::LazyRRT(si));
    }
    ssPtr->setPlanner(planner);

    ompl::tools::ExperienceSetup *ePtr =
        dynamic_cast< ot::ExperienceSetup * >(ssPtr);
    if (ePtr != NULL) {
        ompl::base::PlannerPtr repairPlanner(new og::RRTConnect(si));
        ePtr->setRepairPlanner(repairPlanner);

        // Disable planning from recall if we are generating experiences
        if (mode == MODE::EXPERIENCE_GENERATION) {
            ePtr->enablePlanningFromRecall(false);
        }
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
    // ssPtr->print();

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
