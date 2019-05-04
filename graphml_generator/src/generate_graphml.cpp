#include <cstdlib>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/tools/lightning/Lightning.h>
#include <ompl/tools/thunder/Thunder.h>
#include <string>

namespace ob = ompl::base;
namespace ot = ompl::tools;
namespace og = ompl::geometric;

using namespace mrpt::maps;

void printGraphDetails(std::vector< ob::PlannerDataPtr > &plannerDatas,
                       std::string &outputDir)
{
    system((std::string("exec rm -rf ") + outputDir + std::string("/*.graphml"))
               .c_str());
    std::cout << "\n\nGenerating GraphFiles..." << std::endl;
    for (int plannerId = 0; plannerId < plannerDatas.size(); plannerId++) {
        std::stringstream ss;
        ss << outputDir.c_str();
        ss << "/graph";
        ss << plannerId;
        ss << ".graphml";

        std::ofstream outfile(ss.str().c_str());
        plannerDatas[plannerId]->printGraphML(outfile);

        std::cout << "Generated graphml file: " << ss.str() << std::endl;
    }
}

ob::RealVectorBounds getMapBounds(std::string &mapFilename,
                                  double mapResolution)
{
    COccupancyGridMap2D gridmap;
    gridmap.loadFromBitmapFile(mapFilename.c_str(), (float)mapResolution, 0.0f,
                               0.0f);
    std::cout << "Loaded map (1) " << mapFilename << std::endl;

    ob::RealVectorBounds bounds(2);
    bounds.low[0] = gridmap.getXMin();
    bounds.low[1] = gridmap.getYMin();
    bounds.high[0] = gridmap.getXMax();
    bounds.high[1] = gridmap.getYMax();

    return bounds;
}

int main(int argc, char **argv)
{
    // Initialize parameters with default values
    double turningRadius = 4.0;
    bool useThunder = true;
    std::string databaseFilename = useThunder
                                       ? "generated/experienceDBs/thunder.db"
                                       : "generated/experienceDBs/lightning.db";
    std::string mapFileName = "maps/map1.png";
    double resolution = 0.1;
    std::string outputDir = "/../../generated/graphFiles";
    bool isHolonomicRobot = true;

    // Parse user params
    if (argc == 8) {
        turningRadius = atof(argv[1]);
        useThunder = (atoi(argv[2]) != 0);
        databaseFilename = std::string(argv[3]);
        mapFileName = std::string(argv[4]);
        resolution = atof(argv[5]);
        outputDir = std::string(argv[6]);
        isHolonomicRobot = (atoi(argv[7]) != 0);
    }
    else {
        std::cout << argc << std::endl;
        std::cout << "Six arguments expected in the below order to generate "
                     "graph files!\n"
                     "turningRadius(float)\n"
                     "useThunder(bool) (0--> False, 1 --> True)\n"
                     "exprienceDbPath(string)\n"
                     "mapFilePath(string)\n"
                     "mapResolution(float)\n"
                     "outputDirectory(string) without trailing backslash\n"
                     "isHolonomicRobot(bool) (0--> False, 1 --> True)\n"
                  << std::endl;
        return 1;
    }

    // Create a state space
    ob::StateSpacePtr space =
    isHolonomicRobot
        ? ob::StateSpacePtr(new ob::SE2StateSpace())
        : ob::StateSpacePtr(new ob::ReedsSheppStateSpace(turningRadius));

    // Get the bounds from the map and set it to the statespace
    ob::RealVectorBounds bounds = getMapBounds(mapFileName, resolution);
    space->as< ob::SE2StateSpace >()->setBounds(bounds);
    std::cout << "Bounds are [(" << bounds.low[0] << "," << bounds.low[1]
              << "),(" << bounds.high[0] << "," << bounds.high[1] << ")]"
              << std::endl;

    // Create a experience setup based on the framework
    ot::ExperienceSetup *setup = NULL;
    if (useThunder) {
        setup = new ot::Thunder(space);
    }
    else {
        setup = new ot::Lightning(space);
    }

    if (setup != NULL) {
        // Initialize the experience setup with some dummy params
        setup->setFilePath(databaseFilename.c_str());
        setup->clear();
        ob::SpaceInformationPtr si(setup->getSpaceInformation());
        ob::PlannerPtr planner(new og::RRTstar(si));
        setup->setPlanner(planner);
        setup->setup();
        setup->print();

        // Extract the Database constents and generate graph files
        std::vector< ob::PlannerDataPtr > plannerDatas;
        setup->getAllPlannerDatas(plannerDatas);
        printGraphDetails(plannerDatas, outputDir);

        delete setup;
        setup = NULL;
    }

    return 0;
}
