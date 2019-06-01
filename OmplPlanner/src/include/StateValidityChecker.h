#ifndef MultipleCircleStateValidityChecker_H
#define MultipleCircleStateValidityChecker_H

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/thunder/Thunder.h>
#include <ompl/tools/lightning/Lightning.h>

using namespace mrpt::maps;
namespace ob = ompl::base;
namespace og = ompl::geometric;

class MultipleCircleStateValidityChecker : public ob::StateValidityChecker {
 public:
  const COccupancyGridMap2D* gridmap;
  float radius;
  double* xCoords;
  double* yCoords;
  int numCoords;
  bool noMap;
  
 MultipleCircleStateValidityChecker(const ob::SpaceInformationPtr &si, const COccupancyGridMap2D* map, double _radius, double* _xCoords, double* _yCoords, int _numCoords) : ob::StateValidityChecker(si) {
    noMap = false;
    radius = (float)_radius;
    xCoords = _xCoords;
    yCoords = _yCoords;
    numCoords = _numCoords;
    gridmap = map;
  }

   MultipleCircleStateValidityChecker(const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si) {
    noMap = true;
    std::cout << "Using empty map for validity checking" << std::endl;
  }

  virtual bool isValid(const ob::State *state) const;
  
};

#endif
