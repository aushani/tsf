#ifndef _VIEWER_LIB_GA_TERRAINTRACKPADMANIPULATOR_HPP_
#define _VIEWER_LIB_GA_TERRAINTRACKPADMANIPULATOR_HPP_

// OSG
#include <osgGA/TerrainManipulator>

namespace osgGA
{

class TerrainTrackpadManipulator : public TerrainManipulator
{
  public:
    TerrainTrackpadManipulator(int flags = DEFAULT_SETTINGS);
    bool performMovement() override;

  protected:
    virtual ~TerrainTrackpadManipulator() = default;
};

}

#endif // _VIEWER_LIB_GA_TERRAINTRACKPADMANIPULATOR_HPP_

