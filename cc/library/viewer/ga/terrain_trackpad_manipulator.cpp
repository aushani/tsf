/*
 * viewer/lib/ga/terrain_trackpad_manipulator.cpp
 */

// local
#include "terrain_trackpad_manipulator.hpp"

namespace osgGA
{

TerrainTrackpadManipulator::TerrainTrackpadManipulator(int flags) :
        TerrainManipulator(flags)
{
}

// copied mostly from StandardManipulator (that's what we're overriding)
// Make movement step of manipulator. Returns true if any movement was made.
bool TerrainTrackpadManipulator::performMovement()
{
    // return if less then two events have been added
    if(_ga_t0.get() == NULL || _ga_t1.get() == NULL)
        return false;

    // get delta time
    double eventTimeDelta = _ga_t0->getTime() - _ga_t1->getTime();
    if(eventTimeDelta < 0.) {
        OSG_WARN << "Manipulator warning: eventTimeDelta = "
                 << eventTimeDelta << std::endl;
        eventTimeDelta = 0.;
    }

    // get deltaX and deltaY
    float dx = _ga_t0->getXnormalized() - _ga_t1->getXnormalized();
    float dy = _ga_t0->getYnormalized() - _ga_t1->getYnormalized();

    // return if there is no movement.
    if(dx == 0. && dy == 0.)
        return false;

    // call appropriate methods
    unsigned int buttonMask = _ga_t1->getButtonMask();

    // TerrainTrackpadManipulator
    // Panning: left, originally middle
    // Zooming: middle, originally right
    // Rotating: right, originally left
    if (buttonMask == GUIEventAdapter::LEFT_MOUSE_BUTTON) {
        return performMovementMiddleMouseButton(eventTimeDelta, dx, dy);
    }
    else if (buttonMask == GUIEventAdapter::MIDDLE_MOUSE_BUTTON ||
             buttonMask == (GUIEventAdapter::LEFT_MOUSE_BUTTON |
                            GUIEventAdapter::RIGHT_MOUSE_BUTTON)) {
        return performMovementRightMouseButton(eventTimeDelta, dx, dy);
    }
    else if (buttonMask == GUIEventAdapter::RIGHT_MOUSE_BUTTON) {
        return performMovementLeftMouseButton(eventTimeDelta, dx, dy);
    }

    return false;
}

} // namespace osgGA
