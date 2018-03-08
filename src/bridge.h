/** Copyright (C) 2013 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef BRIDGE_H
#define BRIDGE_H

namespace cura {
    class Polygons;
    class SliceLayer;

int bridgeAngle(Polygons outline, const SliceLayer* prevLayer, Polygons& supportedRegions);

}//namespace cura

#endif//BRIDGE_H
