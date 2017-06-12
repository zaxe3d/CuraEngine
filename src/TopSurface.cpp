//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill.h"
#include "LayerPlan.h"
#include "TopSurface.h"

namespace cura
{

TopSurface::TopSurface()
{
    //Do nothing. Areas stays empty.
}

TopSurface::TopSurface(const SliceMeshStorage& mesh, const size_t layer_number)
{
    //The top surface is all parts of the mesh where there's no mesh above it, so find the layer above it first.
    Polygons mesh_above;
    if (layer_number < mesh.layers.size() - 1)
    {
        mesh_above = mesh.layers[layer_number + 1].getOutlines();
    } //If this is the top-most layer, mesh_above stays empty.

    Polygons mesh_this = mesh.layers[layer_number].getOutlines();
    areas = mesh_this.difference(mesh_above);
    to_height = mesh.layers[layer_number].printZ;
    from_height = to_height;
    if (layer_number == 0)
    {
        from_height -= mesh.getSettingInMicrons("layer_height_0");
    }
    else
    {
        from_height -= mesh.getSettingInMicrons("layer_height");
    }
}

void TopSurface::sand(const SliceMeshStorage& mesh, const GCodePathConfig& line_config, LayerPlan& layer)
{
    if (areas.empty())
    {
        return; //Nothing to do.
    }
    //Generate the lines to cover the surface.
    const EFillMethod pattern = mesh.getSettingAsFillMethod("sanding_pattern");
    const coord_t line_spacing = mesh.getSettingInMicrons("sanding_line_spacing");
    const coord_t outline_offset = -mesh.getSettingInMicrons("sanding_inset");
    const coord_t line_width = line_config.getLineWidth();
    const double direction = mesh.skin_angles[layer.getLayerNr() % mesh.skin_angles.size()] + 90.0; //Always perpendicular to the skin lines.
    constexpr coord_t infill_overlap = 0;
    constexpr coord_t shift = 0;
    Infill infill_generator(pattern, areas, outline_offset, line_width, line_spacing, infill_overlap, direction, layer.z - 10, shift);
    Polygons sand_polygons;
    Polygons sand_lines;
    infill_generator.generate(sand_polygons, sand_lines);

    //Add the lines as travel moves to the layer plan.
    const float sanding_flow = mesh.getSettingAsRatio("sanding_flow");
    if (!sand_polygons.empty())
    {
        layer.addPolygonsByOptimizer(sand_polygons, &line_config, nullptr, EZSeamType::SHORTEST, Point(0, 0), 0, false, sanding_flow);
    }
    if (!sand_lines.empty())
    {
        layer.addLinesByOptimizer(sand_lines, &line_config, SpaceFillType::PolyLines, 0, sanding_flow);
    }
}

void TopSurface::sandBelow(const SliceMeshStorage& mesh, const GCodePathConfig& line_config, const TopSurface& top_surface_below, LayerPlan& layer)
{
    const coord_t line_spacing = mesh.getSettingInMicrons("sanding_line_spacing");
    const double sanding_flow = mesh.getSettingAsRatio("sanding_flow");
    const unsigned int number_of_passes = mesh.getSettingAsCount("sanding_passes");
    const double nozzle_angle = mesh.getSettingInAngleRadians("machine_nozzle_expansion_angle");
    const coord_t minimum_sand_distance = (to_height - from_height) / tan(nozzle_angle);
    const coord_t minimum_sand_distance2 = minimum_sand_distance * minimum_sand_distance;

    std::vector<Point> below_perimeter_points = top_surface_below.areas.perimeterPoints(line_spacing);
    if (below_perimeter_points.empty())
    {
        return;
    }
    const Point first_low_point = below_perimeter_points[0];
    for (unsigned int pass = 0; pass < number_of_passes; ++pass)
    {
        layer.addTravel_simple(first_low_point); //Move above the first point so we don't collide with our model when doing the initial travel move.
        for (Point low_point : below_perimeter_points)
        {
            Point high_point(low_point);
            PolygonUtils::moveInside(areas, high_point, 0); //Move to the edge of the polygon.
            if (vSize2(high_point - low_point) < minimum_sand_distance2) //Nozzle angle doesn't allow us to move like this.
            {
                continue;
            }
            const Point3 high_point3(high_point.X, high_point.Y, layer.z);
            const Point3 low_point3(low_point.X, low_point.Y, top_surface_below.from_height);
            layer.addTravel_simple(low_point3);
            layer.addExtrusionMove(high_point3, &line_config, SpaceFillType::Lines, sanding_flow);
        }
    }
}

}