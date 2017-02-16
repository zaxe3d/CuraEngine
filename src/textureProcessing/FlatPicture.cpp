/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#include <vector>

#include "FlatPicture.h"

namespace cura
{
    
FlatPicture::FlatPicture(const char* filename, const MeshGroup& meshgroup, GCodeExport& gcode)
: gcode(gcode)
{
    
    log("Loading %s from disk...\n", filename);
    mat.loadImage(filename);

    coord_t z = 300;
    coord_t layer_height = 300;
    
    retraction_config.distance = 6.5;
    retraction_config.primeSpeed = 25;
    retraction_config.speed = 25;
    retraction_config.zHop = 2000;
    
    // generate white lines
    std::vector<std::vector<PointWidth>> black_lines;
    std::vector<std::vector<PointWidth>> white_lines;
    generateLines(black_lines, white_lines);
    
    
//     SettingsBase settings;
//     settings.setSetting("machine_gcode_flavor", "Griffin");
//     settings.setSetting("machine_use_extruder_offset_to_offset_coords", "true");
//     settings.setSetting("machine_extruder_count", "2");
//     settings.setSetting("material_diameter", "2.85");
//     settings.setSetting("extruder_prime_pos_x", "170");
//     settings.setSetting("extruder_prime_pos_abs", "true");
//     settings.setSetting("machine_nozzle_size", "0.4");
//     settings.setSetting("retraction_prime_speed", "15");
//     settings.setSetting("machine_width", "233");
//     settings.setSetting("machine_depth", "215");
//     settings.setSetting("machine_height", "200");
//     settings.setSetting("machine_name", "UM3");
//     settings.setSetting("layer_height", "0.1");
//     MeshGroup meshgroup(&settings);
//     meshgroup.getExtruderTrain(1)->setSetting("machine_nozzle_offset_x", "18");
//     meshgroup.getExtruderTrain(1)->setSetting("machine_nozzle_offset_y", "0");
    gcode.preSetup(&meshgroup);
    
    gcode.writeCode(";START_OF_HEADER");
    gcode.writeCode(";HEADER_VERSION:0.1");
    gcode.writeCode(";FLAVOR:Griffin");
    gcode.writeCode(";GENERATOR.NAME:Cura_SteamEngine");
    gcode.writeCode(";GENERATOR.VERSION:master");
    gcode.writeCode(";GENERATOR.BUILD_DATE:2016-11-28");
    gcode.writeCode(";TARGET_MACHINE.NAME:Ultimaker 3");
    gcode.writeCode(";EXTRUDER_TRAIN.0.INITIAL_TEMPERATURE:210");
    gcode.writeCode(";EXTRUDER_TRAIN.0.MATERIAL.VOLUME_USED:730");
    gcode.writeCode(";EXTRUDER_TRAIN.0.MATERIAL.GUID:506c9f0d-e3aa-4bd4-b2d2-23e2425b1aa9");
    gcode.writeCode(";EXTRUDER_TRAIN.0.NOZZLE.DIAMETER:0.4");
    gcode.writeCode(";EXTRUDER_TRAIN.1.INITIAL_TEMPERATURE:100");
    gcode.writeCode(";EXTRUDER_TRAIN.1.MATERIAL.VOLUME_USED:628");
    gcode.writeCode(";EXTRUDER_TRAIN.1.MATERIAL.GUID:3ee70a86-77d8-4b87-8005-e4a1bc57d2ce");
    gcode.writeCode(";EXTRUDER_TRAIN.1.NOZZLE.DIAMETER:0.4");
    gcode.writeCode(";BUILD_PLATE.INITIAL_TEMPERATURE:60");
    gcode.writeCode(";PRINT.TIME:2232");
    gcode.writeCode(";PRINT.SIZE.MIN.X:118.537");
    gcode.writeCode(";PRINT.SIZE.MIN.Y:6");
    gcode.writeCode(";PRINT.SIZE.MIN.Z:0.27");
    gcode.writeCode(";PRINT.SIZE.MAX.X:181");
    gcode.writeCode(";PRINT.SIZE.MAX.Y:187");
    gcode.writeCode(";PRINT.SIZE.MAX.Z:19.67");
    gcode.writeCode(";END_OF_HEADER");
    gcode.writeCode(";Generated with Cura_SteamEngine master");
    gcode.writeCode("");
    
    for (int layer_nr = 0; layer_nr < 4; layer_nr++)
    {
        gcode.setZ(z);
        if (layer_nr == 0)
        {
            gcode.writeCode("");
            gcode.startExtruder(0);
//             gcode.writeCode("T0");
            gcode.writeCode("M104 T1 S210");
            gcode.writeCode("M109 S210");
//             gcode.writeMove(Point3(170, 6, 2), 150, 0.0);
            gcode.writePrimeTrain(travel_speed);
//             gcode.writeCode("G280");
            gcode.resetExtrusionValue();
            gcode.writeRetraction(retraction_config, true);
//             gcode.writeCode("G92 E0");
//             gcode.writeCode("G1 F1500 E-6.5");
            gcode.writeCode("");
        }
        else
        {
            gcode.writeCode("");
            gcode.writeCode("G1 F1500 E-6.5");
            gcode.switchExtruder(0, retraction_config);
//             gcode.writeCode("T0");
            gcode.writeCode("M109 S210");
//             gcode.resetExtrusionValue();
            gcode.writeRetraction(retraction_config, true);
//             gcode.writeCode("G92 E-6.5");
            gcode.writeCode("");
        }
    
        //gcode.startExtruder(0);
        gcode.writeMove(Point3(offset.X, offset.Y, z), travel_speed, 0.0);
        drawLines(black_lines, layer_height, 0); //layer_nr % 2 == 1);
        //gcode.switchExtruder(1, retraction_config);
        
        gcode.switchExtruder(1, retraction_config);
        if (layer_nr == 0)
        {
            gcode.writePrimeTrain(travel_speed);
            gcode.resetExtrusionValue();
        }
        gcode.writeRetraction(retraction_config, true);
        gcode.writeCode("M109 S210");
        /*
        if (layer_nr == 0)
        {
            gcode.writeCode("");
            gcode.switchExtruder(1, retraction_config);
//             gcode.writeCode("G1 F1500 E-6.5");
//             gcode.writeCode("T1");
            gcode.writeCode("M109 S210");
            gcode.writeMove(Point3(164, 6, 2), 150, 0.0);
            gcode.writeCode("G280");
            gcode.resetExtrusionValue();
            gcode.writeRetraction(retraction_config, true);
//             gcode.writeCode("G92 E0");
//             gcode.writeCode("G1 F1500 E-6.5");
            gcode.writeCode("");
        }
        else
        {
            gcode.writeCode("");
            gcode.writeCode("G1 F1500 E-6.5");
            gcode.writeCode("T1");
            gcode.writeCode("M109 S210");
            gcode.resetExtrusionValue();
            gcode.writeRetraction(retraction_config, true);
//             gcode.writeCode("G92 E-6.5");
            gcode.writeCode("");
        }
        */
        
        gcode.writeMove(Point3(offset.X + size.X, offset.Y, z), travel_speed, 0.0);
        drawLines(white_lines, layer_height, 0); // layer_nr % 2 == 1);
    
        layer_height = (layer_nr == 0)? 200 : 100; // layer heights: 0.3, 0.2, 0.1, 0.1, 0.1, ...
        z += layer_height;
    }
}

void FlatPicture::generateLines(std::vector< std::vector< FlatPicture::PointWidth > >& black_lines, std::vector< std::vector< FlatPicture::PointWidth > >& white_lines)
{
    assert(black_lines.empty());
    for (coord_t x = 0; x < size.X; x += line_dist)
    {
        black_lines.emplace_back();
        std::vector<PointWidth>& line = black_lines.back();
        for (coord_t y = 0; y < size.Y; y += sample_dist)
        {
            coord_t width = line_dist * (1.0 - mat.getColor((float) x / (float) size.X, (float) y / (float) size.Y, ColourUsage::GREY));
            line.push_back(PointWidth{Point(x, y), width});
        }
    }
    
    assert(white_lines.empty());
    for (unsigned int line_idx = 0; line_idx < black_lines.size() - 1; line_idx++)
    {
        const std::vector<PointWidth>& prev_white_line = black_lines[line_idx];
        const std::vector<PointWidth>& next_white_line = black_lines[line_idx + 1];
        white_lines.emplace_back();
        std::vector<PointWidth>& black_line = white_lines.back();
        assert(prev_white_line.size() == next_white_line.size());
        for (unsigned int point_idx = 0; point_idx < prev_white_line.size(); point_idx++)
        {
            const PointWidth& prev_point = prev_white_line[point_idx];
            const PointWidth& next_point = next_white_line[point_idx];
            PointWidth pw;
            pw.width = (prev_point.width + next_point.width) / 2;
            pw.location = (prev_point.location + Point(prev_point.width / 2, 0) + next_point.location - Point(next_point.width / 2, 0)) / 2;
            black_line.push_back(pw);
        }
    }
}


void FlatPicture::drawLines(const std::vector< std::vector< FlatPicture::PointWidth > >& lines, coord_t layer_height, bool transposed)
{
    for (unsigned int line_idx = 0; line_idx < lines.size(); line_idx++)
    {
        const std::vector<PointWidth>& line = lines[line_idx];
        // TODO: make connection from prev line

        int point_start_idx;
        int point_end_idx;
        int point_update_dir;
        if (line_idx % 2 == 0)
        {
            point_start_idx = 0;
            point_end_idx = line.size();
            point_update_dir = 1;
        }
        else
        {
            point_start_idx = line.size() - 1;
            point_end_idx = -1;
            point_update_dir = -1;
        }
        PointWidth prev = line[point_start_idx];
        gcode.writeMove(offset + ((transposed)? Point(prev.location.Y, prev.location.X) : prev.location), travel_speed, 0.0);
        for (int point_idx = point_start_idx + point_update_dir; point_idx * point_update_dir < point_end_idx * point_update_dir; point_idx += point_update_dir)
        {
            const PointWidth& next = line[point_idx];
            
            coord_t width = std::max(coord_t(1), (prev.width + next.width) / 2);
            double speed = nominal_speed * nominal_extrusion_width / width;
            speed = std::min(speed, max_speed);
            
            double extrusion_mm3_per_mm = flow * INT2MM(width) * INT2MM(layer_height);
            log("generating white point with width %f and speed %f\n", extrusion_mm3_per_mm / 0.3, speed);
            
            gcode.writeMove(offset + ((transposed)? Point(next.location.Y, next.location.X) : next.location), speed, extrusion_mm3_per_mm);
            
            prev = next;
        }
        
    }
}



} // namespace cura