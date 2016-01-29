/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "pathOrderOptimizer.h"
#include "utils/logoutput.h"
#include "utils/BucketGrid2D.h"
#include "utils/TravellingSalesman.h"

#define INLINE static inline

namespace cura
{

/**
 *
 */
void PathOrderOptimizer::optimize()
{
    bool picked[polygons.size()];
    memset(picked, false, sizeof (bool) * polygons.size()); /// initialized as falses

    for (unsigned int i_polygon = 0; i_polygon < polygons.size(); i_polygon++) /// find closest point to initial starting point within each polygon +initialize picked
    {
        int best = -1;
        float bestDist = std::numeric_limits<float>::infinity();
        PolygonRef poly = polygons[i_polygon];
        for (unsigned int i_point = 0; i_point < poly.size(); i_point++) /// get closest point in polygon
        {
            float dist = vSize2f(poly[i_point] - startPoint);
            if (dist < bestDist)
            {
                best = i_point;
                bestDist = dist;
            }
        }
        polyStart.push_back(best);
        //picked.push_back(false); /// initialize all picked values as false

        assert(poly.size() != 2);
    }


    Point prev_point = startPoint;
    for (unsigned int polygon_idx = 0; polygon_idx < polygons.size(); polygon_idx++) /// actual path order optimizer
    {
        int best = -1;
        float bestDist = std::numeric_limits<float>::infinity();


        for (unsigned int polygon2_idx = 0; polygon2_idx < polygons.size(); polygon2_idx++)
        {
            if (picked[polygon2_idx] || polygons[polygon2_idx].size() < 1) /// skip single-point-polygons
                continue;

            assert(polygons[polygon2_idx].size() != 2);

            float dist = vSize2f(polygons[polygon2_idx][polyStart[polygon2_idx]] - prev_point);
            if (dist < bestDist)
            {
                best = polygon2_idx;
                bestDist = dist;
            }

        }


        if (best > -1) /// should always be true; we should have been able to identify the best next polygon
        {
            assert(polygons[best].size() != 2);

            prev_point = polygons[best][polyStart[best]];

            picked[best] = true;
            polyOrder.push_back(best);
        }
        else
            logError("Failed to find next closest polygon.\n");
    }

    prev_point = startPoint;
    for (unsigned int n = 0; n < polyOrder.size(); n++) /// decide final starting points in each polygon
    {
        int poly_idx = polyOrder[n];
        int point_idx = getPolyStart(prev_point, poly_idx);
        polyStart[poly_idx] = point_idx;
        prev_point = polygons[poly_idx][point_idx];

    }
}

int PathOrderOptimizer::getPolyStart(Point prev_point, int poly_idx)
{
    switch (type)
    {
        case EZSeamType::BACK:      return getFarthestPointInPolygon(poly_idx);
        case EZSeamType::RANDOM:    return getRandomPointInPolygon(poly_idx);
        case EZSeamType::SHORTEST:  return getClosestPointInPolygon(prev_point, poly_idx);
        default:                    return getClosestPointInPolygon(prev_point, poly_idx);
    }
}

int PathOrderOptimizer::getClosestPointInPolygon(Point prev_point, int poly_idx)
{
    PolygonRef poly = polygons[poly_idx];
    int best_point_idx = -1;
    float bestDist = std::numeric_limits<float>::infinity();
    bool orientation = poly.orientation();
    for (unsigned int i_point = 0; i_point < poly.size(); i_point++)
    {
        float dist = vSize2f(poly[i_point] - prev_point);
        Point n0 = normal(poly[(i_point - 1 + poly.size()) % poly.size()] - poly[i_point], 2000);
        Point n1 = normal(poly[i_point] - poly[(i_point + 1) % poly.size()], 2000);
        float dot_score = dot(n0, n1) - dot(crossZ(n0), n1); /// prefer binnenbocht
        if (orientation)
            dot_score = -dot_score;
        if (dist + dot_score < bestDist)
        {
            best_point_idx = i_point;
            bestDist = dist;
        }
    }
    return best_point_idx;
}

int PathOrderOptimizer::getRandomPointInPolygon(int poly_idx)
{
    return rand() % polygons[poly_idx].size();
}

int PathOrderOptimizer::getFarthestPointInPolygon(int poly_idx)
{
    PolygonRef poly = polygons[poly_idx];
    int best_point_idx = -1;
    float best_y = std::numeric_limits<float>::min();
    for (unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
    {
        if (poly[point_idx].Y > best_y)
        {
            best_point_idx = point_idx;
            best_y = poly[point_idx].Y;
        }
    }
    return best_point_idx;
}

LineOrderOptimizer::LineOrderOptimizer(const Point& start_point, unsigned long long cluster_grid_size)
: cluster_grid_size(cluster_grid_size == 0 ? 2000 : cluster_grid_size) //Initialise cluster_grid_size to 2000 if the input grid size is invalid (e.g. no infill).
{
    this->startPoint = start_point;
}

/**
 *
 */
void LineOrderOptimizer::optimize()
{
    if (lines.empty()) //Nothing to do. Terminate early.
    {
        return;
    }

    //Since polyOrder must be filled with indices, an index in the polygons vector represents each line.
    std::vector<Cluster> line_clusters = cluster();

    //Define how the TSP solver should use its elements.
    std::function<std::vector<std::pair<Point, Point>> (size_t)> get_orientations = [&](size_t cluster_index)->std::vector<std::pair<Point, Point>> //How to get the possible orientations of a cluster.
    {
        std::vector<std::pair<Point, Point>> result;
        const size_t first_line_index = line_clusters[cluster_index][0]; //The first line in the current cluster.
        const size_t last_line_index = line_clusters[cluster_index].back(); //The last line in the current cluster.
        const PolygonRef first_line = lines[first_line_index];
        const PolygonRef last_line = lines[last_line_index];
        const Point start_normal = first_line[polyStart[first_line_index]]; //Start of the path, not mirrored.
        const Point end_normal = last_line[(polyStart[last_line_index] + last_line.size() - 1) % last_line.size()]; //End of the path, not mirrored.
        result.push_back(std::pair<Point, Point>(start_normal, end_normal));
        result.push_back(std::pair<Point, Point>(end_normal, start_normal)); //Can also insert in reverse!
        if (line_clusters[cluster_index].size() > 1u) //If the cluster has one line, mirroring the line is equal to reversing the path. Otherwise, we must also include mirrored options.
        {
            const Point start_mirrored = first_line[(polyStart[first_line_index] + first_line.size() - 1) % last_line.size()]; //Start of the path, mirrored.
            const Point end_mirrored = first_line[(polyStart[first_line_index] + first_line.size() - 1) % last_line.size()]; //End of the path, mirrored.
            result.push_back(std::pair<Point, Point>(start_mirrored, end_mirrored));
            result.push_back(std::pair<Point, Point>(end_mirrored, start_mirrored));
        }
        return result;
    };
    TravellingSalesman<size_t> tspsolver(get_orientations); //Solves the macro TSP problem of ordering the clusters.
    std::vector<size_t> cluster_orientations;
    std::vector<size_t> unoptimised(line_clusters.size());
    std::iota(unoptimised.begin(), unoptimised.end(), 0);
    std::vector<size_t> optimised = tspsolver.findPath(unoptimised, cluster_orientations, &startPoint); //Approximate the shortest path with the TSP solver.

    //Actually put the paths in their correct order for the output.
    polyOrder.reserve(lines.size());
    for (size_t cluster_index = 0; cluster_index < optimised.size(); cluster_index++)
    {
        Cluster cluster = line_clusters[optimised[cluster_index]];
        //Determine in what orientation we should place the cluster depending on cluster_orientations[cluster_index].
        size_t orientation = cluster_orientations[cluster_index];
        if (cluster.size() == 1) //Singleton clusters have only 2 possible orientations.
        {
            polyOrder.push_back(static_cast<int>(cluster[0]));
            if (orientation >= 1)
            {
                polyStart[cluster[0]] = 1 - polyStart[cluster[0]];
            }
        }
        else //Larger clusters have 4 possible orientations.
        {
            if ((orientation & 1) == 0) //Not reversed.
            {
                for (size_t polygon_index = 0; polygon_index < cluster.size(); polygon_index++)
                {
                    polyOrder.push_back(static_cast<int>(cluster[polygon_index]));
                }
            }
            else //Reversed.
            {
                for (size_t polygon_index = cluster.size(); polygon_index-- > 0; ) //Insert the lines in backward direction.
                {
                    polyOrder.push_back(static_cast<int>(cluster[polygon_index]));
                }
            }
            if (orientation >= 2u) //Mirrored.
            {
                for (size_t polygon_index : cluster) //Mirror each line in the cluster.
                {
                    polyStart[polygon_index] = 1 - polyStart[polygon_index];
                }
            }
        }
    }
}

std::vector<std::vector<size_t>> LineOrderOptimizer::cluster()
{
    polyStart.resize(lines.size()); //Polystart should always contain an entry for all polygons.
    BucketGrid2D<size_t> grid(cluster_grid_size);
    for (size_t polygon_index = 0; polygon_index < lines.size(); polygon_index++) //First put every endpoint of all lines in the grid.
    {
        grid.insert(lines[polygon_index][0], polygon_index);
        grid.insert(lines[polygon_index].back(), polygon_index);
    }

    std::vector<Cluster> clusters;
    bool picked[lines.size()]; //For each polygon, whether it is already in a cluster.
    memset(picked, 0, lines.size()); //Initialise to false.
    for (size_t polygon_index = 0; polygon_index < lines.size(); polygon_index++) //Find clusters with nearest neighbour-ish search.
    {
        if (picked[polygon_index]) //Already in a cluster.
        {
            continue;
        }
        clusters.push_back(Cluster()); //Make a new cluster for this line.
        clusters.back().push_back(polygon_index);
        polyStart[polygon_index] = 0; //Choose one possible mirroring of the lines. This determines the start point of each line. The mirror of a cluster is also checked by the TSP solver (but the combination of directions of each individual line is determined here below).
        picked[polygon_index] = true;
        size_t current_polygon = polygon_index; //We'll do a walk to the nearest valid neighbour. A neighbour is valid if it is not picked yet and if both its endpoints are near.
        size_t best_polygon = current_polygon;
        while (best_polygon != static_cast<size_t>(-1)) //Keep going until there is no valid neighbour.
        {
            const PolygonRef current_line = lines[current_polygon];
            Point current_start = current_line[polyStart[current_polygon]]; //Start and end point of the current polygon. These are used to find the distance to the next polygon.
            Point current_end = current_line[(polyStart[current_polygon] + current_line.size() - 1) % current_line.size()];
            best_polygon = static_cast<size_t>(-1);
            unsigned long long best_distance = cluster_grid_size * cluster_grid_size + 1; //grid_size squared since vSize2 gives squared distance.
            size_t best_start;
            for (size_t neighbour : grid.findNearbyObjects(current_line[0]))
            {
                if (picked[neighbour]) //Don't use neighbours that are already in another cluster.
                {
                    continue;
                }
                tryCluster(neighbour, current_start, current_end, &best_polygon, &best_distance, &best_start);
            }
            if (best_polygon != static_cast<size_t>(-1)) //We found one.
            {
                current_polygon = best_polygon;
                clusters.back().push_back(best_polygon);
                polyStart[best_polygon] = best_start;
                picked[best_polygon] = true;
            }
        }
    }
    return clusters;
}

void LineOrderOptimizer::tryCluster(size_t line, const Point current_start, const Point current_end, size_t* best_polygon, unsigned long long* best_distance, size_t* best_start)
{
    //Input checking.
    if (!best_polygon || !best_distance || !best_start) //Output parameter missing.
    {
        return;
    }
    
    const unsigned long long distance_start_start = vSize2(current_start - lines[line][0]);
    const unsigned long long distance_start_end = vSize2(current_start - lines[line].back());
    const unsigned long long distance_end_start = vSize2(current_end - lines[line][0]);
    const unsigned long long distance_end_end = vSize2(current_end - lines[line].back());
    if (distance_start_start < cluster_grid_size * cluster_grid_size && distance_end_end < cluster_grid_size * cluster_grid_size) //Two lines are alongside each other.
    {
        if (distance_end_end < *best_distance) //Best neighbour and orientation so far.
        {
            *best_polygon = line;
            *best_distance = distance_end_end;
            *best_start = lines[line].size() - 1;
        }
    }
    if (distance_start_end < cluster_grid_size * cluster_grid_size && distance_end_start < cluster_grid_size * cluster_grid_size) //Two lines are alongside each other, but in reverse direction.
    {
        if (distance_end_start < *best_distance)
        {
            *best_polygon = line;
            *best_distance = distance_end_start;
            *best_start = 0;
        }
    }
}

}//namespace cura
