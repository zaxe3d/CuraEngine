/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "pathOrderOptimizer.h"
#include "utils/logoutput.h"
#include "utils/BucketGrid2D.h"
#include "utils/TravellingSalesman.h"

#define INLINE static inline

namespace cura {

/**
*
*/
void PathOrderOptimizer::optimize()
{
    bool picked[polygons.size()];
    memset(picked, false, sizeof(bool) * polygons.size());/// initialized as falses
    
    for(unsigned int i_polygon=0 ; i_polygon<polygons.size() ; i_polygon++) /// find closest point to initial starting point within each polygon +initialize picked
    {
        int best = -1;
        float bestDist = std::numeric_limits<float>::infinity();
        PolygonRef poly = polygons[i_polygon];
        for(unsigned int i_point=0; i_point<poly.size(); i_point++) /// get closest point in polygon
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
    for(unsigned int i_polygon=0 ; i_polygon<polygons.size() ; i_polygon++) /// actual path order optimizer
    {
        int best = -1;
        float bestDist = std::numeric_limits<float>::infinity();


        for(unsigned int i_polygon=0 ; i_polygon<polygons.size() ; i_polygon++)
        {
            if (picked[i_polygon] || polygons[i_polygon].size() < 1) /// skip single-point-polygons
                continue;

            assert (polygons[i_polygon].size() != 2);

            float dist = vSize2f(polygons[i_polygon][polyStart[i_polygon]] - prev_point);
            if (dist < bestDist)
            {
                best = i_polygon;
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
    for(unsigned int n=0; n<polyOrder.size(); n++) /// decide final starting points in each polygon
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
    for(unsigned int i_point=0 ; i_point<poly.size() ; i_point++)
    {
        float dist = vSize2f(poly[i_point] - prev_point);
        Point n0 = normal(poly[(i_point-1+poly.size())%poly.size()] - poly[i_point], 2000);
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
    for(unsigned int point_idx=0 ; point_idx<poly.size() ; point_idx++)
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
    if(polygons.empty()) //Nothing to do. Terminate early.
    {
        return;
    }
    
    //Since polyOrder must be filled with indices, an index in the polygons vector represents each line.
    std::vector<Cluster> line_clusters = cluster();
    
    //Define how the TSP solver should use its elements.
    std::function<std::vector<std::pair<Point, Point>>(size_t)> get_orientations = [&](size_t cluster_index) -> std::vector<std::pair<Point, Point>> //How to get the possible orientations of a cluster.
    {
        std::vector<std::pair<Point, Point>> result;
        Point start_normal = polygons[line_clusters[cluster_index][0]][polyStart[line_clusters[cluster_index][0]]]; //Start of the path, not mirrored.
        Point end_normal = polygons[line_clusters[cluster_index].back()][(polyStart[line_clusters[cluster_index].back()] - 1) % polygons[line_clusters[cluster_index].back()].size()]; //End of the path, not mirrored.
        result.push_back(std::pair<Point, Point>(start_normal, end_normal));
        result.push_back(std::pair<Point, Point>(end_normal, start_normal)); //Can also insert in reverse!
        if (line_clusters[cluster_index].size() > 1u) //If the cluster has one line, mirroring the line is equal to reversing the path. Otherwise, we must also include mirrored options.
        {
            Point start_mirrored = polygons[line_clusters[cluster_index][0]][(polyStart[line_clusters[cluster_index][0]] - 1) % polygons[line_clusters[cluster_index].back()].size()]; //Start of the path, mirrored.
            Point end_mirrored = polygons[line_clusters[cluster_index][0]][(polyStart[line_clusters[cluster_index][0]] - 1) % polygons[line_clusters[cluster_index].back()].size()]; //End of the path, mirrored.
            result.push_back(std::pair<Point, Point>(start_mirrored, end_mirrored));
            result.push_back(std::pair<Point, Point>(end_mirrored, start_mirrored));
        }
        return result;
    };
    TravellingSalesman<size_t> tspsolver(get_orientations); //Solves the macro TSP problem of ordering the clusters.
    std :: vector<size_t> cluster_orientations;
    std::vector<size_t> unoptimised(line_clusters.size());
    std::iota(unoptimised.begin(),unoptimised.end(),0);
    std :: vector<size_t> optimised = tspsolver.findPath(unoptimised, cluster_orientations, &startPoint); //Approximate the shortest path with the TSP solver.
    
    //Actually put the paths in their correct order for the output.
    polyOrder.reserve(polygons.size());
    for(size_t cluster_index = 0;cluster_index < optimised.size();cluster_index++)
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
                for (size_t polygon_index = 0; polygon_index < cluster . size(); polygon_index++)
                {
                    polyOrder . push_back(static_cast<int>(cluster[polygon_index]));
                }
            }
            else //Reversed.
            {
                for (size_t polygon_index = cluster . size(); polygon_index-- > 0; ) //Insert the lines in backward direction.
                {
                    polyOrder . push_back(static_cast<int>(cluster[polygon_index]));
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
    polyStart.resize(polygons.size()); //Polystart should always contain an entry for all polygons.
    BucketGrid2D<size_t> grid(cluster_grid_size);
    for(size_t polygon_index = 0;polygon_index < polygons.size();polygon_index++) //First put every endpoint of all lines in the grid.
    {
        grid.insert(polygons[polygon_index][0],polygon_index);
        grid.insert(polygons[polygon_index].back(),polygon_index);
    }
    
    std::vector<Cluster> clusters;
    bool picked[polygons.size()]; //For each polygon, whether it is already in a cluster.
    memset(picked,0,polygons.size()); //Initialise to false.
    for(size_t polygon_index = 0;polygon_index < polygons.size();polygon_index++) //Find clusters with nearest neighbour-ish search.
    {
        if(picked[polygon_index]) //Already in a cluster.
        {
            continue;
        }
        clusters.push_back(Cluster()); //Make a new cluster for this line.
        clusters.back().push_back(polygon_index);
        polyStart[polygon_index] = 0; //Choose one possible mirroring of the lines. This determines the start point of each line. The mirror of a cluster is also checked by the TSP solver (but the combination of directions of each individual line is determined here below).
        picked[polygon_index] = true;
        size_t current_polygon = polygon_index; //We'll do a walk to the nearest valid neighbour. A neighbour is valid if it is not picked yet and if both its endpoints are near.
        size_t best_polygon = current_polygon;
        while(best_polygon != static_cast<size_t>(-1)) //Keep going until there is no valid neighbour.
        {
            Point current_start = polygons[current_polygon][polyStart[current_polygon]]; //Start and end point of the current polygon. These are used to find the distance to the next polygon.
            Point current_end = polygons[current_polygon][(polyStart[current_polygon] - 1) % polygons[current_polygon].size()];
            best_polygon = static_cast<size_t>(-1);
            unsigned long long best_distance = cluster_grid_size * cluster_grid_size + 1; //grid_size squared since vSize2 gives squared distance.
            size_t best_start;
            for(size_t neighbour : grid.findNearbyObjects(polygons[current_polygon][0]))
            {
                if(picked[neighbour]) //Don't use neighbours that are already in another cluster.
                {
                    continue;
                }
                unsigned long long distance_start_start = vSize2(current_start - polygons[neighbour][0]);
                unsigned long long distance_start_end = vSize2(current_start - polygons[neighbour].back());
                unsigned long long distance_end_start = vSize2(current_end - polygons[neighbour][0]);
                unsigned long long distance_end_end = vSize2(current_end - polygons[neighbour].back());
                if (distance_start_start < cluster_grid_size * cluster_grid_size && distance_end_end < cluster_grid_size * cluster_grid_size) //Two lines are alongside each other.
                {
                    if (distance_end_end < best_distance) //Best neighbour and orientation so far.
                    {
                        best_polygon = neighbour;
                        best_distance = distance_end_end;
                        best_start = polygons[neighbour].size() - 1;
                    }
                }
                if (distance_start_end < cluster_grid_size * cluster_grid_size && distance_end_start < cluster_grid_size * cluster_grid_size) //Two lines are alongside each other, but in reverse direction.
                {
                    if (distance_end_start < best_distance)
                    {
                        best_polygon = neighbour;
                        best_distance = distance_end_start;
                        best_start = 0;
                    }
                }
            }
            if(best_polygon != static_cast<size_t>(-1)) //We found one.
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

}//namespace cura
