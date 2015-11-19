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


/**
*
*/
void LineOrderOptimizer::optimize()
{
    if(polygons.empty()) //Nothing to do. Terminate early.
    {
        return;
    }
    std::vector<std::vector<size_t>> line_clusters = cluster();
    //Since we know that the lines are created diagonally, we can order lines within a cluster by the y-coordinate of intersecting x=0.
    for(std::vector<size_t> line_cluster : line_clusters)
    {
        std::vector<std::pair<long long,size_t>> intercepts; //Rather than comparing y-intersect on the go, pre-compute all y-intersects and sort pairs of lines with their y-intersects.
        intercepts.reserve(line_cluster.size());
        for(size_t line : line_cluster)
        {
            assert(polygons[line][0].X != polygons[line][1].X); //Lines should be diagonal. If they are exactly vertical, the following code will give division by zero.
            //The standard formula for converting two-point line representation of the line through (a,b) and (c,d) to y-intercept form is: y = (x(b - d)) / (a - c) + (ad - bc) / (a - c).
            //However, to compute only the y-intercept itself, we make x=0, resulting in y = (ad - bc) / (a - c).
            long long y_intercept = (polygons[line][0].X * polygons[line][1].Y - polygons[line][0].Y * polygons[line][1].X) / (polygons[line][0].X - polygons[line][1].X);
            intercepts.push_back(std::make_pair(y_intercept,line));
        }
        std::sort(intercepts.begin(),intercepts.end()); //Actually sort the lines by y-intersect.
        for(size_t line_index = 0;line_index < intercepts.size();line_index++)
        {
            line_cluster[line_index] = intercepts[line_index].second;
        }
    }
    
    TravellingSalesman<size_t> tspsolver([&](size_t cluster_index) -> Point
        {
            return polygons[line_clusters[cluster_index][0]][0];
        }
        ,[&](size_t cluster_index) -> Point
        {
            return polygons[line_clusters[cluster_index].back()].back();
        }
    ); //Solves the macro TSP problem of ordering the clusters.
    std::vector<bool> reverse_clusters;
    std::vector<size_t> unoptimised(line_clusters.size());
    std::iota(unoptimised.begin(),unoptimised.end(),0);
    std::vector<size_t> optimised = tspsolver.findPath(unoptimised,reverse_clusters,&startPoint); //Approximate the shortest path with the TSP solver.
    
    //Actually put the paths in their correct order for the output.
    polyOrder.reserve(polygons.size());
    polyStart.resize(polygons.size()); //Polystart should always contain an entry for all polygons.
    for(size_t cluster_index = 0;cluster_index < optimised.size();cluster_index++)
    {
        std::vector<size_t> cluster = line_clusters[optimised[cluster_index]];
        if(!reverse_clusters[cluster_index]) //Insert the lines in forward direction.
        {
            for(size_t polygon_index = 0;polygon_index < cluster.size();polygon_index++)
            {
                polyOrder.push_back(static_cast<int>(cluster[polygon_index]));
                polyStart[cluster[polygon_index]] = polygon_index % 2;
            }
        }
        else //Insert the lines in backward direction.
        {
            for(size_t polygon_index = 1;polygon_index < cluster.size();polygon_index++)
            {
                polyOrder.push_back(static_cast<int>(cluster[cluster.size() - polygon_index - 1]));
                polyStart[cluster[polygon_index]] = (polygon_index + 1) % 2;
            }
        }
    }
}

inline void LineOrderOptimizer::checkIfLineIsBest(unsigned int i_line_polygon, int& best, float& bestDist, Point& prev_point, Point& incommingPerpundicularNormal)
{
    { /// check distance to first point on line (0)
        float dist = vSize2f(polygons[i_line_polygon][0] - prev_point);
        dist += abs(dot(incommingPerpundicularNormal, normal(polygons[i_line_polygon][1] - polygons[i_line_polygon][0], 1000))) * 0.0001f; /// penalize sharp corners
        if (dist < bestDist)
        {
            best = i_line_polygon;
            bestDist = dist;
            polyStart[i_line_polygon] = 0;
        }
    }
    { /// check distance to second point on line (1)
        float dist = vSize2f(polygons[i_line_polygon][1] - prev_point);
        dist += abs(dot(incommingPerpundicularNormal, normal(polygons[i_line_polygon][0] - polygons[i_line_polygon][1], 1000) )) * 0.0001f; /// penalize sharp corners
        if (dist < bestDist)
        {
            best = i_line_polygon;
            bestDist = dist;
            polyStart[i_line_polygon] = 1;
        }
    }
}

std::vector<std::vector<size_t>> LineOrderOptimizer::cluster()
{
    long long grid_size = 5000; //Maximum distance of lines that get clustered.
    BucketGrid2D<size_t> grid(grid_size);
    for(size_t polygon_index = 0;polygon_index < polygons.size();polygon_index++) //First put every endpoint of all lines in the grid.
    {
        grid.insert(polygons[polygon_index][0],polygon_index);
        grid.insert(polygons[polygon_index].back(),polygon_index);
    }
    
    std::vector<std::vector<size_t>> clusters;
    bool picked[polygons.size()]; //For each polygon, whether it is already in a cluster.
    memset(picked,0,polygons.size()); //Initialise to false.
    for(size_t polygon_index = 0;polygon_index < polygons.size();polygon_index++) //Find clusters with nearest neighbour-ish search.
    {
        if(picked[polygon_index]) //Already in a cluster.
        {
            continue;
        }
        clusters.push_back(std::vector<size_t>()); //Make a new cluster for this line.
        clusters.back().push_back(polygon_index);
        picked[polygon_index] = true;
        size_t current_polygon = polygon_index; //We'll do a walk to the nearest valid neighbour. A neighbour is valid if it is not picked yet and if both its endpoints are near.
        size_t best_polygon = current_polygon;
        while(best_polygon != static_cast<size_t>(-1)) //Keep going until there is no valid neighbour.
        {
            best_polygon = static_cast<size_t>(-1);
            long long best_distance = grid_size * grid_size << 1; //grid_size squared since vSize2 gives squared distance, and *2 since both endpoints are considered.
            for(size_t neighbour : grid.findNearbyObjects(polygons[current_polygon][0]))
            {
                if(picked[neighbour]) //Don't use neighbours that are already in another cluster.
                {
                    continue;
                }
                long long distance = vSize2(polygons[current_polygon][0] - polygons[neighbour][0]) + vSize2(polygons[current_polygon].back() - polygons[neighbour].back());
                if(distance < best_distance)
                {
                    best_polygon = neighbour;
                    best_distance = distance;
                }
                distance = vSize2(polygons[current_polygon].back() - polygons[neighbour][0]) + vSize2(polygons[current_polygon][0] - polygons[neighbour].back()); //Also try reversing the direction of the line.
                if(distance < best_distance)
                {
                    best_polygon = neighbour;
                    best_distance = distance;
                }
            }
            if(best_polygon != static_cast<size_t>(-1)) //We found one.
            {
                current_polygon = best_polygon;
                clusters.back().push_back(best_polygon);
                picked[best_polygon] = true;
            }
        }
    }
    return clusters;
}

}//namespace cura
