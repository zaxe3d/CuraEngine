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
    //Create the TSP solver that will solve all our problems.
    TravellingSalesman<size_t> tspsolver([&](size_t polygon_index) -> Point
        {
            return polygons[polygon_index][0];
        }
        ,[&](int polygon_index) -> Point
        {
            return polygons[polygon_index][polygons[polygon_index].size() - 1];
        }
    );
    
    std::vector<std::vector<size_t>> line_clusters = cluster();
    
    
    std::vector<bool> reverse_polygons;
    std::vector<size_t> unoptimised;
    unoptimised.reserve(polygons.size()); //We have to convert the polygons to their indices before shuffling in order to be able to output indices.
    for(size_t index = 0;index < polygons.size();index++)
    {
        unoptimised.push_back(index);
    }
    std::vector<size_t> optimised = tspsolver.findPath(unoptimised,reverse_polygons,&startPoint); //Approximate the solution with the TSP solver.
    
    //Actually put the paths in their correct order for the output.
    polyOrder.reserve(optimised.size());
    polyStart.reserve(optimised.size());
    for(size_t polygon = 0;polygon < optimised.size();polygon++)
    {
        polyOrder.push_back(static_cast<int>(optimised[polygon]));
        if(reverse_polygons[polygon])
        {
            polyStart.push_back(polygons[optimised[polygon]].size() - 1); //If we're traversing in reverse, start at the end of this polygon.
        }
        else
        {
            polyStart.push_back(0);
        }
    }
    
    
    /*int gridSize = 5000; // the size of the cells in the hash grid.
    BucketGrid2D<unsigned int> line_bucket_grid(gridSize);
    bool picked[polygons.size()];
    memset(picked, false, sizeof(bool) * polygons.size());/// initialized as falses
    
    for(unsigned int i_polygon=0 ; i_polygon<polygons.size() ; i_polygon++) /// find closest point to initial starting point within each polygon +initialize picked
    {
        int best = -1;
        float bestDist = std::numeric_limits<float>::infinity();
        PolygonRef poly = polygons[i_polygon];
        for(unsigned int i_point=0; i_point<poly.size(); i_point++) /// get closest point from polygon
        {
            float dist = vSize2f(poly[i_point] - startPoint);
            if (dist < bestDist)
            {
                best = i_point;
                bestDist = dist;
            }
        }
        polyStart.push_back(best);

        assert(poly.size() == 2);

        line_bucket_grid.insert(poly[0], i_polygon);
        line_bucket_grid.insert(poly[1], i_polygon);

    }


    Point incommingPerpundicularNormal(0, 0);
    Point prev_point = startPoint;
    for(unsigned int i_polygon=0 ; i_polygon<polygons.size() ; i_polygon++) /// actual path order optimizer
    {
        int best = -1;
        float bestDist = std::numeric_limits<float>::infinity();

        for(unsigned int i_close_line_polygon :  line_bucket_grid.findNearbyObjects(prev_point)) /// check if single-line-polygon is close to last point
        {
            if (picked[i_close_line_polygon] || polygons[i_close_line_polygon].size() < 1)
                continue;


            checkIfLineIsBest(i_close_line_polygon, best, bestDist, prev_point, incommingPerpundicularNormal);

        }

        if (best == -1) /// if single-line-polygon hasn't been found yet
        {
           for(unsigned int i_polygon=0 ; i_polygon<polygons.size() ; i_polygon++)
            {
                if (picked[i_polygon] || polygons[i_polygon].size() < 1) /// skip single-point-polygons
                    continue;
                assert(polygons[i_polygon].size() == 2);

                checkIfLineIsBest(i_polygon, best, bestDist, prev_point, incommingPerpundicularNormal);

            }
        }

        if (best > -1) /// should always be true; we should have been able to identify the best next polygon
        {
            assert(polygons[best].size() == 2);

            int endIdx = polyStart[best] * -1 + 1; /// 1 -> 0 , 0 -> 1
            prev_point = polygons[best][endIdx];
            incommingPerpundicularNormal = crossZ(normal(polygons[best][endIdx] - polygons[best][polyStart[best]], 1000));

            picked[best] = true;
            polyOrder.push_back(best);
        }
        else
            logError("Failed to find next closest line.\n");
    }

    Point prev_point = startPoint;
    for(unsigned int n=0; n<polyOrder.size(); n++) /// decide final starting points in each polygon
    {
        int nr = polyOrder[n];
        PolygonRef poly = polygons[nr];
        int best = -1;
        float bestDist = std::numeric_limits<float>::infinity();
        bool orientation = poly.orientation();
        for(unsigned int i=0;i<poly.size(); i++)
        {
            float dist = vSize2f(polygons[nr][i] - prev_point);
            Point n0 = normal(poly[(i+poly.size()-1)%poly.size()] - poly[i], 2000);
            Point n1 = normal(poly[i] - poly[(i + 1) % poly.size()], 2000);
            float dot_score = dot(n0, n1) - dot(crossZ(n0), n1);
            if (orientation)
                dot_score = -dot_score;
            if (dist + dot_score < bestDist)
            {
                best = i;
                bestDist = dist + dot_score;
            }
        }

        polyStart[nr] = best;
        assert(poly.size() == 2);
        prev_point = poly[best *-1 + 1]; /// 1 -> 0 , 0 -> 1

    }*/
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
    BucketGrid2D<size_t> grid(5000);
    for(size_t polygon_index = 0;polygon_index < polygons.size();polygon_index++) //First put every endpoint of all lines in the grid.
    {
        grid.insert(polygons[polygon_index][0],polygon_index);
        grid.insert(polygons[polygon_index][polygons[polygon_index].size() - 1],polygon_index);
    }
    std::vector<std::vector<size_t>> clusters;
    bool picked[polygons.size()]; //For each polygon, whether it is already picked.
    for(size_t i = 0;i < polygons.size();i++)
    {
        picked[i] = false;
    }
    for(size_t polygon_index = 0;polygon_index < polygons.size();polygon_index++) //Then greedily find all clusters.
    {
        if(picked[polygon_index]) //Already in a cluster.
        {
            continue;
        }
        clusters.push_back(std::vector<size_t>()); //Make a new cluster for anything close to this polygon.
        clusters.back().push_back(polygon_index);
        picked[polygon_index] = true;
        for(size_t other_index : grid.findNearbyObjects(polygons[polygon_index][0])) //Put all polygons in a cluster with both endpoints near.
        {
            if(picked[other_index]) //Already clustered. Skip this one.
            {
                continue;
            }
            for(size_t near_other_index : grid.findNearbyObjects(polygons[polygon_index][polygons[polygon_index].size() - 1]))
            {
                if(near_other_index == other_index) //This polygon is also near to the other endpoint.
                {
                    clusters.back().push_back(other_index); //Put it in the same cluster.
                    picked[other_index] = true;
                    break;
                }
            }
        }
    }
    return clusters;
}

}//namespace cura
