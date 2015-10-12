/* 
 * File:   TravellingSalesman.cpp
 * Author: Ruben Dulek <Ruben at Ultimaker>
 * 
 * Created on 09 October 2015, 17:31
 */

#include "TravellingSalesman.h"

#include "BucketGrid2D.h" //To speed up the search.
#include <algorithm> //For std::shuffle to randomly shuffle the array.
#include <random> //For the RNG. Note: CuraEngine should be deterministic! Always use a FIXED SEED!
#include <cassert> //For debug asserts.

#define BUCKET_SIZE 5000 //Size of buckets in bucket grid. Larger increases speed for sparse objects. Smaller increases speed for dense objects.

namespace cura
{


TravellingSalesman::TravellingSalesman()
{
    //Do nothing.
}

TravellingSalesman::~TravellingSalesman()
{
    //Do nothing.
}

std::vector<Point> TravellingSalesman::findPath(std::vector<Point>& points,Point* starting_point)
{
    /* This approximation algorithm of TSP implements the random insertion
     * heuristic. Random insertion has in tests proven to be almost as good as
     * Christofides (111% of the optimal path length rather than 110% on random
     * graphs) but is much faster to compute. */
    if(points.empty())
    {
        if(!starting_point) //No starting point either.
        {
            return std::vector<Point>();
        }
        else //Result should only contain the starting point.
        {
            std::vector<Point> result;
            result.push_back(*starting_point);
            return result;
        }
    }
    
    std::vector<Waypoint*> shuffled(points.size()); //Convert all points to waypoints and shuffle them.
    for(Point& point : points)
    {
        Waypoint* waypoint = new Waypoint;
        waypoint->before = nullptr;
        waypoint->after = nullptr;
        waypoint->point = point;
        shuffled.push_back(waypoint);
    }
    auto rng = std::default_random_engine(1337); //Always use a fixed seed! Wouldn't want it to be nondeterministic.
    std::shuffle(shuffled.begin(),shuffled.end(),rng); //"Randomly" shuffles the waypoints.
    
    //Make a beginning of the path depending on whether or not we have a starting point.
    Waypoint* path_start;
    size_t next_to_insert = 0; //Due to the check at the start, we know that shuffled always contains at least 1 element.
    if(starting_point) //If we have a fixed starting point, insert it first.
    {
        path_start = new Waypoint;
        path_start->before = nullptr;
        path_start->after = nullptr;
        path_start->point = *starting_point;
    }
    else //We don't have a fixed starting point, so take the first element to begin with. In the loop later we will then also allow inserting before this point.
    {
        path_start = shuffled[next_to_insert++];
    }
    BucketGrid2D<Waypoint*> bucket_grid(5000); //Distribute all points into bins for faster lookup.
    bucket_grid.insert(path_start->point,path_start);
    
    for(;next_to_insert < shuffled.size();next_to_insert++) //Now randomly insert the rest of the points.
    {
        Waypoint* waypoint = shuffled[next_to_insert];
        std::vector<Waypoint*> nearby = bucket_grid.findNearbyObjects(waypoint->point); //Candidates for where to insert points include before and after all 'nearby' points. This is again a heuristic.
        float best_insertion_distance = std::numeric_limits<float>::infinity(); //Minimise this distance.
        Waypoint* best_insertion = nullptr;
        bool insert_after = false; //Whether to insert before or after the selected point.
        for(Waypoint* nearby_waypoint : nearby)
        {
            //First try inserting before.
            if(!starting_point || *starting_point != nearby_waypoint->point) //Never insert before the starting point if we have any.
            {
                float insertion_distance;
                if(!nearby_waypoint->before) //Beginning of the path.
                {
                    insertion_distance = distance(*waypoint,*nearby_waypoint); //Just one distance to compute.
                }
                else
                {
                    float removed_distance = distance(*(nearby_waypoint->before),*nearby_waypoint); //Distance of the original move that we'll remove.
                    float before_distance = distance(*waypoint,*(nearby_waypoint->before));
                    float after_distance = distance(*waypoint,*nearby_waypoint);
                    insertion_distance = before_distance + after_distance - removed_distance;
                }
                if(insertion_distance < best_insertion_distance) //Hey! We found a new best place to insert this.
                {
                    best_insertion = nearby_waypoint;
                    insert_after = false;
                    best_insertion_distance = insertion_distance;
                }
            }
            //Then try inserting after.
            float insertion_distance;
            if(!nearby_waypoint->after) //End of the path.
            {
                insertion_distance = distance(*waypoint,*nearby_waypoint); //Just one distance to compute.
            }
            else
            {
                float removed_distance = distance(*(nearby_waypoint->after),*nearby_waypoint); //Distance of the original move that we'll remove.
                float before_distance = distance(*waypoint,*nearby_waypoint);
                float after_distance = distance(*waypoint,*(nearby_waypoint->after));
                insertion_distance = before_distance + after_distance - removed_distance;
            }
            if(insertion_distance < best_insertion_distance) //Hey! We found a new best place to insert this.
            {
                best_insertion = nearby_waypoint;
                insert_after = true;
                best_insertion_distance = insertion_distance;
            }
        }
        assert(best_insertion); //The nearby vector was empty!
        //Actually insert the waypoint at the best position we found.
        bucket_grid.insert(best_insertion->point,best_insertion);
        if(insert_after)
        {
            if(best_insertion->after)
            {
                Waypoint* best_after = best_insertion->after;
                best_after->before = waypoint;
                waypoint->after = best_after;
            }
            best_insertion->after = waypoint;
            waypoint->before = best_insertion;
        }
        else
        {
            if(best_insertion->before)
            {
                Waypoint* best_before = best_insertion->before;
                best_before->after = waypoint;
                waypoint->before = best_before;
            }
            else //Be sure to update the start of the path.
            {
                path_start = waypoint;
            }
            best_insertion->before = waypoint;
            waypoint->after = best_insertion;
        }
    }
    
    //Now that we've inserted all points, linearise them into one vector.
    std::vector<Point> result(shuffled.size() + 1);
    while(path_start)
    {
        result.push_back(path_start->point);
        Waypoint* to_delete = path_start;
        path_start = path_start->after;
        delete to_delete;
    }
    return result;
}

float TravellingSalesman::distance(Waypoint a,Waypoint b)
{
    int dx = a.point.X - b.point.X;
    int dy = a.point.Y - b.point.Y;
    return std::sqrt(dx * dx + dy * dy);
}

}