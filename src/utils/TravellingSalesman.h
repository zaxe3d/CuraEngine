//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TRAVELLINGSALESMAN_H
#define	TRAVELLINGSALESMAN_H

#include "intpoint.h" //For the Point type.

#include <algorithm> //For std::shuffle to randomly shuffle the array.
#include <random> //For the RNG. Note: CuraEngine should be deterministic! Always use a FIXED SEED!

namespace cura
{

template<class E> struct Waypoint
{
    Waypoint* before;
    Waypoint* after;
    Point start_point;
    Point end_point;
    E element;
};

/*!
 * \brief A class of functions implementing solutions of Travelling Salesman.
 * 
 * Various variants can be implemented here, such as the shortest path past a
 * set of points or of lines.
 */
template<class E> class TravellingSalesman
{
public:
    /*!
     * \brief Constructs an instance of Travelling Salesman.
     */
    TravellingSalesman(std::function<Point(E)> get_start,std::function<Point(E)> get_end) : get_start(get_start),get_end(get_end)
    {
        //Do nothing. All parameters are already copied to fields.
    }
    
    /*!
     * \brief Destroys the instance, releasing all memory used by it.
     */
    virtual ~TravellingSalesman();
    
    /*!
     * \brief Computes a short path along all specified elements.
     * 
     * A short path is computed that includes all specified elements, but not
     * always the shortest path. Finding the shortest path is known as the
     * Travelling Salesman Problem, and this is an NP-complete problem. The
     * solution returned by this function is just a heuristic approximation.
     * 
     * This version of the Travelling Salesman approximation will also try to
     * reverse the start and end point of the elements. This will involve
     * roughly twice as much computation time as when the same elements are fed
     * to findPathNoReverse(std::vector<Element>,Point*). It may be useful for
     * when the elements may be traversed in reverse direction and this may
     * influence the total path length, such as extruding a number of lines.
     * 
     * \param elements The elements past which the path must run.
     * \param reversed_elements Output parameter to indicate which elements must
     * be reversed to get the most optimal path.
     * \param starting_point A fixed starting point of the path, if any. If this
     * is <em>nullptr</em>, the path may start at the start or end point of any
     * element, depending on which the heuristic deems shortest.
     * \return A vector of elements, in an order that would make a short path.
     */
    std::vector<E> findPath(std::vector<E> elements,std::vector<bool>& reversed_elements,Point* starting_point = nullptr);
    
    /*!
     * \brief Computes a short path along all specified elements.
     * 
     * A short path is computed that includes all specified elements, but not
     * always the shortest path. Finding the shortest path is known as the
     * Travelling Salesman Problem, and this is an NP-complete problem. The
     * solution returned by this function is just a heuristic approximation.
     * 
     * This version of the Travelling Salesman approximation does not try to
     * reverse the start and end point of the elements. This will be faster to
     * compute than when it does try to reverse the start- and endpoint of the
     * elements, since fewer possibilities will be checked. It may be useful for
     * when the direction of going through the elements doesn't matter, such as
     * when the elements are points. It may also be useful for when the
     * direction of going through the elements needs to be fixed.
     * 
     * \param elements The elements past which the path must run.
     * \param starting_point A fixed starting point of the path, if any. If this
     * is <em>nullptr</em>, the path may start at the start point of any
     * element, depending on which the heuristic deems shortest.
     * \return A vector of elements, in an order that would make a short path.
     * In order to reconstruct the shortest path, add moves from the ending
     * point of each element to the starting point of the next element.
     */
    std::vector<E> findPathNoReverse(std::vector<E> elements,Point* starting_point = nullptr);
    
protected:
    /*!
     * \brief Function to use to get the start point of an element.
     */
    std::function<Point(E)> get_start;
    
    /*!
     * \brief Function to use to get the end point of an element.
     */
    std::function<Point(E)> get_end;
    
private:
    /*!
     * \brief Helper function to compute the distance from A to B.
     * 
     * This method is written because the built-in <em>vSizeMM</em> function of
     * Clipper converts to double, which is overkill for this application.
     * 
     * \param a The first point to compute the distance from, A.
     * \param b The second point to compute the distance to, B.
     * \return The distance from A to B.
     */
    float distance(cura::Point a,cura::Point b);
};

////BELOW FOLLOWS THE IMPLEMENTATION.////

template<class E> TravellingSalesman<E>::~TravellingSalesman()
{
    //Do nothing.
}

template<class E> std::vector<E> TravellingSalesman<E>::findPath(std::vector<E> elements,std::vector<bool>& reversed_elements,Point* starting_point)
{
    return std::vector<E>();
}

template<class E> std::vector<E> TravellingSalesman<E>::findPathNoReverse(std::vector<E> elements,Point* starting_point)
{
    /* This approximation algorithm of TSP implements the random insertion
     * heuristic. Random insertion has in tests proven to be almost as good as
     * Christofides (111% of the optimal path length rather than 110% on random
     * graphs) but is much faster to compute. */
    if(elements.empty())
    {
        if(!starting_point) //No starting point either.
        {
            return std::vector<E>();
        }
        else //Result should only contain the starting point.
        {
            std::vector<E> result;
            result.push_back(*starting_point);
            return result;
        }
    }
    
    std::vector<Waypoint<E>*> shuffled; //Convert all points to waypoints and shuffle them.
    shuffled.reserve(elements.size());
    for(E& element : elements)
    {
        Waypoint<E>* waypoint = new Waypoint<E>;
        waypoint->before = nullptr;
        waypoint->after = nullptr;
        waypoint->start_point = get_start(element);
        waypoint->end_point = get_end(element);
        waypoint->element = element;
        shuffled.push_back(waypoint);
    }
    auto rng = std::default_random_engine(1337); //Always use a fixed seed! Wouldn't want it to be nondeterministic.
    std::shuffle(shuffled.begin(),shuffled.end(),rng); //"Randomly" shuffles the waypoints.
    
    //Make a beginning of the path depending on whether or not we have a starting point.
    Waypoint<E>* path_start;
    size_t next_to_insert = 0; //Due to the check at the start, we know that shuffled always contains at least 1 element.
    if(starting_point) //If we have a fixed starting point, insert it first.
    {
        path_start = new Waypoint<E>;
        path_start->before = nullptr;
        path_start->after = nullptr;
        //Note: The start point of this waypoint is never set, since it should not be used. There should never be anything before the startpoint.
        path_start->end_point = *starting_point;
    }
    else //We don't have a fixed starting point, so take the first element to begin with. In the loop later we will then also allow inserting before this point.
    {
        path_start = shuffled[next_to_insert++];
    }
    
    for(;next_to_insert < shuffled.size();next_to_insert++) //Now randomly insert the rest of the points.
    {
        Waypoint<E>* waypoint = shuffled[next_to_insert];
        float best_insertion_distance = std::numeric_limits<float>::infinity(); //Minimise this distance.
        Waypoint<E>* best_insertion = nullptr;
        if(!starting_point) //We have no starting point, so inserting before the first point is also allowed.
        {
            best_insertion_distance = distance(waypoint->end_point,path_start->start_point); //Since this is the first iteration, it is always the shortest distance.
            //Leave best_insertion at nullptr. This is to indicate that it must be inserted before the entire path!
        }
        for(Waypoint<E>* insert_after = path_start;insert_after != nullptr;insert_after = insert_after.after) //Loop through the current path to determine where to insert it.
        {
            float insertion_distance;
            if(!insert_after->after) //End of the path.
            {
                insertion_distance = distance(insert_after->end_point,waypoint->start_point); //Just one distance to compute.
            }
            else
            {
                float removed_distance = distance(insert_after->end_point,insert_after->after->start_point); //Distance of the original move that we'll remove.
                float before_distance = distance(insert_after->end_point,waypoint->start_point); //Distance from current path to waypoint.
                float after_distance = distance(waypoint->end_point,insert_after->start_point); //Distance from waypoint to current path.
                insertion_distance = before_distance + after_distance - removed_distance;
            }
            if(insertion_distance < best_insertion_distance) //Hey! We found a new best place to insert this.
            {
                best_insertion = insert_after;
                best_insertion_distance = insertion_distance;
            }
        }
        //Actually insert the waypoint at the best position we found.
        if(!best_insertion) //If this is nullptr, it should be inserted before the start of the path.
        {
            path_start->before = waypoint; //Link the two together.
            waypoint->after = path_start;
            path_start = waypoint; //Be sure to update the start of the path.
        }
        else //Insert after the best_insertion waypoint.
        {
            if(best_insertion->after) //Not the last waypoint.
            {
                Waypoint<E>* best_after = best_insertion->after;
                best_after->before = waypoint;
                waypoint->after = best_after;
            }
            best_insertion->after = waypoint;
            waypoint->before = best_insertion;
        }
    }
    
    //Now that we've inserted all points, linearise them into one vector.
    std::vector<E> result;
    result.reserve(shuffled.size());
    while(path_start)
    {
        result.push_back(path_start->element);
        Waypoint<E>* to_delete = path_start; //Pick the next element from the list.
        path_start = path_start->after; //And remove it from the list.
        delete to_delete; //Free each waypoint from memory. They are no longer required.
    }
    return result;
}

template<class E> float TravellingSalesman<E>::distance(Point a,Point b)
{
    int dx = a.X - b.X;
    int dy = a.Y - b.Y;
    return std::sqrt(dx * dx + dy * dy);
}

}

#endif /* TRAVELLINGSALESMAN_H */

