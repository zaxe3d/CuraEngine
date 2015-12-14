//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TRAVELLINGSALESMAN_H
#define	TRAVELLINGSALESMAN_H

#include "intpoint.h" //For the Point type.

#include <algorithm> //For std::shuffle to randomly shuffle the array.
#include <list> //For std::list, which is used to insert stuff in the result in constant time.
#include <random> //For the RNG. Note: CuraEngine should be deterministic! Always use a FIXED SEED!

namespace cura
{

/*!
 * \brief Struct that holds all information of one element of the path.
 * 
 * It needs to know the actual element in the path, but also where the element's
 * own path starts and ends.
 * 
 * \tparam E The type of element data stored in this waypoint. Note that these
 * are copied into the waypoint on construction and out of the waypoint right
 * before deletion.
 */
template<class E> struct Waypoint
{
    /*!
     * \brief Constructs a new waypoint with the specified possible start and
     * end points and the specified element.
     * 
     * \param orientations The possible start and end points of the waypoints
     * for each orientation the element could be placed in.
     * \param element The element that's to be bound to this waypoint.
     */
    Waypoint(std::vector<std::pair<Point, Point>> orientations, E element) : orientations(orientations), element(element)
    {
    }
    
    /*!
     * \brief The possible orientations in which the waypoint could be placed in
     * the path.
     * 
     * This defines in what direction or way the element in this waypoint should
     * be traversed in the final path. The Travelling Salesman solution only
     * requires the start and end point of this traversal in order to piece the
     * waypoint into the path.
     */
    std::vector<std::pair<Point, Point>> orientations;
    
    /*!
     * \brief The actual element this waypoint holds.
     */
    E element;
    
    /*!
     * \brief The optimal orientation of this waypoint in the final path.
     * 
     * This is computed during the <em>TravellingSalesman::findPath</em>
     * function. It indicates an index in \link orientations that provides the
     * shortest path.
     */
    size_t best_orientation;
};

/*!
 * \brief A class of functions implementing solutions of Travelling Salesman.
 * 
 * Various variants can be implemented here, such as the shortest path past a
 * set of points or of lines.
 * 
 * \tparam E The type of elements that must be ordered by this instance of
 * <em>TravellingSalesman</em>. Note that each element is copied twice in a run
 * of \link findPath, so if this type is difficult to copy, provide pointers to
 * the elements instead.
 */
template<class E> class TravellingSalesman
{
    typedef typename std :: list<Waypoint<E>*> :: iterator WaypointListIterator; //To help the compiler with templates in templates.

public:
    /*!
     * \brief Constructs an instance of Travelling Salesman.
     * 
     * \param get_orientations A function to get possible orientations for
     * elements in the path. Each orientation defines a possible way that the
     * element could be inserted in the path. To do that it must provide a
     * start point and an end point for each orientation.
     */
    TravellingSalesman(std::function<std::vector<std::pair<Point, Point>>(E)> get_orientations) : get_orientations(get_orientations)
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
     * The approximation will try to insert random elements at the best location
     * in the current path, thereby incrementally constructing a good path. Each
     * element can be inserted in multiple possible orientations, defined by the
     * <em>get_orientations</em> function.
     * 
     * \param elements The elements past which the path must run.
     * \param element_orientations Output parameter to indicate for each element
     * in which orientation it must be placed to minimise the travel time. The
     * resulting integers correspond to the index of the options given by the
     * <em>get_orientations</em> constructor parameter.
     * \param starting_point A fixed starting point of the path, if any. If this
     * is <em>nullptr</em>, the path may start at the start or end point of any
     * element, depending on which the heuristic deems shortest.
     * \return A vector of elements, in an order that would make a short path.
     */
    std :: vector<E> findPath(std :: vector<E> elements, std :: vector<size_t>& element_orientations, Point* starting_point = nullptr);
    
protected:
    /*!
     * \brief Function to use to get the possible orientations of an element.
     * 
     * Each orientation has a start point and an end point, in that order.
     */
    std::function<std::vector<std::pair<Point, Point>>(E)> get_orientations;
    
private:
    /*!
     * \brief Puts all elements in waypoints, caching their endpoints.
     * 
     * The <em>get_start</em> and <em>get_end</em> functions are called on each
     * element. The results are stored along with the element in a waypoint and
     * a pointer to the waypoint is added to the resulting vector.
     * 
     * Note that this creates a waypoint for each element on the heap. These
     * waypoints need to be deleted when the algorithm is done. This is why this
     * function must always stay private.
     * 
     * \param elements The elements to put into waypoints.
     * \return A vector of waypoints with the specified elements in them.
     */
    std::vector<Waypoint<E>*> fillWaypoints(std::vector<E> elements);
};

////BELOW FOLLOWS THE IMPLEMENTATION.////

template<class E> TravellingSalesman<E>::~TravellingSalesman()
{
    //Do nothing.
}

template<class E> std :: vector<E> TravellingSalesman<E> :: findPath(std :: vector<E> elements, std :: vector<size_t>& element_orientations, Point* starting_point)
{
    /* This approximation algorithm of TSP implements the random insertion
     * heuristic. Random insertion has in tests proven to be almost as good as
     * Christofides (111% of the optimal path length rather than 110% on random
     * graphs) but is much faster to compute. */
    if(elements.empty())
    {
        return std::vector<E>();
    }

    std::vector<Waypoint<E>*> shuffle = fillWaypoints(elements);
    auto rng = std::default_random_engine(0xDECAFF); //Always use a fixed seed! Wouldn't want it to be nondeterministic.
    std::shuffle(shuffle.begin(),shuffle.end(),rng); //"Randomly" shuffles the waypoints.

    std::list<Waypoint<E>*> result;

    if (!starting_point) //If there is no starting point, just insert the initial element.
    {
        shuffle[0]->best_orientation = 0; //Choose an arbitrary orientation for the first element.
        result . push_back(shuffle[0]); //Due to the check at the start, we know that shuffled always contains at least 1 element.
    }
    else //If there is a starting point, insert the initial element after it.
    {
        int64_t best_distance = std :: numeric_limits<int64_t> :: max(); //Change in travel distance to insert the waypoint. Minimise this distance by varying the orientation.
        size_t best_orientation; //In what orientation to insert the element.
        for (size_t orientation = 0; orientation < shuffle[0]->orientations.size(); orientation++)
        {
            int64_t distance = vSize(*starting_point - shuffle[0]->orientations[orientation].first); //Distance from the starting point to the start point of this element.
            if (distance < best_distance)
            {
                best_distance = distance;
                best_orientation = orientation;
            }
        }
        shuffle[0]->best_orientation = best_orientation;
        result . push_back(shuffle[0]);
    }

    //Now randomly insert the rest of the points.
    for(size_t next_to_insert = 1;next_to_insert < shuffle.size();next_to_insert++)
    {
        Waypoint<E>* waypoint = shuffle[next_to_insert];
        int64_t best_distance = std :: numeric_limits<int64_t> :: max(); //Change in travel distance to insert the waypoint. Minimise this distance by varying the insertion point and orientation.
        WaypointListIterator best_insert; //Where to insert the element. It will be inserted before this element. If it's nullptr, insert at the very front.
        size_t best_orientation; //In what orientation to insert the element.
        
        //First try inserting before the first element.
        for (size_t orientation = 0; orientation < waypoint->orientations.size(); orientation++)
        {
            int64_t before_distance = 0;
            if (starting_point) //If there is a starting point, we're inserting between the first point and the starting point.
            {
                before_distance = vSize(*starting_point - waypoint->orientations[orientation].first);
            }
            int64_t after_distance = vSize(waypoint->orientations[orientation].second - (*result.begin()) -> orientations[(*result.begin())->best_orientation].first); //From the end of this element to the start of the first element.
            int64_t distance = before_distance + after_distance;
            if (distance < best_distance)
            {
                best_distance = distance;
                best_insert = result . begin();
                best_orientation = orientation;
            }
        }

        //Try inserting at the other positions.
        for (WaypointListIterator before_insert = result . begin(); before_insert != result . end(); before_insert++)
        {
            WaypointListIterator after_insert = before_insert;
            after_insert++; //Get the element after the current element.
            if(after_insert == result.end()) //There is no next element. We're inserting at the end of the path.
            {
                for (size_t orientation = 0; orientation < waypoint->orientations.size(); orientation++)
                {
                    int64_t distance = vSize((*before_insert)->orientations[(*before_insert)->best_orientation].second - waypoint->orientations[orientation].first); //From the end of the last element to the start of this element.
                    if (distance < best_distance)
                    {
                        best_distance = distance;
                        best_insert = after_insert;
                        best_orientation = orientation;
                    }
                }
            }
            else //There is a next element. We're inserting somewhere in the middle.
            {
                for (size_t orientation = 0; orientation < waypoint->orientations.size(); orientation++)
                {
                    int64_t removed_distance = vSize((*before_insert)->orientations[(*before_insert)->best_orientation].second - (*after_insert)->orientations[(*after_insert)->best_orientation].first); //Distance of the original move that we'll remove.
                    int64_t before_distance = vSize((*before_insert)->orientations[(*before_insert)->best_orientation].second - waypoint->orientations[orientation].first); //From end of previous element to start of this element.
                    int64_t after_distance = vSize(waypoint->orientations[orientation].second - (*after_insert)->orientations[(*after_insert)->best_orientation].first); //From end of this element to start of next element.
                    int64_t distance = before_distance + after_distance - removed_distance;
                    if (distance < best_distance)
                    {
                        best_distance = distance;
                        best_insert = after_insert;
                        best_orientation = orientation;
                    }
                }
            }
        }

        //Actually insert the waypoint at the best position we found.
        waypoint->best_orientation = best_orientation;
        if (best_insert == result . end()) //We must insert at the very end.
        {
            result . push_back(waypoint);
        }
        else //We must insert before best_insert.
        {
            result . insert(best_insert, waypoint);
        }
    }

    //Now that we've inserted all points, linearise them into one vector.
    std::vector<E> result_vector;
    result_vector.reserve(elements.size());
    element_orientations . clear(); //Prepare the reversed_elements vector for storing whether elements should be reversed.
    element_orientations . reserve(elements . size());
    for(Waypoint<E>* waypoint : result)
    {
        result_vector.push_back(waypoint->element);
        element_orientations.push_back(waypoint->best_orientation);
        delete waypoint; //Free the waypoint from memory. It is no longer needed from here on, since we copied the element in it to the output.
    }
    return result_vector;
}

template<class E> std::vector<Waypoint<E>*> TravellingSalesman<E>::fillWaypoints(std::vector<E> elements)
{
    std::vector<Waypoint<E>*> result;
    result.reserve(elements.size());
    
    for(E element : elements) //Put every element in a waypoint.
    {
        Waypoint<E>* waypoint = new Waypoint<E>(get_orientations(element), element); //Yes, this must be deleted when the algorithm is done!
        result.push_back(waypoint);
    }
    return result;
}

}

#endif /* TRAVELLINGSALESMAN_H */

