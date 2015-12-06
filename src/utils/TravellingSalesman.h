//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TRAVELLINGSALESMAN_H
#define	TRAVELLINGSALESMAN_H

#include "intpoint.h" //For the Point type.

#include <algorithm> //For std::shuffle to randomly shuffle the array.
#include <list> //For std::list, which is used to insert stuff in the result in constant time.
#include <random> //For the RNG. Note: CuraEngine should be deterministic! Always use a FIXED SEED!
#include <cassert> //For asserting that the lists returned by the function parameters have the same size.

namespace cura
{

/*!
 * \brief Struct that holds all information of one element of the path.
 * 
 * It needs to know the actual element in the path, but also where the element's
 * own path starts and ends.
 */
template<class E> struct Waypoint
{
    /*!
     * \brief Constructs a new waypoint with the specified possible start and
     * end points and the specified element.
     * 
     * \param start_points The possible start points of the waypoint, one for
     * each orientation.
     * \param end_points The possible end points of the waypoint, one for each
     * orientation.
     * \param element The element that's to be bound to this waypoint.
     */
    Waypoint(std :: vector<Point> start_points, std :: vector<Point> end_points, E element) : start_points(start_points), end_points(end_points), element(element)
    {
    }
    
    /*!
     * \brief Constructs a new waypoint with the specified possible start and
     * end point.
     * 
     * The waypoint will contain no element. Please keep track of this, as
     * trying to access the element will access uninitialised memory.
     * 
     * \param start_points The possible start points of the waypoint, one for
     * each orientation.
     * \param end_points The possible end points of the waypoint, one for each
     * orientation.
     */
    Waypoint(std :: vector<Point> start_points, std :: vector<Point> end_points) : start_points(start_points), end_points(end_points)
    {
    }
    
    /*!
     * \brief The possible starting points of the waypoint's internal path.
     * 
     * This will hold one element for each orientation in which the element can
     * be inserted in the path.
     */
    std :: vector<Point> start_points;
    
    /*!
     * \brief The possible ending points of the waypoint's internal path.
     * 
     * This will hold one element for each orientation in which the element can
     * be inserted in the path.
     */
    std :: vector<Point> end_points;
    
    /*!
     * \brief The actual element this waypoint holds.
     */
    E element;
    
    /*!
     * \brief The optimal orientation of this waypoint in the final path.
     * 
     * This is computed during the <em>TravellingSalesman::findPath</em>
     * function.
     */
    size_t orientation;
};

/*!
 * \brief A class of functions implementing solutions of Travelling Salesman.
 * 
 * Various variants can be implemented here, such as the shortest path past a
 * set of points or of lines.
 */
template<class E> class TravellingSalesman
{
    typedef typename std::list<Waypoint<E>*>::iterator ListElement; //To help the compiler with templates in templates.

public:
    /*!
     * \brief Constructs an instance of Travelling Salesman.
     * 
     * \param get_start A function to get possible starting points for elements
     * in the path. The vector contains different orientations in which the
     * elements could be traversed in the final path. This function must always
     * return a vector of the same length as the vector returned by \p get_end
     * on the same element.
     * \param get_end A function to get possible ending points for elements in
     * the path. The vector contains different orientations in which the
     * elements could be traversed in the final path. This function must always
     * return a vector of the same length as the vector returned by \p get_start
     * on the same element.
     */
    TravellingSalesman(std :: function<std :: vector<Point>(E)> get_starts, std :: function<std :: vector<Point>(E)> get_ends) : get_starts(get_starts), get_ends(get_ends)
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
     * <em>get_start</em> and <em>get_end</em>
     * This version of the Travelling Salesman approximation will also try to
     * reverse the start and end point of the elements. This will involve
     * roughly twice as much computation time as when the same elements are fed
     * to findPathNoReverse(std::vector<Element>,Point*). It may be useful for
     * when the elements may be traversed in reverse direction and this may
     * influence the total path length, such as extruding a number of lines.
     * 
     * \param elements The elements past which the path must run.
     * \param reversed_elements Output parameter to indicate for each element in
     * which orientation it must be placed to minimise the travel time. The
     * resulting integers correspond to the index of the options given by the
     * <em>get_start</em> and <em>get_end</em> constructor parameters.
     * \param starting_point A fixed starting point of the path, if any. If this
     * is <em>nullptr</em>, the path may start at the start or end point of any
     * element, depending on which the heuristic deems shortest.
     * \return A vector of elements, in an order that would make a short path.
     */
    std :: vector<E> findPath(std :: vector<E> elements, std :: vector<size_t>& element_orientations, Point* starting_point = nullptr);
    
protected:
    /*!
     * \brief Function to use to get the possible start points of an element.
     */
    std :: function<std :: vector<Point>(E)> get_starts;
    
    /*!
     * \brief Function to use to get the possible end points of an element.
     */
    std :: function<std :: vector<Point>(E)> get_ends;
    
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
        shuffle[0] -> orientation = 0; //Choose an arbitrary orientation for the first element.
        result . push_back(shuffle[0]); //Due to the check at the start, we know that shuffled always contains at least 1 element.
    }
    else //If there is a starting point, insert the initial element after it.
    {
        int64_t best_distance = std :: numeric_limits<int64_t> :: max(); //Change in travel distance to insert the waypoint. Minimise this distance by varying the orientation.
        size_t best_orientation; //In what orientation to insert the element.
        for (size_t orientation = 0; orientation < shuffle[0] -> start_points . size(); orientation++)
        {
            int64_t distance = vSize(*starting_point - shuffle[0] -> start_points[orientation]); //Distance from the starting point to the start point of this element.
            if (distance < best_distance)
            {
                best_distance = distance;
                best_orientation = orientation;
            }
        }
        shuffle[0] -> orientation = best_orientation;
        result . push_back(shuffle[0]);
    }

    for(size_t next_to_insert = 1;next_to_insert < shuffle.size();next_to_insert++) //Now randomly insert the rest of the points.
    {
        Waypoint<E>* waypoint = shuffle[next_to_insert];
        int64_t best_distance = std :: numeric_limits<int64_t> :: max(); //Change in travel distance to insert the waypoint. Minimise this distance by varying the insertion point and orientation.
        ListElement best_insert; //Where to insert the element. It will be inserted before this element. If it's nullptr, insert at the very front.
        size_t best_orientation; //In what orientation to insert the element.
        
        //First try inserting before the first element.
        for (size_t orientation = 0; orientation < waypoint -> start_points . size(); orientation++)
        {
            int64_t before_distance = 0;
            if (starting_point) //If there is a starting point, we're inserting between the first point and the starting point.
            {
                before_distance = vSize(*starting_point - waypoint -> start_points[orientation]);
            }
            int64_t after_distance = vSize(waypoint -> end_points[orientation] - (*result.begin()) -> start_points[(*result . begin()) -> orientation]); //From the end of this element to the start of the first element.
            int64_t distance = before_distance + after_distance;
            if (distance < best_distance)
            {
                best_distance = distance;
                best_insert = result . begin();
                best_orientation = orientation;
            }
        }
        for (ListElement before_insert = result . begin(); before_insert != result . end(); before_insert++) //Try inserting at the other positions.
        {
            ListElement after_insert = before_insert;
            after_insert++; //Get the element after the current element.
            if(after_insert == result.end()) //There is no next element. We're inserting at the end of the path.
            {
                for (size_t orientation = 0; orientation < waypoint -> start_points . size(); orientation++)
                {
                    int64_t distance = vSize((*before_insert) -> end_points[(*before_insert) -> orientation] - waypoint -> start_points[orientation]); //From the end of the last element to the start of this element.
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
                for (size_t orientation = 0; orientation < waypoint -> start_points . size(); orientation++)
                {
                    int64_t removed_distance = vSize((*before_insert) -> end_points[(*before_insert) -> orientation] - (*after_insert) -> start_points[(*after_insert) -> orientation]); //Distance of the original move that we'll remove.
                    int64_t before_distance = vSize((*before_insert) -> end_points[(*before_insert) -> orientation] - waypoint -> start_points[orientation]); //From end of previous element to start of this element.
                    int64_t after_distance = vSize(waypoint -> end_points[orientation] - (*after_insert) -> start_points[(*after_insert) -> orientation]); //From end of this element to start of next element.
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
        waypoint -> orientation = best_orientation;
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
        element_orientations . push_back(waypoint -> orientation);
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
        std :: vector<Point> starts = get_starts(element);
        std :: vector<Point> ends = get_ends(element);
        assert(starts . size() == ends . size()); //Functions must give an equal number of start points and end points.
        Waypoint<E>* waypoint = new Waypoint<E>(get_starts(element), get_ends(element), element); //Yes, this must be deleted when the algorithm is done!
        result.push_back(waypoint);
    }
    return result;
}

}

#endif /* TRAVELLINGSALESMAN_H */

