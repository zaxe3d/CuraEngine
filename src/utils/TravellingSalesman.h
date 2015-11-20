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
 */
template<class E> struct Waypoint
{
    /*!
     * \brief Constructs a new waypoint with the specified start and end point
     * and the specified element.
     * 
     * \param start_point The start point of the waypoint.
     * \param end_point The end point of the waypoint.
     * \param element The element that's to be bound to this waypoint.
     */
    Waypoint(Point start_point,Point end_point,E element) : start_point(start_point),end_point(end_point),element(element)
    {
        average_point = Point((start_point.X + end_point.X) >> 1,(start_point.Y + end_point.Y) >> 1); //Compute the average point from the start and end points.
    }
    
    /*!
     * \brief Constructs a new waypoint with the specified start and end point.
     * 
     * The waypoint will contain no element. Please keep track of this, as
     * trying to access the element will access uninitialised memory.
     * 
     * \param start_point The start point of the waypoint.
     * \param end_point The end point of the waypoint.
     */
    Waypoint(Point start_point,Point end_point) : start_point(start_point),end_point(end_point)
    {
        average_point = Point((start_point.X + end_point.X) >> 1,(start_point.Y + end_point.Y) >> 1); //Compute the average point from the start and end points.
    }
    
    /*!
     * \brief The starting point of the waypoint's own path.
     * 
     * If this waypoint contains something other than a point, this may be
     * different from the ending point.
     */
    Point start_point;
    
    /*!
     * \brief The ending point of the waypoint's own path.
     * 
     * If this waypoint contains something other than a point, this may be
     * different from the starting point.
     */
    Point end_point;
    
    /*!
     * \brief The average between <em>start_point</em> and <em>end_point</em>.
     */
    Point average_point;
    
    /*!
     * \brief The actual element this waypoint holds.
     */
    E element;
    
    /*!
     * \brief Whether or not this element should be reversed in the eventual
     * path.
     */
    bool is_reversed; //TODO: This messes up the memory alignment completely!
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
    
    /*!
     * \brief Computes a short path along all specified elements.
     * 
     * A short path is computed that includes all specified elements, but not
     * always the shortest path. Finding the shortest path is known as the
     * Travelling Salesman Problem, and this is an NP-complete problem. The
     * solution returned by this function is just a heuristic approximation.
     * 
     * This is the generic version of findPath() and findPathNoReverse().
     * Whether it allows inserting the elements backwards depends on the
     * \p allow_reverse parameter.
     * 
     * \param elements The elements past which the path must run.
     * \param reversed_elements If reversed elements are allowed, this will
     * indicate which elements are reversed. The order of these elements will
     * coincide with the elements in the returned vector. If \p allow_reverse is
     * <em>true</em>, this may not be <em>nullptr</em>.
     * \param starting_point A fixed starting point of the path, if any. If this
     * is <em>nullptr</em>, the path may start at the start point of any
     * element, depending on which the heuristic deems shortest.
     * \param allow_reverse Whether elements may be inserted backwards.
     * \return A vector of elements, in an order that would make a short path.
     * In order to reconstruct the shortest path, add moves from the endpoints
     * of each element to the appropriate endpoint of the next element,
     * depending on whether the corresponding element of the
     * \p reversed_elements parameter indicates if the element should be
     * reversed.
     */
    std::vector<E> findPath(std::vector<E> elements,std::vector<bool>* reversed_elements,Point* starting_point,bool allow_reverse);
    
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

template<class E> std::vector<E> TravellingSalesman<E>::findPath(std::vector<E> elements,std::vector<bool>& reversed_elements,Point* starting_point)
{
    return findPath(elements,&reversed_elements,starting_point,true); //Find path allowing reverse elements.
}

template<class E> std::vector<E> TravellingSalesman<E>::findPathNoReverse(std::vector<E> elements,Point* starting_point)
{
    return findPath(elements,nullptr,starting_point,false); //Find path not allowing reverse elements.
}

template<class E> std::vector<E> TravellingSalesman<E>::findPath(std::vector<E> elements,std::vector<bool>* reversed_elements,Point* starting_point,bool allow_reverse)
{
    /* This approximation algorithm of TSP implements the random insertion
     * heuristic. Random insertion has in tests proven to be almost as good as
     * Christofides (111% of the optimal path length rather than 110% on random
     * graphs) but is much faster to compute. */
    if(elements.empty())
    {
        return std::vector<E>();
    }
    if(!reversed_elements) //No way to indicate which elements are reversed.
    {
        allow_reverse = false; //Then don't reverse them at all, just for safety.
    }
    
    std::vector<Waypoint<E>*> shuffle = fillWaypoints(elements);
    auto rng = std::default_random_engine(0xDECAFF); //Always use a fixed seed! Wouldn't want it to be nondeterministic.
    std::shuffle(shuffle.begin(),shuffle.end(),rng); //"Randomly" shuffles the waypoints.
    
    std::list<Waypoint<E>*> result;
    result.push_back(shuffle[0]); //Due to the check at the start, we know that shuffled always contains at least 1 element.
    
    for(size_t next_to_insert = 1;next_to_insert < shuffle.size();next_to_insert++) //Now randomly insert the rest of the points.
    {
        Waypoint<E>* waypoint = shuffle[next_to_insert];
        int64_t best_distance = std::numeric_limits<int64_t>::max(); //Minimise this distance.
        ListElement best_insert; //Where to insert the element. It will be inserted after this element. If it's nullptr, insert at the very front.
        
        //First try inserting before the first element.
        int64_t distance = vSize(waypoint->average_point - (*result.begin())->average_point); //From this element to the first element.
        if(distance < best_distance)
        {
            best_distance = distance;
            best_insert = result.begin();
        }
        for(ListElement before_insert = result.begin();before_insert != result.end();before_insert++)
        {
            ListElement after_insert = before_insert;
            after_insert++; //Get the element after the current element.
            if(after_insert == result.end()) //There is no next element. We're inserting at the end of the path.
            {
                int64_t distance = vSize((*before_insert)->average_point - waypoint->average_point); //From the last element to this element.
                if(distance < best_distance)
                {
                    best_distance = distance;
                    best_insert = after_insert;
                }
            }
            else //There is a next element. We're inserting somewhere in the middle.
            {
                int64_t removed_distance = vSize((*before_insert)->average_point - (*after_insert)->average_point); //Distance of the original move that we'll remove.
                int64_t before_distance = vSize((*before_insert)->average_point - waypoint->average_point); //From end of previous element to start of this element.
                int64_t after_distance = vSize(waypoint->average_point - (*after_insert)->average_point); //From end of this element to start of next element.
                int64_t distance = before_distance + after_distance - removed_distance;
                if(distance < best_distance)
                {
                    best_distance = distance;
                    best_insert = after_insert;
                }
            }
        }
        //Actually insert the waypoint at the best position we found.
        if(best_insert == result.end()) //We must insert at the very start.
        {
            result.push_back(waypoint);
        }
        else //We must insert after best_insert.
        {
            result.insert(best_insert,waypoint);
        }
    }
    
    //Now determine for each element in which direction it should be placed.
    if(allow_reverse)
    {
        ListElement element = result.begin();
        (*element)->is_reversed = false; //For the first element, the direction doesn't matter, but choosing this determines the direction of the complete path (unless a starting point constrains that later).
        ListElement previous_element = element;
        element++;
        for(;element != result.end();previous_element = element,element++) //For loop continually increments the 2 elements.
        {
            int64_t forward_distance,backward_distance;
            if(!(*previous_element)->is_reversed)
            {
                forward_distance = vSize2((*previous_element)->end_point - (*element)->start_point); //From the end of the previous element to the start of this element.
                backward_distance = vSize2((*previous_element)->end_point - (*element)->end_point); //From the end of the previous element to the end of this element.
            }
            else
            {
                forward_distance = vSize2((*previous_element)->start_point - (*element)->start_point); //From the "end" of the previous element to the start of this element.
                backward_distance = vSize2((*previous_element)->start_point - (*element)->end_point); //From the "end" of the previous element to the start of this element.
            }
            (*element)->is_reversed = backward_distance < forward_distance;
        }
    }
    
    if(starting_point) //If there is a starting point, add it at the best position.
    {
        Waypoint<E>* starting_waypoint = new Waypoint<E>(*starting_point,*starting_point);
        int64_t best_distance = std::numeric_limits<int64_t>::max(); //Minimise this distance.
        ListElement best_insert; //Where to insert the element. It will be inserted after this element. If it's nullptr, insert at the very front.
        
        int64_t distance = vSize(starting_waypoint->average_point - (*result.begin())->average_point); //From the starting point to the first element.
        if(distance < best_distance)
        {
            best_distance = distance;
            best_insert = result.begin();
        }
        for(ListElement before_insert = result.begin();before_insert != result.end();before_insert++)
        {
            ListElement after_insert = before_insert;
            after_insert++; //Get the element after the current element.
            if(after_insert == result.end()) //There is no next element. We're inserting at the end of the path.
            {
                int64_t distance = vSize((*before_insert)->average_point - starting_waypoint->average_point); //From the last element to this element.
                if(distance < best_distance)
                {
                    best_distance = distance;
                    best_insert = after_insert;
                }
            }
            else //There is a next element. We're inserting somewhere in the middle.
            {
                int64_t removed_distance = vSize((*before_insert)->average_point - (*after_insert)->average_point); //Distance of the original move that we'll remove.
                int64_t after_distance = vSize(starting_waypoint->average_point - (*after_insert)->average_point); //From the starting point to the new start of the path.
                int64_t path_closing_distance = vSize((*result.begin())->average_point - (*result.end())->average_point); //We'd cycle the start of the path, which means that the original start and end of the path will be connected.
                int64_t distance = after_distance + path_closing_distance - removed_distance;
                if(distance < best_distance)
                {
                    best_distance = distance;
                    best_insert = after_insert;
                }
            }
        }
        if(best_insert == result.end()) //Starting point should be inserted at the beginning.
        {
            //Then the path is already correct.
        }
        else if(best_insert == --result.end()) //Starting point should be inserted at the end. This means that the path must be reversed.
        {
            for(Waypoint<E>* waypoint : result)
            {
                waypoint->is_reversed = !waypoint->is_reversed; //Reverse all waypoints.
            }
        }
        else //Starting point should be somewhere in the middle! Uh-oh!
        {
            while(result.begin() != best_insert) //Move elements from the beginning to the end of the list until the required insertion point is at the start.
            {
                Waypoint<E>* waypoint = *(result.begin());
                result.pop_front();
                result.push_back(waypoint);
            }
        }
    }
    
    //Now that we've inserted all points, linearise them into one vector.
    std::vector<E> result_vector;
    result_vector.reserve(elements.size());
    if(allow_reverse)
    {
        reversed_elements->clear(); //Prepare the reversed_elements vector for storing whether elements should be reversed.
        reversed_elements->reserve(elements.size());
    }
    for(Waypoint<E>* waypoint : result)
    {
        result_vector.push_back(waypoint->element);
        if(allow_reverse)
        {
            reversed_elements->push_back(waypoint->is_reversed);
        }
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
        Waypoint<E>* waypoint = new Waypoint<E>(get_start(element),get_end(element),element); //Yes, this must be deleted when the algorithm is done!
        result.push_back(waypoint);
    }
    return result;
}

}

#endif /* TRAVELLINGSALESMAN_H */

