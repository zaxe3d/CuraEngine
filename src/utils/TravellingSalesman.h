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
     * \brief Constructs a new waypoint.
     * 
     * The fields of the waypoint are not yet set. You must initialise them
     * afterwards.
     */
    Waypoint()
    {
        //Do nothing. Set the fields yourself.
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
    
    //Make a beginning of the path depending on whether or not we have a starting point.
    size_t next_to_insert = 0; //Due to the check at the start, we know that shuffled always contains at least 1 element.
    Waypoint<E>* starting_waypoint = nullptr;
    if(starting_point) //If we have a fixed starting point, insert it first.
    {
        starting_waypoint = new Waypoint<E>;
        //Note: The start point of this waypoint is never set, since it should not be used. There should never be anything before the startpoint.
        starting_waypoint->end_point = *starting_point;
        result.push_back(starting_waypoint);
    }
    else //We don't have a fixed starting point, so take the first element to begin with. In the loop later we will then also allow inserting before this point.
    {
        result.push_back(shuffle[next_to_insert++]);
    }
    
    for(;next_to_insert < shuffle.size();next_to_insert++) //Now randomly insert the rest of the points.
    {
        Waypoint<E>* waypoint = shuffle[next_to_insert];
        float best_distance = std::numeric_limits<float>::infinity(); //Minimise this distance.
        bool best_direction = false; //Direction of how to insert the element. False is normal. True is reverse.
        ListElement best_insert = result.end(); //Where to insert the element. It will be inserted after this element. If it's result.end(), insert at the very front. Sorry, it is the only unused value.
        if(!starting_point) //We have no starting point, so inserting before the first point is also allowed.
        {
            if(!allow_reverse)
            {
                float this_distance = distance(waypoint->end_point,(*result.begin())->start_point); //From end of this element to start of next element.
                if(this_distance < best_distance)
                {
                    best_distance = this_distance;
                }
            }
            else //Inserting in reverse is allowed. This means that we must try the reverse direction and also check which endpoint of the other waypoints we must latch onto.
            {
                float this_distance = distance(waypoint->end_point,(*result.begin())->is_reversed ? (*result.begin())->end_point : (*result.begin())->start_point); //From end of this element to 'start' of next element.
                if(this_distance < best_distance)
                {
                    best_distance = this_distance;
                    best_direction = false;
                }
                this_distance = distance(waypoint->start_point,(*result.begin())-> is_reversed ? (*result.begin())->end_point : (*result.begin())->start_point); //From start of this element to 'start' of next element.
                if(this_distance < best_distance)
                {
                    best_distance = this_distance;
                    best_direction = true;
                }
            }
            //Leave best_insertion at result.end(). This is to indicate that it must be inserted before the entire path!
        }
        for(ListElement before_insert = result.begin();before_insert != result.end();before_insert++)
        {
            ListElement after_insert = before_insert;
            after_insert++; //Get the element after the current element.
            if(after_insert == result.end()) //There is no next element. We're inserting at the end of the path.
            {
                if(!allow_reverse)
                {
                    float this_distance = distance((*before_insert)->end_point,waypoint->start_point); //From end of previous element to start of this element.
                    if(this_distance < best_distance)
                    {
                        best_distance = this_distance;
                        best_insert = before_insert;
                    }
                }
                else //Inserting in reverse is allowed.
                {
                    float this_distance = distance((*before_insert)->is_reversed ? (*before_insert)->start_point : (*before_insert)->end_point,waypoint->start_point); //From 'end' of previous element to start of this element.
                    if(this_distance < best_distance)
                    {
                        best_distance = this_distance;
                        best_direction = false;
                        best_insert = before_insert;
                    }
                    //Try reverse too.
                    this_distance = distance((*before_insert)->is_reversed ? (*before_insert)->start_point : (*before_insert)->end_point,waypoint->end_point); //From 'end' of previous element to end of this element.
                    if(this_distance < best_distance)
                    {
                        best_distance = this_distance;
                        best_direction = true;
                        best_insert = before_insert;
                    }
                }
            }
            else //There is a next element. We're inserting somewhere in the middle.
            {
                if(!allow_reverse)
                {
                    float removed_distance = distance((*before_insert)->end_point,(*after_insert)->start_point); //Distance of the original move that we'll remove.
                    float before_distance = distance((*before_insert)->end_point,waypoint->start_point); //From end of previous element to start of this element.
                    float after_distance = distance(waypoint->end_point,(*after_insert)->start_point); //From end of this element to start of next element.
                    float this_distance = before_distance + after_distance - removed_distance;
                    if(this_distance < best_distance)
                    {
                        best_distance = this_distance;
                        best_insert = before_insert;
                    }
                }
                else
                {
                    float removed_distance = distance((*before_insert)->is_reversed ? (*before_insert)->start_point : (*before_insert)->end_point,(*after_insert)->is_reversed ? (*after_insert)->end_point : (*after_insert)->start_point); //From 'end' of previous element to 'start' of next element.
                    float before_distance = distance((*before_insert)->is_reversed ? (*before_insert)->start_point : (*before_insert)->end_point,waypoint->start_point); //From 'end' of previous element to start of this element.
                    float after_distance = distance(waypoint->end_point,(*after_insert)->is_reversed ? (*after_insert)->end_point : (*after_insert)->start_point); //From end of this element to 'start' of next element.
                    float this_distance = before_distance + after_distance - removed_distance;
                    if(this_distance < best_distance)
                    {
                        best_distance = this_distance;
                        best_direction = false;
                        best_insert = before_insert;
                    }
                    //Try reverse too.
                    before_distance = distance((*before_insert)->is_reversed ? (*before_insert)->start_point : (*before_insert)->end_point,waypoint->end_point); //From 'end' of previous element to end of this element.
                    after_distance = distance(waypoint->start_point,(*after_insert)->is_reversed ? (*after_insert)->end_point : (*after_insert)->start_point); //From start of this element to 'start' of next element.
                    this_distance = before_distance + after_distance - removed_distance;
                    if(this_distance < best_distance)
                    {
                        best_distance = this_distance;
                        best_direction = true;
                        best_insert = before_insert;
                    }
                }
            }
        }
        //Actually insert the waypoint at the best position we found.
        waypoint->is_reversed = best_direction; //Will remain false if allow_reverse is false.
        if(best_insert == result.end()) //We must insert at the very start.
        {
            result.push_front(waypoint);
        }
        else //We must insert after best_insert.
        {
            result.insert(best_insert,waypoint);
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
        if(waypoint == starting_waypoint) //Don't include the waypoint of the starting point, if any.
        {
            continue;
        }
        result_vector.push_back(waypoint->element);
        if(allow_reverse)
        {
            reversed_elements->push_back(waypoint->is_reversed);
        }
        delete waypoint; //Free the waypoint from memory. It is no longer needed from here on, since we copied the element in it to the output.
    }
    return result_vector;
}

template<class E> float TravellingSalesman<E>::distance(Point a,Point b)
{
    int dx = a.X - b.X;
    int dy = a.Y - b.Y;
    return std::sqrt(dx * dx + dy * dy);
}

template<class E> std::vector<Waypoint<E>*> TravellingSalesman<E>::fillWaypoints(std::vector<E> elements)
{
    std::vector<Waypoint<E>*> result;
    result.reserve(elements.size());
    
    for(E element : elements) //Put every element in a waypoint.
    {
        Waypoint<E>* waypoint = new Waypoint<E>(); //Yes, this must be deleted when the algorithm is done!
        waypoint->element = element;
        waypoint->start_point = get_start(element);
        waypoint->end_point = get_end(element);
        result.push_back(waypoint);
    }
    return result;
}

}

#endif /* TRAVELLINGSALESMAN_H */

