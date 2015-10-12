//Copyright (c) 2015 Ultimaker B.V.
//UltiScanTastic is released under the terms of the AGPLv3 or higher.

#ifndef TRAVELLINGSALESMAN_H
#define	TRAVELLINGSALESMAN_H

#include "intpoint.h" //For the Point type.

namespace cura
{

struct Waypoint
{
    Waypoint* before;
    Waypoint* after;
    Point point;
};

/*!
 * \brief A class of functions implementing solutions of Travelling Salesman.
 * 
 * Various variants can be implemented here, such as the shortest path past a
 * set of points or of lines.
 */
class TravellingSalesman
{
public:
    /*!
     * \brief Constructs an instance of Travelling Salesman.
     */
    TravellingSalesman();
    
    /*!
     * \brief Destroys the instance, releasing all memory used by it.
     */
    virtual ~TravellingSalesman();
    
    /*!
     * \brief Computes a short path along all specified points.
     * 
     * A short path is computed that includes all specified points, but not
     * always the shortest path. Finding the shortest path is known as the
     * Travelling Salesman Problem, and this is an NP-complete problem. The
     * solution returned by this function is just an approximation.
     * 
     * \param points The waypoints past which the path must run.
     * \param starting_point A fixed starting point of the path, if any. If this
     * is <em>nullptr</em>, the path may start at any waypoint.
     */
    std::vector<Point> findPath(std::vector<Point>& points,Point* starting_point = nullptr);
    
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
    float distance(cura::Waypoint a,cura::Waypoint b);
};

}

#endif	/* TRAVELLINGSALESMAN_H */

