//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "TravellingSalesmanTest.h"

namespace cura
{

CPPUNIT_TEST_SUITE_REGISTRATION(TravellingSalesmanTest);

void TravellingSalesmanTest::setUp()
{
    tsp_points = new TravellingSalesman<Point>(
        [&](Point point) -> Point { return point; } //For points, just return the point itself as both start and end points.
       ,[&](Point point) -> Point { return point; }
    );
    tsp_paths = new TravellingSalesman<std::vector<Point>>(
        [&](std::vector<Point> path) -> Point { return path.empty() ? Point(0,0) : path[0]; } //For paths, return the first and last element of the path.
       ,[&](std::vector<Point> path) -> Point { return path.empty() ? Point(0,0) : path[path.size() - 1]; }
    );
}

void TravellingSalesmanTest::tearDown()
{
    delete tsp_points;
    delete tsp_paths;
}

void TravellingSalesmanTest::twoPointsTest()
{
    std::vector<Point> points;
    points.push_back(Point(0,0));
    points.push_back(Point(100,0));
    std::vector<Point> result = tsp_points->findPathNoReverse(points);
    
    bijective(points,result); //Assert that all points in the two vectors are equal.
}

void TravellingSalesmanTest::fivePointsTest()
{
    std::vector<Point> points;
    points.push_back(Point(-100,0));
    points.push_back(Point(0,-100));
    points.push_back(Point(0,100));
    points.push_back(Point(100,0));
    points.push_back(Point(0,0));
    
    std::vector<Point> result = tsp_points->findPathNoReverse(points);
    
    bijective(points,result); //Assert that all points in the two vectors are equal.
}

void TravellingSalesmanTest::bijective(std::vector<Point> first,std::vector<Point> second)
{
    CPPUNIT_ASSERT_MESSAGE("The resulting path does not have the same length as the provided point set.",first.size() == second.size());
    
    std::vector<bool> matched; //Indicates which of the elements of the second vector are already matched.
    matched.reserve(second.size());
    for(size_t i = 0;i < matched.size();i++) //Initialise all matched to false.
    {
        matched.push_back(false);
    }
    for(Point first_point : first)
    {
        for(size_t second_point_index = 0;second_point_index < second.size();second_point_index++)
        {
            if(first_point == second[second_point_index])
            {
                matched[second_point_index] = true;
                goto CONTINUE_FIRST;
            }
        }
        CPPUNIT_FAIL("A point in the original path was not included in the result.");
        
        CONTINUE_FIRST: ; //Continue with the outer loop.
    }
    for(bool match : matched) //Check if there are any unmatched points left.
    {
        CPPUNIT_ASSERT_MESSAGE("A point in the result did not come from the original path.",match);
    }
}

}