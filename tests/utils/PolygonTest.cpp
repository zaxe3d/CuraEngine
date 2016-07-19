//Copyright (c) 2016 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#define LARGE_TEST_SIZE 10000

#include "PolygonTest.h"


namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(PolygonTest);

void PolygonTest::setUp()
{
    ClipperLib::Path small_path;
    small_polygon = PolygonRef(small_path);
    small_polygon.emplace_back(0, 0);
    small_polygon.emplace_back(0, 100);
    small_polygon.emplace_back(100, 100);
    small_polygon.emplace_back(100, 0);

    ClipperLib::Path large_path;
    large_polygon = PolygonRef(large_path);
    for (size_t vertex_id = 0; vertex_id < LARGE_TEST_SIZE; ++vertex_id)
    {
        large_polygon.emplace_back(vertex_id % 2 * 1000, vertex_id * 10); //Creates a sawtooth shape.
    }
    large_polygon.emplace_back(-10, LARGE_TEST_SIZE * 10 + 10);
    large_polygon.emplace_back(-10, 0);
}

void PolygonTest::tearDown()
{
    //Nothing to do. The polygons don't need changing.
}

void PolygonTest::insideTest()
{
    CPPUNIT_ASSERT(small_polygon.inside(Point(50, 50)));
}

}
