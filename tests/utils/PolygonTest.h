//Copyright (c) 2016 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef POLYGONTEST_H
#define	POLYGONTEST_H

#include <clipper/clipper.hpp> //To create the paths.
#include <cppunit/TestFixture.h> //Making unit tests.
#include <cppunit/extensions/HelperMacros.h>

#include "../src/utils/polygon.h" //The polygon we're testing.

namespace cura
{

class PolygonTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(PolygonTest);
    CPPUNIT_TEST(insideTest);
    CPPUNIT_TEST_SUITE_END();

public:
    /*!
     * \brief Sets up the test suite to prepare for testing.
     */
    void setUp();

    /*!
     * \brief Tears down the test suite when testing is done.
     */
    void tearDown();

    //The actual test cases.
    void insideTest();

private:
    Polygon small_polygon;
    Polygon large_polygon;
};

}

#endif	/* POLYGONTEST_H */

