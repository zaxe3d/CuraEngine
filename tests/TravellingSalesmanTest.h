//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TRAVELLINGSALESMANTEST_H
#define TRAVELLINGSALESMANTEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <../src/utils/TravellingSalesman.h>

namespace cura
{

class TravellingSalesmanTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(TravellingSalesmanTest);
    CPPUNIT_TEST(twoPointsTest);
    CPPUNIT_TEST(fivePointsTest);
    CPPUNIT_TEST_SUITE_END();

public:
    /*!
     * \brief Sets up the test suite to prepare for testing.
     * 
     * In this case, an instance of <em>TravellingSalesman</em> is produced with
     * pre-defined functions for getting the start and end points of a point and
     * another instance with paths.
     */
    void setUp();

    /*!
     * \brief Tears down the test suite when testing is done.
     * 
     * The two instances of TravellingSalesman are deleted.
     */
    void tearDown();

    //The test cases.
    void twoPointsTest();
    void fivePointsTest();

private:
    /*!
     * \brief A TSP solver that approximates the shortest path between a set of
     * points.
     */
    TravellingSalesman<Point>* tsp_points;

    /*!
     * \brief A TSP solver that approximates the shortest path that includes a
     * set of paths.
     */
    TravellingSalesman<std::vector<Point>>* tsp_paths;
    
    /*!
     * \brief Asserts that the two sets implied by the provided vectors are
     * bijective.
     * 
     * They are bijective if and only if all points in the first vector are also
     * in the second vector and vice versa.
     * 
     * \param first The first vector to check whether it is bijective with the
     * second vector.
     * \param second The second vector to check whether it is bijective with the
     * first vector.
     */
    void bijective(std::vector<Point> first,std::vector<Point> second);
};

}

#endif /* TRAVELLINGSALESMANTEST_H */

