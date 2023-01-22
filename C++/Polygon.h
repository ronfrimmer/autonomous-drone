/*
//
// Created by rbdstudent on 17/06/2021.
//

#ifndef ORB_SLAM2_POLYGON_H
#define ORB_SLAM2_POLYGON_H

#include "include/Point.h"
#include "include/Auxiliary.h"
#include "include/Line.h"
#include "Pizza.h"
#include <set>
#include <vector>
#include <algorithm>
#include "DBSCAN.h"

class Polygon {
public:
    Polygon(std::vector<Point> points, const Point &polygonCenter, bool isExit = false);

    std::vector<Point> getExitPointsByPolygon(bool isDebug = false);
    std::vector<Point> vertices;

private:
    std::vector<std::pair<Point, double>> getRawPolygonCorners();

    void smoothPolygon(int angleRange = 10);

    void createPointsWithDistance(const std::vector<Point> &CurrentPoints);

    void filterPointsInsidePolygon();

    std::vector<std::pair<double, std::vector<Point>>> getSlicesWithVariances(int currentAngle);

    std::vector<Point> points;

    std::vector<Point> getNavigationPoints(const std::vector<Point> &goodPoints, int minSamples = 15);

    std::vector<Point>
    filterCheckpoints(const std::vector<Point> &rawNavigationPoints, int minAngleDistance =  25) const;

    Point getNavigationPointFromCluster(const std::vector<Point> &cluster);

    std::vector<Point>
    filterPointsByVariances(const std::vector<std::pair<double, std::vector<Point>>> &slices, double epsilon);

    std::vector<std::pair<Point, double>> pointsWithDistance;
    std::vector<std::pair<Point, double>> pointsOutsidePolygon;
    Point polygonCenter;
    std::vector<Line> edges;
    int pizzaAngle = 30;

    bool isExit;

    std::vector<Point> getNavigationPointsByVertexSharpAngle(const std::vector<Point> &goodPoints, int maxAngle);
};


#endif //ORB_SLAM2_POLYGON_H
*/

//
// Created by rbdstudent on 17/06/2021.
//

#ifndef ORB_SLAM2_POLYGON_H
#define ORB_SLAM2_POLYGON_H

#include "include/Point.h"
#include "include/Auxiliary.h"
#include "include/Line.h"
#include "Pizza.h"
#include <set>
#include <vector>
#include <algorithm>
#include "DBSCAN.h"

class Polygon {
public:
    Polygon(std::vector<Point> points, Point polygonCenter, bool isExit = false);
    std::vector<Point> vertices;
    std::vector<Point> getExitPointsByPolygon(bool isDebug = false);
    double distanceFromPolygon(const Point& pt);


private:
    std::vector<std::pair<Point, double>> getRawPolygonCorners();

    void smoothPolygon(int angleRange = 10);

    void createPointsWithDistance();

    void filterPointsInsidePolygon();

    std::vector<std::pair<double, std::vector<Point>>> getSlicesWithVariances(int angle);

    std::vector<Point> points;

    std::vector<Point> getNavigationPoints(std::vector<Point> goodPoints, int minSamples = 15);

    std::vector<Point> filterCheckpoints(std::vector<Point> rawNavigationPoints, int minAngleDistance = 20);

    Point getNavigationPointFromCluster(std::vector<Point> cluster);

    std::vector<Point>
    filterPointsByVariances(std::vector<std::pair<double, std::vector<Point>>> slices, double epsilon);

    std::vector<std::pair<Point, double>> pointsWithDistance;
    std::vector<std::pair<Point, double>> pointsOutsidePolygon;
    Point polygonCenter;
    std::vector<Line> edges;
    int angle = 25;
    bool isExit;

    //ronaliza
    std::unordered_map<int, std::vector<std::pair<Point, double>>> Ronaliza_pizza(const Point& middle, const std::vector<std::pair<Point, double>>& pwd, int angle);
    std::vector<std::pair<int,std::pair<double,double>>> getSlicesPointsMeanAndVAR(const std::vector<std::pair<int, std::vector<std::pair<Point, double>>>>& vectorSlices);
    static bool comparisonRule(std::pair<int, std::pair<double, double>> x, std::pair<int, std::pair<double, double>> y);
    static bool comparisonRule_sliceNUM(std::pair<int, std::pair<double, double>> x, std::pair<int, std::pair<double, double>> y);
    void High_VAR_fill(std::vector<std::pair<int, std::pair<double, double>>>& VAR, std::vector<std::pair<int, double>> distance);
    static bool comparison_rule_slice(std::pair<int, std::vector<std::pair<Point, double>>> x, std::pair<int, std::vector<std::pair<Point, double>>> y);




};


#endif //ORB_SLAM2_POLYGON_H