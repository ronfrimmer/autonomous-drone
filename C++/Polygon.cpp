//
// Created by rbdstudent on 17/06/2021.
//

#include "include/Polygon.h"
#include<iostream>
#include<sstream>
#include<fstream>
#include<iomanip>


Polygon::Polygon(std::vector<Point> points,Point polygonCenter, bool isExit) {
    this->points = points;
    this->isExit = isExit;
    this->polygonCenter = polygonCenter;
}



std::vector<Point> Polygon::filterCheckpoints(std::vector<Point> rawNavigationPoints, int minAngleDistance) {
    std::vector<std::pair<double, Point>> pointsWithAngles;
    auto center = polygonCenter;
    std::sort(rawNavigationPoints.begin(), rawNavigationPoints.end(), [&center](const Point &p1, const Point &p2) {
        return Auxiliary::calculateDistanceXY(p1, center) > Auxiliary::calculateDistanceXY(p2, center);
    });
    for (Point point : rawNavigationPoints) {
        double angle = Auxiliary::getAngleFromSlope((point.y - polygonCenter.y) / (point.x - polygonCenter.x));
        angle += angle < 0 ? 180 : 0;
        if (point.y < polygonCenter.y) {
            while (angle < 180) {
                angle += 180;
            }
        }
        pointsWithAngles.push_back({angle, point});
    }
    std::vector<Point> goodCheckpoints;
    pointsWithAngles.push_back(pointsWithAngles.front());
    for (auto firstAngle = pointsWithAngles.begin(); firstAngle < pointsWithAngles.end() - 1; ++firstAngle) {
        bool toAdd = true;
        for (auto secondAngle = firstAngle + 1; secondAngle < pointsWithAngles.end(); secondAngle++) {
            if (abs(firstAngle->first - secondAngle->first) < minAngleDistance) {
                toAdd = false;
                break;
            }
        }
        if (toAdd) {
            goodCheckpoints.push_back(firstAngle->second);
        }
    }
    goodCheckpoints.push_back(pointsWithAngles.back().second);
    return goodCheckpoints;
}

std::vector<Point> Polygon::getNavigationPoints(std::vector<Point> goodPoints, int minSamples) {
    auto dbscan = DBSCAN(minSamples, 0.15, goodPoints);
    int numberOfClusters = dbscan.run();
    if (!numberOfClusters){
        return std::vector<Point>{};
    }
    std::vector<Point> clusteredPoints = dbscan.getPoints();
    std::vector<Point> navigationPoints;
    std::sort(clusteredPoints.begin(), clusteredPoints.end(), [](Point point1, Point point2) {
        return point1.label < point2.label;
    });
    int currentLabel = 1;
    auto it = std::find_if(clusteredPoints.begin(), clusteredPoints.end(), [](Point point) {
        return point.label == 1;
    });
    std::vector<Point> filteredClusteredPoints(it, clusteredPoints.end());
    std::vector<Point> cluster;
    for (auto point : filteredClusteredPoints) {
        if (point.label == currentLabel) {
            cluster.push_back(point);
        } else {
            navigationPoints.push_back(getNavigationPointFromCluster(cluster));
            currentLabel += 1;
            cluster.clear();
            cluster.push_back(point);
        }
    }
    //get from the last cluster
    navigationPoints.push_back(getNavigationPointFromCluster(cluster));
    return navigationPoints;
}
Point Polygon::getNavigationPointFromCluster(std::vector<Point> cluster){
    double maxDistanceToPolygon = -1;
    Point bestPoint;
    for (Point clusterPoint : cluster) {
        double distanceToPolygon = Auxiliary::getDistanceToClosestSegment(clusterPoint, edges);
        if (maxDistanceToPolygon < distanceToPolygon) {
            maxDistanceToPolygon = distanceToPolygon;
            bestPoint = clusterPoint;
        }
    }
    return bestPoint;
}
std::vector<Point>
Polygon::filterPointsByVariances(std::vector<std::pair<double, std::vector<Point>>> slices, double epsilon) {
    std::vector<Point> goodPoints;
    std::vector<double> variances;
    for (auto slice : slices) {
        variances.push_back(slice.first);
    }
    auto minVariance = std::min_element(variances.begin(), variances.end());
    auto maxVariance = std::max_element(variances.begin(), variances.end());
    auto varianceDifference = *maxVariance - *minVariance;
    for (auto slice: slices) {
        double ratio = (slice.first - *minVariance) / varianceDifference;
        for (Point point : slice.second) {
            double minDistance = 10000;
            for (auto edge : edges) {
                double distance = Auxiliary::distanceBetweenPointAndSegment(point, edge);
                minDistance = distance < minDistance ? distance : minDistance;
            }
            if (minDistance > (1 - ratio) * epsilon) {
                goodPoints.push_back(point);
            }
        }
    }
    return goodPoints;
}

std::vector<std::pair<double, std::vector<Point>>> Polygon::getSlicesWithVariances(int angle) {
    auto pizzaSlices = Pizza::createPizzaSlices(polygonCenter, pointsOutsidePolygon, angle);
    std::vector<std::pair<double, std::vector<Point>>> slices;
    for (auto pizzaSlice : pizzaSlices) {
        int pizzaSliceSize = pizzaSlice.second.size();
        std::vector<Point> pizzaPoints;
        if (pizzaSliceSize > 2) {
            double sum = 0.0;
            for (auto point : pizzaSlice.second) {
                sum += point.second;
                pizzaPoints.push_back(point.first);
            }
            double mean = sum / pizzaSliceSize;
            double variance = 0.0;
            for (auto point : pizzaSlice.second) {
                variance += pow((point.second - mean), 2);
            }
            slices.push_back({variance / pizzaSliceSize, pizzaPoints});
        }
    }
    return slices;
}

void Polygon::createPointsWithDistance() {
    pointsWithDistance = std::vector<std::pair<Point, double>>{};
    for (Point point : points) {
        pointsWithDistance.push_back({point, Auxiliary::calculateDistanceXY(polygonCenter, point)});
    }
}

void Polygon::filterPointsInsidePolygon() {
    pointsOutsidePolygon = std::vector<std::pair<Point, double>>{};
    int verticesAmount = vertices.size();
    for (auto point : pointsWithDistance) {
        int amountOfCrossing = 0;
        for (int i = 0; i < verticesAmount; i++) {
            Point currentVertex = vertices[i];
            Point nextVertex = vertices[(i + 1) % verticesAmount];
            if ((currentVertex.x < point.first.x && point.first.x < nextVertex.x) ||
                (currentVertex.x > point.first.x && point.first.x > nextVertex.x)) {
                double ratio = (point.first.x - nextVertex.x) / (currentVertex.x - nextVertex.x);
                amountOfCrossing += ((ratio * currentVertex.y) + ((1 - ratio) * nextVertex.y)) >= point.first.y ?
                                    1 : 0;
            }
        }
        if (amountOfCrossing % 2 == 0) {
            pointsOutsidePolygon.push_back(point);
        }
    }
}

void Polygon::smoothPolygon(int angleRange) {
    vertices.push_back(vertices[0]);
    bool stop = false;
    while (!stop) {
        edges = std::vector<Line>{};
        for (int i = 1; i < vertices.size(); ++i) {
            edges.emplace_back(Line(vertices[i - 1], vertices[i]));

        }
        for (int i = 1; i < edges.size(); ++i) {
            Line fromEdge = edges[i - 1];
            double currentAngle = Auxiliary::getAngleBySlopes(fromEdge, edges[i]);
            if (!(currentAngle > angleRange && currentAngle < 180 - angleRange)) {
                for (auto verticesIt = vertices.begin(); verticesIt < vertices.end(); verticesIt++) {
                    if (*verticesIt == fromEdge.getPoint2()) {
                        vertices.erase(verticesIt);
                        break;
                    }
                }
                break;
            }
            if (i == edges.size() - 1) {
                stop = true;
            }
        }
    }
}

std::vector<std::pair<Point, double>> Polygon::getRawPolygonCorners() {
    angle = 10;  // TODO : smart choice - due to points amount, room / open space?
    std::vector<Line> lines = Pizza::createPizzaLines(polygonCenter, angle);
    auto slices = Pizza::createPizzaSlices(polygonCenter, pointsWithDistance, angle);
    std::vector<std::pair<Point, double>> polygonVertices;
    auto sortRule = [](std::pair<Point, double> point1, std::pair<Point, double> point2) -> bool {
        return point2.second < point1.second;
    };
    for (auto slice : slices) {
        std::sort(slice.second.begin(), slice.second.end(), sortRule);
        std::pair<Point, double> medianPoint = slice.second[slice.second.size() * 0.5];
        polygonVertices.push_back(medianPoint);
    }
    return polygonVertices;
}

//ronaliza
std::unordered_map<int, std::vector<std::pair<Point, double>>> Polygon:: Ronaliza_pizza(const Point& middle, const std::vector<std::pair<Point, double>>& pwd, int angle) {
    std::unordered_map<int, std::vector<std::pair<Point, double>>> slices;
    double pi = 3.14159;
    for (const auto& p : pwd) {
        int degree = int((180 / pi)*atan2(p.first.y - middle.y, p.first.x - middle.x) + 360) % 360;
        int slicekey = int(degree / angle);
        auto slice = slices.find(slicekey);
        if (slice == slices.end()) {
            slices.insert({ slicekey, std::vector<std::pair<Point, double>>{} });
        }
        slices.at(slicekey).push_back(p);
    }
    return slices;
}
std::vector<std::pair<int,std::pair<double,double>>> Polygon:: getSlicesPointsMeanAndVAR(const std::vector<std::pair<int, std::vector<std::pair<Point, double>>>>& vectorSlices) {
    std::vector<std::pair<int, std::pair<double, double>>>statistic;
    double sum;
    double Var;
    int count;
    for (auto& slice : vectorSlices)
    {
        double x = slice.second.back().first.x;
        double y = slice.second.back().first.y;
        sum = 0;
        count = 0;
        Var = 0;
        for (auto& point : slice.second)
        {
            sum = sum + point.second;
            count = count + 1;
        }
        double slice_avg = double(sum / count);
        for (auto& point : slice.second)
        {
            Var = Var + pow(point.second-slice_avg,2);
        }
        std::pair<double, double> result{slice_avg,double(sqrt(Var/(count-1.5)))};
        std::pair<int, std::pair<double, double>>ans(slice.first, result);
        statistic.emplace_back(ans);
    }
    return statistic;
}
bool Polygon:: comparisonRule(std::pair<int, std::pair<double, double>> x, std::pair<int, std::pair<double, double>> y)
{
    return x.second.second < y.second.second;
}
bool Polygon:: comparisonRule_sliceNUM(std::pair<int, std::pair<double, double>> x, std::pair<int, std::pair<double, double>> y)
{
    return x.first < y.first;
}
void Polygon:: High_VAR_fill(std::vector<std::pair<int, std::pair<double, double>>>& VAR, std::vector<std::pair<int, double>> distance)
{
    int i = 0;
    while (i < VAR.size())
    {
        bool flag = true;
        double count = 0;
        double sum = 0;
        double total_distance = 0;
        while (flag && i < VAR.size()-1)
        {
            for (auto d : distance)
            {
                if (d.first == VAR.at(i+count).first)
                    total_distance = d.second;
            }
            if (VAR.at(i+count).first + 1 == VAR.at(i + 1+count).first)
            {
                sum = sum + total_distance;
                count++;
            }
            else if (count != 0)
            {
                sum = sum + total_distance;
                flag=false;
                for (int j = i; j <= (i + count); j++)
                {
                    VAR.at(j).second.first = sum / (count + 1);
                }
            }
            else
            {
                flag = false;
            }
            if ((i + count) == (VAR.size()-1))
            {
                for (auto d : distance)
                {
                    if (d.first == VAR.at(i+count).first)
                        total_distance = d.second;
                }
                sum = sum + total_distance;
                flag = false;
                for (int j = i; j <= (i + count); j++)
                {
                    VAR.at(j).second.first = sum / (count + 1);
                }
            }
        }
        i = i + count + 1;
    }

}
bool Polygon:: comparison_rule_slice(std::pair<int, std::vector<std::pair<Point, double>>> x, std::pair<int, std::vector<std::pair<Point, double>>> y)
{
    return x.first < y.first;
}


std::vector<Point> Polygon::getExitPointsByPolygon(bool isDebug) {
    std::vector<std::pair<double,double>> python_points;
    createPointsWithDistance();
    Point center (0,0,0);
    Point center2 (9,9,9);
    int angle_Res = 2;
    double precent = 0.05;
    std::unordered_map<int, std::vector<std::pair<Point, double>>> slices = Polygon:: Ronaliza_pizza(center, pointsWithDistance, angle_Res);
    std::vector<std::pair<int, std::vector<std::pair<Point, double>>>> vectorSlices;
    for (const auto& slice : slices) {
        if (slice.second.size()>=2)
        {
            vectorSlices.emplace_back(slice);
        }
    }
    //empty slices
    std::sort(vectorSlices.begin(), vectorSlices.end(), comparison_rule_slice);
    int i = 0;
    std::vector<std::pair<int, std::pair<double, double>>> empty_slices;
    for (const auto& slice : vectorSlices)
    {
        while (slice.first != i)
        {
            std::pair<int, std::pair<double, double>>ans(i, { 0,0 });
            empty_slices.emplace_back(ans);
            i++;
        }
        if(slice.first == i)
        {
            i++;
        }
    }
    for (int k = i; k < (360 / angle_Res); k++)
    {
        std::pair<int, std::pair<double, double>>ans(k, { 0,0 });
        empty_slices.emplace_back(ans);
    }



    //distance vector
    std::vector<std::pair<int, std::pair<double, double>>> stat = getSlicesPointsMeanAndVAR(vectorSlices);
    std::vector<std::pair<int, double>>distance;
    for (const auto& slice : vectorSlices) {
        double d = 0;
        for (const auto& p : slice.second)
        {
            if (p.second > d)
                d = p.second;
        }
        std::pair<int, double> result = { slice.first,d };
        distance.emplace_back(result);
    }

    //empty slices
    for (int j = 0; j < empty_slices.size(); j++)
    {
        double mu_previous = 0;
        double new_mu=0;
        double mu_complementary = 0;
        int angle = empty_slices.at(j).first * angle_Res;
        int matching_angle = (angle + 180 - angle_Res) % 360;
        if (matching_angle == 0)
            matching_angle = 360;
        if (j != 0)//have problem if the first slice num is 0
        {
            mu_previous = empty_slices.at(j - 1).second.first;
            mu_complementary = 0;
        }
        for (const auto& slice : stat)
        {
            if (empty_slices.at(j).first - 1 == slice.first)
                mu_previous = slice.second.first;
            if (slice.first == (matching_angle / angle_Res))
                mu_complementary = slice.second.first;
        }
        if (mu_complementary == 0)
            empty_slices.at(j).second.first = mu_previous;
        else
        {
            double ratio_factor = mu_previous / mu_complementary;
            for (const auto& slice : stat)
            {
                if (slice.first == ((((matching_angle % 360) + angle_Res) / angle_Res)%360))
                    empty_slices.at(j).second.first = slice.second.first * ratio_factor;
                if (empty_slices.at(j).first-1==slice.first)
                    new_mu=slice.second.first;
            }
            if (empty_slices.at(j).second.first == 0)
            {
                for (const auto& s : empty_slices)
                {
                    if (s.first == ((((matching_angle % 360) + angle_Res) / angle_Res) % 360))
                        empty_slices.at(j).second.first = s.second.first * ratio_factor;
                }
            }
            if (empty_slices.at(j).second.first == 0&&j!=0)
                empty_slices.at(j).second.first = empty_slices.at(j - 1).second.first;
            if (j==0)
                empty_slices.at(j).second.first= new_mu;

        }
    }





    //statistics
    std::sort(stat.begin(), stat.end(), comparisonRule);
    int begin = 0;
    int end = stat.size() - (int)(360 / angle_Res) * precent;
    std::vector<std::pair<int, std::pair<double, double>>> filtered_stat;
    std::vector<std::pair<int, std::pair<double, double>>> top_var_slices;
    for (int i = begin; i < end; i++)
    {
        filtered_stat.emplace_back(stat.at(i));
    }
    int end2 = stat.size() - (int)(360 / angle_Res) * precent;
    for (int j = end2; j < stat.size(); j++)
    {
        top_var_slices.emplace_back(stat.at(j));
    }
    std::sort(top_var_slices.begin(), top_var_slices.end(), comparisonRule_sliceNUM);
    High_VAR_fill(top_var_slices, distance);

//python vector

    for (auto x:empty_slices) {
        std::pair<double, double> ans(x.first * angle_Res + angle_Res / 2, x.second.first);
        python_points.emplace_back(ans);
    }
    for (auto x:filtered_stat) {
        std::pair<double, double> ans(x.first * angle_Res + angle_Res / 2, x.second.first);
        python_points.emplace_back(ans);
    }
    for (auto x:top_var_slices) {
        std::pair<double, double> ans(x.first * angle_Res + angle_Res / 2, x.second.first);
        python_points.emplace_back(ans);
    }

//read file

    std::ofstream my_file;
    my_file.open  ("/home/ronel/AutonomousDroneCPP_aliza_ron/Tello_code_and_data2/Tello_code_and_data/example33.txt");
    for (auto x:python_points)
    {
        my_file  << 1000*(x.second) << " " << x.first << std::endl;
    }
    my_file.close();

    std::cout<<"print from top var:"<<std::endl;
    std::cout<<"sliceNum:"<<top_var_slices.at(2).first<<std::endl;
    std::cout<<"mu:"<<top_var_slices.at(2).second.first<<std::endl;
    std::cout<<"sigma:"<<top_var_slices.at(2).second.second<<std::endl;

    std::cout<<"print from top empty:"<<std::endl;
    std::cout<<"sliceNum:"<<empty_slices.at(2).first<<std::endl;
    std::cout<<"mu:"<<empty_slices.at(2).second.first<<std::endl;
    std::cout<<"sigma:"<<empty_slices.at(2).second.second<<std::endl;
//call python

    system("cd /home/ronel/AutonomousDroneCPP_aliza_ron/Tello_code_and_data2/Tello_code_and_data ; python3 App_scan.py");
//sleep(2);

//read from file the exit points

    std::vector<Point> exit_points;
    std::ifstream myFile("/home/ronel/AutonomousDroneCPP_aliza_ron/Tello_code_and_data2/Tello_code_and_data/door_points.csv");
    std::string line;
    float angle_out;
    double magic_num = 23.9;
    double distance_out;
    while (std::getline(myFile, line)) {
        std::stringstream lineStream(line);
        Point point;
        lineStream >> angle_out;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> distance_out;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.frameId;

        point.x= ((distance_out/magic_num)*cos(angle_out*3.14159/180));
        point.y= ((distance_out/magic_num)*sin(angle_out*3.14159/180));
        exit_points.push_back(point);
    }
    for (auto x:exit_points)
    {
        std::cout  << x.x<<" "<<x.y<< std::endl;
    }



    return exit_points;
}