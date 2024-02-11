#include "rov_ctrl/tasks/SafetyBoundaries.hpp"

using namespace ctb;
namespace ikcl {

SafetyBoundaries::SafetyBoundaries(std::string taskID, std::shared_ptr<rml::RobotModel> robotModel, std::string frameID)
    : tpik::ReactiveTask(taskID, 1, robotModel->Dof(), tpik::TaskOption::Default)
    , robotModel_(robotModel)
    , frameID_(frameID)

{
    boundariesInitialized_ = false;
    bodyF_alignVector_ = Eigen::Vector3d::Zero();
    x_.resize(1,1);
}

void SafetyBoundaries::Update() noexcept(false)
{
    CheckInitialization();
    if (!boundariesInitialized_) {
        tpik::NotInitialziedTaskParameterException jointsLimitException;
        std::string how = "[SafetyBoundaries] The boundaries have not been set, use SetBoundaries() for task " + ID_;
        jointsLimitException.SetHow(how);
        throw(jointsLimitException);
    }

    // From LatLong to Euclidian
    ctb::LatLong2LocalUTM(vehiclePositionLatLong_, 0.0, centroid_, vehiclePosition_);

    EvaluateAlignmentAndDistance();
    x_.at(0) = d_;

    ReactiveTask::Update();
}

void SafetyBoundaries::UpdateJacobian()
{
    J_ = bodyF_alignVector_.transpose() * robotModel_->CartesianJacobian(frameID_).block(0, 0, 3, dof_);
    ReactiveTask::UpdateJacobian();
}

bool SafetyBoundaries::ConfigFromFile(libconfig::Config& confObj)
{

    if (!ReactiveTask::ConfigFromFile(confObj)){
        std::cerr << "[SafetyBoundaries::ConfigFromFile] Failed ConfigFromFile()" << std::endl;
        return false;
    }
    enabled_ = taskParameter_.taskEnable;
    std::cout << "== SAFETY BOUNDARIES" << std::endl;
    //std::cout << "== decreasingBellShapeParameter_.xmin = " << decreasingBellShapeParameter_.xmin << std::endl;
    //std::cout << "== decreasingBellShapeParameter_.xmax = " << decreasingBellShapeParameter_.xmax << std::endl;
    std::cout << "Enable:" << taskParameter_.taskEnable << std::endl;

    std::cout << "ROV_SB Centroid: " << centroid_.latitude << ", " << centroid_.longitude << std::endl;

    return true;
}

bool SafetyBoundaries::InitializePolygon(const ulisse_msgs::msg::CoordinateList& boundaries)
{
    segments_.clear();
    segment_t seg;
    std::string polygon = "polygon((";

    LatLong coordinateGeo;
    Eigen::Vector3d coordinateUTM;

    try {
        bool first = true;
        for (auto coordinate : boundaries.coordinates) {
            if (first) {
                first = false;
            } else {
                polygon = polygon.append(", ");
            }

            coordinateGeo.latitude = coordinate.latitude;
            coordinateGeo.longitude = coordinate.longitude;

            LatLong2LocalUTM(coordinateGeo, 0.0, centroid_, coordinateUTM);

            polygon = polygon + boost::lexical_cast<std::string>(coordinateUTM[0]) + " " + boost::lexical_cast<std::string>(coordinateUTM[1]);
        }
    } catch (std::exception& e) {
        // output exception information
        std::cout << "Initializate Polygon fails: " << e.what() << std::endl;
    }

    polygon = polygon.append("))");
    boost::geometry::read_wkt(polygon, poly_);

    // Generate a list with all the segments
    double x, y;

    for (auto it = boost::begin(boost::geometry::exterior_ring(poly_));
         (it + 1) != boost::end(boost::geometry::exterior_ring(poly_)); ++it) {

        x = (*it).x();
        y = (*it).y();

        point_t p(x, y);
        x = (*(it + 1)).x();
        y = (*(it + 1)).y();

        point_t next(x, y);
        MakeSegments(p, next, seg);

        segments_.push_front(seg);
    }

    boundariesInitialized_ = true;

    return true;
}

Eigen::Vector3d SafetyBoundaries::GetAlignVector(const std::string &frameID)
{
    Eigen::Vector3d frameID_alignVector =
        robotModel_->TransformationMatrix(frameID, robotModel_->BodyFrameID()).RotationMatrix() * bodyF_alignVector_;

    return frameID_alignVector;
}

void SafetyBoundaries::EvaluateAlignmentAndDistance()
{
    Eigen::Vector3d UTM_alignVector = Eigen::Vector3d::Zero();
    Eigen::Vector3d worldF_alignVector = Eigen::Vector3d::Zero();

    std::list<segment_t> segments, minDistsegments;

    bool isConvex = false;
    point_t currentPosition(vehiclePosition_[0], vehiclePosition_[1]);

    // copy the list of all the segments of the polygon
    segments = segments_;

    // Take the two nearest segments
    ExtractMinDistanceSegments(segments, currentPosition, minDistsegments);

    // Check if the two nearest segments are a convex side of the polygon or not
    isConvex = IsConvex(minDistsegments, poly_);

    // Depending on the type of the two nearest segments, the strategy to compute
    // the aligment vector to cameback to safty position is different.

    // Check if the robot is near the two segment convex
    if (isConvex) {
        // Compute the direct of alignent to escape from the border in case of
        // convex side of the polygon
        ComputeAlignVectorConvex(minDistsegments, currentPosition, poly_, decreasingBellShapeParameter_,  d_, UTM_alignVector);
    }
    // check if the robot is near the two segment concav
    else {
        // Compute the direct of alignent to escape from the border in case of
        // concave side of the polygon
        ComputeAlignVectorConcave(minDistsegments.front(), currentPosition, poly_, d_, UTM_alignVector);
    }

    // Transform the align vector form UTM to NED
    worldF_alignVector << UTM_alignVector.y(), UTM_alignVector.x(), 0.0;
    worldF_alignVector.normalize();

    // Project it on the body frame
    bodyF_alignVector_ = robotModel_->TransformationMatrix(robotModel_->BodyFrameID()).RotationMatrix().transpose()
        * worldF_alignVector;


    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    //std::cout << "Update(): d = " << d_ << std::endl;
    //std::cout << "Update(): UTM_alignVector = " << UTM_alignVector.transpose().format(CleanFmt) << std::endl;
    //std::cout << "Safety Task x_:" << x_.transpose().format(CleanFmt) << std::endl;

    //std::cout << "Update(): x_bar_ = " << x_bar_ << std::endl;
    //std::cout << "Update(): x_dot_bar_ = " << x_dot_bar_ << std::endl;

}

//////////////// UTILITY FUNCTIONS //////////////////////

void ComputeAlignVectorConcave(const segment_t& segment, const point_t& currentPosition,
    const polygon_t& poly, double& distance, Eigen::Vector3d& UTM_alignVector)
{
    //std::cout << "ComputeAlignVectorConcave():" << std::endl;
    // There are three situation in which the robot can be:

    // Starting point of the first segment the one at min dist
    point_t p1 { boost::geometry::get<0, 0>(segment), boost::geometry::get<0, 1>(segment) };

    // Ending point of the first segment
    point_t p2 { boost::geometry::get<1, 0>(segment), boost::geometry::get<1, 1>(segment) };

    // Compute the distaces form the starting and ending point and the distance
    // from the segment
    double d = boost::geometry::distance(currentPosition, segment);
    double dp1 = boost::geometry::distance(currentPosition, p1);
    double dp2 = boost::geometry::distance(currentPosition, p2);

    // Find the end of the segment nearest to the current position
    // if the distance form the current position to the end of the segment is
    // ugual to d then i am in the middle zone

    if (dp1 < dp2 && dp1 == d) {
        //std::cout << "if (dp1 < dp2 && dp1 == d) " << std::endl;
        double normAlignVector = std::sqrt(std::pow(currentPosition.y() - p1.y(), 2) + std::pow(currentPosition.x() - p1.x(), 2));

        UTM_alignVector(0) = (currentPosition.x() - p1.x()) / normAlignVector;
        UTM_alignVector(1) = (currentPosition.y() - p1.y()) / normAlignVector;

        // If the robot is outside the polygon the directionis the opposite
        if (!boost::geometry::covered_by(currentPosition, poly)) {
            //std::cout << "Robot is outside the polygon. d = -d" << std::endl;
            d = -d;
            UTM_alignVector(0) = -UTM_alignVector(0);
            UTM_alignVector(1) = -UTM_alignVector(1);
        }
    } else if (dp1 > dp2 && dp2 == d) {
        //std::cout << "if (dp1 > dp2 && dp2 == d) " << std::endl;
        double normAlignVector = std::sqrt(std::pow(currentPosition.y() - p2.y(), 2) + std::pow(currentPosition.x() - p2.x(), 2));

        UTM_alignVector(0) = (currentPosition.x() - p2.x()) / normAlignVector;
        UTM_alignVector(1) = (currentPosition.y() - p2.y()) / normAlignVector;

        // If the robot is outside the polygon the directionis the opposite
        if (!boost::geometry::covered_by(currentPosition, poly)) {
            //std::cout << "Robot is outside the polygon. d = -d" << std::endl;
            d = -d;
            UTM_alignVector(0) = -UTM_alignVector(0);
            UTM_alignVector(1) = -UTM_alignVector(1);
        }

    } else {
        //std::cout << "We are inside the segment: alignVector normal to the segment" << std::endl;
        // If is inside one of the two area defined by the two segment I will take
        // as alignVector the normal to the segment
        point_t u;
        ComputeNormalVector2Segment(segment, poly, u);
        UTM_alignVector(0) = u.x();
        UTM_alignVector(1) = u.y();

        if (!boost::geometry::covered_by(currentPosition, poly)) {
            //std::cout << "Robot is outside the polygon. d = -d" << std::endl;
            d = -d;
        }
    }
    distance = d;
}

void ComputeAlignVectorConvex(const std::list<segment_t>& segments, const point_t& currentPosition,
    const polygon_t& poly, const tpik::BellShapedParameter& bellParam, double& distance, Eigen::Vector3d& UTM_alignVector)
{
    //std::cout << "ComputeAlignVectorConvex():" << std::endl;

    //The variable that is continue in the convex case is the distance vector from the inner border of the safety zone
    //As in the concave case we have basically theree situations in which the robot can be:
    //- the robot is near of of the two nearest segments: only one activaction function is active
    //- the robot is the middle zone of the two segments: two activation function are active
    //- the robot is outside the segment

    //Compute the distance from the nearest segment
    double d = boost::geometry::distance(currentPosition, segments.front());

    std::list<point_t> points;

    ComputeIntersectionPointMiddleZone(segments, poly, bellParam, points);

    point_t intersecP = points.front();
    points.pop_front();

    point_t pMin = points.front();
    points.pop_front();

    point_t pMax = points.front();

    if (currentPosition.x() < pMax.x() && currentPosition.x() > pMin.x() &&
        currentPosition.y() < pMax.y() && currentPosition.y() > pMin.y()) {
        // Find the intersection point in wich the direction of the alignment must be direct in the middle zone of the two segments
        //std::cout << "Intersection in the middle zone" << std::endl;


        // Compute the align vector if the robot is in the middle zone of two segments
        // In this case the align vector is the direction vector from the current position to the inner point
        UTM_alignVector(0) = (currentPosition.x() - intersecP.x());
        UTM_alignVector(1) = (currentPosition.y() - intersecP.y());

        UTM_alignVector = UTM_alignVector.norm() * UTM_alignVector;

        // If the robot is outside the polygon the directionis the opposite
        if (!boost::geometry::covered_by(currentPosition, poly)) {
            //std::cout << "Robot is outside the polygon. d = -d" << std::endl;
            d = -d;
            UTM_alignVector(0) = -UTM_alignVector(0);
            UTM_alignVector(1) = -UTM_alignVector(1);
        }

    } else {
        // Compute the align vector if the robot is near one of the two nearest segments
        // In this case the align vector is the normal to the segment
        // std::cout << "Robot is near one of the two nearest segments" << std::endl;

        point_t u;

        ComputeNormalVector2Segment(segments.front(), poly, u);

        UTM_alignVector(0) = u.x();
        UTM_alignVector(1) = u.y();

        // If the robot is outside the polygon the directionis the opposite
        if (!boost::geometry::covered_by(currentPosition, poly)) {
            //std::cout << "Robot is outside the polygon. d = -d" << std::endl;
            d = -d;
        }
    }

    distance = d;
}

bool ComputeIntersectionPointMiddleZone(const std::list<segment_t>& segments, const polygon_t& poly,
    const tpik::BellShapedParameter& decreasingBellShapeParameter_, std::list<point_t>& points)
{
    // Starting point of the first segment
    point_t p1 { boost::geometry::get<0, 0>(segments.front()), boost::geometry::get<0, 1>(segments.front()) };
    // Ending point of the segment
    point_t p2 { boost::geometry::get<1, 0>(segments.front()), boost::geometry::get<1, 1>(segments.front()) };
    // Starting point of the second segment
    point_t s1 { boost::geometry::get<0, 0>(segments.back()), boost::geometry::get<0, 1>(segments.back()) };
    // Ending point of the second segment
    point_t s2 { boost::geometry::get<1, 0>(segments.back()), boost::geometry::get<1, 1>(segments.back()) };

    point_t frontSegDirPerp, backSegDirPerp, frontSegDir, backSegDir;

    ComputeNormalVector2Segment(std::move(segments.front()), poly, frontSegDirPerp, frontSegDir);
    ComputeNormalVector2Segment(segments.back(), poly, backSegDirPerp, backSegDir);

    point_t newP1 = { p1.x() + decreasingBellShapeParameter_.xmax(0) * frontSegDirPerp.x(), p1.y()
            + decreasingBellShapeParameter_.xmax(0) * frontSegDirPerp.y() };
    point_t newP2 = { p2.x() + decreasingBellShapeParameter_.xmax(0) * frontSegDirPerp.x(), p2.y()
            + decreasingBellShapeParameter_.xmax(0) * frontSegDirPerp.y() };
    point_t newS1 = { s1.x() + decreasingBellShapeParameter_.xmax(0) * backSegDirPerp.x(), s1.y()
            + decreasingBellShapeParameter_.xmax(0) * backSegDirPerp.y() };
    point_t newS2 = { s2.x() + decreasingBellShapeParameter_.xmax(0) * backSegDirPerp.x(), s2.y()
            + decreasingBellShapeParameter_.xmax(0) * backSegDirPerp.y() };

    std::list<segment_t> newSegments;
    MakeSegments(newP1, newP2, newSegments.front());
    MakeSegments(newS1, newS2, newSegments.back());
    std::deque<point_t> out;
    // Find the intersection point
    if (!boost::geometry::intersection(newSegments.front(), newSegments.back(), out))
        return -1;

    point_t intersecP;

    intersecP.set<0>((out.front()).x());
    intersecP.set<1>((out.front()).y());

    points.push_back(intersecP);

    // Compute the area'plane in which the alignment must be direct to the intersection point
    double k = 50.0; //m
    point_t r1 = { intersecP.x() + k * frontSegDir.x(), intersecP.y() + k * frontSegDir.y() };
    point_t r2 = { intersecP.x() + k * backSegDir.x(), intersecP.y() + k * backSegDir.y() };

    // Find the min/max x e y
    double xMin = Eigen::Vector3d { intersecP.x(), r1.x(), r2.x() }.minCoeff();
    double xMax = Eigen::Vector3d { intersecP.x(), r1.x(), r2.x() }.maxCoeff();
    double yMin = Eigen::Vector3d { intersecP.y(), r1.y(), r2.y() }.minCoeff();
    double yMax = Eigen::Vector3d { intersecP.y(), r1.y(), r2.y() }.maxCoeff();

    point_t pMin = { xMin, yMin };
    point_t pMax = { xMax, yMax };

    points.push_back(pMin);
    points.push_back(pMax);

    return 0;
}

void ComputeNormalVector2Segment(const segment_t& segment, const polygon_t& poly,
    point_t& uPerp)
{
    // Function to compute the normale vector to a segment that
    // point towards the center of the polygon

    // Starting point of the first segment
    point_t p1 { boost::geometry::get<0, 0>(segment), boost::geometry::get<0, 1>(segment) };

    // Ending point of the segment
    point_t p2 { boost::geometry::get<1, 0>(segment), boost::geometry::get<1, 1>(segment) };

    point_t u, midP;

    // Find the magnitude of the segment
    double p2p1Mag = std::sqrt(std::pow(p2.y() - p1.y(), 2) + std::pow(p2.x() - p1.x(), 2));

    // Direction of the segment
    u.set<0>(1 / p2p1Mag * (p2.x() - p1.x()));
    u.set<1>(1 / p2p1Mag * (p2.y() - p1.y()));

    // Find the medium point of the segment
    midP.set<0>((p2.x() + p1.x()) / 2);
    midP.set<1>((p2.y() + p1.y()) / 2);

    // Take the direction perpendicular to the segment
    // there are two possible direction orthogonal to the segment. Take the
    // one that point towards the polygon
    uPerp.set<0>(-u.y());
    uPerp.set<1>(u.x());

    // Compute the controid of the poly
    point_t centroid;
    boost::geometry::centroid(poly, centroid);

    point_t direction2Centr;
    // Find the magnitude of direction2Centr
    double direction2CentrMag = std::sqrt(std::pow(midP.y() - centroid.y(), 2) + std::pow(midP.x() - centroid.x(), 2));

    // Direction of the segment
    direction2Centr.set<0>(1 / direction2CentrMag * (centroid.x() - midP.x()));
    direction2Centr.set<1>(1 / direction2CentrMag * (centroid.y() - midP.y()));

    // If not take the oder direction
    if (boost::geometry::dot_product(direction2Centr, uPerp) < 0) {
        uPerp.set<0>(u.y());
        uPerp.set<1>(-u.x());
    }
}

void ComputeNormalVector2Segment(const segment_t& segment, const polygon_t& poly,
    point_t& uPerp, point_t& u)
{
    // Function to compute the normale vector to a segment that
    // point towards the center of the polygon

    // Starting point of the first segment
    point_t p1 { boost::geometry::get<0, 0>(segment), boost::geometry::get<0, 1>(segment) };

    // Ending point of the segment
    point_t p2 { boost::geometry::get<1, 0>(segment), boost::geometry::get<1, 1>(segment) };

    point_t midP;

    // Find the magnitude of the segment
    double p2p1Mag = std::sqrt(std::pow(p2.y() - p1.y(), 2) + std::pow(p2.x() - p1.x(), 2));

    // Direction of the segment
    u.set<0>(1 / p2p1Mag * (p2.x() - p1.x()));
    u.set<1>(1 / p2p1Mag * (p2.y() - p1.y()));

    // find the medium point of the segment
    midP.set<0>((p2.x() + p1.x()) / 2);
    midP.set<1>((p2.y() + p1.y()) / 2);

    // Take the direction perpendicular to the segment
    // there are two possible direction orthogonal to the segment. Take the
    // one that point towards the polygon
    uPerp.set<0>(-u.y());
    uPerp.set<1>(u.x());

    // Compute the controid of the poly
    point_t centroid;
    boost::geometry::centroid(poly, centroid);

    point_t direction2Centr;
    // Find the magnitude of direction2Centr
    double direction2CentrMag = std::sqrt(std::pow(midP.y() - centroid.y(), 2) + std::pow(midP.x() - centroid.x(), 2));

    // Direction of the segment
    direction2Centr.set<0>(1 / direction2CentrMag * (centroid.x() - midP.x()));
    direction2Centr.set<1>(1 / direction2CentrMag * (centroid.y() - midP.y()));

    // If not take the oder direction
    if (boost::geometry::dot_product(direction2Centr, uPerp) < 0) {
        uPerp.set<0>(u.y());
        uPerp.set<1>(-u.x());
    }
}

void ExtractMinDistanceSegments(std::list<segment_t>& originalSegments, const point_t& currentPosition,
    std::list<segment_t>& segments)
{

    // Function that comeback the two min distance segment

    // Current evalueting distance
    double d = static_cast<double>(INFINITY);
    // Current min distance
    double minD = static_cast<double>(INFINITY);

    // List iterators
    std::list<segment_t>::iterator it, tmp;

    it = originalSegments.begin();

    // Extract the first min distance segment
    for (std::list<segment_t>::iterator it = originalSegments.begin(); it != originalSegments.end(); it++) {

        // Distance from the current potion to the i-th segment
        d = boost::geometry::distance(currentPosition, *it);

        if (d <= minD) {
            minD = d;
            tmp = it;
        }
    }

    segments.push_back(*tmp);
    // Erase the the found segment and repeat the same procedure to find the
    // second min dist segment
    originalSegments.erase(tmp);

    d = static_cast<double>(INFINITY);
    minD = static_cast<double>(INFINITY);

    for (std::list<segment_t>::iterator it = originalSegments.begin(); it != originalSegments.end(); it++) {

        // Distance from the current potion to the i-th segment
        d = boost::geometry::distance(currentPosition, *it);
        if (d <= minD) {
            minD = d;
            tmp = it;
        }
    }

    segments.push_back(*tmp);
}

bool IsConvex(const std::list<segment_t>& segments, const polygon_t& poly)
{
    // Function to detect if the current side of the polygon is convex or not
    bool isConvex = false;
    // Direction vector of the line passing throught the two segments
    point_t midP;

    // Starting point of the first segment
    point_t p1 { boost::geometry::get<0, 0>(segments.front()), boost::geometry::get<0, 1>(segments.front()) };
    // Ending point of the first segment
    point_t p2 { boost::geometry::get<1, 0>(segments.front()), boost::geometry::get<1, 1>(segments.front()) };

    // Starting point of the second segment
    point_t s1 { boost::geometry::get<0, 0>(segments.back()), boost::geometry::get<0, 1>(segments.back()) };
    // Ending point of the second segment
    point_t s2 { boost::geometry::get<1, 0>(segments.back()), boost::geometry::get<1, 1>(segments.back()) };

    // Find the medium point of the segment
    if (p2.x() == s1.x() && p2.y() == s1.y()) {
        midP.set<0>((s2.x() + p1.x()) / 2);
        midP.set<1>((s2.y() + p1.y()) / 2);
    } else {
        midP.set<0>((s1.x() + p2.x()) / 2);
        midP.set<1>((s1.y() + p2.y()) / 2);
    }

    // Check if the medium point is inside the polygon. If it is inside then the
    // side of the polygon is convex
    if (boost::geometry::covered_by(midP, poly))
        isConvex = true;

    return isConvex;
}

void MakeSegments(point_t const& p, point_t const& next, segment_t& seg)
{
    boost::geometry::set<0, 0>(seg, p.x());
    boost::geometry::set<0, 1>(seg, p.y());
    boost::geometry::set<1, 0>(seg, next.x());
    boost::geometry::set<1, 1>(seg, next.y());
}


} // namespace ikcl
