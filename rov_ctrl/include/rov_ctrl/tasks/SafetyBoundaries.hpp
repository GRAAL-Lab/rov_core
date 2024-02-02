#ifndef __CONTROLSAFETYBOUNDARIES_H__
#define __CONTROLSAFETYBOUNDARIES_H__

#include "tpik/TPIKDefines.h"
#include "rov_ctrl/ctrl_data_structs.hpp"
#include "ulisse_msgs/msg/coordinate_list.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <tpik/TPIKlib.h>

using namespace ctb;

namespace ikcl {

typedef boost::geometry::model::d2::point_xy<double> point_t;
typedef boost::geometry::model::polygon<point_t> polygon_t;
typedef boost::geometry::model::segment<point_t> segment_t;

void MakeSegments(point_t const& p, point_t const& next, segment_t& seg);

bool IsConvex(const std::list<segment_t>& segments, const polygon_t& poly);

void ComputeAlignVectorConcave(const segment_t& segment, const point_t& currentPosition,
    const polygon_t& poly, double& distance, Eigen::Vector3d& UTM_alignVector);

void ComputeAlignVectorConvex(const std::list<segment_t>& segment, const point_t& currentPosition,
    const polygon_t& poly, const tpik::BellShapedParameter& bellParam, double& distance, Eigen::Vector3d& UTM_alignVector);

void ComputeNormalVector2Segment(const segment_t& segment, const polygon_t& poly,
    point_t& alignVector);

void ComputeNormalVector2Segment(const segment_t& segment, const polygon_t& poly,
    point_t& alignVector, point_t& u);

bool ComputeIntersectionPointMiddleZone(const std::list<segment_t>& segments, const polygon_t& poly,
    const tpik::BellShapedParameter& bellParam, std::list<point_t>& points);

void ExtractMinDistanceSegments(std::list<segment_t>& segments, const point_t& currentPosition,
    std::list<segment_t>& segment);

/**
 * @brief The SafetyBoundaries class implementing the repulsion mechanic from the boundaries.
 */

class SafetyBoundaries : public tpik::ReactiveTask {
public:
    /**
   * @brief SafetyBoundaries class constructor
   * @param taskID task id
   * @param robotModel shared ptr to the rml::RobotModel
   * @param frameID id of the frame to control
   */
    SafetyBoundaries(std::string taskID, std::shared_ptr<rml::RobotModel> robotModel, std::string frameID);

    /**
   * @brief Method updating the task Jacobian, reference, internal activation
   * function. Implementation of the pure virtual method of the base class
   * tpik::InequalityTask.
   * @note An exception is throw if deltaJL has not been initialized yet.
   */
    void Update() noexcept(false) override;

    bool InitializePolygon(const ulisse_msgs::msg::CoordinateList &boundaries);

    /*
     * Method that gets the centroid
     */
    auto Centroid() const -> const ctb::LatLong& { return centroid_; }

    /*
     * Method that sets the centroid
     */
    auto Centroid() -> ctb::LatLong& { return centroid_; }

    /**
   * @brief Overloading of the cout operator
   */
    friend std::ostream& operator<<(std::ostream& os, SafetyBoundaries const& safetyBoundaries)
    {
        os << "\033[1;37m"
           << "SAFETY BOUNDARIES " << (tpik::ReactiveTask&)safetyBoundaries
           << std::setprecision(4);
        os << "\033[1;37m"
           << "\033[1;37m"
           << "frameID \n"
           << "\033[0m" << safetyBoundaries.frameID_ << "\n";
        return os;
    }

    auto VehiclePosition() -> LatLong& { return vehiclePositionLatLong_; }

    Eigen::Vector3d GetAlignVector(const std::string& frameID);

    bool ConfigFromFile(libconfig::Config& confObj) override;

protected:
    /**
   * @brief Method updating the internal activation function.
   * Implementation of the pure virtual method of the base class
   * tpik::InequalityTask.
   */
    //void UpdateInternalActivationFunction() override;
    /**
   * @brief Method updating the Jacobian.
   * Implementation of the pure virtual method of the base class
   * tpik::InequalityTask.
   */
    void UpdateJacobian() override;

    void EvaluateAlignmentAndDistance();

    std::shared_ptr<rml::RobotModel> robotModel_; //!< The shared ptr to the robot model
    std::string frameID_; //!< The id of the frame to be controlled
    bool boundariesInitialized_; //!< Boolean used to state whehther deltaJL has been setted
    std::list<segment_t> segments_;
    ctb::LatLong centroid_;
    polygon_t poly_;
    Eigen::Vector3d bodyF_alignVector_;
    Eigen::Vector3d vehiclePosition_;
    LatLong vehiclePositionLatLong_;
    double d_;
};
} // namespace ikcl
#endif
