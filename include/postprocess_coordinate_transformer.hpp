#pragma once

#include "app_config.hpp"
#include "postprocess_types.hpp"

class PostprocessCoordinateTransformer
{
public:
    bool configureLidarOffset(double forwardM, double rightM, double downM);
    bool lidarMeasurementToBodyPoint(double angleDeg, double distanceM, CartesianPoint& bodyPoint) const;
    bool rotateBodyPointToNed(const CartesianPoint& bodyPoint, const TimedAttitudeSample& attitude, CartesianPoint& nedPoint) const;
    bool geodeticPositionToLocalNed(
        const TimedPositionSample& 
        referencePosition, 
        const TimedPositionSample& currentPosition, 
        CartesianPoint& localNedPosition
    ) const;
    bool createLocalNedLidarPoint(
        const TimedLidarSample& lidarSample, 
        const InterpolatedPose& pose, 
        const TimedPositionSample& referencePosition, 
        CartesianPoint& localNedPoint
    ) const;
    bool localNedToGeodeticPosition(
        const TimedPositionSample& referencePosition,
        const CartesianPoint& localNedPosition,
        TimedPositionSample& geodeticPosition
    ) const;
private:
    double lidarOffsetForwardM_ = AppConfig::LIDAR_OFFSET_FORWARD_M;
    double lidarOffsetRightM_ = AppConfig::LIDAR_OFFSET_RIGHT_M;
    double lidarOffsetDownM_ = AppConfig::LIDAR_OFFSET_DOWN_M;
};