#include "app_config.hpp"
#include "postprocess_coordinate_transformer.hpp"

#include <cmath>

bool PostprocessCoordinateTransformer::lidarMeasurementToBodyPoint(double angleDeg, double distanceM, CartesianPoint& bodyPoint) const {
    bodyPoint = CartesianPoint{};

    if(!std::isfinite(angleDeg) || !std::isfinite(distanceM) || distanceM <= 0.0) {
        return false;
    }

    constexpr double degreesToRadians = 3.14159265358979323846 / 180.0;
    const double angleRad = angleDeg * degreesToRadians;

    bodyPoint.xM = 0.0;
    bodyPoint.yM = -distanceM * std::sin(angleRad);
    bodyPoint.zM = distanceM * std::cos(angleRad);

    const bool resultFinite = std::isfinite(bodyPoint.xM && std::isfinite(bodyPoint.yM) && std::isfinite(bodyPoint.zM));

    if(!resultFinite) {
        bodyPoint = CartesianPoint{};
        return false;
    }

    return true;
}

bool PostprocessCoordinateTransformer::rotateBodyPointToNed(const CartesianPoint& bodyPoint, const TimedAttitudeSample& attitude, CartesianPoint& nedPoint) const {
    nedPoint = CartesianPoint{};

    const bool bodyPointFinite = std::isfinite(bodyPoint.xM)
        && std::isfinite(bodyPoint.yM)
        && std::isfinite(bodyPoint.zM);

    const bool quaternionFinite = std::isfinite(attitude.q0W)
        && std::isfinite(attitude.q1X)
        && std::isfinite(attitude.q2Y)
        && std::isfinite(attitude.q3Z);

    if(!bodyPointFinite || !quaternionFinite) {
        return false;
    }

    double w = attitude.q0W;
    double x = attitude.q1X;
    double y = attitude.q2Y;
    double z = attitude.q3Z;

    const double quaternionNorm = std::sqrt(w * w + x * x + y * y + z * z);

    if(quaternionNorm <= 1.0e-12) {
        return false;
    }
    
    w /= quaternionNorm;
    x /= quaternionNorm;
    y /= quaternionNorm;
    z /= quaternionNorm;

    const double rotation00 = 1.0 - 2.0 * (y * y + z * z);
    const double rotation01 = 2.0 * (x * y - w * z);
    const double rotation02 = 2.0 * (x * z + w * y);
    const double rotation10 = 2.0 * (x * y + w * z);
    const double rotation11 = 1.0 - 2.0 * (x * x + z * z);
    const double rotation12 = 2.0 * (y * z - w * x);
    const double rotation20 = 2.0 * (x * z - w * y);
    const double rotation21 = 2.0 * (y * z + w * x);
    const double rotation22 = 1.0 - 2.0 * (x * x + y * y);

    nedPoint.xM = rotation00 * bodyPoint.xM + rotation01 * bodyPoint.yM + rotation02 * bodyPoint.zM;
    nedPoint.yM = rotation10 * bodyPoint.xM + rotation11 * bodyPoint.yM + rotation12 * bodyPoint.zM;
    nedPoint.zM = rotation20 * bodyPoint.xM + rotation21 * bodyPoint.yM + rotation22 * bodyPoint.zM;

    const bool resultFinite = std::isfinite(nedPoint.xM) && std::isfinite(nedPoint.yM) && std::isfinite(nedPoint.zM);

    if(!resultFinite) {
        nedPoint = CartesianPoint{};
        return false;
    }

    return true;
}

bool PostprocessCoordinateTransformer::geodeticPositionToLocalNed(const TimedPositionSample& referencePosition, const TimedPositionSample& currentPosition, CartesianPoint& localNedPosition) const {
    localNedPosition = CartesianPoint{};

    const bool referenceFinite = std::isfinite(referencePosition.latitudeDeg)
        && std::isfinite(referencePosition.longitudeDeg)
        && std::isfinite(referencePosition.altitudeM);

    const bool currentFinite = std::isfinite(currentPosition.latitudeDeg)
        && std::isfinite(currentPosition.longitudeDeg)
        && std::isfinite(currentPosition.altitudeM);

    if(!referenceFinite || !currentFinite) {
        return false;
    }

    const bool referenceRangeValid = referencePosition.latitudeDeg >= -90.0
        && referencePosition.latitudeDeg <= 90.0
        && referencePosition.longitudeDeg >= -180.0
        && referencePosition.longitudeDeg <= 180.0;

    const bool currentRangeValid = currentPosition.latitudeDeg >= -90.0
        && currentPosition.latitudeDeg <= 90.0
        && currentPosition.longitudeDeg >= -180.0
        && currentPosition.longitudeDeg <= 180.0;

    if(!referenceRangeValid || !currentRangeValid) {
        return false;
    }

    constexpr double degreesToRadians = 3.14159265358979323846 / 180.0;
    constexpr double semiMajorAxisM = 6378137.0;
    constexpr double inverseFlattening = 298.257223563;
    constexpr double flattening = 1.0 / inverseFlattening;
    constexpr double eccentricitySquared = flattening * (2.0 - flattening);

    const auto geodeticToEcef = [degreesToRadians, semiMajorAxisM, eccentricitySquared](
        const TimedPositionSample& position,
        CartesianPoint& ecefPoint
    ) {
        const double latitudeRad = position.latitudeDeg * degreesToRadians;
        const double longitudeRad = position.longitudeDeg * degreesToRadians;
        const double sinLatitude = std::sin(latitudeRad);
        const double cosLatitude = std::cos(latitudeRad);
        const double sinLongitude = std::sin(longitudeRad);
        const double cosLongitude = std::cos(longitudeRad);
        const double primeVerticalRadiusM = semiMajorAxisM / std::sqrt(1.0 - eccentricitySquared * sinLatitude * sinLatitude);

        ecefPoint.xM = (primeVerticalRadiusM + position.altitudeM) * cosLatitude * cosLongitude;
        ecefPoint.yM = (primeVerticalRadiusM + position.altitudeM) * cosLatitude * sinLongitude;
        ecefPoint.zM = (primeVerticalRadiusM * (1.0 - eccentricitySquared) + position.altitudeM) * sinLatitude;
    };

    CartesianPoint referenceEcef{};
    CartesianPoint currentEcef{};

    geodeticToEcef(referencePosition, referenceEcef);
    geodeticToEcef(currentPosition, currentEcef);

    const double deltaX = currentEcef.xM - referenceEcef.xM;
    const double deltaY = currentEcef.yM - referenceEcef.yM;
    const double deltaZ = currentEcef.zM - referenceEcef.zM;
    const double referenceLatitudeRad = referencePosition.latitudeDeg * degreesToRadians;
    const double referenceLongitudeRad = referencePosition.longitudeDeg * degreesToRadians;
    const double sinLatitude = std::sin(referenceLatitudeRad);
    const double cosLatitude = std::cos(referenceLatitudeRad);
    const double sinLongitude = std::sin(referenceLongitudeRad);
    const double cosLongitude = std::cos(referenceLongitudeRad);

    localNedPosition.xM = -sinLatitude * cosLongitude * deltaX - sinLatitude * sinLongitude * deltaY + cosLatitude * deltaZ;
    localNedPosition.yM = -sinLongitude * deltaX + cosLongitude * deltaY;
    localNedPosition.zM = -cosLatitude * cosLongitude * deltaX - cosLatitude * sinLongitude * deltaY - sinLatitude * deltaZ;

    const bool resultFinite = std::isfinite(localNedPosition.xM) && std::isfinite(localNedPosition.yM) && std::isfinite(localNedPosition.zM);

    if(!resultFinite) {
        localNedPosition = CartesianPoint{};
        return false;
    }

    return true;
}

bool PostprocessCoordinateTransformer::createLocalNedLidarPoint(
    const TimedLidarSample& lidarSample, 
    const InterpolatedPose& pose,
    const TimedPositionSample& referencePosition,
    CartesianPoint& localNedPoint
) const {
    localNedPoint = CartesianPoint{};

    if(!lidarSample.measurementValid || !pose.positionValid || !pose.attitudeValid) {
        return false;
    }

    CartesianPoint lidarPointBody{};

    if(!lidarMeasurementToBodyPoint(lidarSample.angleDeg, lidarSample.distanceM, lidarPointBody)) {
        return false;
    }

    lidarPointBody.xM += lidarOffsetForwardM_;
    lidarPointBody.yM += lidarOffsetRightM_;
    lidarPointBody.zM += lidarOffsetDownM_;

    TimedAttitudeSample attitude{};

    attitude.timestampUs = pose.timestampUs;
    attitude.q0W = pose.q0W;
    attitude.q1X = pose.q1X;
    attitude.q2Y = pose.q2Y;
    attitude.q3Z = pose.q3Z;

    CartesianPoint lidarPointNed{};

    if(!rotateBodyPointToNed(lidarPointBody, attitude, lidarPointNed)) {
        return false;
    }

    TimedPositionSample currentPosition{};

    currentPosition.timestampUs = pose.timestampUs;
    currentPosition.latitudeDeg = pose.latitudeDeg;
    currentPosition.longitudeDeg = pose.longitudeDeg;
    currentPosition.altitudeM = pose.altitudeM;
    currentPosition.source = pose.positionSource;

    CartesianPoint aircraftPositionNed{};

    if(!geodeticPositionToLocalNed(referencePosition, currentPosition, aircraftPositionNed)) {
        return false;
    }

    localNedPoint.xM = aircraftPositionNed.xM + lidarPointNed.xM;
    localNedPoint.yM = aircraftPositionNed.yM + lidarPointNed.yM;
    localNedPoint.zM = aircraftPositionNed.zM + lidarPointNed.zM;

    const bool resultFinite = std::isfinite(localNedPoint.xM)
        && std::isfinite(localNedPoint.yM)
        && std::isfinite(localNedPoint.zM);

    if(!resultFinite) {
        localNedPoint = CartesianPoint{};
        
        return false;
    }

    return true;
}

bool PostprocessCoordinateTransformer::localNedToGeodeticPosition(
    const TimedPositionSample& referencePosition,
    const CartesianPoint& localNedPosition,
    TimedPositionSample& geodeticPosition
) const {
    geodeticPosition = TimedPositionSample{};

    const bool referenceFinite =
        std::isfinite(referencePosition.latitudeDeg)
        && std::isfinite(referencePosition.longitudeDeg)
        && std::isfinite(referencePosition.altitudeM);

    const bool localPointFinite =
        std::isfinite(localNedPosition.xM)
        && std::isfinite(localNedPosition.yM)
        && std::isfinite(localNedPosition.zM);

    if(!referenceFinite || !localPointFinite) {
        return false;
    }

    const bool referenceRangeValid =
        referencePosition.latitudeDeg >= -90.0
        && referencePosition.latitudeDeg <= 90.0
        && referencePosition.longitudeDeg >= -180.0
        && referencePosition.longitudeDeg <= 180.0;

    if(!referenceRangeValid) {
        return false;
    }

    constexpr double pi = 3.14159265358979323846;
    constexpr double degreesToRadians = pi / 180.0;
    constexpr double radiansToDegrees = 180.0 / pi;

    constexpr double semiMajorAxisM = 6378137.0;
    constexpr double inverseFlattening = 298.257223563;
    constexpr double flattening = 1.0 / inverseFlattening;
    constexpr double semiMinorAxisM =
        semiMajorAxisM * (1.0 - flattening);

    constexpr double eccentricitySquared =
        flattening * (2.0 - flattening);

    constexpr double secondEccentricitySquared =
        (
            semiMajorAxisM * semiMajorAxisM
            - semiMinorAxisM * semiMinorAxisM
        )
        / (semiMinorAxisM * semiMinorAxisM);

    const double referenceLatitudeRad =
        referencePosition.latitudeDeg * degreesToRadians;

    const double referenceLongitudeRad =
        referencePosition.longitudeDeg * degreesToRadians;

    const double sinLatitude =
        std::sin(referenceLatitudeRad);

    const double cosLatitude =
        std::cos(referenceLatitudeRad);

    const double sinLongitude =
        std::sin(referenceLongitudeRad);

    const double cosLongitude =
        std::cos(referenceLongitudeRad);

    const double primeVerticalRadiusM =
        semiMajorAxisM
        / std::sqrt(
            1.0
            - eccentricitySquared
                * sinLatitude
                * sinLatitude
        );

    const double referenceEcefX =
        (
            primeVerticalRadiusM
            + referencePosition.altitudeM
        )
        * cosLatitude
        * cosLongitude;

    const double referenceEcefY =
        (
            primeVerticalRadiusM
            + referencePosition.altitudeM
        )
        * cosLatitude
        * sinLongitude;

    const double referenceEcefZ =
        (
            primeVerticalRadiusM
                * (1.0 - eccentricitySquared)
            + referencePosition.altitudeM
        )
        * sinLatitude;

    const double northM = localNedPosition.xM;
    const double eastM = localNedPosition.yM;
    const double downM = localNedPosition.zM;

    const double deltaEcefX =
        -sinLatitude * cosLongitude * northM
        - sinLongitude * eastM
        - cosLatitude * cosLongitude * downM;

    const double deltaEcefY =
        -sinLatitude * sinLongitude * northM
        + cosLongitude * eastM
        - cosLatitude * sinLongitude * downM;

    const double deltaEcefZ =
        cosLatitude * northM
        - sinLatitude * downM;

    const double ecefX =
        referenceEcefX + deltaEcefX;

    const double ecefY =
        referenceEcefY + deltaEcefY;

    const double ecefZ =
        referenceEcefZ + deltaEcefZ;

    const double horizontalDistanceM =
        std::hypot(ecefX, ecefY);

    if(horizontalDistanceM <= 1.0e-9) {
        return false;
    }

    const double longitudeRad =
        std::atan2(ecefY, ecefX);

    const double auxiliaryAngle =
        std::atan2(
            ecefZ * semiMajorAxisM,
            horizontalDistanceM * semiMinorAxisM
        );

    const double sinAuxiliary =
        std::sin(auxiliaryAngle);

    const double cosAuxiliary =
        std::cos(auxiliaryAngle);

    const double latitudeRad =
        std::atan2(
            ecefZ
                + secondEccentricitySquared
                    * semiMinorAxisM
                    * sinAuxiliary
                    * sinAuxiliary
                    * sinAuxiliary,
            horizontalDistanceM
                - eccentricitySquared
                    * semiMajorAxisM
                    * cosAuxiliary
                    * cosAuxiliary
                    * cosAuxiliary
        );

    const double sinResultLatitude =
        std::sin(latitudeRad);

    const double resultPrimeVerticalRadiusM =
        semiMajorAxisM
        / std::sqrt(
            1.0
            - eccentricitySquared
                * sinResultLatitude
                * sinResultLatitude
        );

    const double altitudeM =
        horizontalDistanceM / std::cos(latitudeRad)
        - resultPrimeVerticalRadiusM;

    geodeticPosition.timestampUs =
        referencePosition.timestampUs;

    geodeticPosition.latitudeDeg =
        latitudeRad * radiansToDegrees;

    geodeticPosition.longitudeDeg =
        longitudeRad * radiansToDegrees;

    geodeticPosition.altitudeM =
        altitudeM;

    geodeticPosition.source =
        referencePosition.source;

    const bool resultFinite =
        std::isfinite(geodeticPosition.latitudeDeg)
        && std::isfinite(geodeticPosition.longitudeDeg)
        && std::isfinite(geodeticPosition.altitudeM);

    if(!resultFinite) {
        geodeticPosition = TimedPositionSample{};

        return false;
    }

    const bool resultRangeValid =
        geodeticPosition.latitudeDeg >= -90.0
        && geodeticPosition.latitudeDeg <= 90.0
        && geodeticPosition.longitudeDeg >= -180.0
        && geodeticPosition.longitudeDeg <= 180.0;

    if(!resultRangeValid) {
        geodeticPosition = TimedPositionSample{};

        return false;
    }

    return true;
}

bool PostprocessCoordinateTransformer::configureLidarOffset(double forwardM, double rightM, double downM) {
    const bool offsetsFinite = std::isfinite(forwardM) && std::isfinite(rightM) && std::isfinite(downM);

    if(!offsetsFinite) {
        return false;
    }

    if(std::abs(forwardM) > 10.0 || std::abs(rightM) > 10.0 || std::abs(downM) > 10.0) {
        return false;
    }

    lidarOffsetForwardM_ = forwardM;
    lidarOffsetRightM_ = rightM;
    lidarOffsetDownM_ = downM;

    return true;
}