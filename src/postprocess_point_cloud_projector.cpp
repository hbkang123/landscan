#include "postprocess_point_cloud_projector.hpp"

#include "postprocess_coordinate_transformer.hpp"
#include "postprocess_crs_transformer.hpp"

bool PostprocessPointCloudProjector::projectPoints(
    const std::vector<GeneratedLidarPoint>& localPoints,
    const TimedPositionSample& referencePosition,
    int sourceEpsgCode,
    int targetEpsgCode,
    std::vector<ProjectedLidarPoint>& projectedPoints
) const {
    projectedPoints.clear();

    if(localPoints.empty()) {
        return false;
    }

    PostprocessCoordinateTransformer coordinateTransformer;

    PostprocessCrsTransformer crsTransformer(
        sourceEpsgCode,
        targetEpsgCode
    );

    if(!crsTransformer.isValid()) {
        return false;
    }

    projectedPoints.reserve(localPoints.size());

    for(const GeneratedLidarPoint& localPoint : localPoints) {
        CartesianPoint localNedPosition{};

        localNedPosition.xM = localPoint.northM;
        localNedPosition.yM = localPoint.eastM;
        localNedPosition.zM = localPoint.downM;

        TimedPositionSample geodeticPosition{};

        const bool convertedToGeodetic =
            coordinateTransformer.localNedToGeodeticPosition(
                referencePosition,
                localNedPosition,
                geodeticPosition
            );

        if(!convertedToGeodetic) {
            projectedPoints.clear();

            return false;
        }

        double projectedXM = 0.0;
        double projectedYM = 0.0;

        const bool projected = crsTransformer.transform(
            geodeticPosition.longitudeDeg,
            geodeticPosition.latitudeDeg,
            projectedXM,
            projectedYM
        );

        if(!projected) {
            projectedPoints.clear();

            return false;
        }

        ProjectedLidarPoint projectedPoint{};

        projectedPoint.timestampUs =
            localPoint.timestampUs;

        projectedPoint.sequenceNumber =
            localPoint.sequenceNumber;

        projectedPoint.xM =
            projectedXM;

        projectedPoint.yM =
            projectedYM;

        projectedPoint.elevationM =
            geodeticPosition.altitudeM;

        projectedPoint.angleDeg =
            localPoint.angleDeg;

        projectedPoint.distanceM =
            localPoint.distanceM;

        projectedPoint.signalStrength =
            localPoint.signalStrength;

        projectedPoint.rtkPositionInfo =
            localPoint.rtkPositionInfo;

        projectedPoints.push_back(projectedPoint);
    }

    return projectedPoints.size() == localPoints.size();
}