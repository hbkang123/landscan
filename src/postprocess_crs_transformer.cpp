#include "postprocess_crs_transformer.hpp"

#include <cmath>
#include <string>

PostprocessCrsTransformer::PostprocessCrsTransformer(int sourceEpsgCode, int targetEpsgCode) {
    if(sourceEpsgCode <= 0 || targetEpsgCode <= 0) {
        return;
    }

    context_ = proj_context_create();

    if(context_ == nullptr) {
        return;
    }

    const std::string sourceCrs = "EPSG:" + std::to_string(sourceEpsgCode);
    const std::string targetCrs = "EPSG:" + std::to_string(targetEpsgCode);

    PJ* originalTransformation = proj_create_crs_to_crs(context_, sourceCrs.c_str(), targetCrs.c_str(), nullptr);

    if(originalTransformation == nullptr) {
        return;
    }

    transformation_ = proj_normalize_for_visualization(context_, originalTransformation);

    proj_destroy(originalTransformation);
}

PostprocessCrsTransformer::~PostprocessCrsTransformer()
{
    if(transformation_ != nullptr) {
        proj_destroy(transformation_);
        transformation_ = nullptr;
    }

    if(context_ != nullptr) {
        proj_context_destroy(context_);
        context_ = nullptr;
    }
}

bool PostprocessCrsTransformer::isValid() const 
{
    return context_ != nullptr && transformation_ != nullptr;
}

bool PostprocessCrsTransformer::transform(double longitudeDeg, double latitudeDeg, double& projectedXM, double& projectedYM) const {
    projectedXM = 0.0;
    projectedYM = 0.0;

    if(!isValid()) {
        return false;
    }

    if(!std::isfinite(longitudeDeg) || !std::isfinite(latitudeDeg)) {
        return false;
    }

    if(longitudeDeg < -180.0 || longitudeDeg > 180.0 || latitudeDeg < -90.0 || latitudeDeg > 90.0) {
        return false;
    }

    proj_errno_reset(transformation_);

    const PJ_COORD inputCoordinate = proj_coord(longitudeDeg, latitudeDeg, 0.0, 0.0);
    const PJ_COORD outputCoordinate = proj_trans(transformation_, PJ_FWD, inputCoordinate);

    if(proj_errno(transformation_) != 0) {
        return false;
    }

    projectedXM = outputCoordinate.xy.x;
    projectedYM = outputCoordinate.xy.y;

    if(!std::isfinite(projectedXM) || !std::isfinite(projectedYM)) {
        projectedXM = 0.0;
        projectedYM = 0.0;
        
        return false;
    }

    return true;
}