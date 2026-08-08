#pragma once

#include <proj.h>

class PostprocessCrsTransformer
{
public:
    PostprocessCrsTransformer(int sourceEpsgCode, int targetEpsgCode);

    ~PostprocessCrsTransformer();

    PostprocessCrsTransformer(const PostprocessCrsTransformer&) = delete;
    PostprocessCrsTransformer& operator=(const PostprocessCrsTransformer&) = delete;

    bool isValid() const;

    bool transform(
        double longitudeDeg,
        double latitudeDeg,
        double& projectdXM,
        double& projectdYM
    ) const;

private:
    PJ_CONTEXT* context_ = nullptr;
    PJ* transformation_ = nullptr;
};