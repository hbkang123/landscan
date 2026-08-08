#include "postprocess_point_cloud_writer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <stdint.h>

namespace {

void writeUint16LittleEndian(std::ofstream& file, uint16_t value)
{
    const char bytes[2] = {static_cast<char>(value & 0xFFU), static_cast<char>((value >> 8U) & 0xFFU)};
    file.write(bytes, sizeof(bytes));
}

void writeUint32LittleEndian(std::ofstream& file, uint32_t value)
{
    const char bytes[4] = {
        static_cast<char>(value & 0xFFU),
        static_cast<char>((value >> 8U) & 0xFFU),
        static_cast<char>((value >> 16U) & 0xFFU),
        static_cast<char>((value >> 24U) & 0xFFU)
    };

    file.write(bytes, sizeof(bytes));
}

void writeInt32LittleEndian(std::ofstream& file, int32_t value)
{
    writeUint32LittleEndian(file, static_cast<uint32_t>(value));
}

void writeDoubleLittleEndian(std::ofstream& file, double value)
{
    uint64_t rawValue = 0;

    static_assert(sizeof(rawValue) == sizeof(value), "Unexpected double size");

    std::memcpy(&rawValue, &value, sizeof(rawValue));

    const char bytes[8] = {
        static_cast<char>(rawValue & 0xFFULL),
        static_cast<char>((rawValue >> 8ULL) & 0xFFULL),
        static_cast<char>((rawValue >> 16ULL) & 0xFFULL),
        static_cast<char>((rawValue >> 24ULL) & 0xFFULL),
        static_cast<char>((rawValue >> 32ULL) & 0xFFULL),
        static_cast<char>((rawValue >> 40ULL) & 0xFFULL),
        static_cast<char>((rawValue >> 48ULL) & 0xFFULL),
        static_cast<char>((rawValue >> 56ULL) & 0xFFULL)
    };

    file.write(bytes, sizeof(bytes));
}

void writeFixedLengthString(std::ofstream& file, const std::string& value, size_t fieldLength)
{
    std::string field(fieldLength, '\0');

    const size_t copyLength = std::min(value.size(), fieldLength);

    std::copy_n(value.begin(), copyLength, field.begin());

    file.write(field.data(), static_cast<std::streamsize>(field.size()));
}

bool writeProjectedCrsVlr(std::ofstream& file, int targetEpsgCode)
{
    if(targetEpsgCode <= 0 || targetEpsgCode > 65535) {
        return false;
    }

    constexpr uint16_t recordId = 34735;
    constexpr uint16_t payloadLength = 32;
    constexpr uint16_t keyCount = 3;

    // LAS VLR header: 54 bytes
    writeUint16LittleEndian(file, 0);
    writeFixedLengthString(file, "LASF_Projection", 16);
    writeUint16LittleEndian(file, recordId);
    writeUint16LittleEndian(file, payloadLength);
    writeFixedLengthString(file, "GeoKeyDirectoryTag", 32);

    // GeoKeyDirectoryTag header
    writeUint16LittleEndian(file, 1);        // KeyDirectoryVersion
    writeUint16LittleEndian(file, 1);        // KeyRevision
    writeUint16LittleEndian(file, 0);        // MinorRevision
    writeUint16LittleEndian(file, keyCount); // NumberOfKeys

    // GTModelTypeGeoKey = Projected coordinate system
    writeUint16LittleEndian(file, 1024);
    writeUint16LittleEndian(file, 0);
    writeUint16LittleEndian(file, 1);
    writeUint16LittleEndian(file, 1);

    // GTRasterTypeGeoKey = PixelIsArea
    writeUint16LittleEndian(file, 1025);
    writeUint16LittleEndian(file, 0);
    writeUint16LittleEndian(file, 1);
    writeUint16LittleEndian(file, 1);

    // ProjectedCSTypeGeoKey = target EPSG
    writeUint16LittleEndian(file, 3072);
    writeUint16LittleEndian(file, 0);
    writeUint16LittleEndian(file, 1);
    writeUint16LittleEndian(
        file,
        static_cast<uint16_t>(targetEpsgCode)
    );

    return file.good();
}
}

bool PostprocessPointCloudWriter::writeLocalNedCsv(
    const std::filesystem::path& sessionDirectory,
    const std::vector<GeneratedLidarPoint>& points,
    const TimedPositionSample& referencePosition
) const {
    if(points.empty()) {
        return false;
    }

    if(!std::filesystem::exists(sessionDirectory) || !std::filesystem::is_directory(sessionDirectory)) {
        return false;
    }

    const std::filesystem::path pointCloudPath = sessionDirectory / "point_cloud.csv";
    const std::filesystem::path referencePath = sessionDirectory / "point_cloud_reference.csv";

    std::ofstream pointCloudFile(pointCloudPath, std::ios::out | std::ios::trunc);

    if(!pointCloudFile.is_open()) {
        return false;
    }

    pointCloudFile << std::setprecision(15);

    pointCloudFile
        << "timestamp_us,"
        << "sequence_number,"
        << "north_m,"
        << "east_m,"
        << "down_m,"
        << "x_east_m,"
        << "y_north_m,"
        << "z_up_m,"
        << "angle_deg,"
        << "distance_m,"
        << "signal_strength,"
        << "rtk_position_info"
        << '\n';

    for(const GeneratedLidarPoint& point : points) {
        const double xEastM = point.eastM;
        const double yNorthM = point.northM;
        const double zUpM = -point.downM;

        pointCloudFile
            << point.timestampUs << ','
            << point.sequenceNumber << ','
            << point.northM << ','
            << point.eastM << ','
            << point.downM << ','
            << xEastM << ','
            << yNorthM << ','
            << zUpM << ','
            << point.angleDeg << ','
            << point.distanceM << ','
            << point.signalStrength << ','
            << static_cast<unsigned int>(point.rtkPositionInfo)
            << '\n';
        
        if(!pointCloudFile.good()) {
            return false;
        }
    }

    pointCloudFile.close();

    if(!pointCloudFile) {
        return false;
    }

    std::ofstream referenceFile(referencePath, std::ios::out | std::ios::trunc);

    if(!referenceFile.is_open()) {
        return false;
    }

    referenceFile << std::setprecision(15);

    referenceFile
        << "reference_time_us,"
        << "reference_latitude_deg,"
        << "reference_longitude_deg,"
        << "reference_altitude_m,"
        << "position_source"
        << '\n';

    referenceFile
        << referencePosition.timestampUs << ','
        << referencePosition.latitudeDeg << ','
        << referencePosition.longitudeDeg << ','
        << referencePosition.altitudeM << ','
        << static_cast<unsigned int>(referencePosition.source)
        << '\n';
    
    referenceFile.close();

    if(!referenceFile) {
        return false;
    }

    return true;
}

bool PostprocessPointCloudWriter::writeLocalLas(
    const std::filesystem::path& sessionDirectory, 
    const std::vector<GeneratedLidarPoint>& points
) const {
    if(points.empty()) {
        return false;
    }

    if(!std::filesystem::exists(sessionDirectory) || !std::filesystem::is_directory(sessionDirectory)) {
        return false;
    }

    if(points.size() > std::numeric_limits<uint32_t>::max()) {
        return false;
    }

    constexpr double coordinateScale = 0.001;
    constexpr double xOffset = 0.0;
    constexpr double yOffset = 0.0;
    constexpr double zOffset = 0.0;

    constexpr uint16_t headerSize = 227;
    constexpr uint32_t pointDataOffset = 227;
    constexpr uint8_t pointDataFormat = 0;
    constexpr uint16_t pointRecordLength = 20;

    bool boundsInitialized = false;

    double minimumX = 0.0;
    double maximumX = 0.0;
    double minimumY = 0.0;
    double maximumY = 0.0;
    double minimumZ = 0.0;
    double maximumZ = 0.0;

    const auto coordinateCanBeQuantized = [coordinateScale](double coordinate, double offset) {
        if(!std::isfinite(coordinate)) {
            return false;
        }

        const double scaledCoordinate = (coordinate - offset) / coordinateScale;

        return scaledCoordinate >= static_cast<double>(std::numeric_limits<int32_t>::min())
            && scaledCoordinate <= static_cast<double>(std::numeric_limits<int32_t>::max());
    };

    for(const GeneratedLidarPoint& point : points) {
        const double x = point.eastM;
        const double y = point.northM;
        const double z = -point.downM;

        if(!coordinateCanBeQuantized(x, xOffset) || !coordinateCanBeQuantized(y, yOffset) || !coordinateCanBeQuantized(z, zOffset)) {
            return false;
        }

        if(!boundsInitialized) {
            minimumX = x;
            maximumX = x;
            minimumY = y;
            maximumY = y;
            minimumZ = z;
            maximumZ = z;

            boundsInitialized = true;
            continue;
        }

        if(x < minimumX) {
            minimumX = x;
        }

        if(x > maximumX) {
            maximumX = x;
        }

        if(y < minimumY) {
            minimumY = y;
        }

        if(y > maximumY) {
            maximumY = y;
        }

        if(z < minimumZ) {
            minimumZ = z;
        }

        if(z > maximumZ) {
            maximumZ = z;
        }
    }

    if(!boundsInitialized) {
        return false;
    }

    const std::filesystem::path lasPath = sessionDirectory / "point_cloud_local.las";

    std::ofstream file(lasPath, std::ios::out | std::ios::binary | std::ios::trunc);

    if(!file.is_open()) {
        return false;
    }

    file.write("LASF", 4);

    writeUint16LittleEndian(file, 0);
    writeUint16LittleEndian(file, 0);

    writeUint32LittleEndian(file, 0);
    writeUint16LittleEndian(file, 0);
    writeUint16LittleEndian(file, 0);
    writeFixedLengthString(file, "", 8);

    file.put(static_cast<char>(1));
    file.put(static_cast<char>(2));

    writeFixedLengthString(file, "LandScan", 32);
    writeFixedLengthString(file, "LandScan", 32);

    writeUint16LittleEndian(file, 0);
    writeUint16LittleEndian(file, 0);

    writeUint16LittleEndian(file, headerSize);
    writeUint32LittleEndian(file, pointDataOffset);
    writeUint32LittleEndian(file, 0);

    file.put(static_cast<char>(pointDataFormat));
    writeUint16LittleEndian(file, pointRecordLength);

    const uint32_t pointCount = static_cast<uint32_t>(points.size());

    writeUint32LittleEndian(file, pointCount);

    writeUint32LittleEndian(file, pointCount);
    writeUint32LittleEndian(file, 0);
    writeUint32LittleEndian(file, 0);
    writeUint32LittleEndian(file, 0);
    writeUint32LittleEndian(file, 0);

    writeDoubleLittleEndian(file, coordinateScale);
    writeDoubleLittleEndian(file, coordinateScale);
    writeDoubleLittleEndian(file, coordinateScale);

    writeDoubleLittleEndian(file, xOffset);
    writeDoubleLittleEndian(file, yOffset);
    writeDoubleLittleEndian(file, zOffset);

    writeDoubleLittleEndian(file, maximumX);
    writeDoubleLittleEndian(file, minimumX);
    writeDoubleLittleEndian(file, maximumY);
    writeDoubleLittleEndian(file, minimumY);
    writeDoubleLittleEndian(file, maximumZ);
    writeDoubleLittleEndian(file, minimumZ);

    if(!file.good()) {
        return false;
    }

    if(static_cast<std::streamoff>(file.tellp()) != static_cast<std::streamoff>(headerSize)) {
        return false;
    }

    for(const GeneratedLidarPoint& point : points) {
        const double x = point.eastM;
        const double y = point.northM;
        const double z = -point.downM;

        const int32_t rawX = static_cast<int32_t>(std::llround((x - xOffset) / coordinateScale));
        const int32_t rawY = static_cast<int32_t>(std::llround((y - yOffset) / coordinateScale));
        const int32_t rawZ = static_cast<int32_t>(std::llround((z - zOffset) / coordinateScale));

        double signalStrength = point.signalStrength;

        if(!std::isfinite(signalStrength)) {
            signalStrength = 0.0;
        }

        signalStrength = std::clamp(signalStrength, 0.0, 65535.0);

        const uint16_t intensity = static_cast<uint16_t>(std::llround(signalStrength));

        double scanAngle = point.angleDeg;

        if(!std::isfinite(scanAngle)) {
            scanAngle = 0.0;
        }

        scanAngle = std::clamp(scanAngle, -90.0, 90.0);

        const int8_t scanAngleRank = static_cast<int8_t>(std::llround(scanAngle));

        constexpr uint8_t returnInformation = 9;
        constexpr uint8_t classification = 1;

        writeInt32LittleEndian(file, rawX);
        writeInt32LittleEndian(file, rawY);
        writeInt32LittleEndian(file, rawZ);
        writeUint16LittleEndian(file, intensity);

        file.put(static_cast<char>(returnInformation));
        file.put(static_cast<char>(classification));
        file.put(static_cast<char>(scanAngleRank));
        file.put(static_cast<char>(point.rtkPositionInfo));

        writeUint16LittleEndian(file, 0);

        if(!file.good()) {
            return false;
        }
    }

    file.close();

    if(!file) {
        return false;
    }

    return true;
}

bool PostprocessPointCloudWriter::writeProjectedCsv(
    const std::filesystem::path& sessionDirectory,
    const std::vector<ProjectedLidarPoint>& points,
    int targetEpsgCode
) const {
    if(points.empty() || targetEpsgCode <= 0) {
        return false;
    }

    if(!std::filesystem::exists(sessionDirectory)
        || !std::filesystem::is_directory(sessionDirectory)) {
        return false;
    }

    const std::string fileName =
        "point_cloud_epsg"
        + std::to_string(targetEpsgCode)
        + ".csv";

    const std::filesystem::path outputPath =
        sessionDirectory / fileName;

    std::ofstream file(
        outputPath,
        std::ios::out | std::ios::trunc
    );

    if(!file.is_open()) {
        return false;
    }

    file << std::setprecision(15);

    file
        << "timestamp_us,"
        << "sequence_number,"
        << "x_m,"
        << "y_m,"
        << "elevation_m,"
        << "angle_deg,"
        << "distance_m,"
        << "signal_strength,"
        << "rtk_position_info,"
        << "epsg_code"
        << '\n';

    for(const ProjectedLidarPoint& point : points) {
        file
            << point.timestampUs << ','
            << point.sequenceNumber << ','
            << point.xM << ','
            << point.yM << ','
            << point.elevationM << ','
            << point.angleDeg << ','
            << point.distanceM << ','
            << point.signalStrength << ','
            << static_cast<unsigned int>(
                point.rtkPositionInfo
            )
            << ','
            << targetEpsgCode
            << '\n';

        if(!file.good()) {
            return false;
        }
    }

    file.close();

    if(!file) {
        return false;
    }

    return true;
}

bool PostprocessPointCloudWriter::writeProjectedLas(
    const std::filesystem::path& sessionDirectory,
    const std::vector<ProjectedLidarPoint>& points,
    int targetEpsgCode
) const {
    if(points.empty()) {
        return false;
    }

    if(targetEpsgCode <= 0 || targetEpsgCode > 65535) {
        return false;
    }

    if(points.size() > std::numeric_limits<uint32_t>::max()) {
        return false;
    }

    if(
        !std::filesystem::exists(sessionDirectory)
        || !std::filesystem::is_directory(sessionDirectory)
    ) {
        return false;
    }

    double minimumX = points.front().xM;
    double maximumX = points.front().xM;

    double minimumY = points.front().yM;
    double maximumY = points.front().yM;

    double minimumZ = points.front().elevationM;
    double maximumZ = points.front().elevationM;

    for(const ProjectedLidarPoint& point : points) {
        const bool pointFinite =
            std::isfinite(point.xM)
            && std::isfinite(point.yM)
            && std::isfinite(point.elevationM)
            && std::isfinite(point.angleDeg)
            && std::isfinite(point.signalStrength);

        if(!pointFinite) {
            return false;
        }

        minimumX = std::min(minimumX, point.xM);
        maximumX = std::max(maximumX, point.xM);

        minimumY = std::min(minimumY, point.yM);
        maximumY = std::max(maximumY, point.yM);

        minimumZ = std::min(minimumZ, point.elevationM);
        maximumZ = std::max(maximumZ, point.elevationM);
    }

    constexpr double scaleX = 0.001;
    constexpr double scaleY = 0.001;
    constexpr double scaleZ = 0.001;

    // 큰 투영좌표를 int32_t 안에 안전하게 저장하기 위한 기준 오프셋
    const double offsetX = std::floor(minimumX / 1000.0) * 1000.0;
    const double offsetY = std::floor(minimumY / 1000.0) * 1000.0;
    const double offsetZ = std::floor(minimumZ / 1000.0) * 1000.0;

    for(const ProjectedLidarPoint& point : points) {
        const double scaledX = (point.xM - offsetX) / scaleX;
        const double scaledY = (point.yM - offsetY) / scaleY;
        const double scaledZ = (point.elevationM - offsetZ) / scaleZ;

        if(
            scaledX < static_cast<double>(std::numeric_limits<int32_t>::min())
            || scaledX > static_cast<double>(std::numeric_limits<int32_t>::max())
            || scaledY < static_cast<double>(std::numeric_limits<int32_t>::min())
            || scaledY > static_cast<double>(std::numeric_limits<int32_t>::max())
            || scaledZ < static_cast<double>(std::numeric_limits<int32_t>::min())
            || scaledZ > static_cast<double>(std::numeric_limits<int32_t>::max())
        ) {
            return false;
        }
    }

    const std::filesystem::path lasPath =
        sessionDirectory
        / (
            "point_cloud_epsg"
            + std::to_string(targetEpsgCode)
            + ".las"
        );

    std::ofstream file(
        lasPath,
        std::ios::out | std::ios::binary | std::ios::trunc
    );

    if(!file.is_open()) {
        return false;
    }

    constexpr uint16_t headerSize = 227;
    constexpr uint32_t vlrHeaderSize = 54;
    constexpr uint32_t vlrPayloadSize = 32;
    constexpr uint32_t pointDataOffset =
        headerSize + vlrHeaderSize + vlrPayloadSize;

    constexpr uint8_t pointDataFormat = 0;
    constexpr uint16_t pointDataRecordLength = 20;
    constexpr uint32_t variableLengthRecordCount = 1;

    const uint32_t pointCount = static_cast<uint32_t>(points.size());

    // LAS 1.2 public header
    file.write("LASF", 4);

    writeUint16LittleEndian(file, 0); // File Source ID
    writeUint16LittleEndian(file, 0); // Global Encoding

    const char projectId[16] = {};
    file.write(projectId, sizeof(projectId));

    file.put(static_cast<char>(1)); // Version major
    file.put(static_cast<char>(2)); // Version minor

    writeFixedLengthString(file, "LandScan", 32);
    writeFixedLengthString(file, "LandScan Postprocess", 32);

    writeUint16LittleEndian(file, 0); // Creation day
    writeUint16LittleEndian(file, 0); // Creation year

    writeUint16LittleEndian(file, headerSize);
    writeUint32LittleEndian(file, pointDataOffset);
    writeUint32LittleEndian(file, variableLengthRecordCount);

    file.put(static_cast<char>(pointDataFormat));
    writeUint16LittleEndian(file, pointDataRecordLength);
    writeUint32LittleEndian(file, pointCount);

    // Number of points by return
    writeUint32LittleEndian(file, pointCount);
    writeUint32LittleEndian(file, 0);
    writeUint32LittleEndian(file, 0);
    writeUint32LittleEndian(file, 0);
    writeUint32LittleEndian(file, 0);

    writeDoubleLittleEndian(file, scaleX);
    writeDoubleLittleEndian(file, scaleY);
    writeDoubleLittleEndian(file, scaleZ);

    writeDoubleLittleEndian(file, offsetX);
    writeDoubleLittleEndian(file, offsetY);
    writeDoubleLittleEndian(file, offsetZ);

    writeDoubleLittleEndian(file, maximumX);
    writeDoubleLittleEndian(file, minimumX);
    writeDoubleLittleEndian(file, maximumY);
    writeDoubleLittleEndian(file, minimumY);
    writeDoubleLittleEndian(file, maximumZ);
    writeDoubleLittleEndian(file, minimumZ);

    if(!file.good()) {
        return false;
    }

    // EPSG 정보를 포함하는 GeoTIFF VLR
    if(!writeProjectedCrsVlr(file, targetEpsgCode)) {
        return false;
    }

    // LAS Point Data Record Format 0
    for(const ProjectedLidarPoint& point : points) {
        const int32_t integerX = static_cast<int32_t>(
            std::llround((point.xM - offsetX) / scaleX)
        );

        const int32_t integerY = static_cast<int32_t>(
            std::llround((point.yM - offsetY) / scaleY)
        );

        const int32_t integerZ = static_cast<int32_t>(
            std::llround((point.elevationM - offsetZ) / scaleZ)
        );

        const double clampedIntensity = std::clamp(
            point.signalStrength,
            0.0,
            static_cast<double>(std::numeric_limits<uint16_t>::max())
        );

        const uint16_t intensity = static_cast<uint16_t>(
            std::llround(clampedIntensity)
        );

        const double clampedScanAngle = std::clamp(
            point.angleDeg,
            -90.0,
            90.0
        );

        const int8_t scanAngleRank = static_cast<int8_t>(
            std::llround(clampedScanAngle)
        );

        writeInt32LittleEndian(file, integerX);
        writeInt32LittleEndian(file, integerY);
        writeInt32LittleEndian(file, integerZ);

        writeUint16LittleEndian(file, intensity);

        file.put(static_cast<char>(9)); // Return 1 of 1
        file.put(static_cast<char>(1)); // Classification: unclassified
        file.put(static_cast<char>(scanAngleRank));
        file.put(static_cast<char>(point.rtkPositionInfo));

        writeUint16LittleEndian(file, 0); // Point Source ID

        if(!file.good()) {
            return false;
        }
    }

    file.close();

    return static_cast<bool>(file);
}