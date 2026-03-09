#include "parameter.hpp"

#include <fstream>
#include <stdexcept>
#include <vector>

#include <opencv2/core/persistence.hpp>

namespace fs = std::filesystem;

namespace gutcpp {

namespace {

cv::FileStorage OpenParameterStorage(const fs::path& path) {
    cv::FileStorage storage;
    if (storage.open(path.string(), cv::FileStorage::READ)) {
        return storage;
    }
    storage.release();
    if (storage.open(path.string(), cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON)) {
        return storage;
    }
    throw std::runtime_error("Failed to open structured parameter file: " + path.string());
}

cv::FileNode RequireNode(const cv::FileNode& node, const std::string& fieldName) {
    if (node.empty()) {
        throw std::runtime_error("Missing parameter field: " + fieldName);
    }
    return node;
}

std::vector<double> ReadDoubleList(const cv::FileNode& node, const std::string& fieldName) {
    std::vector<double> values;
    RequireNode(node, fieldName) >> values;
    if (values.empty()) {
        throw std::runtime_error("Failed to read numeric list: " + fieldName);
    }
    return values;
}

cv::Scalar ReadScalar3(const cv::FileNode& node, const std::string& fieldName) {
    const std::vector<double> values = ReadDoubleList(node, fieldName);
    if (values.size() != 3) {
        throw std::runtime_error("Expected 3 values for field: " + fieldName);
    }
    return cv::Scalar(values[0], values[1], values[2]);
}

double ReadDouble(const cv::FileNode& node, const std::string& fieldName) {
    double value = 0.0;
    RequireNode(node, fieldName) >> value;
    return value;
}

int ReadInt(const cv::FileNode& node, const std::string& fieldName) {
    int value = 0;
    RequireNode(node, fieldName) >> value;
    return value;
}

std::string ReadString(const cv::FileNode& node, const std::string& fieldName) {
    std::string value;
    RequireNode(node, fieldName) >> value;
    if (value.empty()) {
        throw std::runtime_error("Empty string field: " + fieldName);
    }
    return value;
}

}

fs::path ResolveParameterPath(const fs::path& pythonRoot, const fs::path& parameterPath) {
    if (parameterPath.is_absolute()) {
        return parameterPath.lexically_normal();
    }
    return (pythonRoot / parameterPath).lexically_normal();
}

Parameter LoadParameter(const fs::path& parameterPath) {
    cv::FileStorage storage = OpenParameterStorage(parameterPath);
    const cv::FileNode hsvNode = RequireNode(storage["HSV"], "HSV");
    const cv::FileNode maybeTargetNode = RequireNode(storage["MayBeTarget"], "MayBeTarget");

    Parameter parameter;
    parameter.parameterPath = parameterPath;
    parameter.hsv.lowerLimit = ReadScalar3(hsvNode["lowerLimit"], "HSV.lowerLimit");
    parameter.hsv.upperLimit = ReadScalar3(hsvNode["upperLimit"], "HSV.upperLimit");
    parameter.maybeTarget.width = ReadDouble(maybeTargetNode["width"], "MayBeTarget.width");
    parameter.maybeTarget.height = ReadDouble(maybeTargetNode["height"], "MayBeTarget.height");
    parameter.maybeTarget.area = ReadDouble(maybeTargetNode["area"], "MayBeTarget.area");
    parameter.outsideRate = ReadDouble(storage["outsideRate"], "outsideRate");
    parameter.insideRate = ReadDouble(storage["insideRate"], "insideRate");
    parameter.kernel = ReadInt(storage["kernel"], "kernel");
    parameter.videoRelativePath = ReadString(storage["video relative path"], "video relative path");
    parameter.start = ReadInt(storage["start"], "start");
    return parameter;
}

void SaveParameter(const Parameter& parameter) {
    if (parameter.parameterPath.empty()) {
        throw std::runtime_error("Parameter path is empty when saving");
    }

    std::ofstream output(parameter.parameterPath, std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        throw std::runtime_error("Failed to open parameter file for writing: " + parameter.parameterPath.string());
    }

    output << "{\n";
    output << "    \"HSV\": {\n";
    output << "        \"lowerLimit\": [\n";
    output << "            " << static_cast<int>(parameter.hsv.lowerLimit[0]) << ",\n";
    output << "            " << static_cast<int>(parameter.hsv.lowerLimit[1]) << ",\n";
    output << "            " << static_cast<int>(parameter.hsv.lowerLimit[2]) << "\n";
    output << "        ],\n";
    output << "        \"upperLimit\": [\n";
    output << "            " << static_cast<int>(parameter.hsv.upperLimit[0]) << ",\n";
    output << "            " << static_cast<int>(parameter.hsv.upperLimit[1]) << ",\n";
    output << "            " << static_cast<int>(parameter.hsv.upperLimit[2]) << "\n";
    output << "        ]\n";
    output << "    },\n";
    output << "    \"MayBeTarget\": {\n";
    output << "        \"width\": " << parameter.maybeTarget.width << ",\n";
    output << "        \"height\": " << parameter.maybeTarget.height << ",\n";
    output << "        \"area\": " << parameter.maybeTarget.area << "\n";
    output << "    },\n";
    output << "    \"outsideRate\": " << parameter.outsideRate << ",\n";
    output << "    \"insideRate\": " << parameter.insideRate << ",\n";
    output << "    \"kernel\": " << parameter.kernel << ",\n";
    output << "    \"video relative path\": \"" << parameter.videoRelativePath << "\",\n";
    output << "    \"start\": " << parameter.start << "\n";
    output << "}\n";
}

fs::path ResolveVideoPath(const Parameter& parameter, const fs::path& pythonRoot) {
    const fs::path videoPath(parameter.videoRelativePath);
    if (videoPath.is_absolute()) {
        return videoPath.lexically_normal();
    }
    return (pythonRoot / videoPath).lexically_normal();
}

}
