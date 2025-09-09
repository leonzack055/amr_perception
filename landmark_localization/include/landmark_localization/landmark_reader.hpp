#pragma once

#include <string>
#include <vector>
#include <iostream>

struct LandmarkInfo {
    std::string landmark_id;
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
};

class LandmarkReader {
public:
    LandmarkReader(const std::string& pbstream_file);
    ~LandmarkReader();

    bool readLandmarks();
    const std::vector<LandmarkInfo>& getLandmarks() const;
    void printLandmarks() const;

private:
    std::string pbstream_file_;
    std::vector<LandmarkInfo> landmarks_;
};