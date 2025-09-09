#pragma once

#include <vector>
#include <unordered_map>
#include <string>

struct Landmark {
    int id;
    double x;
    double y;
    double translation_weight;
    double rotation_weight;
    
    Landmark() : id(-1), x(0.0), y(0.0), translation_weight(1.0), rotation_weight(1.0) {}
    Landmark(int id, double x, double y, double translation_weight, double rotation_weight) : 
        id(id), x(x), y(y), translation_weight(translation_weight), rotation_weight(rotation_weight) {}
};

class LandmarkMatcher {
public:
    LandmarkMatcher(double threshold = 1.0);
    ~LandmarkMatcher() = default;

    void setDistanceThreshold(double threshold);
    double getDistanceThreshold() const;
    
    void addPriorLandmark(const Landmark& landmark);
    bool getPriorLandmark(int id, Landmark& landmark) const;
    std::vector<int> getPriorLandmarkIDs() const;
    void clearPriorMap();
    
    std::vector<int> matchLandmarks(const std::vector<Landmark>& detectedLandmarks);

private:
    std::unordered_map<int, Landmark> priorMap_;
    double distanceThreshold_;
};