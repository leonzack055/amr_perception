#include "landmark_localization/landmark_matcher.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <chrono>

// 计算两个landmark之间的欧几里得距离
double calculateDistance(const Landmark & a, const Landmark & b)
{
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

// 匈牙利算法实现
class HungarianAlgorithm
{
private:
  std::vector<std::vector<double>> costMatrix;
  int n;   // 矩阵大小
  std::vector<double> u, v;
  std::vector<int> p, way;
  std::vector<bool> used;
  const double epsilon = 1e-10;   // 浮点数精度容差

public:
  HungarianAlgorithm(const std::vector<std::vector<double>> & matrix)
  : costMatrix(matrix), n(matrix.size())
  {
    u.assign(n + 1, 0.0);
    v.assign(n + 1, 0.0);
    p.assign(n + 1, 0);
    way.assign(n + 1, 0);
  }

  std::vector<int> solve()
  {
    std::cout << "Starting Hungarian algorithm" << std::endl;

    for (int i = 1; i <= n; i++) {
      p[0] = i;
      int j0 = 0;
      std::vector<double> minv(n + 1, std::numeric_limits<double>::max());
      used.assign(n + 1, false);

      int iteration_count = 0;
      const int max_iterations = n * n;       // 安全计数器

      do {
        iteration_count++;
        if (iteration_count > max_iterations) {
          // std::cerr << "Warning: Hungarian algorithm exceeded maximum iterations ("
          //           << max_iterations << ")" << std::endl;
          break;
        }

        used[j0] = true;
        int i0 = p[j0];
        double delta = std::numeric_limits<double>::max();
        int j1 = 0;

        // 寻找最小值和delta
        for (int j = 1; j <= n; j++) {
          if (!used[j]) {
            double cur = costMatrix[i0 - 1][j - 1] - u[i0] - v[j];

            // 处理可能的NaN或异常值
            if (std::isnan(cur)) {cur = std::numeric_limits<double>::max();}

            if (cur < minv[j] - epsilon) {
              minv[j] = cur;
              way[j] = j0;
            }

            if (minv[j] < delta - epsilon) {
              delta = minv[j];
              j1 = j;
            }
          }
        }

        // 检查delta是否有效
        if (delta == std::numeric_limits<double>::max() || std::isnan(delta)) {
          // std::cerr << "Error: Invalid delta value detected" << std::endl;
          break;
        }

        // 更新u和v
        for (int j = 0; j <= n; j++) {
          if (used[j]) {
            u[p[j]] += delta;
            v[j] -= delta;
          } else {
            minv[j] -= delta;
          }
        }

        j0 = j1;

      } while (p[j0] != 0);

      // 路径回溯
      do {
        int j1 = way[j0];
        p[j0] = p[j1];
        j0 = j1;
      } while (j0 != 0);
    }

    std::cout << "Hungarian algorithm completed" << std::endl;

    // 构建结果
    std::vector<int> result(n);
    for (int j = 1; j <= n; j++) {
      if (p[j] > 0 && p[j] <= n) {
        result[p[j] - 1] = j - 1;
      } else {
        // 处理异常情况
        result[j - 1] = j - 1;         // 默认分配
        // std::cerr << "Warning: Invalid assignment at position " << j << std::endl;
      }
    }

    return result;
  }
};

LandmarkMatcher::LandmarkMatcher(double threshold)
: distanceThreshold_(threshold) {}

void LandmarkMatcher::setDistanceThreshold(double threshold)
{
  distanceThreshold_ = threshold;
}

double LandmarkMatcher::getDistanceThreshold() const
{
  return distanceThreshold_;
}

void LandmarkMatcher::addPriorLandmark(const Landmark & landmark)
{
  priorMap_[landmark.id] = landmark;
}

bool LandmarkMatcher::getPriorLandmark(int id, Landmark & landmark) const
{
  auto it = priorMap_.find(id);
  if (it != priorMap_.end()) {
    landmark = it->second;
    return true;
  }
  return false;
}

std::vector<int> LandmarkMatcher::getPriorLandmarkIDs() const
{
  std::vector<int> ids;
  for (const auto & pair : priorMap_) {
    ids.push_back(pair.first);
  }
  return ids;
}

void LandmarkMatcher::clearPriorMap()
{
  priorMap_.clear();
}

std::vector<int> LandmarkMatcher::matchLandmarks(const std::vector<Landmark> & detectedLandmarks)
{
  // 开始计时
  auto start_time = std::chrono::high_resolution_clock::now();

  int n = detectedLandmarks.size();
  int m = priorMap_.size();

  if (n == 0 || m == 0) {
    // 结束计时
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "匹配耗时: " << duration.count() << " 毫秒" << std::endl;

    return std::vector<int>(n, -1);     // 返回全部未匹配
  }

  // 创建成本矩阵
  std::vector<std::vector<double>> costMatrix(n, std::vector<double>(m, 0.0));

  // 提取先验landmark到向量中以便按索引访问
  std::vector<Landmark> priorLandmarks;
  for (const auto & pair : priorMap_) {
    // std::cout << "Prior Landmark ID: " << pair.second.id
    //           << " Position: (" << pair.second.x << ", " << pair.second.y << ")" << std::endl;
    priorLandmarks.push_back(pair.second);
  }

  // 填充成本矩阵（使用距离作为成本）
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < m; j++) {
      costMatrix[i][j] = calculateDistance(detectedLandmarks[i], priorLandmarks[j]);
    }
  }

  // 如果检测到的landmark数量少于先验landmark数量，需要扩展矩阵
  int original_n = n;
  if (n < m) {
    for (int i = n; i < m; i++) {
      costMatrix.push_back(std::vector<double>(m, std::numeric_limits<double>::max()));
      n = m;       // 更新n的大小
    }
  }

  // std::cout << "costMatrix size: " << costMatrix.size() << " x " << costMatrix[0].size() << std::endl;

  // 应用匈牙利算法
  HungarianAlgorithm ha(costMatrix);
  std::vector<int> assignment = ha.solve();

  // std::cout << "匹配结果priorLandmarks索引: ";
  // for (int idx : assignment) {
  //     std::cout << idx << " ";
  // }
  // std::cout << std::endl;

  // 处理结果，应用距离阈值
  std::vector<int> result(original_n, -1);   // 初始化为全部未匹配

  // 结束计时
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "匹配耗时: " << duration.count() << " 毫秒" << std::endl;

  for (int i = 0; i < original_n; i++) {
    int matchedIndex = assignment[i];
    // if (matchedIndex < priorLandmarks.size()) {
    if (matchedIndex >= 0 && matchedIndex < static_cast<int>(priorLandmarks.size())) {
      double distance = costMatrix[i][matchedIndex];
      if (distance <= distanceThreshold_) {
        result[i] = priorLandmarks[matchedIndex].id;
        std::cout << "匹配成功: 检测点 (" << detectedLandmarks[i].x << ", "
                  << detectedLandmarks[i].y << ") 与先验点 ID "
                  << priorLandmarks[matchedIndex].id << " 距离 " << distance << std::endl;
      } else {
        // 距离超过阈值，舍弃该匹配
        result[i] = -1;
        std::cout << "舍弃匹配: 检测点 (" << detectedLandmarks[i].x << ", "
                  << detectedLandmarks[i].y << ") 与先验点 ID "
                  << priorLandmarks[matchedIndex].id << " 距离 " << distance
                  << " 超过阈值 " << distanceThreshold_ << std::endl;
      }
    }
  }

  // std::cout << "最终匹配结果ID: ";
  // for (int id : result) {
  //     std::cout << id << " ";
  // }
  // std::cout << std::endl;

  return result;
}
