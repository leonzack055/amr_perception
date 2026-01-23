#ifndef REFLECTOR_DETECTOR_HPP_
#define REFLECTOR_DETECTOR_HPP_

#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>
#include <functional>
#include <fstream>
#include <chrono>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Eigen>

#include "absl/strings/substitute.h"

namespace transforms
{
template<typename T>
T NormalizeAngleDifference(T difference)
{
  const T kPi = T(M_PI);
  while (difference > kPi) {difference -= 2. * kPi;}
  while (difference < -kPi) {difference += 2. * kPi;}
  return difference;
}

template<typename FloatType>
class Rigid2
{
public:
  using Vector = Eigen::Matrix<FloatType, 2, 1>;
  using Rotation2D = Eigen::Rotation2D<FloatType>;

  Rigid2()
  : translation_(Vector::Zero()), rotation_(Rotation2D::Identity()) {}
  Rigid2(const Vector & translation, const Rotation2D & rotation)
  : translation_(translation), rotation_(rotation) {}
  Rigid2(const Vector & translation, const double rotation)
  : translation_(translation), rotation_(rotation) {}

  static Rigid2 Rotation(const double rotation)
  {
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Rotation(const Rotation2D & rotation)
  {
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Translation(const Vector & vector)
  {
    return Rigid2(vector, Rotation2D::Identity());
  }

  static Rigid2<FloatType> Identity() {return Rigid2<FloatType>();}

  template<typename OtherType>
  Rigid2<OtherType> cast() const
  {
    return Rigid2<OtherType>(
      translation_.template cast<OtherType>(),
      rotation_.template cast<OtherType>());
  }

  const Vector & translation() const {return translation_;}

  Rotation2D rotation() const {return rotation_;}

  double normalized_angle() const
  {
    return NormalizeAngleDifference(rotation().angle());
  }

  Rigid2 inverse() const
  {
    const Rotation2D rotation = rotation_.inverse();
    const Vector translation = -(rotation * translation_);
    return Rigid2(translation, rotation);
  }

  std::string DebugString() const
  {
    return absl::Substitute(
      "{ t: [$0, $1], r: [$2] }", translation().x(),
      translation().y(), rotation().angle());
  }
  // 关键字内存对齐
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Vector translation_;
  Rotation2D rotation_;
};

template<typename FloatType>
Rigid2<FloatType> operator*(
  const Rigid2<FloatType> & lhs,
  const Rigid2<FloatType> & rhs)
{
  return Rigid2<FloatType>(
    lhs.rotation() * rhs.translation() + lhs.translation(),
    lhs.rotation() * rhs.rotation());
}

template<typename FloatType>
typename Rigid2<FloatType>::Vector operator*(
  const Rigid2<FloatType> & rigid,
  const typename Rigid2<FloatType>::Vector & point)
{
  return rigid.rotation() * point + rigid.translation();
}

// This is needed for gmock.
template<typename T>
std::ostream & operator<<(
  std::ostream & os,
  const transforms::Rigid2<T> & rigid)
{
  os << rigid.DebugString();
  return os;
}

using Rigid2d = Rigid2<double>;
using Rigid2f = Rigid2<float>;

template<typename FloatType>
class Rigid3
{
public:
  using Vector = Eigen::Matrix<FloatType, 3, 1>;
  using Quaternion = Eigen::Quaternion<FloatType>;
  using AngleAxis = Eigen::AngleAxis<FloatType>;

  Rigid3()
  : translation_(Vector::Zero()), rotation_(Quaternion::Identity()) {}
  Rigid3(const Vector & translation, const Quaternion & rotation)
  : translation_(translation), rotation_(rotation) {}
  Rigid3(const Vector & translation, const AngleAxis & rotation)
  : translation_(translation), rotation_(rotation) {}

  static Rigid3 Rotation(const AngleAxis & angle_axis)
  {
    return Rigid3(Vector::Zero(), Quaternion(angle_axis));
  }

  static Rigid3 Rotation(const Quaternion & rotation)
  {
    return Rigid3(Vector::Zero(), rotation);
  }

  static Rigid3 Translation(const Vector & vector)
  {
    return Rigid3(vector, Quaternion::Identity());
  }

  static Rigid3 FromArrays(
    const std::array<FloatType, 4> & rotation,
    const std::array<FloatType, 3> & translation)
  {
    return Rigid3(
      Eigen::Map<const Vector>(translation.data()),
      Eigen::Quaternion<FloatType>(
        rotation[0], rotation[1],
        rotation[2], rotation[3]));
  }

  static Rigid3<FloatType> Identity() {return Rigid3<FloatType>();}

  template<typename OtherType>
  Rigid3<OtherType> cast() const
  {
    return Rigid3<OtherType>(
      translation_.template cast<OtherType>(),
      rotation_.template cast<OtherType>());
  }

  const Vector & translation() const {return translation_;}
  const Quaternion & rotation() const {return rotation_;}

  Rigid3 inverse() const
  {
    const Quaternion rotation = rotation_.conjugate();
    const Vector translation = -(rotation * translation_);
    return Rigid3(translation, rotation);
  }

  std::string DebugString() const
  {
    return absl::Substitute(
      "{ t: [$0, $1, $2], q: [$3, $4, $5, $6] }",
      translation().x(), translation().y(),
      translation().z(), rotation().w(), rotation().x(),
      rotation().y(), rotation().z());
  }

  bool IsValid() const
  {
    return !std::isnan(translation_.x()) && !std::isnan(translation_.y()) &&
           !std::isnan(translation_.z()) &&
           std::abs(FloatType(1) - rotation_.norm()) < FloatType(1e-3);
  }
  // 关键字内存对齐
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Vector translation_;
  Quaternion rotation_;
};

template<typename FloatType>
Rigid3<FloatType> operator*(
  const Rigid3<FloatType> & lhs,
  const Rigid3<FloatType> & rhs)
{
  return Rigid3<FloatType>(
    lhs.rotation() * rhs.translation() + lhs.translation(),
    (lhs.rotation() * rhs.rotation()).normalized());
}

template<typename FloatType>
typename Rigid3<FloatType>::Vector operator*(
  const Rigid3<FloatType> & rigid,
  const typename Rigid3<FloatType>::Vector & point)
{
  return rigid.rotation() * point + rigid.translation();
}

// This is needed for gmock.
template<typename T>
std::ostream & operator<<(
  std::ostream & os,
  const transforms::Rigid3<T> & rigid)
{
  os << rigid.DebugString();
  return os;
}

using Rigid3d = Rigid3<double>;
using Rigid3f = Rigid3<float>;

// Returns the yaw component in radians of the given 3D 'rotation'. Assuming
// 'rotation' is composed of three rotations around X, then Y, then Z, returns
// the angle of the Z rotation.
template<typename T>
T GetYaw(const Eigen::Quaternion<T> & rotation)
{
  const Eigen::Matrix<T, 3, 1> direction =
    rotation * Eigen::Matrix<T, 3, 1>::UnitX();
  return atan2(direction.y(), direction.x());
}

// Returns the yaw component in radians of the given 3D transformation
// 'transform'.
template<typename T>
T GetYaw(const Rigid3<T> & transform)
{
  return GetYaw(transform.rotation());
}

template<typename T>
Rigid3<T> FromYaw(T yaw)
{
  return Rigid3<T>(Rigid3<T>::Vector::Zero, Rigid3<T>::AngleAxis(yaw, Eigen::Matrix<T, 3, 1>::UnitZ()));
}

} // namespace transforms


struct PoseStamped
{
  transforms::Rigid3d pose;
  std::string header = "";
};

struct Detection
{
  PoseStamped pose;
  double diameter;
  double confidence;
  double translationW;
  double rotationW;
};

struct ReflectivePost
{
  struct { double x = 0.0; double y = 0.0; double z = 0.0; } position;   // In lidar frame
  double translation_weight = 0.0;
  double rotation_weight = 0.0;
};

class ReflectorBar
{
public:
  Detection g_detection_; // 全局位姿下的坐标
  std::string id_str_;
  int64_t time_;
  int age_;
  int id_;
  bool optimized_;
};
using ReflectorBarMap = std::map<int, ReflectorBar>;

#endif //REFLECTOR_DETECTOR_HPP_
