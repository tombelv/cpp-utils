#pragma once

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <vector>

#include <dart/dart.hpp>
#include "math.h"

namespace utils {

Eigen::Vector3d getRPY(dart::dynamics::BodyNode *body) {
  Eigen::Matrix3d rotMatrix = body->getTransform().rotation();
  return utils::getRPY(rotMatrix);
}

} // end namespace utils















