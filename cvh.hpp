//
// Created by yongqi on 18-12-28.
//

#ifndef CPP_ACCUMULATION_CVH_HPP
#define CPP_ACCUMULATION_CVH_HPP

#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "utils.hpp"

namespace cvh {

/**
 * generate random color
 * @return color scalar
 */
inline cv::Scalar random_color() {
  static cv::RNG rng(1);
  int icolor = (unsigned) rng;
  return cv::Scalar(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);
}

/**
 * generate bbox from polygon points
 * @param points
 * @return
 */
static cv::Rect polygon2bbox(const std::vector<cv::Point> &points) {
  int xmin = std::numeric_limits<int>::max();
  int xmax = 0;
  int ymin = xmin;
  int ymax = xmax;
  for (const auto &p : points) {
    xmin = std::min(xmin, p.x);
    xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y);
    ymax = std::max(ymax, p.y);
  }
  return cv::Rect(xmin, ymin, xmax - xmin, ymax - ymin);
}

/**
 * generate mask from polygon points
 * @param points
 * @return
 */
static cv::Mat polygon2mask(const std::vector<cv::Point> &points) {
  cv::Rect rect = polygon2bbox(points);
  cv::Mat mask(rect.height, rect.width, CV_8U, cv::Scalar(0));
  std::vector<cv::Point> shift_points(points);
  for (auto &p : shift_points) {
    p.x -= rect.x;
    p.y -= rect.y;
  }
  cv::fillConvexPoly(mask, shift_points, cv::Scalar(255), cv::LINE_AA);
  return mask;
}

/**
 * apply mask to image
 * @param img rgb image
 * @param mask uchar mat
 * @param color random rgb color
 */
static void apply_mask(cv::Mat &img, const cv::Mat &mask, const cv::Scalar &color) {
  assert(img.size == mask.size);
  float alpha = 0.5;
  for (int y = 0; y < mask.rows; ++y) {
    for (int x = 0; x < mask.cols; ++x) {
      if (mask.at<uchar>(y, x) > 0) {
        auto &vec = img.at<cv::Vec3b>(y, x);
        vec[0] = static_cast<uchar>(alpha * color[0] + (1 - alpha) * vec[0]);
        vec[1] = static_cast<uchar>(alpha * color[1] + (1 - alpha) * vec[1]);
        vec[2] = static_cast<uchar>(alpha * color[2] + (1 - alpha) * vec[2]);
      }
    }
  }
}

/**
 * draw a label on image
 */
inline void put_label(cv::Mat &img, const std::string &label, const cv::Point &p, const cv::Scalar &color) {
  int fontface = cv::FONT_HERSHEY_SIMPLEX;
  double scale = 0.4;
  int thickness = 1;
  int baseline = 0;

  cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
  cv::rectangle(img, p + cv::Point(0, baseline), p + cv::Point(text.width, -text.height), CV_RGB(0, 0, 0), CV_FILLED);
  cv::putText(img, label, p, fontface, scale, color, thickness, 8);
}

/**
 * write depth in meter into visualized png picture
 * @param depth_file save file name
 * @param depth_meter depth in meter Mat
 */
static void write_depth(const std::string &depth_file, const cv::Mat &depth_meter) {
  cv::Mat depth_mat(depth_meter.size(), CV_16UC1);
  for (int y = 0; y < depth_mat.rows; y++)
    for (int x = 0; x < depth_mat.cols; x++) {
      unsigned short depth_short = static_cast<unsigned short>(std::round(depth_meter.at<float>(y, x) * 10000.));
      depth_short = (depth_short >> 13 | depth_short << 3);
      depth_mat.at<unsigned short>(y, x) = depth_short;
    }
  std::vector<int> compression_params;
  compression_params.emplace_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.emplace_back(9);
  cv::imwrite(depth_file, depth_mat, compression_params);
}

/**
 * read visualized png depth map into depth in meter
 * @param depth_file read file name
 * @return depth in meter mat
 */
static cv::Mat read_depth(const std::string &depth_file) {
  cv::Mat depth_mat = cv::imread(depth_file, CV_16UC1);
  cv::Mat depth_meter(depth_mat.size(), CV_32F);
  for (int y = 0; y < depth_mat.rows; ++y)
    for (int x = 0; x < depth_mat.cols; ++x) {
      auto depth_short = depth_mat.at<unsigned short>(y, x);
      depth_short = (depth_short << 13 | depth_short >> 3);
      depth_meter.at<float>(y, x) = depth_short / 10000.f;
    }
  return depth_meter;
}

/**
 * load param from filestorage if name exist
 * @tparam T
 * @param fs
 * @param name
 * @param param
 * @return
 */
template<typename T>
inline bool load_param(const cv::FileStorage &fs, const std::string &name, T &param) {
  if (fs[name].empty()) {
    std::cout << "using default " << name << ": " << param << std::endl;
    return false;
  } else {
    fs[name] >> param;
    std::cout << "load " << name << ": " << param << std::endl;
    return true;
  }
}

/**
 * make angle between [-pi, pi)
 * @param x input angle in radius
 * @return normalized angle
 */
inline double normalize_angle(double x) {
  x = fmod(x + M_PI, 2 * M_PI);
  return x < 0 ? x += 2 * M_PI : x - M_PI;
}

inline float normalize_anlge(float x) {
  x = fmod(x + M_PI, 2 * M_PI);
  return x < 0 ? x += 2 * M_PI : x - M_PI;
}

/**
 * radian to degree
 * @tparam T
 * @param x
 * @return
 */
template<typename T>
inline T rad2deg(T x) {
  return x / M_PI * 180.;
}

/**
 * degree to radian
 * @tparam T
 * @param x
 * @return
 */
template<typename T>
inline T deg2rad(T x) {
  return x * M_PI / 180.;
}

/**
 * Convert pose(in Rz(yaw)Ry(pitch)Rx(roll) order) into transformation matrix
 * @tparam T
 * @param x
 * @param y
 * @param z
 * @param roll
 * @param pitch
 * @param yaw
 * @param tf
 */
template<typename T>
void pose2matrix(const T &x, const T &y, const T &z,
                 const T &roll, const T &pitch, const T &yaw,
                 Eigen::Matrix<T, 4, 4> &t) {
  T A = cos(yaw),  B = sin(yaw),  C = cos(pitch), D = sin (pitch),
      E = cos(roll), F = sin(roll), DE = D*E,       DF = D*F;

  t(0, 0) = A*C;  t(0, 1) = A*DF - B*E;  t(0, 2) = B*F + A*DE;  t(0, 3) = x;
  t(1, 0) = B*C;  t(1, 1) = A*E + B*DF;  t(1, 2) = B*DE - A*F;  t(1, 3) = y;
  t(2, 0) = -D;   t(2, 1) = C*F;         t(2, 2) = C*E;         t(2, 3) = z;
  t(3, 0) = 0;    t(3, 1) = 0;           t(3, 2) = 0;           t(3, 3) = 1;
}

/**
 * Convert transformation matrix into pose
 * @tparam T
 * @param matrix
 * @param x
 * @param y
 * @param z
 * @param roll
 * @param pitch
 * @param yaw
 */
template<typename T>
void matrix2pose(const Eigen::Matrix<T, 4, 4> &t,
                 T &x, T &y, T &z,
                 T &roll, T &pitch, T &yaw) {
  x = t(0, 3);
  y = t(1, 3);
  z = t(2, 3);
  roll = atan2(t(2, 1), t(2, 2));
  pitch = atan2(-t(2, 0), sqrt(t(2, 1) * t(2, 1) + t(2, 2) * t(2, 2)));
  yaw = atan2(t(1, 0), t(0, 0));
}

/**
 * calculate the angle between two vector
 * @tparam T
 * @param vec1
 * @param vec2
 * @param dim
 * @return angle in [0, pi]
 */
template <typename T>
T included_angle(const std::vector<T> &vec1, const std::vector<T> &vec2, int dim = 3) {
  assert(vec1.size() == vec2.size());
  assert(vec1.size() >= dim);
  T dot = 0;
  T len_sqr1 = 0;
  T len_sqr2 = 0;
  for (int i = 0; i < dim; ++i) {
    dot += vec1[i] * vec2[i];
    len_sqr1 += vec1[i] * vec1[i];
    len_sqr2 += vec2[i] * vec2[i];
  }
  return acos(dot / sqrt(len_sqr1) / sqrt(len_sqr2));
}

}

#endif //CPP_ACCUMULATION_CVH_HPP
