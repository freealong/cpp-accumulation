//
// Created by yongqi on 18-12-28.
//

#ifndef CPP_ACCUMULATION_UTILS_HPP
#define CPP_ACCUMULATION_UTILS_HPP

#include <iostream>
#include <chrono>

namespace utils {

/**
 * format string using c style
 * @tparam Args
 * @param format
 * @param args
 * @return
 */
template<typename ... Args>
std::string string_format(const std::string& format, Args ... args) {
  int size = snprintf(nullptr, 0, format.c_str(), args ...) + 1; // Extra space for '\0'
  std::unique_ptr<char[]> buf(new char[ size ]);
  snprintf(buf.get(), size, format.c_str(), args ...);
  return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
}

/**
 * split a file's full path name into path and filename
 * @param full_path
 * @param path
 * @param name
 * @return
 */
inline bool split_path(const std::string &full_path, std::string &path, std::string &name) {
  if (full_path.empty()) {
    return false;
  }
  auto pos = full_path.rfind('/');
  if (pos == std::string::npos) {
    path.assign(".");
    name = full_path;
  } else {
    path = full_path.substr(0, pos);
    name = full_path.substr(pos+1);
  }
  return true;
}

/**
 * split string
 * @param str
 * @param delim
 * @return
 */
static std::vector<std::string> split(const std::string& str, const std::string& delim) {
  std::vector<std::string> tokens;
  size_t prev = 0, pos = 0;
  do
  {
    pos = str.find(delim, prev);
    if (pos == std::string::npos) pos = str.length();
    std::string token = str.substr(prev, pos-prev);
    if (!token.empty()) tokens.push_back(token);
    prev = pos + delim.length();
  }
  while (pos < str.length() && prev < str.length());
  return tokens;
}

/**
 * get current time and date in string
 * @param format
 * @return
 */
inline std::string current_time_and_date(const std::string &format = "%Y-%m-%d_%X") {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), format.c_str());
  return ss.str();
}

/**
 * cout vector
 * @tparam T
 * @param os
 * @param vec
 * @return
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec) {
  const auto iter_begin = vec.begin();
  const auto iter_end   = vec.end();
  os << "[";
  for (auto iter = iter_begin; iter != iter_end; ++iter) {
    std::cout << ((iter != iter_begin) ? "," : "") << *iter;
  }
  os << "]";
  return os;
}

template <typename timestep>
class Timer {
 public:
  typedef std::chrono::high_resolution_clock clock;

  Timer() {
    reset();
  }

  void reset() {
    beg_ = clock::now();
  }

  inline timestep elapsed() const {
    return std::chrono::duration_cast<timestep>(clock::now() - beg_);
  }

  friend std::ostream& operator<<(std::ostream &os, const Timer &t) {
    return os << t.elapsed().count();
  }

 private:
  clock::time_point beg_;
};

}

#endif //CPP_ACCUMULATION_UTILS_HPP
