#pragma once

#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

namespace onion_relocalization {

inline void saveDescriptor(const std::string& path,
                           const Eigen::MatrixXd& descriptor) {
  if (descriptor.rows() <= 0 || descriptor.cols() <= 0 ||
      !descriptor.allFinite()) {
    throw std::runtime_error("cannot save an empty or non-finite descriptor");
  }

  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot create descriptor file: " + path);
  }
  const char magic[8] = {'O', 'N', 'I', 'O', 'N', 'S', 'C', '1'};
  const std::uint32_t rows =
      static_cast<std::uint32_t>(descriptor.rows());
  const std::uint32_t cols =
      static_cast<std::uint32_t>(descriptor.cols());
  output.write(magic, sizeof(magic));
  output.write(reinterpret_cast<const char*>(&rows), sizeof(rows));
  output.write(reinterpret_cast<const char*>(&cols), sizeof(cols));
  output.write(reinterpret_cast<const char*>(descriptor.data()),
               static_cast<std::streamsize>(
                   sizeof(double) * descriptor.size()));
  if (!output) {
    throw std::runtime_error("failed while writing descriptor file: " +
                             path);
  }
}

inline Eigen::MatrixXd loadDescriptor(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("cannot open descriptor file: " + path);
  }
  char magic[8] = {};
  std::uint32_t rows = 0;
  std::uint32_t cols = 0;
  input.read(magic, sizeof(magic));
  input.read(reinterpret_cast<char*>(&rows), sizeof(rows));
  input.read(reinterpret_cast<char*>(&cols), sizeof(cols));
  const char expected[8] = {'O', 'N', 'I', 'O', 'N', 'S', 'C', '1'};
  if (!input || std::memcmp(magic, expected, sizeof(expected)) != 0 ||
      rows == 0 || cols == 0 || rows > 4096 || cols > 4096) {
    throw std::runtime_error("invalid descriptor header: " + path);
  }
  Eigen::MatrixXd descriptor(rows, cols);
  input.read(reinterpret_cast<char*>(descriptor.data()),
             static_cast<std::streamsize>(
                 sizeof(double) * descriptor.size()));
  if (!input || !descriptor.allFinite()) {
    throw std::runtime_error("invalid descriptor payload: " + path);
  }
  return descriptor;
}

}  // namespace onion_relocalization
