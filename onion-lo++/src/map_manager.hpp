/**
MIT License

Copyright (c) 2025 Xiaolong Cheng <chengxiaolong658@163.com>.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

* Note:
* The `struct VoxelBlock`, `VoxelHash`, and functions `UpdateModelDeviation`,
* `ComputeModelError`, `ComputeThreshold`, which compute the threshold parameter
* for point registration, are copied from the work KISS-ICP
(https://github.com/PRBonn/kiss-icp),
* which is licensed under the MIT License.
*
* Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
Stachniss.
*
* The implementation of the map structure in this file is heavily inspired by
the work KISS-ICP.
* Thanks for their great effort and for open sourcing the code for the
community.

*/

#ifndef TRAJLO_MAP_MANAGER_H
#define TRAJLO_MAP_MANAGER_H

#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tsl/robin_map.h>

#include <pcl/point_cloud.h>

#include <trajlo/common_type.h>
#include <trajlo/map_point_type.hpp>
#include <trajlo/sophus_utils.hpp>


namespace traj {
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector6dVector = std::vector<Vector6d>;
class MapManager {
 public:
  using Ptr = std::shared_ptr<MapManager>;
  using Voxel = Eigen::Vector3i;
  int max_points_per_voxel_ = 20;
  double reg_thresh_ = 1.5;
  double planer_threshold_ = 0.01;
  struct VoxelBlock {
    std::vector<Vector6d> points; 
    /*
    inline void AddPoint(const Vector6d &point, int max_points) {
      if (points.size() < static_cast<size_t>(max_points))
        std::cout<<point[4]<<std::endl;
        points.push_back(point);
    }
    */
    
	inline void AddPoint(const Vector6d &point, int max_points, double window_ms = 100.0) {
		double current_time = point[4];  // 当前点时间戳
		// 删除超出时间窗口的点
		points.erase(
		    std::remove_if(points.begin(), points.end(),
		                   [current_time, window_ms](const Vector6d &p) {
		                       return current_time - p[4] > window_ms;
		                   }),
		    points.end());
		// 添加新点（保持最大点数限制）
		if (points.size() < static_cast<size_t>(max_points)) {
		    points.push_back(point);
		}
	}

  };

  struct VoxelHash {
    size_t operator()(const Voxel &voxel) const {
      const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
      return ((1 << 20) - 1) &
           (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
  };

  MapManager(double voxel_size, double planer_threshold)
      : voxel_size_(voxel_size),
        planer_threshold_(planer_threshold)
        {}

  std::vector<Vector6d> DownSampling(const std::vector<Vector6d> &points, double ds_size, bool de_factor);

  void PreProcess(const std::vector<Vector6d> &points,
                  const tStampPair &tp, double ds_size, bool de_factor);

  void MapInit(const std::vector<Vector6d> &points);

  void Update(const posePair &pp, const tStampPair &tp);

  bool LoadGlobalMap(const std::string &path, bool read_only,
                     std::string *error = nullptr);

  pcl::PointCloud<MapPointXYZIL>::Ptr ExportMapCloud() const;

  inline bool IsReadOnly() const { return read_only_map_; }

  inline bool HasMapLabels() const { return map_has_labels_; }

  std::size_t MapPointCount() const;

  struct ResultTuple {
    ResultTuple() {
      JTJ.setZero();
      JTr.setZero();
      error = 0;
      inlier = 0;
    }

    ResultTuple operator+(const ResultTuple &other) {
      this->JTJ += other.JTJ;
      this->JTr += other.JTr;
      this->error += other.error;
      this->inlier += other.inlier;
      return *this;
    }

    Eigen::Matrix<double, 12, 12> JTJ;
    Eigen::Matrix<double, 12, 1> JTr;
    double error;
    double inlier;
  };

  // find neighborhoods in seven voxels
  std::array<Eigen::Vector3i, 7> coord{
      Voxel(0, 0, 0),  Voxel(1, 0, 0),  Voxel(0, 1, 0), Voxel(0, 0, 1),
      Voxel(-1, 0, 0), Voxel(0, -1, 0), Voxel(0, 0, -1)};

  void PointRegistrationNormal(/*const posePair& pp,*/
                               const posePairLin &ppl, const tStampPair &tp,
                               Eigen::Matrix<double, 12, 12> &H_icp,
                               Eigen::Matrix<double, 12, 1> &b_icp,
                               double &error, double &inliers);

  inline bool IsInit() { return init_flag; }

  inline void SetInit() { init_flag = true; }


 private:

  // map
  bool init_flag = false;
  bool read_only_map_ = false;
  bool map_has_labels_ = true;
  double voxel_size_;
  tsl::robin_map<Voxel, VoxelBlock, VoxelHash> map;
  std::map<tStampPair, std::vector<Vector6d>> reg_points_database;
  std::map<tStampPair, std::vector<Vector6d>> map_points_database;
  int64_t last_scan_ts;
};

}  // namespace traj

#endif  // TRAJLO_MAP_MANAGER_H
