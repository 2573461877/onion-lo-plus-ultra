#pragma once

#include <algorithm>
#include <tsl/robin_map.h>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <thread>
#include <mutex>
#include <iostream>
#include <cmath>
#include <cstdint>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <trajlo/common_type.h>
#include <trajlo/map_point_type.hpp>

using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector3dVector = std::vector<Vector3d>;
using Vector3i = Eigen::Matrix<int, 3, 1>;
using Voxel = Vector3i;

struct Save_VoxelHashMap {
    double voxel_size_;
    double reslt_;

    struct Color_VoxelBlock {
        mutable Vector3dVector points;
        mutable Vector3dVector new_points;

        explicit Color_VoxelBlock(const Vector3d& point) {
            points.push_back(point);
            new_points.push_back(point);
        }

        inline void AddPoint(const Vector3d& point, double dis) const {
            bool is_within_range = false;
            for (const auto& existing_point : points) {
                double distance = (existing_point - point).norm();
                if (distance < dis) {
                    is_within_range = true;
                    break;
                }
            }
            if (!is_within_range) {
                points.push_back(point);
                new_points.push_back(point);
            }
        }
    };

    struct Color_VoxelHash {
        size_t operator()(const Voxel& voxel) const {
            const uint32_t* vec = reinterpret_cast<const uint32_t*>(voxel.data());
            return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
        }
    };

    inline bool Color_Empty() const { return colormap_.empty(); }

    void XYZ_AddPoints(const std::vector<Vector3d>& points) {
        if (voxel_size_ < 1e-4) {
            std::cerr << "[Warning] voxel_size_ too small, may cause memory explosion!" << std::endl;
            return;
        }

        for (const auto& point : points) {
            int voxel_x = static_cast<int>(std::round(point[0] / voxel_size_));
            int voxel_y = static_cast<int>(std::round(point[1] / voxel_size_));
            int voxel_z = static_cast<int>(std::round(point[2] / voxel_size_));
            Vector3i voxel(voxel_x, voxel_y, voxel_z);

            auto search = colormap_.find(voxel);
            if (search != colormap_.end()) {
                auto& voxel_block = search->second;
                voxel_block.AddPoint(point, reslt_);
            } else {
                colormap_.emplace(voxel, Color_VoxelBlock(point));
            }
        }

        const size_t kMaxVoxels = 2e6;
        if (colormap_.size() > kMaxVoxels) {
            std::cerr << "\033[31m[Error] colormap_ size exceeded limit (" << kMaxVoxels << "), clearing.\033[0m" << std::endl;
            colormap_.clear();
            colormap_.rehash(0);
        }
    }

    std::vector<Eigen::Vector3d> Pointcloud() const {
        std::vector<Eigen::Vector3d> save_points;

        size_t estimated_point_count = 0;
        for (const auto& [_, block] : colormap_) {
            estimated_point_count += block.new_points.size();
        }

        const size_t kMaxPoints = 5e6;
        if (estimated_point_count > kMaxPoints) {
            std::cerr << "\033[31m[Error] Extracted pointcloud too large (" << estimated_point_count
                      << "), skipping to avoid bad_alloc.\033[0m" << std::endl;
            return {};
        }

        save_points.reserve(estimated_point_count);

        for (const auto& [_, voxel_block] : colormap_) {
            save_points.insert(save_points.end(),
                               voxel_block.new_points.begin(),
                               voxel_block.new_points.end());

            const_cast<Vector3dVector&>(voxel_block.new_points).clear();
        }

        return save_points;
    }

    tsl::robin_map<Voxel, Color_VoxelBlock, Color_VoxelHash> colormap_;
};

// Persistent global map used only for map export.  It is deliberately
// independent from MapManager's rolling registration map so that local-map
// distance pruning and point aging never remove points from the saved map.
class GlobalMapVoxelCache {
 public:
    struct Key {
        int x = 0;
        int y = 0;
        int z = 0;
        std::uint32_t label = 0U;

        bool operator==(const Key& other) const {
            return x == other.x && y == other.y && z == other.z &&
                   label == other.label;
        }
    };

    struct KeyHash {
        std::size_t operator()(const Key& key) const {
            std::size_t seed = static_cast<std::size_t>(key.x) * 73856093U;
            seed ^= static_cast<std::size_t>(key.y) * 19349663U;
            seed ^= static_cast<std::size_t>(key.z) * 83492791U;
            seed ^= static_cast<std::size_t>(key.label) * 2654435761U;
            return seed;
        }
    };

    void Configure(double voxel_size) {
        std::lock_guard<std::mutex> lock(mutex_);
        voxel_size_ = std::max(1e-3, voxel_size);
    }

    void AddPoints(const std::vector<traj::PointXYZI>& points) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& point : points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z)) {
                continue;
            }

            const auto label = static_cast<std::uint32_t>(
                std::max(0L, std::lround(point.label)));
            const Key key{
                static_cast<int>(std::floor(point.x / voxel_size_)),
                static_cast<int>(std::floor(point.y / voxel_size_)),
                static_cast<int>(std::floor(point.z / voxel_size_)),
                label};

            if (points_.find(key) != points_.end()) {
                continue;
            }

            traj::MapPointXYZIL map_point;
            map_point.x = point.x;
            map_point.y = point.y;
            map_point.z = point.z;
            map_point.intensity = point.intensity;
            map_point.label = label;
            points_.insert({key, map_point});
        }
    }

    pcl::PointCloud<traj::MapPointXYZIL>::Ptr Snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        pcl::PointCloud<traj::MapPointXYZIL>::Ptr cloud(
            new pcl::PointCloud<traj::MapPointXYZIL>());
        cloud->points.reserve(points_.size());
        for (const auto& item : points_) {
            cloud->points.emplace_back(item.second);
        }
        cloud->width = static_cast<std::uint32_t>(cloud->points.size());
        cloud->height = 1;
        cloud->is_dense = false;
        return cloud;
    }

    bool SavePCD(const std::string& path, bool binary_compressed,
                 std::size_t* saved_points, std::string* error) const {
        const auto cloud = Snapshot();
        if (cloud->empty()) {
            if (error) *error = "global map cache is empty";
            return false;
        }

        const int status = binary_compressed
            ? pcl::io::savePCDFileBinaryCompressed(path, *cloud)
            : pcl::io::savePCDFileBinary(path, *cloud);
        if (status != 0) {
            if (error) *error = "PCL failed to write " + path;
            return false;
        }
        if (saved_points) *saved_points = cloud->size();
        return true;
    }

    std::size_t Size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return points_.size();
    }

 private:
    double voxel_size_ = 0.10;
    mutable std::mutex mutex_;
    tsl::robin_map<Key, traj::MapPointXYZIL, KeyHash> points_;
};

