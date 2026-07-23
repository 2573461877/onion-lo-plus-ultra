#pragma once
#include<iostream>
#include <cmath>
#include <omp.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 
#include <tbb/parallel_for.h>
#include <tbb/concurrent_hash_map.h>
#include <tbb/tbb.h>
#include <mutex>
#include <vector>
#include <functional> 
#include <unordered_map>
#include <algorithm>
#include <numeric>
#include <tsl/robin_map.h>
#include "nanoflann.hpp"
extern std::mutex print_mutex;
using namespace std;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector6dVector = std::vector<Vector6d>;
struct PointCloud6D {
    const Vector6dVector& pts;

    PointCloud6D(const Vector6dVector& points) : pts(points) {}

    inline size_t kdtree_get_point_count() const { return pts.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return pts[idx](dim);
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};

using KDTree6D = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud6D>,
    PointCloud6D,
    3
>;

inline Vector6dVector detectIntensityEdges(
    const Vector6dVector& points,
    int k = 6,
    double intensity_var_thresh = 150.0) 
{
    Vector6dVector output = points;
    PointCloud6D cloud(points);
    KDTree6D kdtree(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(20));
    kdtree.buildIndex();

    #pragma omp parallel for schedule(static)
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        const auto& query_pt = points[i];

        std::vector<size_t> neighbor_indices(k);
        std::vector<double> distances(k);
        nanoflann::KNNResultSet<double> resultSet(k);
        resultSet.init(neighbor_indices.data(), distances.data());
        kdtree.findNeighbors(resultSet, query_pt.data(), nanoflann::SearchParameters(k));
	    double intensity_sum = 0.0;
	    std::vector<double> intensities(k);
	    for (int j = 0; j < k; ++j) {
	        const auto& pt = points[neighbor_indices[j]];
	        intensities[j] = pt[4];
	        intensity_sum += pt[4];
	    }
	    double mean_intensity = intensity_sum / k;

	    double var = 0.0;
	    for (int j = 0; j < k; ++j) {
	        double diff = intensities[j] - mean_intensity;
	        var += diff * diff;
	    }
	    var /= k;

	    if (var > intensity_var_thresh) {
    		if (var>300 || output[i][4]>200 || output[i][4]<50)
    			output[i][5] = 1.0;
	    }
      
		if (output[i][5] == 0.0){
		    Eigen::Vector3f mean_point = Eigen::Vector3f::Zero();
		    Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero();

		    for (size_t j = 0; j < neighbor_indices.size(); ++j) {
		        const auto& pt = points[neighbor_indices[j]];
		        mean_point += Eigen::Vector3f(pt[0], pt[1], pt[2]);
		    }
		    mean_point /= neighbor_indices.size();

		    for (size_t j = 0; j < neighbor_indices.size(); ++j) {
		        const auto& pt = points[neighbor_indices[j]];
		        Eigen::Vector3f diff = Eigen::Vector3f(pt[0], pt[1], pt[2]) - mean_point;
		        covariance_matrix += diff * diff.transpose();
		    }
		    covariance_matrix /= neighbor_indices.size();

		    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix);
		    Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues();
		    float eps = 1e-6f;
		    float sum = eigenvalues.sum() + eps;
		    float e0 = eigenvalues[0]; 
		    float e1 = eigenvalues[1];
		    float e2 = eigenvalues[2];
		    if (e2 / e0 > 3 && e1 / e0 > 3) {
		        output[i][5] = 0.0;
		    } else {
		        output[i][5] = 3.0;
		    }
		}
    }

    return output;
}





class Onion {
private:
    double layer_thickness = 5;
    int seg_min_num = 6;
    int exp_key_num = 1200;
    double min_range = 0.0;
    double max_range = 300;
    double angle_v = 0.4;
    double angle_h = 0.01;
    double seg_angle = 0.4;
    //------------------------------Layer------------------------------------
	struct Layer {
		int face;
		int R;

		bool operator==(const Layer& other) const {
		    return face == other.face && R == other.R;
		}
	};

	struct LayerHash {
		std::size_t hash(const Layer& layer) const {
		    std::size_t h1 = std::hash<int>()(layer.face);
		    std::size_t h2 = std::hash<int>()(layer.R);
		    return h1 ^ (h2 << 1);
		}

		std::size_t operator()(const Layer& layer) const {
		    return hash(layer);
		}

		bool equal(const Layer& lhs, const Layer& rhs) const {
		    return lhs == rhs;
		}
	};

	struct LayerBlock {
		double volume = 0.0;
		double plane_num = 0.0;
		int seg_num = 0;

		Vector6dVector seg_points;
		Vector6dVector p_points;
		Vector6dVector np_points;
		Vector6dVector points;

		LayerBlock() = default;

		explicit LayerBlock(const Vector6d &point) {
		    AddPoint(point);
		}

		inline void AddPoint(const Vector6d &point) {
		    points.push_back(point);
		}

		void Merge(const LayerBlock& other) {
		    points.insert(points.end(), other.points.begin(), other.points.end());
		}

		void Merge(LayerBlock&& other) {
		    points.insert(points.end(),
		                  std::make_move_iterator(other.points.begin()),
		                  std::make_move_iterator(other.points.end()));
		}
	};
	//----------------------------Cell---------------------------------
	struct Cell {
		int x, y, z;
		bool operator==(const Cell& other) const {
		    return x == other.x && y == other.y && z == other.z;
		}
	};
	struct CellHash {
		std::size_t hash(const Cell& cell) const {
		    std::size_t hx = std::hash<int>()(cell.x);
		    std::size_t hy = std::hash<int>()(cell.y);
		    std::size_t hz = std::hash<int>()(cell.z);
		    return hx ^ (hy << 1) ^ (hz << 2);
		}

		bool equal(const Cell& a, const Cell& b) const {
		    return a == b;
		}
	};
	struct CellBlock {
		int num_points_ = 0;
		bool intensity_fea = 0;
		bool line_fea = 0;
		bool planar_fea = 0;
		Vector6dVector points;
		void AddPoint(const Vector6d &point) {
		    points.push_back(point);
		    num_points_++;
		}
		void Merge(const CellBlock& other) {
		    points.insert(points.end(), other.points.begin(), other.points.end());
		    num_points_ += other.num_points_;
		}
	};
	void thread_safe_print(const std::string& message1, const std::string& message2, const std::string& message3, const std::string& message4, const std::string& message5, const std::string& message6) {
		std::lock_guard<std::mutex> lock(print_mutex);
		std::cout <<message1<< message2<<message3<<message4<<message5<<message6<<std::endl;
	}
public:
	double Total_V;
	double Plane_rotio;
	int Total_NUM;
	int Raw_points_num;
	double Onion_Factor;
	double Onion_resolution;
	Vector6dVector Seg_points;
	tbb::concurrent_hash_map<Layer, LayerBlock, LayerHash> onion_ball;
	void Create_Onion(Vector6dVector PointCloud, double Resolution_v, double Resolution_h, double key_num) {
		exp_key_num = key_num;
		angle_v = Resolution_v;
		angle_h = Resolution_h;
		seg_angle = std::max(angle_v, angle_h);
		Raw_points_num = PointCloud.size();

		tbb::combinable<std::unordered_map<Layer, LayerBlock, LayerHash>> local_onion_balls;

		tbb::parallel_for(tbb::blocked_range<size_t>(0, PointCloud.size()), [&](const tbb::blocked_range<size_t>& r) {
		    auto& local_map = local_onion_balls.local();

		    for (size_t i = r.begin(); i < r.end(); ++i) {
		        const auto& point = PointCloud[i];
		        double x = point[0];
		        double y = point[1];
		        double z = point[2];
		        double dist = std::sqrt(x * x + y * y + z * z);
		        if (dist > max_range || dist < min_range)
		        	continue;
		        double theta = std::acos(z / dist); 
		        double phi = std::atan2(y, x); 

		        Layer layer;
		        layer.R = std::ceil(dist / layer_thickness);

		        if (theta <= M_PI / 4) {
		            layer.face = 1;  // +Z
		        } else if (theta >= 3 * M_PI / 4) {
		            layer.face = -1;  // -Z
		        } else if (phi >= -M_PI / 4 && phi <= M_PI / 4) {
		            layer.face = 2;  // +X
		        } else if (phi >= 3 * M_PI / 4 || phi <= -3 * M_PI / 4) {
		            layer.face = -2;  // -X
		        } else if (phi > M_PI / 4 && phi < 3 * M_PI / 4) {
		            layer.face = 3;  // +Y
		        } else {
		            layer.face = -3;  // -Y
		        }

		        auto it = local_map.find(layer);
		        if (it == local_map.end()) {
		            local_map[layer] = LayerBlock(point);
		        } else {
		            it->second.AddPoint(point);
		        }
		    }
		});

		local_onion_balls.combine_each([&](std::unordered_map<Layer, LayerBlock, LayerHash>& local_map) {
		    for (auto& pair : local_map) {
		        tbb::concurrent_hash_map<Layer, LayerBlock, LayerHash>::accessor accessor;
		        if (onion_ball.insert(accessor, pair.first)) {
		            accessor->second = std::move(pair.second);
		        } else {
		            accessor->second.Merge(std::move(pair.second));
		        }
		    }
		});
	}

void IG_Classifier(const Vector6dVector& input_point,
                   double voxelSize,
                   double v_size,
                   Vector6dVector& p_point,
                   Vector6dVector& np_point,
                   double &plane_rate,
                   double &V,
                   Vector6dVector& all_points) 
{
    double plane_num = 0;
    double no_plane_num = 0;

    p_point.reserve(input_point.size());
    all_points.reserve(input_point.size());
    np_point.reserve(input_point.size());

    tbb::concurrent_hash_map<Cell, CellBlock, CellHash> segVoxels;
    tbb::concurrent_hash_map<Cell, CellBlock, CellHash> fixedVoxels;

    for (const auto& p : input_point) {
        Cell cell_key{
            static_cast<int>(std::floor(p[0] / voxelSize)),
            static_cast<int>(std::floor(p[1] / voxelSize)),
            static_cast<int>(std::floor(p[2] / voxelSize))
        };
        tbb::concurrent_hash_map<Cell, CellBlock, CellHash>::accessor accessor;
        if (segVoxels.insert(accessor, cell_key)) {
            accessor->second = CellBlock();
        }
        accessor->second.AddPoint(p);
        Cell fixed_cell_key{
            static_cast<int>(std::floor(p[0] / v_size)),
            static_cast<int>(std::floor(p[1] / v_size)),
            static_cast<int>(std::floor(p[2] / v_size))
        };
        tbb::concurrent_hash_map<Cell, CellBlock, CellHash>::accessor fixed_accessor;
        if (fixedVoxels.insert(fixed_accessor, fixed_cell_key)) {
            fixed_accessor->second = CellBlock();
        }
        fixed_accessor->second.AddPoint(p);
    }

    int occupiedVoxelCount = 0;
    for (auto it = fixedVoxels.begin(); it != fixedVoxels.end(); ++it) {
        if (it->second.num_points_ > 5) {
            occupiedVoxelCount++;
        }
    }
    double computedVolume = occupiedVoxelCount * std::pow(v_size, 3);

    double NUM = 0;
    double p_NUM = 0;
    double np_NUM = 0;
    double use_points = 0;

    for (auto& [cell_key, cell_block] : segVoxels) {
        double point_count = cell_block.num_points_;

        if (point_count >= seg_min_num) {
            NUM++;
            use_points += point_count;

            Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
            for (const auto& point : cell_block.points) {
                centroid[0] += point[0];
                centroid[1] += point[1];
                centroid[2] += point[2];
            }
            centroid /= static_cast<float>(point_count);

            Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero();
            for (const auto& point : cell_block.points) {
                Eigen::Vector3f p(point[0], point[1], point[2]);
                covariance_matrix += (p - centroid) * (p - centroid).transpose();
            }
            covariance_matrix /= static_cast<float>(point_count);

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix);
            Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues();
            float eps = 1e-6f;
            float sum = eigenvalues.sum() + eps;
            float e0 = eigenvalues[0] / sum;
            float e1 = eigenvalues[1] / sum;
            float e2 = eigenvalues[2] / sum;
            bool isPlane = e2 / e0 > 10 && e1 / e0 > 10;
            if (isPlane) {
                p_NUM++;
                auto intencity = detectIntensityEdges(cell_block.points);
                for (auto& p : intencity) {
                    plane_num++;
                    p_point.push_back(p);
                    all_points.push_back(p);
                }
            } else {
                np_NUM++;
                for (auto& p : cell_block.points) {
                    p[5] = 2.0;
                    no_plane_num++;
                    np_point.push_back(p);
                    all_points.push_back(p);
                }
            }
        } else {
            for (auto& p : cell_block.points) {
                p[5] = 0.0;
                plane_num++;
                p_point.push_back(p);
                all_points.push_back(p);
            }
        }
    }

    if (use_points > 0) {
        plane_rate = p_NUM / use_points;
        V = computedVolume;
    }
}


//-----------------------------------Classifier--------------------------------
Vector6dVector Classifier() {
    Seg_points.reserve(Raw_points_num);
    double total_volume = 0.0;
    double total_num = 0.0;
    double plane_num = 0.0;
	double Layer_size = 0.0;
	Layer_size=onion_ball.size();
    std::mutex mutex;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, onion_ball.size()), [&](const tbb::blocked_range<size_t>& range) {
        Vector6dVector local_points_thread;
        double local_total_num = 0.0;
        double local_plane_num = 0.0;
        double local_total_volume = 0.0;
        for (size_t i = range.begin(); i < range.end(); ++i) {
            auto it = onion_ball.begin();
            std::advance(it, i);
            const auto& layer = it->first;
            auto& block = it->second;
            if (layer.R >= 1.0 && layer.R < 50.0 && block.points.size() > 10) {
                double R = layer.R * layer_thickness;
                double class_size = std::round(R * std::sin(3.0 *seg_angle * M_PI / 180.0) * 10.0) / 10.0;
                double v_size = std::round(R * std::sin(3.0 * M_PI / 180.0) * 10.0) / 10.0;
                Vector6dVector plane_points_local;
                Vector6dVector no_plane_points_local;
                Vector6dVector total_points;
                double volume = 0.0;
                double plane_rate_local = 0.0;
                IG_Classifier(block.points, class_size, v_size, plane_points_local, 
                				no_plane_points_local, plane_rate_local, volume, total_points);
                block.seg_points = total_points;
                block.p_points = plane_points_local;
                block.np_points = no_plane_points_local;
                block.seg_num = total_points.size();
                block.plane_num = plane_points_local.size();
                block.volume = volume;
                local_points_thread.insert(local_points_thread.end(), total_points.begin(), total_points.end());
                local_total_num += block.seg_num;
                local_plane_num += block.plane_num;
                local_total_volume += block.volume;
                /*
                thread_safe_print(
                    "layer.Face " + std::to_string(layer.face),
                    "  layer.R " + std::to_string(layer.R),
                    "  v_size " + std::to_string(v_size),
                    "  class_size " + std::to_string(class_size),
                    "  block.volume " + std::to_string(block.volume),
                    "  block.seg_num " + std::to_string(block.seg_num)
                );
                */
            }
        }  
        std::lock_guard<std::mutex> lock(mutex);
        Seg_points.insert(Seg_points.end(), local_points_thread.begin(), local_points_thread.end());
        total_num += local_total_num;
        plane_num += local_plane_num;
        total_volume += local_total_volume;
    });
    Total_NUM = Seg_points.size();
    Plane_rotio = static_cast<double>(plane_num) / Total_NUM;
    Total_V = total_volume;
    Onion_Factor = std::pow(total_volume/exp_key_num, 1.0/3.0);
    if (Onion_Factor<0.1) Onion_Factor=0.1;
    return Seg_points;
}

using Vector3i = Eigen::Matrix<int, 3, 1>;
using Voxel = Vector3i;
struct VoxelHash {
    size_t operator()(const Voxel &voxel) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
};

void clear(){
	Seg_points.clear();
	onion_ball.clear();
}
};
