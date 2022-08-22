#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <geometry/motor.h>

#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/random_sample.h>

namespace gaalign {
    class PointCloud {
    public:
        explicit PointCloud() = default;

        explicit PointCloud(std::string filePath);

        std::string getName() const;

        Eigen::Vector3d getPoint(const int &index) const;

        Eigen::Vector3d getNormal(const int &index) const;

        int getIndex(const int &index) const;

        void setPoint(const int &index, const Eigen::Vector3d &point);

        void setNormal(const int &index, const Eigen::Vector3d &normal);

        void addPoint(const Eigen::Vector3d &point);

        void addNormal(const Eigen::Vector3d &normal);

        void addIndex(const int &id);

        const int size() const;

        void applyMotor(const Motor &motor);

        Eigen::Vector3d getDimensions() const;

        double getMaxDimension() const;

        PointCloud downsample(double subSampleCellSize) const;

        PointCloud subsample(int count) const;

        pcl::PointCloud<pcl::PointNormal>::Ptr toPCL() const;

        pcl::PointCloud<pcl::PointXYZLNormal>::Ptr toPCLWithIndices() const;

        void saveToPly(const std::string &path) const;

        void resetBoundingBox();

    private:
        std::string m_name;

        std::vector<Eigen::Vector3d> m_points;
        std::vector<Eigen::Vector3d> m_normals;
        std::vector<int> m_indices;

        Eigen::Vector3d boundsMin;
        Eigen::Vector3d boundsMax;

        void updateBoundingBox(const Eigen::Vector3d &point);
    };
}
