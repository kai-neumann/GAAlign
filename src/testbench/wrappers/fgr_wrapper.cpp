//
// Created by Kai on 20.04.2022.
//

#include "fgr_wrapper.h"
#include <boost/filesystem.hpp>
#include <pcl/features/fpfh_omp.h>
#include "fstream"
#include <boost/algorithm/string.hpp>

gaalign::FGRWrapper::FGRWrapper(std::string thirdPartyFolder) {
    m_folder = thirdPartyFolder;
    m_executable_path = thirdPartyFolder + "/FastGlobalRegistration.exe";
    std::cout << m_executable_path << std::endl;

    if(!boost::filesystem::exists(m_executable_path)) {
        std::cout << "Error: Failed to initialize the Wrapper for 'FastGlobalRegistration (FGR)' because the executable was not found" << std::endl;
        exit(EXIT_FAILURE);
    }
}

gaalign::PointCloud
gaalign::FGRWrapper::calculateRegistration(const gaalign::PointCloud &source, const gaalign::PointCloud &target, double overlap) const {
    std::cout << "Exporting point clouds.." << std::endl;

    // Export both point clouds to binary files
    exportPointCloud(source, m_folder + "/source.bin");
    exportPointCloud(target, m_folder + "/target.bin");

    // Execute the external executable
    std::string call = m_executable_path + " \"" + m_folder + "/target.bin\" \"" + m_folder + "/source.bin\" \"" + m_folder + "/transform.txt\"";
    system(call.c_str());

    // Delete the point clouds
    boost::filesystem::remove(m_folder + "/source.bin");
    boost::filesystem::remove(m_folder + "/target.bin");

    // Load the transform
    std::ifstream file(m_folder + "/transform.txt");
    std::vector<double> values;

    if (file.is_open()) {
        std::string line;
        int id = -1;
        while (std::getline(file, line)) {
            if(id == -1) {
                id++;
                continue;
            }

            std::vector<std::string> splitted;
            boost::split(splitted,line,boost::is_any_of(" "));
            for(int i=0; i<splitted.size(); i++) {
                //std::cout << "[" << id << "," << i << "]: " << splitted[i] << " -> " << std::stod(splitted[i]) << std::endl;
                values.push_back(std::stod(splitted[i]));
                //std::cout << mat(id, i) << std::endl;
            }
        }
        file.close();
    }

    // Unpack
    Eigen::Matrix4d mat;
    for(int i=0; i<values.size(); i++) {
        mat(i / 4, i%4) = values[i];;
    }

    // Apply the matrix to all points in the point cloud
    gaalign::PointCloud result = source;
    #pragma omp parallel for
    for(int i=0; i<source.size(); i++) {
        // Apply transformation
        Eigen::Vector4d point = mat * Eigen::Vector4d(source.getPoint(i).x(), source.getPoint(i).y(), source.getPoint(i).z(), 1);
        Eigen::Vector4d normal = mat * Eigen::Vector4d(source.getNormal(i).x(), source.getNormal(i).y(), source.getNormal(i).z(), 1);

        // Set transformed point / normal to point cloud
        result.setPoint(i, Eigen::Vector3d(point.x(), point.y(), point.z()));
        result.setNormal(i, Eigen::Vector3d(normal.x(), normal.y(), normal.z()));
    }

    // Also clean up the transform.txt
    boost::filesystem::remove(m_folder + "/transform.txt");

    return result;
}

std::string gaalign::FGRWrapper::getName() const {
    return "FGR";
}

void gaalign::FGRWrapper::exportPointCloud(const gaalign::PointCloud &pointCloud, const std::string &path) const {
    pcl::PointCloud<pcl::PointNormal>::Ptr pointsFilteredPtr = pointCloud.toPCL();

    pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features(new pcl::PointCloud<pcl::FPFHSignature33>());
    fest.setRadiusSearch(0.05*pointCloud.getMaxDimension());
    fest.setInputCloud(pointsFilteredPtr);
    fest.setInputNormals(pointsFilteredPtr);
    fest.compute(*object_features);

    FILE* fid = fopen(path.c_str(), "wb");
    int nV = pointsFilteredPtr->size(), nDim = 33;
    fwrite(&nV, sizeof(int), 1, fid);
    fwrite(&nDim, sizeof(int), 1, fid);
    for (int v = 0; v < nV; v++) {
        const pcl::PointNormal &pt = pointsFilteredPtr->points[v];
        float xyz[3] = {pt.x, pt.y, pt.z};
        fwrite(xyz, sizeof(float), 3, fid);
        const pcl::FPFHSignature33 &feature = object_features->points[v];
        fwrite(feature.histogram, sizeof(float), 33, fid);
    }
    fclose(fid);
}
