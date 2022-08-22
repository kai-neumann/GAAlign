//
// Created by Kai on 20.04.2022.
//

#include "super4pcs_wrapper.h"
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

gaalign::Super4PCSWrapper::Super4PCSWrapper(std::string thirdPartyFolder) {
    m_folder = thirdPartyFolder;
    m_executable_path = thirdPartyFolder + "/Super4PCS.exe";
    std::cout << m_executable_path << std::endl;

    if(!boost::filesystem::exists(m_executable_path)) {
        std::cout << "Error: Failed to initialize the Wrapper for 'Super4PCS' because the executable was not found" << std::endl;
        exit(EXIT_FAILURE);
    }
}

gaalign::PointCloud gaalign::Super4PCSWrapper::calculateRegistration(const gaalign::PointCloud &source,
                                                                     const gaalign::PointCloud &target, double overlap) const {

    // Save both point clouds to plys
    source.saveToPly(m_folder + "/source.ply");
    target.saveToPly(m_folder + "/target.ply");

    // Execute the external executable
    std::string call = m_executable_path + " -i \"" + m_folder + "/source.ply\" \"" + m_folder + "/target.ply\" -o " + std::to_string(overlap) + " -d " +
            std::to_string(0.025*fmax(source.getMaxDimension(), target.getMaxDimension())) + " -n 5000 -t 500 -r \"" + m_folder + "/super4pcs_result.ply\" > \"" + m_folder + "/super4pcs.log\"";
    system(call.c_str());

    // Delete the point clouds
    boost::filesystem::remove(m_folder + "/source.ply");
    boost::filesystem::remove(m_folder + "/target.ply");

    // Loop over the lines of the log file
    std::ifstream file(m_folder + "/super4pcs.log");
    std::vector<double> values;

    if (file.is_open()) {
        std::string line;
        bool foundMatrix = false;
        int matrixLinesProcessed = 0;
        while (std::getline(file, line)) {
            if(!foundMatrix) {
                if(boost::starts_with(line, "(Homogeneous) Transformation")) {
                    std::cout << "Found the matrix!" << std::endl;
                    foundMatrix = true;
                }
                continue;
            }

            std::vector<std::string> splitted;
            boost::split(splitted,line,boost::is_any_of(" "));
            for(int i=0; i<splitted.size(); i++) {
                if(splitted[i].size() > 1) {
                    values.push_back(std::stod(splitted[i]));
                }
            }

            matrixLinesProcessed++;
            if(matrixLinesProcessed > 2) {
                break;
            }
        }
        file.close();
    }

    // Add the last row
    values.push_back(0);
    values.push_back(0);
    values.push_back(0);
    values.push_back(1);

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

    // Also clean up the results
    //boost::filesystem::remove(m_folder + "/super4pcs.log");
    boost::filesystem::remove(m_folder + "/super4pcs_result.ply");

    return result;
}

std::string gaalign::Super4PCSWrapper::getName() const {
    return "Super4PCS";
}
