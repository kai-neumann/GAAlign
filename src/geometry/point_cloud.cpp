#include "point_cloud.h"
#include "iostream"
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <happly/happly.h>

gaalign::PointCloud::PointCloud(std::string filePath) {
    std::cout << "Loading point cloud from " << filePath << std::endl;

    // Check if file exists
    if(!boost::filesystem::exists(filePath)) {
        std::cout << "ERROR: The given point cloud does not exist!" << std::endl;
    }

    // Extract the name
    m_name = boost::filesystem::path(filePath).filename().string();

    // Clear the internal points
    m_points.clear();
    m_normals.clear();
    m_indices.clear();

    // Construct the data object by reading from file
    happly::PLYData plyIn(filePath);

    // Get the positions
    std::vector<double> posX = plyIn.getElement("vertex").getProperty<double>("x");
    std::vector<double> posY = plyIn.getElement("vertex").getProperty<double>("y");
    std::vector<double> posZ = plyIn.getElement("vertex").getProperty<double>("z");

    // Check if the model has normals
    bool foundNx = false;
    bool foundNy = false;
    bool foundNz = false;
    for(const auto& name : plyIn.getElement("vertex").getPropertyNames()) {
        foundNx |= name == "nx";
        foundNy |= name == "ny";
        foundNz |= name == "nz";
    }
    bool hasNormals = foundNx && foundNy && foundNz;

    // Get the normals
    std::vector<double> normX;
    std::vector<double> normY;
    std::vector<double> normZ;
    if(hasNormals) {
        normX = plyIn.getElement("vertex").getProperty<double>("nx");
        normY = plyIn.getElement("vertex").getProperty<double>("ny");
        normZ = plyIn.getElement("vertex").getProperty<double>("nz");
    }

    // Convert to output data structure
    m_points.resize(posX.size());
    m_normals.resize(normX.size());
    m_indices.resize(normX.size());

    #pragma omp parallel for
    for(int i=0; i<m_points.size(); i++) {
        // Convert the points
        m_points[i] = Eigen::Vector3d(posX[i], posY[i], posZ[i]);
        m_indices[i] = i;

        // Convert the normals, if they exists
        if(hasNormals) {
            m_normals[i] = Eigen::Vector3d(normX[i], normY[i], normZ[i]);
        }
    }

    // Recalculate the bounding box
    resetBoundingBox();
}

Eigen::Vector3d gaalign::PointCloud::getPoint(const int &index) const {
    return m_points[index];
}

const int gaalign::PointCloud::size() const {
    return m_points.size();
}

void gaalign::PointCloud::applyMotor(const Motor &motor) {
    #pragma omp parallel for
    for(int i=0; i<m_points.size(); i++) {
        m_points[i] = transformPointWithMotor(m_points[i], motor);
    }
}

Eigen::Vector3d gaalign::PointCloud::getDimensions() const{
    return boundsMax - boundsMin;
}

void gaalign::PointCloud::setPoint(const int &index, const Eigen::Vector3d &point) {
    m_points[index] = point;

    // Also update the bounding box
    updateBoundingBox(point);
}

void gaalign::PointCloud::addPoint(const Eigen::Vector3d &point) {
    // Add to list of points
    m_points.push_back(point);

    // Also update the bounding box
    updateBoundingBox(point);
}

void gaalign::PointCloud::updateBoundingBox(const Eigen::Vector3d &point) {
    if(point.x() < boundsMin.x()) boundsMin.x() = point.x();
    if(point.y() < boundsMin.y()) boundsMin.y() = point.y();
    if(point.z() < boundsMin.z()) boundsMin.z() = point.z();
    if(point.x() > boundsMax.x()) boundsMax.x() = point.x();
    if(point.y() > boundsMax.y()) boundsMax.y() = point.y();
    if(point.z() > boundsMax.z()) boundsMax.z() = point.z();
}

Eigen::Vector3d gaalign::PointCloud::getNormal(const int &index) const {
    return m_normals[index];
}

double gaalign::PointCloud::getMaxDimension() const {
    Eigen::Vector3d dim = getDimensions();
    return fmax(dim.x(), fmax(dim.y(), dim.z()));
}

void gaalign::PointCloud::setNormal(const int &index, const Eigen::Vector3d &point) {
    m_normals[index] = point;
}

void gaalign::PointCloud::addNormal(const Eigen::Vector3d &normal) {
    m_normals.push_back(normal);
}



gaalign::PointCloud gaalign::PointCloud::downsample(double subSampleCellSize) const{
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr inputPtr = toPCLWithIndices();

    // Subsample the point cloud
    //std::cout << "Subsampling Point cloud.."<< std::endl;
    pcl::PointCloud<pcl::PointXYZLNormal> filtered;
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr pointsFilteredPtr = filtered.makeShared();

    pcl::UniformSampling<pcl::PointXYZLNormal> uniform_sampling(true);
    uniform_sampling.setInputCloud (inputPtr);
    uniform_sampling.setRadiusSearch (subSampleCellSize);
    uniform_sampling.filter (*pointsFilteredPtr);

    //std::cout << "Subsampled to " << pointsFilteredPtr->size() << " Points (out of " << m_points.size() << " points)" << std::endl;

    // Convert to new point cloud
    PointCloud out;
    for(auto & i : *pointsFilteredPtr) {
        out.addPoint(Eigen::Vector3d(i.x, i.y, i.z));
        out.addNormal(Eigen::Vector3d(i.normal_x, i.normal_y, i.normal_z));
        out.addIndex(i.label);
    }

    // Also keep the name
    out.m_name = m_name;

    return out;
}

gaalign::PointCloud gaalign::PointCloud::subsample(int count) const {
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr inputPtr = toPCLWithIndices();

    // Subsample the point cloud
    pcl::PointCloud<pcl::PointXYZLNormal> filtered;
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr pointsFilteredPtr = filtered.makeShared();

    pcl::RandomSample<pcl::PointXYZLNormal> randomSample(true);
    randomSample.setSample(count);
    randomSample.setInputCloud(inputPtr);
    randomSample.filter(*pointsFilteredPtr);

    // Convert to new point cloud
    PointCloud out;
    for(auto & i : *pointsFilteredPtr) {
        out.addPoint(Eigen::Vector3d(i.x, i.y, i.z));
        out.addNormal(Eigen::Vector3d(i.normal_x, i.normal_y, i.normal_z));
        out.addIndex(i.label);
    }

    // Also keep the name
    out.m_name = m_name;

    return out;
}

pcl::PointCloud<pcl::PointNormal>::Ptr gaalign::PointCloud::toPCL() const {
    // Convert the point cloud and normals into a PCL point cloud
    pcl::PointCloud<pcl::PointNormal> input;

    // Create pointer (this is done now to avoid costly copying later on)
    pcl::PointCloud<pcl::PointNormal>::Ptr inputPtr = input.makeShared();

    // Resize to correct size
    inputPtr->resize(m_points.size());

    #pragma omp parallel for
    for(int i=0; i<m_points.size(); i++) {
        pcl::PointNormal pn;
        pn.x = m_points[i].x();
        pn.y = m_points[i].y();
        pn.z = m_points[i].z();
        if(!m_normals.empty()) {
            pn.normal_x = m_normals[i].x();
            pn.normal_y = m_normals[i].y();
            pn.normal_z = m_normals[i].z();
        }
        inputPtr->at(i) = pn;
    }

    return inputPtr;
}

void gaalign::PointCloud::saveToPly(const std::string& path) const {
    // Save using happly
    // Remove the old file, if it exists
    if (boost::filesystem::exists(path)) {
        boost::filesystem::remove(path);
    }

    // Convert the dense point cloud into the format needed by happly
    std::vector<std::array<double, 3>> vertexPositions;
    std::vector<double> nx;
    std::vector<double> ny;
    std::vector<double> nz;

    // Resize all to the correct size
    vertexPositions.resize(m_points.size());
    nx.resize(m_points.size());
    ny.resize(m_points.size());
    nz.resize(m_points.size());

    // Convert
    #pragma omp parallel for
    for(int i=0; i<m_points.size(); i++) {
        // Position
        vertexPositions[i][0] = m_points[i].x();
        vertexPositions[i][1] = m_points[i].y();
        vertexPositions[i][2] = m_points[i].z();

        // Normals
        if(!m_normals.empty()) {
            nx[i] = m_normals[i].x();
            ny[i] = m_normals[i].y();
            nz[i] = m_normals[i].z();
        }
    }

    // Create an empty object
    happly::PLYData plyOut;

    // Add vertices and colors
    plyOut.addVertexPositions(vertexPositions);

    // Add confidence
    plyOut.getElement("vertex").addProperty<double>("nx", nx);
    plyOut.getElement("vertex").addProperty<double>("ny", ny);
    plyOut.getElement("vertex").addProperty<double>("nz", nz);

    // Write the object to file
    plyOut.write(path, happly::DataFormat::Binary);
}

pcl::PointCloud<pcl::PointXYZLNormal>::Ptr gaalign::PointCloud::toPCLWithIndices() const {
    // Convert the point cloud and normals into a PCL point cloud
    pcl::PointCloud<pcl::PointXYZLNormal> input;

    // Create pointer (this is done now to avoid costly copying later on)
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr inputPtr = input.makeShared();

    // Resize to correct size
    inputPtr->resize(m_points.size());

    #pragma omp parallel for
    for(int i=0; i<m_points.size(); i++) {
        pcl::PointXYZLNormal pn;
        pn.x = m_points[i].x();
        pn.y = m_points[i].y();
        pn.z = m_points[i].z();
        pn.normal_x = m_normals[i].x();
        pn.normal_y = m_normals[i].y();
        pn.normal_z = m_normals[i].z();
        pn.label = m_indices[i];
        inputPtr->at(i) = pn;
    }

    return inputPtr;
}

void gaalign::PointCloud::addIndex(const int &id) {
    m_indices.push_back(id);
}

int gaalign::PointCloud::getIndex(const int &index) const {
    return m_indices[index];
}

void gaalign::PointCloud::resetBoundingBox() {
    // Intialize the bounds with large values
    boundsMin = Eigen::Vector3d(100000, 100000, 100000);
    boundsMax = Eigen::Vector3d(-100000, -100000, -100000);

    for(int i=0; i<m_points.size(); i++) {
        updateBoundingBox(m_points[i]);
    }
}

std::string gaalign::PointCloud::getName() const {
    return m_name;
}






