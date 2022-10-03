#include <iostream>
#include <string>
#include <random>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <registration/registration_pipeline.h>
#include <registration/registration_step.h>
#include <optimization/gradient_descent/gradient_descent.h>
#include <correspondence/feature/feature_search.h>
#include <correspondence/distance/distance_search.h>

#ifdef ENABLE_CUDA
#include <optimization/gradient_descent_cuda/gradient_descent_cuda.h>
#endif

int main(int argc, char** argv) {
    // Declare additional supported options.
    std::string sourcePath, targetPath;
    int downscaleFactor = 1;
    boost::program_options::options_description desc("GAAlign: Geometric Algebra based Alignment \n\nUsage:\tgaalign.exe [sourcePath point cloud] [targetPath point cloud] --option1 value1");
    desc.add_options()
            ("help", "produce help message")
            ("source", boost::program_options::value<std::string>(&sourcePath), "The point cloud that needs to be aligned")
            ("target", boost::program_options::value<std::string>(&targetPath), "The targetPath point cloud that the other cloud is aligned to")
            ;

    boost::program_options::positional_options_description p;
    p.add("source", 1);
    p.add("target", 1);

    boost::program_options::variables_map vm;
    try {
        boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
        boost::program_options::notify(vm);
    }
    catch(std::exception& e){
        std::cerr << "Error: Invalid program arguments!" << std::endl << std::endl;
        std::cout << desc << "\n";
        return EXIT_FAILURE;
    }

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    // Check if a sourcePath point cloud was specified
    if(!vm.count("source")) {
        std::cerr << "Error: Please specify a sourcePath point cloud!\n" << std::endl;
        std::cerr << desc << "\n";
        return EXIT_FAILURE;
    }

    // Check if a targetPath point cloud was specified
    if(!vm.count("target")) {
        std::cerr << "Error: Please specify a targetPath point cloud!\n" << std::endl;
        std::cerr << desc << "\n";
        return EXIT_FAILURE;
    }

    // Check if the sourcePath point cloud exists
    if(!boost::filesystem::exists(sourcePath) || !boost::filesystem::is_regular_file(sourcePath)) {
        std::cerr << "Error: The specifed sourcePath point cloud is invalid or does not exist!\n" << std::endl;
        return EXIT_FAILURE;
    }

    // Check if the targetPath point cloud exists
    if(!boost::filesystem::exists(targetPath) || !boost::filesystem::is_regular_file(targetPath)) {
        std::cerr << "Error: The specifed targetPath point cloud is invalid or does not exist!\n" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Aligning '" << boost::filesystem::path(sourcePath).filename().string() << "' to '"
              << boost::filesystem::path(targetPath).filename().string() << "'" << std::endl;

    // Read both point clouds
    gaalign::PointCloud source(sourcePath); // The point cloud that gets aligned
    gaalign::PointCloud target(targetPath); // The point cloud that is used as targetPath

    // For testing: Slightly perturb the source
    /*std::cout << "Warning: Input SOURCE Point cloud is getting DISTURBED for testing purposes." << std::endl;
    std::normal_distribution<double> distribution(0, 0.005);
    std::mt19937 re((std::random_device())());
    for(int i=0; i<source.size(); i++) {
        Eigen::Vector3d p = source.getPoint(i);
        source.setPoint(i, Eigen::Vector3d(p.x() + distribution(re), p.y() + distribution(re), p.z() + distribution(re)));
        Eigen::Vector3d n = source.getNormal(i);
        source.setNormal(i, Eigen::Vector3d(n.x() + distribution(re), n.y() + distribution(re), n.z() + distribution(re)));
    }
    for(int i=0; i<target.size(); i++) {
        Eigen::Vector3d p = target.getPoint(i);
        target.setPoint(i, Eigen::Vector3d(p.x() + distribution(re), p.y() + distribution(re), p.z() + distribution(re)));
        Eigen::Vector3d n = target.getNormal(i);
        target.setNormal(i, Eigen::Vector3d(n.x() + distribution(re), n.y() + distribution(re), n.z() + distribution(re)));
    }*/

    // Initialize the Pipeline
    gaalign::RegistrationPipeline pipeline;
    pipeline.getSettings().verbose = true;
    pipeline.getSettings().visualizeResult = true;

    // -----------------------------------------------------------------------------------------------------------------

    // Add the coarse alignment based on feature-based correspondences and a gradient descent
    gaalign::RegistrationStep coarse;

    std::shared_ptr<gaalign::FeatureSearch> featureSearch = std::make_shared<gaalign::FeatureSearch>();
    featureSearch->getSettings().visualizeMatches = false;
    coarse.setCorrespondenceSearch(featureSearch);

    std::shared_ptr<gaalign::GradientDescentOptimizer> optimCoarse = std::make_shared<gaalign::GradientDescentOptimizer>();
    //std::shared_ptr<gaalign::GradientDescentOptimizerCUDA> optimCoarse = std::make_shared<gaalign::GradientDescentOptimizerCUDA>();
    optimCoarse->getSettings().verbose = false;
    optimCoarse->getSettings().printTiming = true;
    coarse.setOptimizer(optimCoarse);

    pipeline.addStep(coarse);

    // -----------------------------------------------------------------------------------------------------------------

    // Add the fine alignment that uses distance-based correspondences and a gradient descent
    gaalign::RegistrationStep fine;
    fine.setRepetitions(5);

    std::shared_ptr<gaalign::DistanceSearch> distanceSearch = std::make_shared<gaalign::DistanceSearch>();
    // TODO: Settings
    fine.setCorrespondenceSearch(distanceSearch);

    std::shared_ptr<gaalign::GradientDescentOptimizer> optimFine = std::make_shared<gaalign::GradientDescentOptimizer>();
    //std::shared_ptr<gaalign::GradientDescentOptimizerCUDA> optimFine = std::make_shared<gaalign::GradientDescentOptimizerCUDA>();
    optimFine->getSettings().printTiming = true;
    fine.setOptimizer(optimFine);

    pipeline.addStep(fine);

    // -----------------------------------------------------------------------------------------------------------------

    // IMPORTANT: If we are using cuda: We need to initialize both optimizers before the actual runtime
    //optimCoarse->init();
    //optimFine->init();


    // Run the pipeline
    pipeline.run(source, target);


    return EXIT_SUCCESS;
}

