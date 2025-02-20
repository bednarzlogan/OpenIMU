#include "IMU_Matrices.hpp"
#include "IMU.hpp"

IMU::IMU(std::string configs_path): _configuration_file_path(configs_path) { 
  // load in the system configurations
  std::ifstream inFile(configs_path);
  if (!inFile.is_open()) {
    std::cerr << "Could not open config file: " << configs_path << std::endl;
  }  
}
