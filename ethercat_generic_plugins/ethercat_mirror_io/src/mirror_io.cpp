#include <numeric>

#include "ethercat_generic_plugins/mirror_io.hpp"

using namespace ethercat_interface;

namespace ethercat_generic_plugins
{

MirrorIO::MirrorIO()
: EcCiA402Drive() {}
MirrorIO::~MirrorIO() {}


bool MirrorIO::setupSlave(
  std::unordered_map<std::string, std::string> slave_paramters,
  std::vector<double> * state_interface,
  std::vector<double> * command_interface)
{
  state_interface_ptr_ = state_interface;
  command_interface_ptr_ = command_interface;
  paramters_ = slave_paramters;

  if (paramters_.find("slave_config") != paramters_.end()) {
    if (!setup_from_config_file(paramters_["slave_config"])) {
      return false;
    }
  } else {
    std::cerr << "EcCiA402Drive: failed to find 'slave_config' tag in URDF." << std::endl;
    return false;
  }

  for(auto& c: pdo_channels_info_) {
    if(c.pdo_type == PdoType::RPDO) {
      if (paramters_.find("state_interface/" + c.interface_name) != paramters_.end() || 
          paramters_.find("command_interface/" + c.interface_name) != paramters_.end()) {
        cmd_to_state_[std::stoi(paramters_["command_interface/" + c.interface_name])] = 
                      std::stoi(paramters_["state_interface/" + c.interface_name]);
      }
    }
  }

  setup_interface_mapping();
  setup_syncs();

  if (paramters_.find("mode_of_operation") != paramters_.end()) {
    mode_of_operation_ = std::stod(paramters_["mode_of_operation"]);
  }

  if (paramters_.find("command_interface/reset_fault") != paramters_.end()) {
    fault_reset_command_interface_index_ = std::stoi(paramters_["command_interface/reset_fault"]);
  }

  return true;
}


void MirrorIO::processData(size_t index, uint8_t * domain_address)
{
  pdo_channels_info_[domain_map_[index]].ec_update(domain_address);
  if(cmd_to_state_.count(pdo_channels_info_[domain_map_[index]].interface_index)) {
    state_interface_ptr_->at(cmd_to_state_[pdo_channels_info_[domain_map_[index]].interface_index]) = pdo_channels_info_[domain_map_[index]].last_value;
  }
}
} // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::MirrorIO, ethercat_interface::EcSlave)
