#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <limits>

#include "yaml-cpp/yaml.h"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_interface/ec_pdo_channel_manager.hpp"
#include "ethercat_generic_plugins/generic_ec_cia402_drive.hpp"

namespace ethercat_generic_plugins
{

class MirrorIO : public EcCiA402Drive
{
public:
  MirrorIO();
  virtual ~MirrorIO();
  /** Returns true if drive has reached "operation enabled" state.
   *  The transition through the state machine is handled automatically. */
  virtual bool setupSlave(
    std::unordered_map<std::string, std::string> slave_paramters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface);

  virtual void processData(size_t index, uint8_t * domain_address);

private:
  std::unordered_map<int,int> cmd_to_state_;

};

}