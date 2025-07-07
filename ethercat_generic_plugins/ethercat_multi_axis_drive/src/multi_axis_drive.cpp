// Copyright 2023 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)

#include <numeric>

#include "ethercat_generic_plugins/multi_axis_drive.hpp"

namespace ethercat_generic_plugins
{

ECMutilAxisDrive::ECMutilAxisDrive()
: GenericEcSlave() {}
ECMutilAxisDrive::~ECMutilAxisDrive() {}

bool ECMutilAxisDrive::initialized() const {return initialized_;}

void ECMutilAxisDrive::processData(size_t index, uint8_t * domain_address)
{
  // Special case: ControlWord
  if ((pdo_channels_info_[index].index & 0xF0FF) == CiA402D_RPDO_CONTROLWORD) {
    if (is_operational_) {
      if (fault_reset_command_interface_index_ >= 0) {
        if (command_interface_ptr_->at(fault_reset_command_interface_index_) == 0) {
          last_fault_reset_command_ = false;
        }
        if (last_fault_reset_command_ == false &&
          command_interface_ptr_->at(fault_reset_command_interface_index_) != 0 &&
          !std::isnan(command_interface_ptr_->at(fault_reset_command_interface_index_)))
        {
          last_fault_reset_command_ = true;
          fault_reset_ = true;
        }
      }

      if (auto_state_transitions_) {
        pdo_channels_info_[index].default_value = transition(
          state_,
          pdo_channels_info_[index].ec_read(domain_address));
          // std::cout << "Axis index " << axis_index_real_ << ": Sending control_word: " << std::hex <<  (int)pdo_channels_info_[index].default_value << std::dec << std::endl;
          // std::flush(std::cout);
      }
    }
  }

  // setup current position as default position
  if ((pdo_channels_info_[index].index & 0xF0FF) == CiA402D_RPDO_POSITION){
    if (mode_of_operation_display_ != ModeOfOperation::MODE_NO_MODE) {
      pdo_channels_info_[index].default_value =
        pdo_channels_info_[index].factor * last_position_ +
        pdo_channels_info_[index].offset;
    }
    pdo_channels_info_[index].override_command =
      (mode_of_operation_display_ != ModeOfOperation::MODE_CYCLIC_SYNC_POSITION) ? true : false;
  }

  // setup mode of operation
  if ((pdo_channels_info_[index].index & 0xF0FF) == CiA402D_RPDO_MODE_OF_OPERATION) {
    if (mode_of_operation_ >= 0 && mode_of_operation_ <= 10) {
      pdo_channels_info_[index].default_value = mode_of_operation_;
    }
    // std::cout << "Axis index " << axis_indexes_[index] << ": Setting mode_of_operation: " << (int)mode_of_operation_ << std::endl;
    // std::flush(std::cout);
  }
  pdo_channels_info_[index].ec_update(domain_address);

  // get mode_of_operation_display_
  if ((pdo_channels_info_[index].index & 0xF0FF) == CiA402D_TPDO_MODE_OF_OPERATION_DISPLAY) {
    mode_of_operation_display_ = pdo_channels_info_[index].last_value;
  }

  if ((pdo_channels_info_[index].index & 0xF0FF) == CiA402D_TPDO_POSITION) {
    last_position_ = pdo_channels_info_[index].last_value;
  }

  // Special case: StatusWord
  if ((pdo_channels_info_[index].index & 0xF0FF) == CiA402D_TPDO_STATUSWORD) {
    status_word_ = pdo_channels_info_[index].last_value;
    if (status_word_ != last_status_word_) {
      state_ = deviceState(status_word_);
      if (state_ != last_state_) {
        std::cout << "Axis " << axis_index_real_
                  << " STATE: " << DEVICE_STATE_STR.at(state_)
                  << " with status word :" << std::hex << status_word_ << std::dec << std::endl;
        std::flush(std::cout);
      }
    }
    last_status_word_ = status_word_;
    last_state_ = state_;
    if (state_ == DeviceState::STATE_OPERATION_ENABLED && last_state_ == DeviceState::STATE_OPERATION_ENABLED)
    {
      initialized_ = true;
    } else {
      initialized_ = false;
    }
    counter_++;
    // std::cout << "Axis index " << axis_index_real_ << ": Getting status_word: " << std::hex << status_word_ << std::dec << std::endl;
    // std::flush(std::cout);
  }

}

// void ECMutilAxisDrive::domains(DomainMap & domains) const{
//   domains = {{axis_index_real_, domain_map_}};
// }

bool ECMutilAxisDrive::setupSlave(
  std::unordered_map<std::string, std::string> slave_paramters,
  std::vector<double> * state_interface,
  std::vector<double> * command_interface)
{
  state_interface_ptr_ = state_interface;
  command_interface_ptr_ = command_interface;
  paramters_ = slave_paramters;

  axis_index_real_ = std::stoi(paramters_["axis"]); // This must be determinated by hardware wiring. Defined in ros2_control.xacro file.

  if (paramters_.find("slave_config") != paramters_.end()) {
    if (!setup_from_config_file(paramters_["slave_config"])) {
      return false;
    }
  } else {
    std::cerr << "ECMutilAxisDrive: failed to find 'slave_config' tag in URDF." << std::endl;
    return false;
  }

  // setup interface mapping
  for (auto i = 0ul; i < pdo_channels_info_.size(); i++) {
    auto& channel = pdo_channels_info_[i];
    if (channel.pdo_type == ethercat_interface::TPDO) {
      if (paramters_.find("state_interface/" + channel.interface_name) != paramters_.end()) {
        channel.interface_index =
          std::stoi(paramters_["state_interface/" + channel.interface_name]);
      }
    }
    if (channel.pdo_type == ethercat_interface::RPDO) {
      if (paramters_.find("command_interface/" + channel.interface_name) != paramters_.end()) {
        channel.interface_index = std::stoi(
          paramters_["command_interface/" + channel.interface_name]);
      }
    }
    channel.setup_interface_ptrs(state_interface_ptr_, command_interface_ptr_);
  }

  if (sm_configs_.size() == 0) {
    syncs_.push_back({0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE});
    syncs_.push_back({1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE});
    syncs_.push_back({2, EC_DIR_OUTPUT, (unsigned int)(rpdos_.size()), rpdos_.data(),
        EC_WD_ENABLE});
    syncs_.push_back({3, EC_DIR_INPUT, (unsigned int)(tpdos_.size()), tpdos_.data(),
        EC_WD_DISABLE});
  } else {
    for (auto & sm : sm_configs_) {
      if (sm.pdo_name == "null") {
        syncs_.push_back({sm.index, sm.type, 0, NULL, sm.watchdog});
      } else if (sm.pdo_name == "rpdo") {
        syncs_.push_back({sm.index, sm.type, (unsigned int)(rpdos_.size()), rpdos_.data(),
            sm.watchdog});
      } else if (sm.pdo_name == "tpdo") {
        syncs_.push_back({sm.index, sm.type, (unsigned int)(tpdos_.size()), tpdos_.data(),
            sm.watchdog});
      }
    }
  }
  syncs_.push_back({0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE});
  std::cout << tpdos_.size() << " TPDOs and " << rpdos_.size() << " RPDOs configured." << std::endl;
  std::flush(std::cout);

  if (paramters_.find("mode_of_operation") != paramters_.end()) {
    mode_of_operation_ = std::stod(paramters_["mode_of_operation"]);
  }

  if (paramters_.find("command_interface/reset_fault") != paramters_.end()) {
    fault_reset_command_interface_index_ = std::stoi(paramters_["command_interface/reset_fault"]);
  }

  return true;
}

bool ECMutilAxisDrive::setup_from_config(YAML::Node drive_config)
{
  if (drive_config.size() != 0) {
    if (drive_config["vendor_id"]) {
      vendor_id_ = drive_config["vendor_id"].as<uint32_t>();
    } else {
      std::cerr << "GenericEcSlave: failed to load drive vendor ID." << std::endl;
      return false;
    }
    if (drive_config["product_id"]) {
      product_id_ = drive_config["product_id"].as<uint32_t>();
    } else {
      std::cerr << "GenericEcSlave: failed to load drive product ID." << std::endl;
      return false;
    }
    if (drive_config["assign_activate"]) {
      assign_activate_ = drive_config["assign_activate"].as<uint32_t>();
    }

    if (drive_config["sm"]) {
      for (const auto & sm : drive_config["sm"]) {
        ethercat_interface::SMConfig config;
        if (config.load_from_config(sm)) {
          sm_configs_.push_back(config);
        }
      }
    }

    if (drive_config["sdo"]) {
      for (const auto & sdo : drive_config["sdo"]) {
        ethercat_interface::SdoConfigEntry config;
        if (config.load_from_config(sdo)) {
          sdo_config.push_back(config);
        }
      }
    }

    auto channels_nbr = 0;

    if (drive_config["rpdo"]) {
      for (auto i = 0ul; i < drive_config["rpdo"].size(); i++) {
        channels_nbr += drive_config["rpdo"][i]["channels"].size();
      }
    }
    if (drive_config["tpdo"]) {
      for (auto i = 0ul; i < drive_config["tpdo"].size(); i++) {
        channels_nbr += drive_config["tpdo"][i]["channels"].size();
      }
    }

    all_channels_.reserve(channels_nbr);
    channels_nbr = 0;

    if (drive_config["rpdo"]) {
      for (auto i = 0ul; i < drive_config["rpdo"].size(); i++) {
        auto rpdo_channels_size = drive_config["rpdo"][i]["channels"].size();
        for (auto c = 0ul; c < rpdo_channels_size; c++) {
          ethercat_interface::EcPdoChannelManager channel_info;
          channel_info.pdo_type = ethercat_interface::RPDO;
          channel_info.load_from_config(drive_config["rpdo"][i]["channels"][c]);
          auto axis_index = drive_config["rpdo"][i]["channels"][c]["axis"].as<uint16_t>();
          if(axis_index_real_ == axis_index) {
            pdo_channels_info_.push_back(channel_info);
            need_to_register_domains_channels_.push_back(channel_info.get_pdo_entry_info());
          }
          all_channels_.push_back(channel_info.get_pdo_entry_info());
        }
        rpdos_.push_back(
          {
            drive_config["rpdo"][i]["index"].as<uint16_t>(),
            (unsigned int)(rpdo_channels_size),
            all_channels_.data() + channels_nbr
          }
        );
        channels_nbr += rpdo_channels_size;
      }
    }

    if (drive_config["tpdo"]) {
      for (auto i = 0ul; i < drive_config["tpdo"].size(); i++) {
        auto tpdo_channels_size = drive_config["tpdo"][i]["channels"].size();

        for (auto c = 0ul; c < tpdo_channels_size; c++) {
          ethercat_interface::EcPdoChannelManager channel_info;
          channel_info.pdo_type = ethercat_interface::TPDO;
          channel_info.load_from_config(drive_config["tpdo"][i]["channels"][c]);
          auto axis_index = drive_config["tpdo"][i]["channels"][c]["axis"].as<uint16_t>();
          if(axis_index_real_ == axis_index) {
            pdo_channels_info_.push_back(channel_info);
            need_to_register_domains_channels_.push_back(channel_info.get_pdo_entry_info());
          }
          all_channels_.push_back(channel_info.get_pdo_entry_info());
        }
        tpdos_.push_back(
          {
            drive_config["tpdo"][i]["index"].as<uint16_t>(),
            (unsigned int)(tpdo_channels_size),
            all_channels_.data() + channels_nbr
          }
        );
        channels_nbr += tpdo_channels_size;
      }
    }

    // Remove gaps from domain mapping
    for (auto i = 0ul; i < need_to_register_domains_channels_.size(); i++) {
      if (need_to_register_domains_channels_[i].index != 0x0000) {
        for(auto j = 0ul; j < all_channels_.size(); j++) {
          if (need_to_register_domains_channels_[i].index == all_channels_[j].index &&
              need_to_register_domains_channels_[i].subindex == all_channels_[j].subindex)
          {
            domain_map_.push_back(j);
            break;
          }
        }
      }
    }

  } else {
    std::cerr << "GenericEcSlave: failed to load slave configuration: empty configuration" <<
      std::endl;
    return false;
  }

  // additional configuration parameters for CiA402 Drives
  if (drive_config["auto_fault_reset"]) {
    auto_fault_reset_ = drive_config["auto_fault_reset"].as<bool>();
  }
  if (drive_config["auto_state_transitions"]) {
    auto_state_transitions_ = drive_config["auto_state_transitions"].as<bool>();
  }

  return true;
}

bool ECMutilAxisDrive::setup_from_config_file(std::string config_file)
{
  // Read drive configuration from YAML file
  try {
    slave_config_ = YAML::LoadFile(config_file);
  } catch (const YAML::ParserException & ex) {
    std::cerr << "ECMutilAxisDrive: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  } catch (const YAML::BadFile & ex) {
    std::cerr << "ECMutilAxisDrive: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  }
  if (!setup_from_config(slave_config_)) {
    return false;
  }
  return true;
}

/** returns device state based upon the status_word */
DeviceState ECMutilAxisDrive::deviceState(uint16_t status_word)
{
  if ((status_word & 0b01001111) == 0b00000000) {
    return STATE_NOT_READY_TO_SWITCH_ON;
  } else if ((status_word & 0b01001111) == 0b01000000) {
    return STATE_SWITCH_ON_DISABLED;
  } else if ((status_word & 0b01101111) == 0b00100001) {
    return STATE_READY_TO_SWITCH_ON;
  } else if ((status_word & 0b01101111) == 0b00100011) {
    return STATE_SWITCH_ON;
  } else if ((status_word & 0b01101111) == 0b00100111) {
    return STATE_OPERATION_ENABLED;
  } else if ((status_word & 0b01101111) == 0b00000111) {
    return STATE_QUICK_STOP_ACTIVE;
  } else if ((status_word & 0b01001111) == 0b00001111) {
    return STATE_FAULT_REACTION_ACTIVE;
  } else if ((status_word & 0b01001111) == 0b00001000) {
    return STATE_FAULT;
  }
  return STATE_UNDEFINED;
}

/** returns the control word that will take device from state to next desired state */
uint16_t ECMutilAxisDrive::transition(DeviceState state, uint16_t control_word)
{
  switch (state) {
    case STATE_START:                     // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
      return control_word;
    case STATE_NOT_READY_TO_SWITCH_ON:    // -> STATE_SWITCH_ON_DISABLED (automatic)
      return control_word;
    case STATE_SWITCH_ON_DISABLED:        // -> STATE_READY_TO_SWITCH_ON
      return (control_word & 0b01111110) | 0b00000110;
    case STATE_READY_TO_SWITCH_ON:        // -> STATE_SWITCH_ON
      return (control_word & 0b01110111) | 0b00000111;
    case STATE_SWITCH_ON:                 // -> STATE_OPERATION_ENABLED
      return (control_word & 0b01111111) | 0b00001111;
    case STATE_OPERATION_ENABLED:         // -> GOOD
      return control_word | 0b00011111;
    case STATE_QUICK_STOP_ACTIVE:         // -> STATE_OPERATION_ENABLED
      return (control_word & 0b01111111) | 0b00001111;
    case STATE_FAULT_REACTION_ACTIVE:     // -> STATE_FAULT (automatic)
      return control_word;
    case STATE_FAULT:                     // -> STATE_SWITCH_ON_DISABLED
      if (auto_fault_reset_ || fault_reset_) {
        fault_reset_ = false;
        return (control_word & 0b11111111) | 0b10000000;     // automatic reset
      } else {
        return control_word;
      }
    default:
      break;
  }
  return control_word;
}

}  // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::ECMutilAxisDrive, ethercat_interface::EcSlave)
