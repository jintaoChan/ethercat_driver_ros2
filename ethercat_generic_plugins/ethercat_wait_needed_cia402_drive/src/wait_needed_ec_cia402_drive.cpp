#include "ethercat_generic_plugins/wait_needed_ec_cia402_drive.hpp"

namespace ethercat_generic_plugins
{

  void WaitNeededEcCiA402Drive::processData(size_t index, uint8_t *domain_address)
  {
    // Special case: ControlWord
    if (pdo_channels_info_[index].index == CiA402D_RPDO_CONTROLWORD)
    {
      if (is_operational_)
      {
        if (fault_reset_command_interface_index_ >= 0)
        {
          if (command_interface_ptr_->at(fault_reset_command_interface_index_) == 0)
          {
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

        if (auto_state_transitions_)
        {
          if (repeat_times_remain_ != 0) {
            pdo_channels_info_[index].default_value = repeat_control_word;
            --repeat_times_remain_;
            // std::cout << "Counting Down: " << repeat_times_remain_ << "\t" << "Sending control_word: " <<  repeat_control_word << std::endl;
            // std::flush(std::cout);
          }
          else {
            pdo_channels_info_[index].default_value = transition(
                state_,
                pdo_channels_info_[index].ec_read(domain_address));
            if (state_ == STATE_OPERATION_ENABLED) {
              if(!terminate_repeat_) {
                terminate_repeat_ = true;
                repeat_control_word = pdo_channels_info_[index].default_value;
                repeat_times_remain_ = repeat_times_;
              }
            }
            else {
              terminate_repeat_ = false;
              repeat_control_word = pdo_channels_info_[index].default_value;
              repeat_times_remain_ = repeat_times_;
            }
          }
          // std::cout << "Sending control_word: " <<  pdo_channels_info_[index].default_value << std::endl;
          // std::flush(std::cout);
        }
      }
    }

    // setup current position as default position
    if (pdo_channels_info_[index].index == CiA402D_RPDO_POSITION)
    {
      if (mode_of_operation_display_ != ModeOfOperation::MODE_NO_MODE)
      {
        pdo_channels_info_[index].default_value =
            pdo_channels_info_[index].factor * last_position_ +
            pdo_channels_info_[index].offset;
      }
      pdo_channels_info_[index].override_command =
          (mode_of_operation_display_ != ModeOfOperation::MODE_CYCLIC_SYNC_POSITION) ? true : false;
      if(repeat_times_remain_ != 0 || state_ != STATE_OPERATION_ENABLED) {
        pdo_channels_info_[index].override_command = true;
      }
      // std::cout << "Writing Position: " <<  
      // pdo_channels_info_[index].command_interface_ptr_->at(pdo_channels_info_[index].interface_index)
      // << "\t" << "Default Value:" << pdo_channels_info_[index].default_value
      // << "\t"<< "Override? : " << pdo_channels_info_[index].override_command
      // << "\t"<< "repeat_times_remain_: " << repeat_times_remain_
      // << "\t"<< "state_: " << state_
      // << std::endl;
      // std::flush(std::cout);
    }

    // setup mode of operation
    if (pdo_channels_info_[index].index == CiA402D_RPDO_MODE_OF_OPERATION)
    {
      if (mode_of_operation_ >= 0 && mode_of_operation_ <= 10)
      {
        pdo_channels_info_[index].default_value = mode_of_operation_;
      }
    }

    pdo_channels_info_[index].ec_update(domain_address);

    // get mode_of_operation_display_
    if (pdo_channels_info_[index].index == CiA402D_TPDO_MODE_OF_OPERATION_DISPLAY)
    {
      mode_of_operation_display_ = pdo_channels_info_[index].last_value;
    }

    if (pdo_channels_info_[index].index == CiA402D_TPDO_POSITION)
    {
      last_position_ = pdo_channels_info_[index].last_value;
    }

    // Special case: StatusWord
    if (pdo_channels_info_[index].index == CiA402D_TPDO_STATUSWORD)
    {
      status_word_ = pdo_channels_info_[index].last_value;
      // std::cout << "Getting status_word: " <<  status_word_ << std::endl;
      // std::flush(std::cout);
    }

    // CHECK FOR STATE CHANGE
    if (index == all_channels_.size() - 1)
    { // if last entry  in domain
      if (status_word_ != last_status_word_)
      {
        state_ = deviceState(status_word_);
        if (state_ != last_state_)
        {
          std::cout << "STATE: " << DEVICE_STATE_STR.at(state_)
                    << " with status word :" << status_word_ << std::endl;
        }
      }
      initialized_ = ((state_ == STATE_OPERATION_ENABLED) &&
                      (last_state_ == STATE_OPERATION_ENABLED))
                         ? true
                         : false;

      last_status_word_ = status_word_;
      last_state_ = state_;
      counter_++;
    }
  }

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::WaitNeededEcCiA402Drive, ethercat_interface::EcSlave)
