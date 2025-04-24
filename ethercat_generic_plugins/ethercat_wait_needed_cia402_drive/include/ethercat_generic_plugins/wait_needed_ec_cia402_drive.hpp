#pragma once
#include "ethercat_generic_plugins/generic_ec_cia402_drive.hpp"


namespace ethercat_generic_plugins
{

class WaitNeededEcCiA402Drive : public EcCiA402Drive
{

public:
  WaitNeededEcCiA402Drive() : EcCiA402Drive() {}
  ~WaitNeededEcCiA402Drive() {}

  virtual void processData(size_t index, uint8_t * domain_address) override;


protected:
  const uint16_t repeat_times_{200};
  uint16_t repeat_times_remain_{0};
  uint16_t repeat_control_word;
  bool terminate_repeat_{false};
};

}