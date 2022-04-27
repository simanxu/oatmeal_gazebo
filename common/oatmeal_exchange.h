#ifndef OATMEAL_GAZEBO_COMMON_OATMEAL_EXCHANGE_H_
#define OATMEAL_GAZEBO_COMMON_OATMEAL_EXCHANGE_H_

#include "common/control_command.h"
#include "common/estimate_result.h"
#include "common/oatmeal_constants.h"
#include "common/status_data.h"
#include "controllers/oatmeal_dynamic.h"

namespace oatmeal {

struct ExchangeData {
  std::shared_ptr<OatmealDynamic> oatmeal;
  std::shared_ptr<EstimateResult> estimate_result;
  std::shared_ptr<ControlCommand> control_command;
};
}  // namespace oatmeal

#endif  // OATMEAL_GAZEBO_COMMON_OATMEAL_EXCHANGE_H_
