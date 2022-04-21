#include "controllers/src/oatmeal_controller.h"

#include <iostream>

OatmealController::OatmealController() {}

OatmealController::~OatmealController() {}

bool OatmealController::Test() {
  std::cout << "Hello Test" << std::endl;
  return true;
}
