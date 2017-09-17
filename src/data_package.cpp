#include "data_package.h"

DataPackage::DataPackage(json Data){

  vector<double> WP_X = Data["ptsx"];
  vector<double> WP_Y = Data["ptsy"];

  WayPointX = WP_X; // assigning Data["ptsx"] directly to WayPointX doesn't work
  WayPointY = WP_Y; // due to ambiguous = operator of json

  Input = {
    Data["x"],
    Data["y"],
    Data["psi"],
    Data["speed"],
    Data["throttle"],
    Data["steering_angle"]
  };
}
