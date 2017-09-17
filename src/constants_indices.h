#ifndef CONSTANTS_INDICES_H
#define CONSTANTS_INDICES_H

#include "kinematic.h"
//#include <stdio.h>
//using namespace std;

const double POS_INF = 1.0e19;

const size_t N = 10;
const double dt = 0.1;

const size_t nVars = N * nState + (N - 1) * nActuator; // number of variables to solve for

const double ref_cte = 0;
const double ref_epsi = 0;
const double ref_v = 50;

const size_t x_Idx     = 0;
const size_t y_Idx     = x_Idx     + N;
const size_t psi_Idx   = y_Idx     + N;
const size_t v_Idx     = psi_Idx   + N;
const size_t CTE_Idx   = v_Idx     + N;
const size_t epsi_Idx  = CTE_Idx   + N;
const size_t steer_Idx = epsi_Idx  + N;
const size_t a_Idx     = steer_Idx + N - 1; // because steer and a has N-1 dim

#endif /* CONSTANTS_INDICES_H */