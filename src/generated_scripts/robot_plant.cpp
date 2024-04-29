//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: robot_plant.cpp
//
// Code generated for Simulink model 'robot_plant'.
//
// Model version                  : 1.28
// Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
// C/C++ source code generated on : Fri Apr 19 14:45:31 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: AMD->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "robot_plant.h"
#include "robot_plant_private.h"

//
// This function updates continuous states using the ODE4 fixed-step
// solver algorithm
//
void ModelClass::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = static_cast<ODE4_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 3;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) std::memcpy(y, x,
                     static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  robot_plant_derivatives();

  // f1 = f(t + (h/2), y + (h/2)*f0)
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  this->step();
  robot_plant_derivatives();

  // f2 = f(t + (h/2), y + (h/2)*f1)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  this->step();
  robot_plant_derivatives();

  // f3 = f(t + h, y + h*f2)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  this->step();
  robot_plant_derivatives();

  // tnew = t + h
  // ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3)
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

void ModelClass::robot_plant_SystemCore_setup(robotics_slmobile_internal_bl_T
  *obj)
{
  static const char_T tmp_3[128] = { '\x00', '\x01', '\x02', '\x03', '\x04',
    '\x05', '\x06', '\x07', '\x08', '\x09', '\x0a', '\x0b', '\x0c', '\x0d',
    '\x0e', '\x0f', '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16',
    '\x17', '\x18', '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ',
    '!', '\"', '#', '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/',
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>',
    '?', '@', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
    'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\',
    ']', '^', '_', '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',
    'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
    '{', '|', '}', '~', '\x7f' };

  static const char_T tmp_0[23] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p',
    'e', 'e', 'd', 'H', 'e', 'a', 'd', 'i', 'n', 'g', 'R', 'a', 't', 'e' };

  static const char_T tmp_1[23] = { 'W', 'h', 'e', 'e', 'l', 'S', 'p', 'e', 'e',
    'd', 's', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-' };

  static const char_T tmp_2[11] = { 'W', 'h', 'e', 'e', 'l', 'S', 'p', 'e', 'e',
    'd', 's' };

  int32_T b_kstr;
  int32_T exitg1;
  int32_T nmatched;
  int32_T partial_match_size_idx_1;
  char_T b[11];
  char_T b_f4[11];
  char_T tmp;
  boolean_T b_bool;
  obj->isInitialized = 1;
  obj->KinModel.WheelRadius = 0.05;
  obj->KinModel.TrackWidth = 0.2;
  obj->KinModel.WheelSpeedRange[0] = (rtMinusInf);
  obj->KinModel.WheelSpeedRange[1] = (rtInf);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    tmp = tmp_2[b_kstr];
    b_f4[b_kstr] = tmp;
    b[b_kstr] = tmp;
  }

  b_bool = false;
  b_kstr = 1;
  do {
    exitg1 = 0;
    if (b_kstr - 1 < 11) {
      if (tmp_3[static_cast<int32_T>(b_f4[b_kstr - 1])] != tmp_3
          [static_cast<int32_T>(b[b_kstr - 1])]) {
        exitg1 = 1;
      } else {
        b_kstr++;
      }
    } else {
      b_bool = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  if (b_bool) {
    nmatched = 1;
    partial_match_size_idx_1 = 11;
    for (b_kstr = 0; b_kstr < 11; b_kstr++) {
      robot_plant_B.partial_match_data[b_kstr] = tmp_2[b_kstr];
    }
  } else {
    for (b_kstr = 0; b_kstr < 23; b_kstr++) {
      robot_plant_B.partial_match_data_c[b_kstr] = tmp_0[b_kstr];
    }

    b_bool = false;
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 11) {
        if (tmp_3[static_cast<int32_T>(b_f4[b_kstr - 1])] != tmp_3
            [static_cast<int32_T>(robot_plant_B.partial_match_data_c[b_kstr - 1])])
        {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    if (b_bool) {
      for (b_kstr = 0; b_kstr < 23; b_kstr++) {
        robot_plant_B.partial_match_data_c[b_kstr] = tmp_0[b_kstr];
      }

      nmatched = 1;
      partial_match_size_idx_1 = 23;
      std::memcpy(&robot_plant_B.partial_match_data[0],
                  &robot_plant_B.partial_match_data_c[0], 23U * sizeof(char_T));
    } else {
      nmatched = 0;
      partial_match_size_idx_1 = 0;
    }
  }

  if ((nmatched == 0) || (partial_match_size_idx_1 == 0)) {
    partial_match_size_idx_1 = 0;
  } else {
    if (0 <= partial_match_size_idx_1 - 1) {
      std::memcpy(&robot_plant_B.partial_match_data_c[0],
                  &robot_plant_B.partial_match_data[0],
                  ((partial_match_size_idx_1 - 1) + 1) * sizeof(char_T));
    }
  }

  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    b[b_kstr] = tmp_2[b_kstr];
  }

  b_bool = false;
  if (partial_match_size_idx_1 == 11) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 11) {
        if (robot_plant_B.partial_match_data_c[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  for (b_kstr = 0; b_kstr < 23; b_kstr++) {
    tmp = tmp_1[b_kstr];
    if (b_bool) {
      robot_plant_B.f.f1[b_kstr] = tmp;
      obj->KinModel.VehicleInputsInternal[b_kstr] = robot_plant_B.f.f1[b_kstr];
    } else {
      robot_plant_B.g.f1[b_kstr] = tmp_0[b_kstr];
      obj->KinModel.VehicleInputsInternal[b_kstr] = robot_plant_B.g.f1[b_kstr];
    }

    robot_plant_B.c_m.f1[b_kstr] = tmp;
    obj->KinModel.VehicleInputsInternal[b_kstr] = robot_plant_B.c_m.f1[b_kstr];
  }

  obj->KinModel.TrackWidth = obj->TrackWidth;
  obj->KinModel.WheelRadius = obj->WheelRadius;
  obj->KinModel.WheelSpeedRange[0] = obj->WheelSpeedRange[0];
  obj->KinModel.WheelSpeedRange[1] = obj->WheelSpeedRange[1];
  obj->TunablePropsChanged = false;
}

// Model step function
void ModelClass::step()
{
  static const char_T tmp[23] = { 'W', 'h', 'e', 'e', 'l', 'S', 'p', 'e', 'e',
    'd', 's', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-' };

  static const char_T tmp_0[23] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p',
    'e', 'e', 'd', 'H', 'e', 'a', 'd', 'i', 'n', 'g', 'R', 'a', 't', 'e' };

  robotics_slmobile_internal_bl_T *obj;
  real_T d;
  real_T r;
  real_T v;
  real_T wL;
  real_T wR;
  int32_T ret;
  boolean_T exitg1;
  boolean_T flag;
  boolean_T p;
  if (rtmIsMajorTimeStep((&robot_plant_M))) {
    // set solver stop time
    rtsiSetSolverStopTime(&(&robot_plant_M)->solverInfo,(((&robot_plant_M)
      ->Timing.clockTick0+1)*(&robot_plant_M)->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep((&robot_plant_M))) {
    (&robot_plant_M)->Timing.t[0] = rtsiGetT(&(&robot_plant_M)->solverInfo);
  }

  // Outport: '<Root>/x' incorporates:
  //   Integrator: '<S1>/Integrator'

  robot_plant_Y.x = robot_plant_X.Integrator_CSTATE[0];

  // Outport: '<Root>/y' incorporates:
  //   Integrator: '<S1>/Integrator'

  robot_plant_Y.y = robot_plant_X.Integrator_CSTATE[1];

  // MATLABSystem: '<S1>/MATLAB System' incorporates:
  //   Inport: '<Root>/omega_l'
  //   Inport: '<Root>/omega_r'
  //   Integrator: '<S1>/Integrator'

  if (robot_plant_DW.obj.TrackWidth != 0.26) {
    flag = (robot_plant_DW.obj.isInitialized == 1);
    if (flag) {
      robot_plant_DW.obj.TunablePropsChanged = true;
      robot_plant_DW.obj.tunablePropertyChanged[0] = true;
    }

    robot_plant_DW.obj.TrackWidth = 0.26;
  }

  if (robot_plant_DW.obj.WheelRadius != 0.06) {
    flag = (robot_plant_DW.obj.isInitialized == 1);
    if (flag) {
      robot_plant_DW.obj.TunablePropsChanged = true;
      robot_plant_DW.obj.tunablePropertyChanged[1] = true;
    }

    robot_plant_DW.obj.WheelRadius = 0.06;
  }

  robot_plant_B.varargin_1[0] = robot_plant_DW.obj.WheelSpeedRange[0];
  robot_plant_B.varargin_1[1] = robot_plant_DW.obj.WheelSpeedRange[1];
  flag = false;
  p = true;
  ret = 0;
  exitg1 = false;
  while ((!exitg1) && (ret < 2)) {
    if (!(robot_plant_B.varargin_1[ret] ==
          robot_plant_ConstInitP.MATLABSystem_WheelSpeedRange[ret])) {
      p = false;
      exitg1 = true;
    } else {
      ret++;
    }
  }

  if (p) {
    flag = true;
  }

  if (!flag) {
    flag = (robot_plant_DW.obj.isInitialized == 1);
    if (flag) {
      robot_plant_DW.obj.TunablePropsChanged = true;
      robot_plant_DW.obj.tunablePropertyChanged[2] = true;
    }

    robot_plant_DW.obj.WheelSpeedRange[0] = (rtMinusInf);
    robot_plant_DW.obj.WheelSpeedRange[1] = (rtInf);
  }

  obj = &robot_plant_DW.obj;
  if (robot_plant_DW.obj.TunablePropsChanged) {
    robot_plant_DW.obj.TunablePropsChanged = false;
    flag = robot_plant_DW.obj.tunablePropertyChanged[1];
    if (flag) {
      wL = robot_plant_DW.obj.WheelRadius;
      obj->KinModel.WheelRadius = wL;
    }

    flag = robot_plant_DW.obj.tunablePropertyChanged[0];
    if (flag) {
      wL = robot_plant_DW.obj.TrackWidth;
      obj->KinModel.TrackWidth = wL;
    }

    flag = robot_plant_DW.obj.tunablePropertyChanged[2];
    if (flag) {
      robot_plant_B.varargin_1[0] = robot_plant_DW.obj.WheelSpeedRange[0];
      robot_plant_B.varargin_1[1] = robot_plant_DW.obj.WheelSpeedRange[1];
      obj->KinModel.WheelSpeedRange[0] = robot_plant_B.varargin_1[0];
      obj->KinModel.WheelSpeedRange[1] = robot_plant_B.varargin_1[1];
    }

    robot_plant_DW.obj.tunablePropertyChanged[0] = false;
    robot_plant_DW.obj.tunablePropertyChanged[1] = false;
    robot_plant_DW.obj.tunablePropertyChanged[2] = false;
  }

  v = 0.0;
  wL = 0.0;
  for (ret = 0; ret < 23; ret++) {
    robot_plant_B.switch_expression[ret] = obj->
      KinModel.VehicleInputsInternal[ret];
  }

  for (ret = 0; ret < 23; ret++) {
    robot_plant_B.b[ret] = tmp[ret];
  }

  ret = memcmp(&robot_plant_B.switch_expression[0], &robot_plant_B.b[0], 23);
  if (ret == 0) {
    ret = 0;
  } else {
    for (ret = 0; ret < 23; ret++) {
      robot_plant_B.b[ret] = tmp_0[ret];
    }

    ret = memcmp(&robot_plant_B.switch_expression[0], &robot_plant_B.b[0], 23);
    if (ret == 0) {
      ret = 1;
    } else {
      ret = -1;
    }
  }

  switch (ret) {
   case 0:
    r = obj->KinModel.WheelSpeedRange[0];
    wL = obj->KinModel.WheelSpeedRange[1];
    if ((robot_plant_U.omega_l > r) || rtIsNaN(r)) {
      r = robot_plant_U.omega_l;
    }

    if ((r < wL) || rtIsNaN(wL)) {
      wL = r;
    }

    r = obj->KinModel.WheelSpeedRange[0];
    wR = obj->KinModel.WheelSpeedRange[1];
    if ((robot_plant_U.omega_r > r) || rtIsNaN(r)) {
      r = robot_plant_U.omega_r;
    }

    if ((r < wR) || rtIsNaN(wR)) {
      wR = r;
    }

    d = obj->KinModel.TrackWidth / 2.0;
    r = obj->KinModel.WheelRadius;
    v = (wR + wL) * r / 2.0;
    wL = (wR - wL) * r / (2.0 * d);
    break;

   case 1:
    d = obj->KinModel.TrackWidth / 2.0;
    r = obj->KinModel.WheelRadius;
    wL = robot_plant_U.omega_r * d;
    wR = (robot_plant_U.omega_l - wL) / r;
    v = (wL + robot_plant_U.omega_l) / r;
    r = obj->KinModel.WheelSpeedRange[0];
    wL = obj->KinModel.WheelSpeedRange[1];
    if ((wR > r) || rtIsNaN(r)) {
      r = wR;
    }

    if ((r < wL) || rtIsNaN(wL)) {
      wL = r;
    }

    r = obj->KinModel.WheelSpeedRange[0];
    wR = obj->KinModel.WheelSpeedRange[1];
    if ((v > r) || rtIsNaN(r)) {
      r = v;
    }

    if ((r < wR) || rtIsNaN(wR)) {
      wR = r;
    }

    d = obj->KinModel.TrackWidth / 2.0;
    r = obj->KinModel.WheelRadius;
    v = (wR + wL) * r / 2.0;
    wL = (wR - wL) * r / (2.0 * d);
    break;
  }

  robot_plant_B.dv[0] = std::cos(robot_plant_X.Integrator_CSTATE[2]);
  robot_plant_B.dv[3] = 0.0;
  robot_plant_B.dv[1] = std::sin(robot_plant_X.Integrator_CSTATE[2]);
  robot_plant_B.dv[4] = 0.0;
  robot_plant_B.dv[2] = 0.0;
  robot_plant_B.dv[5] = 1.0;
  for (ret = 0; ret < 3; ret++) {
    // MATLABSystem: '<S1>/MATLAB System'
    robot_plant_B.MATLABSystem[ret] = 0.0;
    robot_plant_B.MATLABSystem[ret] += robot_plant_B.dv[ret] * v;
    robot_plant_B.MATLABSystem[ret] += robot_plant_B.dv[ret + 3] * wL;
  }

  // Outport: '<Root>/Vx'
  robot_plant_Y.Vx = robot_plant_B.MATLABSystem[0];

  // Outport: '<Root>/Vy'
  robot_plant_Y.Vy = robot_plant_B.MATLABSystem[1];

  // Outport: '<Root>/psi_dot'
  robot_plant_Y.psi_dot = robot_plant_B.MATLABSystem[2];

  // Gain: '<Root>/Gain' incorporates:
  //   Integrator: '<S1>/Integrator'

  wL = 0.5 * robot_plant_X.Integrator_CSTATE[2];

  // Outport: '<Root>/q' incorporates:
  //   Constant: '<Root>/Constant'
  //   Trigonometry: '<Root>/Cos'
  //   Trigonometry: '<Root>/Sin'

  robot_plant_Y.q[0] = std::cos(wL);
  robot_plant_Y.q[3] = std::sin(wL);
  robot_plant_Y.q[1] = 0.0;
  robot_plant_Y.q[2] = 0.0;
  if (rtmIsMajorTimeStep((&robot_plant_M))) {
    rt_ertODEUpdateContinuousStates(&(&robot_plant_M)->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++(&robot_plant_M)->Timing.clockTick0;
    (&robot_plant_M)->Timing.t[0] = rtsiGetSolverStopTime(&(&robot_plant_M)
      ->solverInfo);

    {
      // Update absolute timer for sample time: [0.001s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.001, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      (&robot_plant_M)->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void ModelClass::robot_plant_derivatives()
{
  ModelClass::XDot_robot_plant_T *_rtXdot;
  _rtXdot = ((XDot_robot_plant_T *) (&robot_plant_M)->derivs);

  // Derivatives for Integrator: '<S1>/Integrator'
  _rtXdot->Integrator_CSTATE[0] = robot_plant_B.MATLABSystem[0];
  _rtXdot->Integrator_CSTATE[1] = robot_plant_B.MATLABSystem[1];
  _rtXdot->Integrator_CSTATE[2] = robot_plant_B.MATLABSystem[2];
}

// Model initialize function
void ModelClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // non-finite (run-time) assignments
  robot_plant_ConstInitP.MATLABSystem_WheelSpeedRange[0] = rtMinusInf;
  robot_plant_ConstInitP.MATLABSystem_WheelSpeedRange[1] = rtInf;

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&(&robot_plant_M)->solverInfo, &(&robot_plant_M)
                          ->Timing.simTimeStep);
    rtsiSetTPtr(&(&robot_plant_M)->solverInfo, &rtmGetTPtr((&robot_plant_M)));
    rtsiSetStepSizePtr(&(&robot_plant_M)->solverInfo, &(&robot_plant_M)
                       ->Timing.stepSize0);
    rtsiSetdXPtr(&(&robot_plant_M)->solverInfo, &(&robot_plant_M)->derivs);
    rtsiSetContStatesPtr(&(&robot_plant_M)->solverInfo, (real_T **)
                         &(&robot_plant_M)->contStates);
    rtsiSetNumContStatesPtr(&(&robot_plant_M)->solverInfo, &(&robot_plant_M)
      ->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&robot_plant_M)->solverInfo,
      &(&robot_plant_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&robot_plant_M)->solverInfo,
      &(&robot_plant_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&robot_plant_M)->solverInfo,
      &(&robot_plant_M)->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&(&robot_plant_M)->solverInfo, (&rtmGetErrorStatus
      ((&robot_plant_M))));
    rtsiSetRTModelPtr(&(&robot_plant_M)->solverInfo, (&robot_plant_M));
  }

  rtsiSetSimTimeStep(&(&robot_plant_M)->solverInfo, MAJOR_TIME_STEP);
  (&robot_plant_M)->intgData.y = (&robot_plant_M)->odeY;
  (&robot_plant_M)->intgData.f[0] = (&robot_plant_M)->odeF[0];
  (&robot_plant_M)->intgData.f[1] = (&robot_plant_M)->odeF[1];
  (&robot_plant_M)->intgData.f[2] = (&robot_plant_M)->odeF[2];
  (&robot_plant_M)->intgData.f[3] = (&robot_plant_M)->odeF[3];
  (&robot_plant_M)->contStates = ((X_robot_plant_T *) &robot_plant_X);
  rtsiSetSolverData(&(&robot_plant_M)->solverInfo, static_cast<void *>
                    (&(&robot_plant_M)->intgData));
  rtsiSetSolverName(&(&robot_plant_M)->solverInfo,"ode4");
  rtmSetTPtr((&robot_plant_M), &(&robot_plant_M)->Timing.tArray[0]);
  (&robot_plant_M)->Timing.stepSize0 = 0.001;

  {
    static const char_T tmp[23] = { 'W', 'h', 'e', 'e', 'l', 'S', 'p', 'e', 'e',
      'd', 's', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-' };

    real_T b;
    int32_T i;
    boolean_T flag;

    // Start for MATLABSystem: '<S1>/MATLAB System'
    robot_plant_DW.obj.isInitialized = 0;

    // InitializeConditions for Integrator: '<S1>/Integrator'
    robot_plant_X.Integrator_CSTATE[0] = 0.0;

    // Start for MATLABSystem: '<S1>/MATLAB System'
    robot_plant_DW.obj.tunablePropertyChanged[0] = false;

    // InitializeConditions for Integrator: '<S1>/Integrator'
    robot_plant_X.Integrator_CSTATE[1] = 0.0;

    // Start for MATLABSystem: '<S1>/MATLAB System'
    robot_plant_DW.obj.tunablePropertyChanged[1] = false;

    // InitializeConditions for Integrator: '<S1>/Integrator'
    robot_plant_X.Integrator_CSTATE[2] = 0.0;

    // Start for MATLABSystem: '<S1>/MATLAB System'
    robot_plant_DW.obj.tunablePropertyChanged[2] = false;
    flag = (robot_plant_DW.obj.isInitialized == 1);
    if (flag) {
      robot_plant_DW.obj.TunablePropsChanged = true;
      robot_plant_DW.obj.tunablePropertyChanged[0] = true;
    }

    robot_plant_DW.obj.TrackWidth = 0.26;
    flag = (robot_plant_DW.obj.isInitialized == 1);
    if (flag) {
      robot_plant_DW.obj.TunablePropsChanged = true;
      robot_plant_DW.obj.tunablePropertyChanged[1] = true;
    }

    robot_plant_DW.obj.WheelRadius = 0.06;
    flag = (robot_plant_DW.obj.isInitialized == 1);
    if (flag) {
      robot_plant_DW.obj.TunablePropsChanged = true;
      robot_plant_DW.obj.tunablePropertyChanged[2] = true;
    }

    robot_plant_DW.obj.WheelSpeedRange[0] = (rtMinusInf);
    robot_plant_DW.obj.WheelSpeedRange[1] = (rtInf);
    robot_plant_SystemCore_setup(&robot_plant_DW.obj);

    // InitializeConditions for MATLABSystem: '<S1>/MATLAB System'
    for (i = 0; i < 23; i++) {
      robot_plant_B.c.f1[i] = tmp[i];
      robot_plant_DW.obj.KinModel.VehicleInputsInternal[i] =
        robot_plant_B.c.f1[i];
    }

    b = robot_plant_DW.obj.TrackWidth;
    robot_plant_DW.obj.KinModel.TrackWidth = b;
    b = robot_plant_DW.obj.WheelRadius;
    robot_plant_DW.obj.KinModel.WheelRadius = b;
    b = robot_plant_DW.obj.WheelSpeedRange[0];
    robot_plant_DW.obj.KinModel.WheelSpeedRange[0] = b;
    b = robot_plant_DW.obj.WheelSpeedRange[1];
    robot_plant_DW.obj.KinModel.WheelSpeedRange[1] = b;

    // End of InitializeConditions for MATLABSystem: '<S1>/MATLAB System'
  }
}

// Model terminate function
void ModelClass::terminate()
{
  // (no terminate code required)
}

// Constructor
ModelClass::ModelClass() :
  robot_plant_B(),
  robot_plant_DW(),
  robot_plant_X(),
  robot_plant_U(),
  robot_plant_Y(),
  robot_plant_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
ModelClass::~ModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
ModelClass::RT_MODEL_robot_plant_T * ModelClass::getRTM()
{
  return (&robot_plant_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
