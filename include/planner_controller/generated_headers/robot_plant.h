//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: robot_plant.h
//
// Code generated for Simulink model 'robot_plant'.
//
// Model version                  : 1.26
// Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
// C/C++ source code generated on : Fri Apr 19 13:57:53 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: AMD->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_robot_plant_h_
#define RTW_HEADER_robot_plant_h_
#include <cmath>
#include <cstring>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "robot_plant_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef ODE4_INTG
#define ODE4_INTG

// ODE4 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[4];                        // derivatives
} ODE4_IntgData;

#endif

// Class declaration for model robot_plant
class ModelClass {
  // public data and function members
 public:
  // Block signals (default storage)
  typedef struct {
    real_T dv[6];
    real_T MATLABSystem[3];            // '<S1>/MATLAB System'
    d_cell_wrap_robot_plant_T c;
    d_cell_wrap_robot_plant_T f;
    d_cell_wrap_robot_plant_T g;
    d_cell_wrap_robot_plant_T c_m;
    char_T partial_match_data[23];
    char_T partial_match_data_c[23];
    char_T switch_expression[23];
    char_T b[23];
    real_T varargin_1[2];
  } B_robot_plant_T;

  // Block states (default storage) for system '<Root>'
  typedef struct {
    robotics_slmobile_internal_bl_T obj;// '<S1>/MATLAB System'
  } DW_robot_plant_T;

  // Continuous states (default storage)
  typedef struct {
    real_T Integrator_CSTATE[3];       // '<S1>/Integrator'
  } X_robot_plant_T;

  // State derivatives (default storage)
  typedef struct {
    real_T Integrator_CSTATE[3];       // '<S1>/Integrator'
  } XDot_robot_plant_T;

  // State disabled
  typedef struct {
    boolean_T Integrator_CSTATE[3];    // '<S1>/Integrator'
  } XDis_robot_plant_T;

  // Constant parameters with dynamic initialization (default storage)
  typedef struct {
    // Expression: WheelSpeedRange
    //  Referenced by: '<S1>/MATLAB System'

    real_T MATLABSystem_WheelSpeedRange[2];
  } ConstInitP_robot_plant_T;

  // External inputs (root inport signals with default storage)
  typedef struct {
    real_T omega_l;                    // '<Root>/omega_l'
    real_T omega_r;                    // '<Root>/omega_r'
  } ExtU_robot_plant_T;

  // External outputs (root outports fed by signals with default storage)
  typedef struct {
    real_T x;                          // '<Root>/x'
    real_T y;                          // '<Root>/y'
    real_T q[4];                       // '<Root>/q'
    real_T Vx;                         // '<Root>/Vx'
    real_T Vy;                         // '<Root>/Vy'
    real_T psi_dot;                    // '<Root>/psi_dot'
  } ExtY_robot_plant_T;

  // Real-time Model Data Structure
  struct RT_MODEL_robot_plant_T {
    const char_T *errorStatus;
    RTWSolverInfo solverInfo;
    X_robot_plant_T *contStates;
    int_T *periodicContStateIndices;
    real_T *periodicContStateRanges;
    real_T *derivs;
    boolean_T *contStateDisabled;
    boolean_T zCCacheNeedsReset;
    boolean_T derivCacheNeedsReset;
    boolean_T CTOutputIncnstWithState;
    real_T odeY[3];
    real_T odeF[4][3];
    ODE4_IntgData intgData;

    //
    //  Sizes:
    //  The following substructure contains sizes information
    //  for many of the model attributes such as inputs, outputs,
    //  dwork, sample times, etc.

    struct {
      int_T numContStates;
      int_T numPeriodicContStates;
      int_T numSampTimes;
    } Sizes;

    //
    //  Timing:
    //  The following substructure contains information regarding
    //  the timing information for the model.

    struct {
      uint32_T clockTick0;
      time_T stepSize0;
      uint32_T clockTick1;
      SimTimeStep simTimeStep;
      boolean_T stopRequestedFlag;
      time_T *t;
      time_T tArray[2];
    } Timing;
  };

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  ModelClass();

  // Destructor
  ~ModelClass();

  // Root-level structure-based inputs set method

  // Root inports set method
  void setExternalInputs(const ExtU_robot_plant_T* pExtU_robot_plant_T)
  {
    robot_plant_U = *pExtU_robot_plant_T;
  }

  // Root-level structure-based outputs get method

  // Root outports get method
  const ModelClass::ExtY_robot_plant_T & getExternalOutputs() const
  {
    return robot_plant_Y;
  }

  // Real-Time Model get method
  ModelClass::RT_MODEL_robot_plant_T * getRTM();

  // private data and function members
 private:
  // Block signals
  B_robot_plant_T robot_plant_B;

  // Block states
  DW_robot_plant_T robot_plant_DW;
  X_robot_plant_T robot_plant_X;       // Block continuous states

  // External inputs
  ExtU_robot_plant_T robot_plant_U;

  // External outputs
  ExtY_robot_plant_T robot_plant_Y;

  // Real-Time Model
  RT_MODEL_robot_plant_T robot_plant_M;

  // private member function(s) for subsystem '<Root>'
  void robot_plant_SystemCore_setup(robotics_slmobile_internal_bl_T *obj);

  // Continuous states update member function
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  // Derivatives member function
  void robot_plant_derivatives();
};

// Constant parameters with dynamic initialization (default storage)
extern ModelClass::ConstInitP_robot_plant_T robot_plant_ConstInitP;// constant parameters 

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S1>/Data Type Duplicate' : Unused code path elimination
//  Block '<S1>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S1>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S1>/Reshape' : Reshape block reduction


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'robot_plant'
//  '<S1>'   : 'robot_plant/Differential Drive Kinematic Model'

#endif                                 // RTW_HEADER_robot_plant_h_

//
// File trailer for generated code.
//
// [EOF]
//
