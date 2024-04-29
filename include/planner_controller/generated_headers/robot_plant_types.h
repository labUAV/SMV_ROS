//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: robot_plant_types.h
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
#ifndef RTW_HEADER_robot_plant_types_h_
#define RTW_HEADER_robot_plant_types_h_
#include "rtwtypes.h"

// Model Code Variants
#ifndef struct_tag_PMfBDzoakfdM9QAdfx2o6D
#define struct_tag_PMfBDzoakfdM9QAdfx2o6D

struct tag_PMfBDzoakfdM9QAdfx2o6D
{
  uint32_T f1[8];
};

#endif                                 //struct_tag_PMfBDzoakfdM9QAdfx2o6D

#ifndef typedef_cell_wrap_robot_plant_T
#define typedef_cell_wrap_robot_plant_T

typedef tag_PMfBDzoakfdM9QAdfx2o6D cell_wrap_robot_plant_T;

#endif                                 //typedef_cell_wrap_robot_plant_T

#ifndef struct_tag_lNwn6HJVMyz4L6armNe4ZG
#define struct_tag_lNwn6HJVMyz4L6armNe4ZG

struct tag_lNwn6HJVMyz4L6armNe4ZG
{
  char_T f1[23];
};

#endif                                 //struct_tag_lNwn6HJVMyz4L6armNe4ZG

#ifndef typedef_d_cell_wrap_robot_plant_T
#define typedef_d_cell_wrap_robot_plant_T

typedef tag_lNwn6HJVMyz4L6armNe4ZG d_cell_wrap_robot_plant_T;

#endif                                 //typedef_d_cell_wrap_robot_plant_T

#ifndef struct_tag_LXP8D2wemZcRfYEEJZJC9C
#define struct_tag_LXP8D2wemZcRfYEEJZJC9C

struct tag_LXP8D2wemZcRfYEEJZJC9C
{
  real_T TrackWidth;
  real_T WheelRadius;
  real_T WheelSpeedRange[2];
  char_T VehicleInputsInternal[23];
};

#endif                                 //struct_tag_LXP8D2wemZcRfYEEJZJC9C

#ifndef typedef_c_differentialDriveKinematics_T
#define typedef_c_differentialDriveKinematics_T

typedef tag_LXP8D2wemZcRfYEEJZJC9C c_differentialDriveKinematics_T;

#endif                                 //typedef_c_differentialDriveKinematics_T

#ifndef struct_tag_zLDXZGSRN3ffJgcF7JolM
#define struct_tag_zLDXZGSRN3ffJgcF7JolM

struct tag_zLDXZGSRN3ffJgcF7JolM
{
  boolean_T tunablePropertyChanged[3];
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  cell_wrap_robot_plant_T inputVarSize[3];
  c_differentialDriveKinematics_T KinModel;
  real_T TrackWidth;
  real_T WheelRadius;
  real_T WheelSpeedRange[2];
};

#endif                                 //struct_tag_zLDXZGSRN3ffJgcF7JolM

#ifndef typedef_robotics_slmobile_internal_bl_T
#define typedef_robotics_slmobile_internal_bl_T

typedef tag_zLDXZGSRN3ffJgcF7JolM robotics_slmobile_internal_bl_T;

#endif                                 //typedef_robotics_slmobile_internal_bl_T
#endif                                 // RTW_HEADER_robot_plant_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
