//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 28-Aug-2017 23:08:56
//

//***********************************************************************
// This automatically generated example C main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************
// Include Files
#include "rt_nonfinite.h"
#include "polysync_controller.h"
#include "main.h"
#include "polysync_controller_terminate.h"
#include "polysync_controller_initialize.h"

// Function Declarations
static void main_polysync_controller();

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
static void main_polysync_controller()
{
  // Call the entry-point 'polysync_controller'.
  polysync_controller();
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  polysync_controller_initialize();

  main_polysync_controller();

  polysync_controller_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
