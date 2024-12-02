/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2024-12-02 19:45:56
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "computeSFO.h"
#include "computeSFO_emxAPI.h"
#include "computeSFO_types.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static emxArray_real_T *argInit_Unboundedx1_real_T(void);

static double argInit_real_T(void);

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : emxArray_real_T *
 */
static emxArray_real_T *argInit_Unboundedx1_real_T(void)
{
    emxArray_real_T *result;
    double *result_data;
    int idx0 = 2;
    /* Set the size of the array.
Change this size to the value that the application requires. */
    result = emxCreateND_real_T(1, &idx0);
    result_data = result->data;
    /* Loop over the array to initialize each element. */
    for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
        /* Set the value of the array element.
Change this value to the value that the application requires. */
        result_data[idx0] = argInit_real_T();
    }
    return result;
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
    return 0.0;
}

/*
 * Arguments    : int argc
 *                char **argv
 * Return Type  : int
 */
int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    /* The initialize function is being called automatically from your entry-point function. So, a call to initialize is
     * not included here. */
    /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
    main_computeSFO();
    /* Terminate the application.
You do not need to do this more than one time. */
    computeSFO_terminate();
    return 0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_computeSFO(void)
{
    emxArray_real_T *buffer;
    double clockOffset;
    /* Initialize function 'computeSFO' input arguments. */
    /* Initialize function input argument 'buffer'. */
    buffer = argInit_Unboundedx1_real_T();
    /* Call the entry-point 'computeSFO'. */
    clockOffset = computeSFO(buffer);
    emxDestroyArray_real_T(buffer);
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
