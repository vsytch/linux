/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation Confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2016 by Nuvoton Technology Corporation                                                   */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*    defs_utils.h                                                                                         */
/*            This file contains NTIL security utilities                                                   */
/* Project:                                                                                                */
/*            SWC DEFS                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef __DEFS_SECURED_H__
#define __DEFS_SECURED_H__


/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                              Secured constants with large hamming distance                              */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Secured constants with large hamming distance                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define SECURED_NULL        ((void*)(0xaa55))

typedef enum
{
    SECURED_TRUE  = 0x9696,
    SECURED_FALSE = 0x6969
} SECURED_BOOLEAN_T;


/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                         Secured utility macros                                          */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/*                                  Secured condition checking utilities                                   */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Macro:           DEFS_SEC_COND_CHECK                                                                    */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  cond    -  condition to check. Variables involved in the condition should be defined   */
/*                            'volatile' in the caller code                                                */
/*                                                                                                         */
/* Returns:         boolean indicating condition satisfaction                                              */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine checks a condition in a secured manner                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define DEFS_SEC_COND_CHECK(cond)         ((cond) && (cond))

/*---------------------------------------------------------------------------------------------------------*/
/* Macro:           DEFS_SEC_COND_CHECK_RET                                                                */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  cond    - Condition to check                                                           */
/*                  err     - Error to return if condition is not met                                      */
/*                                                                                                         */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine checks a condition securely and returns the required error if the         */
/*                  condition was not met                                                                  */
/*                                                                                                         */
/* Example:                                                                                                */
/*                                                                                                         */
/*    DEFS_STATUS myFunc(void* ptr)                                                                        */
/*    {                                                                                                    */
/*        DEFS_STATUS_COND_CHECK(ptr, DEFS_STATUS_INVALID_PARAMETER);                                      */
/*        ...                                                                                              */
/*    }                                                                                                    */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define DEFS_SEC_COND_CHECK_RET(cond, err)                                                                 \
{                                                                                                          \
    if (!DEFS_SEC_COND_CHECK(cond))                                                                        \
    {                                                                                                      \
        return err;                                                                                        \
    }                                                                                                      \
}

/*---------------------------------------------------------------------------------------------------------*/
/* Macro:           DEFS_SEC_COND_CHECK_FAIL                                                               */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  cond        - Condition to check                                                       */
/*                  failureFunc - A function that should be called in case the condition failed            */
/*                  ..          - parameters to failure function                                           */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine checks a condition securely and calls failure function if the             */
/*                  condition was not met                                                                  */
/*                                                                                                         */
/* Example:                                                                                                */
/*                                                                                                         */
/*    DEFS_STATUS myFunc(void* ptr)                                                                        */
/*    {                                                                                                    */
/*        DEFS_SEC_COND_CHECK_FAIL(x==5, DEFS_STATUS_INVALID_PARAMETER, FAIL_log);                         */
/*        ...                                                                                              */
/*    }                                                                                                    */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define DEFS_SEC_COND_CHECK_FAIL(cond, failureFunc, ...)                                                   \
{                                                                                                          \
    if (!DEFS_SEC_COND_CHECK(cond))                                                                        \
    {                                                                                                      \
        failureFunc(__VA_ARGS__);                                                                          \
    }                                                                                                      \
}

/*---------------------------------------------------------------------------------------------------------*/
/* Macro:           DEFS_SEC_COND_CHECK_ACTION                                                             */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  cond    - Condition to check                                                           */
/*                  action  - An action to be performed on failure                                         */
/*                                                                                                         */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This macro checks a condition securely and performs an action in case of error         */
/*                                                                                                         */
/* Example:                                                                                                */
/*                                                                                                         */
/*    status myFunc(void* ptr)                                                                             */
/*    {                                                                                                    */
/*        DEFS_SEC_COND_CHECK_ACTION(ptr, myaction);                                                       */
/*        ...                                                                                              */
/*                                                                                                         */
/*    }                                                                                                    */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define DEFS_SEC_COND_CHECK_ACTION(cond, action)                                                           \
    if (DEFS_SEC_COND_CHECK(cond))                                                                         \
    {                                                                                                      \
        action;                                                                                            \
    }                                                                                                      \

/*---------------------------------------------------------------------------------------------------------*/
/* Macro:           DEFS_SEC_COND_CHECK_GOTO                                                               */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  cond    - Condition to check                                                           */
/*                  endlabel  - Label to jump to if the condition is not met                               */
/*                                                                                                         */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This macro checks a condition securely and jumps to a label in case of error           */
/*                                                                                                         */
/* Example:                                                                                                */
/*                                                                                                         */
/*    status myFunc(void* ptr)                                                                             */
/*    {                                                                                                    */
/*        DEFS_STATUS_CHECK_RETVAL_GOTO(ptr, myFuncCleanup);                                               */
/*        ...                                                                                              */
/*                                                                                                         */
/* myFuncCleanup:                                                                                          */
/*        ...                                                                                              */
/*    }                                                                                                    */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define DEFS_SEC_COND_CHECK_GOTO(cond, endlabel)                                                           \
    DEFS_SEC_COND_CHECK_ACTION(cond, goto endlabel)

/*---------------------------------------------------------------------------------------------------------*/
/* Macro:           DEFS_SEC_FUNC_CHECK_RET                                                                */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  func    - function to check                                                            */
/*                                                                                                         */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This macro runs a function and securely makes sure it returns DEFS_STATUS_OK.          */
/*                  Otherwise it returns the error returned by the function.                               */
/*                                                                                                         */
/* Example:                                                                                                */
/*                                                                                                         */
/*    DEFS_STATUS myFunc(INT p1, INT p2);                                                                  */
/*                                                                                                         */
/*    DEFS_STATUS otherFunc                                                                                */
/*    {                                                                                                    */
/*        ...                                             // Some code                                     */
/*        DEFS_SEC_FUNC_CHECK_RET(myFunc(p1,p2));         // Executing myFunc                              */
/*        ...                                                                                              */
/*    }                                                                                                    */
/*                                                                                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define DEFS_SEC_FUNC_CHECK_RET(func)                                                                      \
{                                                                                                          \
    volatile status rc = DEFS_STATUS_FAIL;                                                                 \
    rc = func;                                                                                             \
                                                                                                           \
    if (!DEFS_SEC_COND_CHECK(rc == DEFS_STATUS_OK))                                                        \
    {                                                                                                      \
        return rc;                                                                                         \
    }                                                                                                      \
}

/*---------------------------------------------------------------------------------------------------------*/
/* Macro:           DEFS_SEC_FUNC_CHECK_FAIL                                                               */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  func        - function to check                                                        */
/*                  failureFunc - a function that should be called in case the function "func" failed      */
/*                  ..          - parameters to failure function                                           */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This macro checks securely if given function returns DEFS_STATUS_OK,                   */
/*                  Otherwise it calls failure function                                                    */
/*                                                                                                         */
/* Example:                                                                                                */
/*                                                                                                         */
/*    status myFirstFunc(void* ptr)                                                                        */
/*    {                                                                                                    */
/*        ...                                                // Some code                                  */
/*        DEFS_SEC_FUNC_CHECK_FAIL(myFunc(p1,p2), MY_ERROR, FAIL_Log);                                     */
/*        ...                                                                                              */
/*    }                                                                                                    */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define DEFS_SEC_FUNC_CHECK_FAIL(func, failureFunc, ...)                                                   \
{                                                                                                          \
    volatile status rc = DEFS_STATUS_FAIL;                                                                 \
    rc = func;                                                                                             \
                                                                                                           \
    if (!DEFS_SEC_COND_CHECK(rc == DEFS_STATUS_OK))                                                        \
    {                                                                                                      \
        failureFunc(__VA_ARGS__);                                                                          \
    }                                                                                                      \
}

/*---------------------------------------------------------------------------------------------------------*/
/* Macro:           DEFS_SEC_FUNC_CHECK_ACTION                                                             */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  func    - function to check                                                            */
/*                  action  - Action done if function doesn't return DEFS_STATUS_OK                        */
/*                                                                                                         */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This macro checks securely if given function returns DEFS_STATUS_OK,                   */
/*                  Otherwise it performs a pre-defined action                                             */
/*                                                                                                         */
/* Example:                                                                                                */
/*                                                                                                         */
/*    status myFirstFunc(void* ptr)                                                                        */
/*    {                                                                                                    */
/*        ...                                                // Some code                                  */
/*        DEFS_SEC_FUNC_CHECK_ACTION(myFunc(p1,p2), break);  // Executing myFunc, doing action on failure  */
/*        ...                                                                                              */
/*    }                                                                                                    */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define DEFS_SEC_FUNC_CHECK_ACTION(func, action)                                                           \
{                                                                                                          \
    volatile status rc = DEFS_STATUS_FAIL;                                                                 \
    rc = func;                                                                                             \
                                                                                                           \
    if (!DEFS_SEC_COND_CHECK(rc == DEFS_STATUS_OK))                                                        \
    {                                                                                                      \
        action;                                                                                            \
    }                                                                                                      \
}

/*---------------------------------------------------------------------------------------------------------*/
/* Macro:           DEFS_SEC_FUNC_CHECK_GOTO                                                               */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  func    - function to check                                                            */
/*                  endlabel  - Label to jump to if the function return code is not DEFS_STATUS_OK         */
/*                                                                                                         */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This macro runs a function securely and jumps to a label in case of error              */
/*                                                                                                         */
/* Example:                                                                                                */
/*                                                                                                         */
/*    status myFirstFunc(void* ptr)                                                                        */
/*    {                                                                                                    */
/*        ...                                                // Some code                                  */
/*        DEFS_SEC_FUNC_CHECK_GOTO(myFunc(p1,p2), myLabel);  // Executing myFunc, jump a label on failure  */
/*        ...                                                                                              */
/*    }                                                                                                    */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define DEFS_SEC_FUNC_CHECK_GOTO(func, endlabel)                                                           \
    DEFS_SEC_FUNC_CHECK_ACTION(func, goto endlabel)


/*---------------------------------------------------------------------------------------------------------*/
/*                               Secured Execution of a function from pointer                              */
/*---------------------------------------------------------------------------------------------------------*/
/* Call the macro in the following way:                                                                    */
/* SEC_EXECUTE_FUNC( func, (arg1, arg2))                                                                   */
/* SEC_EXECUTE_FUNC_RET( retvar, func, (arg1, arg2, arg3))                                                 */
/* SEC_EXECUTE_FUNC_RET_VALUE( func, (arg1, arg2, arg3), default_return_value)                             */
/*---------------------------------------------------------------------------------------------------------*/
#define SEC_EXECUTE_FUNC(func, args)                \
    if (func != SECURED_NULL)                       \
    {                                               \
        func args;                                  \
    }

#define SEC_EXECUTE_FUNC_RET(ret, func, args)       \
    if (func != SECURED_NULL)                       \
    {                                               \
        ret = func args;                            \
    }

#define SEC_EXECUTE_FUNC_RET_VALUE(func, args, default_value)   ((func != SECURED_NULL)? func args : default_value)


/*---------------------------------------------------------------------------------------------------------*/
/*                                     Secured flow control utilities                                      */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Macro:           DEFS_FLOW_MONITOR_DECLARE                                                              */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This macro declares a typically local varible representing flow counter                */
/*---------------------------------------------------------------------------------------------------------*/
#define DEFS_FLOW_MONITOR_DECLARE()      volatile int flowCounter = 0


/*---------------------------------------------------------------------------------------------------------*/
/* Macro:           DEFS_FLOW_MONITOR_INCREMENT                                                            */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This macro increments the flow counters. Typically called in critical point of the     */
/*                  flow. This point is defined as one that is important to make sure the flow indeed      */
/*                  passed by                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define DEFS_FLOW_MONITOR_INCREMENT()    flowCounter++

/*---------------------------------------------------------------------------------------------------------*/
/* Macro:           DEFS_FLOW_MONITOR_COMPARE                                                              */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  x           - an expected value of flow counter. This typically would be a constant    */
/*                                defined in the code                                                      */
/*                  failureFunc - a function that should be called in case the flow has been compromised   */
/*                                the functions receives variable number of parameters                     */
/*                  ...         - parameters to the failure function                                       */
/*                                                                                                         */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This securely compares the value of actual flow counter with its expected value and    */
/*                  calls a failure function if the comparison fails                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define DEFS_FLOW_MONITOR_COMPARE(x, failureFunc, ...)                                                      \
                         if (DEFS_SEC_COND_CHECK(x != flowCounter))                                         \
                         {                                                                                  \
                            failureFunc(__VA_ARGS__);                                                       \
                         }                                                                                  \


#endif /* __DEFS_SECURED_H__ */
