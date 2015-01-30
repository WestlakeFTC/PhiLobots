#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="pidctrl.h" />
///
/// <summary>
///     This module contains the library functions for PID Control.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _PIDCTRL_H
#define _PIDCTRL_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PIDCTRL

//
// Constants.
//
#define PIDCTRLO_INVERSE_INPUT  0x0001
#define PIDCTRLO_INVERSE_OUTPUT 0x0002
#define PIDCTRLO_ABS_SETPT      0x0004
#define PIDCTRLO_NO_OSCILLATE   0x0008
#define PIDCTRLO_INTEGRATE      0x0010

#define PIDCTRLF_LIMIT_SETPT    0x0001

//
// Macros.
//
/**
 *  This macro returns the SetPoint of the PID controller.
 *
 *  @param p Points to the PID Control structure.
 *
 *  @return Returns the setpoint value.
 */
#define PIDCtrlGetTarget(p)     ((p).setPoint)

/**
 *  This macro returns the previous error of the PID controller.
 *
 *  @param p Points to the PID Control structure.
 *
 *  @return Returns the error value.
 */
#define PIDCtrlGetError(p)      ((p).prevError)

/**
 *  This macro prints the PID controller info: Target, Error, Input and
 *  Output to the debug stream.
 *
 *  @param p Points to the PID Control structure.
 */
#define PIDCtrlDebugInfo(p)     TPrintfLine(                                \
                                    "T=%5.1f, E=%5.1f, I=%5.1f, O=%5.1f",   \
                                    (p).setPoint, (p).prevError,            \
                                    PIDCtrlGetInput(p), (p).prevOutput)
//
// Type definitions.
//
typedef struct
{
    float           Kp;
    float           Ki;
    float           Kd;
    float           tolerance;
    unsigned long   settlingTime;
    float           minOutput;
    float           maxOutput;
    float           setPtLowLimit;
    float           setPtHighLimit;
    int             options;
    int             flags;
    float           prevError;
    float           totalError;
    unsigned long   startSettling;
    float           setPoint;
    float           prevOutput;
} PIDCTRL;

//
// Import function prototypes.
//

/**
 *  This function is called by the PID controller to get the current input
 *  value.
 *
 *  @param pidCtrl Points to the PID Control structure.
 */
float
PIDCtrlGetInput(
    PIDCTRL &pidCtrl
    );

/**
 *  This function resets the PID control.
 *
 *  @param pidCtrl Points to the PID Control structure to be reset.
 */
void
PIDCtrlReset(
    PIDCTRL &pidCtrl
    )
{
    TFuncName("PIDCtrlReset");
    TLevel(API);
    TEnter();

    pidCtrl.setPoint = 0.0;
    pidCtrl.prevError = 0.0;
    pidCtrl.totalError = 0.0;
    pidCtrl.prevOutput = 0.0;

    TExit();
    return;
}   //PIDCtrlReset

/**
 *  This function initializes the PID control object.
 *
 *  @param pidCtrl Points to the PID Control structure.
 *  @param Kp Specifies the Kp constant.
 *  @param Ki Specifies the Ki constant.
 *  @param Kd Specifies the Kd constant.
 *  @param tolerance Specifies the on-target tolerance.
 *  @param settlingTime Specifies the on-target settling time in msec.
 *  @param options Optionally specifies the PID controller options:
 *         PIDCTRLO_INVERSE_INPUT - Setpoint is inverse.
 *         PIDCTRLO_INVERSE_OUTPUT - Output should be reversed
 *         PIDCTRLO_ABS_SETPT - Setpoint is absolute.
 *         PIDCTRLO_NO_OSCILLATE - No oscillation.
 *         PIDCTRLO_INTEGRATE - Integrate the output (for speed control).
 *  @param initialOutput Optionally specifies the initial prevOutput value,
 *         applicable only in speed control mode.
 */
void
PIDCtrlInit(
    PIDCTRL &pidCtrl,
    float Kp,
    float Ki,
    float Kd,
    float tolerance,
    unsigned long settlingTime,
    int options = 0,
    float initialOutput = 0.0
    )
{
    TFuncName("PIDCtrlInit");
    TLevel(INIT);
    TEnter();

    pidCtrl.Kp = Kp;
    pidCtrl.Ki = Ki;
    pidCtrl.Kd = Kd;
    pidCtrl.tolerance = tolerance;
    pidCtrl.settlingTime = settlingTime;
    pidCtrl.minOutput = MOTOR_MIN_VALUE;
    pidCtrl.maxOutput = MOTOR_MAX_VALUE;
    pidCtrl.setPtLowLimit = 0.0;
    pidCtrl.setPtHighLimit = 0.0;
    pidCtrl.options = options;
    pidCtrl.flags = 0;
    pidCtrl.startSettling = 0;
    pidCtrl.setPoint = 0.0;
    pidCtrl.prevOutput = initialOutput;
    PIDCtrlReset(pidCtrl);

    TExit();
    return;
}   //PIDCtrlInit

/**
 *  This function sets the PID constants.
 *
 *  @param pidCtrl Points to the PID Control structure.
 *  @param Kp Specifies the Kp constant.
 *  @param Ki Specifies the Ki constant.
 *  @param Kd Specifies the Kd constant.
 */
void
PIDCtrlSetPID(
    PIDCTRL &pidCtrl,
    float Kp,
    float Ki,
    float Kd
    )
{
    TFuncName("PIDCtrlSetPID");
    TLevel(API);
    TEnterMsg(("Kp=%f,Ki=%f,Kd=%f", Kp, Ki, Kd));

    pidCtrl.Kp = Kp;
    pidCtrl.Ki = Ki;
    pidCtrl.Kd = Kd;

    TExit();
    return;
}   //PIDCtrlSetPID

/**
 *  This function sets the output power limits.
 *
 *  @param pidCtrl Points to the PID Control structure.
 *  @param minOutput Specifies the minimum output level.
 *  @param maxOutput Specifies the maximum output level.
 */
void
PIDCtrlSetPowerLimits(
    PIDCTRL &pidCtrl,
    float minOutput,
    float maxOutput
    )
{
    TFuncName("PIDCtrlSetPwrLimits");
    TLevel(API);
    TEnterMsg(("Min=%5.1f,Max=%5.1f", minOutput, maxOutput));

    pidCtrl.minOutput = minOutput;
    pidCtrl.maxOutput = maxOutput;

    TExit();
    return;
}   //PIDCtrlSetPowerLimits

/**
 *  This function sets the setpoint limits.
 *
 *  @param pidCtrl Points to the PID Control structure.
 *  @param setPtLowLimit Specifies the low limit of the setpoint.
 *  @param setPtHighLimit Specifies the high limit of the setpoint.
 *
 *  @note If both setPtLowLimit and setPtHighLimit are zero, it means the
 *        the setpoint limits are cleared.
 */
void
PIDCtrlSetSetpointLimits(
    PIDCTRL &pidCtrl,
    float setPtLowLimit,
    float setPtHighLimit
    )
{
    TFuncName("PIDCtrlSetSetpointLimits");
    TLevel(API);
    TEnterMsg(("Lo=%5.1f,Hi=%5.1f", setPtLowLimit, setPtHighLimit));

    pidCtrl.setPtLowLimit = setPtLowLimit;
    pidCtrl.setPtHighLimit = setPtHighLimit;
    if ((setPtLowLimit == 0.0) && (setPtHighLimit == 0.0))
    {
        pidCtrl.flags &= ~PIDCTRLF_LIMIT_SETPT;
    }
    else
    {
        pidCtrl.flags |= PIDCTRLF_LIMIT_SETPT;
    }

    TExit();
    return;
}   //PIDCtrlSetSetPointLimits

/**
 *  This function sets the SetPoint.
 *
 *  @param pidCtrl Points to the PID Control structure.
 *  @param setPoint Specifies the SetPoint target.
 *  @param currInput Specifies the current input value.
 */
void
PIDCtrlSetTarget(
    PIDCTRL &pidCtrl,
    float setPoint,
    float currInput
    )
{
    TFuncName("PIDCtrlSetTarget");
    TLevel(API);
    TEnter();

    if (!(pidCtrl.options & PIDCTRLO_ABS_SETPT))
    {
        setPoint += currInput;
    }

    if (pidCtrl.flags & PIDCTRLF_LIMIT_SETPT)
    {
        if (setPoint < pidCtrl.setPtLowLimit)
        {
            setPoint = pidCtrl.setPtLowLimit;
        }
        else if (setPoint > pidCtrl.setPtHighLimit)
        {
            setPoint = pidCtrl.setPtHighLimit;
        }
    }

    pidCtrl.setPoint = setPoint;
    pidCtrl.prevError = setPoint - currInput;
    pidCtrl.totalError = 0.0;
    pidCtrl.startSettling = nPgmTime;

    TExit();
    return;
}   //PIDCtrlSetTarget

/**
 *  This function determines if we are on target by checking if the previous
 *  error is within target tolerance and remain within tolerance for at least
 *  the settling period.
 *
 *  @param pidCtrl Points to the PID Control structure.
 *
 *  @returns Returns true if we are on target, false otherwise.
 */
bool
PIDCtrlIsOnTarget(
    PIDCTRL &pidCtrl
    )
{
    bool fOnTarget = false;

    TFuncName("PIDCtrlIsOnTarget");
    TLevel(HIFREQ);
    TEnter();

    if (pidCtrl.options & PIDCTRLO_NO_OSCILLATE)
    {
        //
        // Do not allow oscillation. If the error is within tolerance, stop.
        //
        if (abs(pidCtrl.prevError) <= pidCtrl.tolerance)
        {
            fOnTarget = true;
        }
    }
    else if (abs(pidCtrl.prevError) > pidCtrl.tolerance)
    {
        pidCtrl.startSettling = nPgmTime;
    }
    else if (nPgmTime - pidCtrl.startSettling >= pidCtrl.settlingTime)
    {
        //
        // We consider OnTarget only if the error is within tolerance for
        // at least the settling time.
        //
        fOnTarget = true;
    }

    TExitMsg(("=%x", fOnTarget));
    return fOnTarget;
}   //PIDCtrlIsOnTarget

/**
 *  This function calculates the output based on the current input.
 *
 *  @param pidCtrl Points to the PID structure.
 *  @param currInput Specifies the current input value.
 *
 *  @return Returns the calculate output value.
 */
float
PIDCtrlOutput(
    PIDCTRL &pidCtrl,
    float currInput
    )
{
    float output;
    float error;
    float adjTotalError;

    TFuncName("PIDCtrlOutput");
    TLevel(API);
    TEnter();

    error = pidCtrl.setPoint - currInput;
    if (pidCtrl.options & PIDCTRLO_INVERSE_INPUT)
    {
        error = -error;
    }
    adjTotalError = pidCtrl.Ki*(pidCtrl.totalError + error);
    if ((adjTotalError >= pidCtrl.minOutput) &&
        (adjTotalError <= pidCtrl.maxOutput))
    {
        pidCtrl.totalError += error;
    }

    output = pidCtrl.Kp*error +
             pidCtrl.Ki*pidCtrl.totalError +
             pidCtrl.Kd*(error - pidCtrl.prevError);

    if (pidCtrl.options & PIDCTRLO_INTEGRATE)
    {
        output += pidCtrl.prevOutput;
    }

    if (pidCtrl.options & PIDCTRLO_INVERSE_OUTPUT)
    {
        output = -output;
    }

    if (output < pidCtrl.minOutput)
    {
        output = pidCtrl.minOutput;
    }
    else if (output > pidCtrl.maxOutput)
    {
        output = pidCtrl.maxOutput;
    }

    pidCtrl.prevError = error;
    pidCtrl.prevOutput = output;

    TExitMsg(("=%f", output));
    return output;
}   //PIDCtrlOutput

#endif  //ifndef _PIDCTRL_H
