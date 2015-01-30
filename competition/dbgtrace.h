#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="dbgtrace.h" />
///
/// <summary>
///     This module contains the tracing functions and definitions.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _DBGTRACE_H
#define _DBGTRACE_H

#pragma systemFile

//
// Constants.
//
#ifndef TRACE_MODULES
  #define TRACE_MODULES         0
#endif
#ifndef TRACE_LEVEL
  #define TRACE_LEVEL           TASK
#endif
#ifndef MSG_LEVEL
  #define MSG_LEVEL             INFO
#endif

//
// Module ID.
//
#define MOD_LIB                 0xfffffff0
#define MOD_GEN_MASK            0x0000000f
#define MOD_BATT                0x00000010
#define MOD_MENU                0x00000020
#define MOD_NXTBTN              0x00000040
#define MOD_JOYSTICK            0x00000080
#define MOD_TIMER               0x00000100
#define MOD_SM                  0x00000200
#define MOD_KALMAN              0x00000400
#define MOD_ANALOG              0x00000800
#define MOD_SENSOR              0x00001000
#define MOD_ENCODER             0x00002000
#define MOD_ACCEL               0x00004000
#define MOD_GYRO                0x00008000
#define MOD_COMPASS             0x00010000
#define MOD_TOUCH               0x00020000
#define MOD_IRSEEKER            0x00040000
#define MOD_RADAR               0x00080000
#define MOD_SERVO               0x00100000
#define MOD_PIDCTRL             0x00200000
#define MOD_DRIVE               0x00400000
#define MOD_PIDDRIVE            0x00800000
#define MOD_PIDMOTOR            0x01000000
#define MOD_LNFOLLOW            0x02000000
#define MOD_WALLFOLLOW          0x04000000

#define MOD_MAIN                0x00000001
#define TGenModId(n)            ((MOD_MAIN << (n)) && MOD_GEN_MASK)

#define INIT                    0
#define API                     1
#define CALLBK                  2
#define EVENT                   3
#define FUNC                    4
#define TASK                    5
#define UTIL                    6
#define HIFREQ                  7

#define FATAL                   0
#define ERR                     1
#define WARN                    2
#define INFO                    3
#define VERBOSE                 4

#ifndef TRACE_PERIOD
    #define TRACE_PERIOD        500     //in msec
#endif

#ifndef SAMPLING_PERIOD
    #define SAMPLING_PERIOD     500     //in msec
#endif

#define TPrintf                 writeDebugStream
#define TPrintfLine             writeDebugStreamLine

//
// Trace macros.
//
#ifdef _DEBUG_TRACE
    #define TModEnterMsg(m,p)   if (g_TraceEnabled && \
                                    ((g_TraceModules & (m)) != 0) && \
                                    (_levelTrace <= g_TraceLevel)) \
                                { \
                                    TracePrefix(_pszFuncName, true, false); \
                                    TPrintf p; \
                                    TPrintf(")\n"); \
                                }
    #define TModEnter(m)        if (g_TraceEnabled && \
                                    ((g_TraceModules & (m)) != 0) && \
                                    (_levelTrace <= g_TraceLevel)) \
                                { \
                                    TracePrefix(_pszFuncName, true, true); \
                                }
    #define TModExitMsg(m,p)    if (g_TraceEnabled && \
                                    ((g_TraceModules & (m)) != 0) && \
                                    (_levelTrace <= g_TraceLevel)) \
                                { \
                                    TracePrefix(_pszFuncName, false, false); \
                                    TPrintfLine p; \
                                }
    #define TModExit(m)         if (g_TraceEnabled && \
                                    ((g_TraceModules & (m)) != 0) && \
                                    (_levelTrace <= g_TraceLevel)) \
                                { \
                                    TracePrefix(_pszFuncName, false, true); \
                                }
    #define TModMsg(m,e,p)      if (g_TraceEnabled && \
                                    ((g_TraceModules & (m)) != 0) && \
                                    ((e) <= g_MsgLevel)) \
                                { \
                                    MsgPrefix(_pszFuncName, e); \
                                    TPrintfLine p; \
                                }
    #define TraceInit(m,l,e)    { \
                                    g_TraceModules = (m); \
                                    g_TraceLevel = (l); \
                                    g_MsgLevel = (e); \
                                    g_TraceEnabled = false; \
                                    g_TraceTime = nPgmTime; \
                                }
    #define TEnable(b)          g_TraceEnabled = b
    #define TFuncName(s)        char *_pszFuncName = s
    #define TLevel(l)           int _levelTrace = l
    #define TEnterMsg(p)        TModEnterMsg(MOD_ID, p)
    #define TEnter()            TModEnter(MOD_ID)
    #define TExitMsg(p)         TModExitMsg(MOD_ID, p)
    #define TExit()             TModExit(MOD_ID)
    #define TMsg(e,p)           TModMsg(MOD_ID, e, p)
    #define TFatal(p)           TModMsg(MOD_ID, FATAL, p)
    #define TErr(p)             TModMsg(MOD_ID, ERR, p)
    #define TWarn(p)            TModMsg(MOD_ID, WARN, p)
    #define TInfo(p)            TModMsg(MOD_ID, INFO, p)
    #define TVerbose(p)         TModMsg(MOD_ID, VERBOSE, p)
    #define TMsgPeriod(t,p)     { \
                                    static unsigned long _nextTime = nPgmTime; \
                                    if (nPgmTime >= _nextTime) \
                                    { \
                                        _nextTime = nPgmTime + (t); \
                                        TModMsg(MOD_ID, INFO, p); \
                                    } \
                                }
    #define TSampling(p)        TMsgPeriod(SAMPLING_PERIOD, p)
    #define TAssertPeriod(t,p,r) { \
                                    unsigned long _currTime = nPgmTime; \
                                    unsigned long _period = _currTime - (t); \
                                    t = _currTime; \
                                    if (abs(_period - (p)) > (r)) \
                                    { \
                                        TWarn(("AssertPeriod=%f", _period)); \
                                    } \
                                }
    #define TPeriodStart()      if (nPgmTime >= g_TraceTime) \
                                { \
                                    g_TraceTime = nPgmTime + TRACE_PERIOD; \
                                    TEnable(true); \
                                }
    #define TPeriodEnd()        TEnable(false)
#else
    #define TraceInit(m,l,e)
    #define TEnable(b)
    #define TFuncName(s)
    #define TLevel(l)
    #define TEnterMsg(p)
    #define TEnter()
    #define TExitMsg(p)
    #define TExit()
    #define TMsg(e,p)
    #define TFatal(p)
    #define TErr(p)
    #define TWarn(p)
    #define TInfo(p)
    #define TVerbose(p)
    #define TMsgPeriod(t,p)
    #define TSampling(p)
    #define TAssertPeriod(t,p,r)
    #define TPeriodStart()
    #define TPeriodEnd()
#endif  //ifdef _DEBUG_TRACE

#ifdef _DEBUG_TRACE

unsigned long   g_TraceModules = 0;
int             g_TraceLevel = 0;
int             g_MsgLevel = 0;
int             g_IndentLevel = 0;
bool            g_TraceEnabled = false;
unsigned long   g_TraceTime = 0;

/**
 *  This function prints the trace prefix string to the debug stream.
 *
 *  @param pszFuncName Specifies the function name.
 *  @param fEnter Specifies whether this is a TEnter or TExit.
 *  @param fNewLine Specifies whether it should print a newline.
 */
void
TracePrefix(
    char *pszFuncName,
    bool fEnter,
    bool fNewLine
    )
{
    if (fEnter)
    {
        g_IndentLevel++;
    }

    for (int i = 0; i < g_IndentLevel; ++i)
    {
        TPrintf("| ");
    }

    TPrintf(pszFuncName);

    if (fEnter)
    {
        TPrintf(fNewLine? "()\n": "(");
    }
    else
    {
        TPrintf(fNewLine? "!\n": "!");
        g_IndentLevel--;
    }

    return;
}   //TracePrefix

/**
 *  This function prints the message prefix string to the debug stream.
 *
 *  @param pszFuncName Specifies the function name.
 *  @param msgLevel Specifies the message level.
 */
void
MsgPrefix(
    char *pszFuncName,
    int msgLevel
    )
{
    char *pszPrefix;

    switch (msgLevel)
    {
        case FATAL:
            pszPrefix = "_Fatal:";
            break;

        case ERR:
            pszPrefix = "_Err:";
            break;

        case WARN:
            pszPrefix = "_Warn:";
            break;

        case INFO:
            pszPrefix = "_Info:";
            break;

        case VERBOSE:
            pszPrefix = "_Verbose:";
            break;

        default:
            pszPrefix = "_Unk:";
            break;
    }
    TPrintf("%s%s", pszFuncName, pszPrefix);

    return;
}   //MsgPrefix

#endif  //ifdef _DEBUG_TRACE
#endif  //ifndef _DBGTRACE_H
