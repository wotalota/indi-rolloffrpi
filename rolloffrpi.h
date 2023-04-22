/*
 Edited version of the Dome Simulator
 Copyright(c) 2014 Jasem Mutlaq. All rights reserved.
 Copyright(c) 2023 Jasem Mutlaq. All rights reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

#include "indidome.h"
#include <pigpiod_if2.h>

class RollOffIno : public INDI::Dome
{
  public:
    RollOffIno();

    virtual ~RollOffIno() = default;
    virtual bool initProperties() override;
    virtual void ISGetProperties(const char *dev) override;
    virtual bool ISNewNumber(const char *dev,const char *name,double values[],char *names[],int n) override;
    const char *getDefaultName() override;
    bool updateProperties() override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    virtual bool saveConfigItems(FILE *fp) override;
    virtual bool ISSnoopDevice(XMLEle *root) override;
    virtual bool Handshake() override;

  protected:
    bool Connect() override;
    bool Disconnect() override;
    void TimerHit() override;
    virtual IPState Move(DomeDirection dir, DomeMotionCommand operation) override;
    virtual IPState Park() override;
    virtual IPState UnPark() override;
    virtual bool Abort() override;
    virtual bool getFullOpenedLimitSwitch(bool*);
    virtual bool getFullClosedLimitSwitch(bool*);

  private:
    void updateRoofStatus();
    bool getRoofLockedSwitch(bool*);
    bool getRoofAuxSwitch(bool*);
    bool setRoofLock(bool switchOn);
    bool setRoofAux(bool switchOn);
    bool readRoofSwitch(const char* roofSwitchId, bool* result);
    bool roofOpen();
    bool roofClose();
    bool roofAbort();
    bool pushRoofButton(const char*, bool switchOn, bool ignoreLock);
    bool initRoofProperties();
    void gpioPinSet();
        //    bool initialContact();
        //    bool evaluateResponse(char*, bool*);
    bool writeIno(const char*);
    bool readIno(char*);
    void msSleep(int);
    bool setupConditions();
    float CalcTimeLeft(timeval);
    double MotionRequest { 0 };
    struct timeval MotionStart { 0, 0 };
    bool contactEstablished = false;
    bool roofOpening = false;
    bool roofClosing = false;
    ILight RoofStatusL[5];
    ILightVectorProperty RoofStatusLP;
    enum { ROOF_STATUS_OPENED, ROOF_STATUS_CLOSED, ROOF_STATUS_MOVING, ROOF_STATUS_LOCKED, ROOF_STATUS_AUXSTATE };

    ISwitch LockS[2];
    ISwitchVectorProperty LockSP;
    enum { LOCK_ENABLE, LOCK_DISABLE };

    ISwitch AuxS[2];
    ISwitchVectorProperty AuxSP;
    enum { AUX_ENABLE, AUX_DISABLE };

    ISState fullyOpenedLimitSwitch {ISS_OFF};
    ISState fullyClosedLimitSwitch {ISS_OFF};
    ISState roofLockedSwitch {ISS_OFF};
    ISState roofAuxiliarySwitch {ISS_OFF};
    INumber RoofTimeoutN[1] {};
    INumberVectorProperty RoofTimeoutNP;
    enum { EXPIRED_CLEAR, EXPIRED_OPEN, EXPIRED_CLOSE };
    unsigned int roofTimedOut;
    bool simRoofOpen = false;
    bool simRoofClosed = true;
    unsigned int communicationErrors = 0;
    int limitMsg = 0;
    bool xmlParkData = false;

#define ROOF_OPENED_SWITCH "OPENED"
#define ROOF_CLOSED_SWITCH "CLOSED"
#define ROOF_LOCKED_SWITCH "LOCKED"
#define ROOF_AUX_SWITCH    "AUXSTATE"

#define ROOF_OPEN_RELAY     "OPEN"
#define ROOF_CLOSE_RELAY    "CLOSE"
#define ROOF_ABORT_RELAY    "ABORT"
#define ROOF_LOCK_RELAY     "LOCK"
#define ROOF_AUX_RELAY      "AUXSET"

#if defined ROLLOFF_RPI

#define MAX_OUT_DEFS 5  // Max # of definitions of output commands
#define MAX_OUT_OPS 6   // Open, Close, Abort, Lock, Aux-request, Unused
#define MAX_OUT_ACTIVE_LIMIT 5 // Max number of definitions of how long to close relay
#define MAX_INP_DEFS 4  // Max # of definitions of input responses
#define MAX_INP_OPS 5   // Fully-opened, Fully-Closed, Locked, Aux-response, Unused

    const char  *GPIO_TAB = "Define GPIO";
    // Labels
    const std::string functionL = "Function ";
    const std::string outPinL = "Output GPIO";
    const std::string outActiveL = "Active When";
    const std::string activeLimitL = "Active Limit";
    const std::string responseL = "Response ";
    const std::string inpPinL = "Input GPIO #";
    const std::string inpActiveL = "Active When";

    // Names
    const std::string function = "OUTRELAY";
    const std::string outPin = "OUTGPIO";
    const std::string outActive = "OUTACT";
    const std::string activeLimit = "OUTLIMIT";
    const std::string response = "INPSWITCH";
    const std::string inpPin = "INPGPIO";
    const std::string inpActive = "INPACT";

    const char* inpOps[MAX_INP_OPS] = {ROOF_OPENED_SWITCH, ROOF_CLOSED_SWITCH, ROOF_LOCKED_SWITCH, ROOF_AUX_SWITCH, "Unused"};
    const char* outOps[MAX_OUT_OPS] = {ROOF_OPEN_RELAY, ROOF_CLOSE_RELAY, ROOF_ABORT_RELAY, ROOF_LOCK_RELAY, ROOF_AUX_RELAY, "Unused"};
    const char* outActiveLimit[MAX_OUT_ACTIVE_LIMIT] = {"0.1s", "0.25s", "0.5s", "0.75s", "No Limit"};
    int activeLimitMilli[MAX_OUT_ACTIVE_LIMIT] = {100, 250, 500, 750, 0};

    ISwitch outFunctionS[MAX_OUT_DEFS][MAX_OUT_OPS];
    ISwitchVectorProperty outFunctionSP[MAX_OUT_DEFS];

    INumber outPinNumberN[MAX_OUT_DEFS][1];
    INumberVectorProperty outPinNumberNP[MAX_OUT_DEFS];

    ISwitch outActivateWhenS[MAX_OUT_DEFS][2];
    ISwitchVectorProperty outActivateWhenSP[MAX_OUT_DEFS];

    ISwitch outActiveLimitS[MAX_OUT_DEFS][MAX_OUT_ACTIVE_LIMIT];
    ISwitchVectorProperty outActiveLimitSP[MAX_OUT_DEFS];

    ISwitch inpFunctionS[MAX_INP_DEFS][MAX_INP_OPS];
    ISwitchVectorProperty inpFunctionSP[MAX_INP_DEFS];

    INumber inpPinNumberN[MAX_INP_DEFS][1];
    INumberVectorProperty inpPinNumberNP[MAX_INP_DEFS];

    ISwitch inpActivateWhenS[MAX_INP_DEFS][2];
    ISwitchVectorProperty inpActivateWhenSP[MAX_INP_DEFS];

    int pi_id;        // pigpiod RPi identifier
    bool roofPropInit = false;
};

