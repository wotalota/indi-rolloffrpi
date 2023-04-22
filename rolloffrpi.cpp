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

/*
 * Modified version of the rolloff roof simulator. Two modes of working depending on build time option.
 * 1. Communicate with remote Arduino interfacing to a roof controller or motor control interface.
 * Uses a simple text string protocol to send messages to an Arduino. The Arduino's USB connection is set
 * by default to 38400 baud. An internet connection is also available with the Arduino using WiFi.
 * The Arduino code determines which pins open/close a relay to start/stop the roof motor, and read
 * state of switches indicating if the roof is  opened or closed.
 *
 * 2. Run the driver on Raspberry pi and make use of its GPIO pin connections to a roof controller.
 * Raspberry pi GPIO use relies upon the pigpio library and installation of the pigpiod daemon to access
 * the GPIO pins without root privilege.
 */

#include "rolloffino.h"
#include "indicom.h"
#include "termios.h"

#include <cmath>
#include <cstring>
#include <ctime>
#include <memory>

#define ROLLOFF_DURATION 30               // Seconds until Roof is fully opened or closed
#define INITIAL_TIMING   500             // Init period at startup
#define INACTIVE_TIMING  1000             // Polling period for updating status lights
#define ACTIVE_POLL_MS   500              // Polling period in milliseconds when roof is in motion
#define ROR_D_PRESS      1000             // Milliseconds after issuing command allowed for a response
#define MAX_CNTRL_COM_ERR 10              // Maximum consecutive errors communicating with Arduino

// Arduino controller interface limits
#define MAXINOCMD        15          // Command buffer
#define MAXINOTARGET     15          // Target buffer
#define MAXINOVAL        127         // Value bufffer, sized to contain NAK error strings
#define MAXINOLINE       63          // Sized to contain outgoing command requests
#define MAXINOBUF        255         // Sized for maximum overall input / output
#define MAXINOERR        255         // System call error message buffer
#define MAXINOWAIT       2           // seconds

// Driver version id
#define VERSION_ID      "20221105"

// We declare an auto pointer to RollOffIno.
std::unique_ptr<RollOffIno> rollOffIno(new RollOffIno());

RollOffIno::RollOffIno()
{
    SetDomeCapability(DOME_CAN_ABORT | DOME_CAN_PARK);           // Need the DOME_CAN_PARK capability for the scheduler
    setDomeConnection(CONNECTION_NONE);
}

bool RollOffIno::ISSnoopDevice(XMLEle *root)
{
    return INDI::Dome::ISSnoopDevice(root);
}

/**************************************************************************************
** INDI is asking us for our default device name.
** Check that it matches Ekos selection menu and ParkData.xml names
***************************************************************************************/
const char *RollOffIno::getDefaultName()
{
    return (const char *)"RollOff ino";
}

RollOffIno::~RollOffIno()
{
    pigpio_stop(pi_id);
}

/*
 Called from an INDI client.
 */
void RollOffIno::ISGetProperties(const char *dev)
{
    INDI::Dome::ISGetProperties(dev);
    defineProperty(&RoofTimeoutNP);
    loadConfig(true, "ROOF_TIMEOUT");

    for (int i = 0; i < MAX_OUT_DEFS; i++)
    {
        defineProperty(&outFunctionSP[i]);
        loadConfig(true, outFunctionSP[i].name);
        defineProperty(&outPinNumberNP[i]);
        loadConfig(true, outPinNumberNP[i].name);
        defineProperty(&outActivateWhenSP[i]);
        loadConfig(true, outActivateWhenSP[i].name);
        defineProperty(&outActiveLimitSP[i]);
        loadConfig(true, outActiveLimitSP[i].name);
    }
    for (int i = 0; i < MAX_INP_DEFS; i++)
    {
        defineProperty(&inpFunctionSP[i]);
        loadConfig(true, inpFunctionSP[i].name);
        defineProperty(&inpPinNumberNP[i]);
        loadConfig(true, inpPinNumberNP[i].name);
        defineProperty(&inpActivateWhenSP[i]);
        loadConfig(true, inpActivateWhenSP[i].name);
    }
}

/**************************************************************************************
** INDI request to init properties. Connected Define properties to Ekos
***************************************************************************************/

bool RollOffIno::initProperties()
{
    INDI::Dome::initProperties();
    IUFillSwitch(&LockS[LOCK_DISABLE], "LOCK_DISABLE", "Off", ISS_ON);
    IUFillSwitch(&LockS[LOCK_ENABLE], "LOCK_ENABLE", "On", ISS_OFF);
    IUFillSwitchVector(&LockSP, LockS, 2, getDeviceName(), "LOCK", "Lock", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillSwitch(&AuxS[AUX_DISABLE], "AUX_DISABLE", "Off", ISS_ON);
    IUFillSwitch(&AuxS[AUX_ENABLE], "AUX_ENABLE", "On", ISS_OFF);
    IUFillSwitchVector(&AuxSP, AuxS, 2, getDeviceName(), "AUX", "Auxiliary", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillLight(&RoofStatusL[ROOF_STATUS_OPENED], "ROOF_OPENED", "Opened", IPS_IDLE);
    IUFillLight(&RoofStatusL[ROOF_STATUS_CLOSED], "ROOF_CLOSED", "Closed", IPS_IDLE);
    IUFillLight(&RoofStatusL[ROOF_STATUS_MOVING], "ROOF_MOVING", "Moving", IPS_IDLE);
    IUFillLight(&RoofStatusL[ROOF_STATUS_LOCKED], "ROOF_LOCK", "Roof Lock", IPS_IDLE);
    IUFillLight(&RoofStatusL[ROOF_STATUS_AUXSTATE], "ROOF_AUXILIARY", "Roof Auxiliary", IPS_IDLE);
    IUFillLightVector(&RoofStatusLP, RoofStatusL, 5, getDeviceName(), "ROOF STATUS", "Roof Status", MAIN_CONTROL_TAB, IPS_BUSY);

    IUFillNumber(&RoofTimeoutN[0], "ROOF_TIMEOUT", "Timeout in Seconds", "%3.0f", 1, 300, 1, 15);
    IUFillNumberVector(&RoofTimeoutNP, RoofTimeoutN, 1, getDeviceName(), "ROOF_MOVEMENT", "Roof Movement", OPTIONS_TAB, IP_RW,
                       60, IPS_IDLE);

    for (int i = 0; i < MAX_OUT_DEFS; i++)
    {
        for (int j = 0; j < MAX_OUT_OPS; j++) {
            IUFillSwitch(&outFunctionS[i][j], outOps[j], "", ISS_OFF);
        }
        IUFillSwitchVector(&outFunctionSP[i], outFunctionS[i], MAX_OUT_OPS, getDeviceName(), (function + std::to_string(i+1)).c_str(), (functionL + std::to_string(i+1)).c_str(),
                           GPIO_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

        IUFillNumber(&outPinNumberN[i][0], outPin.c_str(), "GPIO pin #", "%1.0f", 2, 27, 1, 0);
        IUFillNumberVector(&outPinNumberNP[i], outPinNumberN[i], 1, getDeviceName(), (outPin + std::to_string(i+1)).c_str(),outPinL.c_str(),
                           GPIO_TAB, IP_RW, 60, IPS_IDLE);

        IUFillSwitch(&outActivateWhenS[i][0], "High", "", ISS_OFF);
        IUFillSwitch(&outActivateWhenS[i][1], "Low", "", ISS_OFF);
        IUFillSwitchVector(&outActivateWhenSP[i], outActivateWhenS[i], 2, getDeviceName(), (outActive + std::to_string(i+1)).c_str(), outActiveL.c_str(),
                           GPIO_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

        for (int j = 0; j < MAX_OUT_ACTIVE_LIMIT; j++) {
            IUFillSwitch(&outActiveLimitS[i][j], outActiveLimit[j], "", ISS_OFF);
        }
        IUFillSwitchVector(&outActiveLimitSP[i], outActiveLimitS[i], MAX_OUT_ACTIVE_LIMIT, getDeviceName(), (activeLimit + std::to_string(i+1)).c_str(), (activeLimitL).c_str(),
                           GPIO_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
     }

    for (int i = 0; i < MAX_INP_DEFS; i++)
    {
        for (int j = 0; j < MAX_INP_OPS; j++) {
            IUFillSwitch(&inpFunctionS[i][j], inpOps[j], "", ISS_OFF);
        }
        IUFillSwitchVector(&inpFunctionSP[i], inpFunctionS[i], MAX_INP_OPS, getDeviceName(), (response + std::to_string(i+1)).c_str(), (responseL + std::to_string(i+1)).c_str(),
                           GPIO_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

        IUFillNumber(&inpPinNumberN[i][0], inpPin.c_str(), "GPIO pin #", "%1.0f", 2, 27, 1, 0);
        IUFillNumberVector(&inpPinNumberNP[i], inpPinNumberN[i], 1, getDeviceName(), (inpPin + std::to_string(i+1)).c_str(),inpPinL.c_str(),
                           GPIO_TAB, IP_RW, 60, IPS_IDLE);

        IUFillSwitch(&inpActivateWhenS[i][0], "High", "", ISS_OFF);
        IUFillSwitch(&inpActivateWhenS[i][1], "Low", "", ISS_OFF);
        IUFillSwitchVector(&inpActivateWhenSP[i], inpActivateWhenS[i], 2, getDeviceName(), (inpActive + std::to_string(i+1)).c_str(), inpActiveL.c_str(),
                           GPIO_TAB,IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    }
    SetParkDataType(PARK_NONE);
    addAuxControls();               // This is for additional standard controls
    return true;
}

/**************************************************************************************
** Client is asking us to establish connection to the device
***************************************************************************************/
bool RollOffIno::Connect()
{
    bool status = true;
    limitMsg = 0;

    // Establish session with igpiod daemon
    pi_id = pigpio_start(NULL, NULL);
    if (pi_id < 0)
    {
        LOGF_ERROR("Unable to contact the pigpiod system service, %s" , pigpio_error(pi_id));
        LOGF_DEBUG("pigpiod_if2 version %lu.", pigpiod_if_version());
        return false;
    }
// Bypass the actual connection attempt, using GPIO pins instead
//    status = INDI::Dome::Connect();
    contactEstablished = true;
    gpioPinSet();
    SetTimer(INITIAL_TIMING);
    return status;
}


/************************************************************************************
 * Called from Dome, BaseDevice to establish contact with device
 ************************************************************************************/
bool RollOffIno::Handshake()
{
    LOGF_DEBUG("Driver id: %s", VERSION_ID);
    return true;
}

/**************************************************************************************
** Client is asking to terminate connection to the device
***************************************************************************************/
bool RollOffIno::Disconnect()
{
    pigpio_stop(pi_id);
    return true;
}

/********************************************************************************************
** INDI request to update the properties because there is a change in CONNECTION state
** This function is called whenever the device is connected or disconnected.
** Connected: Define and register property to the client
** Disconnect: Remove property from client
*********************************************************************************************/
bool RollOffIno::updateProperties()
{
    INDI::Dome::updateProperties();
    if (isConnected())
    {
        xmlParkData = InitPark();
        defineProperty(&LockSP);            // Lock Switch,
        defineProperty(&AuxSP);             // Aux Switch,
        defineProperty(&RoofStatusLP);      // All the roof status lights
        defineProperty(&RoofTimeoutNP);

        for (int i = 0; i < MAX_OUT_DEFS; i++)
        {
            defineProperty(&outFunctionSP[i]);
            defineProperty(&outPinNumberNP[i]);
            defineProperty(&outActivateWhenSP[i]);
            defineProperty(&outActiveLimitSP[i]);
        }

        for (int i = 0; i < MAX_INP_DEFS; i++)
        {
            defineProperty(&inpFunctionSP[i]);
            defineProperty(&inpPinNumberNP[i]);
            defineProperty(&inpActivateWhenSP[i]);
        }

        setupConditions();                               // Get state of Dome::
    }
    else
    {
        deleteProperty(RoofStatusLP.name);  // Delete the roof status lights
        deleteProperty(LockSP.name);        // Delete the Lock Switch buttons
        deleteProperty(AuxSP.name);         // Delete the Auxiliary Switch buttons
        deleteProperty(RoofTimeoutNP.name);

        for (int i = 0; i < MAX_OUT_DEFS; i++)
        {
            deleteProperty(outFunctionSP[i].name);
            deleteProperty(outPinNumberNP[i].name);
            deleteProperty(outActivateWhenSP[i].name);
            deleteProperty(outActiveLimitSP[i].name);
        }

        for (int i = 0; i < MAX_INP_DEFS; i++)
        {
            deleteProperty(inpFunctionSP[i].name);
            deleteProperty(inpPinNumberNP[i].name);
            deleteProperty(inpActivateWhenSP[i].name);
        }
    }
    return true;
}

/**************************************************************************************
** Called when save button pushed
***************************************************************************************/
bool RollOffIno::saveConfigItems(FILE *fp)
{
    bool status = INDI::Dome::saveConfigItems(fp);
    IUSaveConfigNumber(fp, &RoofTimeoutNP);

    for (int i = 0; i < MAX_OUT_DEFS; i++)
    {
        IUSaveConfigSwitch(fp, &outFunctionSP[i]);
        IUSaveConfigNumber(fp, &outPinNumberNP[i]);
        IUSaveConfigSwitch(fp, &outActivateWhenSP[i]);
        IUSaveConfigSwitch(fp, &outActiveLimitSP[i]);
    }

    for (int i = 0; i < MAX_INP_DEFS; i++)
    {
        IUSaveConfigSwitch(fp, &inpFunctionSP[i]);
        IUSaveConfigNumber(fp, &inpPinNumberNP[i]);
        IUSaveConfigSwitch(fp, &inpActivateWhenSP[i]);
    }
    return status;
}

/*
 * Called by infrastructure when number property modified
 * IDset* informs Client of the change
 */
bool RollOffIno::ISNewNumber(const char *dev,const char *name,double values[],char *names[],int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (!strcmp(RoofTimeoutNP.name, name))
        {
            IUUpdateNumber(&RoofTimeoutNP, values, names, n);
            RoofTimeoutNP.s = IPS_OK;
            IDSetNumber(&RoofTimeoutNP, nullptr);
            return true;
        }

        // Look for GPIO definition numbers
        for (int i=0; i < MAX_OUT_DEFS; i++)
        {
            if (!strcmp(name, outPinNumberNP[i].name))
            {
                IUUpdateNumber(&outPinNumberNP[i], values, names, n);
                outPinNumberNP[i].s = IPS_OK;
                IDSetNumber(&outPinNumberNP[i], nullptr);
                return true;
            }
            else
            {
                if (!strcmp(name, inpPinNumberNP[i].name))
                {
                    IUUpdateNumber(&inpPinNumberNP[i], values, names, n);
                    inpPinNumberNP[i].s = IPS_OK;
                    IDSetNumber(&inpPinNumberNP[i], nullptr);
                    return true;
                }
            }
        }
    }
    return INDI::Dome::ISNewNumber(dev,name,values,names,n);
}

/********************************************************************************************
 * Called by infrastructure when switch property modified
 * IDset* informs Client of the change
*********************************************************************************************/
bool RollOffIno::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    bool switchOn = false;
    // Make sure the call is for our device
    if(dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Check if the call for our Lock switch
        if (strcmp(name, LockSP.name) == 0)
        {
            // Find out which state is requested by the client
            const char *actionName = IUFindOnSwitchName(states, names, n);
            // If it is the same state as actionName, then we do nothing. i.e.
            // if actionName is LOCK_ON and our Lock switch is already on, we return
            int currentLockIndex = IUFindOnSwitchIndex(&LockSP);
            DEBUGF(INDI::Logger::DBG_SESSION, "Lock state Requested: %s, Current: %s", actionName, LockS[currentLockIndex].name);
            if (!strcmp(actionName, LockS[currentLockIndex].name))
            {
                DEBUGF(INDI::Logger::DBG_SESSION, "Lock switch is already %s", LockS[currentLockIndex].label);
                LockSP.s = IPS_IDLE;
                IDSetSwitch(&LockSP, NULL);
                return true;
            }
            // Update the switch state
            IUUpdateSwitch(&LockSP, states, names, n);
            currentLockIndex = IUFindOnSwitchIndex(&LockSP);
            LockSP.s = IPS_OK;
            IDSetSwitch(&LockSP, nullptr);
            if (strcmp(LockS[currentLockIndex].name, "LOCK_ENABLE") == 0)
                switchOn = true;
            setRoofLock(switchOn);
            updateRoofStatus();
            return true;
        }

        // Check if the call for our Aux switch
        if (strcmp(name, AuxSP.name) == 0)
        {
            // Find out which state is requested by the client
            const char *actionName = IUFindOnSwitchName(states, names, n);
            // If it is the same state as actionName, then we do nothing. i.e.
            // if actionName is AUX_ON and our Aux switch is already on, we return
            int currentAuxIndex = IUFindOnSwitchIndex(&AuxSP);
            DEBUGF(INDI::Logger::DBG_SESSION, "Auxiliary state Requested: %s, Current: %s", actionName, AuxS[currentAuxIndex].name);
            if (!strcmp(actionName, AuxS[currentAuxIndex].name))
            {
                DEBUGF(INDI::Logger::DBG_SESSION, "Auxiliary switch is already %s", AuxS[currentAuxIndex].label);
                AuxSP.s = IPS_IDLE;
                IDSetSwitch(&AuxSP, NULL);
                return true;
            }
            // Update the switch
            IUUpdateSwitch(&AuxSP, states, names, n);
            currentAuxIndex = IUFindOnSwitchIndex(&AuxSP);
            AuxSP.s = IPS_OK;
            IDSetSwitch(&AuxSP, nullptr);
            if (strcmp(AuxS[currentAuxIndex].name, "AUX_ENABLE") == 0)
                switchOn = true;
            setRoofAux(switchOn);
            updateRoofStatus();
            return true;
        }

        // Look if GPIO definition relay
        for (int i=0; i < MAX_OUT_DEFS; i++)
        {
            if (!strcmp(name, outFunctionSP[i].name))
            {
                IUUpdateSwitch(&outFunctionSP[i], states, names, n);
                outFunctionSP[i].s = IPS_OK;
                IDSetSwitch(&outFunctionSP[i], nullptr);
                return true;
            }
            else if (!strcmp(name, outActivateWhenSP[i].name))
            {
                IUUpdateSwitch(&outActivateWhenSP[i], states, names, n);
                outActivateWhenSP[i].s = IPS_OK;
                IDSetSwitch(&outActivateWhenSP[i], nullptr);
                return true;
            }
            else if (!strcmp(name, outActiveLimitSP[i].name))
            {
                IUUpdateSwitch(&outActiveLimitSP[i], states, names, n);
                outActiveLimitSP[i].s = IPS_OK;
                IDSetSwitch(&outActiveLimitSP[i], nullptr);
                return true;
            }
        }

        // Look if GPIO definition switch
        for (int i=0; i < MAX_INP_DEFS; i++)
        {
            if (!strcmp(name, inpFunctionSP[i].name))
            {
                IUUpdateSwitch(&inpFunctionSP[i], states, names, n);
                inpFunctionSP[i].s = IPS_OK;
                IDSetSwitch(&inpFunctionSP[i], nullptr);
                return true;
            }
            else if (!strcmp(name, inpActivateWhenSP[i].name))
            {
                IUUpdateSwitch(&inpActivateWhenSP[i], states, names, n);
                inpActivateWhenSP[i].s = IPS_OK;
                IDSetSwitch(&inpActivateWhenSP[i], nullptr);
                return true;
            }
        }
    }
    return INDI::Dome::ISNewSwitch(dev, name, states, names, n);
}

/********************************************************************************************
** Set GPIO pins mode and pull up or pull down settings to match the definition
*********************************************************************************************/
void RollOffIno::gpioPinSet()
{
    int required = 0;
    unsigned int gpio;
    int err;
    int pud;
    const char* sActivate;
    const char* sPud;
    const char* sTime = "";

    LOG_DEBUG("Summary of GPIO pins defined: ");
    for (int i = 0; i < MAX_OUT_DEFS; i++)                 // definition 1, 2, ..
    {
        for (int j = 0; j < MAX_OUT_OPS; j++)              // Open, Close, . .
        {
            // Find the function associated with this definition position
            if (outFunctionS[i][j].s != ISS_ON || (strcmp(outFunctionS[i][j].name, "Unused") == 0))
                continue;

            gpio = outPinNumberN[i][0].value;
            if (((strcmp(outFunctionS[i][j].name, outOps[0]) == 0) ||
                 (strcmp(outFunctionS[i][j].name, outOps[1]) == 0)) && gpio > 2)
                required++;

            // Set pin to output mode
            if ((err = set_mode(pi_id, gpio, PI_OUTPUT)) != 0)
            {
                LOGF_ERROR("Failed to set %s GPIO pin %d to output mode %s", outFunctionS[i][j].name, gpio,
                           pigpio_error(err));
                break;
            }

            // For output pins no need for pullup/pulldown. Clear any prior setting
            pud = PI_PUD_OFF;
            if ((err = set_pull_up_down(pi_id, gpio, PI_PUD_OFF)) != 0)
            {
                LOGF_ERROR("Failed to set %s GPIO pin %d internal resistor", outFunctionS[i][j].name, gpio,
                           pigpio_error(err));
                break;
            }

            // For debug summary
            if (outActivateWhenS[i][0].s == ISS_ON)
                sActivate = outActivateWhenS[i][0].name;
            else
                sActivate = outActivateWhenS[i][1].name;
            for (int k = 0; k < MAX_OUT_ACTIVE_LIMIT; k++)
            {
                if (outActiveLimitS[i][k].s == ISS_ON)
                {
                    sTime = outActiveLimitS[i][k].name;
                    break;
                }
            }

            // Set relay off
            if ((strcmp(sActivate, "High") == 0))
            {
                err = gpio_write(pi_id, gpio, 0);
            }
            else
            {
                err = gpio_write(pi_id, gpio, 1);
            }
            if (err != 0)
                LOGF_WARN("GPIO write failed for %s, %d, returned: %s", outFunctionS[i][j].name, gpio, pigpio_error(err));

            // Summarize the settings for this function
            LOGF_DEBUG("Position %s, Function %s, Pin %d, Mode Output, Activate %s, Resistor off, Timed %s",
                       outFunctionSP[i].name, outFunctionS[i][j].name, gpio, sActivate, sTime);
        }   // End Open, Close, . .
    }       // End definition 1, 2, . .

    for (int i = 0; i < MAX_INP_DEFS; i++)                 // definition 1, 2, . .
    {
        for (int j = 0; j < MAX_INP_OPS; j++)              // opened, closed, . .
        {
            // Find the function associated with this definition position
            if (inpFunctionS[i][j].s != ISS_ON || (strcmp(inpFunctionS[i][j].name, "Unused") == 0))
                continue;

            gpio = inpPinNumberN[i][0].value;
            if (((strcmp(inpFunctionS[i][j].name, inpOps[0]) == 0) ||
                 (strcmp(inpFunctionS[i][j].name, inpOps[1]) == 0)) && gpio > 2)
                required++;

             if ((err = set_mode(pi_id, gpio, PI_INPUT)) != 0)
            {
                LOGF_ERROR("Failed to set %s GPIO pin %d to input mode %s", inpFunctionS[i][j].name, gpio, pigpio_error(err));
                break;
            }
            if (inpActivateWhenS[i][0].s == ISS_ON)
                sActivate = inpActivateWhenS[i][0].name;
            else
                sActivate = inpActivateWhenS[i][1].name;
            if (strcmp(sActivate, "High") == 0)
            {
                pud = PI_PUD_DOWN;
                sPud = "down";
            }
            else
            {
                pud = PI_PUD_UP;
                sPud = "up";
            }
            if ((err = set_pull_up_down(pi_id, gpio, pud)) != 0)
            {
                LOGF_ERROR("Failed to set %s GPIO pin %d internal resistor", inpFunctionS[i][j].name, gpio,
                           pigpio_error(err));
                break;
            }

            // Summarize the settings for this function
            LOGF_DEBUG("Position %s, Function %s, Pin %d, Mode Input, Activate %s, Resistor pull %s",
                       inpFunctionSP[i].name, inpFunctionS[i][j].name, gpio, sActivate, sPud);
        }   // End opened, closed. . .
    }       // End definition 1, 2, . .

    // Minimal is open, close, opened, closed
    if (required < 4)
        LOG_ERROR("The GPIO definitions must include relays OPEN, CLOSE, and switches OPENED, CLOSED");
    return;
}

/********************************************************************************************
** Establish conditions on a connect.
*********************************************************************************************/
bool RollOffIno::setupConditions()
{
    if (contactEstablished)
    {
        LOG_DEBUG("Obtaining initial state of opened and closed switches");
        updateRoofStatus();
    }
    else
    {
        LOG_DEBUG("Could not read opened and closed switch state. Default to previous settings from Dome ParkedData.xml");
        // If we have Dome parking data
        if (xmlParkData)
        {
            DEBUG(INDI::Logger::DBG_SESSION, "Dome parking data was obtained");
            if (isParked())
            {
                fullyClosedLimitSwitch = ISS_ON;
                fullyOpenedLimitSwitch = ISS_OFF;
            }
            else
            {
                fullyOpenedLimitSwitch = ISS_ON;
                fullyClosedLimitSwitch = ISS_OFF;
            }
        }
            // If we do not have Dome parking data
        else
        {
            DEBUG(INDI::Logger::DBG_SESSION, "Dome parking data was not obtained");
            fullyOpenedLimitSwitch = ISS_OFF;
            fullyClosedLimitSwitch = ISS_OFF;
        }
    }

    Dome::DomeState curState = getDomeState();
    switch (curState)
    {
        case DOME_UNKNOWN:
            DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_UNKNOWN");
            break;
        case    DOME_ERROR:
            DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_ERROR");
            break;
        case DOME_IDLE:
            DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_IDLE ");
            break;
        case     DOME_MOVING:
            DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_MOVING");
            break;
        case     DOME_SYNCED:
            DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_SYNCED");
            break;
        case     DOME_PARKING:
            DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_PARKING");
            break;
        case    DOME_UNPARKING:
            DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_UNPARKING");
            break;
        case    DOME_PARKED:
            if (isParked())
            {
                DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_PARKED");
            }
            else
            {
                DEBUG(INDI::Logger::DBG_SESSION,"Dome state is DOME_PARKED but Dome status is unparked");
            }
            break;
        case    DOME_UNPARKED:
            if (!isParked())
            {
                DEBUG(INDI::Logger::DBG_SESSION,"Dome state: DOME_UNPARKED");
            }
            else
            {
                DEBUG(INDI::Logger::DBG_SESSION,"Dome state is DOME_UNPARKED but Dome status is parked");
            }
            break;
    }

    // Report apparent inconsistency in Dome state, parked status and roof state.
    if (isParked())
    {
        if (fullyOpenedLimitSwitch == ISS_ON)
        {
            DEBUG(INDI::Logger::DBG_WARNING,"Dome indicates it is parked but roof opened switch is set.");
        }
        else if (fullyClosedLimitSwitch == ISS_OFF)
        {
            DEBUG(INDI::Logger::DBG_WARNING,"Dome indicates it is parked but roof closed switch not set.");
        }
        else
        {
            // When Dome status agrees with roof status (closed), but Dome state differs
            if (curState != DOME_PARKED)
            {
                DEBUG(INDI::Logger::DBG_SESSION,"Dome status indicates it is parked but its state is not DOME_PARKED.");
            }
        }
    }
    else
    {
        if (fullyClosedLimitSwitch == ISS_ON)
        {
            DEBUG(INDI::Logger::DBG_WARNING,"Dome status indicates unparked but roof closed switch is set.");
        }
        else if (fullyOpenedLimitSwitch == ISS_OFF)
        {
            DEBUG(INDI::Logger::DBG_WARNING,"Dome indicates it is unparked but roof open switch is not set.");
        }
        else
        {
            if (curState != DOME_UNPARKED)
            {
                DEBUG(INDI::Logger::DBG_SESSION,"Dome status indicates it is unparked but its state is not DOME_UNPARKED.");
            }
        }
    }
    return true;
}

void RollOffIno::updateRoofStatus()
{
    bool auxiliaryState = false;
    bool lockedState = false;
    bool openedState = false;
    bool closedState = false;
    getFullOpenedLimitSwitch(&openedState);
    getFullClosedLimitSwitch(&closedState);
    getRoofLockedSwitch(&lockedState);
    getRoofAuxSwitch(&auxiliaryState);

    if (!openedState && !closedState && !roofOpening && !roofClosing)
    {
        if (limitMsg <= 10)
        {
            limitMsg++;
            LOG_WARN ("Roof stationary, neither opened or closed, adjust to match PARK button");
        }
        else if (limitMsg == 11)
        {
            limitMsg++;
            LOG_ERROR ("Roof stationary, not opened or closed. Will stop reporting this error.");
        }
    }
    else
        limitMsg = 0;

    if (openedState && closedState)
        DEBUG(INDI::Logger::DBG_WARNING, "Roof showing it is both opened and closed according to the controller");

    RoofStatusL[ROOF_STATUS_AUXSTATE].s = IPS_IDLE;
    RoofStatusL[ROOF_STATUS_LOCKED].s = IPS_IDLE;
    RoofStatusL[ROOF_STATUS_OPENED].s = IPS_IDLE;
    RoofStatusL[ROOF_STATUS_CLOSED].s = IPS_IDLE;
    RoofStatusL[ROOF_STATUS_MOVING].s = IPS_IDLE;
    RoofStatusLP.s = IPS_IDLE;

    if (auxiliaryState)
    {
        RoofStatusL[ROOF_STATUS_AUXSTATE].s = IPS_OK;
    }
    if (lockedState)
    {
        RoofStatusL[ROOF_STATUS_LOCKED].s = IPS_ALERT;         // Red to indicate lock is on
        if (closedState)
        {
            RoofStatusL[ROOF_STATUS_CLOSED].s = IPS_OK;            // Closed and locked roof status is normal
            RoofStatusLP.s = IPS_OK;                               // Summary roof status
        }
        // An actual roof lock would not be expected unless roof was closed.
        // Although the controller might be using it to prevent motion for some other reason.
        else if (openedState)
        {
            RoofStatusL[ROOF_STATUS_OPENED].s = IPS_OK;         // Possible, rely on open/close lights to indicate situation
            RoofStatusLP.s = IPS_OK;
        }
        else if (roofOpening || roofClosing)
        {
            RoofStatusLP.s = IPS_ALERT;                            // Summary roof status
            RoofStatusL[ROOF_STATUS_MOVING].s = IPS_ALERT;         // Should not be moving while locked
        }
    }
    else
    {
        if (openedState || closedState)
        {
            if (openedState && !closedState)
            {
                roofOpening = false;
                RoofStatusL[ROOF_STATUS_OPENED].s = IPS_OK;
                RoofStatusLP.s = IPS_OK;
            }
            if (closedState && !openedState)
            {
                roofClosing = false;
                RoofStatusL[ROOF_STATUS_CLOSED].s = IPS_OK;
                RoofStatusLP.s = IPS_OK;
            }
        }
        else if (roofOpening || roofClosing)
        {
            if (roofOpening)
            {
                RoofStatusL[ROOF_STATUS_OPENED].s = IPS_BUSY;
                RoofStatusL[ROOF_STATUS_MOVING].s = IPS_BUSY;
            }
            else if (roofClosing)
            {
                RoofStatusL[ROOF_STATUS_CLOSED].s = IPS_BUSY;
                RoofStatusL[ROOF_STATUS_MOVING].s = IPS_BUSY;
            }
            RoofStatusLP.s = IPS_BUSY;
        }

        // Roof is stationary, neither opened or closed
        else
        {
            if (roofTimedOut == EXPIRED_OPEN)
                RoofStatusL[ROOF_STATUS_OPENED].s = IPS_ALERT;
            else if (roofTimedOut == EXPIRED_CLOSE)
                RoofStatusL[ROOF_STATUS_CLOSED].s = IPS_ALERT;
            RoofStatusLP.s = IPS_ALERT;
        }
    }
    IDSetLight(&RoofStatusLP, nullptr);
}

/********************************************************************************************
** Each timer tick, if roof active
********************************************************************************************/
void RollOffIno::TimerHit()
{
    double timeleft = CalcTimeLeft(MotionStart);
    uint32_t delay = INACTIVE_TIMING;   // inactive timer setting to maintain roof status lights
    if (!isConnected())
        return; //  No need to reset timer if we are not connected anymore

    if (isSimulation())
    {
        if (timeleft - 5 <= 0)           // Use timeout approaching to set faux switch indicator
        {
            if (DomeMotionS[DOME_CW].s == ISS_ON)              // Opening
            {
                simRoofOpen = true;
                simRoofClosed = false;
            }
            else if (DomeMotionS[DOME_CCW].s == ISS_ON)        // Closing
            {
                simRoofClosed = true;
                simRoofOpen = false;
            }
        }
    }

    updateRoofStatus();

    if (DomeMotionSP.s == IPS_BUSY)
    {
        // Abort called stop movement.
        if (MotionRequest < 0)
        {
            DEBUG(INDI::Logger::DBG_WARNING, "Roof motion is stopped");
            setDomeState(DOME_IDLE);
        }
        else
        {
            // Roll off is opening
            if (DomeMotionS[DOME_CW].s == ISS_ON)
            {
                if (fullyOpenedLimitSwitch == ISS_ON)
                {
                    DEBUG(INDI::Logger::DBG_DEBUG, "Roof is open");
                    SetParked(false);
                }
                // See if time to open has expired.
                else if (timeleft <= 0)
                {
                    LOG_WARN("Time allowed for opening the roof has expired?");
                    setDomeState(DOME_IDLE);
                    roofOpening = false;
                    roofTimedOut = EXPIRED_OPEN;
                }
                else
                {
                    delay = ACTIVE_POLL_MS;           // opening active
                }
            }
            // Roll Off is closing
            else if (DomeMotionS[DOME_CCW].s == ISS_ON)
            {
                if (fullyClosedLimitSwitch == ISS_ON)
                {
                    DEBUG(INDI::Logger::DBG_DEBUG, "Roof is closed");
                    SetParked(true);
                }
                // See if time to open has expired.
                else if (timeleft <= 0)
                {
                    LOG_WARN("Time allowed for closing the roof has expired?");
                    setDomeState(DOME_IDLE);
                    roofClosing = false;
                    roofTimedOut = EXPIRED_CLOSE;
                }
                else
                {
                    delay = ACTIVE_POLL_MS;           // closing active
                }
            }
        }
    }

    // Added to highlight WiFi issues, not able to recover lost connection without a reconnect
    if (communicationErrors > MAX_CNTRL_COM_ERR)
    {
        LOG_ERROR("Too many errors communicating with Arduino");
        LOG_ERROR("Try a fresh connect. Check communication equipment and operation of Arduino controller.");
        INDI::Dome::Disconnect();
        initProperties();
        communicationErrors = 0;
    }

    // Even when no roof movement requested, will come through occasionally. Use timer to update roof status
    // in case roof has been operated externally by a remote control, locks applied...
    gettimeofday(&MotionStart, nullptr);
    SetTimer(delay);
}

float RollOffIno::CalcTimeLeft(timeval start)
{
    double timesince;
    double timeleft;
    struct timeval now
    {
        0, 0
    };
    gettimeofday(&now, nullptr);

    timesince =
            (double)(now.tv_sec * 1000.0 + now.tv_usec / 1000) - (double)(start.tv_sec * 1000.0 + start.tv_usec / 1000);
    timesince = timesince / 1000;
    timeleft  = MotionRequest - timesince;
    return timeleft;
}

void RollOffIno::msSleep (int mSec)
{
    struct timespec req = {0,0};
    req.tv_sec = mSec/1000;
    req.tv_nsec = (mSec % 1000) * 1000000L;
    nanosleep(&req, (struct timespec *)nullptr);
}

/*
 * Direction: DOME_CW Clockwise = Open; DOME-CCW Counter clockwise = Close
 * Operation: MOTION_START, | MOTION_STOP
 */
IPState RollOffIno::Move(DomeDirection dir, DomeMotionCommand operation)
{
    LOG_DEBUG("Roof received dome motion directive.");

    updateRoofStatus();
    if (operation == MOTION_START)
    {
        if (roofLockedSwitch)
        {
            LOG_WARN("Roof is externally locked, no movement possible");
            return IPS_ALERT;
        }
        if (roofOpening)
        {
            LOG_DEBUG("Roof is in process of opening, wait for completion.");
            return IPS_OK;
        }
        if (roofClosing)
        {
            LOG_DEBUG("Roof is in process of closing, wait for completion.");
            return IPS_OK;
        }

        // Open Roof
        // DOME_CW --> OPEN. If we are asked to "open" while we are fully opened as the
        // limit switch indicates, then we simply return false.
        if (dir == DOME_CW)
        {
            if (fullyOpenedLimitSwitch == ISS_ON)
            {
                LOG_WARN("DOME_CW directive received but roof is already fully opened");
                SetParked(false);
                return IPS_ALERT;
            }


            // Initiate action
            if (roofOpen())
            {
                roofOpening = true;
                roofClosing = false;
                LOG_INFO("Roof is opening...");
            }
            else
            {
                LOG_WARN("Failed to operate controller to open roof");
                return IPS_ALERT;
            }
        }

        // Close Roof
        else if (dir == DOME_CCW)
        {
            if (fullyClosedLimitSwitch == ISS_ON)
            {
                SetParked(true);
                LOG_WARN("DOME_CCW directive received but roof is already fully closed");
                return IPS_ALERT;
            }
            else if (INDI::Dome::isLocked())
            {
                DEBUG(INDI::Logger::DBG_WARNING,
                      "Cannot close dome when mount is locking. See: Telescope parkng policy, in options tab");
                return IPS_ALERT;
            }
            // Initiate action
            if (roofClose())
            {
                roofClosing = true;
                roofOpening = false;
                LOG_INFO("Roof is closing...");
            }
            else
            {
                LOG_WARN("Failed to operate controller to close roof");
                return IPS_ALERT;
            }
        }
        roofTimedOut = EXPIRED_CLEAR;
        MotionRequest = (int)RoofTimeoutN[0].value;
        LOGF_DEBUG("Roof motion timeout setting: %d", (int)MotionRequest);
        gettimeofday(&MotionStart, nullptr);
        SetTimer(INACTIVE_TIMING);
        return IPS_BUSY;
    }
    return    IPS_ALERT;
}

/*
 * Close Roof
 *
 */
IPState RollOffIno::Park()
{
    IPState rc = INDI::Dome::Move(DOME_CCW, MOTION_START);

    if (rc == IPS_BUSY)
    {
        LOG_INFO("RollOff ino is parking...");
        return IPS_BUSY;
    }
    else
        return IPS_ALERT;
}

/*
 * Open Roof
 *
 */
IPState RollOffIno::UnPark()
{
    IPState rc = INDI::Dome::Move(DOME_CW, MOTION_START);
    if (rc == IPS_BUSY)
    {
        LOG_INFO("RollOff ino is unparking...");
        return IPS_BUSY;
    }
    else
        return IPS_ALERT;
}

/*
 * Abort motion
 */
bool RollOffIno::Abort()
{
    bool lockState;
    bool openState;
    bool closeState;

    updateRoofStatus();
    lockState = (roofLockedSwitch == ISS_ON);
    openState = (fullyOpenedLimitSwitch == ISS_ON);
    closeState = (fullyClosedLimitSwitch == ISS_ON);

    if (lockState)
    {
        LOG_WARN("Roof is externally locked, no action taken on abort request");
        return true;
    }

    if (closeState && DomeMotionSP.s != IPS_BUSY)
    {
        LOG_WARN("Roof appears to be closed and stationary, no action taken on abort request");
        return true;
    }
    else if (openState && DomeMotionSP.s != IPS_BUSY)
    {
        LOG_WARN("Roof appears to be open and stationary, no action taken on abort request");
        return true;
    }
    else if (DomeMotionSP.s != IPS_BUSY)
    {
        LOG_WARN("Roof appears to be partially open and stationary, no action taken on abort request");
    }
    else if (DomeMotionSP.s == IPS_BUSY)
    {
        if (DomeMotionS[DOME_CW].s == ISS_ON)
        {
            LOG_WARN("Abort roof action requested while the roof was opening. Direction correction may be needed on the next move request.");
        }
        else if (DomeMotionS[DOME_CCW].s == ISS_ON)
        {
            LOG_WARN("Abort roof action requested while the roof was closing. Direction correction may be needed on the next move request.");
        }
        roofClosing = false;
        roofOpening = false;
        MotionRequest = -1;
        roofAbort();
    }

    // If both limit switches are off, then we're neither parked nor unparked.
    if (fullyOpenedLimitSwitch == ISS_OFF && fullyClosedLimitSwitch == ISS_OFF)
    {
        IUResetSwitch(&ParkSP);
        ParkSP.s = IPS_IDLE;
        IDSetSwitch(&ParkSP, nullptr);
    }
    return true;
}

bool RollOffIno::getFullOpenedLimitSwitch(bool* switchState)
{
    if (isSimulation())
    {
        if (simRoofOpen)
        {
            fullyOpenedLimitSwitch = ISS_ON;
            *switchState = true;
        }
        else
        {
            fullyOpenedLimitSwitch = ISS_OFF;
            *switchState = false;
        }
        return true;
    }

    if (readRoofSwitch(ROOF_OPENED_SWITCH, switchState))
    {
        if (*switchState)
            fullyOpenedLimitSwitch = ISS_ON;
        else
            fullyOpenedLimitSwitch = ISS_OFF;
        return true;
    }
    else
    {
        LOG_WARN("Unable to obtain from the controller whether or not the roof is opened");
        return false;
    }
}

bool RollOffIno::getFullClosedLimitSwitch(bool* switchState)
{
    if (isSimulation())
    {
        if (simRoofClosed)
        {
            fullyClosedLimitSwitch = ISS_ON;
            *switchState = true;
        }
        else
        {
            fullyClosedLimitSwitch = ISS_OFF;
            *switchState = false;
        }
        return true;
    }

    if (readRoofSwitch(ROOF_CLOSED_SWITCH, switchState))
    {
        if (*switchState)
            fullyClosedLimitSwitch = ISS_ON;
        else
            fullyClosedLimitSwitch = ISS_OFF;
        return true;
    }
    else
    {
        LOG_WARN("Unable to obtain from the controller whether or not the roof is closed");
        return false;
    }
}

bool RollOffIno::getRoofLockedSwitch(bool* switchState)
{
    // If there is no lock switch, return success with status false
    if (isSimulation())
    {
        roofLockedSwitch = ISS_OFF;
        *switchState = false;                     // Not locked
        return true;
    }
    if (readRoofSwitch(ROOF_LOCKED_SWITCH, switchState))
    {
        if (*switchState)
            roofLockedSwitch = ISS_ON;
        else
            roofLockedSwitch = ISS_OFF;
        return true;
    }
    else
    {
        LOG_WARN("Unable to obtain from the controller whether or not the roof is externally locked");
        return false;
    }
}

bool RollOffIno::getRoofAuxSwitch(bool* switchState)
{
    // If there is no lock switch, return success with status false
    if (isSimulation())
    {
        if (AuxS[AUX_ENABLE].s == ISS_OFF)
        {
            roofAuxiliarySwitch = ISS_OFF;
            *switchState = false;
            return true;
        }
        else
        {
            roofAuxiliarySwitch = ISS_ON;
            *switchState = true;
            return true;
        }
    }
    if (readRoofSwitch(ROOF_AUX_SWITCH, switchState))
    {
        if (*switchState)
            roofAuxiliarySwitch = ISS_ON;
        else
            roofAuxiliarySwitch = ISS_OFF;
        return true;
    }
    else
    {
        LOG_WARN("Unable to obtain from the controller whether or not the obs Aux switch is being used");
        return false;
    }
}
/*
 * -------------------------------------------------------------------------------------------
 *
 */
bool RollOffIno::roofOpen()
{
    if (isSimulation())
    {
        return true;
    }
    return pushRoofButton(ROOF_OPEN_RELAY, true, false);
}

bool RollOffIno::roofClose()
{
    if (isSimulation())
    {
        return true;
    }
    return pushRoofButton(ROOF_CLOSE_RELAY, true, false);
}

bool RollOffIno::roofAbort()
{
    if (isSimulation())
    {
        return true;
    }
    return pushRoofButton(ROOF_ABORT_RELAY, true, false);
}

bool RollOffIno::setRoofLock(bool switchOn)
{
    if (isSimulation())
    {
        return false;
    }
    return pushRoofButton(ROOF_LOCK_RELAY, switchOn, true);
}

bool RollOffIno::setRoofAux(bool switchOn)
{
    if (isSimulation())
    {
        return false;
    }
    return pushRoofButton(ROOF_AUX_RELAY, switchOn, true);
}

/*
 * If a single button controller, whether roof is moving or stopped, the state of the external controller
 * will determine the effect on the roof. This could mean stopping, or starting in a reversed direction.
 *
 * The initial implementation using GPIO pins will only be supporting external controllers that control when
 * the roof stops moving. This implies that for buttons/relays that cause a roof motion the length of the on
 * state will temporary, less that a second. The existing polling timer will be used and the code will wait locally.
 */

bool RollOffIno::pushRoofButton(const char* button, bool switchOn, bool ignoreLock)
{
    bool status = false;
    bool roofLocked = false;
    bool found = false;
    bool moveFunction = false;
    bool wantHigh = false;
    unsigned int level;
    unsigned int gpio = 0;
    int intervalMilli = 0;

    if (!contactEstablished)
    {
        LOG_WARN("No contact with the roof controller has been established");
        return false;
    }
    status = getRoofLockedSwitch(&roofLocked);      // In case it has been locked since the driver connected
    if ((!status || roofLocked) && !ignoreLock)
    {
        LOG_WARN("Roof external lock state prevents roof movement");
        return false;
    }

    status = true;
    for (int i = 0; i < MAX_OUT_DEFS; i++)
    {
        for (int j = 0; j < MAX_OUT_OPS; j++)
        {
            if ((strcmp(button, outFunctionS[i][j].name) == 0) && (outFunctionS[i][j].s == ISS_ON))
            {
                if ((strcmp(button, ROOF_OPEN_RELAY) == 0) || (strcmp(button, ROOF_CLOSE_RELAY) == 0) ||
                    (strcmp(button, ROOF_ABORT_RELAY) == 0))
                {
                    moveFunction = true;
                }
                found = true;
                break;
            }
        }
        if (found)
        {
            gpio = outPinNumberN[i][0].value;
            // switchOn true if turning relay on
            if ((strcmp(outActivateWhenS[i][0].name, "High") == 0) && (outActivateWhenS[i][0].s == ISS_ON))
                wantHigh = switchOn;
            else if ((strcmp(outActivateWhenS[i][1].name, "Low") == 0) && (outActivateWhenS[i][1].s == ISS_ON))
                wantHigh = !switchOn;

            for (int j = 0; j < MAX_OUT_ACTIVE_LIMIT; j++)
            {
                if ((outActiveLimitS[i][j].s == ISS_ON))
                {
                    intervalMilli = activeLimitMilli[j];
                    if ((intervalMilli == 0) && moveFunction)
                    {
                        status = false;
                        LOGF_WARN("%s needs a Active Limit interval, No Limit only available for Lock and Aux.", button);
                    }
                    break;
                }
            }
            break;
        }
    }
    if (!found && ((strcmp(button, ROOF_OPEN_RELAY) == 0) || (strcmp(button, ROOF_CLOSE_RELAY) == 0) ||
                   (strcmp(button, ROOF_ABORT_RELAY) == 0)))
    {
        LOGF_WARN("A GPIO pin definition for %s was not found.", button);
        return false;
    }

    // If a definition of the named optional relay is not found assume it is not being used.
    if (!found && status)
        return true;

    if (found && !status)
    {
        LOGF_WARN("A usable GPIO pin definition for %s was not found.", button);
        return status;
    }

    // If useTimer associate gpio pin, active hi/lo, with setting a timer to handle turning off the relay.
    level = wantHigh ? 1 : 0;
    //LOGF_WARN("*** GPIO write turn ON for: %s, pin: %d, level: %d, delay: %d", button, gpio, level, intervalMilli);
    status = gpio_write(pi_id, gpio, level);
    if (status != 0)
    {
        LOGF_WARN("GPIO write failed for %s, %d, returned: %s", button, gpio, pigpio_error(status));
        return false;
    }
    else
    {
        if (intervalMilli > 0)
        {
            msSleep(intervalMilli);
            level = wantHigh ? 0 : 1;
            //LOGF_WARN("*** GPIO write turn OFF for: %s, pin: %d, level: %d, after delay of %d", button, gpio, level, intervalMilli);
            status = gpio_write(pi_id, gpio, level);
            if (status != 0)
            {
                LOGF_WARN("GPIO write reset failed for %s, %d, returned: %s", button, gpio, pigpio_error(status));
                return false;
            }
        }
    }
    return true;
}

bool RollOffIno::readRoofSwitch(const char* roofSwitchId, bool *result)
{
    bool found = false;
    bool activeHigh = false;
    bool activeLow = false;
    unsigned int gpio = 0;
    int retValue = 0;

    *result = false;

    if (!contactEstablished)
    {
        LOG_WARN("No contact with the roof controller has been established");
        return false;
    }

    // For each definition entry. Find the function name that matches the name of the switch wanted.
    // See if it is selected. If not move on to the next entry. If a match found get its gpio pin # and high|low
    // activation indicator.
    for (int i = 0; i < MAX_INP_DEFS; i++)
    {
        for (int j = 0; j < MAX_INP_OPS; j++)
        {
            if ((strcmp(roofSwitchId, inpFunctionS[i][j].name) == 0) && (inpFunctionS[i][j].s == ISS_ON))
            {
                found = true;
                break;
            }
        }
        if (found)
        {
            gpio = inpPinNumberN[i][0].value;
            if ((strcmp(inpActivateWhenS[i][0].name, "High") == 0) && (inpActivateWhenS[i][0].s == ISS_ON))
                activeHigh = true;
            else if ((strcmp(inpActivateWhenS[i][1].name, "Low") == 0) && (inpActivateWhenS[i][1].s == ISS_ON))
                activeLow = true;
            break;
        }
    }
    if (!found)
    {
        if ((strcmp(roofSwitchId, ROOF_OPENED_SWITCH) == 0) || (strcmp(roofSwitchId, ROOF_CLOSED_SWITCH) == 0))
        {
            LOGF_WARN("A usable GPIO pin definition for %s was not found.", roofSwitchId);
            return false;
        }
            // If a definition of another switch is not found assume it is not being used.
        else
            return true;
    }

    // Read gpio pin. if high & activeHigh or low && activeLow set *result true else set *result false.
    retValue = gpio_read(pi_id, gpio);
    if (retValue == PI_BAD_GPIO)
    {
        LOGF_WARN("GPIO read failed for %s, %d, returned: %s", roofClosing, gpio, pigpio_error(retValue));
        return false;
    }

    if ((activeHigh && retValue == 1) || (activeLow && retValue == 0))
    {
        *result = true;
        //LOGF_WARN("*** Reading: %s, pin: %d, value: %d, ON ***", roofSwitchId, gpio, retValue);
    }
    else
    {
        *result = false;
        //LOGF_WARN("*** Reading: %s, pin: %d, value: %d, OFF ***", roofSwitchId, gpio, retValue);
    }
    return true;
}

