/*
   INDI Developers Manual
   Tutorial #2

   "Simple Telescope Driver"

   We develop a simple telescope simulator.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

/** \file simplescope.h
    \brief Construct a basic INDI telescope device that simulates GOTO commands.
    \author Jasem Mutlaq

    \example simplescope.h
    A simple GOTO telescope that simulator slewing operation.
*/

#pragma once

#include "inditelescope.h"
#include "indiguiderinterface.h"
#include "indifocuserinterface.h"
#include "indilightboxinterface.h"


namespace Connection
{
    class Serial;
}


class Astroid : public INDI::Telescope, public INDI::GuiderInterface//, public INDI::FocuserInterface, public INDI::LightBoxInterface
{
    public:
        Astroid();
        virtual const char *getDefaultName() override;
        virtual bool Connect() override;
        virtual bool Disconnect() override;
        virtual bool initProperties() override;
        virtual bool updateProperties() override;

        virtual void ISGetProperties(const char *dev) override;
        virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;




    protected:
        bool Handshake() override;

        // Telescope specific functions
        virtual bool ReadScopeStatus() override;
        virtual bool Goto(double, double) override;
        virtual bool MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) override;
        virtual bool MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) override;
        virtual bool Abort() override;
        virtual bool SetTrackMode(uint8_t mode) override;
        virtual bool SetTrackEnabled(bool enabled) override;
        virtual bool SetTrackRate(double raRate, double deRate) override;
        virtual bool Sync(double ra, double dec) override;

        // Guider functions
        virtual IPState GuideNorth(uint32_t ms) override;
        virtual IPState GuideSouth(uint32_t ms) override;
        virtual IPState GuideEast(uint32_t ms) override;
        virtual IPState GuideWest(uint32_t ms) override;


        // Focuser functions

        // LightBox functions

    private:
        double currentRA {0};
        double currentDEC {90};
        double targetRA {0};
        double targetDEC {0};

        int mcRate = 0;

        bool guidingNS = false;
        bool guidingEW = false;

        // slew rate, degrees/s
        static const uint8_t SLEW_RATE = 3;

        INumber GuideRateN[2];
        INumberVectorProperty GuideRateNP;

        // Serial
        bool sendCommand(const char *cmd);




};

