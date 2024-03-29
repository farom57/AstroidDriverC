/*******************************************************************************
 Copyright(c) 2019 Jasem Mutlaq. All rights reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#pragma once

#include "indiguiderinterface.h"
#include "indifocuserinterface.h"
#include "inditelescope.h"
#include "messages.h"
#include "libastro.h"

#define STEP_BY_TURN (50. * 3. * 144.)

#define GOTO_STOP_DISTANCE 10./3600. // 10 arcsec
#define GOTO_SLOW_DISTANCE 3. // 3 deg
#define GOTO_SLOW_SPEED 5 // TODO define as funct of refresh rate
#define MAX_SPEED (623*2)
#define GOTO_SPEED 400.


/**
 * @brief The MountDriver class provides a simple example for development of a new
 * mount driver. Modify the driver to fit your needs.
 *
 * It supports the following features:
 * + Sideral and Custom Tracking rates.
 * + Goto & Sync
 * + NWSE Hand controller direciton key slew.
 * + Tracking On/Off.

 *
 * On startup and by default the mount shall point to the celestial pole.
 *
 * @author Jasem Mutlaq
 */
class Astroid : public INDI::Telescope, public INDI::GuiderInterface, public INDI::FocuserInterface
{
    public:
        Astroid();

        virtual const char *getDefaultName() override;
        virtual bool initProperties() override;
        virtual bool updateProperties() override;

        virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;



    protected:
        ///////////////////////////////////////////////////////////////////////////////
        /// Communication
        ///////////////////////////////////////////////////////////////////////////////
        /**
         * @brief Handshake Attempt communication with the mount.
         * @return true if successful, false otherwise.
         */
        virtual bool Handshake() override;

        /**
         * @brief ReadScopeStatus Query the mount status, coordinate, any status indicators, pier side..etc.
         * @return True if query is successful, false otherwise.
         */
        virtual bool ReadScopeStatus() override;

        /**
         * @brief sendCommand Send the commanded state
         * @param ack true if acknowledment is required
         * @return true if successful, false otherwise.
         */
        bool sendCommand(bool ack = true);
        bool updateSpeed(bool ack = true);
        Status_message last_status;
        Cmd_message command;


        ///////////////////////////////////////////////////////////////////////////////
        /// Motions commands.
        ///////////////////////////////////////////////////////////////////////////////

        /**
         * @brief MoveNS Start or Stop motion in the North/South DEC Axis.
         * @param dir Direction
         * @param command Start or Stop
         * @return true if successful, false otherwise.
         */
        virtual bool MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) override;

        /**
         * @brief MoveWE Start or Stop motion in the East/West RA Axis.
         * @param dir Direction
         * @param command Start or Stop
         * @return true if successful, false otherwise.
         */
        virtual bool MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) override;

        /**
         * @brief Abort Abort all motion. If tracking, stop it.
         * @return True if successful, false otherwise.
         */
        virtual bool Abort() override;

        ///////////////////////////////////////////////////////////////////////////////
        /// Pulse Guiding Commands
        ///////////////////////////////////////////////////////////////////////////////
        virtual IPState GuideNorth(uint32_t ms) override;
        virtual IPState GuideSouth(uint32_t ms) override;
        virtual IPState GuideEast(uint32_t ms) override;
        virtual IPState GuideWest(uint32_t ms) override;

        ///////////////////////////////////////////////////////////////////////////////
        /// Tracking Commands
        ///////////////////////////////////////////////////////////////////////////////
        virtual bool SetTrackMode(uint8_t mode) override;
        virtual bool SetTrackEnabled(bool enabled) override;
        virtual bool SetTrackRate(double raRate, double deRate) override;

        ///////////////////////////////////////////////////////////////////////////////
        /// GOTO & Sync commands
        ///////////////////////////////////////////////////////////////////////////////
        virtual bool Goto(double RA, double DE) override;
        virtual bool Sync(double RA, double DE) override;

        ///////////////////////////////////////////////////////////////////////////////
        /// Focus Commands
        ///////////////////////////////////////////////////////////////////////////////
        bool SetFocuserSpeed(int speed);
        IPState MoveFocuser(FocusDirection dir, int speed, uint16_t duration);
        IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks);
        //bool ReverseFocuser(bool enabled);
        bool AbortFocuser();


        /**
         * @brief hexDump Helper function to print non-string commands to the logger so it is easier to debug
         * @param buf buffer to format the command into hex strings.
         * @param data the command
         * @param size length of the command in bytes.
         * @note This is called internally by sendCommand, no need to call it directly.
         */
        void hexDump(char * buf, const uint8_t * data, int size);




    private:
        ///////////////////////////////////////////////////////////////////////////////
        /// Additional Properties
        ///////////////////////////////////////////////////////////////////////////////
        INumber GuideRateN[2];
        INumberVectorProperty GuideRateNP;
        INumber AuxN[3];
        INumberVectorProperty AuxNP;

        ISwitch TargetPierSideS[4];
        ISwitchVectorProperty TargetPierSideSP;
        static const int PIER_AUTO = 2;
        static const int PIER_CURRENT = 3;

        ///////////////////////////////////////////////////////////////////////////////
        /// Class Variables
        ///////////////////////////////////////////////////////////////////////////////
        double sync_coord_HA;
        double sync_step_HA;
        double sync_coord_DE;
        double sync_step_DE;
        double goto_target_RA;
        double goto_target_DE;
        bool goto_active;
        double _ra = 0, _de = 0; // it differs from indi angle in that if _de is outside of -90 -> 90 it indicates a different pier side

        double motion_speeds[4] = {1., 10., 50., 400};
        double focus_speed = 1.;

        double track_speed_HA = 1.;
        double track_speed_DE = 0.;

        uint8_t track_mode=TRACK_SIDEREAL; // TRACK_SIDEREAL, TRACK_SOLAR, TRACK_LUNAR, TRACK_CUSTOM

        double track_custom_speed_HA = 1;
        double track_custom_speed_DE = 0;

        double slew_DE_speed = 0;
        double slew_RA_speed = 0;
        double slew_focus_speed = 0;

        double power_HA = 1;
        double power_DE = 1;
        double power_FOCUS = 1;




        int GuideNSTID { -1 };
        int GuideWETID { -1 };
        int FocusTID { -1 };

        ///////////////////////////////////////////////////////////////////////////////
        /// Class Functions
        ///////////////////////////////////////////////////////////////////////////////

        void stopNSPulse();
        void stopWEPulse();
        void stopFocusPulse();
        void processGoto();


        /////////////////////////////////////////////////////////////////////////////
        /// Static Helper Values
        /////////////////////////////////////////////////////////////////////////////
        static double mod360(double x);
        static double mod24(double x);
        static bool normalize_ra_de(double *ra, double *de);
        static void stopNSPulseHelper(void *p);
        static void stopWEPulseHelper(void *p);
        static void stopFocusPulseHelper(void *p);
};
