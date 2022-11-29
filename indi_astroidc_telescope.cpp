/*
   INDI Developers Manual
   Tutorial #2

   "Simple Telescope Driver"

   We develop a simple telescope simulator.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

/** \file simplescope.cpp
    \brief Construct a basic INDI telescope device that simulates GOTO commands.
    \author Jasem Mutlaq

    \example simplescope.cpp
    A simple GOTO telescope that simulator slewing operation.
*/

#include "indi_astroidc_telescope.h"

#include "indicom.h"

#include <cmath>
#include <memory>
#include "libindi/connectionplugins/connectionserial.h"

static std::unique_ptr<Astroid> simpleScope(new Astroid());

#define RA_AXIS     0
#define DEC_AXIS    1
#define GUIDE_NORTH 0
#define GUIDE_SOUTH 1
#define GUIDE_WEST  0
#define GUIDE_EAST  1

Astroid::Astroid()
{
    SetTelescopeCapability(TELESCOPE_CAN_GOTO | TELESCOPE_CAN_SYNC | TELESCOPE_CAN_ABORT | TELESCOPE_HAS_PIER_SIDE | TELESCOPE_HAS_TRACK_MODE | TELESCOPE_CAN_CONTROL_TRACK | TELESCOPE_HAS_TRACK_RATE , 4);
    setTelescopeConnection(CONNECTION_SERIAL);
}

/**************************************************************************************
** We init our properties here. The only thing we want to init are the Debug controls
***************************************************************************************/
bool Astroid::initProperties()
{
    // ALWAYS call initProperties() of parent first
    INDI::Telescope::initProperties();


    // Telescope

    // Guider
    initGuiderProperties(getDeviceName(), MOTION_TAB);

    // Lightbox
    //initLightBoxProperties(getDeviceName(), "Aux");

    addDebugControl();
    setDriverInterface(TELESCOPE_INTERFACE | GUIDER_INTERFACE); //FOCUSER_INTERFACE | AUX_INTERFACE | LIGHTBOX_INTERFACE);
    setDefaultPollingPeriod(250);

    serialConnection->setDefaultBaudRate(Connection::Serial::B_9600);
    //tty_set_debug(true);

    return true;
}



/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char *Astroid::getDefaultName()
{
    return "Astroid";
}

/**************************************************************************************
** Client is asking us to slew to a new position
***************************************************************************************/
bool Astroid::Goto(double ra, double dec)
{
    targetRA  = ra;
    targetDEC = dec;
    char RAStr[64] = {0}, DecStr[64] = {0};

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, targetRA, 2, 3600);
    fs_sexa(DecStr, targetDEC, 2, 3600);

    // Mark state as slewing
    TrackState = SCOPE_SLEWING;

    // Inform client we are slewing to a new position
    LOGF_INFO("Slewing to RA: %s - DEC: %s", RAStr, DecStr);

    // Success!
    return true;
}

/**************************************************************************************
** Client is asking us to abort our motion
***************************************************************************************/
bool Astroid::Abort()
{
    return true;
}

/**************************************************************************************
** Client is asking us to report telescope status
***************************************************************************************/
bool Astroid::ReadScopeStatus()
{
    char buf[200];
    int nbytes = 0, rc = 0;

    if (!isConnected()){
        LOG_ERROR("Error not connected");
    }

    if ((rc = tty_read_section_expanded (PortFD, buf, 0x55,0, 100000, &nbytes)) != TTY_OK)
    {
        LOGF_ERROR("Preamble not found, result: %d", rc);
        return false;
    }
    LOGF_INFO("discarded %d bytes", nbytes);

    if ((rc = tty_read_expanded (PortFD, buf, 56, 0, 5000, &nbytes)) != TTY_OK)
    {
        LOGF_ERROR("Error reading. Result: %d nbytes: %d", rc, nbytes, PortFD);
        return false;
    }else{
        LOGF_INFO("Read %d bytes", nbytes);
        uint32_t ms_count = (buf[0]<<24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];
        uint32_t step_ha = (buf[4]<<24) + (buf[5] << 16) + (buf[6] << 8) + buf[7];
        uint32_t step_de = (buf[8]<<24) + (buf[9] << 16) + (buf[10] << 8) + buf[11];
        LOGF_INFO("ms_count: %d, step_ha: %d, step_de", ms_count, step_ha, step_de);
        float ustep_ha = *(float*)(buf+12);
        uint32_t test = (buf[12]<<24) + (buf[13] << 16) + (buf[14] << 8) + buf[15];
        float ustep_ha2 = *(float*)(&test);
        LOGF_INFO("ustep_ha: %f ustep_ha2: %f test:%d",ustep_ha, ustep_ha2, test);

    }

    static struct timeval ltv
    {
        0, 0
    };
    struct timeval tv
    {
        0, 0
    };
    double dt = 0, da_ra = 0, da_dec = 0, dx = 0, dy = 0;
    int nlocked;

    /* update elapsed time since last poll, don't presume exactly POLLMS */
    gettimeofday(&tv, nullptr);

    if (ltv.tv_sec == 0 && ltv.tv_usec == 0)
        ltv = tv;

    dt  = tv.tv_sec - ltv.tv_sec + (tv.tv_usec - ltv.tv_usec) / 1e6;
    ltv = tv;

    // Calculate how much we moved since last time
    da_ra  = SLEW_RATE * dt;
    da_dec = SLEW_RATE * dt;

    /* Process per current state. We check the state of EQUATORIAL_EOD_COORDS_REQUEST and act acoordingly */
    switch (TrackState)
    {
        case SCOPE_SLEWING:
            // Wait until we are "locked" into positon for both RA & DEC axis
            nlocked = 0;

            // Calculate diff in RA
            dx = targetRA - currentRA;

            // If diff is very small, i.e. smaller than how much we changed since last time, then we reached target RA.
            if (fabs(dx) * 15. <= da_ra)
            {
                currentRA = targetRA;
                nlocked++;
            }
            // Otherwise, increase RA
            else if (dx > 0)
                currentRA += da_ra / 15.;
            // Otherwise, decrease RA
            else
                currentRA -= da_ra / 15.;

            // Calculate diff in DEC
            dy = targetDEC - currentDEC;

            // If diff is very small, i.e. smaller than how much we changed since last time, then we reached target DEC.
            if (fabs(dy) <= da_dec)
            {
                currentDEC = targetDEC;
                nlocked++;
            }
            // Otherwise, increase DEC
            else if (dy > 0)
                currentDEC += da_dec;
            // Otherwise, decrease DEC
            else
                currentDEC -= da_dec;

            // Let's check if we recahed position for both RA/DEC
            if (nlocked == 2)
            {
                // Let's set state to TRACKING
                TrackState = SCOPE_TRACKING;

                LOG_INFO("Telescope slew is complete. Tracking...");
            }
            break;

        default:
            break;
    }

    char RAStr[64] = {0}, DecStr[64] = {0};

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, currentRA, 2, 3600);
    fs_sexa(DecStr, currentDEC, 2, 3600);

    LOGF_DEBUG("Current RA: %s Current DEC: %s", RAStr, DecStr);

    NewRaDec(currentRA, currentDEC);
    return true;
}


void Astroid::ISGetProperties(const char *dev)
{
    INDI::Telescope::ISGetProperties(dev);
}


bool Astroid::updateProperties()
{

    INDI::Telescope::updateProperties();

    if (isConnected())
    {
        defineProperty(&GuideNSNP);
        defineProperty(&GuideWENP);
        defineProperty(&GuideRateNP);
        loadConfig(true, GuideRateNP.name);
    }
    else
    {
        deleteProperty(GuideNSNP.name);
        deleteProperty(GuideWENP.name);
        deleteProperty(GuideRateNP.name);
    }

    return true;
}

bool Astroid::Connect()
{
    LOG_INFO("Connecting");
    Telescope::Connect();
    LOG_INFO("Astroid is online.");
    SetTimer(getCurrentPollingPeriod());

    return true;
}

bool Astroid::Disconnect()
{
    LOG_INFO("Disconnecting");
    Telescope::Disconnect();
    LOG_INFO("Astroid is offline.");
    return true;
}

bool Astroid::Sync(double ra, double dec)
{
    return false;
}

bool Astroid::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    //  first check if it's for our device

    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (strcmp(name, "GUIDE_RATE") == 0)
        {
            IUUpdateNumber(&GuideRateNP, values, names, n);
            GuideRateNP.s = IPS_OK;
            IDSetNumber(&GuideRateNP, nullptr);
            return true;
        }

        if (strcmp(name, GuideNSNP.name) == 0 || strcmp(name, GuideWENP.name) == 0)
        {
            processGuiderProperties(name, values, names, n);
            return true;
        }
    }

    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}


bool Astroid::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Slew mode
        if (strcmp(name, SlewRateSP.name) == 0)
        {
            if (IUUpdateSwitch(&SlewRateSP, states, names, n) < 0)
                return false;

            SlewRateSP.s = IPS_OK;
            IDSetSwitch(&SlewRateSP, nullptr);
            return true;
        }
    }

    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

bool Astroid::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    mcRate = static_cast<int>(IUFindOnSwitchIndex(&SlewRateSP)) + 1;

    int rate = (dir == INDI_DIR_NS::DIRECTION_NORTH) ? mcRate : -mcRate;
    LOGF_DEBUG("MoveNS dir %s, motion %s, rate %d", dir == DIRECTION_NORTH ? "N" : "S", command == 0 ? "start" : "stop", rate);


    return true;
}

bool Astroid::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    mcRate = static_cast<int>(IUFindOnSwitchIndex(&SlewRateSP)) + 1;

    int rate = (dir == INDI_DIR_WE::DIRECTION_EAST) ? -mcRate : mcRate;
    LOGF_DEBUG("MoveWE dir %d, motion %s, rate %d", dir == DIRECTION_EAST ? "E" : "W", command == 0 ? "start" : "stop", rate);

    return true;
}

IPState Astroid::GuideNorth(uint32_t ms)
{
    double rate = GuideRateN[DEC_AXIS].value;

    guidingNS = true;
    return IPS_BUSY;
}

IPState Astroid::GuideSouth(uint32_t ms)
{
    double rate = GuideRateN[DEC_AXIS].value;

    guidingNS = true;
    return IPS_BUSY;
}

IPState Astroid::GuideEast(uint32_t ms)
{
    double rate = GuideRateN[RA_AXIS].value;

    guidingEW = true;
    return IPS_BUSY;
}

IPState Astroid::GuideWest(uint32_t ms)
{
    double rate = GuideRateN[RA_AXIS].value;

    guidingEW = true;
    return IPS_BUSY;
}

bool Astroid::SetTrackMode(uint8_t mode)
{
    switch (static_cast<TelescopeTrackMode>(mode))
    {
        case TRACK_SIDEREAL:

            return true;
        case TRACK_SOLAR:

            return true;
        case TRACK_LUNAR:

            return true;
        case TRACK_CUSTOM:
            SetTrackRate(TrackRateN[AXIS_RA].value, TrackRateN[AXIS_DE].value);
            return true;
    }
    return false;
}


bool Astroid::SetTrackEnabled(bool enabled)
{

    return true;
}

bool Astroid::SetTrackRate(double raRate, double deRate)
{
    return true;
}

/*
bool Astroid::saveConfigItems(FILE *fp)
{
    INDI::Telescope::saveConfigItems(fp);

    return true;
}
*/

bool Astroid::sendCommand(const char *cmd)
{
    /*int nbytes_read = 0, nbytes_written = 0, tty_rc = 0;
    char res[8] = {0};
    LOGF_DEBUG("CMD <%s>", cmd);

    if (!isSimulation())
    {
        tcflush(PortFD, TCIOFLUSH);
        if ((tty_rc = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            char errorMessage[MAXRBUF];
            tty_error_msg(tty_rc, errorMessage, MAXRBUF);
            LOGF_ERROR("Serial write error: %s", errorMessage);
            return false;
        }
    }

    if (isSimulation())
    {
        strncpy(res, "OK#", 8);
        nbytes_read = 3;
    }
    else
    {
        if ((tty_rc = tty_read_section(PortFD, res, '#', 1, &nbytes_read)) != TTY_OK)
        {
            char errorMessage[MAXRBUF];
            tty_error_msg(tty_rc, errorMessage, MAXRBUF);
            LOGF_ERROR("Serial read error: %s", errorMessage);
            return false;
        }
    }

    res[nbytes_read - 1] = '\0';
    LOGF_DEBUG("RES <%s>", res);
    */

    return true;
}

bool Astroid::Handshake()
{
    LOGF_INFO("Handshake. PortFD: %d",PortFD);
    return true;
}
