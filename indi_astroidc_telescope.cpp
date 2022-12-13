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

#include "libastro.h"

#include "indi_astroidc_telescope.h"

#include "indicom.h"
#include "connectionplugins/connectionserial.h"


//#include <libnova/sidereal_time.h>
//#include <libnova/transform.h>

#include <termios.h>
#include <cmath>
#include <cstring>
#include <memory>
#include <unistd.h>

// Single unique pointer to the driver.
static std::unique_ptr<Astroid> telescope_sim(new Astroid());

Astroid::Astroid()
{
    // Let's specify the driver version
    setVersion(1, 0);

    // Set capabilities supported by the mount.
    // The last parameters is the number of slew rates available.
    SetTelescopeCapability(TELESCOPE_CAN_SYNC | TELESCOPE_CAN_GOTO | TELESCOPE_CAN_ABORT | TELESCOPE_HAS_PIER_SIDE |
                           TELESCOPE_HAS_TIME | TELESCOPE_HAS_TRACK_MODE | TELESCOPE_CAN_CONTROL_TRACK | TELESCOPE_HAS_TRACK_RATE | TELESCOPE_HAS_LOCATION,
                           4);
    setTelescopeConnection(CONNECTION_SERIAL);
}

const char *Astroid::getDefaultName()
{
    return "Astroid";
}

bool Astroid::initProperties()
{
    // Make sure to init parent properties first
    INDI::Telescope::initProperties();

    // How fast do we guide compared to sidereal rate
    IUFillNumber(&GuideRateN[AXIS_RA], "GUIDE_RATE_WE", "W/E Rate", "%.1f", 0, 10, 0.1, 0.5);
    IUFillNumber(&GuideRateN[AXIS_DE], "GUIDE_RATE_NS", "N/S Rate", "%.1f", 0, 10, 0.1, 0.5);
    IUFillNumberVector(&GuideRateNP, GuideRateN, 2, getDeviceName(), "GUIDE_RATE", "Guiding Rate", MOTION_TAB, IP_RW, 0,
                       IPS_IDLE);

    // Fill the slex rates
    IUFillSwitch(&SlewRateS[SLEW_GUIDE], "SLEW_GUIDE", "1x", ISS_OFF);
    IUFillSwitch(&SlewRateS[SLEW_CENTERING], "SLEW_CENTERING", "10x", ISS_OFF);
    IUFillSwitch(&SlewRateS[SLEW_FIND], "SLEW_FIND", "50x", ISS_OFF);
    IUFillSwitch(&SlewRateS[SLEW_MAX], "SLEW_MAX", "400x", ISS_ON);
    IUFillSwitchVector(&SlewRateSP, SlewRateS, 5, getDeviceName(), "TELESCOPE_SLEW_RATE", "Slew Rate", MOTION_TAB,
                       IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Add Tracking Modes. If you have SOLAR, LUNAR..etc, add them here as well.
    AddTrackMode("TRACK_SIDEREAL", "Sidereal", true);
    AddTrackMode("TRACK_SOLAR", "Solar");
    AddTrackMode("TRACK_LUNAR", "Lunar");
    AddTrackMode("TRACK_CUSTOM", "Custom");

    // Target Pier sidereal
    IUFillSwitch(&TargetPierSideS[PIER_WEST], "PIER_WEST", "West (pointing east)", ISS_OFF);
    IUFillSwitch(&TargetPierSideS[PIER_EAST], "PIER_EAST", "East (pointing west)", ISS_OFF);
    IUFillSwitch(&TargetPierSideS[PIER_AUTO], "PIER_AUTO", "Auto", ISS_ON);
    IUFillSwitch(&TargetPierSideS[PIER_CURRENT], "PIER_CURRENT", "Current", ISS_OFF);
    IUFillSwitchVector(&TargetPierSideSP, TargetPierSideS, 4, getDeviceName(), "TARGET_PIER_SIDE", "Target Pier Side", MAIN_CONTROL_TAB,
                       IP_RW, ISR_ATMOST1, 60, IPS_IDLE);


    // The mount is initially in SLEWING state.
    TrackState = SCOPE_SLEWING;
    setPierSide(PIER_EAST);


    // Let init the pulse guiding properties
    initGuiderProperties(getDeviceName(), MOTION_TAB);
    syncDriverInfo();

    // Add debug controls
    addDebugControl();

    // Set the driver interface to indicate that we can also do pulse guiding
    setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);

    // We want to query the mount every 100ms by default. The user can override this value.
    setDefaultPollingPeriod(1000);



    return true;
}

bool Astroid::updateProperties()
{
    INDI::Telescope::updateProperties();



    if (isConnected())
    {
        defineProperty(&GuideNSNP);
        defineProperty(&GuideWENP);
        defineProperty(&GuideRateNP);
        defineProperty(&TargetPierSideSP);
    }
    else
    {
        deleteProperty(GuideNSNP.name);
        deleteProperty(GuideWENP.name);
        deleteProperty(GuideRateNP.name);
        deleteProperty(TargetPierSideSP.name);
    }

    return true;
}

bool Astroid::Handshake()
{

    // The device is reinitialized when the connection is open. It happens before handshake is called.
    // 0.5s delay to let the device to initialize correctly
    usleep(500000);
    int err_count = 0;

    while(err_count < 5){

        if(Astroid::ReadScopeStatus()){
            LOG_INFO("Connection sucessful");

            // Reset sync
            sync_coord_HA = get_local_sidereal_time(m_Location.longitude);
            sync_step_HA = 0;
            sync_coord_DE = 0;
            sync_step_DE = 0;

            return true;
        }
        LOGF_INFO("Connection error: %d attempt", err_count+1);
        usleep(500000);
        err_count++;
    }

    LOG_ERROR("Connection canceled");
    return false;
}

bool Astroid::ReadScopeStatus()
{

    uint8_t buf[200];
    int nbytes = 0, rc = 0;

    if ((rc = tty_read_section_expanded (PortFD, (char *)buf, 0x55,0, 100000, &nbytes)) != TTY_OK)
    {
        LOGF_WARN("Preamble not found, result: %d", rc);
        return false;
    }
    LOGF_DEBUG("discarded %d bytes", nbytes);

    if ((rc = tty_read_expanded (PortFD, (char *)buf, Status_message::SIZE, 0, 5000, &nbytes)) != TTY_OK)
    {
        LOGF_WARN("Error reading. Result: %d nbytes: %d", rc, nbytes, PortFD);
        return false;
    }

    tcflush(PortFD, TCIOFLUSH);

    char hex[200];
    hexDump(hex,buf,56);
    LOG_DEBUG("Message: ");
    LOG_DEBUG(hex);

    if(!last_status.set_buf(buf)){
        uint8_t checksum_calculated = 0;
        for(int i = 0; i < 55; i++){
            checksum_calculated += buf[i];
        }
        LOGF_WARN("Message NOK, checksum: received=%02X calculated=%02X",buf[55],checksum_calculated);
        return false;
    }


    double de = mod360((last_status.getDE() - sync_step_DE) / STEP_BY_TURN * 360. + sync_coord_DE +90)-90;
    double ha = mod24((last_status.getHA() - sync_step_HA) / STEP_BY_TURN * 24. + sync_coord_HA);
    double ra = mod24(get_local_sidereal_time(m_Location.longitude)-ha);

    char de_str[20], ha_str[20], ra_str[20];
    fs_sexa(de_str,de,4,360000);
    fs_sexa(ha_str,ha,4,360000);
    fs_sexa(ra_str,ra,4,360000);
    LOGF_DEBUG("DE:%s HA:%s RA:%s", de_str,ha_str,ra_str);

    _ra = ra;
    _de = de;

    //normalize_ra_de(&ra, &de);
    TelescopePierSide pier_side = (normalize_ra_de(&ra, &de) ? PIER_WEST : PIER_EAST);
    setPierSide(pier_side);

    fs_sexa(de_str,de,4,360000);
    fs_sexa(ha_str,ha,4,360000);
    fs_sexa(ra_str,ra,4,360000);
    if(pier_side == PIER_WEST){
        LOGF_DEBUG("--> Pier West (pointing east) DE:%s HA:%s RA:%s", de_str,ha_str,ra_str);
    }else{
        LOGF_DEBUG("--> Pier East (pointing west) DE:%s HA:%s RA:%s", de_str,ha_str,ra_str);
    }

    NewRaDec(ra, de);

    processGoto();


    return true;
}



void Astroid::processGoto(){

    if(!goto_active){
        return;
    }

    /*
    char de_sync_str[20], ha_sync_str[20];
    fs_sexa(de_sync_str,sync_coord_DE,4,360000);
    LOGF_INFO("sync_step_DE:%f sync_coord_DE:%s", sync_step_DE, last_status.getDE(), de_sync_str);
    LOGF_INFO("step_de:%d ustep_de:%f last_status.getDE():%f",last_status.step_de, last_status.ustep_de, last_status.getDE());
    fs_sexa(ha_sync_str,sync_coord_HA,4,360000);
    LOGF_INFO("sync_step_HA:%f last_status.getHA():%f sync_coord_HA:%s", sync_step_HA, last_status.getHA(), ha_sync_str);
    LOGF_INFO("step_ha:%d ustep_ha:%f last_status.getHA():%f",last_status.step_ha, last_status.ustep_ha, last_status.getHA());*/

    static struct timeval ltv { 0, 0 };
    struct timeval tv { 0, 0 };
    double dt = 0;
    /* update elapsed time since last poll, don't presume exactly POLLMS */
    gettimeofday(&tv, nullptr);
    if (ltv.tv_sec == 0 && ltv.tv_usec == 0)
        ltv = tv;
    dt  = tv.tv_sec - ltv.tv_sec + (tv.tv_usec - ltv.tv_usec) / 1e6;
    ltv = tv;


    bool ra_done = false, de_done = false;

    // DE
    double distance_DE = goto_target_DE - _de;
    if (fabs(distance_DE) < GOTO_STOP_DISTANCE){
        slew_DE_speed = 0;
        de_done = true;
    }else if(fabs(distance_DE) < GOTO_STOP_DISTANCE){
        slew_DE_speed = GOTO_SPEED*distance_DE/GOTO_SLOW_DISTANCE;
    }else{
        slew_DE_speed = GOTO_SPEED*(distance_DE > 0 ? 1 : -1);
    }

    // RA
    double lst = get_local_sidereal_time(m_Location.longitude);
    double target_HA, current_HA;
    //if(goto_target_DE < 90.){
        target_HA = rangeHA(lst - goto_target_RA);
    //    LOGF_INFO("goto_target_DE=%f < 90 target_HA=rangeHA(lst - goto_target_RA)=%f lst=%f goto_target_RA=%f",goto_target_DE,target_HA,lst,goto_target_RA);
    //}else{
    //    target_HA = rangeHA(lst - goto_target_RA + 12.);
    //    LOGF_INFO("goto_target_DE=%f >= 90 target_HA=rangeHA(lst - goto_target_RA + 12.)=%f lst=%f goto_target_RA=%f",goto_target_DE,target_HA,lst,goto_target_RA);
    //}
    //if(_de < 90.){
        current_HA = rangeHA(lst - _ra);
    //}else{
        current_HA = rangeHA(lst - _ra + 12.);
    //}

    double distance_RA = rangeHA(target_HA - current_HA) * 15.; // between -180 and 180
    if (fabs(distance_RA) < GOTO_STOP_DISTANCE){
        slew_RA_speed = 0;
        ra_done=true;
    }else if(fabs(distance_RA) < GOTO_SLOW_DISTANCE){
        slew_RA_speed = GOTO_SPEED*distance_RA/GOTO_SLOW_DISTANCE;
    }else{
        slew_RA_speed = GOTO_SPEED*(distance_RA > 0 ? 1 : -1);
    }
    LOGF_INFO("GOTO: dRA=%f sRA=%f dDE=%f sDE=%f tHA=%F cHA=%f tDE=%f cDE=%f lst=%f tRA=%f cRA=%f", distance_RA,slew_RA_speed, distance_DE,slew_DE_speed,target_HA,current_HA, goto_target_DE, _de, lst,goto_target_RA, _ra);

    goto_active = !(ra_done && de_done);
    if(ra_done && de_done){
        EqNP.s = IPS_OK;
        IDSetNumber(&EqNP, nullptr);
        LOG_INFO("GOTO successful");
        goto_active = false;
    }

    updateSpeed(false);

}


// Force de to be within -90..90 otherwise add +12h to ra and correct de. Return true if already in -90..90
bool Astroid::normalize_ra_de(double *ra, double *de){
    *de = mod360(*de+90)-90; // de is now within -90 and 270
    if(*de>90){
        *de=180-*de;
        *ra=mod24(*ra+12);
        return false;
    }
    return true;
}

bool Astroid::sendCommand(bool ack)
{
    int nbytes_written = 0;
    uint8_t cmd[32];
    command.get_bytes(cmd);
    //uint8_t cmd[]={0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00,0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x42};

    char hex_cmd[32 * 3 + 1] = {0};

    hexDump(hex_cmd, cmd, 32);

    LOGF_DEBUG("CMD %s", hex_cmd);

    errno = 0;

    nbytes_written = write(PortFD, cmd, 32);


    //tcflush(PortFD, TCIOFLUSH);

    if (nbytes_written != 32)
    {
        LOGF_ERROR("nbytes_written: %d < 32 errno=%d", nbytes_written, errno);
        return false;
    }

    tcdrain(PortFD);
    usleep(10000);
    tcflush(PortFD, TCIOFLUSH);
    usleep(10000);

    if(!ack){
        return true;
    }

    if(!ReadScopeStatus()){
        LOG_INFO("CMD check first try failed, errno=%d");
        if(!ReadScopeStatus()){
            LOG_ERROR("CMD check failed: cannot read status after 2 tries");
            return false;
        }
    }


    if( command.speed_ha==last_status.move_speed_ha &&
        command.speed_de==last_status.move_speed_de &&
        command.power_ha==last_status.power_ha &&
        command.power_de==last_status.power_de &&
        command.power_aux_1==last_status.power_aux_1 &&
        command.power_aux_2==last_status.power_aux_2 &&
        command.power_aux_3==last_status.power_aux_3 &&
        command.speed_focus==last_status.move_speed_focus){
        LOG_DEBUG("CMD check OK");
        return true;
    }

    LOGF_INFO("speed_ha: %f %f",command.speed_ha,last_status.move_speed_ha);
    LOGF_INFO("speed_de: %f %f",command.speed_de,last_status.move_speed_de);
    LOGF_INFO("speed_focus: %f %f",command.speed_focus,last_status.move_speed_focus);
    LOGF_INFO("power_ha: %f %f",command.power_ha,last_status.power_ha);
    LOGF_INFO("power_de: %f %f",command.power_de,last_status.power_de);
    LOGF_INFO("power_aux_1: %d %d",command.power_aux_1,last_status.power_aux_1);
    LOGF_INFO("power_aux_2: %d %d",command.power_aux_2,last_status.power_aux_2);
    LOGF_INFO("power_aux_3: %d %d",command.power_aux_3,last_status.power_aux_3);

    LOG_WARN("CMD check failed: incorrect return. Retrying");
    usleep(10000);
    tcflush(PortFD, TCIOFLUSH);
    usleep(10000);

    errno = 0;
    nbytes_written = write(PortFD, cmd, 32);

    if (nbytes_written != 32)
    {
        LOGF_ERROR("nbytes_written: %d < 32 errno=%d", nbytes_written, errno);
        return false;
    }

    tcdrain(PortFD);
    usleep(10000);

    if(!ReadScopeStatus()){
        LOG_DEBUG("CMD check first try failed");
        if(!ReadScopeStatus()){
            LOG_ERROR("CMD check failed: cannot read status after 2 tries");
            return false;
        }
    }

    if( command.speed_ha==last_status.move_speed_ha &&
        command.speed_de==last_status.move_speed_de &&
        command.power_ha==last_status.power_ha &&
        command.power_de==last_status.power_de &&
        command.power_aux_1==last_status.power_aux_1 &&
        command.power_aux_2==last_status.power_aux_2 &&
        command.power_aux_3==last_status.power_aux_3 &&
        command.speed_focus==last_status.move_speed_focus){
        LOG_DEBUG("CMD check OK");
        return true;
    }

    LOG_ERROR("CMD check failed: incorrect return x2");

    return false;
}

bool Astroid::Goto(double RA, double DE)
{
    int target_pier_side = IUFindOnSwitchIndex(&TargetPierSideSP);

    if(target_pier_side == PIER_AUTO){
        target_pier_side = expectedPierSide(RA);
    }
    if(target_pier_side == PIER_CURRENT){
        target_pier_side = getPierSide();
    }
    if(target_pier_side == PIER_UNKNOWN){
        target_pier_side = PIER_WEST;
    }

    normalize_ra_de(&RA, &DE);

    char RAStr[20]={0}, DecStr[20]={0};
    fs_sexa(RAStr, RA, 2, 3600);
    fs_sexa(DecStr, DE, 2, 3600);

    if(target_pier_side == PIER_WEST){
        goto_target_RA = RA;
        goto_target_DE = DE;
        LOGF_INFO("Slewing to:  RA %s - DEC %s - Pier West", RAStr, DecStr);
    }else{
        goto_target_RA = range24(RA + 12);
        goto_target_DE = 180. - DE;
        LOGF_INFO("Slewing to:  RA %s - DEC %s - Pier East", RAStr, DecStr);
    }

    goto_active = true;
    TrackState = SCOPE_SLEWING;

    return true;
}

bool Astroid::Sync(double RA, double DE)
{
    int target_pier_side = IUFindOnSwitchIndex(&TargetPierSideSP);

    if(target_pier_side == PIER_AUTO){
        target_pier_side = expectedPierSide(RA);
        LOGF_INFO("Syncing to RA %f autodetected PIER %d", RA, target_pier_side);
        LOGF_INFO("get_local_sidereal_time: %f m_Location.longitude%: %f get_local_hour_angle(lst, ra): %f", get_local_sidereal_time(m_Location.longitude), m_Location.longitude, get_local_hour_angle(get_local_sidereal_time(m_Location.longitude), RA));
    }
    if(target_pier_side == PIER_CURRENT){
        target_pier_side = getPierSide();
    }
    if(target_pier_side == PIER_UNKNOWN){
        target_pier_side = PIER_WEST;
    }

    normalize_ra_de(&RA, &DE);


    char RAStr[20]={0}, DecStr[20]={0};
    fs_sexa(RAStr, RA, 2, 3600);
    fs_sexa(DecStr, DE, 2, 3600);

    sync_step_HA = last_status.getHA();
    sync_step_DE = last_status.getDE();
    if(target_pier_side == PIER_WEST){
        sync_coord_HA = range24(get_local_sidereal_time(m_Location.longitude) - RA);
        sync_coord_DE = DE;
        LOGF_INFO("Sync to:  RA %s - DEC %s - Pier West", RAStr, DecStr);
    }else{
        sync_coord_HA = range24(get_local_sidereal_time(m_Location.longitude) - RA + 12);
        sync_coord_DE = 180. - DE;
        LOGF_INFO("Sync to:  RA %s - DEC %s - Pier East", RAStr, DecStr);
    }

    return true;
}



bool Astroid::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Guide Rate
        if (strcmp(name, "GUIDE_RATE") == 0)
        {
            IUUpdateNumber(&GuideRateNP, values, names, n);
            GuideRateNP.s = IPS_OK;
            IDSetNumber(&GuideRateNP, nullptr);
            return true;
        }

        // For guiding pulse, let's pass the properties up to the guide framework
        if (strcmp(name, GuideNSNP.name) == 0 || strcmp(name, GuideWENP.name) == 0)
        {
            processGuiderProperties(name, values, names, n);
            return true;
        }
    }

    // Otherwise, send it up the chains to INDI::Telescope to process any further properties
    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

/*bool Astroid::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n){

    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        //  This one is for us
        if (!strcmp(name, TargetPierSideSP.name)){
            TargetPierSideSP.s = IPS_OK;
            IUUpdateSwitch(&TargetPierSideSP, states, names, n);
            //  Update client display
            IDSetSwitch(&TargetPierSideSP, nullptr);
            return true;
        }
    }
    return ISNewSwitch(dev, name, states, names, n);
}*/

bool Astroid::updateSpeed(bool ack){

    if(TrackState == SCOPE_SLEWING){
        switch(track_mode){
            case TRACK_SIDEREAL:
                track_speed_HA = 1;
                track_speed_DE = 0;
                break;
            case TRACK_SOLAR:
                track_speed_HA = TRACKRATE_SOLAR/TRACKRATE_SIDEREAL;
                track_speed_DE = 0;
                break;
            case TRACK_LUNAR:
                track_speed_HA = TRACKRATE_LUNAR/TRACKRATE_SIDEREAL;
                track_speed_DE = 0;
                break;
            case TRACK_CUSTOM:
                track_speed_HA = track_custom_speed_HA;
                track_speed_DE = track_custom_speed_DE;
                break;
            default:
                track_speed_HA = 1;
                track_speed_DE = 0;
        }
    }else{
        track_speed_HA = 0;
        track_speed_DE = 0;
    }

    double speed_DE = (track_speed_DE+slew_DE_speed);
    double speed_HA = track_speed_HA-slew_RA_speed;

    command.speed_ha = speed_HA;
    command.speed_de = speed_DE;
    command.power_ha = power_HA;
    command.power_de = power_DE;


    if(!sendCommand(ack)){
        TrackRateNP.s=IPS_ALERT;
        IDSetNumber(&TrackRateNP, "Failed to update the speed");
    }

    TrackRateN[AXIS_RA].value=track_speed_HA*TRACKRATE_SIDEREAL;
    TrackRateN[AXIS_DE].value=track_speed_DE*TRACKRATE_SIDEREAL;
    TrackRateNP.s = (speed_HA == 0) && (speed_DE = 0 )? IPS_IDLE : IPS_OK;
    IDSetNumber(&TrackRateNP, nullptr);

    return true;
}

bool Astroid::Abort()
{
    goto_active = false;
    slew_DE_speed = 0;
    slew_RA_speed = 0;
    track_speed_DE = 0;
    track_speed_HA = fmax(fmin(1,track_speed_HA),0); // clip between 0 and 1

    return updateSpeed();
}

bool Astroid::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    if(command==MOTION_START){
        slew_DE_speed = (dir == DIRECTION_NORTH ? -1.: 1.) * motion_speeds[IUFindOnSwitchIndex(&SlewRateSP)] * (getPierSide() == PIER_EAST ? 1 : -1);
    }else{ // MOTION_STOP
        slew_DE_speed = 0;
    }

    return updateSpeed();
}

bool Astroid::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    if(command==MOTION_START){
        slew_RA_speed = (dir == DIRECTION_EAST ? 1.: -1.) * motion_speeds[IUFindOnSwitchIndex(&SlewRateSP)];
    }else{ // MOTION_STOP
        slew_RA_speed = 0;
    }

    return updateSpeed();
}

IPState Astroid::GuideNorth(uint32_t ms)
{
    LOGF_DEBUG("Guiding: N %.0f ms", ms);

    if (GuideNSTID)
    {
        IERmTimer(GuideNSTID);
        GuideNSTID = 0;
    }

    slew_DE_speed = GuideRateN[AXIS_DE].value * (getPierSide() == PIER_EAST ? 1 : -1);

    if (!updateSpeed())
        return IPS_ALERT;

    GuideNSTID = IEAddTimer(static_cast<int>(ms), stopNSPulseHelper, this);
    return IPS_BUSY;
}

IPState Astroid::GuideSouth(uint32_t ms)
{
    LOGF_DEBUG("Guiding: S %.0f ms", ms);

    if (GuideNSTID)
    {
        IERmTimer(GuideNSTID);
        GuideNSTID = 0;
    }

    slew_DE_speed = -GuideRateN[AXIS_DE].value * (getPierSide() == PIER_EAST ? 1 : -1);

    if (!updateSpeed())
        return IPS_ALERT;


    GuideNSTID = IEAddTimer(static_cast<int>(ms), stopNSPulseHelper, this);
    return IPS_BUSY;
}

IPState Astroid::GuideEast(uint32_t ms)
{
    LOGF_DEBUG("Guiding: E %.0f ms", ms);

    if (GuideWETID)
    {
        IERmTimer(GuideWETID);
        GuideWETID = 0;
    }

    slew_RA_speed = GuideRateN[AXIS_RA].value;

    if (!updateSpeed())
        return IPS_ALERT;

    GuideWETID = IEAddTimer(static_cast<int>(ms), stopWEPulseHelper, this);
    return IPS_BUSY;
}

IPState Astroid::GuideWest(uint32_t ms)
{
    LOGF_DEBUG("Guiding: W %.0f ms", ms);

    if (GuideWETID)
    {
        IERmTimer(GuideWETID);
        GuideWETID = 0;
    }

    slew_RA_speed = -GuideRateN[AXIS_RA].value;

    if (!updateSpeed())
        return IPS_ALERT;

    GuideWETID = IEAddTimer(static_cast<int>(ms), stopWEPulseHelper, this);
    return IPS_BUSY;
}

void Astroid::stopNSPulseHelper(void *p)
{
    static_cast<Astroid *>(p)->stopNSPulse();
}

void Astroid::stopWEPulseHelper(void *p)
{
    static_cast<Astroid *>(p)->stopWEPulse();
}

void Astroid::stopNSPulse(){
    slew_DE_speed = 0;
    if(updateSpeed()){
        GuideNSNP.s = IPS_IDLE;
        LOG_DEBUG("Guiding: DEC axis stopped.");
    }else{
        GuideNSNP.s = IPS_ALERT;
        LOG_ERROR("Failed to stop DEC axis.");
    }

    GuideNSTID = 0;
    GuideNSNP.np[0].value = 0;
    GuideNSNP.np[1].value = 0;
    IDSetNumber(&GuideNSNP, nullptr);
}

void Astroid::stopWEPulse(){
    slew_RA_speed = 0;
    if(updateSpeed()){
        GuideWENP.s = IPS_IDLE;
        LOG_DEBUG("Guiding: RA axis stopped.");
    }else{
        GuideWENP.s = IPS_ALERT;
        LOG_ERROR("Failed to stop RA axis.");
    }

    GuideWETID = 0;
    GuideWENP.np[0].value = 0;
    GuideWENP.np[1].value = 0;
    IDSetNumber(&GuideWENP, nullptr);
}


bool Astroid::SetTrackMode(uint8_t mode)
{
    track_mode = mode;

    return updateSpeed();
}

bool Astroid::SetTrackEnabled(bool enabled)
{
    TrackState = (enabled ? SCOPE_SLEWING : SCOPE_IDLE);

    return updateSpeed();
}

bool Astroid::SetTrackRate(double raRate, double deRate)
{
    track_custom_speed_HA = raRate / TRACKRATE_SIDEREAL;
    track_custom_speed_DE = deRate / TRACKRATE_SIDEREAL;

    return updateSpeed();
}



void Astroid::hexDump(char * buf, const uint8_t * data, int size)
{
    for (int i = 0; i < size; i++)
        sprintf(buf + 3 * i, "%02X ", data[i]);

    if (size > 0)
        buf[3 * size - 1] = '\0';
}

double Astroid::mod360(double x) {

    double res = fmod(x,360.);
    res += (res < 0 ? 360. : 0);
    return res;
}

double Astroid::mod24(double x) {

    double res = fmod(x,24.);
    res += (res < 0 ? 24. : 0);
    return res;
}
