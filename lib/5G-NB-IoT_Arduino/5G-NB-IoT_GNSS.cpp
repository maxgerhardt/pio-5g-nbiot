#include "5G-NB-IoT_GNSS.h"

_5G_NB_IoT_GNSS::_5G_NB_IoT_GNSS()
{

}

_5G_NB_IoT_GNSS::~_5G_NB_IoT_GNSS()
{

}

_5G_NB_IoT_GNSS::_5G_NB_IoT_GNSS(Stream &atserial, Stream &dserial) : _5G_NB_IoT_Common(atserial, dserial)
{

}

bool _5G_NB_IoT_GNSS::SetGNSSConstellation(GNSS_Constellation_t constellation)
{
    char cmd[32], buf[32];
    strcpy(cmd, GNSS_CONFIGURATION);
    sprintf(buf, "=\"gnssconfig\",%d", constellation);
    strcat(cmd, buf);
    if(sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 2)){
        return true;
    }
    return false;
}

bool _5G_NB_IoT_GNSS::SetGNSSAutoRun(bool auto_run)
{
    char cmd[32];
    strcpy(cmd, GNSS_CONFIGURATION);
    if(auto_run){
        strcat(cmd, "\"autogps\",1");
        if(sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 2)){
            return true;
        }
    }else {
        strcat(cmd, "\"autogps\",0");
        if(sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 2)){
            return true;
        }
    }
    return false;
}

bool _5G_NB_IoT_GNSS::SetGNSSEnableNMEASentences(bool enable)
{
    char cmd[32];
    strcpy(cmd, GNSS_CONFIGURATION);
    if (enable){
        strcat(cmd, "=\"nmeasrc\",1");
    }else {
        strcat(cmd, "=\"nmeasrc\",0");
    }
    if(sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 2)){
        return true;
    }
    return false;
}

bool _5G_NB_IoT_GNSS::TurnOnGNSS(GNSS_Work_Mode_t mode, Cmd_Status_t status)
{
    char cmd[16],buf[8];
    strcpy(cmd, GNSS_TURN_ON);
    if (status == READ_MODE){
        strcat(cmd, "?");
        if(sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 2)){
            char *sta_buf = searchStrBuffer(": ");
            char *end_buf = searchStrBuffer(RESPONSE_CRLF_OK);
            *end_buf = '\0';
            if(atoi(sta_buf + 2) == 1){
                return true;
            }
        }
    }else if (status == WRITE_MODE){
        sprintf(buf, "=%d", mode);
        strcat(cmd, buf);
        if(sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 10)){
            return true;
        }
    }
    return false;
}

bool _5G_NB_IoT_GNSS::TurnOffGNSS()
{
    char cmd[16];
    strcpy(cmd, GNSS_TURN_OFF);
    if (sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 10)){
        return true;
    }
    return false;
}

bool _5G_NB_IoT_GNSS::GetGNSSPositionInformation(char *position)
{
    char cmd[16];
    strcpy(cmd, GNSS_GET_POSITION);
    strcat(cmd, "=2");
    if(sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 10)){
        char *end_buf = searchStrBuffer(RESPONSE_CRLF_OK);
        *end_buf = '\0';
        char *sta_buf = searchStrBuffer(": ");
        strcpy(position, sta_buf + 2);
        return true;
    }
    return false;
}

bool _5G_NB_IoT_GNSS::GetGNSSNMEASentences(NMEA_Type_t type, char *sentences)
{
    char cmd[32];
    strcpy(cmd, GNSS_ACQUIRE_NMEA);
    switch(type)
    {
        case GPGGA:
            strcat(cmd, "=\"GCA\"");
            break;
        case GPRMC:
            strcat(cmd, "=\"RMC\"");
            break;
        case GPGSV:
            strcat(cmd, "=\"GSV\"");
            break;
        case GPGSA:
            strcat(cmd, "=\"GSA\"");
            break;
        case GPVTG:
            strcat(cmd, "=\"VTG\"");
            break;
        default:
            return false;
    }
    if(sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 10)){
        char *sta_buf = searchStrBuffer(": ");
        char *end_buf = searchStrBuffer(RESPONSE_CRLF_OK);
        *end_buf = '\0';
        strcpy(sentences, sta_buf + 2);
        return true;
    }
    return false;
}

bool _5G_NB_IoT_GNSS::SetGNSSOutputPort(GNSS_OutputPort_t outport)
{
	char cmd[32];
	strcpy(cmd, GNSS_CONFIGURATION);
	if (outport == NOPORT) {
		strcat(cmd, "=\"outport\",\"none\"");
	}
	else if (outport == USBNMEA) {
		strcat(cmd, "=\"outport\",\"usbnmea\"");
	}
	else if (outport == UARTNMEA) {
		strcat(cmd, "=\"outport\",\"uartnmea\"");
	}
	if (sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 2)) {
		return true;
	}
	return false;
}

