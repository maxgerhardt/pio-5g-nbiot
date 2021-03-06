#include "5G-NB-IoT_MQTT.h"

_5G_NB_IoT_MQTT::_5G_NB_IoT_MQTT()
{

}

_5G_NB_IoT_MQTT::~_5G_NB_IoT_MQTT()
{

}

_5G_NB_IoT_MQTT::_5G_NB_IoT_MQTT(Stream &atserial, Stream &dserial) : _5G_NB_IoT_SSL(atserial, dserial)
{

}

bool _5G_NB_IoT_MQTT::SetMQTTConfigureParameters(unsigned int mqtt_index, unsigned int pdp_index, Mqtt_Version_t version, unsigned int keep_alive_time, Mqtt_Session_Type_t type)
{
    char cmd[32], buf[32];
    strcpy(cmd, MQTT_CONFIG_PARAMETER);
    sprintf(buf, "=\"version\",%d,%d", mqtt_index, version);
    strcat(cmd, buf);
    if(!sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 2)){
        return false;
    }
    memset(cmd, '\0', 32);
    memset(buf, '\0', 32);
    strcpy(cmd, MQTT_CONFIG_PARAMETER);
    sprintf(buf, "=\"pdpcid\",%d,%d", mqtt_index, pdp_index);
    strcat(cmd, buf);
    if(!sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 2)){
        return false;
    }
    memset(cmd, '\0', 32);
    memset(buf, '\0', 32);
    strcpy(cmd, MQTT_CONFIG_PARAMETER);
    sprintf(buf, "=\"keepalive\",%d,%d", mqtt_index, keep_alive_time);
    strcat(cmd, buf);
    if(!sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 2)){
        return false;
    }
    memset(cmd, '\0', 32);
    memset(buf, '\0', 32);
    strcpy(cmd, MQTT_CONFIG_PARAMETER);
    sprintf(buf, "=\"session\",%d,%d", mqtt_index, type);
    strcat(cmd, buf);
    if(!sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 2)){
        return false;
    }
    return true;
}

bool _5G_NB_IoT_MQTT::SetMQTTEnableSSL(unsigned int mqtt_index, unsigned int ssl_index, bool enable)
{
    char cmd[32], buf[16];
    strcpy(cmd, MQTT_CONFIG_PARAMETER);
    if(enable){
        sprintf(buf, "=\"ssl\",%d,1,%d", mqtt_index, ssl_index);
    }else {
        sprintf(buf, "=\"ssl\",%d,0,%d", mqtt_index, ssl_index);
    }
    strcat(cmd, buf);
    if(sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 2)){
        return true;
    }
    return false;
}

bool _5G_NB_IoT_MQTT::SetMQTTMessageTimeout(unsigned int mqtt_index, unsigned int pkt_timeout, unsigned int retry_num, bool timeout_notice)
{
    char cmd[32], buf[32];
    strcpy(cmd, MQTT_CONFIG_PARAMETER);
    if(timeout_notice){
        sprintf(buf, "=\"timeout\",%d,%d,%d,1", mqtt_index, pkt_timeout, retry_num);
    }else {
        sprintf(buf, "=\"timeout\",%d,%d,%d,0", mqtt_index, pkt_timeout, retry_num);
    }
    strcat(cmd, buf);
    if(sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 2)){
        return true;
    }
    return false;
}

bool _5G_NB_IoT_MQTT::SetMQTTAlibabaCloudDeviceInformation(unsigned int mqtt_index, char *product_key, char *device_name, char *device_secret)
{
    char cmd[128], buf[128];
    strcpy(cmd, MQTT_CONFIG_PARAMETER);
    sprintf(buf, "=\"aliauth\",%d,\"%s\",\"%s\",\"%s\"", mqtt_index, product_key, device_name, device_secret);
    strcat(cmd, buf);
    if(sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 2)){
        return true;
    }
    return false;
}

Mqtt_Network_Result_t _5G_NB_IoT_MQTT::OpenMQTTNetwork(unsigned int mqtt_index, char *host_name, unsigned int port)
{
    char cmd[128], buf[128];
    strcpy(cmd, MQTT_OPEN_NETWORK);
    sprintf(buf, "=%d,\"%s\",%d", mqtt_index, host_name, port);
    strcat(cmd, buf);
    if(sendAndSearch(cmd, MQTT_OPEN_NETWORK, RESPONSE_ERROR, 150)){
        unsigned long start_time = millis();
        while(millis() - start_time < 200UL){
            if(serialAvailable()){
                readResponseByteToBuffer();
            }
        }
        char *sta_buf = searchChrBuffer(',');
        return (Mqtt_Network_Result_t)atoi(sta_buf + 1);
    }
    return (Mqtt_Network_Result_t)-2;
}

bool _5G_NB_IoT_MQTT::CloseMQTTNetwork(unsigned int mqtt_index)
{
    char cmd[16], buf[8];
    strcpy(cmd, MQTT_CLOSE_NETWORK);
    sprintf(buf, "=%d", mqtt_index);
    strcat(cmd, buf);
    if(sendAndSearch(cmd, MQTT_CLOSE_NETWORK, RESPONSE_ERROR, 2)){
        unsigned long start_time = millis();
        while(millis() - start_time < 200UL){
            if(serialAvailable()){
                readResponseByteToBuffer();
            }
        }
        char *sta_buf = searchChrBuffer(',');
        if(atoi(sta_buf + 1) == 0){
            return true;
        }
    }
    return false;
}

Mqtt_Client_Result_Status_t _5G_NB_IoT_MQTT::CreateMQTTClient(unsigned int mqtt_index, char *client_id, char *username, char *password)
{
    char cmd[128], buf[128];
    strcpy(cmd, MQTT_CREATE_CLIENT);
    if (username != "" && password != ""){
        sprintf(buf, "=%d,\"%s\",\"%s\",\"%s\"", mqtt_index, client_id, username, password);
    } else {
        sprintf(buf, "=%d,\"%s\"", mqtt_index, client_id);
    }
    strcat(cmd, buf);
    if (sendAndSearch(cmd, MQTT_CREATE_CLIENT, RESPONSE_ERROR, 150)){
        unsigned long start_time = millis();
        while((millis() - start_time) < 200UL) {
            if(serialAvailable()) {
                readResponseByteToBuffer();
            }
        }
        char temp[16];
        char *sta_buf = searchChrBuffer(',');
        strcpy(temp, sta_buf + 1);
        sta_buf = strchr(temp, ',');
        *sta_buf = '\0';
        return (Mqtt_Client_Result_Status_t)atoi(temp);
    }
    return (Mqtt_Client_Result_Status_t)-1;
}

bool _5G_NB_IoT_MQTT::CloseMQTTClient(unsigned int mqtt_index)
{
    char cmd[16], buf[8];
    strcpy(cmd, MQTT_CLOSE_CLIENT);
    sprintf(buf, "=%d", mqtt_index);
    strcat(cmd, buf);
    if(sendAndSearch(cmd, MQTT_CLOSE_CLIENT, RESPONSE_OK, 2)){
        unsigned long start_time = millis();
        while(millis() - start_time < 200UL){
            if(serialAvailable()){
                readResponseByteToBuffer();
            }
        }
        char *sta_buf = searchChrBuffer(',');
        if(atoi(sta_buf + 1) == 0){
            return true;
        }
    }
    return false;
}

Mqtt_Client_Result_Status_t _5G_NB_IoT_MQTT::MQTTSubscribeTopic(unsigned int mqtt_index, unsigned int msg_id, char *topic, Mqtt_Qos_t qos)
{
    char cmd[512], buf[512];
    strcpy(cmd, MQTT_SUBSCRIBE_TOPICS);
    sprintf(buf, "=%d,%d,\"%s\",%d", mqtt_index, msg_id, topic, qos);
    strcat(cmd, buf);
    if (sendAndSearch(cmd, MQTT_SUBSCRIBE_TOPICS, RESPONSE_ERROR, 150)){
        unsigned long start_time = millis();
        while(millis() - start_time < 200UL){
            if(serialAvailable()){
                readResponseByteToBuffer();
            }
        }
        int i = 0;
        char temp[16], *p[4];
        char *sta_buf = searchChrBuffer(',');
        strcpy(temp, sta_buf + 1);
        p[0] = strtok(temp, ",");
        while(p[i] != NULL){
            i++;
            p[i] = strtok(NULL,",");
        }
        p[i] = '\0';
        return (Mqtt_Client_Result_Status_t)atoi(p[1]);
    }
    return (Mqtt_Client_Result_Status_t)-1;
}

Mqtt_Client_Result_Status_t _5G_NB_IoT_MQTT::MQTTUnsubscribeTopic(unsigned int mqtt_index, unsigned int msg_id, char *topic)
{
    char cmd[40], buf[32];
    strcpy(cmd, MQTT_UNSUBSCRIBE_TOPICS);
    sprintf(buf, "=%d,%d,\"%s\"", mqtt_index, msg_id, topic);
    strcat(cmd, buf);
    if(sendAndSearch(cmd, MQTT_UNSUBSCRIBE_TOPICS, RESPONSE_ERROR, 150)){
        unsigned long start_time = millis();
        while(millis() - start_time < 200UL){
            if(serialAvailable()){
                readResponseByteToBuffer();
            }
        }
        char temp[16];
        char *sta_buf = searchChrBuffer(',');
        strcpy(temp, sta_buf + 1);
        sta_buf = strchr(temp, ',');
        *sta_buf = '\0';
        return (Mqtt_Client_Result_Status_t)atoi(temp);
    }
    return (Mqtt_Client_Result_Status_t)-1;
}

Mqtt_Client_Result_Status_t _5G_NB_IoT_MQTT::MQTTPublishMessages(unsigned int mqtt_index, unsigned int msg_id, Mqtt_Qos_t qos, char *topic, bool retain, char *publish_data)
{
    char cmd[512], buf[128];
    strcpy(cmd, MQTT_PUBLISH_MESSAGES);
    if(retain){
        sprintf(buf, "=%d,%d,%d,1,\"%s\"", mqtt_index, msg_id, qos, topic);
    } else {
        sprintf(buf, "=%d,%d,%d,0,\"%s\"", mqtt_index, msg_id, qos, topic);
    }
    strcat(cmd, buf);
    if (sendAndSearchChr(cmd, '>', 2)){
	
	strcpy(cmd, publish_data);
	
	char ctrl_z[] = { (char)0x1A, '\0' };

//        strcat(publish_data, ctrl_z);
        strcat(cmd, ctrl_z);

//        if (sendDataAndCheck(publish_data, MQTT_PUBLISH_MESSAGES, RESPONSE_ERROR, 150)){
        if (sendDataAndCheck(cmd, MQTT_PUBLISH_MESSAGES, RESPONSE_ERROR, 150)){
            unsigned long start_time = millis();
            while(millis() - start_time < 500UL){
                if(serialAvailable()){
                    readResponseByteToBuffer();
                }
            }
            memset(publish_data, '\0', strlen(publish_data));
            char temp[16];
            char *sta_buf = searchChrBuffer(',');
            strcpy(temp, sta_buf + 1);
            sta_buf = strchr(temp, ',');
            *sta_buf = '\0';
            return (Mqtt_Client_Result_Status_t)atoi(temp);
        }
    }
    return (Mqtt_Client_Result_Status_t)-1;
}

Mqtt_URC_Event_t _5G_NB_IoT_MQTT::WaitCheckMQTTURCEvent(char *event, unsigned int timeout)
{
    Cmd_Response_t ret = readResponseAndSearch(MQTT_RECV_DATA, MQTT_STATUS, timeout);
    unsigned long start_time = millis();
    while (millis() - start_time < 200UL){
        if (serialAvailable()){
            readResponseByteToBuffer();
        }
    }
    if (ret == SUCCESS_RESPONSE){
        char *sta_buf = searchStrBuffer(": ");
        //strcpy(event, sta_buf + 2);
		
		// Data are received in JSON format which starts with "{" and ends with "}"
		// Example 
		//+QMTRECV: 3,0,"MyTopic","{ 
		//  "DeviceID": "866425031237797", 
		//	"Timestamp": 122219, 
		//	"Device": "Servo", 
		//	"OpCode": "Write", 
		//	"Angle": 30, 
		//	"Unit": "Degree" 
		//	}"
		char *JsonData = strchr(sta_buf + 2, '{');		
        strcpy(event, JsonData);		
        return MQTT_RECV_DATA_EVENT;
    } else if (ret == FIAL_RESPONSE){
        char *sta_buf = searchStrBuffer(": ");
        strcpy(event, sta_buf + 2);
        return MQTT_STATUS_EVENT;
    }
    return (Mqtt_URC_Event_t)-1;
}