
#ifndef MAVLINK_BRIDGE_HEADER_H_
#define MAVLINK_BRIDGE_HEADER_H_

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "mavlink/mavlink_types.h"
#include "serial/serial.h"

mavlink_system_t mavlink_system;

static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{

    if (chan == MAVLINK_COMM_0)
    {
    	serial::Serial sc("/dev/ttyAMA0",57600);
    	sc.write(&ch,1);
    	sc.close();
    }
}


#endif /* MAVLINK_BRIDGE_HEADER_H_ */
