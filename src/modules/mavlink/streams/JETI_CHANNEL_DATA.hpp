/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

 #ifndef JETI_CHANNEL_DATA_HPP
 #define JETI_CHANNEL_DATA_HPP

   #include <uORB/topics/jeti_channel_data.h>

   class MavlinkStreamJetiChannelData : public MavlinkStream
   {
   public:
       static MavlinkStream *new_instance(Mavlink *mavlink)
       {
	   return new MavlinkStreamJetiChannelData(mavlink);
       }
       const char *get_name() const
       {
	   return MavlinkStreamJetiChannelData::get_name_static();
       }
       static const char *get_name_static()
       {
	   return "JETI_CHANNEL_DATA";
       }
       static uint16_t get_id_static()
       {
	   return MAVLINK_MSG_ID_JETI_CHANNEL_DATA;
       }
       uint16_t get_id()
       {
	   return get_id_static();
       }
       unsigned get_size()
       {
	   return MAVLINK_MSG_ID_JETI_CHANNEL_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
       }

   private:
       //Subscription to array of uORB battery status instances
       uORB::SubscriptionMultiArray<jeti_channel_data_s> _jeti_channel_data_subs{ORB_ID::jeti_channel_data};
       // SubscriptionMultiArray subscription is needed because battery has multiple instances.
       // uORB::Subscription is used to subscribe to a single-instance topic

       /* do not allow top copying this class */
       MavlinkStreamJetiChannelData(MavlinkStreamJetiChannelData &);
       MavlinkStreamJetiChannelData& operator = (const MavlinkStreamJetiChannelData &);

   protected:
       explicit MavlinkStreamJetiChannelData(Mavlink *mavlink) : MavlinkStream(mavlink)
       {}

	   bool send() override
	   {
		   bool updated = false;

		   // Loop through _battery_status_subs (subscription to array of BatteryStatus instances)
		   for (auto &jeti_channel_data_sub : _jeti_channel_data_subs) {
	       // battery_status_s is a struct that can hold the battery object topic
			   jeti_channel_data_s jeti_channel_data;

			   // Update battery_status and publish only if the status has changed
			   if (jeti_channel_data_sub.update(&jeti_channel_data)) {
		   // mavlink_battery_status_demo_t is the MAVLink message object
				   mavlink_jeti_channel_data_t jeti_channel_data_msg{};

				   jeti_channel_data_msg.timestamp = jeti_channel_data.timestamp;
				   jeti_channel_data_msg.crc16 = jeti_channel_data.crc16;


		   //Send the message
				   mavlink_msg_jeti_channel_data_send_struct(_mavlink->get_channel(), &jeti_channel_data_msg);
				   updated = true;
			   }
		   }

		   return updated;
	   }

   };
   #endif // JETI_CHANNEL_DATA_HPP
