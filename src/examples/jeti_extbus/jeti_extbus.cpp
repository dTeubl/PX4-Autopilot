/****************************************************************************
 *
 *   Copyright (c) 2012-2024 PX4 Development Team. All rights reserved.
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
/**
 * @file jeti_extbus.cpp
 *
 * @author Daniel Teubl <teubl.dani@gmail.com>
 */

#include <math.h>
#include <poll.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <unistd.h>

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/uORB.h>

extern "C" __EXPORT int jeti_extbus_main(int argc, char *argv[]);

auto jeti_extbus_main(int argc, char *argv[]) -> int {
	PX4_INFO("Hello Sky Sim!");

	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

	orb_set_interval(sensor_sub_fd, 200);

	/* advertise attitude topic */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	px4_pollfd_struct_t fds[] = {
	    {.fd = sensor_sub_fd, .events = POLLIN},
	};

	for (auto cnt{0}; cnt < 10; ++cnt) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1
		 * second) */
		auto poll_ret = px4_poll(fds, 1, 1000);
		PX4_INFO("INFO: %d", poll_ret);
		if (fds[0].revents & POLLIN) {
			/* obtained data for the first file descriptor */
			struct sensor_combined_s raw;
			orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);

			PX4_INFO("Accelerometer:\t%d\t%8.4f\t%8.4f\t%8.4f", cnt,
				 (double)raw.accelerometer_m_s2[0],
				 (double)raw.accelerometer_m_s2[1],
				 (double)raw.accelerometer_m_s2[2]);

			att.q[0] = 0.5 * cnt;
			att.q[1] = cnt;
			att.q[2] = raw.accelerometer_m_s2[2];

			orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
		}
	}

	return OK;
}

