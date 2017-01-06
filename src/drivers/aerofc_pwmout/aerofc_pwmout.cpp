/****************************************************************************
 *
 *   Copyright (c)  2017 Intel Corporation. All rights reserved.
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

#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>
#include <termios.h>
#include <cmath>

#include <systemlib/px4_macros.h>
#include <drivers/device/device.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/test_motor.h>
#include <uORB/topics/multirotor_motor_limits.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/param/param.h>
#include <systemlib/pwm_limit/pwm_limit.h>

/*
 * This driver connects to ESCs via serial.
 */

#define AEROFC_PWMOUT_MAX_MOTOR_NUM 16

static int _uart_fd = -1;
class AEROFC_PWMOUT : public device::CDev
{
public:
	AEROFC_PWMOUT(int channels_count);
	virtual ~AEROFC_PWMOUT();
	virtual int ioctl(file *filp, int cmd, unsigned long arg);
	void cycle();

private:
	const uint16_t RPMSTOPPED = 800;
	const uint16_t RPMMAX = 2300;

	bool _is_armed;

	unsigned	_poll_fds_num;
	// subscriptions
	int	_armed_sub;
	int _test_motor_sub;
	orb_advert_t        	_outputs_pub = nullptr;
	actuator_outputs_s      _outputs;
	static actuator_armed_s	_armed;

	// It needs to support the numbe of ESC
	int	_control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

	pollfd	_poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

	actuator_controls_s _controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

	orb_id_t	_control_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

	orb_advert_t      _to_mixer_status; 	///< mixer status flags
	uint8_t           _channels_count; // The number of ESC channels

	MixerGroup	*_mixers;
	uint32_t	_groups_required;
	uint32_t	_groups_subscribed;
	volatile bool	_initialized;
	unsigned	_pwm_default_rate;
	unsigned	_current_update_rate;
	void		subscribe();

	void		work_stop();
	uint16_t crc16(uint8_t *p, uint8_t len);
	void send_esc_outputs(const float *pwm, const unsigned num_pwm);
	static int control_callback(uintptr_t handle,
				    uint8_t control_group,
				    uint8_t control_index,
				    float &input);
};

actuator_armed_s AEROFC_PWMOUT::_armed = {};

namespace
{
AEROFC_PWMOUT	*aerofc_pwmout = nullptr;
}

# define AEROFC_PWMOUT_DEVICE_PATH	"/dev/aerofc_pwmout"

AEROFC_PWMOUT::AEROFC_PWMOUT(int channels_count):
	CDev("aerofc_pwmout", AEROFC_PWMOUT_DEVICE_PATH),
	_is_armed(false),
	_poll_fds_num(0),
	_armed_sub(-1),
	_test_motor_sub(-1),
	_outputs_pub(nullptr),
	_control_subs{ -1},
	_to_mixer_status(nullptr),
	_channels_count(channels_count),
	_mixers(nullptr),
	_groups_required(0),
	_groups_subscribed(0),
	_initialized(false),
	_pwm_default_rate(400),
	_current_update_rate(0)

{
	_control_topics[0] = ORB_ID(actuator_controls_0);
	_control_topics[1] = ORB_ID(actuator_controls_1);
	_control_topics[2] = ORB_ID(actuator_controls_2);
	_control_topics[3] = ORB_ID(actuator_controls_3);
	memset(_controls, 0, sizeof(_controls));
	memset(_poll_fds, 0, sizeof(_poll_fds));

	for (size_t i = 0; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
		_outputs.output[i] = NAN;
	}

	_outputs.noutputs = 0;
}

AEROFC_PWMOUT::~AEROFC_PWMOUT()
{
	if (_initialized) {
		/* tell the task we want it to go away */
		work_stop();

		int i = 10;

		do {
			/* wait 50ms - it should wake every 100ms or so worst-case */
			usleep(50000);
			i--;

		} while (_initialized && i > 0);
	}

	// clean up the alternate device node
	//unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	aerofc_pwmout = nullptr;
}

void AEROFC_PWMOUT::subscribe()
{
	/* subscribe/unsubscribe to required actuator control groups */
	uint32_t sub_groups = _groups_required & ~_groups_subscribed;
	uint32_t unsub_groups = _groups_subscribed & ~_groups_required;
	_poll_fds_num = 0;

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (sub_groups & (1 << i)) {
			DEVICE_DEBUG("subscribe to actuator_controls_%d", i);
			_control_subs[i] = orb_subscribe(_control_topics[i]);
		}

		if (unsub_groups & (1 << i)) {
			DEVICE_DEBUG("unsubscribe from actuator_controls_%d", i);
			orb_unsubscribe(_control_subs[i]);
			_control_subs[i] = -1;
		}

		if (_control_subs[i] > 0) {
			_poll_fds[_poll_fds_num].fd = _control_subs[i];
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num++;
		}
	}
}

uint16_t AEROFC_PWMOUT::crc16(uint8_t *p, uint8_t len)
{
	uint16_t crc = 0;

	for (uint8_t *pp = p; pp < p + len; pp++) {
		uint16_t val = *pp;

		crc = crc ^ (val << 8);

		for (unsigned int i = 0; i < 8; i++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;

			} else {
				crc = crc << 1;
			}
		}
	}

	return crc;
}

void AEROFC_PWMOUT::send_esc_outputs(const float *pwm, const unsigned num_pwm)
{
#pragma pack(push,1)
	struct {
		uint8_t magic;
		uint16_t ch[AEROFC_PWMOUT_MAX_MOTOR_NUM];
		uint16_t crc;
	} packet;
#pragma pack(pop)

	memset(&packet, 0, sizeof(packet));

	packet.magic = 0xA2;

	for (uint8_t i = 0; i < num_pwm; i++) {
		if (pwm[i] > RPMMAX) {
			packet.ch[i] = RPMMAX;

		} else if (pwm[i] < RPMSTOPPED) {
			packet.ch[i] = RPMSTOPPED;

		} else {
			packet.ch[i] = pwm[i];
		}

		packet.ch[i] = __builtin_bswap16(packet.ch[i]);
	}

	packet.crc = crc16((uint8_t *) &packet, sizeof(packet) - sizeof(packet.crc));
	packet.crc = __builtin_bswap16(packet.crc);

	int ret = ::write(_uart_fd, &packet, sizeof(packet));

	if (ret != sizeof(packet)) {
		PX4_WARN("TX ERROR: ret: %d, errno: %d", ret, errno);
	}
}

void AEROFC_PWMOUT::cycle()
{
	if (!_initialized) {
		_current_update_rate = 0;
		/* advertise the mixed control outputs, insist on the first group output */
		_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &_outputs);
		multirotor_motor_limits_s multirotor_motor_limits = {};
		_to_mixer_status = orb_advertise(ORB_ID(multirotor_motor_limits), &multirotor_motor_limits);
		_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
		_test_motor_sub = orb_subscribe(ORB_ID(test_motor));
		_initialized = true;
	}

	if (_groups_subscribed != _groups_required) {
		subscribe();
		_groups_subscribed = _groups_required;
		_current_update_rate = 0;
	}

	unsigned max_rate = _pwm_default_rate ;

	if (_current_update_rate != max_rate) {
		_current_update_rate = max_rate;
		int update_rate_in_ms = int(1000 / _current_update_rate);

		/* reject faster than 500 Hz updates */
		if (update_rate_in_ms < 2) {
			update_rate_in_ms = 2;
		}

		/* reject slower than 10 Hz updates */
		if (update_rate_in_ms > 100) {
			update_rate_in_ms = 100;
		}

		DEVICE_DEBUG("adjusted actuator update interval to %ums", update_rate_in_ms);

		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] > 0) {
				orb_set_interval(_control_subs[i], update_rate_in_ms);
			}
		}

		// set to current max rate, even if we are actually checking slower/faster
		_current_update_rate = max_rate;
	}



	/* check if anything updated */
	int ret = ::poll(_poll_fds, _poll_fds_num, 5);


	/* this would be bad... */
	if (ret < 0) {
		DEVICE_LOG("poll error %d", errno);

	} else { /* update even in the case of a timeout, to check for test_motor commands */

		/* get controls for required topics */
		unsigned poll_id = 0;

		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] > 0) {
				if (_poll_fds[poll_id].revents & POLLIN) {
					orb_copy(_control_topics[i], _control_subs[i], &_controls[i]);

				}

				poll_id++;
			}
		}

		size_t num_outputs = _channels_count;

		/* can we mix? */
		if (_is_armed && _mixers != nullptr) {

			/* do mixing */
			num_outputs = _mixers->mix(&_outputs.output[0], num_outputs, NULL);
			_outputs.noutputs = num_outputs;
			_outputs.timestamp = hrt_absolute_time();

			/* publish mixer status */
			multirotor_motor_limits_s multirotor_motor_limits = {};
			multirotor_motor_limits.saturation_status = _mixers->get_saturation_status();

			orb_publish(ORB_ID(multirotor_motor_limits), _to_mixer_status, &multirotor_motor_limits);

			/* disable unused ports by setting their output to NaN */
			for (size_t i = num_outputs; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
				_outputs.output[i] = NAN;
			}

			/* iterate actuators */
			for (unsigned i = 0; i < num_outputs; i++) {
				/* last resort: catch NaN, INF and out-of-band errors */
				if (i < _outputs.noutputs &&
				    PX4_ISFINITE(_outputs.output[i]) &&
				    _outputs.output[i] >= -1.0f &&
				    _outputs.output[i] <= 1.0f) {
					/* scale for PWM output 1250 - 1950us */
					_outputs.output[i] = 1600 + (350 * _outputs.output[i]);

				} else {
					/*
					 * Value is NaN, INF or out of band - set to the minimum value.
					 * This will be clearly visible on the servo status and will limit the risk of accidentally
					 * spinning motors. It would be deadly in flight.
					 */
					_outputs.output[i] = RPMSTOPPED;
				}
			}

		} else {

			_outputs.noutputs = num_outputs;
			_outputs.timestamp = hrt_absolute_time();

			/* check for motor test commands */
			bool test_motor_updated;
			orb_check(_test_motor_sub, &test_motor_updated);

			if (test_motor_updated) {
				struct test_motor_s test_motor;
				orb_copy(ORB_ID(test_motor), _test_motor_sub, &test_motor);
				_outputs.output[test_motor.motor_number] = RPMSTOPPED + ((RPMMAX - RPMSTOPPED) * test_motor.value);
				PX4_INFO("setting motor %i to %.1lf", test_motor.motor_number,
					 (double)_outputs.output[test_motor.motor_number]);
			}

			/* set the invalid values to the minimum */
			for (unsigned i = 0; i < num_outputs; i++) {
				if (!PX4_ISFINITE(_outputs.output[i])) {
					_outputs.output[i] = RPMSTOPPED;
				}
			}

			/* disable unused ports by setting their output to NaN */
			for (size_t i = num_outputs; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
				_outputs.output[i] = NAN;
			}

		}

		const unsigned esc_count = num_outputs;
		float motor_out[AEROFC_PWMOUT_MAX_MOTOR_NUM];

		// Use the system defaults
		for (int i = 0; i < esc_count; ++i) {
			motor_out[i] = _outputs.output[i];
		}

		send_esc_outputs(motor_out, esc_count);

		/* and publish for anyone that cares to see */
		orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &_outputs);

	}

	bool updated;

	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

		if (_is_armed != _armed.armed) {
			/* reset all outputs */
			for (size_t i = 0; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
				_outputs.output[i] = NAN;
			}
		}

		/* do not obey the lockdown value, as lockdown is for PWMSim */
		_is_armed = _armed.armed;

	}


}

void AEROFC_PWMOUT::work_stop()
{
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_control_subs[i] > 0) {
			orb_unsubscribe(_control_subs[i]);
			_control_subs[i] = -1;
		}
	}

	orb_unsubscribe(_armed_sub);
	_armed_sub = -1;
	orb_unsubscribe(_test_motor_sub);
	_test_motor_sub = -1;

	DEVICE_LOG("stopping");
	_initialized = false;
}

int
AEROFC_PWMOUT::control_callback(uintptr_t handle,
				uint8_t control_group,
				uint8_t control_index,
				float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];

	/* limit control input */
	if (input > 1.0f) {
		input = 1.0f;

	} else if (input < -1.0f) {
		input = -1.0f;
	}

	/* throttle not arming - mark throttle input as invalid */
	if (_armed.prearmed && !_armed.armed) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* set the throttle to an invalid value */
			input = NAN;
		}
	}

	return 0;
}

int
AEROFC_PWMOUT::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	switch (cmd) {

	case MIXERIOCRESET:
		if (_mixers != nullptr) {
			delete _mixers;
			_mixers = nullptr;
			_groups_required = 0;
		}

		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strnlen(buf, 1024);

			if (_mixers == nullptr) {
				_mixers = new MixerGroup(control_callback, (uintptr_t)_controls);
			}

			if (_mixers == nullptr) {
				_groups_required = 0;
				ret = -ENOMEM;

			} else {

				ret = _mixers->load_from_buf(buf, buflen);

				if (ret != 0) {
					DEVICE_DEBUG("mixer load failed with %d", ret);
					delete _mixers;
					_mixers = nullptr;
					_groups_required = 0;
					ret = -EINVAL;

				} else {

					_mixers->groups_required(_groups_required);
				}
			}

			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}



	return ret;
}

namespace aerofc_pwmout_drv
{


volatile bool _task_should_exit = false; // flag indicating if aerofc_pwmout task should exit
static char _device[32] = {};
static bool _is_running = false;         // flag indicating if aerofc_pwmout app is running
static px4_task_t _task_handle = -1;     // handle to the task main thread
static int _supported_channel_count = 0;

static bool _flow_control_enabled = false;

void usage();

void start();
void stop();
int aerofc_pwmout_start(void);
int aerofc_pwmout_stop(void);

void task_main_trampoline(int argc, char *argv[]);

void task_main(int argc, char *argv[]);

int initialise_uart();

int deinitialize_uart();

int enable_flow_control(bool enabled);

int aerofc_pwmout_start(void)
{
	int ret = OK;

	if (aerofc_pwmout == nullptr) {

		aerofc_pwmout = new AEROFC_PWMOUT(_supported_channel_count);

		if (aerofc_pwmout == nullptr) {
			ret = -ENOMEM;

		} else {
			ret = aerofc_pwmout->init();

			if (ret != OK) {
				PX4_ERR("failed to initialize aerofc_pwmout (%i)", ret);
				delete aerofc_pwmout;
				aerofc_pwmout = nullptr;
			}
		}
	}

	return ret;
}

int aerofc_pwmout_stop(void)
{
	int ret = OK;

	if (aerofc_pwmout != nullptr) {

		delete aerofc_pwmout;
		aerofc_pwmout = nullptr;
	}

	return ret;
}

int initialise_uart()
{
	// open uart
	_uart_fd = open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	int termios_state = -1;

	if (_uart_fd < 0) {
		PX4_ERR("failed to open uart device!");
		return -1;
	}

	// set baud rate
	int speed = B115200;
	struct termios uart_config;
	tcgetattr(_uart_fd, &uart_config);

	// clear ONLCR flag (which appends a CR for every LF)
	uart_config.c_oflag &= ~ONLCR;

	// set baud rate
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("failed to set baudrate for %s: %d\n", _device, termios_state);
		close(_uart_fd);
		return -1;
	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("tcsetattr failed for %s\n", _device);
		close(_uart_fd);
		return -1;
	}

	// setup output flow control
	if (enable_flow_control(false)) {
		PX4_WARN("hardware flow disable failed");
	}

	return _uart_fd;
}

int enable_flow_control(bool enabled)
{
	struct termios uart_config;

	int ret = tcgetattr(_uart_fd, &uart_config);

	if (enabled) {
		uart_config.c_cflag |= CRTSCTS;

	} else {
		uart_config.c_cflag &= ~CRTSCTS;
	}

	ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);

	if (!ret) {
		_flow_control_enabled = enabled;
	}

	return ret;
}

int deinitialize_uart()
{
	return close(_uart_fd);
}

void task_main(int argc, char *argv[])
{

	_is_running = true;

	if (initialise_uart() < 0) {
		PX4_ERR("Failed to initialize UART.");

		while (_task_should_exit == false) {
			usleep(100000);
		}

		_is_running = false;
		return;
	}

	if (aerofc_pwmout_start() != OK) {
		PX4_ERR("failed to start aerofc_pwmout.");
		_is_running = false;
		return;
	}


	// Main loop
	while (!_task_should_exit) {

		aerofc_pwmout->cycle();

	}


	_is_running = false;
}

void task_main_trampoline(int argc, char *argv[])
{
	task_main(argc, argv);
}

void start()
{
	ASSERT(_task_handle == -1);

	_task_should_exit = false;

	/* start the task */
	_task_handle = px4_task_spawn_cmd("aerofc_pwmout_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX,
					  1000,
					  (px4_main_t)&task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		PX4_ERR("task start failed");
		_task_handle = -1;
		return;
	}
}

void stop()
{
	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO("aerofc_pwmout_stop");
	}

	aerofc_pwmout_stop();
	deinitialize_uart();
	_task_handle = -1;
}

void usage()
{
	PX4_INFO("usage: aerofc_pwmout start -d /dev/ttyS2 -n <1-8>");
	PX4_INFO("       aerofc_pwmout stop");
	PX4_INFO("       aerofc_pwmout status");
}

} // namespace aerofc_pwmout

// driver 'main' command
extern "C" __EXPORT int aerofc_pwmout_main(int argc, char *argv[]);

int aerofc_pwmout_main(int argc, char *argv[])
{
	const char *device = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	char *verb = nullptr;

	if (argc >= 2) {
		verb = argv[1];
	}

	while ((ch = px4_getopt(argc, argv, "d:n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			strncpy(aerofc_pwmout_drv::_device, device, strlen(device));
			break;

		case 'n':
			aerofc_pwmout_drv::_supported_channel_count = atoi(myoptarg);
			break;
		}
	}

	if (!aerofc_pwmout && aerofc_pwmout_drv::_task_handle != -1) {
		aerofc_pwmout_drv::_task_handle = -1;
	}

	// Start/load the driver.
	if (!strcmp(verb, "start")) {
		if (aerofc_pwmout_drv::_is_running) {
			PX4_WARN("aerofc_pwmout already running");
			return 1;
		}

		// Check on required arguments
		if (aerofc_pwmout_drv::_supported_channel_count == 0 || device == nullptr || strlen(device) == 0) {
			aerofc_pwmout_drv::usage();
			return 1;
		}

		aerofc_pwmout_drv::start();
	}

	else if (!strcmp(verb, "stop")) {
		if (!aerofc_pwmout_drv::_is_running) {
			PX4_WARN("aerofc_pwmout is not running");
			return 1;
		}

		aerofc_pwmout_drv::stop();
	}

	else if (!strcmp(verb, "status")) {
		PX4_WARN("aerofc_pwmout is %s", aerofc_pwmout_drv::_is_running ? "running" : "not running");
		return 0;

	} else {
		aerofc_pwmout_drv::usage();
		return 1;
	}

	return 0;
}
