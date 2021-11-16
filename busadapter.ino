
/* https://www.arduino.cc/reference/en/language/functions/communication/serial/ */
/* https://www.arduino.cc/en/Reference/Wire */
/* https://www.arduino.cc/en/Reference/WireSetClock */
/* https://github.com/cirthix/SoftIIC */

/*

Nano Every pinout
https://store.arduino.cc/arduino-nano-every

         USB CONNECTOR
(led, D13)      (power led)
------------------------
 D13           |           D12  (MISO SC1)
 +3V3          |           D11  (MOSI SC1)
 aref          |           D10
 A0/D14        |           D9
 A1/D15        |           D8
 A2/D16        |           D7
 A3/D17        |           D6
 A4/D18 (SDA)  |           D5
 A5/D19 (SCL)  |           D4
 A6/D20        |           D3
 A7/D21        |           D2
 +5V           |           GND
 reset         |           reset
 GND           |           RX
 VIN           |           TX

 */

#include <Wire.h>

#define VERSION_STRING "BUSADAPTER,v=1"

static int debug1 = 0;
static int debug2 = 0;
static int debug3 = 0;
static int debug4 = 0;

enum command_codes {

	CMD_VERSION = 0,
	CMD_INIT = 1,
	CMD_GET_BUS_TYPE = 2,
	CMD_WRITE = 3,
	CMD_READ = 4,
	CMD_TRANSACTION = 5,
	CMD_SET_PIN_MODE = 7,
	CMD_DIGITAL_WRITE = 8,
	CMD_DEBUG1 = 51,
	CMD_DEBUG2 = 52,
	CMD_DEBUG3 = 53

};

enum message_codes {

	RESPONSE_OK = 0,
	RESPONSE_ERROR = 1,
	RESPONSE_CMD_UNAVAILABLE = 2,
	RESPONSE_WRONG_COMMAND = 3,
	RESPONSE_BAD_PARAMETERS = 4,
	RESPONSE_SHORT_WRITE = 5,
	RESPONSE_NACK_ON_ADDRESS = 6,
	RESPONSE_NACK_ON_DATA = 7,
	RESPONSE_DATA_TOO_LONG = 8,
	RESPONSE_OTHER_ERROR = 9

};
typedef byte response_code_t;

enum mode_code_t {
	MODE_UNDEFINED = 0,
	MODE_I2C_MASTER = 1,
	MODE_I2C_SLAVE = 2
};

enum pin_constants_t {
	PIN_MODE_DIGITAL_OUT = 0x01,
	PIN_STATUS_HIGH = 0x10,
	PIN_STATUS_LOW = 0x11,
	PIN_STATUS_TOGGLE = 0x12
};

#define BUFFER_SIZE 256

static mode_code_t work_mode = MODE_UNDEFINED;

typedef struct {

	bool ready;
	byte code;
	byte payload[BUFFER_SIZE];
	int payload_size;
	int bytes_to_read;

} command_t;
static command_t command;

typedef struct {

	bool ready;
	byte data[BUFFER_SIZE];
	int sent;
	int size;

} message_t;

static message_t response;
static message_t to_client_msg;
static message_t from_client_msg;


enum error_status_flags {
	STFLAGS_LOST_RECV_MESSAGE
};
static uint16_t error_flags = 0;

static bool builtin_led_busy = false;

void setup() {

	Serial.begin(115200);

	pinMode(LED_BUILTIN, OUTPUT);
	blinking_pause(500);

	reset_command(&command);
	reset_message(&to_client_msg);
	reset_message(&from_client_msg);

}


void loop() {

	static unsigned long timeout_20hz = 0;

	unsigned long time;

	receive_command(&command);
	serve_common_commands(&command, &response);

	switch (work_mode) {

		case MODE_I2C_MASTER:
			serve_mode_i2c_master(&command, &response);
		break;

		default:
			serve_mode_undefined(&command, &response);
		break;

	}


	if (command.ready) {
		/* nobody picked up this command */
		set_message(&response, RESPONSE_WRONG_COMMAND, NULL, 0);
		reset_command(&command);
	}

	send_message(&to_client_msg);
	send_message(&response);

	time = millis();

	if (time >= timeout_20hz) {
		run_20hz();
		timeout_20hz = time + 50;
	}

}


void run_20hz() {

	if (builtin_led_busy) {
		return;
	}

	static int counter = 0;

	digitalWrite(LED_BUILTIN, (counter == 39) ? HIGH : LOW);
	counter = (counter + 1) % 40;

}


void reset_command(command_t *cmd) {

	cmd->ready = false;
	cmd->payload_size = -1;
	cmd->bytes_to_read = -1;

}


void receive_command(command_t *cmd) {

	if (cmd->ready) {
		return;
	}

	while (Serial.available() > 0) {

		if (cmd->bytes_to_read < 0) {
			cmd->bytes_to_read = Serial.read();
			if (cmd->bytes_to_read == 0) {
				cmd->bytes_to_read = -1;
			}
		} else if (cmd->payload_size < 0) {

			cmd->code = Serial.read();
			cmd->payload_size = 0;
			cmd->bytes_to_read -= 1;
			if (cmd->bytes_to_read == 0) {
				cmd->ready = true;
			}

		} else {

			cmd->payload[cmd->payload_size] = Serial.read();
			cmd->payload_size += 1;

			cmd->bytes_to_read -= 1;
			if (cmd->bytes_to_read == 0) {
				cmd->ready = true;
			}

		}

	}

}


void reset_message(message_t *msg) {

	msg->ready = false;

}


bool set_message(message_t *msg, response_code_t code, void *payload, byte size) {

	if (size > (BUFFER_SIZE - 2)) {
		size = (BUFFER_SIZE - 2);
	}

	msg->data[0] = size + 1;
	msg->data[1] = code;

	if (payload != NULL) {
		memcpy(&msg->data[2], payload, size);
		msg->size = size + 2;
	} else {
		msg->size = 2;
	}
	msg->sent = 0;
	msg->ready = true;

}


void send_message(message_t *msg) {

	if (!msg->ready) {
		return;
	}

	int written = Serial.write(&msg->data[msg->sent], msg->size - msg->sent);
	msg->sent += written;
	if (msg->sent >= msg->size) {
		reset_message(msg);
	}

}


/* ********************************************************************** */


void serve_common_commands(command_t *cmd, message_t *resp) {

	if (!cmd->ready || resp->ready) {
		return;
	}

	bool command_executed = true;
	bool ok;
	int debug_reply[4] = {0, 0, 0, 0};
	byte n, m;

	switch (cmd->code) {

		case CMD_SET_PIN_MODE:

			if (cmd->payload_size == 2) {

				n = cmd->payload[0];   /* pin number */
				m = cmd->payload[1];   /* mode */
				ok = true;

				switch (m) {
					case PIN_MODE_DIGITAL_OUT:
						pinMode(n, OUTPUT);
					break;
					default:
						ok = false;
					break;
				}

				if (ok) {
					if (n == LED_BUILTIN) {
						builtin_led_busy = true;
					}
					set_message(resp, RESPONSE_OK, NULL, 0);
				} else {
					set_message(resp, RESPONSE_BAD_PARAMETERS, NULL, 0);
				}

			} else {
				set_message(resp, RESPONSE_BAD_PARAMETERS, NULL, 0);
			}

		break;

		case CMD_DIGITAL_WRITE:

			if (cmd->payload_size == 2) {

				n = cmd->payload[0];   /* pin number */
				m = cmd->payload[1];   /* status */
				ok = true;

				switch (m) {
					case PIN_STATUS_HIGH:
						digitalWrite(n, HIGH);
					break;
					case PIN_STATUS_LOW:
						digitalWrite(n, LOW);
					break;
					case PIN_STATUS_TOGGLE:
						digitalWrite(n, !digitalRead(n));
					break;
					default:
						ok = false;
					break;
				}

				set_message(resp, ok ? RESPONSE_OK : RESPONSE_BAD_PARAMETERS, NULL, 0);

			} else {
				set_message(resp, RESPONSE_BAD_PARAMETERS, NULL, 0);
			}

		break;

		case CMD_VERSION:
			set_message(resp, RESPONSE_OK, VERSION_STRING, sizeof(VERSION_STRING));
		break;


		case CMD_DEBUG1:
			debug_reply[0] = debug1;
			debug_reply[1] = debug2;
			debug_reply[2] = debug3;
			debug_reply[3] = debug4;
			set_message(resp, RESPONSE_OK, (byte *)&debug_reply, sizeof(debug_reply));
		break;

		case CMD_DEBUG2:
			debug_reply[0] = 0x666;
			debug_reply[1] = 0x666;
			debug_reply[2] = 0x666;
			debug_reply[3] = 0x666;
			set_message(resp, RESPONSE_OK, (byte *)&debug_reply, sizeof(debug_reply));
		break;

		default:
			command_executed = false;
		break;

	}

	if (command_executed) {
		reset_command(cmd);
	}

}


/* ********************************************************************** */


void serve_mode_undefined(command_t *cmd, message_t *resp) {

	if (!cmd->ready || resp->ready) {
		return;
	}

	bool command_executed = true;

	switch (cmd->code) {

		case CMD_INIT:

			set_message(
				resp, cmd_init(cmd->payload, cmd->payload_size),
				NULL, 0
			);

		break;

		default:
			command_executed = false;
		break;

	}

	if (command_executed) {
		reset_command(cmd);
	}

}

void serve_mode_i2c_master(command_t *cmd, message_t *resp) {

	if (!cmd->ready || resp->ready) {
		return;
	}

	bool command_executed = true;
	byte local_buffer[BUFFER_SIZE];
	int i, count;
	response_code_t resp_code;

	switch (cmd->code) {

		case CMD_WRITE:

			if (cmd->payload_size > 0) {

				Wire.beginTransmission(cmd->payload[0]);
				Wire.write(&cmd->payload[1], cmd->payload_size - 1);
				resp_code = end_transmission_to_error_code(Wire.endTransmission());

			} else {
				resp_code = RESPONSE_BAD_PARAMETERS;
			}
			set_message(resp, resp_code, NULL, 0);

		break;

		case CMD_READ:

			if (cmd->payload_size == 2) {

				Wire.requestFrom((int)cmd->payload[0], (int)cmd->payload[1]); /* address, read size */
				i = 0;
				while (Wire.available()) {
					if (i < sizeof(local_buffer)) {
						local_buffer[i++] = Wire.read();
					}
				}
				set_message(resp, RESPONSE_OK, local_buffer, i);

			} else {
				set_message(resp, RESPONSE_BAD_PARAMETERS, NULL, 0);
			}

		case CMD_TRANSACTION:

			if (cmd->payload_size >= 3) {

				Wire.beginTransmission(cmd->payload[0]);
				Wire.write(&cmd->payload[2], cmd->payload_size - 2);
				resp_code = end_transmission_to_error_code(
					Wire.endTransmission(false)  /* send a restart sequence on the i2c bus */
				);

				if (resp_code == RESPONSE_OK) {
					Wire.requestFrom((int)cmd->payload[0], (int)cmd->payload[1]); /* address, read size */
					i = 0;
					while (Wire.available()) {
						if (i < sizeof(local_buffer)) {
							local_buffer[i++] = Wire.read();
						}
					}

					set_message(resp, RESPONSE_OK, local_buffer, i);

				} else {
					Wire.endTransmission();
					set_message(resp, resp_code, NULL, 0);
				}

			} else {
				set_message(resp, RESPONSE_BAD_PARAMETERS, NULL, 0);
			}
		break;

		default:
			command_executed = false;
		break;

	}

	if (command_executed) {
		reset_command(cmd);
	}

}


response_code_t end_transmission_to_error_code(byte err) {

	switch (err) {
		case 0:
			return RESPONSE_OK;
		break;
		case 1:
			return RESPONSE_DATA_TOO_LONG;
		break;
		case 2:
			return RESPONSE_NACK_ON_ADDRESS;
		break;
		case 3:
			return RESPONSE_NACK_ON_DATA;
		break;
		case 4:
		default:
			return RESPONSE_OTHER_ERROR;
		break;
	}

}

/* ********************************************************************** */


response_code_t cmd_init(byte *params, size_t param_size) {

	typedef struct {
		byte mode;
		byte slave_addr;
		uint16_t clock_speed_khz;
	} __attribute__((packed)) init_params_t;

	if (param_size != sizeof(init_params_t)) {
		return RESPONSE_BAD_PARAMETERS;
	}

	init_params_t *init_params = (init_params_t *)params;

	switch (init_params->mode) {

		case MODE_I2C_MASTER:
			setup_mode_i2c_master(init_params->clock_speed_khz);
			work_mode = MODE_I2C_MASTER;
		break;

		default:
			return RESPONSE_BAD_PARAMETERS;
		break;

	}

	return RESPONSE_OK;

}


/* ********************************************************************** */

void setup_mode_i2c_master(unsigned int clock_speed_khz) {

	Wire.begin();
	Wire.setClock(clock_speed_khz * 1000);

}

/* ********************************************************************** */

void blinking_pause(int ms) {

	int i;

	for (i = 0; i < ms; ) {

		digitalWrite(LED_BUILTIN, HIGH);
		delay(30);
		digitalWrite(LED_BUILTIN, LOW);
		delay(30);

		i += 60;

	}

}



