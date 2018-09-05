#include "serial.h"

/*
 * Constructor : Initialization of the port device name, the baud rate communication speed,
 * throws boost::system::system_error if it cannot open the serial device.
 * */
SimpleSerial::SimpleSerial(std::string port, unsigned int baud_rate,
		size_t timeout) :
		io(), serial(io, port), timeout(timeout), timer(
				serial.get_io_service()), read_error(true) {
	serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
}

/**
 * This function write a string (given in parameter) to the serial device. It throws
 * boost::system::system_error on failure.
 */
void SimpleSerial::writeString(std::string s) {
	boost::asio::write(serial, boost::asio::buffer(s.c_str(), s.size()));
}
/**
 * This function is blocked until a line is received from the serial device. An eventual '\n' or '\r\n'
 * characters at the end of the string will be removed. It return a string containing the received line.
 * It throws boost::system::system_error on failure.
 * Blocks
 */
std::string SimpleSerial::readLine() {
	using namespace boost;
	bool stringComplete = false;
	char inChar = '\n';
	std::string inputString;
	boost::system::error_code ec;

	while (!stringComplete) {
		// get the new byte
		asio::read(serial, boost::asio::buffer(&inChar, 1), ec);

		// add it to the inputString
		if (inChar == '\n') {
			stringComplete = true;
		} else {
			inputString += inChar;
		}
	}

	return (inputString);
}

/**
 * This function is called when an "async_read" completes or has been canceled.
 * */
void SimpleSerial::read_complete(const boost::system::error_code& error,
		size_t bytes_transferred) {
	read_error = (error || bytes_transferred == 0);

	// Read has finished, so cancel the timer
	timer.cancel();
}

/**
 * This function is called when the timer's deadline expires.
 * */
void SimpleSerial::time_out(const boost::system::error_code& error) {
	// Was the timeout was cancelled?
	if (error) {
		return;
	}

	// No, we have timed out, so kill the read operation
	// The read callback will be called with an error
	serial.cancel();
}

/**
 * This function reads a character. If the read times out, then it return false.
 * */
bool SimpleSerial::read_char(char& val) {
	val = c = '\0';

	// After a timeout & cancel it seems we need to do a reset for subsequent reads to work
	serial.get_io_service().reset();

	// Asynchronously read 1 character
	boost::asio::async_read(serial, boost::asio::buffer(&c, 1),
			boost::bind(&SimpleSerial::read_complete, this,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));

	// Setup a deadline time to implement our timeout
	timer.expires_from_now(boost::posix_time::milliseconds(timeout));
	timer.async_wait(
			boost::bind(&SimpleSerial::time_out, this,
					boost::asio::placeholders::error));

	// This will block until a character is read or until the it is cancelled.
	serial.get_io_service().run();

	if (!read_error) {
		val = c;
	}

	return !read_error;
}

/**
 * Set of getters
 * */

boost::asio::serial_port& SimpleSerial::getPort() {
	return (serial);
}
