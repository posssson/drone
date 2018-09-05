#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

class SimpleSerial {
public:
	SimpleSerial(std::string port, unsigned int baud_rate, size_t timeout);

	void writeString(std::string s);
	std::string readLine();
	void read_complete(const boost::system::error_code& error,
			size_t bytes_transferred);
	void time_out(const boost::system::error_code& error);
	bool read_char(char& val);

	boost::asio::serial_port& getPort();

private:
	boost::asio::io_service io;
	boost::asio::serial_port serial;
	size_t timeout;
	boost::asio::deadline_timer timer;
	bool read_error;
	char c;
};

