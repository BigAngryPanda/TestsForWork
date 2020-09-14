#include <iostream>

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <unistd.h>

const char socket_net_addr[] = "127.0.0.1";
const size_t net_port = 8080;

class Socket
{
	int socket_handler;
	int opt = 1;

public:
	Socket(const char* socket_addr, size_t port)
	{
		int temp_socket_handler = socket(AF_INET, SOCK_STREAM, 0);

		if (temp_socket_handler == -1)
		{
			throw std::runtime_error("Error: failed to create socket");
		}

		if (setsockopt(temp_socket_handler, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
		{
			throw std::runtime_error("Error: failed to create socket port");
		}

		sockaddr_in serv_addr;
		serv_addr.sin_family = AF_INET;
		serv_addr.sin_addr.s_addr = INADDR_ANY;
		serv_addr.sin_port = htons(port);

		size_t addrlen = sizeof(serv_addr);

		if (inet_pton(AF_INET, socket_net_addr, &serv_addr.sin_addr) == -1)
		{
			throw std::runtime_error("Error: failed to convert address");
		}

		if (bind(temp_socket_handler, (struct sockaddr *) &serv_addr, addrlen) == -1) 
	    { 
	        throw std::runtime_error("Error: failed to bind socket");
	    }

	    if (listen(temp_socket_handler, 1) == -1)
	    {
	    	throw std::runtime_error("Error: failed to listen incoming connection");
	    }

	    socket_handler = accept(temp_socket_handler, (struct sockaddr *) &serv_addr,  (socklen_t*) &addrlen);

	    if (socket_handler == -1)
	    {
	    	throw std::runtime_error("Error: failed to establish connection");
	    }
	}

	~Socket()
	{
		close(socket_handler);
	}
	
	int read(size_t* output)
	{
		int bytes_read = recv(socket_handler, output, sizeof(size_t), 0);

		if (bytes_read == 0)
		{
			throw std::runtime_error("Receive 0 bytes, probably client was terminated");
		}

		if (bytes_read < 0)
		{
			throw std::runtime_error("Error: cannot read the socket");
		}		

		return bytes_read;
	}
};

bool is_valid(size_t input)
{
	if ((input > 9) && (input % 32 == 0))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void run(Socket& s)
{
	size_t input;

	while (true)
	{
		try
		{
			s.read(&input);
		}
		catch (std::runtime_error e)
		{
			std::cout << e.what() << std::endl;
			return;
		}

		if (is_valid(input))
		{
			std::cout << "Correct data " << input << std::endl;
		}
		else
		{
			std::cout << "Invalid data" << std::endl;
		}
	}
}

int main(int argc, char const *argv[])
{
	try
	{
		Socket s(socket_net_addr, net_port);
		run(s);
	}
	catch (std::runtime_error e)
	{
		std::cout << e.what();
		return 1;
	}

	return 0;
}