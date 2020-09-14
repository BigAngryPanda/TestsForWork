#include <iostream>
#include <cstring>
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <signal.h>

#include <unistd.h>
#include <cstdlib>

/*
	In the worst case we have 64 symbol string full of 'KB'
*/
const size_t str_buff_size = 128;

const char socket_net_addr[] = "127.0.0.1";
const size_t net_port = 8080;

class Buffer
{
	bool read_flag;

	std::mutex buffer_mutex;

	std::condition_variable cv;

	char* data;
	size_t buffer_size;
	size_t buffer_capacity;

	void change_state()
	{
		read_flag = !read_flag;
	}

public:
	Buffer(size_t total_size)
	{
		read_flag = false;
		buffer_capacity = total_size;
		data = (char*) mmap(NULL, total_size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);

		if (data == MAP_FAILED)
		{
			throw std::runtime_error("Error: failed to allocate shared memory");
		}

		memset(data, 0, buffer_capacity);
		buffer_size = 0;
	}

	~Buffer()
	{
		munmap(data, buffer_capacity);
	};
	
	void write(std::string const& input)
	{
		std::unique_lock<std::mutex> lck(buffer_mutex);
		cv.wait(lck, [this]{ return !read_flag; });

		memcpy(data, input.data(), input.size());
		buffer_size = input.size();

		change_state();
		cv.notify_all();
	}

	std::string read()
	{
		std::unique_lock<std::mutex> lck(buffer_mutex);
		cv.wait(lck, [this]{ return read_flag; });

		std::string result(data, buffer_size);
		memset(data, 0, buffer_size);
		buffer_size = 0;

		change_state();
		cv.notify_all();

		return result;
	}
};

class Socket
{
	int socket_handler;

public:
	Socket(const char* socket_addr, size_t port)
	{
		socket_handler = socket(AF_INET, SOCK_STREAM, 0);

		if (socket_handler == -1)
		{
			throw std::runtime_error("Error: failed to create socket");
		}

		sockaddr_in client_addr;
		client_addr.sin_family = AF_INET;
		client_addr.sin_port = htons(port); 

		if (inet_pton(AF_INET, socket_addr, &client_addr.sin_addr) == -1)
		{
			throw std::runtime_error("Error: failed to convert address");
		}

		if (connect(socket_handler, (sockaddr*) &client_addr, sizeof(client_addr)) == -1)
		{
			std::cout << "Cannot connect to the Programm 2. Trying again..." << std::endl;

			while (connect(socket_handler, (sockaddr*) &client_addr, sizeof(client_addr)) == -1) {}

			std::cout << "Connected!" << std::endl;
		}
	}

	~Socket()
	{
		close(socket_handler);
	}
	
	void write(size_t msg)
	{
		if (send(socket_handler, &msg, sizeof(size_t), 0) == -1)
		{
			throw std::runtime_error("Error: cannot write to the socket");
		}
	}
};

bool validate(std::string const& str)
{
	if (str.size() > 64)
	{
		return false;
	}

	for (char symbol: str)
	{
		if (symbol < '0' || symbol > '9')
		{
			return false;
		}
	}

	return true;
}

std::vector<int> count_char(std::string const& str)
{
	std::vector<int> result(10, 0);

	for (char symbol: str)
	{
		++result[symbol - '0'];
	}

	return result;
}

std::string format_str(std::string const& str)
{
	std::string result;
	result.reserve(str_buff_size);

	std::vector<int> char_nums = count_char(str);

	for (int i = char_nums.size() - 2; i >= 0; i -= 2)
	{
		result.append(char_nums[i+1], (char) (i+1) + '0');

		for (int j = 0; j < char_nums[i]; ++j)
		{
			result.append("KB");
		}
	}

	return result;
}

void run_thread_1(Buffer& buff)
{
	while (true)
	{
		std::string input;
		std::cin >> input;

		if (!validate(input))
		{
			std::cerr << "Wrong input format. Discarded." << std::endl;
			continue;
		}

		buff.write(format_str(input));
	}
}

size_t summ_of_nums(std::string const& str)
{
	size_t result = 0;

	for (char symbol: str)
	{
		switch (symbol)
		{
			case 'K':
			case 'B':
				break;
			default:
				result += symbol - '0';
		}
	}

	return result;
}

void run_thread_2(Buffer& buff, Socket& sock)
{
	while (true)
	{
		std::string input = buff.read();

		size_t summ = summ_of_nums(input);

		std::cout << input << std::endl;

		try
		{
			sock.write(summ);
		}
		catch(std::runtime_error e)
		{
			std::cerr << e.what() << std::endl;
		}
	}
}

void sigpipe_handler(int t)
{
	std::cerr << "Fatal error: cannot write to socket. Terminated" << std::endl;
	std::exit(1);
}

int main(int argc, char const *argv[])
{
	signal(SIGPIPE, sigpipe_handler);

	try
	{
		Buffer buff(str_buff_size);
		Socket sock(socket_net_addr, net_port);

		std::thread t1 = std::thread(run_thread_1, std::ref(buff));
		std::thread t2 = std::thread(run_thread_2, std::ref(buff), std::ref(sock));

		t1.join();
		t2.join();
	}
	catch(std::runtime_error e)
	{
		std::cerr << e.what() << std::endl;

		return 1;
	}

	return 0;
}