/*
	parse CSV
	calculate cells if it is needed
	each cell is non-negative number or exp like =%COLUMN ROW% + - * / %COLUMN ROW%
	
	example
	,A,B,Cell
	1,1,0,1
	2,2,=A1+Cell30,0
	30,0,=B1+A1,5
*/
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdexcept>
#include <stack>

std::vector<std::string> split_by(const std::string& source, const std::string& splitters)
{
	std::vector<std::string> result;

	std::string temp;

	for (char symbol : source)
	{
		// symbol is not delimiter
		if (splitters.find(symbol) == std::string::npos)
		{
			temp += symbol;
		}
		else if (!temp.empty())
		{
			result.push_back(temp);
			temp.clear();
		}
	}

	if (!temp.empty() || result.empty())
	{
		result.push_back(temp);
	}

	return result;
}

class CSVHeader
{
	size_t total_size;
	std::unordered_map<std::string, size_t> columns;

public:
	CSVHeader(std::string& input)
	{
		std::stringstream str_stream(input);

		std::string raw_column;

		size_t counter = 0;

		while (std::getline(str_stream, raw_column, ','))
		{
			// space and tab
			std::vector<std::string> temp = split_by(raw_column, std::string(" \t"));

			if (temp.size() != 1)
			{
				throw std::runtime_error("Wrong column format in header: " + raw_column);
			}

			if (columns.find(temp[0]) != columns.end() && !temp[0].empty())
			{
				throw std::runtime_error("Duplicating column names in header: " + raw_column);
			}

			if (!temp[0].empty())
			{
				columns.insert({temp[0], counter});
			}

			++counter;
		}

		total_size = counter;
	}

	size_t get_index(const std::string& column_name) const
	{
		return columns.at(column_name);
	}

	size_t get_size() const
	{
		return total_size;
	}

	~CSVHeader() = default;
};

struct Address
{
	size_t row;
	size_t column;

	Address() = default;

	Address(size_t row, size_t line) : row(row), column(line) {}

	~Address() = default;

	bool operator==(const Address& addr) const
	{
		return row == addr.row && column == addr.column;
	}

	// For debug
	std::string to_string() const
	{
		return std::string("{\n\tRow (virtual): " + std::to_string(row) 
				+ "\n\tColumn (real): " 
				+ std::to_string(column) + "\n}\n");
	}
};

struct AddressHasher
{
	size_t operator()(const Address& addr) const
    { 
        return addr.row + addr.column; 
    }
};

enum class OperationType
{
	Add,
	Sub,
	Mul,
	Div,
};

void skip_untill(const std::string& op_token, std::string::const_iterator& it)
{
	while ((*it == ' ' || *it == '\t') && it != op_token.end())
	{
		++it;
	}
}

std::string read_untill(const std::string& op_token, std::string::const_iterator& it, const std::string& limiters)
{
	std::string result;

	for (; it != op_token.end(); ++it)
	{
		// symbol is not delimiter
		if (limiters.find(*it) == std::string::npos)
		{
			result += *it;
		}
		else
		{
			return result;
		}
	}

	return result;
}

struct Operation
{
	Address l_op;
	Address r_op;
	OperationType op_t;

	Operation(const Address& left_arg, const Address& right_arg, OperationType op) 
		: l_op(left_arg), r_op(right_arg), op_t(op) {}

	~Operation() = default;

	// For debug
	std::string to_string() const
	{
		return std::string("Operation\n") + std::string("Left arg\n") 
					+ l_op.to_string() 
					+ std::string("Right arg\n") 
					+ r_op.to_string() 
					+ std::string("\n");
	}
};

class CSVTable
{
	size_t line_size;
	size_t line_counter;

	std::unordered_map<size_t, size_t> addr_table;               // Real addresses (def by line order)

	std::unordered_map<Address, Operation, AddressHasher> operation_table; // Virtual addresses (def by lines itself)

	std::vector<long int*> lines;

	const CSVHeader& handler;

public:
	CSVTable(const CSVHeader& handler) : handler(handler), line_size(handler.get_size()) { }

	void add(const std::string& line)
	{
		std::string debug_msg = "Line (real): " + std::to_string(line_counter) + ". ";

		if (!line_size)
		{
			return;
		}

		try
		{
			lines.push_back(new long int[line_size]);
		}
		catch(...)
		{
			throw std::runtime_error(debug_msg + "Out of memory error");
		}

		std::stringstream str_stream(line);

		std::string raw_token;

		size_t i = 0, line_num;

		if (std::getline(str_stream, raw_token, ','))
		{
			try
			{
				line_num = parse_int(raw_token);
				lines.back()[i] = line_num;

				debug_msg += " (table): " + std::to_string(line_num) + ". ";

				if (addr_table.find(line_num) != addr_table.end())
				{
					throw std::runtime_error(debug_msg + "Duplicating line number");
				}

				addr_table.insert({line_num, line_counter});

				++i;
			}
			catch(std::runtime_error e)
			{
				throw std::runtime_error(debug_msg + e.what());
			}
		}
		else
		{
			throw std::runtime_error(debug_msg + "Wrong line format: empty or invalid line, found " + raw_token);
		}

		while (std::getline(str_stream, raw_token, ','))
		{
			try
			{
				long int num = parse_int(raw_token);

				lines.back()[i] = num;

				++i;

				continue;
			}
			catch(std::invalid_argument e)
			{
				throw std::runtime_error(debug_msg + e.what());
			}
			catch(...) { /* Just ignore it, maybe it is an operation */ }

			try
			{
				operation_table.insert({Address(line_num, i), parse_operation(raw_token)});
			}
			catch(std::runtime_error e)
			{
				throw std::runtime_error(debug_msg + e.what());
			}

			if (i >= line_size)
			{
				throw std::runtime_error(debug_msg + "Wrong line format: too many fields");
			}

			++i;
		}

		++line_counter;
	}

	~CSVTable()
	{
		for (long int* addr : lines)
		{
			delete[] addr;
		}
	}

	void resolve()
	{
		while (!operation_table.empty())
		{
			perform_operation(operation_table.begin());
			operation_table.erase(operation_table.begin());
		}
	}

	void print(const std::string& header)
	{
		std::cout << header << std::endl;

		for (long int* line : lines)
		{
			std::cout << line[0];

			for (int i = 1; i < line_size; ++i)
			{
				std::cout << "," << line[i];
			}

			std::cout << std::endl;
		}
	}

private:
	long int parse_int(const std::string& token)
	{
		long int num;

		try
		{
			num = std::stoll(token);

			if (num < 0)
			{
				throw std::invalid_argument("Wrong record member format: number must be non-negative, found " + token);
			}
		}
		catch(...) 
		{
			throw std::runtime_error("Wrong record member format: first element must be non-negative number, found " + token);
		}

		return num;
	}

	Operation parse_operation(const std::string& token)
	{
		std::string err_msg = "Wrong record member format: not a number or operation, found " + token;

		std::string::const_iterator it = token.begin();

		skip_untill(token, it);

		if (it == token.end() || *it != '=')
		{
			throw std::runtime_error(err_msg);
		}

		++it;

		skip_untill(token, it);

		if (it == token.end())
		{
			throw std::runtime_error(err_msg);
		}

		std::string arg = read_untill(token, it, " \t+-*\\");

		if (arg.empty() || it == token.end())
		{
			throw std::runtime_error(err_msg);
		}

		Address left_arg;

		try
		{
			left_arg = parse_id(arg);
		}
		catch(std::runtime_error e)
		{
			throw e;
		}

		skip_untill(token, it);

		if (it == token.end())
		{
			throw std::runtime_error(err_msg);
		}

		OperationType op;

		switch (*it)
		{
			case '+':
				op = OperationType::Add;
				break;
			case '-':
				op = OperationType::Sub;
				break;			
			case '*':
				op = OperationType::Mul;
				break;
			case '/':
				op = OperationType::Div;
				break;
			default:
				throw std::runtime_error(err_msg);
		}

		++it;

		if (it == token.end())
		{
			throw std::runtime_error(err_msg);
		}

		skip_untill(token, it);

		if (it == token.end())
		{
			throw std::runtime_error(err_msg);
		}

		arg = read_untill(token, it, " \t");

		if (arg.empty())
		{
			throw std::runtime_error(err_msg);
		}

		Address right_arg;

		try
		{
			right_arg = parse_id(arg);
		}
		catch(std::runtime_error e)
		{
			throw e;
		}

		while (it != token.end())
		{
			if (*it != ' ' || *it != '\t')
			{
				throw std::runtime_error(err_msg);
			}

			++it;
		}

		return Operation(left_arg, right_arg, op);
	}

	Address parse_id(const std::string& token)
	{
		std::string num_buffer;

		std::string::const_reverse_iterator it = token.rbegin();
		std::string::const_iterator rev_it = token.end() - 1;

		while (*it >= '0' && *it <= '9' && it != token.rend())
		{
			num_buffer = *it + num_buffer;

			++it;
			--rev_it;
		}

		if (num_buffer.empty())
		{
			throw std::runtime_error("Wrong record member format: ill-formed reference, found " + token);
		}

		size_t vrt_row_addr = std::stoll(num_buffer);
		size_t real_column_addr;

		try
		{
			real_column_addr = handler.get_index(std::string(token.begin(), rev_it + 1));
		}
		catch(...)
		{
			throw std::runtime_error("Wrong record member format: no such column name in reference, found " + token);
		}

		return Address(vrt_row_addr, real_column_addr);
	}

	long int get_by_addr(const Address& addr)
	{
		size_t real_row_addr = addr_table.at(addr.row);
		return lines[real_row_addr][addr.column];
	}

	long int execute_op(const Address& addr)
	{
		Operation op = operation_table.at(addr); 

		long int left_operand, right_operand;

		try
		{
			left_operand = get_by_addr(op.l_op);
			right_operand = get_by_addr(op.r_op);
		}
		catch(...)
		{
			throw std::runtime_error("Error: no such row in table, found " + std::to_string(addr.row));
		}

		switch (op.op_t)
		{
			case OperationType::Add:
				return left_operand + right_operand;
			case OperationType::Sub:
				return left_operand - right_operand;
			case OperationType::Mul:
				return left_operand*right_operand;
			default:
				return left_operand/right_operand;
		}
	}

	void write_by_addr(const Address& addr, long int value)
	{
		size_t real_row_addr;
		try
		{
			real_row_addr = addr_table.at(addr.row);
		}
		catch(...)
		{
			throw std::runtime_error("Error: no such row in table, found " + std::to_string(addr.row));
		}

		lines[real_row_addr][addr.column] = value;
	}

	bool is_ldepended(const Address& addr)
	{
		return operation_table.find(operation_table.at(addr).l_op) != operation_table.end();
	}

	bool is_rdepended(const Address& addr)
	{
		return operation_table.find(operation_table.at(addr).r_op) != operation_table.end();
	}

	Address get_laddr(const Address& addr)
	{
		return operation_table.at(addr).l_op;
	}

	Address get_raddr(const Address& addr)
	{
		return operation_table.at(addr).r_op;
	}

	bool is_unique(const std::unordered_set<Address, AddressHasher>& unique_addr, const Address& addr)
	{
		return unique_addr.find(addr) == unique_addr.end();
	}

	void perform_operation(std::unordered_map<Address, Operation, AddressHasher>::iterator it)
	{
		std::unordered_set<Address, AddressHasher> unique_addr;

		std::stack<Address> addr_stack;

		addr_stack.push(it->first);
		unique_addr.insert(it->first);

		while (!addr_stack.empty())
		{
			if (is_ldepended(addr_stack.top()))
			{
				addr_stack.push(get_laddr(addr_stack.top()));

				if (is_unique(unique_addr, addr_stack.top()))
				{
					unique_addr.insert(addr_stack.top());
				}
				else
				{
					throw std::runtime_error("Error, cyclic dependency was found");
				}

				continue;
			}

			if (is_rdepended(addr_stack.top()))
			{
				addr_stack.push(get_raddr(addr_stack.top()));

				if (is_unique(unique_addr, addr_stack.top()))
				{
					unique_addr.insert(addr_stack.top());
				}
				else
				{
					throw std::runtime_error("Error, cyclic dependency was found");
				}

				continue;
			}

			unique_addr.erase(addr_stack.top());

			try
			{
				long int result = execute_op(addr_stack.top());
				write_by_addr(addr_stack.top(), result);
			}
			catch(std::runtime_error e)
			{
				throw e;
			}

			operation_table.erase(addr_stack.top());
			addr_stack.pop();
		}
	}
};

int main(int argc, char const* argv[])
{
	if (argc < 2)
	{
		std::cout << "Error, no input files" << std::endl;
		return 1;
	}

	std::ifstream file;
	file.open (argv[1]);

	std::string line, header_line;

	std::getline(file, header_line);

	CSVHeader csv_head(header_line);

	try
	{
		CSVTable table(csv_head);

		while (std::getline(file, line))
		{
			table.add(line);
		}

		table.resolve();

		table.print(header_line);
	}
	catch(std::runtime_error e)
	{
		std::cout << e.what() << std::endl;

		return 2;
	}

	return 0;
}