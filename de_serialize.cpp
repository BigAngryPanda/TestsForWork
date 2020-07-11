#include <iostream>
#include <string>
#include <map>

/*
	Task: Serialize/Desirialize class List
*/

/*
	Notes
	It is supposed that first ListNode.prev == NULL
	It is supposed that last ListNode.next == NULL
	

	Binary format:
	List length
	<
		ListNode rand as index (-1 if NULL)
		ListNode data.size() + 1
		data.data() (with '\0', next data.size() + 1 bytes)
	>*
	
	Algorithm
	Each node has own index (first node is 0, second node is 1 etc.)
	I store one ListNode after another (first is first, second is second etc.)
	Without 'rand' pointer we would just store 'data' field but with it we need extra work
	For each node we write in file index of 'rand node'
	And then we deserialize all we need is to store address of each node and their 'rand node index'
	And then match them
*/

struct ListNode {
	ListNode * prev;
	ListNode * next;
	ListNode * rand; // point to one of the other node, otherwise NULL
	std::string data;
};

class List {
public:
	void Serialize (FILE * file); // write to file (file was opened with fopen(path, "wb"))
	void Deserialize (FILE * file); // read from file (file was opened fopen(path, "rb"))
private:
	ListNode * head;
	ListNode * tail;
	int count;
};

// source must not be empty
std::map<ListNode *, int> GetAddrTable (ListNode * first) {
	std::map<ListNode*, int> result;

	result[first] = 0;

	ListNode * current_node = first->next;

	int i = 1;

	while (current_node != NULL) {
		result[current_node] = i;

		++i;

		current_node = current_node->next;
	}

	return result;
}

void SerializeData (FILE * file, ListNode * node) {
	fprintf (file, "%ld\n", node->data.size() + 1);
	fwrite (node->data.data(), sizeof(char), node->data.size() + 1, file);
	fprintf(file, "\n");
}

void List::Serialize (FILE * file) {
	if (this->head == NULL) {
		fprintf (file, "%d\n", 0);

		return;
	}

	std::map<ListNode *, int> addr_table = GetAddrTable (this->head);

	fprintf (file, "%d\n", (int) addr_table.size());

	ListNode * current_node = this->head;

	// Probably good place for custom iterator...
	while (current_node != NULL) {
		if (current_node->rand == NULL) {
			fprintf (file, "%d\n", -1);
		}
		else {
			fprintf(file, "%d\n", addr_table[current_node->rand]);
		}

		SerializeData(file, current_node);

		current_node = current_node->next;
	}

}

ListNode* ReadNode (FILE * file, int & index) {
	ListNode* result = new ListNode;

	size_t str_size;

	fscanf (file, "%d\n", &index);   // Read node.rand index
	fscanf (file, "%ld\n", &str_size);    // Read data size

	char* raw_str = new char[str_size];

	fread (raw_str, sizeof(char), str_size, file);

	result->prev = NULL;
	result->next = NULL;
	result->rand = NULL;
	result->data = std::string(raw_str);

	delete raw_str;

	return result;
}

void List::Deserialize (FILE * file) {
	fscanf (file, "%d\n", &this->count);

	if (this->count == 0) {
		this->head = NULL;
		this->tail = NULL;

		return;
	}

	int* indices = new int[this->count];
	ListNode** addrs = new ListNode*[this->count]; 

	ListNode * l = ReadNode (file, indices[0]);
	addrs[0] = l;

	this->head = l;
	this->tail = l;

	for (int i = 1; i < this->count; ++i) {
		ListNode * l = ReadNode (file, indices[i]);
		addrs[i] = l;

		l->prev = this->tail;

		this->tail->next = l;

		this->tail = l;
	}

	for (int i = 0; i < this->count; ++i) {
		if (indices[i] != -1) {
			addrs[i]->rand = addrs[indices[i]];
		}
	}

	delete indices;
	delete addrs;
}