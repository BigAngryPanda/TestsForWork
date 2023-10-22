#include <cstddef>
#include <cstdio>
#include <iostream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <map>

/*
	Task: Serialize/Desirialize class List
*/

struct ListNode {
	ListNode* prev;
	ListNode* next;
	ListNode* rand; // point to one of the other node, otherwise NULL
	std::string data;
};

class List {
public:
	void Serialize (FILE* file);   // write to file (file was opened with fopen(path, "wb"))
	void Deserialize (FILE* file); // read from file (file was opened fopen(path, "rb"))
	void Clear();

#ifdef DEBUG
	void Set (ListNode* h, ListNode* t, int c) {
		head = h;
		tail = t;
		count = c;
	}

	void Debug () {
		std::cout << "Head " << head << std::endl;
		std::cout << "Tail " << tail << std::endl;
		std::cout << "Count " << count << std::endl;

		ListNode* curr = head;

		for (int i = 0; i < count; ++i) {
			std::cout << "Node " << i << std::endl;
			std::cout << "addr: " << curr << std::endl;
			std::cout << "prev: " << curr->prev << std::endl;
			std::cout << "next: " << curr->next << std::endl;
			std::cout << "rand: " << curr->rand << std::endl;
			std::cout << "data: " << curr->data << std::endl;

			curr = curr->next;
		}
	}
#endif

private:
	ListNode* head{NULL};
	ListNode* tail{NULL};
	int count{0};
};

/*
    Format: |--node addr--|--rand addr--|--data length--|--data--|
*/
void List::Serialize (FILE* file) {
	fwrite(&count, sizeof(int), 1, file);

	ListNode* curr = head;

	while (curr != NULL) {
		fwrite(&curr, sizeof(ListNode*), 1, file);
		fwrite(&curr->rand, sizeof(ListNode*), 1, file);

		size_t len = curr->data.size();
		fwrite(&len, sizeof(size_t), 1, file);

		fwrite(curr->data.c_str(), len, 1, file);

		curr = curr->next;
	}
}

void List::Deserialize (FILE* file) {
	if (fread(&count, sizeof(int), 1, file) != 1) {
		throw std::runtime_error("Failed to read num of elements");
	}

	if (count == 0) {
		return;
	}

	std::map<ListNode*, ListNode*> addr_map;

	ListNode dummy;

	ListNode* prev = &dummy;

	for (int i = 0; i < count; ++i) {
		ListNode* curr = new ListNode;

		prev->next = curr;

		curr->prev = prev;
		curr->next = NULL;

		tail = curr;

		ListNode* old_addr;
		if (fread(&old_addr, sizeof(ListNode*), 1, file) != 1) {
			head = dummy.next;
			Clear();
			throw std::runtime_error("Failed to read node addr");
		}

		addr_map[old_addr] = curr;

		if (fread(&curr->rand, sizeof(ListNode*), 1, file) != 1) {
			head = dummy.next;
			Clear();
			throw std::runtime_error("Failed to read node rand addr");
		}

		size_t size;
		if (fread(&size, sizeof(size_t), 1, file) != 1) {
			head = dummy.next;
			Clear();
			throw std::runtime_error("Failed to read data length");
		}

		curr->data.resize(size);

		if (fread(&curr->data[0], 1, size, file) != size) {
			head = dummy.next;
			Clear();
			throw std::runtime_error("Failed to read data");
		}

		prev = curr;
	}

	head = dummy.next;
	head->prev = NULL;

	ListNode* curr = head;

	for (int i = 0; i < count; ++i) {
		if (curr->rand != NULL) {
			curr->rand = addr_map[curr->rand];
		}

		curr = curr->next;
	}
}

void List::Clear() {
	count = 0;

	ListNode* curr = head;

	while (curr != NULL) {
		ListNode* next = curr->next;
		delete curr;

		curr = next;
	}

	tail = NULL;
}

#ifdef DEBUG
int main() {
	std::cout << "Original list" << std::endl;

	ListNode nodes[3];

	nodes[0].prev = NULL;
	nodes[0].next = &nodes[1];
	nodes[0].rand = &nodes[2];
	nodes[0].data = "node 0";

	nodes[1].prev = &nodes[0];
	nodes[1].next = &nodes[2];
	nodes[1].rand = NULL;
	nodes[1].data = "node 1";

	nodes[2].prev = &nodes[1];
	nodes[2].next = NULL;
	nodes[2].rand = &nodes[1];

	List orig;
	orig.Set(&nodes[0], &nodes[2], 3);

	orig.Debug();

	FILE* out = fopen("test.temp", "wb");

	orig.Serialize(out);

	std::fclose(out);

	std::cout << "New list" << std::endl;

	List l;

	FILE* in = std::fopen("test.temp", "rb");

	l.Deserialize(in);

	std::fclose(in);

	l.Debug();

	std::cout << "Empty list" << std::endl;

	List orig_empty;

	orig_empty.Debug();

	out = fopen("empty", "wb");

	orig_empty.Serialize(out);

	std::fclose(out);

	std::cout << "Restored empty list" << std::endl;
	List new_empty;

	in = std::fopen("empty.temp", "rb");

	new_empty.Deserialize(in);

	std::fclose(in);

	new_empty.Debug();
}
#endif