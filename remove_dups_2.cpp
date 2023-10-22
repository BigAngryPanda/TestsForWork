#include <cstddef>
#include <cstdio>

#ifdef DEBUG
#include <cassert>
#include <cstring>
#include <cstdlib>
#endif

/*
    Task: remove pairs of symbols in string (in-place)

    See test cases
*/
void RemoveDups(char* str) {
    if (str == nullptr) {
        return;
    }

    int i = 0;
	int offset = 1;

	while (str[i+offset] != '\0') {
		if (i == -1) {
			str[0] = str[i+offset];
			str[i+offset] = '\0';
			++i;
		}
		else if (str[i] == str[i+offset]) {
			str[i] = '\0';
			str[i+offset] = '\0';
			--i;
			offset += 2;
		}
		else {
			char symbol = str[i+offset];
			str[i+offset] = '\0';
			str[i+1] = symbol;
			++i;
		}
	}
}

/*
    Test cases:

    "AAA BBB AAA" -> "A B A"
    "abcdqwertt" -> "abcdqwer"
    "" -> ""
    "AAAAAAA" -> "A"
    "A A A B" -> "A A A B"
*/

#ifdef DEBUG

void Test(const char* test_case, const char* answer) {
	size_t datalen = std::strlen(test_case);

	char* data = (char*)malloc(datalen+1);

	memcpy(data, test_case, datalen+1);

	printf("\"%s\" -> ", data);
	RemoveDups(data);
	printf("\"%s\"\n", data);

	assert(strcmp(data, answer) == 0);

	free(data);
}

int main() {
	Test("ABBBC", "ABC");
	Test("ABBAC", "C");
	Test("AAA BBB AAA", "A B A");
	Test("abcdqwertt", "abcdqwer");
	Test("", "");
	Test("AAAAAAA", "A");
	Test("A A A B", "A A A B");
	Test("A A A BB", "A A A ");
	Test("aA Ty", "aA Ty");
	Test("abcdedcba", "abcdedcba");
	Test("abcdeedcba", "");
}
#endif