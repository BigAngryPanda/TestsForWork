#include <cstddef>
#include <cstdio>

/*
    Task: remove pairs of symbols in string (in-place)

    See test cases
*/
void RemoveDups(char* str) {
    if (str == nullptr) {
        return;
    }

    std::size_t i = 0;
    std::size_t tail = 0; // last position to write

    while (str[i] != '\0' && str[i+1] != '\0') {
        if (str[i] == str[i+1]) {
            str[i] = '\0';
            str[i+1] = '\0';

            i += 2;
        }
        else {
            str[tail] = str[i];
            ++tail;
            ++i;
        }
    }

    str[tail] = str[i];
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
int main() {
    char data[] = "A A A B";
    RemoveDups(data);
    printf("%s\n", data);
}
#endif