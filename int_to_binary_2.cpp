#include <cstdio>
#include <type_traits>
#include <climits>

/*
	Task: represent int in binary form
*/
template <class T>
typename std::enable_if<std::is_signed<T>::value, void>::type
print_bits(T num) {
	for (int i = sizeof(T)*CHAR_BIT - 1; i >= 0; --i) {
		printf ("%d", (num >> i) & 1);
	}
}