#include <cstdio>

/*
	Task: represent int in binary form
*/

/*
	I'm not sure about leading zeroes (should I trim them?)
	Probably, another solution will be something like

	while (num) {
		printf("%d", num % 2);
		num = num / 2;
	}
*/
void print_bit_form (int num) {
	for (int i = sizeof(int)*8 - 1; i >= 0; --i) {
		printf ("%d", (num >> i) & 1);
	}
}