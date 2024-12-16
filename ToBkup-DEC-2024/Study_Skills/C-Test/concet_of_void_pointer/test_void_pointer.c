

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

int main()
{
	void *p = malloc(4);

	((char*)p)[0] = 192;
	((char*)p)[1] = 168;
	((char*)p)[2] = 0;
	((char*)p)[3] = 20;

	printf("%d, %d, %d, %d\n", ((char*)p)[0], ((char*)p)[1], ((char*)p)[2], ((char*)p)[3]);

	return 0;
}
