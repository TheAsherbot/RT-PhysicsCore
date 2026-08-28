#include "dummy.h"

int test(int i)
{
	return privateTest(i, 2);
}

int privateTest(int i, int x)
{
	return i * x;
}
