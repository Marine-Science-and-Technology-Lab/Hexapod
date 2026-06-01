//g++ test.cpp -lgalil -lgclib -lgclibo -o test

#include "Galil.h"
#include <iostream>

using namespace std;

int main()
{
	try
	{
		cout << Galil::libraryVersion() << '\n';
		Galil g("10.1.3.169");
		cout << g.connection() << '\n';
		return 0;
	}
	catch(string s)
	{
		cout << s << endl;
		return 1;
	}
}
