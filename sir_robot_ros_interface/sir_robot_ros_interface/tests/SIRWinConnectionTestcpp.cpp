#ifdef CONTEST

#include "SIRWinConnection.h"
#include <iostream>
#include <string>

using namespace std;

int main() {
	SIRWinConnection *con = new SIRWinConnection;
	int erno;

	//connect
	if ((erno = con->Connect())) {
		cout << con->getError(erno) << endl;
		return 0;
	}

	//send and receive
	string str = "ADDTS 140.325 15.245 256.215 35.025 -152.025 20.845:";
	while (str != "exit") {
		if ((erno = con->Send(str))) {
			cout << con->getError(erno) << endl;
			return 0;
		}
		erno = 1;
		while (erno != 0) {
			erno = con->Receive(str);
		}
		cout << "Gelen Mesaj: " << str << endl;
	}

	//disconnect
	if ((erno = con->disconnect())) {
		cout << con->getError(erno) << endl;
	}
	return 0;
}

#endif