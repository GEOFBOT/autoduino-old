#include <iostream>
#include <string>
#include <vector>
#include <wiringSerial.h>

using namespace std;

int main()
{
  int arduino;
  if((arduino = serialOpen("/dev/ttyACM0", 9600)) < 0) {
    cerr << "Cannot connect to /dev/ttyACM0 at 9600 baud!" << endl;
    return -1;
  }

  vector<int> dist;
  string line;

  while(true) {
    char c = (char)serialGetchar(arduino);
    if(c == '\n') {
      // test
    } else if(c != ' ' && c != '\n') 
      line += c;
  }
  return 0;
}
