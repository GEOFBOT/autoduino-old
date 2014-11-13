#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <wiringSerial.h>
#include <floatfann.h>

using namespace std;

int main(int argc, char* argv[]) {
  if(argc != 2) {
    cout << "Improper use.  run [net-file]" << endl;
    return -1;
  }
  
  string file = argv[1];

  int arduino = serialOpen("/dev/ttyACM0", 9600);
  if(!arduino) {
    cout << "Your arduino is not on /dev/ttyACM0 at 9600 baud!" << endl;
    return -2;
  }

  fann_type *output;
  fann_type input[6];
  struct fann *ann = fann_create_from_file(file.c_str());
  
  string line = "";
  string temp = "";
  string command = "";
  istringstream iss;
  vector<int> dist;

  while(true) {
    char c = serialGetchar(arduino);
    if (c != '\n')
      line += c;
    else if(line.size() > 10) {
      iss.str(line);
      cout << line << endl;
      for(int i=0;i<6;i++) {
        getline(iss, temp, ' ');
        dist.push_back(stoi(temp));
        input[i] = dist[i];
      }
      output = fann_run(ann, input);
      if(round(output[1]) == 1) {
        command += "f";
      } else if (round(output[1]) == -1) {
        command += "r";
      }
      if(round(output[2]) == 1) {
        command += "a";
      } else if (round(output[2]) == -1) {
        command += "d";
      }    
      serialPrintf(arduino, command);
      line = "";
      temp = "";
      command = "";
      for(int i=0;i<6;i++) {
        input[i] = 0;
        dist.clear();
        iss.str();
        iss.clear();
      }
      }
}
