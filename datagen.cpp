#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <wiringSerial.h>
#include <fann.h>

using namespace std;

int main(int argc, char* argv[]) {
  ofstream data;
  if(argc != 2) {
    cout << "Improper use.  datagen [data-file]" << endl;
    return -1;
  }
  
  string file = argv[1];
  data.open(file.c_str());
  if(!data.is_open()) {
    cout << "File " << file << " can't be opened." << endl;
    return 1;
  }

  int arduino = serialOpen("/dev/ttyACM0", 9600);
  if(!arduino) {
    cout << "Your arduino is not on /dev/ttyACM0 at 9600 baud!" << endl;
    return -2;
  }

  string line = "";
  string temp = "";
  string temp2 = "";
  istringstream iss;
  vector<int> dist;
  int comm[6] = {0,0,0,0,0,0};

  vector<string> inputs;
  vector<string> outputs;
  ostringstream oss;


  bool done = false;
  bool again = true;
  while(!done) {
    char c = serialGetchar(arduino);
    if (c != '\n')
      line += c;
    else if(line.size() > 10) {
      iss.str(line);
      cout << line << endl;
      for(int i=0;i<6;i++) {
	getline(iss, temp, ' ');
	dist.push_back(stoi(temp));
	//cout << dist[i] << endl;
	//cout << dist[i] << ' ' << endl;
      }
      again = true;
      while(again) {
	cout << "Input (frsadc/q): ";
	getline(cin, temp2);
	cout << temp2;
	again = false;
	for(int i=0;i<temp2.size();i++) {
	  char c = temp2[i];
	  switch(c) {
	  case 'f':
	    comm[0] = 1;
	    comm[1] = 0;
	    comm[2] = 0;
	    break;
	  case 'r':
	    comm[0] = 0;
	    comm[1] = 1;
	    comm[2] = 0;
	    break;
	  case 's':
	    comm[0] = 0;
	    comm[1] = 0;
	    comm[2] = 1;
	    break;
	  case 'a':
	    comm[3] = 1;
	    comm[4] = 0;
	    comm[5] = 0;
	    break;
	  case 'd':
	    comm[3] = 0;
	    comm[4] = 1;
	    comm[5] = 0;
	    break;
	  case 'c':
	    comm[3] = 0;
	    comm[4] = 0;
	    comm[5] = 1;
	    break;
	  case 'q':
	    done = true;
	    break;
	  default:
	    cout << "Invalid input, try again!" << endl;
	    again = true;
	    break;
	  }
	}
      }
      if(!done) {
	serialPrintf(arduino, temp2.c_str());
	for(int i=0;i<6;i++) {
	  oss << comm[i] << ' ';
	}
	inputs.push_back(line);
	outputs.push_back(oss.str());
	line = "";
	temp = "";
	temp2 = "";
	dist.clear();
	iss.str("");
	oss.str("");
      } else {
      data << inputs.size() << " 6 6" << endl;
      for(int i=0;i<inputs.size();i++) {
	data << inputs[i] << endl;
	data << outputs[i] << endl;
      }
      data.close();
      }
    }
  }
}
