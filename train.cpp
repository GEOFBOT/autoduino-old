// Not done yet
#include <iostream>
#include <string>
#include <fann.h>

using namespace std;

int main(int argc, char* argv[]) {
  if(argc != 3) {
    cout << "Improper use.  train [data-file] [net-file]" << endl;
    return -1;
  }
  
  string data = argv[1];
  string net = argv[2];
  
  struct fann *ann = fann_create_standard(3, 6, 4, 2);
  fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
  fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);
  
  fann_train_on_file(ann, data.c_str(), 500000, 1000, 0.001);
  fann_save(ann, net.c_str());
  
  fann_destroy(ann);
  return 0;
}
