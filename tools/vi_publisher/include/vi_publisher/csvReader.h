#pragma once
#ifndef CSVREADER_H_
#define CSVREADER_H_

#include <fstream>

using namespace std;

class csvReader
{
public:
    csvReader(string filename);
    ~csvReader();
    bool readline();
    float data[10];
private:
    ifstream _csvInput;
};

#endif  // CSVREADER_H_
