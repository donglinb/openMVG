#include <sstream>

#include "csvReader.h"

csvReader::csvReader(string filename)
{
    _csvInput.open(filename.c_str());
    if(!_csvInput)
        throw std::runtime_error("Can not open csv file "+filename+"\n");
    string header;
    getline(_csvInput,header);
}

csvReader::~csvReader()
{
    if(_csvInput)
        _csvInput.close();
}

bool csvReader::readline()
{
    if(!_csvInput)
        return false;
    if(_csvInput.eof())
        return false;
    
    string line;
    getline(_csvInput,line);
    
    istringstream _readStr(line);
    string part;
    
    for(int i=0;i<10;i++)
    {
        getline(_readStr,part,',');
        data[i] = atof(part.c_str());
    }
    
    bool flag = false;
    for(int i=1;i<10;i++)
    {
        if(data[i])
        {
            flag = true;
            break;
        }
    }
    
    return flag;
}
    
