#include <iostream>
#include "UtilesIO.h"
#include "Utiles.h"

using namespace std;

ostream & writeArithmeticArray(ostream & os, const char * pre, double * data, size_t size, bool newLine)
{
    os << currentDateTime() << " " << pre;
    for(size_t i = 0; i < size; ++i){
        os << " " << data[i];
    }

    if(newLine)
        os << endl;

    return os;
}
