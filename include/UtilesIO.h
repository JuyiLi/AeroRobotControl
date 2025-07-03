#ifndef UTILESIO_H
#define UTILESIO_H

//
// Auther:      liuRuJia
// Create time: 21/11/24
// Modify time: 21/11/24
// Vertion:     1.0
//

#include <iostream>

//Write arithmetic array to stream. "size" presents arithmetic array size.
std::ostream & writeArithmeticArray(std::ostream & os, const char * prefix, double * data, size_t size, bool newLine = true);


#endif // UTILESIO_H
