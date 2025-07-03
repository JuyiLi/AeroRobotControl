#ifndef UTILES_H
#define UTILES_H

//
// Auther:      liuRuJia
// Create time: 21/08/24
// Modify time: 21/08/27
// Vertion:     1.1
//


#include <vector>
#include <string>
#include <array>


//
//Linux operation.
//
//System Signals process.
void configSystemSignalsProcess();

//Set thread real time priority.
void configThreadRTPrioritySet(int threadRTGrade);

//Print thread ID.
void printThreadID(std::string threadName);


//
//Time and duration.
//
//Sleep for u microseconds.
void sleep_us(unsigned int us_val);

//Sleep for m milliseconds.
void sleep_ms(unsigned int ms_val);

//Current time.(ms)
long long currentLinuxTime(void);

// Get current date/time, format is YYYY_MM_DD.HH:mm:ss
const std::string currentDateTime();

// Get current date/time, format is YYYY_MM_DD
const std::string currentDate();


//
//String operation.
//
//Split string function one. Filters out excess no char beween two delimiters.
void splitString1(std::vector<std::string> &tokens, const std::string &str, const std::string &delimiters = " ");

//Split string function one. Don't filters out no char beween two excess delimiters.
void splitString2(std::vector<std::string> &tokens, const std::string &str, char delim = ' ');

//Transform char array(see data as acsii) to string.
std::string asciiToString(char * data, size_t size);







#endif // UTILES_H
