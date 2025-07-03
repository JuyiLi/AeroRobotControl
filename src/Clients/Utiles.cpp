#include <cstdio>
#include <cstdlib>
#include <signal.h>
#include "Utiles.h"
#include <unistd.h>
#include <sched.h>
#include <time.h>
#include <sstream>
#include <iostream>
#include <array>
#include <sys/syscall.h>

using namespace std;

//
//Static variables
//
static const string printPre("Utiles: ");
static const string printErr("UtilesError: ");


//
//Linux operation.
//
//System Signals process function pointer.
void systemSignalsProcessPointer(int sig)
{
    cout << printPre << "SIGPIPE GET!" << endl;
}

//System Signals process.
void configSystemSignalsProcess()
{
    //Define signal process data struct.
    struct sigaction sigPB;
    sigPB.sa_handler = systemSignalsProcessPointer;
    sigemptyset(&sigPB.sa_mask);
    sigPB.sa_flags = 0;

    //Resgister system signal.(SIGPIPE)
    sigaction(SIGPIPE, &sigPB, NULL);
}

//Set real time thread priority.
void configThreadRTPrioritySet(int threadRTGrade)
{
    //Define priority grade.
    struct sched_param sched;
    sched.sched_priority = threadRTGrade;

    //Set priority of RT thread with SCHED_FIFO(RT pattern).
    if(sched_setscheduler(getpid(), SCHED_RR, &sched) < 0){
        cout << printErr << "Set thread priority error. Process exit." << endl;
        exit(EXIT_FAILURE);
    }
    cout << printPre << "Thread pid: " << getpid() << endl;
}

//Print thread ID.
void printThreadID(string threadName)
{
    cout << threadName << " thread ID: " << syscall(SYS_gettid) << endl;
}



//
//Time and duration.
//
//Sleep for u microseconds.
void sleep_us(unsigned int us)
{
    struct timespec reqtime;
    reqtime.tv_sec = 0;
    reqtime.tv_nsec = us * 1000;
    nanosleep(&reqtime, NULL);
}

//Sleep for m milliseconds.
void sleep_ms(unsigned int ms)
{
    unsigned int i = 0;
    for (i=0; i < ms; i++)
        sleep_us(1000);
}

//Current time.(ms)
long long currentLinuxTime(void)
{
    struct timespec time_m = {0, 0};

    clock_gettime(CLOCK_REALTIME, &time_m);
    return static_cast<long long>(time_m.tv_sec * 1000 + time_m.tv_nsec / 1000000);
}

// Get current date/time, format is YYYY_MM_DD.HH:mm:ss
const std::string currentDateTime()
{
    time_t now = time(0);
    char buf[80];
    struct tm tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y_%m_%d.%X", &tstruct);

    return buf;
}

// Get current date/time, format is YYYY_MM_DD
const std::string currentDate()
{
    time_t now = time(0);
    char buf[80];
    struct tm tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y_%m_%d", &tstruct);

    return buf;
}


//
//String operation.
//
//Split string function one. Filters out excess no char beween two delimiters.
void splitString1(vector<string> &tokens, const string &str, const string &delimiters) {
    // Skip delimiters at beginning
    string::size_type lastPos = str.find_first_not_of(delimiters, 0);

    // Find first non-delimiter
    string::size_type pos = str.find_first_of(delimiters, lastPos);

    while (string::npos != pos || string::npos != lastPos) {
        // Found a token, add it to the vector
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next non-delimiter
        pos = str.find_first_of(delimiters, lastPos);
    }
}

//Split string function one. Don't filters out no char beween two excess delimiters.
void splitString2(vector<string> &tokens, const string &str, char delim) {
    stringstream ss(str); //convert string to stream
    string item;
    while(getline(ss, item, delim)) {
        tokens.push_back(item); //add token to vector
    }
}

//Transform char array(see data as acsii) to string.
string asciiToString(char * data, size_t size)
{
    string t;
    string last;
    for(size_t i = 0; i < size; ++i){
        t = data[i];
        last = last + t;
    }

    return last;
}



