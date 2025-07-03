#include <iostream>
#include <unistd.h>
#include <string>
#include <memory>
#include <map>
#include <vector>
#include <cstring>
#include <cmath>
#include "AeroRobotControl.h"
#include "UtilesIO.h"
#include "Utiles.h"
#include <sstream>
#include <fstream>
#include <chrono>
#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

using namespace std;
using namespace AeroRobot;

using Array8d = array<double, 8>;
using Array7d = array<double, 7>;
using Array6d = array<double, 6>;
using Array2d = array<double, 2>;


std::ostream & operator<<(std::ostream &os, const Array6d & a)
{
    for(size_t i = 0; i < 6; ++i){
        os << a[i] << " ";
    }

    return os;
}

void Array6dToDouble6d(double * ld_ptr, const Array6d &rd)
{
    for(size_t i = 0; i < 6; ++i){
        ld_ptr[i] = rd[i];
    }
}


Array6d Double6dToArray6d(const double * const rd_ptr)
{
    return {rd_ptr[0], rd_ptr[1], rd_ptr[2], rd_ptr[3], rd_ptr[4], rd_ptr[5]};
}



static std::shared_ptr<spdlog::logger> testLog_ptr;
void setTestSpdLogger(const std::string & robotIP)
{
    /**
     * @brief fInit 初始化次数设置。
     */
    static bool fInit = false;
    if(fInit){
        return;
    }


    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::trace);

    auto daily_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(std::string(getenv("HOME")) + "/ARCLOG/RobotControlTest_Daily_Log_" + robotIP + "_.txt", 0, 0);
    daily_sink->set_level(spdlog::level::trace);

    spdlog::sinks_init_list sink_list = { daily_sink, console_sink };
    testLog_ptr = std::make_shared<spdlog::logger>("RobotControlTestLog", sink_list.begin(), sink_list.end());

    testLog_ptr->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
    testLog_ptr->flush_on(spdlog::level::err);
    spdlog::register_logger(testLog_ptr);
    spdlog::flush_every(std::chrono::seconds(1));

    fInit = true;
}


int main(int argc, char *argv[])
{
    AeroRobotControl arc("192.168.1.207");
    arc.connectToRobot();
    arc.setServoOn();

    double j[7] = {0};
    arc.getCurrentPoseJointSpace(j);
    cout << j[0] << " " << j[1] << " " << j[2] << " " << j[3] << " " << j[4] << " " << j[5] << endl;

    sleep(2);

    arc.setServoOff();

    arc.disconnectToRobot();
    return 1;


//    std::string robotIP{"127.0.0.1"};

////    if(argc >= 3){
////        stringstream input;
////        input << "输入参数数量： " << argc << " 具体参数： ";
////        for(int i = 1; i < argc; ++i){
////            input << argv[i] << " ";
////        }
////        spdlog::info("{}.", input.str());

////        if(!strcmp(argv[1], "--IP")){
////            robotIP = argv[2];
////        }
////    }
////    setTestSpdLogger(robotIP);

//    //==================================================================== 快速数据 ====================================================================

//    //@brief 0位置。
////    double joint0[6]  = {0, 0, 0, 0, 0, 0};
////    //@brief 60度位置。
////    double joint60[6] = {60.0 / 180.0 * M_PI, 60.0 / 180.0 * M_PI, -60.0 / 180.0 * M_PI, 60.0 / 180.0 * M_PI, 60.0 / 180.0 * M_PI, 60.0 / 180.0 * M_PI};
////    //@brief -60度位置。
////    double joint60Minus[6]= {-60.0 / 180.0 * M_PI, -60.0 / 180.0 * M_PI, 60.0 / 180.0 * M_PI, -60.0 / 180.0 * M_PI, -60.0 / 180.0 * M_PI, -60.0 / 180.0 * M_PI};
////    //@brief 静态电流测试位置。
////    double jointn30[6]= {-60.0 / 180.0 * M_PI, -60.0 / 180.0 * M_PI, 0.0 / 180.0 * M_PI, 60.0 / 180.0 * M_PI, 30.0 / 180.0 * M_PI, 60.0 / 180.0 * M_PI};
////    //@brief 笛卡尔运动初始位置1。
////    double CartMoveOriginalPos1[6]= {0.0 / 180.0 * M_PI, 0.0 / 180.0 * M_PI, 90.0 / 180.0 * M_PI, 0.0 / 180.0 * M_PI, 90.0 / 180.0 * M_PI, 0.0 / 180.0 * M_PI};
////    //@brief 笛卡尔运动初始位置2。
////    double CartMoveOriginalPos2[6]= {0.0 / 180.0 * M_PI, 0.0 / 180.0 * M_PI, 90.0 / 180.0 * M_PI, 90.0 / 180.0 * M_PI, 90.0 / 180.0 * M_PI, 0.0 / 180.0 * M_PI};

////    double test[7] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, -0.1};

//    //=================================================================================================================================================


//    //===================================================================== 运动功能测试 =====================================================================

////    AeroRobotControl * armTestPtr = new AeroRobotControl(robotIP.c_str());
////    if(!armTestPtr->connectToRobot()){
////        testLog_ptr->info("Connect to arm failed." );
////        sleep(1);
////        delete armTestPtr;
////    }
////    //delete armTestPtr;
////
////    sleep(5);
////    AeroRobotControl * armTestPtr1 = new AeroRobotControl(robotIP.c_str());
////    if(!armTestPtr1->connectToRobot()){
////        testLog_ptr->info("Connect to arm failed." );
////        sleep(1);
////        delete armTestPtr1;
////    }
////    delete armTestPtr1;
////    exit(1);

//    AeroRobotControl armTest(robotIP.c_str(), INFO_SERVICE_WAY);
//    if(!armTest.connectToRobot()){
//        testLog_ptr->info("Connect to arm failed." );
//        sleep(1);
//        //exit(1);
//    }
//    sleep(2);
//    //armTest.disconnectToRobot();
//    //sleep(5);
////    if(!armTest.connectToRobot()){
////        testLog_ptr->info("Connect to arm failed." );
////        sleep(1);
////        //exit(1);
////    }
////    armTest.disconnectToRobot();
////
////    sleep(2);
////    exit(1);



////    sleep(1);
////    armTest.setServoOff();
////    sleep(1);
////    armTest.isPrintingError(true);
////    armTest.setServoOn();
////    sleep(1);

////    exit(1);
//    std::thread getInfoThread([&](){
//        while (true) {
//            double cart[6];
//            armTest.getCurrentPoseCartSpace(cart);
//            testLog_ptr->info("Cart : {}.", Double6dToArray6d(cart));
//            usleep(3000);
//        }
//    });
//    std::thread getInfo2Thread([&](){
//        while (true) {
//            double joint[6];
//            armTest.getCurrentPoseJointSpace(joint);
//            testLog_ptr->info("Joint: {}.", Double6dToArray6d(joint));
//            usleep(8000);
//        }
//    });

//    sleep(2);
//    bool ret = armTest.moveTaskToUntilFinish(CartMoveOriginalPos1, 60);
//    while (!ret){
//        testLog_ptr->info("Move to original position failed.");
//        sleep(1);
//        ret = armTest.moveTaskToUntilFinish(CartMoveOriginalPos1, 60);
//    }

//    getInfoThread.join();
//    getInfo2Thread.join();
//    spdlog::info("After get sth.");

//    double pos1[6]{-80 / 180.0 * M_PI, -70 / 180.0 * M_PI, 230 / 180.0 * M_PI, 200 / 180.0 * M_PI, 0, 0};
//    double pos0[6]{0, 0, 0, 0, -20 / 180.0 * M_PI, 0};
//    double pos_1[6]{290 / 180.0 * M_PI, 300 / 180.0 * M_PI, -140 / 180.0 * M_PI, -170 / 180.0 * M_PI, -380 / 180.0 * M_PI, 0};

//    while (true){
//        armTest.moveJointToUntilFinish(pos1, 1000);
//        testLog_ptr->info("运动到1.");
//        sleep(1);
//        armTest.moveJointToUntilFinish(pos0, 1000);
//        testLog_ptr->info("运动到0.");
//        sleep(1);
//        armTest.moveJointToUntilFinish(pos_1, 1000);
//        testLog_ptr->info("运动到-1.");
//        sleep(1);
//    }

//    armTest.moveJointToUntilFinish(pos1, 1000);
//    testLog_ptr->info("运动到1.");
//    sleep(10);
//    armTest.moveJointToUntilFinish(pos0, 1000);
//    testLog_ptr->info("运动到0.");
//    sleep(10);
//    armTest.moveJointToUntilFinish(pos_1, 1000);
//    testLog_ptr->info("运动到-1.");
//    sleep(10);


//    getInfoThread.join();
//    getInfo2Thread.join();



//    while(true){
//        static int times = 0;
//        testLog_ptr->info("[START MOTION]: Times: {}. Move to CartMoveOriginalPos1: {}", times, Double6dToArray6d(CartMoveOriginalPos1));
//        if (!armTest.moveJointToUntilFinish(joint0)){
//            testLog_ptr->info("Move the joint0 failed." );
//            break;
//        }
//        chrono::system_clock::time_point testStartTime = chrono::system_clock::now();



//        /**
//         * @brief 0. TEST
//         */
//        testLog_ptr->info("\n\n========================= TEST =========================");
//        chrono::system_clock::time_point testTime = chrono::system_clock::now();

//        if (!armTest.test(test)){
//            testLog_ptr->info("TEST failed." );
//            break;
//        }
//        sleep(10);
//        chrono::duration<double> testTimeEnd = chrono::system_clock::now() - testTime;
//        testLog_ptr->info("TEST duration: {}s.", testTimeEnd.count());


//        armTest.set_joint_move_speed(0.3);
//        if (!armTest.moveJointToUntilFinish(CartMoveOriginalPos1, 360)){
//            testLog_ptr->info("Move failed." );
//            break;
//        }


//        /**
//         * @brief 1. theSameJointPoint TEST
//         */
//        testLog_ptr->info("\n\n========================= theSameJointPoint TEST =========================");
//        chrono::system_clock::time_point theSameJointPointstart = chrono::system_clock::now();
//        armTest.set_joint_move_speed(0.3);
//        if (!armTest.moveJointToUntilFinish(CartMoveOriginalPos1, 360)){
//            testLog_ptr->info("Move the same joint point failed." );
//            break;
//        }
//        chrono::duration<double> theSameJointPointd = chrono::system_clock::now() - theSameJointPointstart;
//        testLog_ptr->info("Move the same joint point duration: {}s.", theSameJointPointd.count());

//        /**
//         * @brief 2. theSameCartPoint TEST
//         */
//        testLog_ptr->info("\n\n========================= theSameCartPoint TEST =========================");
//        chrono::system_clock::time_point theSameCartPointstart = chrono::system_clock::now();
//        double theSameCartPoint[6];
//        armTest.getCurrentPoseCartSpace(theSameCartPoint);
//        if (!armTest.moveTaskToUntilFinish(theSameCartPoint)){
//            testLog_ptr->info("Move the same cart point failed." );
//            break;
//        }
//        chrono::duration<double> theSameCartPointd = chrono::system_clock::now() - theSameCartPointstart;
//        testLog_ptr->info("Move the same cart point duration: {}s.", theSameCartPointd.count());



//        /**
//         * @brief 3. moveTaskToUntilFinish TEST
//         */
//        testLog_ptr->info("\n\n========================= moveTaskToUntilFinish TEST =========================");
//        double currCart[6];
//        armTest.getCurrentPoseCartSpace(currCart);
//        auto moveTaskToUntilFinishLambd = [&](int index, double diff){
//            chrono::system_clock::time_point start = chrono::system_clock::now();
//            testLog_ptr->info("[NEW MOTION]: moveTaskToUntilFinish index[{}], diff[{}].", index, diff);
//            for (int i = 0; i < 10; ++i){
//                currCart[index] += diff;
//                if (!armTest.moveTaskToUntilFinish(currCart)){
//                    testLog_ptr->warn("moveTaskToUntilFinish failed: index[{}], diff[{}].", index, diff);
//                    exit(1);
//                }
//            }
//            chrono::duration<double> d = chrono::system_clock::now() - start;
//            testLog_ptr->info("moveTaskToUntilFinish duration: {}s.", d.count());
//        };
//        moveTaskToUntilFinishLambd(0, 0.02);
//        moveTaskToUntilFinishLambd(1, 0.02);
//        moveTaskToUntilFinishLambd(2, 0.02);
//        moveTaskToUntilFinishLambd(0, -0.02);
//        moveTaskToUntilFinishLambd(1, -0.02);
//        moveTaskToUntilFinishLambd(2, -0.02);

//        /**
//         * @brief 4. setEmergencyOn TEST
//         */
//        armTest.setEmergencyOn();
//        double speed[2] = {0.08, 0.4};
//        if(!armTest.set_Task_move_speed(speed))
//            testLog_ptr->info("Set task move speed failed." );
//        sleep(1);
//        armTest.setEmergencyOff();
//        sleep(1);

//        /**
//         * @brief 5. moveTaskToWithSpeed TEST
//         */
//        testLog_ptr->info("\n\n========================= moveTaskToWithSpeed TEST =========================");
//        if (!armTest.moveJointToUntilFinish(CartMoveOriginalPos1, 360)){
//            testLog_ptr->info("Move failed." );
//            break;
//        }
//        armTest.getCurrentPoseCartSpace(currCart);
//        double CartWithSpeed[8] = {currCart[0], currCart[1], currCart[2], currCart[3], currCart[4], currCart[5], 0.3, 0.5};
//        auto moveTasktoWithSpeedLambd = [&](int index, double diff){
//            chrono::system_clock::time_point start = chrono::system_clock::now();
//            testLog_ptr->info("[NEW MOTION]: moveTaskToWithSpeed index[{}], diff[{}], speed[{}, {}].", index, diff, CartWithSpeed[6], CartWithSpeed[7]);

//            CartWithSpeed[index] += diff;
//            if (!armTest.moveTaskToWithSpeed(CartWithSpeed)){
//                testLog_ptr->warn("moveTaskToWithSpeed failed: index[{}], diff[{}].", index, diff);
//                exit(1);
//            }
//            while(!armTest.isMoveFinished()){
//                usleep(10000);
//            }
//            sleep_ms(200);

//            chrono::duration<double> d = chrono::system_clock::now() - start;
//            testLog_ptr->info("moveTaskToWithSpeed duration: {}s.", d.count());
//        };
//        moveTasktoWithSpeedLambd(0, 0.4);
//        moveTasktoWithSpeedLambd(0, -0.4);
//        moveTasktoWithSpeedLambd(1, 0.3);
//        moveTasktoWithSpeedLambd(1, -0.3);
//        moveTasktoWithSpeedLambd(2, 0.2);
//        moveTasktoWithSpeedLambd(2, -0.2);


//        /**
//         * @brief 6. moveTaskBy TEST
//         */
//        testLog_ptr->info("\n\n========================= moveTaskBy TEST =========================");
//        double CartBy[6] = {0, 0, 0, 0, 0, 0};
//        auto moveTaskByLambd = [&](int index, double diff){
//            chrono::system_clock::time_point start = chrono::system_clock::now();
//            testLog_ptr->info("[NEW MOTION]: moveTaskBy index[{}], diff[{}], speed[{}, {}].", index, diff, CartWithSpeed[6], CartWithSpeed[7]);
//            for (int i = 0; i < 10; ++i){
//                CartBy[index] = diff;
//                if (!armTest.moveTaskBy(CartBy)){
//                    testLog_ptr->warn("moveTaskBy failed: index[{}], diff[{}].", index, diff);
//                    exit(1);
//                }
//                CartBy[0] = 0;
//                while(!armTest.isMoveFinished()){
//                    usleep(10000);
//                }
//            }
//            chrono::duration<double> d = chrono::system_clock::now() - start;
//            testLog_ptr->info("moveTaskBy duration: {}s.", d.count());
//        };
//        moveTaskByLambd(0, 0.02);
//        moveTaskByLambd(1, 0.02);
//        moveTaskByLambd(2, 0.02);


//        /**
//         * @brief 7. moveTaskByTCP TEST
//         */
//        testLog_ptr->info("\n\n========================= moveTaskByTCP TEST =========================");
//        auto moveTaskByTCPLambd = [&](int index, double diff){
//            chrono::system_clock::time_point start = chrono::system_clock::now();
//            testLog_ptr->info("[NEW MOTION]: moveTaskByTCP index[{}], diff[{}], speed[{}, {}].", index, diff, CartWithSpeed[6], CartWithSpeed[7]);
//            for (int i = 0; i < 10; ++i){
//                CartBy[index] = diff;
//                if (!armTest.moveTaskByTCP(CartBy)){
//                    testLog_ptr->warn("moveTaskByTCP failed: index[{}], diff[{}].", index, diff);
//                    exit(1);
//                }
//                CartBy[0] = 0;
//                while(!armTest.isMoveFinished()){
//                    usleep(10000);
//                }
//            }
//            chrono::duration<double> d = chrono::system_clock::now() - start;
//            testLog_ptr->info("moveTaskByTCP duration: {}s.", d.count());
//        };
//        moveTaskByTCPLambd(0, 0.02);
//        moveTaskByTCPLambd(1, 0.02);
//        moveTaskByTCPLambd(2, 0.02);


//        /**
//         * @brief 8. moveJointToUntilFinish TEST
//         */
//        testLog_ptr->info("\n\n========================= moveJointToUntilFinish TEST =========================");
//        auto moveJointToUntilFinishLambd = [&](double * target_ptr){
//            chrono::system_clock::time_point start = chrono::system_clock::now();
//            testLog_ptr->info("[NEW MOTION]: moveJointToUntilFinish : {}.", Double6dToArray6d(target_ptr));
//            if (!armTest.moveJointToUntilFinish(target_ptr, 360)){
//                testLog_ptr->info("moveJointToUntilFinish failed." );
//                exit(1);
//            }
//            chrono::duration<double> d = chrono::system_clock::now() - start;
//            testLog_ptr->info("moveJointToUntilFinish duration: {}s.", d.count());
//        };
//        moveJointToUntilFinishLambd(joint0);
//        moveJointToUntilFinishLambd(joint60);
//        moveJointToUntilFinishLambd(joint60Minus);
//        moveJointToUntilFinishLambd(joint0);


//        /**
//         * @brief 9. JOG MoveContinuousJoint TEST
//         */
//        testLog_ptr->info("\n\n========================= MoveContinuousJoint TEST =========================");
//        armTest.JogSpeedDirectSet(3);
//        auto MoveContinuousJointLambd = [&](int axis, bool direction){
//            chrono::system_clock::time_point start = chrono::system_clock::now();
//            testLog_ptr->info("[NEW MOTION]: MoveContinuousJoint : axis[{}], direction[{}].", axis, direction);
//            if (!armTest.MoveContinuousJoint(axis, direction)){
//                testLog_ptr->info("JOG MoveContinuousJoint failed. axis[{}], direction[{}].", axis, direction);
//                exit(1);
//            }
//            sleep(2);
//            armTest.JogMoveStop();
//            sleep(1);
//            chrono::duration<double> d = chrono::system_clock::now() - start;
//            testLog_ptr->info("MoveContinuousJoint duration: {}s.", d.count());
//        };
//        MoveContinuousJointLambd(1, true);
//        MoveContinuousJointLambd(2, true);
//        MoveContinuousJointLambd(3, true);
//        MoveContinuousJointLambd(4, true);
//        MoveContinuousJointLambd(5, true);
//        MoveContinuousJointLambd(6, true);
//        MoveContinuousJointLambd(1, false);
//        MoveContinuousJointLambd(2, false);
//        MoveContinuousJointLambd(3, false);
//        MoveContinuousJointLambd(4, false);
//        MoveContinuousJointLambd(5, false);
//        MoveContinuousJointLambd(6, false);



//        /**
//         * @brief 10. JOG MoveContinuousCartesian TEST
//         */
//        testLog_ptr->info("\n\n========================= MoveContinuousCartesian TEST =========================");
//        if (!armTest.moveJointToUntilFinish(CartMoveOriginalPos1)){
//            testLog_ptr->info("MoveContinuousCartesian Move failed." );
//            exit(1);
//        }
//        auto MoveContinuousCartesianLambd = [&](int axis, bool direction, int refCoordinate, int posRota){
//            chrono::system_clock::time_point start = chrono::system_clock::now();
//            if(posRota == 0){
//                testLog_ptr->info("[NEW MOTION]: MoveContinuousCartesianPose : axis[{}], direction[{}], refCoordinate[{}].", axis, direction, refCoordinate);
//                if (!armTest.MoveContinuousCartesianPose(axis, direction, refCoordinate)){
//                    testLog_ptr->info("JOG MoveContinuousCartesian failed. axis[{}], direction[{}], refCoordinate[{}].", axis, direction, refCoordinate);
//                    exit(1);
//                }
//            }
//            else{
//                testLog_ptr->info("[NEW MOTION]: MoveContinuousCartesianRota : axis[{}], direction[{}], refCoordinate[{}].", axis, direction, refCoordinate);
//                if (!armTest.MoveContinuousCartesianRota(axis, direction, refCoordinate)){
//                    testLog_ptr->info("JOG MoveContinuousCartesian failed. axis[{}], direction[{}], refCoordinate[{}].", axis, direction, refCoordinate);
//                    exit(1);
//                }

//            }
//            sleep(2);
//            armTest.JogMoveStop();
//            sleep(1);
//            chrono::duration<double> d = chrono::system_clock::now() - start;
//            if(posRota == 0){
//                testLog_ptr->info("MoveContinuousCartesianPose duration: {}s.", d.count());
//            }
//            else{
//                testLog_ptr->info("MoveContinuousCartesianRota duration: {}s.", d.count());
//            }
//        };
//        MoveContinuousCartesianLambd(1, true, 0, 0);
//        MoveContinuousCartesianLambd(1, false, 0, 0);
//        MoveContinuousCartesianLambd(2, true, 0, 0);
//        MoveContinuousCartesianLambd(2, false, 0, 0);
//        MoveContinuousCartesianLambd(3, true, 0, 0);
//        MoveContinuousCartesianLambd(3, false, 0, 0);
//        MoveContinuousCartesianLambd(1, true, 1, 0);
//        MoveContinuousCartesianLambd(1, false, 1, 0);
//        MoveContinuousCartesianLambd(2, true, 1, 0);
//        MoveContinuousCartesianLambd(2, false, 1, 0);
//        MoveContinuousCartesianLambd(3, true, 1, 0);
//        MoveContinuousCartesianLambd(3, false, 1, 0);

//        MoveContinuousCartesianLambd(1, true, 0, 1);
//        MoveContinuousCartesianLambd(1, false, 0, 1);
//        MoveContinuousCartesianLambd(2, true, 0, 1);
//        MoveContinuousCartesianLambd(2, false, 0, 1);
//        MoveContinuousCartesianLambd(3, true, 0, 1);
//        MoveContinuousCartesianLambd(3, false, 0, 1);
//        MoveContinuousCartesianLambd(1, true, 1, 1);
//        MoveContinuousCartesianLambd(1, false, 1, 1);
//        MoveContinuousCartesianLambd(2, true, 1, 1);
//        MoveContinuousCartesianLambd(2, false, 1, 1);
//        MoveContinuousCartesianLambd(3, true, 1, 1);
//        MoveContinuousCartesianLambd(3, false, 1, 1);


//        /**
//         * @brief 11. moveJointToContinuesWithDurat TEST
//         */
//        testLog_ptr->info("\n\n========================= moveJointToContinuesWithDurat TEST =========================");
//        if (!armTest.moveJointToUntilFinish(joint0)){
//            testLog_ptr->info("MoveContinuousCartesian Move failed." );
//            exit(1);
//        }
//        auto moveJointToContinuesWithDuratLambd = [&](double runTime, double sleepTime, double distance){
//            chrono::system_clock::time_point start = chrono::system_clock::now();
//            testLog_ptr->info("[NEW MOTION]: moveJointToContinuesWithDurat : runTime[{}], sleepTime[{}], distance[{}].", runTime, sleepTime, distance);

//            double currJoint[6];
//            armTest.getCurrentPoseJointSpace(currJoint);
//            for (int i = 0; i < 20; ++i){
//                for (int i = 0; i < 6; ++i){
//                    currJoint[i] += distance;
//                }
//                double currJointWithSpeed[7] = {currJoint[0], currJoint[1], currJoint[2], currJoint[3], currJoint[4], currJoint[5], runTime};
//                if (!armTest.moveJointToContinuesWithDurat(currJointWithSpeed)){
//                    testLog_ptr->info("moveJointToContinuesWithDurat failed." );
//                    exit(1);
//                }
//                usleep(sleepTime * 1000000);
//            }

//            chrono::duration<double> d = chrono::system_clock::now() - start;
//            testLog_ptr->info("moveJointToContinuesWithDurat duration: {}s.", d.count());
//        };
//        moveJointToContinuesWithDuratLambd(1, 0.8, 0.05);
//        moveJointToContinuesWithDuratLambd(3, 2.5, -0.1);



//        /**
//         * @brief 12. moveTaskToContinues TEST
//         */
//        testLog_ptr->info("\n\n========================= moveTaskToContinues TEST =========================");
//        if (!armTest.moveJointToUntilFinish(CartMoveOriginalPos1, 360)){
//            testLog_ptr->info("moveTaskToContinues Move failed." );
//            exit(1);
//        }
//        auto moveTaskToContinuesLambd = [&](int index, double distance, double tVel){
//            chrono::system_clock::time_point start = chrono::system_clock::now();
//            testLog_ptr->info("[NEW MOTION]: moveTaskToContinues : index[{}], distance[{}], tVel[{}].", index, distance, tVel);

//            double currCart[6];
//            armTest.getCurrentPoseCartSpace(currCart);
//            for (int i = 0; i < 10; ++i){
//                currCart[index] += distance;
//                double currCartWithSpeed[8] = {currCart[0], currCart[1], currCart[2], currCart[3], currCart[4], currCart[5], tVel, 0.5};
//                if (!armTest.moveTaskToContinues(currCartWithSpeed)){
//                    testLog_ptr->info("moveTaskToContinues failed." );
//                    exit(1);
//                }
//                usleep(fabs(distance) / tVel * 0.8 * 1000000);
//            }

//            chrono::duration<double> d = chrono::system_clock::now() - start;
//            testLog_ptr->info("moveTaskToContinues duration: {}s.", d.count());
//        };
//        moveTaskToContinuesLambd(0, 0.02, 0.05);
//        moveTaskToContinuesLambd(0, -0.02, 0.05);
//        moveTaskToContinuesLambd(1, 0.02, 0.05);
//        moveTaskToContinuesLambd(1, -0.02, 0.05);
//        moveTaskToContinuesLambd(2, 0.02, 0.05);
//        moveTaskToContinuesLambd(2, -0.02, 0.05);
//        sleep(1);

//        /**
//         * @brief 13. moveTaskToLineSerial TEST
//         */
//        testLog_ptr->info("\n\n========================= moveTaskToLineSerial TEST =========================");
//        if (!armTest.moveJointToUntilFinish(CartMoveOriginalPos1, 360)){
//            testLog_ptr->info("moveTaskToContinues Move failed." );
//            exit(1);
//        }
//        sleep(1);
//        double currCart1[6];
//        armTest.getCurrentPoseCartSpace(currCart1);
//        if (!armTest.pushLineSerialPoints(currCart1, 0.16)){
//            testLog_ptr->info("pushLineSerialPoints failed." );
//            exit(1);
//        }

//        currCart1[0] += 0.1;
//        currCart1[1] += 0.1;
//        if (!armTest.pushLineSerialPoints(currCart1, 0.1)){
//            testLog_ptr->info("pushLineSerialPoints failed." );
//            exit(1);
//        }

//        currCart1[0] -= 0.1;
//        currCart1[1] += 0.1;
//        currCart1[2] += 0.1;
//        if (!armTest.pushLineSerialPoints(currCart1, 0.15)){
//            testLog_ptr->info("pushLineSerialPoints failed." );
//            exit(1);
//        }

//        currCart1[0] -= 0.1;
//        currCart1[1] += 0.1;
//        currCart1[2] += 0.1;
//        if (!armTest.pushLineSerialPoints(currCart1, 0.05)){
//            testLog_ptr->info("pushLineSerialPoints failed." );
//            exit(1);
//        }

//        if (!armTest.pushLineSerialPoints(currCart1, 0.15)){
//            testLog_ptr->info("pushLineSerialPoints failed." );
//            exit(1);
//        }

//        currCart1[2] -= 0.1;
//        if (!armTest.pushLineSerialPoints(currCart1, 0.2)){
//            testLog_ptr->info("pushLineSerialPoints failed." );
//            exit(1);
//        }

//        currCart1[1] -= 0.1;
//        if (!armTest.pushLineSerialPoints(currCart1, 0.05)){
//            testLog_ptr->info("pushLineSerialPoints failed." );
//            exit(1);
//        }

//        currCart1[0] += 0.15;
//        currCart1[1] -= 0.05;
//        currCart1[2] -= 0.1;
//        if (!armTest.pushLineSerialPoints(currCart1, 0.09)){
//            testLog_ptr->info("pushLineSerialPoints failed." );
//            exit(1);
//        }

//        if (!armTest.pushLineSerialPoints(currCart1, 0.3)){
//            testLog_ptr->info("pushLineSerialPoints failed." );
//            exit(1);
//        }

//        currCart1[0] += 0.05;
//        currCart1[1] -= 0.05;
//        currCart1[2] -= 0.2;
//        if (!armTest.pushLineSerialPoints(currCart1, 0.13)){
//            testLog_ptr->info("pushLineSerialPoints failed." );
//            exit(1);
//        }

//        currCart1[0] -= 0.12;
//        currCart1[1] -= 0.05;
//        currCart1[2] += 0.17;
//        if (!armTest.pushLineSerialPoints(currCart1, 0.13)){
//            testLog_ptr->info("pushLineSerialPoints failed." );
//            exit(1);
//        }

//        currCart1[0] -= 0.07;
//        currCart1[1] += 0.13;
//        currCart1[2] += 0.16;
//        if (!armTest.pushLineSerialPoints(currCart1, 0.34)){
//            testLog_ptr->info("pushLineSerialPoints failed." );
//            exit(1);
//        }

//        currCart1[0] -= 0.07;
//        currCart1[1] -= 0.13;
//        currCart1[2] -= 0.08;
//        if (!armTest.pushLineSerialPoints(currCart1, 0.11)){
//            testLog_ptr->info("pushLineSerialPoints failed." );
//            exit(1);
//        }

//        currCart1[0] += 0.03;
//        currCart1[1] += 0.17;
//        currCart1[2] -= 0.09;
//        if (!armTest.pushLineSerialPoints(currCart1, 0.21)){
//            testLog_ptr->info("pushLineSerialPoints failed." );
//            exit(1);
//        }

//        currCart1[0] += 0.03;
//        currCart1[1] -= 0.11;
//        currCart1[2] += 0.09;
//        if (!armTest.pushLineSerialPoints(currCart1, 0.07)){
//            testLog_ptr->info("pushLineSerialPoints failed." );
//            exit(1);
//        }

//        if (!armTest.pushLineSerialPoints(currCart1, 0.12)){
//            testLog_ptr->info("pushLineSerialPoints failed." );
//            exit(1);
//        }


//        if (!armTest.moveTaskToLineSerial()){
//            testLog_ptr->info("moveTaskToLineSerial failed." );
//            exit(1);
//        }
//        sleep(25);


//        if (!armTest.moveJointToUntilFinish(joint0, 360)){
//            testLog_ptr->info("Move failed." );
//            break;
//        }


//        /**
//         * @brief 14. moveJointToWithDefinedSpeed TEST
//         */
//        testLog_ptr->info("\n\n========================= moveJointToWithDefinedSpeed TEST =========================");
//        if (!armTest.moveJointToUntilFinish(joint0, 360)){
//            testLog_ptr->info("moveTaskToContinues Move failed." );
//            exit(1);
//        }
//        sleep(1);

//        double joint60WithSpeed[7] = {60.0 / 180.0 * M_PI, 60.0 / 180.0 * M_PI, -60.0 / 180.0 * M_PI, 60.0 / 180.0 * M_PI, 60.0 / 180.0 * M_PI, 60.0 / 180.0 * M_PI, 0.3};
//        if (!armTest.moveJointToWithDefinedSpeed(joint60WithSpeed)){
//            testLog_ptr->info("moveTaskToWithDefinedSpeed Move failed." );
//            exit(1);
//        }
//        sleep(6);

//        double joint60_WithSpeed[7] = {-60.0 / 180.0 * M_PI, -60.0 / 180.0 * M_PI, 60.0 / 180.0 * M_PI, -60.0 / 180.0 * M_PI, -60.0 / 180.0 * M_PI, -60.0 / 180.0 * M_PI, 0.35};
//        if (!armTest.moveJointToWithDefinedSpeed(joint60_WithSpeed)){
//            testLog_ptr->info("moveTaskToWithDefinedSpeed Move failed." );
//            exit(1);
//        }
//        sleep(12);

//        double joint0WithSpeed[7] = {0, 0, 0, 0, 0, 0, 0.5};
//        if (!armTest.moveJointToWithDefinedSpeed(joint0WithSpeed)){
//            testLog_ptr->info("moveTaskToWithDefinedSpeed Move failed." );
//            exit(1);
//        }
//        sleep(5);

//        double joint30WithSpeed[7] = {30.0 / 180.0 * M_PI, 30.0 / 180.0 * M_PI, -30.0 / 180.0 * M_PI, 30.0 / 180.0 * M_PI, 30.0 / 180.0 * M_PI, 30.0 / 180.0 * M_PI, 0.42};
//        if (!armTest.moveJointToWithDefinedSpeed(joint30WithSpeed)){
//            testLog_ptr->info("moveTaskToWithDefinedSpeed Move failed." );
//            exit(1);
//        }
//        sleep(4);


//        /**
//         * @brief 15. moveTaskToWithDefinedSpeed TEST
//         */
//        testLog_ptr->info("\n\n========================= moveTaskToWithDefinedSpeed TEST =========================");
//        if (!armTest.moveJointToUntilFinish(CartMoveOriginalPos1, 360)){
//            testLog_ptr->info("moveTaskToContinues Move failed." );
//            exit(1);
//        }
//        sleep(1);

//        double cart[6];
//        armTest.getCurrentPoseCartSpace(cart);

//        double cartWithSpeed[8] = {cart[0]+0.4, cart[1], cart[2], cart[3], cart[4], cart[5], 0.13, 0.5};
//        if (!armTest.moveTaskToWithDefinedSpeed(cartWithSpeed)){
//            testLog_ptr->info("moveTaskToWithDefinedSpeed Move failed." );
//            exit(1);
//        }
//        sleep(6);

//        double cart1WithSpeed[8] = {cart[0]-0.4, cart[1], cart[2], cart[3], cart[4], cart[5], 0.17, 0.5};
//        if (!armTest.moveTaskToWithDefinedSpeed(cart1WithSpeed)){
//            testLog_ptr->info("moveTaskToWithDefinedSpeed Move failed." );
//            exit(1);
//        }
//        sleep(12);

//        double cart2WithSpeed[8] = {cart[0], cart[1], cart[2], cart[3], cart[4], cart[5], 0.23, 0.5};
//        if (!armTest.moveTaskToWithDefinedSpeed(cart2WithSpeed)){
//            testLog_ptr->info("moveTaskToWithDefinedSpeed Move failed." );
//            exit(1);
//        }
//        sleep(5);

//        double cart3WithSpeed[8] = {cart[0], cart[1]+0.3, cart[2], cart[3], cart[4], cart[5], 0.4, 0.5};
//        if (!armTest.moveTaskToWithDefinedSpeed(cart3WithSpeed)){
//            testLog_ptr->info("moveTaskToWithDefinedSpeed Move failed." );
//            exit(1);
//        }
//        sleep(4);

//        double cart4WithSpeed[8] = {cart[0], cart[1]-0.3, cart[2], cart[3], cart[4], cart[5], 0.42, 0.5};
//        if (!armTest.moveTaskToWithDefinedSpeed(cart4WithSpeed)){
//            testLog_ptr->info("moveTaskToWithDefinedSpeed Move failed." );
//            exit(1);
//        }
//        sleep(10);


//        chrono::duration<double> d = chrono::system_clock::now() - testStartTime;
//        testLog_ptr->info("[FINISH MOTION] Times: {}, duration: {}s.\n\n", times, d.count());

//        times++;
//    }


//    armTest.setServoOff();




//    //=================================================================================================================================================


//    //================================================================= ROBOT TCP 测试 =================================================================

////    AeroRobotControl armTest("127.0.0.1");
////    if(!armTest.connectToRobot()){
////        cout << "Connect to arm up failed." << endl;
////        sleep(1);
////        exit(1);
////    }
////    sleep(1);
////    armTest.setServoOn();
////    sleep(1);

////    auto printCurrentTCP = [&]{
////        string name;
////        Array6d tcpPosE;
////        armTest.getCurrentRobotTCP(name, tcpPosE);
////        testLog_ptr->info("Name: {}, PosEuler: {}. ", name, tcpPosE);
////    };

////    auto printCurrentCart = [&]{
////        double currentCart[6];
////        armTest.getCurrentPoseCartSpace(currentCart);
////        testLog_ptr->info("Current Cart: {}, {}, {}, {}, {}, {}.", currentCart[0], currentCart[1], currentCart[2], currentCart[3], currentCart[4], currentCart[5]);
////    };

////    map<string, Array6d> testTcps;
////    testTcps.emplace(string{"zero"}, Array6d{0, 0, 0, 0, 0, 0});
////    testTcps.emplace(string{"x0.1"}, Array6d{0.1, 0, 0, 0, 0, 0});
////    testTcps.emplace(string{"y0.1"}, Array6d{0, 0.1, 0, 0, 0, 0});
////    testTcps.emplace(string{"z0.1"}, Array6d{0, 0, 0.1, 0, 0, 0});


////    printCurrentTCP();
////    printCurrentCart();


////    armTest.setNewCurrentRobotTCP(string{"z0.2"}, Array6d{0, 0, 0.2, 0, 0, 0});
////    printCurrentTCP();
////    sleep(1);
////    printCurrentCart();
////    sleep(1);

////    for(size_t j = 0; j < 2; ++j){
////        for(auto &singleTcp : testTcps){
////            testLog_ptr->info("======================= test for set New ===========================================");
////            armTest.setNewCurrentRobotTCP(singleTcp.first, singleTcp.second);
////            printCurrentTCP();
////            sleep(1);
////            printCurrentCart();

////            sleep(3);

//////            testLog_ptr->info("======================= test for setCurrentRobotTCP 错误 ===========================================");
//////            if(!armTest.setCurrentRobotTCP("nullTCP")){
//////                testLog_ptr->info("没有nullTCP，设置失败。");
//////            }

//////            sleep(1);

////            testLog_ptr->info("======================= test for setCurrentRobotTCP ===========================================");
////            armTest.setCurrentRobotTCP(string{"z0.2"});
////            printCurrentTCP();
////            sleep(1);
////            printCurrentCart();

////            sleep(5);

////        }
////    }




//    //=================================================================================================================================================



//    //================================================================= 静态数据采集测试 =================================================================

////    AeroRobotControl armTest("192.168.1.207");


////    if(!armTest.connectToRobot()){
////        cout << "Connect to arm up failed." << endl;
////        sleep(1);
////        exit(1);
////    }
////    sleep(1);

////    armTest.setServoOn();
////    sleep(3);


////    armTest.setServoOff();
////    sleep(1);
////    exit(1);

////    armTest.set_joint_move_speed(0.1);
////    armTest.moveJointToUntilFinish(joint30);
////    armTest.moveJointToUntilFinish(joint0);
////    sleep(1);

////    armTest.set_joint_move_speed(0.5);
////    armTest.moveJointToUntilFinish(joint30);
////    armTest.moveJointToUntilFinish(joint0);
////    sleep(1);

////    ofstream dataout("./ampere10.txt");
////    armTest.moveJointTo(jointn30);

////    while (1) {
////        double rs[6];
////        armTest.getCurrentAmpereJointSpace(rs);
////        double jointpos[6];
////        armTest.getCurrentPoseJointSpace(jointpos);
////        double accel[6];
////        armTest.getCurrentAccelJointSpace(accel);
////        dataout << currentLinuxTime() << " " << jointpos[0] << " " << jointpos[1] << " " << jointpos[2]
////                   << " " << jointpos[3] << " " << jointpos[4] << " " << jointpos[5]
////                   << " " << accel[0] << " " << accel[1] << " " << accel[2]
////                      << " " << accel[3] << " " << accel[4] << " " << accel[5]
////                << " " << rs[0] << " " << rs[1] << " " << rs[2] << " " << rs[3] << " " << rs[4] << " " << rs[5] << endl;
////        cout <<  accel[0] << " " << accel[1] << " " << accel[2]
////             << " " << accel[3] << " " << accel[4] << " " << accel[5] << endl;

////        usleep(10000);

////    }

//    //=================================================================================================================================================


//    //===================================================================== 控制模式 =====================================================================



//    //cout << "huiling" << endl;
//    //armTest.moveJointToUntilFinish(joint0);

//    //    if (!armTest.set_csv()){
//    //        cout<<"切换CSV失败"<<endl;
//    //        exit(1);
//    //    };
//    //    sleep(2);

//    //    double speed[6] = {0, 0, 0, 0, 0.5, 0.5};
//    //    cout<<"CSV运动"<<endl;
//    //    armTest.moveJointSpeedTo(speed);
//    //    sleep(10);

//    //    double speed0[6] = {0, 0, 0, 0, 0, 0};
//    //    cout<<"CSV运动停止"<<endl;
//    //    armTest.moveJointSpeedTo(speed0);
//    //    sleep(5);

//    //    if (!armTest.set_csp()){
//    //        cout<<"切换CSP失败"<<endl;
//    //        exit(1);
//    //    };
//    //    cout<<"CSP运动"<<endl;
//    //    armTest.moveJointToUntilFinish(joint0);



//    //=================================================================================================================================================


//    //===================================================================== 其他测试 =====================================================================


////    sleep(1);
////    double curr[6];
////    arm.getCurrentPoseCartSpace(curr);
////    double t1[6] = {-0.108, 0.535, 0.8335, 19.25 / 180 * 3.13159, -168.38 / 180 * 3.13159, -96.2 / 180 * 3.13159};
////    double t2[6] = {-0.209, 0.499, 0.855, 19.36 / 180 * 3.13159, -167.43 / 180 * 3.13159, -96.24 / 180 * 3.13159};
////    double t1Up[6] = {-0.466, 0.525, 0.76, 40.68 / 180 * 3.13159, -170.7 / 180 * 3.13159, -95.8 / 180 * 3.13159};
////    double t2Up[6] = {-0.2687, 0.69553, 0.7155, 43.16 / 180 * 3.13159, -149.07 / 180 * 3.13159, -95.87 / 180 * 3.13159};


////    armUp.moveTaskToUntilFinish(t1Up);
////    armUp.moveTaskToUntilFinish(t2Up);

////    double t1Down[6] = {-33.22 / 180 * M_PI, -12.0 / 180 * M_PI, -98.0 / 180 * M_PI, -130.0 / 180 * M_PI, -83.0 / 180 * M_PI, 119.0 / 180 * M_PI};
////    double t2Down[6] = {-17.38 / 180 * M_PI, -3.0 / 180 * M_PI, -105.0 / 180 * M_PI, -104.0 / 180 * M_PI, -89.0 / 180 * M_PI, 124.0 / 180 * M_PI};


////    armDown.set_joint_move_speed(0.1);
////    armDown.moveJointToUntilFinish(t1Down);
////    armDown.moveJointToUntilFinish(t2Down);

////    double t1Up[6] = {147.0 / 180.0 * M_PI, 80.0 / 180.0 * M_PI, -58.0 / 180.0 * M_PI, 26.0 / 180 * M_PI, 96.0 / 180 * M_PI, 32.0 / 180 * M_PI};
////    armUp.moveJointToUntilFinish(t1Up);





////    double tq[8];
////    tq[6] = 0.1;
////    tq[7] = 0;
////    for(int i = 0; i < 6; ++i){
////        tq[i] = cq[i];
////    }

////    for(int i = 0; i < 20; ++i){
////        tq[2] += 0.01;
////        armTest.moveTaskToContinues(tq);
////        usleep(80000);
////    }

////    sleep(1);
////    armTest.setServoOff();

////    double tj[6] = {0, 0, 0, 0, 0, 0.1};
////    armTest.getCurrentPoseJointSpace(tj);
////    armTest.moveJointTo(tj);

////    double cj[6] = {1, 1, 1, 1, 1, 1};
////    for(size_t i = 0; i < 10000000; ++i){
////        armTest.getCurrentPoseCartSpace(cj);
////        writeArithmeticArray(cout, "Current: ", cj, 6);
////        usleep(500000);
////    }




////    while (1) {
////        sleep(5);
////    }

//    //=================================================================================================================================================



}
