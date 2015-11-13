/*	
Team:			VolcanoBot A
Project:		TBD
File:			oni-to-pcd.cpp
Description:	Reads an oni file recorded using the Openni2 or Openni library and outputs point clouds (pcd files)
*/

#include "../include/oni-to-pcd.h"
#include "../include/errorMsgHandler.h"
#include "../include/mainwindow.h"
#include <QApplication>
#include <iostream>
#include <boost/lockfree/queue.hpp>
#include <boost/lockfree/spsc_queue.hpp>

#include <boost/thread/thread.hpp>
#include <boost/atomic.hpp>
#include <boost/lockfree/policies.hpp>


// test for queue
boost::atomic_int producer_count(0);
boost::atomic_int consumer_count(0);

boost::lockfree::queue<int> queue(128);

const int iterations = 10000000;
const int producer_thread_count = 4;
const int consumer_thread_count = 4;

void producer(void)
{
    for (int i = 0; i != iterations; ++i) {
        int value = ++producer_count;
        while (!queue.push(value))
            ;
    }
}

boost::atomic<bool> done (false);
boost::atomic_int consumerNotDoneCount(0);
void consumer(void)
{
    int value;
    while (!done) {
        while (queue.pop(value))
            ++consumer_count;
	consumerNotDoneCount++;
    }
std::cout << "outside of consumer while " << consumer_count << "\n" ;
    while (queue.pop(value)) {
std::cout << "inside second loop\n"; 
     ++consumer_count;
}
}


/***************************************************************
here for execution of code in standalone, will be removed once 
integrated in project
***************************************************************/
int main (int argc, char* argv[]) {
/*
using namespace std;
    cout << "boost::lockfree::queue is ";
    if (!queue.is_lock_free())
        cout << "not ";
    cout << "lockfree" << endl;

    boost::thread_group producer_threads, consumer_threads;

    for (int i = 0; i != producer_thread_count; ++i)
        producer_threads.create_thread(producer);

    for (int i = 0; i != consumer_thread_count; ++i)
        consumer_threads.create_thread(consumer);

    producer_threads.join_all();
    done = true;

    consumer_threads.join_all();

    cout << "produced " << producer_count << " objects." << endl;
    cout << "consumed " << consumer_count << " objects." << endl;
    cout << "consumer count " << consumerNotDoneCount << "\n";
*/

    //boost::lockfree::queue<std::string>* a = new boost::lockfree::queue<std::string>(100);
//    boost::lockfree::spsc_queue<std::string>* q;
//    boost::lockfree::spsc_queue<std::string>* a = new boost::lockfree::spsc_queue<std::string>(200);

//    std::string t = "hello";
//    a->push(t);
//    if(!a->empty())
//        std::cout << "a not empty\n";
//    q = a;
//    if(q->pop(t))
//        std::cout << "q had " << t << "\n";
//    if(a->empty()){
//        std::cout << "a is empty\n";
//    }


//    std::string x = "sdfds";
//    a.push(x);
//    x = "hey";
//    q = &a;
//    if(!a.empty())
//        std::cout << "a is not empty\n";
//    if(q->pop(x))
//    std::cout << "top of q is  " <<x << std::endl;
//    if(a.empty())
//        std::cout << "a is empty\n";

//    if(a.pop(x))
//        std::cout << x << std::endl;


    //a->push("hello");

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
    return 0;


}

