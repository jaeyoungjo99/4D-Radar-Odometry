#ifndef __ATOM_TASK_H__
#define __ATOM_TASK_H__
#pragma once
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <mutex>
#include <std_msgs/Float64.h>

class AtomTask{
    protected:
        AtomTask(int id, std::string task_name, double period);
        ~AtomTask();

    public:
        void Exec(int num_thread=4);
        void MainLoop();

    protected:
        virtual void Init()=0;
        virtual void Run()=0;
        virtual void Publish()=0;
        virtual void Terminate()=0;

    private:
        ros::Publisher pub_life_;
        void PublishLifeSignal();

    protected:
        std::string	  task_name_;
        int           task_id_;
        double	      task_period_;   
        double	      task_rate_;
        ros::Time	  update_time_;
        ros::Duration duration_time_;
        ros::Duration execution_time_;
};

AtomTask::AtomTask(int id, std::string task_name, double period)
:task_name_(""),
task_period_(0.0),
task_rate_(0.0)
{
    task_name_ = task_name;
    task_id_ = id;
    task_period_ = period;
    task_rate_ = 1.0/period;

    update_time_ = ros::Time::now();
    execution_time_ = ros::Duration(0.0);

    ros::NodeHandle nh;
    pub_life_ = nh.advertise<std_msgs::Float64>("task_" + task_name_, 100);
}

AtomTask::~AtomTask(){
}

void AtomTask::Exec(int num_thread){
    boost::thread main_thread( boost::bind(  &AtomTask::MainLoop, this ) );

    ros::AsyncSpinner spinner(num_thread);
    spinner.start();
    ros::waitForShutdown();

    main_thread.join();
}

void AtomTask::MainLoop(){
    ros::Rate loop_rate(task_rate_);
    ROS_WARN_STREAM("[" << task_name_ << "] Initialize node (ID: " 
                            << task_id_ << ", Period: " << task_period_ << "s)");

    Init();
    ros::Time last_log_time = ros::Time::now();
    while( ros::ok() ){
        // PublishLifeSignal();

        update_time_ = ros::Time::now();
        
        // Run algorithm
        Run();

        execution_time_ = ros::Time::now() - update_time_;

        if ((ros::Time::now() - last_log_time).toSec() >= 1.0) {
            if (execution_time_.toSec() > task_period_) {
                ROS_ERROR_STREAM("[" << task_name_ << "] Rate: " << task_period_*1000.0 << 
                                 "ms, Exec Time:" << (execution_time_).toSec()*1000.0 << "ms");
            }
            else {
                // ROS_INFO_STREAM("[" << task_name_ << "] Rate: " << task_period_*1000.0 << 
                //                 "ms, Exec Time:" << (execution_time_).toSec()*1000.0 << "ms");
            }
            last_log_time = ros::Time::now();
        }

        // Publish topics
        Publish();    

        ros::spinOnce();
        loop_rate.sleep();
    }
    Terminate();
}

void AtomTask::PublishLifeSignal(){
    std_msgs::Float64 time;
    time.data = update_time_.toSec();
    pub_life_.publish( time  );
}

#endif // __ATOM_TASK_H__