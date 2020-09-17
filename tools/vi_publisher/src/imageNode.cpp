#include <iostream>
#include <mutex>
#include <thread>
#include <list>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include<sensor_msgs/Image.h>

#include<opencv2/opencv.hpp>

using namespace std;

class ImageReader
{
public:
    ImageReader(string filename, bool useCache=false)
    {
        cap.open(filename);
        if(!cap.isOpened())
        {
            throw std::runtime_error("Can not open file "+filename+'\n');
        }
        
        this->useCache = useCache;
        frame_rate = cap.get(cv::CAP_PROP_FPS);
        total_frame = cap.get(cv::CAP_PROP_FRAME_COUNT);
        // encoding = cap.get(cv::CAP_PROP_FOURCC);
        curr_frame = 0;
        cache_thread = 0;

        if(useCache)
        {
            running = true;
            cache_thread = new std::thread(&fillCache,this);
        }
    }

    ~ImageReader()
    {
        if(useCache)
        {
            running = false;
            cache_thread->join();
        }
        if(cap.isOpened())
        {
            cap.release();
        }
    }

    float getFrameRate()
    {
        return frame_rate;
    }

    int getTotalFrame()
    {
        return total_frame;
    }

    bool getFrame(cv::Mat& frame)
    {
        if(useCache)
        {
            std::unique_lock<mutex> lock(mMutexCache);
            if(!Cache.empty())
            {
                frame = Cache.front().clone();
                Cache.pop_front();
            }
            else
            {
                if(curr_frame>=total_frame)
                {
                    return false;
                }
                cap>>frame;
                curr_frame++;
            }
        }
        else
        {
            if(curr_frame>=total_frame)
            {
                return false;
            }
            cap>>frame;
            curr_frame++;
        }

        return !frame.empty();
    }

    static void fillCache(ImageReader* reader)
    {
        while(reader->running && reader->curr_frame<reader->total_frame)
        {
            {
                std::unique_lock<mutex> lock(reader->mMutexCache);
                if(reader->Cache.size()<10)
                {
                    cv::Mat frame;
                    reader-> cap>>frame;
                    reader->Cache.push_back(frame.clone());
                }
            }
            usleep(10);
        }
    }

private:
    cv::VideoCapture cap;

    float frame_rate;
    int curr_frame;
    int total_frame;
    // string encoding;

    bool useCache;
    bool running;
    std::list<cv::Mat> Cache;
    std::mutex mMutexCache;
    std::thread* cache_thread;
};

int main(int argc, char** argv)
{
    if(argc < 2)
    {
        cout<<"usage: image_node video.mp4"<<endl;
        exit(-1);
    }

    ImageReader reader(argv[1],true);
    
    ros::init(argc,argv,"image_node");
    ros::NodeHandle n;

    ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("cam0/image_raw", 10);
    ros::Rate loop_rate(reader.getFrameRate());
    cv::Mat frame;

    while(ros::ok())
    {
        if(reader.getFrame(frame) && !frame.empty())
        {
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            
            cv::Mat frame_resized;
            cv::resize(frame,frame_resized,cv::Size(),0.5,0.5);
            cv_bridge::CvImage img_data(header,"rgb8",frame_resized);
            // cv_bridge::CvImage img_data(header,"rgb8",frame);
            img_pub.publish(img_data.toImageMsg());
            // img_pub.publish(img_data.toCompressedImageMsg());   
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}