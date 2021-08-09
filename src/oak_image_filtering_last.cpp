#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


const int BUFFER_SIZE = 1;


class ImageFiltering
{

private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    image_transport::CameraSubscriber color_sub_;
    image_transport::CameraSubscriber depth_sub_;

    image_transport::CameraPublisher color_pub_;
    image_transport::CameraPublisher depth_pub_;

    cv::Mat new_color_image_;
    cv::Mat new_depth_image_;

    sensor_msgs::CameraInfo new_color_info_;
    sensor_msgs::CameraInfo new_depth_info_;

    cv::Mat buffer_pixel_[BUFFER_SIZE];
    cv::Mat buffer_index_[BUFFER_SIZE];

    bool reading_, first_;


public:

    bool color_ready_, depth_ready_;

    ImageFiltering(): it_(nh_)
    {
        first_ = true;
        reading_ = true;

        color_sub_ = it_.subscribeCamera("/stereo_rgb_node/color/image", 10 + BUFFER_SIZE, &ImageFiltering::colorCallback, this);
        depth_sub_ = it_.subscribeCamera("/stereo_rgb_node/stereo/depth", 10 + BUFFER_SIZE, &ImageFiltering::depthCallback, this);

        color_pub_ = it_.advertiseCamera("/stereo_rgb_node/filtered/color/image", 10 + BUFFER_SIZE);
        depth_pub_ = it_.advertiseCamera("/stereo_rgb_node/filtered/depth/image", 10 + BUFFER_SIZE);

        //cv::Mat *buffer_pixel_ = new cv::Mat[BUFFER_SIZE];
        //cv::Mat *buffer_index_ = new cv::Mat[BUFFER_SIZE];
    }


    void colorCallback(const sensor_msgs::ImageConstPtr& image,
                       const sensor_msgs::CameraInfoConstPtr& info)
    {
        if (reading_)
        {
            new_color_image_ = (cv_bridge::toCvShare(image)->image).clone();
            new_color_info_ = *info;
        }
        color_ready_ = true;
    }


    void depthCallback(const sensor_msgs::ImageConstPtr& image,
                       const sensor_msgs::CameraInfoConstPtr& info)
    {
        if (reading_)
        {
            new_depth_image_ = (cv_bridge::toCvShare(image)->image).clone();
            new_depth_info_ = *info;
        }
        depth_ready_ = true;
    }


    void mainLoop(int mask_median, int mask_mean, double image_scale, double min_range, double max_range, int count, int ho, int hf, int wo, int wf)
    {
        struct timeval to, tf;
        gettimeofday(&to, NULL);

        reading_ = false;

        cv::Mat input_color_image = new_color_image_;
        cv::Mat input_depth_image = new_depth_image_;

        sensor_msgs::CameraInfo input_color_info = new_color_info_;
        sensor_msgs::CameraInfo input_depth_info = new_depth_info_;

        reading_ = true;

        cv::Mat resized_color;
        cv::resize(input_color_image, resized_color, cv::Size(), image_scale, image_scale, cv::INTER_NEAREST);

        cv::Mat resized_depth = matchDepthToColor(input_depth_image, image_scale, ho, hf, wo, wf);
        //cv::resize(input_depth_image, resized_depth, cv::Size(), image_scale, image_scale, cv::INTER_NEAREST);

        //cv::Mat byte_depth = convertFloatToByte(resized_depth, min_range, max_range);

        
        if (first_)
        {
            first_ = false;

            for (int k = 0; k < BUFFER_SIZE; k++)
            {
                buffer_pixel_[k] = resized_depth.clone();
                buffer_index_[k] = cv::Mat(resized_depth.rows, resized_depth.cols, CV_8UC1, k);
            }
        }
        
        cv::Mat sequenced_image = medianSequence(resized_depth);
        

        cv::Mat median_depth; cv::medianBlur(sequenced_image, median_depth, mask_median);
        cv::medianBlur(median_depth, median_depth, mask_median);
        cv::medianBlur(median_depth, median_depth, mask_median);
        cv::medianBlur(median_depth, median_depth, mask_median);

        cv::Mat mean_depth = nonZeroMean(median_depth, mask_mean);

        //cv::Mat float_depth = convertByteToFloat(mean_depth, min_range, max_range);

        std_msgs::Header header;
        header.seq = count;
        header.stamp = ros::Time::now();

        header.frame_id = "oak_filtered_frame";

        sensor_msgs::Image output_color_image;
        cv_bridge::CvImage(header, "bgr8", resized_color).toImageMsg(output_color_image);

        sensor_msgs::Image output_depth_image;
        cv_bridge::CvImage(header, "16UC1", mean_depth).toImageMsg(output_depth_image);

        sensor_msgs::CameraInfo output_color_info = updateInfo(input_color_info, image_scale, count);
        sensor_msgs::CameraInfo output_depth_info = updateInfo(input_depth_info, image_scale, count);

        color_pub_.publish(output_color_image, output_color_info);
        depth_pub_.publish(output_depth_image, output_depth_info);

        gettimeofday(&tf, NULL);
        int t = (1000*tf.tv_sec + tf.tv_usec/1000) - (1000*to.tv_sec + to.tv_usec/1000);
        //ROS_INFO("processing time: %d ms", t);
    }


    cv::Mat matchDepthToColor(cv::Mat unmatched_image, double image_scale, int ho, int hf, int wo, int wf)
    {
        int h = unmatched_image.rows;
        int w = unmatched_image.cols;

        hf = h - hf - ho;
        wf = w - wf - wo;
        //ROS_INFO("%d %d %d %d", ho, hf, wo, wf);

        cv::Mat cropped_depth = unmatched_image(cv::Rect(wo, ho, wf, hf));

        cv::Mat matched_image;
        cv::resize(cropped_depth, matched_image, cv::Size(image_scale * w, image_scale * h), 0, 0, cv::INTER_NEAREST);

        return matched_image;
    }


    cv::Mat nonZeroMean(cv::Mat raw_image, int mask_size)
    {
        int h = raw_image.rows;
        int w = raw_image.cols;
        int numel = h * w;

        cv::Mat mean_image = raw_image.clone();

        ushort* raw_ptr = (ushort*)raw_image.data;
        ushort* mean_ptr = (ushort*)mean_image.data;

        int mo = mask_size / 2;
        int mf = (mask_size - 1) / 2 + 1;
        int hf = h - mo;
        int wf = w - mo;

        for (int i = mf; i < hf; i++)
        {
            for (int j = mf; j < wf; j++)
            {
                int num = 0;
                int sum = 0;

                int k = i * w + j;

                int px_min = raw_ptr[k] - raw_ptr[k] / 10;
                int px_max = raw_ptr[k] + raw_ptr[k] / 10;
                //ROS_INFO("i:%d, j:%d, raw:%d", i, j, raw_ptr[k]);

                for (int x = -mf; x < mo; x++)
                {
                    for (int y = -mf; y < mo; y++)
                    {
                        //ROS_INFO("x:%d, y:%d, raw:%d", x, y, raw_ptr[k + x * w + y]);
                        int px = raw_ptr[k + x * w + y];

                        if (px > px_min && px < px_max)
                        {
                            sum += px;
                            num++;
                            //ROS_INFO("sum:%4d, num:%d", sum, num);
                        }
                    }
                }

                mean_ptr[k] = num > 0 ? sum / num : 0;
                //ROS_INFO("i:%d, j:%d, mean:%d", i, j, mean_ptr[i * w + j]);
            }
        }

        return mean_image;
    }


    cv::Mat convertFloatToByte(cv::Mat float_image, double min_range, double max_range)
    {
        int h = float_image.rows;
        int w = float_image.cols;
        int numel = h * w;

        double prec = (max_range - min_range) / 255.0;

        cv::Mat byte_image = cv::Mat(h, w, CV_8UC1);

        float* float_ptr = (float*)float_image.data;
        uint8_t* byte_ptr = byte_image.data;

        for (int n = 0; n < numel; n++)
        {
            int value = (int)round((float_ptr[n] - min_range) / prec);
            byte_ptr[n] = value > 0 && value < 255 ? value : 0;
            //ROS_INFO("%f %d %d", float_ptr[n], value, byte_ptr[n]);
        }

        return byte_image;
    }


    cv::Mat convertByteToFloat(cv::Mat byte_image, double min_range, double max_range)
    {
        int height = byte_image.rows;
        int width = byte_image.cols;
        int numel = height * width;

        double prec = (max_range - min_range) / 255.0;

        cv::Mat float_image = cv::Mat(height, width, CV_32FC1);

        uint8_t* byte_ptr = byte_image.data;
        float* float_ptr = (float*)float_image.data;

        for (int n = 0; n < numel; n++)
            float_ptr[n] = byte_ptr[n] > 0 ? prec * byte_ptr[n] + min_range : 0.0;

        return float_image;
    }


    cv::Mat medianSequence(cv::Mat image)
    {
        int height = image.rows;
        int width = image.cols;
        int numel = height * width;

        ushort* image_ptr = (ushort*)image.data;
        ushort* pixel_ptr[BUFFER_SIZE];
        uint8_t* index_ptr[BUFFER_SIZE];

        for (int k = 0; k < BUFFER_SIZE; k++)
        {
            pixel_ptr[k] = (ushort*)(buffer_pixel_[k].data);
            index_ptr[k] = buffer_index_[k].data;
        }

        for (int n = 0; n < numel; n++)
        {
            int idx_old = 0;
            int idx_new = 0;

            //if(n==100000){ROS_INFO("\n\n");}
            //if(n==100000){ROS_INFO("n=%d img=%d", n, image_ptr[n]);}

            for (int k = 0; k < BUFFER_SIZE; k++)
            {
                //if(n==100000){ROS_INFO("%d", image_ptr[n]);}
                //if(n==100000){ROS_INFO("%d", pixel_ptr[k][n]);}
                if (pixel_ptr[k][n] < image_ptr[n])
                    idx_new = k + 1;

                if (index_ptr[k][n] == 0)
                    idx_old = k;

                //if(n==100000){ROS_INFO("idx=%d pix=%d", index_ptr[k][n], pixel_ptr[k][n]);}

                index_ptr[k][n]--;
            }

            //if(n==100000){ROS_INFO("old=%d new=%d", idx_old, idx_new);}

            if (idx_old < idx_new)
            {
                idx_new--;
                for (int i = idx_old; i < idx_new - 1; i++)
                    pixel_ptr[i][n] = pixel_ptr[i + 1][n];
            }
            else
            {
                for (int i = idx_old; i >= idx_new + 1; i--)
                    pixel_ptr[i][n] = pixel_ptr[i - 1][n];
            }

            //if(n==100000){ROS_INFO("\n");}
            //for (int k = 0; k < BUFFER_SIZE; k++)
                //if(n==100000){ROS_INFO("pix=%d idx=%d", pixel_ptr[k][n], index_ptr[k][n]);}

            pixel_ptr[idx_new][n] = image_ptr[n];
            index_ptr[idx_old][n] = BUFFER_SIZE - 1;

            //if(n==100000){ROS_INFO("\n");}
            //for (int k = 0; k < BUFFER_SIZE; k++)
                //if(n==100000){ROS_INFO("pix=%d idx=%d", pixel_ptr[k][n], index_ptr[k][n]);}
        }

        return buffer_pixel_[BUFFER_SIZE / 2];
    }


    sensor_msgs::CameraInfo updateInfo(sensor_msgs::CameraInfo camera_info, float image_scale, int count)
    {
        sensor_msgs::CameraInfo new_info;

        new_info.header.seq = count;
        new_info.header.stamp = ros::Time::now();
        new_info.header.frame_id = camera_info.header.frame_id;

        new_info.height = camera_info.height * image_scale;
        new_info.width = camera_info.width * image_scale;

        new_info.distortion_model = camera_info.distortion_model;

        for (int k = 0; k < 8; k++)
            new_info.D.push_back(camera_info.D[k]);

        for (int k = 0; k < 9; k++)
            new_info.K[k] = camera_info.K[k] * image_scale;

        new_info.R = camera_info.R;

        for (int k = 0; k < 12; k++)
            new_info.P[k] = camera_info.P[k] * image_scale;

        return new_info;
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "oak_image_filtering_node");

    ros::NodeHandle nh("~");

    int mask_median; nh.param("mask_median", mask_median, 13);
    int mask_mean; nh.param("mask_mean", mask_mean, 1);
    int publish_rate; nh.param("publish_rate", publish_rate, 10);
    double image_scale; nh.param("image_scale", image_scale, 0.5);
    double min_range; nh.param("min_range", min_range, 0.5);
    double max_range; nh.param("max_range", max_range, 8.0);

    int ho; nh.param("ho", ho, 60);
    int hf; nh.param("hf", hf, 40);
    int wo; nh.param("wo", wo, 70);
    int wf; nh.param("wf", wf, 100);

    ImageFiltering filter;

    filter.color_ready_ = false;
    filter.depth_ready_ = false;
    while ((!filter.color_ready_ || !filter.depth_ready_) && ros::ok())
        ros::spinOnce();

    int count = 0;

    ros::Rate rate(publish_rate);
    while (ros::ok())
    {
        filter.mainLoop(mask_median, mask_mean, image_scale, min_range, max_range, count++, ho, hf, wo, wf);
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();

    return 0;
}
