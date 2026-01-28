#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>
#include <opencv2/opencv.hpp>
#include <chrono>

namespace video_texture_plugin {
class VideoTexturePlugin : public gz::sim::System, 
                            public gz::sim::ISystemConfigure, 
                            public gz::sim::ISystemPostUpdate {
    cv::VideoCapture cap;
    gz::transport::Node node;
    gz::transport::Node::Publisher img_pub;
    std::string topicName = "video_stream";
    double fps{30.0};
    std::chrono::steady_clock::time_point lastFrameTime;

public:
    void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &_sdf,
                  gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
        
        std::string videoPath = _sdf->Get<std::string>("video_path", "").first;
        this->fps = _sdf->Get<double>("fps", 30.0).first;

        this->cap.open(videoPath);
        this->img_pub = this->node.Advertise<gz::msgs::Image>(this->topicName);
        this->lastFrameTime = std::chrono::steady_clock::now();
        
        if (this->cap.isOpened()) {
            std::cout << "[VideoPlugin] Lecture video OK : " << videoPath << std::endl;
        } else {
            std::cerr << "[VideoPlugin] ERREUR : Impossible de lire " << videoPath << std::endl;
        }
    }

    void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &) override {
        if (_info.paused) return;

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - this->lastFrameTime).count() < (1.0 / this->fps)) return;
        this->lastFrameTime = now;

        cv::Mat frame;
        if (this->cap.read(frame)) {
            gz::msgs::Image msg;
            msg.set_width(frame.cols);
            msg.set_height(frame.rows);
            msg.set_step(frame.cols * 3);
            msg.set_pixel_format_type(gz::msgs::PixelFormatType::RGB_INT8);
            
            cv::Mat frameRGB;
            cv::cvtColor(frame, frameRGB, cv::COLOR_BGR2RGB);
            msg.set_data(frameRGB.data, frameRGB.rows * frameRGB.cols * 3);
            
            this->img_pub.Publish(msg);
        } else {
            this->cap.set(cv::CAP_PROP_POS_FRAMES, 0); // Loop
        }
    }
};
}

GZ_ADD_PLUGIN(video_texture_plugin::VideoTexturePlugin, gz::sim::System, 
              video_texture_plugin::VideoTexturePlugin::ISystemConfigure, 
              video_texture_plugin::VideoTexturePlugin::ISystemPostUpdate)