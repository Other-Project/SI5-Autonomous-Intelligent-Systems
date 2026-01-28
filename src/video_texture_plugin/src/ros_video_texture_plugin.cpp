#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/Material.hh>

#include <gz/common/Console.hh>
#include <gz/common/Image.hh>

#include <opencv2/opencv.hpp>

#include <string>
#include <chrono>

namespace video_texture_plugin
{
class VideoTexturePlugin :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
private:
    std::string videoPath;
    std::string visualName;
    double fps{30.0};
    bool debug{false};

    gz::rendering::ScenePtr scene;
    gz::rendering::VisualPtr visual;
    gz::rendering::MaterialPtr material;

    cv::VideoCapture cap;
    cv::Mat frameRGB;

    bool initialized{false};
    std::chrono::steady_clock::time_point lastFrameTime;
    std::string tempTexturePath{"/tmp/gz_video_frame.png"};

public:
    void Configure(
        const gz::sim::Entity &,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &,
        gz::sim::EventManager &) override
    {
        this->videoPath = _sdf->Get<std::string>("video_path", "").first;
        this->visualName = _sdf->Get<std::string>("visual_name", "").first;
        this->fps = _sdf->Get<double>("fps", 30.0).first;
        this->debug = _sdf->Get<bool>("debug", false).first;

        if (this->videoPath.empty())
        {
            gzerr << "[VideoTexturePlugin] video_path missing\n";
            return;
        }

        this->cap.open(this->videoPath);
        if (!this->cap.isOpened())
        {
            gzerr << "[VideoTexturePlugin] Failed to open video: " << this->videoPath << "\n";
            return;
        }

        this->lastFrameTime = std::chrono::steady_clock::now();
        gzmsg << "[VideoTexturePlugin] Video opened: " << this->videoPath << "\n";
    }

    void PostUpdate(
        const gz::sim::UpdateInfo &,
        const gz::sim::EntityComponentManager &) override
    {
        if (!this->initialized)
        {
            // Récupération de la scène
            this->scene = gz::rendering::sceneFromFirstRenderEngine();
            if (!this->scene)
            {
                gzerr << "[VideoTexturePlugin] No rendering scene found\n";
                return;
            }

            // Recherche du visual par son nom
            this->visual = this->scene->VisualByName(this->visualName);
            if (!this->visual)
            {
                gzerr << "[VideoTexturePlugin] Visual not found: " << this->visualName << "\n";
                return;
            }

            // Récupérer ou créer le matériau
            this->material = this->visual->Material();
            if (!this->material)
            {
                this->material = this->scene->CreateMaterial();
                this->visual->SetMaterial(this->material);
            }

            this->material->SetLightingEnabled(false);
            this->material->SetAmbient(1, 1, 1, 1);
            this->material->SetDiffuse(1, 1, 1, 1);

            this->initialized = true;
            gzmsg << "[VideoTexturePlugin] Rendering ready for visual: " << this->visualName << "\n";
        }

        // Limiter le FPS
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - this->lastFrameTime).count() < 1.0 / this->fps)
            return;

        this->lastFrameTime = now;

        // Lire la frame suivante
        cv::Mat frameBGR;
        if (!this->cap.read(frameBGR))
        {
            if (this->debug) gzmsg << "[VideoTexturePlugin] Video ended, looping\n";
            this->cap.set(cv::CAP_PROP_POS_FRAMES, 0);
            return;
        }

        cv::cvtColor(frameBGR, this->frameRGB, cv::COLOR_BGR2RGB);
        this->UpdateTexture();
    }

    void UpdateTexture()
    {
        if (this->frameRGB.empty())
            return;

        // Convertir la frame en gz::common::Image
        gz::common::Image img;
        img.SetFromData(
            this->frameRGB.data,
            this->frameRGB.cols,
            this->frameRGB.rows,
            gz::common::Image::RGB_INT8);

        // Sauvegarder temporairement
        img.SavePNG(this->tempTexturePath);

        // Appliquer sur le matériau
        this->material->SetTexture(this->tempTexturePath);

        if (this->debug)
            gzmsg << "[VideoTexturePlugin] Texture updated on visual: " << this->visualName << "\n";
    }
};
} // namespace video_texture_plugin

GZ_ADD_PLUGIN(
    video_texture_plugin::VideoTexturePlugin,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPostUpdate)
