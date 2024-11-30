#include <QPainter>
#include <QFont>
#include <QGuiApplication>
#include <QScreen>
#include <QTime>

#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <OgreTextureManager.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <rviz_rendering/render_system.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>

#include <OGRE/Overlay/OgreOverlay.h>
#include <OGRE/Overlay/OgreOverlayManager.h>
#include <OGRE/Overlay/OgreOverlayContainer.h>

namespace rviz_overlay_plugin
{
    class OverlayDisplay : public rviz_common::Display
    {
        Q_OBJECT
    public:
        OverlayDisplay()
            : timer_value_(0.0), update_required_(false), gear_index_(-1), percentage_(0)
        {
            // Get screen size to set width and height relative to the screen
            QScreen *screen = QGuiApplication::primaryScreen();
            QRect screenGeometry = screen->geometry();

            width_ = screenGeometry.width() / 3;    // Set width to 1/3 of the screen width
            height_ = screenGeometry.height() - 10; // Set height to 1/4 of the screen height for a smaller box on top

            left_ = screenGeometry.width() - width_; // Position at the top-right corner
            top_ = 0;                                // Align to the top of the screen

            // Define properties for other settings
            left_property_ = new rviz_common::properties::IntProperty("Left", left_, "Left position of the overlay window.", this, SLOT(updateLeft()));
            top_property_ = new rviz_common::properties::IntProperty("Top", top_, "Top position of the overlay window.", this, SLOT(updateTop()));
            fg_color_property_ = new rviz_common::properties::ColorProperty("Foreground Color", QColor(25, 255, 240), "Color to draw the timer.", this, SLOT(updateFGColor()));
            fg_alpha_property_ = new rviz_common::properties::FloatProperty("Foreground Alpha", 0.7, "Alpha value for foreground.", this, SLOT(updateFGAlpha()));

            // Background color and opacity properties
            bg_color_ = QColor("#0b00b0c"); // Set background color using hex string
            bg_alpha_ = 0.2 * 255;          // Set alpha to 50% of 255 (which is 127)
            fg_alpha_ = 0.7 * 255;          // Initialize foreground alpha to 70%

            // Add a string property to set the topic name
            topic_property_ = new rviz_common::properties::StringProperty("Topic", "/timer", "The topic to subscribe to.", this, SLOT(updateTopic()));
        }

        ~OverlayDisplay()
        {
            // Clean up the texture and material if they were created
            if (texture_)
            {
                Ogre::TextureManager::getSingleton().remove(texture_->getName());
            }
            if (material_)
            {
                Ogre::MaterialManager::getSingleton().remove(material_->getName());
            }
        }

    protected:
        void onInitialize() override
        {
            rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);

            // Create the texture and material for the overlay
            std::stringstream ss;
            ss << "OverlayTexture" << this;
            texture_ = Ogre::TextureManager::getSingleton().createManual(
                ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                Ogre::TEX_TYPE_2D, width_, height_, 0, Ogre::PF_BYTE_BGRA,
                Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

            ss << "Material";
            material_ = Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
            material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_->getName());
            material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

            material_->setDepthCheckEnabled(false);
            material_->setDepthWriteEnabled(false);
            material_->setLightingEnabled(false);

            // Set up overlay
            ss.str("");
            ss << "Overlay" << this;
            overlay_ = Ogre::OverlayManager::getSingleton().create(ss.str());

            ss.str("");
            ss << "OverlayPanel" << this;
            panel_ = static_cast<Ogre::OverlayContainer *>(
                Ogre::OverlayManager::getSingleton().createOverlayElement("Panel", ss.str()));
            panel_->setMetricsMode(Ogre::GMM_PIXELS);
            panel_->setPosition(left_, top_);
            panel_->setDimensions(width_, height_);
            panel_->setMaterialName(material_->getName());

            overlay_->add2D(panel_);
            overlay_->show();

            onEnable();
        }

        virtual void onEnable() override
        {
            updateTopic(); // Subscribe to the topic if it's set

            // Subscribe to the gear topic to display the gear index
            gear_subscription_ = context_->getRosNodeAbstraction().lock()->get_raw_node()->create_subscription<std_msgs::msg::Int32>(
                "/gear_selection", rclcpp::SystemDefaultsQoS(),
                [this](const std_msgs::msg::Int32::SharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    gear_index_ = msg->data; // Convert 1-based index to 0-based for QStringList
                    update_required_ = true;
                });

            percentage_subscription_ = context_->getRosNodeAbstraction().lock()->get_raw_node()->create_subscription<std_msgs::msg::Int32>(
                "/percentage", rclcpp::SystemDefaultsQoS(),
                [this](const std_msgs::msg::Int32::SharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    percentage_ = std::clamp(msg->data, 0, 100); // Ensure percentage stays between 0 and 100
                    update_required_ = true;
                });

            update_required_ = true;
        }

        virtual void onDisable() override
        {
            subscription_.reset(); // Unsubscribe when the display is disabled
            gear_subscription_.reset();
            percentage_subscription_.reset();
        }

        // Process messages received from the topic
        void processMessage(const std_msgs::msg::Float32::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            timer_value_ = msg->data;
            update_required_ = true;
        }

        // Update the overlay display every frame
        virtual void update(float /*wall_dt*/, float /*ros_dt*/) override
        {
            if (update_required_)
            {
                update_required_ = false;
                drawOverlay(timer_value_);
            }
        }

        // Draw the overlay on the texture
        void drawOverlay(float value)
        {
            Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();
            pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
            const Ogre::PixelBox &pb = pixelBuffer->getCurrentLock();

            QImage overlayImage(static_cast<uchar *>(pb.data), width_, height_, QImage::Format_ARGB32);
            overlayImage.fill(Qt::transparent);
            QPainter painter(&overlayImage);

            painter.setRenderHint(QPainter::Antialiasing);

            // Set up the background color with transparency
            QColor bg_color_with_alpha = bg_color_;
            bg_color_with_alpha.setAlpha(bg_alpha_); // Set alpha for 50% opacity

            // Remove any pen to ensure no border is drawn
            painter.setPen(Qt::NoPen);

            // Draw the rounded rectangle background without border
            painter.setBrush(bg_color_with_alpha);
            painter.drawRoundedRect(10, 10, width_ - 20, height_ - 20, 30, 30); // Draw rounded rectangle with radius of 30

            // Draw the numeric value larger and in the center
            QFont font = painter.font();
            font.setPointSize(200); // Further increased font size for the number to make it more prominent
            font.setBold(true);     // Make the font bold
            painter.setFont(font);
            painter.setPen(Qt::white); // Set text color to white for good contrast

            // Draw the speed value at the center
            QRect valueRect(width_ / 2 - 150, height_ / 2 - 300, 300, 250); // Increased the size of the rectangle to make the number larger and centered properly
            painter.drawText(valueRect, Qt::AlignCenter, QString::number(value, 'f', 0));

            // Draw "km/h" text below the number, with smaller size and some spacing
            font.setPointSize(20); // Set larger font size for the "km/h" text for better visibility
            font.setBold(false);
            painter.setFont(font);
            QRect unitRect(width_ / 2 - 50, height_ / 2, 100, 50); // Position "km/h" text below the number with enough spacing
            painter.drawText(unitRect, Qt::AlignCenter, "km/h");

            // Draw the labels ("P", "R", "D", "N") below the circle
            QStringList labels = {"P", "R", "D", "N"};
            font.setPointSize(30); // Set slightly larger font size for the labels for better readability
            font.setBold(false);
            painter.setFont(font);
            painter.setPen(Qt::white); // Set text color to gray for labels

            int labelY = height_ - 320;                                       // Y position below the circle
            int labelSpacing = width_ / 5;                                    // Space between each label (one-fifth of the width)
            int startX = (width_ - (labelSpacing * (labels.size() - 1))) / 2; // Calculate starting X position for evenly spaced labels

            for (int i = 0; i < labels.size(); ++i)
            {
                QRect labelRect(startX + i * labelSpacing - 20, labelY - 20, 40, 40);

                // If the label is "D", draw a yellow rounded rectangle behind it
                if (i == gear_index_)
                {
                    QColor highlightColor = QColor(Qt::yellow);
                    highlightColor.setAlpha(0.6 * 255); // 50% opacity (127 out of 255)
                    painter.setBrush(highlightColor);
                    painter.setPen(Qt::NoPen); // No border for the rounded rectangle
                    painter.drawRoundedRect(labelRect, 11, 11);
                }

                // Draw the label text
                painter.setPen(Qt::white); // Set text color to gray for labels
                painter.drawText(labelRect, Qt::AlignCenter, labels[i]);
            }

            // Draw the progress bar below the labels
            int bar_margin = 100; // Margin for the progress bar
            int bar_width = width_ - 2 * bar_margin;
            int bar_height = 30;                                // Height of the progress bar
            int bar_y = labelY + 100;                           // Position below the labels
            int filled_width = (percentage_ * bar_width) / 100; // Calculate filled width based on percentage_

            QRect bar_background(bar_margin, bar_y, bar_width, bar_height);
            QRect bar_filled(bar_margin, bar_y, filled_width, bar_height);

            int corner_radius = 10; // Corner radius for rounded rectangles

            // Draw the progress bar border with rounded corners
            QColor border_color = QColor(0, 0, 0, 255);                            // Black border with full opacity
            painter.setPen(QPen(border_color, 1));                                 // Set border color and thickness
            painter.setBrush(Qt::NoBrush);                                         // No fill for the border
            painter.drawRoundedRect(bar_background, corner_radius, corner_radius); // Rounded border

            // Draw the progress bar background with rounded corners
            QColor bar_bg_color = QColor(200, 200, 200, 50); // Light gray with some transparency
            painter.setBrush(bar_bg_color);
            painter.setPen(Qt::NoPen);                                             // No border for the background
            painter.drawRoundedRect(bar_background, corner_radius, corner_radius); // Rounded background

            // Draw the filled part of the progress bar with rounded corners
            QColor bar_fill_color = QColor(230, 230, 230, 80); // Green
            painter.setBrush(bar_fill_color);
            QRect filled_rect(bar_margin, bar_y, filled_width, bar_height);

            // Ensure the filled part has rounded corners only if it fills the entire width
            if (filled_width < bar_width)
            {
                // Filled part without rounded corners on the right side
                painter.drawRect(filled_rect);
            }
            else
            {
                // Fully filled bar with rounded corners
                painter.drawRoundedRect(filled_rect, corner_radius, corner_radius);
            }

            // Draw the percentage text on the progress bar
            // font.setPointSize(20);
            // font.setBold(true);
            // painter.setFont(font);
            // painter.setPen(Qt::black);
            // painter.drawText(bar_background, Qt::AlignCenter, QString("%1%").arg(percentage_));

            // Draw the current time to the left of the green circle
            QTime currentTime = QTime::currentTime();
            QString timeText = currentTime.toString("hh:mm"); // Format time as HH:MM

            QFont timeFont = painter.font();
            timeFont.setPointSize(18); // Adjust the font size for better readability
            timeFont.setBold(false);
            painter.setFont(timeFont);
            painter.setPen(Qt::white); // Set text color to white for good contrast

            // Adjust position for time text to be left of the circle
            int timeTextX = width_ - 80;                            // Position slightly left of the green circle
            int timeTextY = 35;                                     // Align vertically with the circle
            QRect timeRect(timeTextX - 80, timeTextY - 15, 80, 30); // Adjust position and size for text
            painter.drawText(timeRect, Qt::AlignRight, timeText);

            // Add a little green circle at the top-left corner
            int circle_radius = 10;                     // Radius of the circle
            int circle_x = width_ - circle_radius - 30; // Fixed margin for the circle (distance from left)
            int circle_y = 30;                          // Fixed margin for the circle (distance from top)

            painter.setBrush(QColor(50, 200, 50, 255)); // Green color for the circle
            painter.setPen(Qt::NoPen);                  // No border for the circle
            painter.drawEllipse(QPoint(circle_x, circle_y), circle_radius, circle_radius);

            painter.end();
            pixelBuffer->unlock();
        }

    protected Q_SLOTS:
        void updateTopic()
        {
            std::string topic_name = topic_property_->getStdString();
            if (topic_name.empty())
            {
                setStatus(rviz_common::properties::StatusProperty::Error, "Topic", "Error subscribing: Empty topic name");
                return;
            }

            auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
            if (!ros_node_abstraction)
            {
                setStatus(rviz_common::properties::StatusProperty::Error, "Topic", "Error subscribing: ROS node unavailable");
                return;
            }

            auto ros_node = ros_node_abstraction->get_raw_node();
            setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "Subscribed successfully.");

            // Create a new subscription to the topic
            subscription_ = ros_node->create_subscription<std_msgs::msg::Float32>(
                topic_name, rclcpp::SystemDefaultsQoS(),
                std::bind(&OverlayDisplay::processMessage, this, std::placeholders::_1));
        }

        void updateTop()
        {
            top_ = top_property_->getInt();
            update_required_ = true;
        }

        void updateLeft()
        {
            left_ = left_property_->getInt();
            update_required_ = true;
        }

        void updateFGColor()
        {
            fg_color_ = fg_color_property_->getColor();
            update_required_ = true;
        }

        void updateFGAlpha()
        {
            fg_alpha_ = fg_alpha_property_->getFloat() * 255.0;
            update_required_ = true;
        }

    private:
        // RViz properties for display customization
        rviz_common::properties::IntProperty *left_property_;
        rviz_common::properties::IntProperty *top_property_;
        rviz_common::properties::ColorProperty *fg_color_property_;
        rviz_common::properties::FloatProperty *fg_alpha_property_;
        rviz_common::properties::StringProperty *topic_property_;

        int width_;
        int height_;
        int left_;    // Set dynamically to align to the right edge of the screen
        int top_ = 0; // Set to 0 to align to the top edge of the screen

        QColor fg_color_ = QColor(25, 255, 240);
        int fg_alpha_ = 178; // Initialized to 70% opacity (255 * 0.7)
        QColor bg_color_ = QColor("#0b0b0c");
        int bg_alpha_ = 127; // 50% opacity

        float timer_value_;
        bool update_required_;

        Ogre::TexturePtr texture_;
        Ogre::MaterialPtr material_;
        Ogre::Overlay *overlay_;
        Ogre::OverlayContainer *panel_;
        std::mutex mutex_;
        int gear_index_;
        int percentage_; // Stores the percentage value for the progress bar

        // ROS2 subscription
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gear_subscription_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr percentage_subscription_;
    };

} // namespace rviz_overlay_plugin

// Export the plugin so that RViz2 can load it
PLUGINLIB_EXPORT_CLASS(rviz_overlay_plugin::OverlayDisplay, rviz_common::Display)

#include "overlay_display.moc"
