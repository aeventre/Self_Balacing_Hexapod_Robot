#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64.hpp"
#include "hexapod_msgs/msg/leg_positions.hpp"
#include "hexapod_msgs/msg/polygon.hpp"

class SupportPolygonCalculator : public rclcpp::Node
{
public:
    SupportPolygonCalculator() : Node("support_polygon_calculator")
    {
        // Subscriptions
        foot_status_sub_ = this->create_subscription<std_msgs::msg::BoolArray>(
            "/foot_status", 10,
            std::bind(&SupportPolygonCalculator::footStatusCallback, this, std::placeholders::_1));

        foot_positions_sub_ = this->create_subscription<hexapod_msgs::msg::LegPositions>(
            "/foot_positions", 10,
            std::bind(&SupportPolygonCalculator::footPositionsCallback, this, std::placeholders::_1));

        com_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/center_of_mass", 10,
            std::bind(&SupportPolygonCalculator::comCallback, this, std::placeholders::_1));

        // Publications
        polygon_pub_ = this->create_publisher<hexapod_msgs::msg::Polygon>("/support_polygon", 10);
        centroid_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/polygon_centroid", 10);
        stability_margin_pub_ = this->create_publisher<std_msgs::msg::Float64>("/stability_margin", 10);

        RCLCPP_INFO(this->get_logger(), "Support Polygon Calculator initialized.");
    }

private:
    // Callback for foot status
    void footStatusCallback(const std_msgs::msg::BoolArray::SharedPtr msg)
    {
        foot_status_ = msg->data;
        processPolygon();
    }

    // Callback for foot positions
    void footPositionsCallback(const hexapod_msgs::msg::LegPositions::SharedPtr msg)
    {
        foot_positions_ = msg->positions;
        processPolygon();
    }

    // Callback for CoM
    void comCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        com_position_ = *msg;
        processPolygon();
    }

    // Main function to compute and publish results
    void processPolygon()
    {
        if (foot_status_.empty() || foot_positions_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for input data.");
            return;
        }

        if (foot_status_.size() != foot_positions_.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Mismatched foot status and positions sizes.");
            return;
        }

        // Construct support polygon
        hexapod_msgs::msg::Polygon polygon_msg;
        for (size_t i = 0; i < foot_status_.size(); ++i)
        {
            if (foot_status_[i]) // If foot is grounded
            {
                polygon_msg.points.push_back(foot_positions_[i]);
            }
        }

        // Publish the support polygon
        polygon_pub_->publish(polygon_msg);

        // Compute and publish the centroid
        auto centroid = computeCentroid(polygon_msg);
        centroid_pub_->publish(centroid);

        // Compute and publish the stability margin
        auto margin = computeStabilityMargin(polygon_msg, com_position_);
        std_msgs::msg::Float64 margin_msg;
        margin_msg.data = margin;
        stability_margin_pub_->publish(margin_msg);

        RCLCPP_INFO(this->get_logger(), "Published polygon, centroid, and stability margin.");
    }

    // Helper: Compute centroid of the polygon
    geometry_msgs::msg::Point computeCentroid(const hexapod_msgs::msg::Polygon &polygon)
    {
        geometry_msgs::msg::Point centroid;
        double x_sum = 0.0, y_sum = 0.0;
        for (const auto &point : polygon.points)
        {
            x_sum += point.x;
            y_sum += point.y;
        }
        size_t n = polygon.points.size();
        if (n > 0)
        {
            centroid.x = x_sum / n;
            centroid.y = y_sum / n;
        }
        return centroid;
    }

    // Helper: Compute stability margin
    double computeStabilityMargin(const hexapod_msgs::msg::Polygon &polygon,
                                  const geometry_msgs::msg::Point &com)
    {
        // Compute the shortest distance from CoM to polygon edges (simplified)
        double min_distance = std::numeric_limits<double>::max();
        for (size_t i = 0; i < polygon.points.size(); ++i)
        {
            const auto &p1 = polygon.points[i];
            const auto &p2 = polygon.points[(i + 1) % polygon.points.size()];
            double distance = pointToLineDistance(com, p1, p2);
            min_distance = std::min(min_distance, distance);
        }
        return min_distance;
    }

    // Helper: Point-to-line distance calculation
    double pointToLineDistance(const geometry_msgs::msg::Point &p,
                                const geometry_msgs::msg::Point &p1,
                                const geometry_msgs::msg::Point &p2)
    {
        // Compute the perpendicular distance from `p` to the line segment `p1-p2`
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double length_sq = dx * dx + dy * dy;
        if (length_sq == 0.0)
        {
            return std::hypot(p.x - p1.x, p.y - p1.y);
        }
        double t = ((p.x - p1.x) * dx + (p.y - p1.y) * dy) / length_sq;
        t = std::max(0.0, std::min(1.0, t));
        double proj_x = p1.x + t * dx;
        double proj_y = p1.y + t * dy;
        return std::hypot(p.x - proj_x, p.y - proj_y);
    }

    // Member variables
    std::vector<bool> foot_status_;
    std::vector<geometry_msgs::msg::Point> foot_positions_;
    geometry_msgs::msg::Point com_position_;

    rclcpp::Subscription<std_msgs::msg::BoolArray>::SharedPtr foot_status_sub_;
    rclcpp::Subscription<hexapod_msgs::msg::LegPositions>::SharedPtr foot_positions_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr com_sub_;

    rclcpp::Publisher<hexapod_msgs::msg::Polygon>::SharedPtr polygon_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr centroid_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stability_margin_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SupportPolygonCalculator>());
    rclcpp::shutdown();
    return 0;
}
