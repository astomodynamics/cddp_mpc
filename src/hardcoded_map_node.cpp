#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

class GridMapNode : public rclcpp::Node
{
public:
  GridMapNode()
  : Node("grid_map_node")
  {
    // Declare parameters
    this->declare_parameter("map_length", 6.0);
    this->declare_parameter("map_width", 3.0);
    this->declare_parameter("obstacle_size", 0.41);
    this->declare_parameter("resolution", 0.1);

    // Get parameter values
    map_length_ = this->get_parameter("map_length").as_double();
    map_width_ = this->get_parameter("map_width").as_double();
    obstacle_size_ = this->get_parameter("obstacle_size").as_double();
    resolution_ = this->get_parameter("resolution").as_double();

    // Create the publisher
    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occupancy_map", 10);

    // Create a timer that calls publishGridMap() every 33 milliseconds (~30Hz)
    timer_ = this->create_wall_timer(33ms, std::bind(&GridMapNode::publishGridMap, this));
  }

private:
  void publishGridMap()
  {
    // Compute the number of cells in the grid
    int width_cells = static_cast<int>(map_width_ / resolution_);
    int height_cells = static_cast<int>(map_length_ / resolution_);

    // Prepare the grid message
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = this->get_clock()->now();
    grid_msg.header.frame_id = "map";
    grid_msg.info.resolution = resolution_;
    grid_msg.info.width = width_cells;
    grid_msg.info.height = height_cells;
    grid_msg.info.origin.position.x = 0.0;
    grid_msg.info.origin.position.y = 0.0;
    grid_msg.info.origin.position.z = 0.0;
    grid_msg.info.origin.orientation.w = 1.0;
    
    // Initialize the grid data with free space (0)
    grid_msg.data.assign(width_cells * height_cells, 0);
    
    // Add obstacles to the grid
    addObstacle(grid_msg, 0.762, 2.54);
    addObstacle(grid_msg, 2.794, 3.429);
    addObstacle(grid_msg, 0.762, 4.318);
    
    // Publish the grid map
    grid_pub_->publish(grid_msg);
    // RCLCPP_INFO(this->get_logger(), "Published occupancy grid map");
  }

  void addObstacle(nav_msgs::msg::OccupancyGrid &grid_msg, double x, double y)
  {
    int width_cells = grid_msg.info.width;
    int height_cells = grid_msg.info.height;
    int obs_cells = static_cast<int>(obstacle_size_ / resolution_);
    int obs_x = static_cast<int>(x / resolution_);
    int obs_y = static_cast<int>(y / resolution_);
    
    // Mark cells corresponding to the obstacle with a value of 100 (occupied)
    for (int i = -obs_cells / 2; i <= obs_cells / 2; ++i)
    {
      for (int j = -obs_cells / 2; j <= obs_cells / 2; ++j)
      {
        int idx_x = obs_x + i;
        int idx_y = obs_y + j;
        if (idx_x >= 0 && idx_x < width_cells && idx_y >= 0 && idx_y < height_cells)
        {
          grid_msg.data[idx_y * width_cells + idx_x] = 100;
        }
      }
    }
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double map_length_, map_width_, obstacle_size_, resolution_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GridMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
