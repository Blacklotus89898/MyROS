#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <SDL2/SDL.h>
#include <vector>

// Initialize SDL
SDL_Window* window = nullptr;
SDL_Renderer* renderer = nullptr;
const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;
int x = SCREEN_WIDTH / 2;
int y = SCREEN_HEIGHT / 2;
const int PIXEL_SIZE = 50; // Size of the pixel to be drawn

// Callback for cmd_vel messages
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // Update pixel position based on linear velocity
    y -= static_cast<int>(msg->linear.x * 10); // Scaling factor for speed
    x -= static_cast<int>(msg->angular.z * 10); // Invert y axis to match SDL coordinates

    // Clip coordinates to be within the screen bounds
    if (x < 0) x = 0;
    if (x >= SCREEN_WIDTH) x = SCREEN_WIDTH - 1;
    if (y < 0) y = 0;
    if (y >= SCREEN_HEIGHT) y = SCREEN_HEIGHT - 1;

    // Clear the screen
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    // Draw the pixel
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // Red color
    SDL_Rect rect = {x - PIXEL_SIZE / 2, y - PIXEL_SIZE / 2, PIXEL_SIZE, PIXEL_SIZE};
    SDL_RenderFillRect(renderer, &rect);

    // Update the screen
    SDL_RenderPresent(renderer);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pixel_controller");
    ros::NodeHandle nh;

    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        ROS_ERROR("Failed to initialize SDL: %s", SDL_GetError());
        return -1;
    }

    window = SDL_CreateWindow("Pixel Controller", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (!window) {
        ROS_ERROR("Failed to create window: %s", SDL_GetError());
        SDL_Quit();
        return -1;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        ROS_ERROR("Failed to create renderer: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }

    // Subscribe to the cmd_vel topic
    ros::Subscriber sub = nh.subscribe("/turtle1/cmd_vel", 10, cmdVelCallback);

    ros::spin();

    // Cleanup SDL
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}

