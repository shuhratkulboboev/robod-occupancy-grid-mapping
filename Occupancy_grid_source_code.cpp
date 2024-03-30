#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <array>
#include <string>
#include <FreeImage.h>

const int GRID_SIZE_X = 100;
const int GRID_SIZE_Y = 100;
struct Record
{
    double time_stamp;
    double x_pos;
    double y_pos;
    double angle;
    std::array<double, 4> ultrasound; // distance
    // Add more fields as needed
};
class OccupancyGrid
{
public:
    OccupancyGrid() : grid(GRID_SIZE_Y, std::vector<char>(GRID_SIZE_X, '.'))
    {

        std::ifstream file("robot.csv");
        // Check if the file is open
        if (!file.is_open())
        {
            std::cerr << "Error opening the file!" << std::endl;
            return;
        }

        // Read each line from the file
        std::string line;
        while (getline(file, line))
        {
            // Create a stringstream from the line
            std::istringstream linestream(line);

            // Define a record to store values for each line
            Record record;

            // Read each value from the stringstream and convert it to double
            std::string time_stamp_str, x_pos_str, y_pos_str, angle_str, ultrasound0_str, ultrasound1_str, ultrasound2_str, ultrasound3_str;
            getline(linestream, time_stamp_str, ',');
            getline(linestream, x_pos_str, ',');
            getline(linestream, y_pos_str, ',');
            getline(linestream, angle_str, ',');
            getline(linestream, ultrasound0_str, ',');
            getline(linestream, ultrasound1_str, ',');
            getline(linestream, ultrasound2_str, ',');
            getline(linestream, ultrasound3_str, ',');
            // Convert string to double
            try
            {
                record.time_stamp = std::stod(time_stamp_str);
                record.x_pos = std::stod(x_pos_str);
                record.y_pos = std::stod(y_pos_str);
                record.angle = std::stod(angle_str);
                record.ultrasound[0] = timeToDistance(std::stod(ultrasound0_str));
                record.ultrasound[1] = timeToDistance(std::stod(ultrasound1_str));
                record.ultrasound[2] = timeToDistance(std::stod(ultrasound2_str));
                record.ultrasound[3] = timeToDistance(std::stod(ultrasound3_str));
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
            // Add the record to the vector
            records.push_back(record);
        }

        // Close the file
        file.close();
    }
    double timeToDistance(double time)
    {
        constexpr const double speed_of_sound = 343.0; // meters per second
        return (0.5 * time * speed_of_sound);
    }
    void updateGrid(double x, double y, double angle, std::array<double, 4> sensor_reading)
    {
        // Convert angle to radians
        double theta = angle * M_PI / 180.0;

        // Calculate sensor positions relative to the robot center
        double sensorX[4] = {2.0, 2.0, -2.0, -2.0}; // represents the sensors position
        double sensorY[4] = {1.0, -1.0, 1.0, -1.0};
        // Update the occupancy grid for sensors
        double sensorGlobalX[4];
        double sensorGlobalY[4];
        for (int i = 0; i < 4; ++i)
        {
            sensorGlobalX[i] = x + sensorX[i] * std::cos(theta) - sensorY[i] * std::sin(theta);
            sensorGlobalY[i] = y + sensorX[i] * std::sin(theta) + sensorY[i] * std::cos(theta);
            // updateGridCell(sensorGlobalX[i], sensorGlobalY[i], static_cast<char>(i + 65));
            // Calculate obstacle position relative to the sensor
            double obstacleX;
            double obstacleY;
            double angleOffset = M_PI / 4; // Default angle offset for i == 0

            if (i == 1)
            {
                angleOffset = -45 * M_PI / 180.0;
            }
            else if (i == 2)
            {
                angleOffset = 135 * M_PI / 180.0;
            }
            else if (i == 3)
            {
                angleOffset = 225 * M_PI / 180.0;
            }

            // Calculate obstacle position relative to the sensor
            obstacleX = sensorGlobalX[i] + sensor_reading[i] * std::cos(theta + angleOffset);
            obstacleY = sensorGlobalY[i] + sensor_reading[i] * std::sin(theta + angleOffset);

            // Update the occupancy grid for the obstacle
            updateGridCell(obstacleX, obstacleY, '#'); // You can use any character or value to represent the obstacle
        }
    }
    double scale(double value)
    {
        return (value * 15) + 20;
    }
    void createMap()
    {
        for (const auto &record : records)
        {
            updateGrid(scale(record.x_pos) , scale(record.y_pos), record.angle, record.ultrasound);
        }
        printGrid();
        saveBitmap("output.bmp");
    }
    void printGrid() const
    {
        for (const auto &row : grid)
        {
            for (char cell : row)
            {
                std::cout << cell << ' ';
            }
            std::cout << '\n';
        }
    }
    void printRecords() const
    {
        // Print the read data
        for (const auto &record : records)
        {
            std::cout << "time_stamp: " << record.time_stamp << ", angle: " << record.angle << std::endl;
            // Print additional fields if you have more
        }
    }
    void saveBitmap(const char *filename) {
        FIBITMAP *bitmap = FreeImage_Allocate(GRID_SIZE_X, GRID_SIZE_Y , 24);

        for (int y = 0; y < GRID_SIZE_Y; ++y) {
            for (int x = 0; x < GRID_SIZE_X; ++x) {
                RGBQUAD color;
                
                // Set color based on grid cell content
                if (grid[y][x] == '#') {
                    // Red color for obstacles
                    color.rgbRed = 255;
                    color.rgbGreen = color.rgbBlue = 0;
                } else {
                    // White color for empty spaces
                    color.rgbRed = color.rgbGreen = color.rgbBlue = 255;
                }

                FreeImage_SetPixelColor(bitmap, x, GRID_SIZE_Y - 1 - y, &color);
            }
        }

        if (FreeImage_Save(FIF_BMP, bitmap, filename, 0))
            std::cout << "Bitmap saved to: " << filename << std::endl;
        else
            std::cerr << "Error saving bitmap!" << std::endl;

        FreeImage_Unload(bitmap);
    }

private:
    std::vector<std::vector<char>> grid;
    std::vector<Record> records;
    void updateGridCell(double x, double y, char representation)
    {
        int gridX = static_cast<int>(std::round(x)); // Offset to center
        int gridY = static_cast<int>(std::round(y)); // Offset to center

        // Update the occupancy grid
        if (isValidGridPosition(gridX, gridY))
        {
            grid[gridY][gridX] = representation;
        }
    }

    bool isValidGridPosition(int x, int y) const
    {
        return x >= 0 && x < GRID_SIZE_X && y >= 0 && y < GRID_SIZE_Y;
    }
};

int main()
{
    OccupancyGrid occupancyGrid;
    occupancyGrid.createMap();
    return 0;
}
