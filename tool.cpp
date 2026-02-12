#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

extern "C" {
	#include "MLX90640_API.h"
	#include "MLX90640_I2C_Driver.h"
}

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Configuration
#define MLX_ADDR 0x33
#define REFRESH_RATE 4  // 4Hz is a good balance for Jetson Nano

bool stop_signal = false;

void handle_signal(int sig) {
    stop_signal = true;
}

int main(int argc, char** argv) {
    signal(SIGINT, handle_signal);

    static uint16_t eeData[832];
    static uint16_t frameData[834];
    static float mlx90640To[768];
    paramsMLX90640 params;
    int status;

    printf("Starting MLX90640 Thermal Interface...\n");

    // 1. Initialize Hardware
    MLX90640_I2CInit();
    
    // Set frequency to 400kHz (Recommended for Jetson)
    MLX90640_I2CFreqSet(400);

    // 2. Dump EEPROM and Extract Parameters
    // This is where your previous "infinite loop" bug was lived!
    status = MLX90640_DumpEE(MLX_ADDR, eeData);
    if (status != 0) {
        fprintf(stderr, "Failed to load EEPROM. Check wiring.\n");
        return -1;
    }

    status = MLX90640_ExtractParameters(eeData, &params);
    if (status != 0) {
        fprintf(stderr, "Parameter extraction failed! (Error: %d)\n", status);
        return -1;
    }

    // 3. Configure Sensor Refresh Rate
    MLX90640_SetRefreshRate(MLX_ADDR, 0x05); // 0x03 = 4Hz, 0x04 = 8Hz

    printf("Calibration Loaded. Entering Main Loop...\n");
    printf("Press Ctrl+C to exit.\n");

    cv::namedWindow("Thermal Cam", cv::WINDOW_NORMAL);

    while (!stop_signal) {
        // A. Capture Frame
        status = MLX90640_GetFrameData(MLX_ADDR, frameData);
        if (status < 0) {
            continue; // Skip bad reads
        }

        // B. Calculate Temperatures
        float vdd = MLX90640_GetVdd(frameData, &params);
        float Ta = MLX90640_GetTa(frameData, &params);
        float tr = Ta - 8.0; // Reflected temperature offset
        float emissivity = 0.95;

        MLX90640_CalculateTo(frameData, &params, emissivity, tr, mlx90640To);

        // C. OpenCV Processing
        // Turn 1D array into 24x32 matrix
        cv::Mat frame(24, 32, CV_32FC1, mlx90640To);
        
        // Auto-normalize colors based on current scene
        double minT, maxT;
        cv::minMaxLoc(frame, &minT, &maxT);
        
        cv::Mat normalized;
        frame.convertTo(normalized, CV_8UC1, 255.0 / (maxT - minT), -minT * 255.0 / (maxT - minT));

        // Apply heatmap and upscale
        cv::Mat colorFrame, resized;
        cv::applyColorMap(normalized, colorFrame, cv::COLORMAP_JET);
        cv::resize(colorFrame, resized, cv::Size(640, 480), 0, 0, cv::INTER_CUBIC);

        // Overlay temperature info
        char text[50];
        sprintf(text, "Max: %.1f C  Avg: %.1f C", maxT, (maxT + minT) / 2);
        cv::putText(resized, text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

        cv::imshow("Thermal Cam", resized);

        if (cv::waitKey(1) == 27) break; // ESC to exit
    }

    printf("\nShutting down...\n");
    cv::destroyAllWindows();
    return 0;
}
