#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <getopt.h>
#include <tiffio.h>

extern "C" {
    #include "MLX90640_API.h"
    #include "MLX90640_I2C_Driver.h"
}

#include <opencv2/opencv.hpp>

#define MLX_ADDR 0x33

bool stop_signal = false;
void handle_signal(int sig) { stop_signal = true; }

// Forward declarations for your saving functions
void save_thermal_tiff_raw(float data[768], time_t timestamp);
void save_thermal_tiff_color(cv::Mat &colorFrame, time_t timestamp);

int main(int argc, char** argv) {
    signal(SIGINT, handle_signal);

    int seconds = -1;       // -s: undefined defaults to continuous
    bool save_color = false; // -c
    bool live_only = false;  // -l
    int opt;

    // Parse CLI arguments
    while ((opt = getopt(argc, argv, "s:cl")) != -1) {
        switch (opt) {
            case 's': seconds = atoi(optarg); break;
            case 'c': save_color = true; break;
            case 'l': live_only = true; break;
            default:
                fprintf(stderr, "Usage: %s [-s seconds] [-c] [-l]\n", argv[0]);
                return -1;
        }
    }

    // Hardware Init
    MLX90640_I2CInit();
    MLX90640_I2CFreqSet(400);

    static uint16_t eeData[832];
    static uint16_t frameData[834];
    static float mlx90640To[768];
    paramsMLX90640 params;

    if (MLX90640_DumpEE(MLX_ADDR, eeData) != 0) return -1;
    MLX90640_ExtractParameters(eeData, &params);
    MLX90640_SetRefreshRate(MLX_ADDR, 0x03); // 4Hz

    cv::namedWindow("Thermal Cam", cv::WINDOW_NORMAL);
    
    time_t start_time = time(NULL);
    int frame_count = 0;

    printf("Starting... Mode: %s\n", live_only ? "Live Only" : (save_color ? "Save Color" : "Save Raw"));

    while (!stop_signal) {
        // Check duration if -s was set
        if (seconds > 0 && difftime(time(NULL), start_time) >= seconds) {
            break;
        }

        if (MLX90640_GetFrameData(MLX_ADDR, frameData) < 0) continue;

        float Ta = MLX90640_GetTa(frameData, &params);
        MLX90640_CalculateTo(frameData, &params, 0.95, Ta - 8.0, mlx90640To);

        // Map to OpenCV
        cv::Mat frame(24, 32, CV_32FC1, mlx90640To);
        double minT, maxT;
        cv::minMaxLoc(frame, &minT, &maxT);

        cv::Mat normalized, colorFrame, resized;
        frame.convertTo(normalized, CV_8UC1, 255.0 / (maxT - minT), -minT * 255.0 / (maxT - minT));
        cv::applyColorMap(normalized, colorFrame, cv::COLORMAP_JET);
        cv::resize(colorFrame, resized, cv::Size(640, 480), 0, 0, cv::INTER_CUBIC);

        cv::imshow("Thermal Cam", resized);

        // Saving Logic
        if (!live_only) {
			time_t capture_time = time(NULL);
			frame_count++;
			

            if (save_color) {
                save_thermal_tiff_color(colorFrame, capture_time);
            } else {
                save_thermal_tiff_raw(mlx90640To, capture_time);
            }
        }

        if (cv::waitKey(1) == 27) break;
    }

    printf("\nFinished. Captured %d frames.\n", frame_count);
    cv::destroyAllWindows();
    return 0;
}

// Fixed Raw Save (Mapping 1D array to 24x32 scanlines)
void save_thermal_tiff_raw(float data[768], time_t timestamp) {
    char filename[64];
	char meta_time[20];
	struct tm *t = localtime(&timestamp);

    sprintf(filename, "frames/raw_capture_%ld.tif", (long)timestamp);
	strftime(meta_time, sizeof(meta_time), "%Y:%m:%d %H:%M:%S", t);

    TIFF *out = TIFFOpen(filename, "w");
    if (!out) return;

    TIFFSetField(out, TIFFTAG_IMAGEWIDTH, 32);
    TIFFSetField(out, TIFFTAG_IMAGELENGTH, 24);
    TIFFSetField(out, TIFFTAG_SAMPLESPERPIXEL, 1);
    TIFFSetField(out, TIFFTAG_BITSPERSAMPLE, 32);
    TIFFSetField(out, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
    TIFFSetField(out, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
    TIFFSetField(out, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(out, TIFFTAG_DATETIME, meta_time);

    for (int y = 0; y < 24; y++) {
        // Point to the correct row in the 1D float array
        TIFFWriteScanline(out, &data[y * 32], y, 0);
    }
    TIFFClose(out);
}

void save_thermal_tiff_color(cv::Mat &colorFrame, time_t timestamp) {
    char filename[64];
	char meta_time[20];
	struct tm *t = localtime(&timestamp);

    sprintf(filename, "frames/color_capture_%ld.tif", (long)timestamp);
	strftime(meta_time, sizeof(meta_time), "%Y:%m:%d %H:%M:%S", t);

    TIFF *out = TIFFOpen(filename, "w");
    if (!out) return;

    TIFFSetField(out, TIFFTAG_IMAGEWIDTH, colorFrame.cols);
    TIFFSetField(out, TIFFTAG_IMAGELENGTH, colorFrame.rows);
    TIFFSetField(out, TIFFTAG_SAMPLESPERPIXEL, 3);
    TIFFSetField(out, TIFFTAG_BITSPERSAMPLE, 8);
    TIFFSetField(out, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
    TIFFSetField(out, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(out, TIFFTAG_DATETIME, meta_time);	

    for (int y = 0; y < colorFrame.rows; y++) {
        cv::Mat rgbRow;
        cv::cvtColor(colorFrame.row(y), rgbRow, cv::COLOR_BGR2RGB);
        TIFFWriteScanline(out, rgbRow.data, y, 0);
    }
    TIFFClose(out);
}
