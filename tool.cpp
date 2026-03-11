#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <getopt.h>
#include <tiffio.h>

extern "C" {
    #include "MLX90640_API.h"
    #include "MLX90640_I2C_Driver.h"
}

#include <opencv2/opencv.hpp>

#define MLX_ADDR 0x33

/* Mux ports for the two cameras (must match MLX90640_I2C_Driver.h) */
static const uint8_t CAM_PORT_1 = MLX90640_MUX_PORT_1;
static const uint8_t CAM_PORT_2 = MLX90640_MUX_PORT_2;
static const int NUM_CAMS = 2;

bool stop_signal = false;
void handle_signal(int sig) { stop_signal = true; }

// Forward declarations (timestamp_ms, cam_id, frame_index = ith capture for that camera, for joining later)
void save_thermal_tiff_raw(float data[768], int64_t timestamp_ms, int cam_id, int frame_index);
void save_thermal_tiff_color(cv::Mat &colorFrame, int64_t timestamp_ms, int cam_id, int frame_index);
void save_thermal_json(float data[768], int64_t timestamp_ms, int cam_id, int frame_index);

static void print_usage(const char *prog) {
    printf("Usage: %s [OPTIONS]\n", prog);
    printf("Capture from two MLX90640 thermal cameras on I2C mux ports 1 and 2.\n\n");
    printf("Options:\n");
    printf("  -s, --seconds N   Run for N seconds then exit (default: run until Ctrl+C)\n");
    printf("  -c, --color      Save color TIFF images (default: save raw JSON)\n");
    printf("  -l, --live       Live view only, do not save to disk\n");
    printf("  -h, --help       Show this help and exit\n");
}

int main(int argc, char** argv) {
    signal(SIGINT, handle_signal);
	
	sigset_t x;
	sigset_t old_x;

	sigemptyset(&x);
	sigaddset(&x, SIGINT);
	sigaddset(&x, SIGTERM);

    int seconds = -1;       // -s: undefined defaults to continuous
    bool save_color = false; // -c
    bool live_only = false;  // -l
    int opt;

    static const struct option long_options[] = {
        { "seconds", required_argument, 0, 's' },
        { "color",   no_argument,       0, 'c' },
        { "live",    no_argument,       0, 'l' },
        { "help",    no_argument,       0, 'h' },
        { 0, 0, 0, 0 }
    };

    while ((opt = getopt_long(argc, argv, "s:clh", long_options, NULL)) != -1) {
        switch (opt) {
            case 's': seconds = atoi(optarg); break;
            case 'c': save_color = true; break;
            case 'l': live_only = true; break;
            case 'h':
                print_usage(argv[0]);
                return 0;
            default:
                fprintf(stderr, "Try '%s --help' for more information.\n", argv[0]);
                return -1;
        }
    }

    // Hardware Init
    MLX90640_I2CInit();
    MLX90640_I2CFreqSet(400);

    static uint16_t eeData1[832], eeData2[832];
    static uint16_t frameData1[834], frameData2[834];
    static float mlx90640To1[768], mlx90640To2[768];
    paramsMLX90640 params1, params2;

    MLX90640_I2CSetPort(CAM_PORT_1);
    if (MLX90640_DumpEE(MLX_ADDR, eeData1) != 0) return -1;
    MLX90640_ExtractParameters(eeData1, &params1);
    MLX90640_SetRefreshRate(MLX_ADDR, 0x03); // 4Hz

    MLX90640_I2CSetPort(CAM_PORT_2);
    if (MLX90640_DumpEE(MLX_ADDR, eeData2) != 0) return -1;
    MLX90640_ExtractParameters(eeData2, &params2);
    MLX90640_SetRefreshRate(MLX_ADDR, 0x03); // 4Hz

    cv::namedWindow("Thermal Cam 1", cv::WINDOW_NORMAL);
    cv::namedWindow("Thermal Cam 2", cv::WINDOW_NORMAL);
    
    time_t start_time = time(NULL);
    int frame_count = 0;
    int frame_index_1 = 0;  /* ith image captured on camera 1 (for joining later) */
    int frame_index_2 = 0;  /* ith image captured on camera 2 */

    printf("Starting... Mode: %s (%d cameras on mux ports %d, %d)\n",
           live_only ? "Live Only" : (save_color ? "Save Color" : "Save Raw"), NUM_CAMS, CAM_PORT_1, CAM_PORT_2);

    while (!stop_signal) {
        // Check duration if -s was set
        if (seconds > 0 && difftime(time(NULL), start_time) >= seconds) {
            break;
        }
		
		// CRITICAL SECTION
		sigprocmask(SIG_BLOCK, &x, &old_x);

        MLX90640_I2CSetPort(CAM_PORT_1);
        int ok1 = MLX90640_GetFrameData(MLX_ADDR, frameData1);
        MLX90640_I2CSetPort(CAM_PORT_2);
        int ok2 = MLX90640_GetFrameData(MLX_ADDR, frameData2);

		sigprocmask(SIG_SETMASK, &old_x, NULL);

        if (ok1 < 0 || ok2 < 0) continue;

        float Ta1 = MLX90640_GetTa(frameData1, &params1);
        MLX90640_CalculateTo(frameData1, &params1, 0.95, Ta1 - 8.0, mlx90640To1);
        float Ta2 = MLX90640_GetTa(frameData2, &params2);
        MLX90640_CalculateTo(frameData2, &params2, 0.95, Ta2 - 8.0, mlx90640To2);

        struct timeval tv;
        gettimeofday(&tv, NULL);
        int64_t capture_ms = (int64_t)tv.tv_sec * 1000 + (int64_t)(tv.tv_usec / 1000);

        // Camera 1 -> window 1
        cv::Mat frame1(24, 32, CV_32FC1, mlx90640To1);
        double minT1, maxT1;
        cv::minMaxLoc(frame1, &minT1, &maxT1);
        cv::Mat norm1, color1, resized1;
        frame1.convertTo(norm1, CV_8UC1, 255.0 / (maxT1 - minT1), -minT1 * 255.0 / (maxT1 - minT1));
        cv::applyColorMap(norm1, color1, cv::COLORMAP_JET);
        cv::resize(color1, resized1, cv::Size(640, 480), 0, 0, cv::INTER_CUBIC);
        cv::imshow("Thermal Cam 1", resized1);

        // Camera 2 -> window 2
        cv::Mat frame2(24, 32, CV_32FC1, mlx90640To2);
        double minT2, maxT2;
        cv::minMaxLoc(frame2, &minT2, &maxT2);
        cv::Mat norm2, color2, resized2;
        frame2.convertTo(norm2, CV_8UC1, 255.0 / (maxT2 - minT2), -minT2 * 255.0 / (maxT2 - minT2));
        cv::applyColorMap(norm2, color2, cv::COLORMAP_JET);
        cv::resize(color2, resized2, cv::Size(640, 480), 0, 0, cv::INTER_CUBIC);
        cv::imshow("Thermal Cam 2", resized2);

        // Saving: both cameras, same timestamp; frame_index lets you join by i later
        if (!live_only) {
            frame_count++;
            if (save_color) {
                save_thermal_tiff_color(color1, capture_ms, 1, frame_index_1++);
                save_thermal_tiff_color(color2, capture_ms, 2, frame_index_2++);
            } else {
                save_thermal_json(mlx90640To1, capture_ms, 1, frame_index_1++);
                save_thermal_json(mlx90640To2, capture_ms, 2, frame_index_2++);
            }
        }

        if (cv::waitKey(1) == 27) break;
    }

    printf("\nFinished. Captured %d frames (cam1: %d, cam2: %d).\n", frame_count, frame_index_1, frame_index_2);
    cv::destroyAllWindows();
    return 0;
}

// Fixed Raw Save (Mapping 1D array to 24x32 scanlines). Name: raw_capture_<cam_id>_<i>_<timestamp>.tif
void save_thermal_tiff_raw(float data[768], int64_t timestamp_ms, int cam_id, int frame_index) {
    char filename[80];
	char meta_time[32];
	time_t sec = (time_t)(timestamp_ms / 1000);
	int ms = (int)(timestamp_ms % 1000);
	struct tm *t = localtime(&sec);

    sprintf(filename, "frames/raw_capture_%d_%d_%lld.tif", cam_id, frame_index, (long long)timestamp_ms);
	strftime(meta_time, sizeof(meta_time), "%Y:%m:%d %H:%M:%S", t);
	snprintf(meta_time + strlen(meta_time), sizeof(meta_time) - strlen(meta_time), ".%03d", ms);

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

// Name: color_capture_<cam_id>_<i>_<timestamp>.tif
void save_thermal_tiff_color(cv::Mat &colorFrame, int64_t timestamp_ms, int cam_id, int frame_index) {
    char filename[80];
	char meta_time[32];
	time_t sec = (time_t)(timestamp_ms / 1000);
	int ms = (int)(timestamp_ms % 1000);
	struct tm *t = localtime(&sec);

    sprintf(filename, "frames/color_capture_%d_%d_%lld.tif", cam_id, frame_index, (long long)timestamp_ms);
	strftime(meta_time, sizeof(meta_time), "%Y:%m:%d %H:%M:%S", t);
	snprintf(meta_time + strlen(meta_time), sizeof(meta_time) - strlen(meta_time), ".%03d", ms);

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

// Name: data_capture_<cam_id>_<i>_<timestamp>.json (i = frame index for joining)
void save_thermal_json(float data[768], int64_t timestamp_ms, int cam_id, int frame_index) {
    char filename[80];
    char meta_time[32];
	float avg = 0;

	for (int i = 0; i < 768; i++) avg += data[i];
	avg /= 768;	

	time_t sec = (time_t)(timestamp_ms / 1000);
	int ms = (int)(timestamp_ms % 1000);
	struct tm *t = localtime(&sec);

    sprintf(filename, "frames/data_capture_%d_%d_%lld.json", cam_id, frame_index, (long long)timestamp_ms);
    strftime(meta_time, sizeof(meta_time), "%Y-%m-%d %T", t);
    snprintf(meta_time + strlen(meta_time), sizeof(meta_time) - strlen(meta_time), ".%03d", ms);

    FILE *f = fopen(filename, "w");
    if (!f) {
        perror("Failed to open JSON file");
        return;
    }

    // Start JSON object (frame_index allows joining by i later)
    fprintf(f, "{\n");
    fprintf(f, "  \"mux_port\": %d,\n", cam_id);
    fprintf(f, "  \"frame_index\": %d,\n", frame_index);
    fprintf(f, "  \"timestamp\": \"%s\",\n", meta_time);
    fprintf(f, "  \"unix_time\": %ld,\n", (long)sec);
    fprintf(f, "  \"unix_time_ms\": %lld,\n", (long long)timestamp_ms);
    fprintf(f, "  \"avg\": %f,\n", avg);
    fprintf(f, "  \"data\": [\n");

    // Write the 768 pixel values
    for (int i = 0; i < 768; i++) {
        // format to 2 decimal places to save space, change to %.4f if higher precision is needed
        fprintf(f, "    %.2f%s", data[i], (i < 767 ? ",\n" : "\n"));
    }

    fprintf(f, "  ]\n");
    fprintf(f, "}\n");

    fclose(f);
}
