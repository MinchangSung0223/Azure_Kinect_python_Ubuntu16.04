// Copyright (c) Microsoft Corporation. All rights reserved.

// Licensed under the MIT License.

 

#include <iostream>
#include <k4a/k4atypes.h>
#include <vector>
#include <k4a/k4a.hpp>
#include <k4arecord/playback.h>

#include <string>

#include "transformation_helpers.h"

#include "turbojpeg.h"



#include "k4adepthpixelcolorizer.h"

#include "k4apixel.h"

#include "k4astaticimageproperties.h"

#include "texture.h"

#include "viewerwindow.h"

#include <unistd.h>

 struct color_point_t
{
    int16_t xyz[3];
    uint8_t rgb[3];
};

using namespace viewer;

using namespace std; 

void ColorizeDepthImage(const k4a::image &depthImage,

                        DepthPixelVisualizationFunction visualizationFn,

                        std::pair<uint16_t, uint16_t> expectedValueRange,

                        std::vector<BgraPixel> *buffer);

uint16_t a[720][1280];

uint16_t* dataread(){

        return (uint16_t*)a;

}

 

uint8_t b[720][1280][4];

uint8_t* dataread_color(){

        return (uint8_t*)b;

}

 

uint16_t c[512][512];

uint16_t* dataread_depth(){

        return (uint16_t*)c;

}

uint8_t d[512][512][4];

uint8_t* dataread_color_to_depth(){

        return (uint8_t*)d;

}
int16_t xyz[720*1280][3];
uint8_t rgb[720*1280][3];

int16_t* dataread_get_pointcloud_xyz(){

        return (int16_t*)xyz;

}
uint8_t* dataread_get_pointcloud_rgb(){

        return (uint8_t*)rgb;

}

int main(){

 return -1;

}

int start(){
    uint8_t deviceId = K4A_DEVICE_DEFAULT;
   // int returnCode = 1;
    k4a_device_t device = NULL;
    const int32_t TIMEOUT_IN_MS = 10000;
    k4a_transformation_t transformation = NULL;
   // k4a_transformation_t transformation_color_downscaled = NULL;
    k4a_capture_t capture = NULL;
    std::string file_name = "";
    uint32_t device_count = 0;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
   // k4a_image_t color_image_downscaled = NULL;

    device_count = k4a_device_get_installed_count();

    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 0;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceId, &device))
    {
        printf("Failed to open device\n");
    	return 0;
    }

    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;//K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true; // ensures that depth and color images are both available in the capture

    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        printf("Failed to get calibration\n");

    }

    transformation = k4a_transformation_create(&calibration);

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        printf("Failed to start cameras\n");
    	return 0;
    }

    // Get a capture
    while(1){
	    switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
	    {
	    case K4A_WAIT_RESULT_SUCCEEDED:
		break;
	    case K4A_WAIT_RESULT_TIMEOUT:
		printf("Timed out waiting for a capture\n");
	    	return 0;
	    case K4A_WAIT_RESULT_FAILED:
		printf("Failed to read a capture\n");
	    	return 0;
	    }

	    // Get a depth image
	    depth_image = k4a_capture_get_depth_image(capture);
	    if (depth_image == 0)
	    {
		printf("Failed to get depth image from capture\n");
	    	return 0;
	    }

	    // Get a color image
	    color_image = k4a_capture_get_color_image(capture);
	    if (color_image == 0)
	    {
		printf("Failed to get color image from capture\n");
	    	return 0;
	    }
	    int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
	    int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
	    k4a_image_t transformed_color_image = NULL;
	    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
		                                         depth_image_width_pixels,
		                                         depth_image_height_pixels,
		                                         depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
		                                         &transformed_color_image))
	    {
		printf("Failed to create transformed color image\n");
		return false;
	    }
	   if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation,
		                                                                       depth_image,
		                                                                       color_image,
		                                                                       transformed_color_image))
	    {
		printf("Failed to compute transformed color image\n");
		return false;
	    }

	   int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
	    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
	    k4a_image_t transformed_depth_image = NULL;
	    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
		                                         color_image_width_pixels,
		                                         color_image_height_pixels,
		                                         color_image_width_pixels * (int)sizeof(uint16_t),
		                                         &transformed_depth_image))
	    {
		printf("Failed to create transformed depth image\n");
		return false;
	    }



	    if (K4A_RESULT_SUCCEEDED !=
		k4a_transformation_depth_image_to_color_camera(transformation, depth_image, transformed_depth_image))
	    {
		printf("Failed to compute transformed depth image\n");
		return false;
	    }


	 k4a_image_t point_cloud_image = NULL;
	    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
		                                         color_image_width_pixels,
		                                         color_image_height_pixels,
		                                         color_image_width_pixels * 3 * (int)sizeof(int16_t),
		                                         &point_cloud_image))
	    {
		printf("Failed to create point cloud image\n");
		return false;
	    }

	    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation,
		                                                                      transformed_depth_image,
		                                                                      K4A_CALIBRATION_TYPE_COLOR,
		                                                                      point_cloud_image))
	    {
		printf("Failed to compute point cloud\n");
		return false;
	    }


    //std::vector<color_point_t> points;

    //printf("width %d height %d ",width,height);
    int16_t *point_cloud_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_cloud_image);
    uint8_t *color_image_data = k4a_image_get_buffer(color_image);
	
            int n1=0;
            int n2=0;
            int n3=0;
            int n4=0;
            int n5=0;
            int n6=0;


	    const uint8_t *colorData = k4a_image_get_buffer(color_image);
	    const uint16_t *transformed_depthData = reinterpret_cast<const uint16_t *>(k4a_image_get_buffer(transformed_depth_image )); 	
	    
	    for (int h = 0; h < 720; ++h)
	    {
		for (int w = 0; w < 1280; ++w)
		{
			a[h][w] = transformed_depthData[n1++];
			b[h][w][0] = colorData[n2++];
			b[h][w][1] = colorData[n2++];
			b[h][w][2] = colorData[n2++];
			b[h][w][3] = colorData[n2++];

		        xyz[n5][0] = point_cloud_image_data[3 * n5 + 0];
			xyz[n5][1] = point_cloud_image_data[3 * n5 + 1];
			xyz[n5][2] = point_cloud_image_data[3 * n5 + 2];
			if (xyz[n5][2] == 0)
			{
			    continue;
			}
			n5++;
			rgb[n6][2] = color_image_data[4 * n6 + 0];
			rgb[n6][1] = color_image_data[4 * n6 + 1];
			rgb[n6][0] = color_image_data[4 * n6 + 2];
			uint8_t alpha = color_image_data[4 * n6 + 3];

			if (rgb[n6][0] == 0 &&rgb[n6][1] == 0 &&rgb[n6][2] == 0 && alpha == 0)
			{
			    continue;
			}
			n6++;

		}
	    }
	    const uint16_t *depthData = reinterpret_cast<const uint16_t *>(k4a_image_get_buffer(depth_image));
	    const uint8_t *transformed_colorData = k4a_image_get_buffer(transformed_color_image ); 		
	    for (int h = 0; h < 512; ++h)
	    {
		for (int w = 0; w < 512; ++w)
		{
	              c[h][w] = depthData[n3++];
		      d[h][w][0] = transformed_colorData[n4++];
		      d[h][w][1] = transformed_colorData[n4++];
		      d[h][w][2] = transformed_colorData[n4++];
	              d[h][w][3] = transformed_colorData[n4++];
		}
	    }
	
	    k4a_image_release(transformed_color_image);
	    k4a_image_release(transformed_depth_image);
	    k4a_image_release(point_cloud_image);
	}
    return 0;
}

	

 

 

 

 

 

 

 

 

 

//-----------------------------------------------------------------------------------------

 

 


// Given a depth image, output a BGRA-formatted color image into buffer, using

// expectedValueRange to define what min/max values the depth image should have.

// Low values are blue, high values are red.

//

void ColorizeDepthImage(const k4a::image &depthImage,

                        DepthPixelVisualizationFunction visualizationFn,

                        std::pair<uint16_t, uint16_t> expectedValueRange,

                        std::vector<BgraPixel> *buffer)

{

    // This function assumes that the image is made of depth pixels (i.e. uint16_t's),

    // which is only true for IR/depth images.

    //

    const k4a_image_format_t imageFormat = depthImage.get_format();

    if (imageFormat != K4A_IMAGE_FORMAT_DEPTH16 && imageFormat != K4A_IMAGE_FORMAT_IR16)

 

    {

        throw std::logic_error("Attempted to colorize a non-depth image!");

    }

 

    const int width = depthImage.get_width_pixels();

    const int height = depthImage.get_height_pixels();

 

    buffer->resize(static_cast<size_t>(width * height));

 

    const uint16_t *depthData = reinterpret_cast<const uint16_t *>(depthImage.get_buffer());

    for (int h = 0; h < height; ++h)

    {

        for (int w = 0; w < width; ++w)

        {

            const size_t currentPixel = static_cast<size_t>(h * width + w);

            (*buffer)[currentPixel] = visualizationFn(depthData[currentPixel],

                                                      expectedValueRange.first,

                                                      expectedValueRange.second);

        }

    }

}

int end(){

     //k4a_device_close(device);

     exit(0);

 

    return -1;

}

extern "C" {

	int Foo_start(){start(); return 0;}

        uint16_t*  Foo_dataread(){return dataread();}

	int Foo_end(){end(); return 0;}

        uint8_t*  Foo_dataread_color(){return dataread_color();}

        uint16_t*  Foo_dataread_depth(){return dataread_depth();}
        int16_t*  Foo_get_pointcloud_xyz(){return dataread_get_pointcloud_xyz();}
        uint8_t*  Foo_get_pointcloud_rgb(){return dataread_get_pointcloud_rgb();}

        uint8_t*  Foo_dataread_color_to_depth(){return dataread_color_to_depth();}

}
