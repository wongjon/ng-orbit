# ICP Implementation Usage

## Requirements
Ensure you have the following packages installed on an Ubuntu 18.04+ OS: 
* Intel RealSense 2.0 SDK
* Open3D
* CMake
* Eigen
* OpenCV (Optional)

## Creating Pointclouds

### CAD Test
Create a pointcloud of the desired object with its CAD. This can be done using a CAD software such as Solidworks. For Solidworks, export your object as a .SLDPRT. Open the .SLDPRT file and click save as a .PLY file. In the advanced saving settings, use the function `convert mesh to body`, edit the feature and click `advanced mesh refinement`. Decreasing the max element size increases the resolution of the pointcloud. Save as `test.PLY` file with the desired resolution to the **ng_realsense/build** folder.

### LiDAR Received
Use the LiDAR L515 camera to generate a pointcloud of the object. It will be helpful for the object to be in isolation with minimal background. Thus, positioning the object above ground and outdoors may be ideal. 

Use `realsense-viewer` in the terminal to launch the Intel RealSense Viewer. Plug in the L515 camera and click **save** in the upper righthand corner to save the point cloud. Save as `received.ply` to the **ng_realsense/build** folder.

## Running ICP Implementation
Open `icp_implementation.cpp`. Ensure the names of the pointclouds match what is saved in the **ng_realsense/build** folder. Exit out of the file and navigate to the **ng_realsense/build** folder with

`$ cd build`

Next, use cmake to build the project with:

`$ make`

Finally, run the project with:

`$ ./ng_realsense`

An Open3D GUI will display the two pointclouds together. If the two pointclouds have matched together, then the resulting **transformation matrix** saved in `transformedroom.ply` can be used to orient the robotic arm and identify where the grapple point is located, as it is known within the CAD pointcloud. 