# realsenseplugin
Sofa components for using realsense cam

## Streamers
Streamers are a class of components that acquire RGB-D data stream from a RealSense camera.
```xml
<GenericScheduler name="scheduler" frequency="1" />
<RealSenseCam
 name="rs"
 scheduler="@scheduler.scheduler"
 intrinsics="/path/to/intrinsics.log"
/>
```
Here we instantiated a RealSenseCam component. It is linked to a generic scheduler (from sofascheduler, wip refactoring) which schedules each frame retrieval.
Intrinsics data is the path to which save the camera's intrinsic parameters.

Depth and color frames are stored in a `cv::Mat` and can be accessed as data in Sofa with `@rs.color` and `@rs.depth`.

If multiple cameras are being used, you may use RealSenseMultiCam component.
It instantiates a RealSenseVirtualCam for each depth camera connected. this is useful in a master/slave situation where a second realsense is feeding information about the observed scene to complement the first observation. Multicam Calibration is still not implemented yet.  
```xml
<Node name="streamers">
 <RealSenseMultiCam name="rs" />
</Node>
```
