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

## Deprojectors
Deprojector are a class of components that uses the RGB-D data retrieved to recreate a pointcloud of the scene.
There are 3 of them.

The pointcloud can be accessed in Sofa as a `vector<defaulttype::Vector3>` and `PointCloudData` from PCLPlugin respectively labeled `@deprojector.output` and `@deprojector.outpcl`.
The attribute `flip` can be used to flip the whole pointcloud over the z-axis.

You can add a translation offset to any pointcloud using the `@deprojector.offset` attribute which is a `defaulttype::Vector3`.
`@deprojector.minmax` is a `Vector2` that is used for depth value filtering.

`@deprojector.downsample` attribute is for downsampling the resulting point cloud.
`@deprojector.drawpcl` is set to 1 if you want to render the pointcloud otherwise is set to 0

`@deprojector.densify` if larger than 1, generates a synthetic volume, stored in `@deprojector.synthvol` as `vector<defaulttype::Vector3>`.

### Whole scene deprojector 
This one is used if you want to create a point cloud for the whole observed frame.
```xml
<RealSenseDeprojector
 name="deproj"
 color="@realsense.color"
 depth="@realsense.depth"
 rscam="@realsense"
 downsample="5"
 drawpcl="1"
/>
```

### Mask deprojector 
This component, if provided with an image binary mask in a `cv::Mat` with the same resolution as realsense RGB-D frames.
```xml
<GetMaskFromContour
 name="mfc"
 image="@rs.color"
 contour="@some/contour.data" 
/>
<RealSenseMaskDeprojector
 name="deproj"
 depth="@rs.depth"
 rscam="@rs"
 input="@mfc.mask"
 downsample="11"
 densify="8"
 drawpcl="0" 
/>
```

### Point deprojector 
The difference with this one is it only reprojects a set of points specified in a `vector<defaulttype::Vector2>`.
```xml
<OpticalFlowOrSomeTracker
 name="tracker"
 in="@interest.markers"
 frame="@rs.color"
/>
<RealSensePointDeprojector
 name="deproj"
 input="@tracker.out"
 depth="@rs.depth"
 rscam="@rs"
 drawpcl="1" />
```
