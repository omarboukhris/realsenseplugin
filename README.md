# realsenseplugin
Sofa components for using realsense cam

## Streamers
Streamers are a class of components that acquire RGB-D data stream from a RealSense camera.
```xml
<RealSenseCam
 name="rs"
 intrinsics="/path/to/intrinsics.log"
/>
```
Here we instantiated a RealSenseCam component. It is linked to a generic scheduler (from sofascheduler, wip refactoring) which schedules each frame retrieval.
Intrinsics data is the path to which save the camera's intrinsic parameters.

Depth and color frames are stored in a `cv::Mat` and can be accessed as data in Sofa with `@rs.color` and `@rs.depth`.

If multiple cameras are being used, you may use RealSenseMultiCam component.
It instantiates a RealSenseVirtualCam for each depth camera connected. this is useful in a master/slave situation where a second realsense is feeding information about the observed scene to complement the first observation. 

Another methode for instantiating multiple cameras is to use `serialid` with an index starting at 0.
```xml
<RealSenseCam
 name="rs0"
 serialid="0"
 intrinsics="/path/to/intrinsics0.log"
/>
<RealSenseCam
 name="rs1"
 serialid="1"
 intrinsics="/path/to/intrinsics1.log"
/>
...
<RealSenseCam
 name="rsn-1"
 serialid="n-1"
 intrinsics="/path/to/intrinsics_n-1.log"
/>
```

## Deprojectors
Deprojector are a class of components that uses the RGB-D data retrieved to recreate a pointcloud of the scene.
There are 3 of them.

The pointcloud can be accessed in Sofa as a `vector<defaulttype::Vector3>` labeled `@deprojector.output`.
The attribute `flip` can be used to flip the whole pointcloud over the z-axis.

You can add a translation offset to any pointcloud using the `@deprojector.offset` attribute which is a `defaulttype::Vector3`.
`@deprojector.minmax` is a `Vector2` that is used for depth value filtering.

`@deprojector.downsample` attribute is for downsampling the resulting point cloud.
`@deprojector.drawpcl` is set to 1 if you want to render the pointcloud otherwise is set to 0

`@deprojector.densify` if larger than 1, generates a synthetic volume by duplicating the pointcloud surface, stored in `@deprojector.synthvol` as `vector<defaulttype::Vector3>`.

### Whole scene deprojector 
This one is used if you want to create a point cloud for the whole observed frame.
```xml
<RealSenseDeprojector
  name="deproj"
  rsframe="@../rs.rsframe"
  rscam="@realsense"
  downsample="5"
  drawpcl="1"
/>
```

### Mask deprojector 
This component, if provided with an image binary mask in a `cv::Mat` with the same resolution as realsense RGB-D frames.
```xml
<MaskFromContour
  name="mfc"
  image="@rs.rsframe"
  contour="@some/contour.data" 
/>
<RealSenseMaskDeprojector
  name="deproj"
  rsframe="@rs.rsframe"
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
  rsframe="@rs.rsframe"
  rscam="@rs"
  drawpcl="1" />
```

## Multicam Calibration 
There two components for calibrating a stereo data acquisition system; `MultiCamCalibrator` and `MultiCamLiveCalibrator`.
Both wrap the same method for calibrating using OpenCV.
```xml
<MultiCamCalibrator
  name="calib"
  calibcam1="/path/to/folder/1/"
  calibcam2="/path/to/folder/2/"
  size="6 9"
/>
```
In this example, a realsense multicamera components instantiates virtual cameras. 
The specified `calibpath` path is used to create folders containing calibration images in generated directories.

The following snippet instantiates two realsense sensors and setups calibration component.
```xml
<RealSenseCam
  name="rs"
  serial="rsserialnum1"
  intrinsics="/path/to/save/intrinsics1.log"
/>

<RealSenseCam
  name="rs"
  serial="rsserialnum2"
  intrinsics="/path/to/save/intrinsics2.log"
/>

<MultiCamLiveCalibrator
  name="calib"
  imgmaster="@rs.rsframe"
  imgslave="@rs2.rsframe"
  size="6 9"
/>

<ProjectionMatrixExport
  name="exporter"
  rotation="@calib.rotation"
  translation="@calib.translation"
  filename="/path/to/file.txt"
/>
```

The resulting rotation/translation are stored in the appropriate sofa data, which are `@calib.rotation` and `@calib.translation`.
These data can be fed to any Realsense Reprojector, respectively through `rotation` and `offset` labels.
The result is exported to a file for usage in another scene.
