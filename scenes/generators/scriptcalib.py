script="""
<?xml version="1.0"?>
<Node name="root" dt="0.01" gravity="0 0 0">
    <RequiredPlugin pluginName="OpenCVPlugin" />
	<RequiredPlugin pluginName="sofascheduler" />
	<RequiredPlugin pluginName="realsenseplugin" />
	<RequiredPlugin pluginName="OptiTrackPlugin" />
	<RequiredPlugin name="PCLPlugin" pluginName="PCLPlugin" />
	<RequiredPlugin pluginName="SofaConstraint" />

    <FreeMotionAnimationLoop />
	<GenericConstraintSolver maxIt="300" tolerance="0.001"/>

<!--     <ConnectToOptiTrack name="motiveData"  listening="1"  drawScale="0" /> -->

    <RealSenseCam
	    name="rs"
		depthMode="1"
		depthScale="6"
		intrinsics="{outdir}/intrinsics.log"
	/>
	<RealSenseDataFrame2ImageData
	    name="rs2color"
		rsframe="@rs.rsframe"
	/>
	<OpenCVProjectiveViewer
	    name='viewer'
		drawCamera='true'
		image='@rs2color.image'
		drawscreenPosition='10 10'
		drawscreenSize='200 200'
		drawcolor="1 1 1 1"
		depth="1.5"
		drawzNear="0.001"
		drawzFar="100"
		drawViewPort="false"
		drawswapMainView="true"
	/>

    <OpenCVProjectionToCorners
	    name="mv2c"
		resolution="@rs.resolution"
		projectionMatrix="@viewer.modelView"
		depth="@viewer.depth"
	/>

    <OpenCVProjectivePointSelector
	    name="ptselector"
		corners="@mv2c.corners"
		image="@rs2color.image"
	/>

    <RealSensePointDeprojector
	    name="ptdeproj"
		input="@ptselector.all"

        rsframe="@rs.rsframe"
		rscam="@rs"

        downsample="5"
		drawpcl="1"
	/>
	<Node name="calibration">

        <OpenCVProjectiveCalibrator
		    name="calib"
			streamer="@../rs2color.image"
		/>
<!--		intrinsicParameters="@../rs.intrinsicParameters"-->

        <OpenCVDynamicCalibrator
		    name="dyncal"
			calibrator="@calib"
			viewer="@../viewer"
			markers2D="@../ptselector.all"
			markers3D="@../ptdeproj.output"
		/>

        <ProjectionMatrixExport
		    name="export"
			modelView="@calib.projectionMatrix"
			filename="{outdir}/modelview.txt"
		/>

    </Node>

</Node>
"""

import sys, os

# arguments are :
#	- path to experiment directory
#	- experiment id/path in directory

if __name__ == "__main__" :
	if len(sys.argv) != 3 :
		print ("unhandled number of arguments")
		exit()
	#	 path to exp / exp number
	outdir = sys.argv[1] + "/" + sys.argv[2]

	try : os.mkdir(sys.argv[1])
	except : pass
	try : os.mkdir(outdir)
	except : pass

	scriptgen = script.format (outdir=outdir)

	fs = open(outdir + "/calib.scn", "w")
	fs.write(scriptgen)
	fs.close()
	
	os.system("/home/andrea/projects/sofa/build/bin/runSofa {outdir}/calib.scn".format(outdir=outdir))

