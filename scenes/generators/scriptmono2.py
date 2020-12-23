script="""
<?xml version="1.0"?>
<Node gravity="0 0 0" dt="0.01" >
    <RequiredPlugin pluginName='SofaOpenglVisual'/>
	<RequiredPlugin pluginName='SofaPreconditioner'/>
	<RequiredPlugin name="OpenCVPlugin" pluginName="OpenCVPlugin" />
	<RequiredPlugin name="CollisionAlgorithm" pluginName="CollisionAlgorithm" />
	<RequiredPlugin name="ConstraintGeometry" pluginName="ConstraintGeometry" />
	<RequiredPlugin name="registrationconstraint" pluginName="registrationconstraint" />

	<RequiredPlugin name="realsenseplugin" pluginName="realsenseplugin" />
	<RequiredPlugin name="PCLPlugin" pluginName="PCLPlugin" />

<!-- 	<RequiredPlugin name="Optimus" pluginName="Optimus" /> -->

    <FreeMotionAnimationLoop />
	<GenericConstraintSolver maxIt="300" tolerance="0.001"/>

    <RealSenseCam
	    name="rs"
		alpha="0.8"
		delta="60"
		depthScale="6"
		intrinsics="{outdir}/intrinsics.log"
	/>

    <RealSenseDataFrame2ImageData
	    name="rs2color"
		rsframe="@rs.rsframe"
	/>

    <OpenCVFilter2D
	    name="gauss0"
		in="@rs2color.image"
		kernelsize="23"
	/>

    <ProjectionMatrixImport
	    name="mvimp"
		filename="{outdir}/modelview.txt"
	/>
	<OpenCVProjectiveViewer
	    name='viewer'
		drawCamera='true'
		image='@gauss0.out'
		projectionMatrix="@mvimp.modelView"
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
		image="@gauss0.out"
	/>

    <Node name="gelMarkers" activated="1">
	    <OpenCVOpticalFlowTracker
		    name="opk"
			in="@../ptselector.markers"
			imageCurr="@../gauss0.out"
			corners="@../mv2c.corners"
			draw_pts="1"
			winsize="16 16"
			iter="2"
			level="10" />
		<RealSensePointDeprojector
		    name="deproj"
			input="@opk.out"
			rsframe="@../rs.rsframe"
			rscam="@../rs"
			drawpcl="1" />
		<MechanicalObject template="Vec3d" name="MO" />
		<FixedGeometry name="markers" position="@deproj.output" drawRadius="0.005" color="0 1 0 1" />
		<EngineToTopology name="pointsTopo" position="@deproj.output" />
	</Node>

    <Node name="detector3D" activated="1">
	    <OpenCVOpticalFlowTracker
		    name="opk"
			in="@../ptselector.contour"
			imageCurr="@../gauss0.out"
			corners="@../mv2c.corners"
			draw_pts="0"
			winsize="16 16"
			iter="2"
			level="10" />

        <!-- 		whole surface -->
		<Node name="surface" >
		    <MaskFromContour
			    name="mfc"
				image="@../../rs.rsframe"
				contour="@../opk.out" />
            <RealSenseDataFrame2ImageData
                rsframe="@mfc.mask"
            />
			<RealSenseMaskDeprojector
			    name="deproj"
				rsframe="@../../rs.rsframe"
				rscam="@../../rs"
				input="@mfc.mask"
				downsample="8"
				densify="8"
				drawpcl="1" />
			<Vec2Pcl name="vecsurf" input="@deproj.output" />
			<PCLAlphaFilter
			    name="alpha"
				inpcl="@vecsurf.outpcl"
				alpha="1."
			/>
			<Pcl2Vec name="filteredSurface" inpcl="@vecsurf.outpcl" />

            <MechanicalObject template="Vec3d" name="MO" position="@filteredSurface.output" />
			<FixedGeometry name="surface" position="@filteredSurface.output" drawRadius="0.005" color="0 1 0 1" />
			<EngineToTopology name="pointsTopo" position="@filteredSurface.output" />
			<Vec2Pcl name="vecConv" input="@deproj.synthvol" />
		</Node>

        <!-- 		contour only -->
		<Node name="contour3D" >
		    <OpenCV2D3DConverter
			    name="detector3D"
				input="@../opk.out"
				corners="@../../mv2c.corners"
				image="@../opk.imageCurr"
				drawRadius="0.0025" />

            <MechanicalObject template="Vec3d" name="MO" />
			<FixedGeometry name="contour" position="@detector3D.output" drawRadius="0.005" color="0 1 0 1" />
			<EngineToTopology name="pointsTopo" position="@detector3D.output" />
		</Node>
	</Node>

    <Node name="exporters">
	    <RealSenseExporter
		    name="export_all_1"
			rsframe="@../rs.rsframe"
			pathcolor="{outdir}/color1.mov"
			pathdepth="{outdir}/depth1.mov"
		/>

        <RealSenseExporter
		    name="export_markers"
			rsframe="@../gelMarkers/deproj.rsframe_out"
			pathpcl="{outdir}/pointcloudMarkers.pcl"
		/>

        <RealSenseExporter
		    name="export_surface1"
			rsframe="@../detector3D/surface/deproj.rsframe_out"
			pathpcl="{outdir}/pointcloudSurface1.pcl"
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

	fs = open(outdir + "/exp.scn", "w")
	fs.write(scriptgen)
	fs.close()
	
	os.system("/home/sperry/projects/sofa-build/bin/runSofa {outdir}/exp.scn".format(outdir=outdir))
