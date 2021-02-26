script="""
<?xml version="1.0"?>
<Node gravity="0 0 0" dt="0.01" >
    <RequiredPlugin pluginName='SofaOpenglVisual'/>
	<RequiredPlugin pluginName='SofaPreconditioner'/>
	<RequiredPlugin name="OpenCVPlugin" pluginName="OpenCVPlugin" />
	<RequiredPlugin name="CollisionAlgorithm" pluginName="CollisionAlgorithm" />
	<RequiredPlugin name="ConstraintGeometry" pluginName="ConstraintGeometry" />
	<RequiredPlugin name="registrationconstraint" pluginName="registrationconstraint" />
	<RequiredPlugin pluginName='SofaConstraint'/> 
    <RequiredPlugin pluginName='SofaImplicitOdeSolver'/> 
    <RequiredPlugin pluginName='SofaLoader'/> 
    <RequiredPlugin pluginName='SofaSimpleFem'/> 
    <RequiredPlugin pluginName='SofaSparseSolver'/>
    <RequiredPlugin pluginName='SofaTopologyMapping'/>
    
    <RequiredPlugin name="realsenseplugin" pluginName="realsenseplugin" />
	<RequiredPlugin name="PCLPlugin" pluginName="PCLPlugin" />

	<FreeMotionAnimationLoop />
	<GenericConstraintSolver maxIt="300" tolerance="0.001"/>

    <RealSenseCam
	    name="rs"
		alpha="0.8"
		delta="60"
		depthScale="6"
		intrinsics="{outdir}/../calib/intrinsics.log"
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
		filename="{outdir}/../calib/modelview.txt"
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
		draw="true"
	/>

    <Node name="gelMarkers" activated="1">
	    <OpenCVOpticalFlowTracker
		    name="opk"
			in="@../ptselector.markers"
			imageCurr="@../gauss0.out"
			corners="@../mv2c.corners"
			draw_pts="0"
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
			draw_pts="1"
			winsize="16 16"
			iter="2"
			level="10" />

        <!-- 		whole surface -->
		<Node name="surface" >
		    <MaskFromContour
			    name="mfc"
				image="@../../rs.rsframe"
				contour="@../opk.out" />
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

    <Node name="Liver" activated="1">
    	<MeshObjLoader name="surface_mesh2" filename="/home/andrea/data/liver_phantom_BOPA/ROI_02_03.obj" scale3d="0.0009 0.0009 0.0009"/>
    	<MeshObjLoader name="surface_mesh3" filename="/home/andrea/data/liver_phantom_BOPA/ROI_02_05.obj" scale3d="0.0009 0.0009 0.0009"/>
    	<MeshObjLoader name="surface_mesh4" filename="/home/andrea/data/liver_phantom_BOPA/ROI_02_07.obj" scale3d="0.0009 0.0009 0.0009"/>
    	<MeshObjLoader name="surface_mesh5" filename="/home/andrea/data/liver_phantom_BOPA/ROI_00_02.obj" scale3d="0.0009 0.0009 0.0009"/>
		<MeshObjLoader name="surface_mesh6" filename="/home/andrea/data/liver_phantom_BOPA/ROI_01_04.obj" scale3d="0.0009 0.0009 0.0009"/>

	    <EulerImplicitSolver vdamping="0" rayleighMass="0.1" rayleighStiffness="0.1" />

        <PCGLinearSolver iterations="20" tolerance="1e-9" preconditioners="solver" use_precond="true" update_step="1" />

        <SparseLDLSolver name="solver"/>

        <MeshVTKLoader
		    name="vloader"
			filename="/home/andrea/data/liver_phantom_BOPA/liver.vtk"
			scale3d="0.0009 0.0009 0.0009"
		/>

        <TetrahedronSetTopologyContainer name="Container" position="@vloader.position" tetrahedra="@vloader.tetrahedra" />
		<TetrahedronSetTopologyModifier name="Modifier"/>

        <MechanicalObject name="mstate" template="Vec3d" position="@vloader.position" />

        <UniformMass name="uniformMass" totalMass="1" />
		<TetrahedronFEMForceField name="FEM" youngModulus="50000" poissonRatio="0.35" method="svd" />

        <Node name="Surface">
		    <TriangleSetTopologyContainer name="Container" position="@../vloader.position"  />
			<TetrahedronSetTopologyModifier name="Modifier"/>

            <Tetra2TriangleTopologicalMapping input="@../Container" output="@Container" flipNormals="true"/>
			<MechanicalObject name="dofs" />

            <TriangleGeometry name="triangles" topology="@Container" />
			<PhongTriangleNormalHandler geometry="@triangles" />

            <AABBBroadPhase
			    name="AABBBroadPhase"
				geometry="@triangles"
				nbox="8 8 8"
				color="1 0 0 0.3" />

            <BarycentricMapping />
		</Node>

        <Node name="markersPosOnGel" >
		    <VisualStyle displayFlags="showCollisionModels" />
			<PointSetTopologyContainer name="Container" />
			<PointSetTopologyModifier name="ModifierPoint"  />
			<MechanicalObject name="dofs" position="" />
			<PointGeometry name="markersOngel" drawRadius="0.000" />
			<BarycentricMapping />
		</Node >

        <UpdateStateAfterRegistration
		    name="saved"
			pointTopo="Modifier"
			pathToOptkTopo="markersPosOnGel/ModifierPoint"
			pathToMstate="markersPosOnGel/dofs"
			markersPose="@../gelMarkers/deproj.output"
			useTime="0" />

        <Node name="Visual">
		    <TriangleSetTopologyContainer name="Container" position="@../mstate.position"  />
			<TetrahedronSetTopologyModifier name="Modifier"/>
			<Tetra2TriangleTopologicalMapping input="@../Container" output="@Container" flipNormals="true"/>
			<OglModel name="VisualModel" color="1.0 0.5 0.5 0.5" position="@../Container.position" triangles="@Container.triangles" />
			<BarycentricMapping />
		</Node>
		<Node name="VisualVeineCave">
			<OglModel name="VisualModel" color="0.0 0.0 1.0 1.0" src="@../surface_mesh2" />
			<BarycentricMapping />
		</Node>
		<Node name="VisualMetastases">
			<OglModel name="VisualModel" color="1.0 0.5 0.5 1.0" src="@../surface_mesh3" />
			<BarycentricMapping />
		</Node>
		<Node name="VisualMetastase2">
			<OglModel name="VisualModel" color="1.0 0.5 0.0 1.0" src="@../surface_mesh4" />
			<BarycentricMapping />
		</Node>
		<Node name="VisualArtere">
			<OglModel name="VisualModel" color="1.0 0.0 0.0 1.0" src="@../surface_mesh5" />
			<BarycentricMapping />
		</Node>
		<Node name="VisualVeinePorte">
			<OglModel name="VisualModel" color="0.5 0.0 0.5 1.0" src="@../surface_mesh6" />
			<BarycentricMapping />
		</Node>
		<LinearSolverConstraintCorrection solverName="solver" />
	</Node>

    <Node name="rigidregistration" activated="1" >
	    <Vec2Pcl name="vecConv" input="@../Liver/mstate.position" />
		<MouseRotationHandler
		    name="mouse"
			input="@vecConv.outpcl"
			mo="@../Liver/mstate" />
		<PCLIterativeClosestPoint
		    name="pclicp"
			source="@vecConv.outpcl"
			mo="@../Liver/mstate"
			target="@../detector3D/surface/vecConv.outpcl" />
	</Node>

    <Node activated="0" name="sliding">
	    <FindClosestProximityAlgorithm
		    name="algoslide"
			from="@../gelMarkers/markers"
			dest="@../Liver/Surface/triangles"
			drawcollision="0" />
		<DistanceFilter name="distance" algo="@algoslide" distance="0.6" />
		<NormalFilter name="normal" algo="@algoslide" angle="-1 0.5" />

        <BindDirection name="normals" />
		<ConstraintBilateral
		    name="sliding"
			input="@algoslide.output"
			directions="@normals"
			maxForce=".1 .1 .1"
			draw_scale="0.05" />

        <FindClosestProximityAlgorithm
		    name="algosurface"
			from="@../detector3D/surface/surface"
			dest="@../Liver/Surface/triangles"
			drawcollision="0" />
		<DistanceFilter name="distancesurf" algo="@algosurface" distance="0.6" />
		<NormalFilter name="normalsurf" algo="@algosurface" angle="-1 0.5" />

        <BindDirection name="normalssurf" />
		<ConstraintBilateral
		    name="surfaceCst"
			input="@algosurface.output"
			directions="@normalssurf"
			maxForce=".1 .1 .1"
			draw_scale="0.05" />

    </Node>

    <Node activated="0" name="pointCloud" >

        <FindClosestProximityAlgorithm
		    name="bindpointcloudAlgo"
			from="@../gelMarkers/markers"
			dest="@../Liver/markersPosOnGel/markersOngel" />

        <DistanceFilter algo="@bindpointcloud" name="distfilter" distance="0.1" />

        <FixedFrameDirection name="normals" />
		<ConstraintBilateral
		    name="bindcst"
			input="@bindpointcloudAlgo.output"
			directions="@normals"
			maxForce="0.8 0.8 0.8" />

    </Node>

    <SwapNodes
	    nodes1="sliding Surface"
		nodes2="pointCloud"
		noinit="true" />

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
	
	os.system("/home/andrea/projects/sofa/build/bin/runSofa {outdir}/exp.scn".format(outdir=outdir))
