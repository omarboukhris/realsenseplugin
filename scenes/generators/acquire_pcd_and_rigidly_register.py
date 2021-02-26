import sys, os
import numpy as np
import Sofa
import Sofa.Gui
import SofaRuntime


class Liver (Sofa.Core.Controller):
    def __init__(self, node, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.createGraph(node)

    def createGraph(self, root):
        root.addObject(
            'RealSenseCam',
            name="rs",
            alpha="0.8",
            delta="60",
            depthScale="6",
            intrinsics=outdir + "/../calib/intrinsics.log",
        )
        root.addObject(
            'RealSenseDataFrame2ImageData',
            name="rs2color",
            rsframe="@rs.rsframe",
        )
        kwargs = {'name': 'gauss0', 'in': '@rs2color.image', 'kernelsize': '23'}
        root.addObject('OpenCVFilter2D', **kwargs)

        root.addObject(
            'ProjectionMatrixImport',
            name="mvimp",
            filename=outdir + "/../calib/modelview.txt",
        )
        root.addObject(
            'OpenCVProjectiveViewer',
            name='viewer',
            drawCamera='true',
            image='@gauss0.out',
            projectionMatrix="@mvimp.modelView",
            drawscreenPosition='10 10',
            drawscreenSize='200 200',
            drawcolor="1 1 1 1",
            depth="1.5",
            drawzNear="0.001",
            drawzFar="100",
            drawViewPort="false",
            drawswapMainView="true",
        )
        root.addObject(
            'OpenCVProjectionToCorners',
            name="mv2c",
            resolution="@rs.resolution",
            projectionMatrix="@viewer.modelView",
            depth="@viewer.depth",
        )
        root.addObject(
            'OpenCVProjectivePointSelector',
            name="ptselector",
            corners="@mv2c.corners",
            image="@gauss0.out",
        )

#  /root/detector3D
        detector3D = root.addChild("detector3D", activated="1")
        kwargs = {'name': "opk",
                  'in': '@../ptselector.contour',
                  'imageCurr': '@../gauss0.out',
                  'corners': '@../mv2c.corners',
                  'draw_pts': '1',
                  'winsize': '16 16',
                  'iter': '2',
                  'level': '10',
                  }
        detector3D.addObject('OpenCVOpticalFlowTracker', **kwargs)

#  /root/detector3D/surface
        surface = detector3D.addChild("surface")
        surface.addObject(
            'MaskFromContour',
            name="mfc",
            image="@../../rs.rsframe",
            contour="@../opk.out",
        )
        self.pcd = surface.addObject(
            'RealSenseMaskDeprojector',
            name="deproj",
            rsframe="@../../rs.rsframe",
            rscam="@../../rs",
            input="@mfc.mask",
            downsample="8",
            densify="8",
            drawpcl="1",
        )
        surface.addObject('Vec2Pcl', name="vecsurf", input="@deproj.output")
        surface.addObject(
            'PCLAlphaFilter',
            name="alpha",
            inpcl="@vecsurf.outpcl",
            alpha="1.",
        )
        surface.addObject('Pcl2Vec', name="filteredSurface", inpcl="@vecsurf.outpcl")
        surface.addObject(
            'MechanicalObject',
            template="Vec3d",
            name="MO",
            position="@filteredSurface.output",
        )
        surface.addObject(
            'FixedGeometry',
            name="surface",
            position="@filteredSurface.output",
            drawRadius="0.005",
            color="0 1 0 1"
        )
        surface.addObject('EngineToTopology', name="pointsTopo", position="@filteredSurface.output")
        surface.addObject('Vec2Pcl', name="vecConv", input="@deproj.synthvol")

#  /root/detector3D/contour3D
        contour3D = detector3D.addChild("contour3D")
        contour3D.addObject(
            'OpenCV2D3DConverter',
            name="detector3D",
            input="@../opk.out",
            corners="@../../mv2c.corners",
            image="@../opk.imageCurr",
            drawRadius="0.0025",
        )
        contour3D.addObject('MechanicalObject', template="Vec3d", name="MO")
        contour3D.addObject(
            'FixedGeometry',
            name="contour",
            position="@detector3D.output",
            drawRadius="0.005",
            color="0 1 0 1",
        )

#  /root/Liver
        Liver = root.addChild("Liver", activated="1")
        # Liver.addObject('StaticSolver', newton_iterations=10, printLog=True)
        Liver.addObject(
            'EulerImplicitSolver',
            vdamping="0.1",
            rayleighMass="0.1",
            rayleighStiffness="0.1",
        )
        # Liver.addObject('CGLinearSolver', iterations=1000)
        Liver.addObject(
            'PCGLinearSolver',
            iterations="20",
            tolerance="1e-9",
            preconditioners="solver",
            use_precond="true",
            update_step="1",
        )
        Liver.addObject('SparseLDLSolver', name="solver")
        Liver.addObject(
            'MeshVTKLoader',
            name="vloader",
            filename="/home/omar/Data/phantom_model/meshes/liverP8.vtu",
            scale3d="0.001 0.001 0.001",
        )
        Liver.addObject(
            'TetrahedronSetTopologyContainer',
            name="Container",
            position="@vloader.position",
            tetrahedra="@vloader.tetrahedra",
        )
        Liver.addObject('TetrahedronSetTopologyModifier', name="Modifier")
        Liver.addObject(
            'MechanicalObject',
            name="mstate",
            template="Vec3d",
            position="@vloader.position",
        )
        Liver.addObject('UniformMass', name="uniformMass", totalMass="1")
        Liver.addObject(
            'TetrahedronFEMForceField',
            name="FEM",
            youngModulus="50",
            poissonRatio="0.45",
            method="large",
        )

#  /root/Liver/Surface
        Surface = Liver.addChild('Surface')
        Surface.addObject('TriangleSetTopologyContainer', name="Container", position="@../vloader.position")
        Surface.addObject('TetrahedronSetTopologyModifier', name="Modifier")
        Surface.addObject(
            'Tetra2TriangleTopologicalMapping',
            input="@../Container",
            output="@Container",
            flipNormals="true",
        )
        Surface.addObject('MechanicalObject', name="surface_mo")
        Surface.addObject('TriangleGeometry', name="triangles", topology="@Container")
        Surface.addObject('BarycentricMapping')


#  /root/Liver/Visual
        Visual = Liver.addChild('Visual')
        Visual.addObject('TriangleSetTopologyContainer', name="Container", position="@../mstate.position")
        Visual.addObject('TetrahedronSetTopologyModifier', name="Modifier")
        Visual.addObject(
            'Tetra2TriangleTopologicalMapping',
            input="@../Container",
            output="@Container",
            flipNormals="true",
        )
        Visual.addObject(
            'OglModel',
            name="VisualModel",
            color="1.0 0.0 0.0 0.5",
            position="@../Container.position",
            triangles="@Container.triangles",
        )
        Visual.addObject('BarycentricMapping')


#  /root/rigidregistration
        rigidregistration = root.addChild('rigidregistration', activated='1')
        rigidregistration.addObject('Vec2Pcl', name="vecConv", input="@../Liver/mstate.position")
        rigidregistration.addObject(
            'MouseRotationHandler',
            name="mouse",
            input="@vecConv.outpcl",
            mo="@../Liver/mstate",
        )
        rigidregistration.addObject(
            'PCLIterativeClosestPoint',
            name="pclicp",
            source="@vecConv.outpcl",
            mo="@../Liver/mstate",
            target="@../detector3D/surface/vecConv.outpcl",
        )

    def onAnimateBeginEvent(self, value):
        print("pcd", self.pcd.output.value)

def createScene(rootNode):
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', maxIt="300", tolerance="0.001")
    rootNode.addObject(Liver(rootNode))
    return rootNode

# if __name__ == "__main__":
# if len(sys.argv) != 3:
#     print("unhandled number of arguments")
#     exit()
#	 path to exp / exp number
# outdir = sys.argv[1] + "/" + sys.argv[2]
outdir = '/home/andrea/realsense_data/test/a1'

# try:
#     os.mkdir(sys.argv[1])
# except:
#     pass
# try:
#     os.mkdir(outdir)
# except:
#     pass

# Add includes
SofaRuntime.PluginRepository.addFirstPath('/home/andrea/projects/plugins/SofaPython3/build/lib')
SofaRuntime.PluginRepository.addFirstPath('/home/andrea/projects/sofa/build/lib')
# SofaRuntime.PluginRepository.addFirstPath('/home/andrea/projects/sofa/share')
# SofaRuntime.PluginRepository.addFirstPath('/home/andrea/projects/kromagnon/build/lib')


# # Register all the common component in the factory.
SofaRuntime.importPlugin("SofaComponentAll")
SofaRuntime.importPlugin("SofaOpenglVisual")
SofaRuntime.importPlugin("SofaPython3")
SofaRuntime.importPlugin("SofaConstraint")
SofaRuntime.importPlugin("CImgPlugin")
SofaRuntime.importPlugin("SofaLoader")
SofaRuntime.importPlugin('SofaOpenglVisual')
SofaRuntime.importPlugin('SofaPreconditioner')

SofaRuntime.importPlugin('CollisionAlgorithm')
SofaRuntime.importPlugin('ConstraintGeometry')
SofaRuntime.importPlugin('registrationconstraint')
SofaRuntime.importPlugin('Registration')


SofaRuntime.importPlugin('realsenseplugin')
SofaRuntime.importPlugin('OpenCVPlugin')
SofaRuntime.importPlugin('PCLPlugin')

# SofaRuntime.importPlugin("Kromagnon")
# SofaRuntime.importPlugin('SofaCaribou')
# SofaRuntime.importPlugin('SofaSparseSolver')

root = Sofa.Core.Node()
root.dt = 0.01
root.name = 'root'
root.gravity = [0.0, 0.0, 0.0]
createScene(root)


