#include "ProjectionMatrixIO.h"
#include <sofa/core/visual/DrawTool.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/VecTypes.h>
namespace sofa
{

namespace realsenseplugin {

namespace utils
{


SOFA_DECL_CLASS(ProjectionMatrixExport)

int ProjectionMatrixExportClass = core::RegisterObject("export model view matrix")
.add< ProjectionMatrixExport >()
;


SOFA_DECL_CLASS(ProjectionMatrixImport)

int ProjectionMatrixImportClass = core::RegisterObject("load model view matrix")
.add< ProjectionMatrixImport >()
;

} // namespace exporter

} // namespace opencvplugin

} // namespace sofa
