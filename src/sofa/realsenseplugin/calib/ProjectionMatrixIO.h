#pragma once

#include <opencv2/highgui.hpp>
#include <sofa/core/objectmodel/DataFileName.h>
#include <SofaUserInteraction/Controller.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/helper/vector.h>
#include <fstream>
#include <iostream>


namespace sofa {

namespace realsenseplugin {

namespace utils {

using sofa::core::objectmodel::DataFileName ;


class ProjectionMatrixExport :  public sofa::core::objectmodel::BaseObject {
public:

    SOFA_CLASS(ProjectionMatrixExport, sofa::core::objectmodel::BaseObject) ;

    DataFileName f_filename;
    Data<defaulttype::Mat3x4d > d_extrinsecMatrix;
    core::objectmodel::DataCallback d_export ;

    Data<defaulttype::Mat3x3 > d_rotation;
    Data<defaulttype::Vector4 > d_translation;
    core::objectmodel::DataCallback d_exportsplit ;

    ProjectionMatrixExport()
    : f_filename(initData (&f_filename, "filename", "file name"))
    , d_extrinsecMatrix(initData(&d_extrinsecMatrix,"modelView", "extrinsec matrix calibration"))
    , d_rotation(initData(&d_rotation,"rotation", "rotation matrix"))
    , d_translation(initData(&d_translation,"translation", "translation vector"))
    {
        d_export.addInputs({&d_extrinsecMatrix});
        d_export.addCallback(std::bind(&ProjectionMatrixExport::exportMat, this));
        d_exportsplit.addInputs({&d_rotation, &d_translation});
        d_exportsplit.addCallback(std::bind(&ProjectionMatrixExport::export_split, this));
    }

    void exportMat () {
        std::ofstream wFile;
        wFile.open(f_filename.getValue());
        if (wFile.is_open()) {
            defaulttype::Mat3x4d extrinsecMatrix = d_extrinsecMatrix.getValue();
            for (size_t i = 0 ; i < 3 ; i++) {
                for (size_t j = 0 ; j < 4 ; j++) {
                    wFile << extrinsecMatrix[i][j] << " " ;
                }
            }
            wFile.close();
        }
    }

    void export_split () {
        std::ofstream wFile;
        wFile.open(f_filename.getValue());
        if (wFile.is_open()) {
            defaulttype::Mat3x3 rotation = d_rotation.getValue();
            defaulttype::Vector4 translation = d_translation.getValue();
            for (size_t i = 0 ; i < 3 ; i++) {
                for (size_t j = 0 ; j < 3 ; j++) {
                    wFile << rotation[i][j] << " " ;
                }
                wFile << translation[i] << " " ;
            }
            wFile.close();
        }
    }

};


class ProjectionMatrixImport :  public sofa::core::objectmodel::BaseObject {
public:

    SOFA_CLASS(ProjectionMatrixImport, sofa::core::objectmodel::BaseObject) ;

    DataFileName f_filename;
    Data<defaulttype::Mat3x4d > d_extrinsecMatrix;
    core::objectmodel::DataCallback d_import ;

    Data<defaulttype::Mat3x3 > d_rotation;
    Data<defaulttype::Vector4 > d_translation;
    core::objectmodel::DataCallback d_importsplit ;

    ProjectionMatrixImport()
    : f_filename(initData (&f_filename, "filename", "file name"))
    , d_extrinsecMatrix(initData(&d_extrinsecMatrix,"modelView", "extrinsec matrix calibration"))
    {
        d_import.addInputs({&f_filename});
        d_import.addCallback(std::bind(&ProjectionMatrixImport::load, this));
        d_importsplit.addInputs({&f_filename});
        d_importsplit.addCallback(std::bind(&ProjectionMatrixImport::load_split, this));
    }

    void init () {
        load() ;
    }

    void load () {
        std::ifstream wFile;
        wFile.open(f_filename.getValue());
        if (wFile.is_open()) {
            defaulttype::Mat3x4d extrinsecMatrix;
            for (size_t i = 0 ; i < 3 ; i++) {
                for (size_t j = 0 ; j < 4 ; j++) {
                    wFile >> extrinsecMatrix[i][j];
                }
            }
            wFile.close();
            std::cout << extrinsecMatrix << std::endl ;
            d_extrinsecMatrix.setValue(extrinsecMatrix);
        }
    }

    void load_split () {
        std::ifstream wFile;
        wFile.open(f_filename.getValue());
        if (wFile.is_open()) {
            defaulttype::Mat3x3 rotation;
            defaulttype::Vector4 translation ;
            for (size_t i = 0 ; i < 3 ; i++) {
                for (size_t j = 0 ; j < 3 ; j++) {
                    wFile >> rotation[i][j];
                }
                wFile >> translation[i];
            }
            wFile.close();
            std::cout << "rotation " << rotation << std::endl
                      << "translation " << translation << std::endl ;
            d_rotation.setValue(rotation);
            d_translation.setValue(translation);
        }
    }

};

}

}

}
