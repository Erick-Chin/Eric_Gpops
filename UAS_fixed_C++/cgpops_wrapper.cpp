// Copyright (c) Yunus M. Agamawi and Anil Vithala Rao.  All Rights Reserved
//
// cgpops_main_test.cpp
// Test Main
// Test CGPOPS functions
//

#include "nlpGlobVarDec.hpp"
#include "cgpopsAuxDec.hpp"
#include "cgpops_main.hpp"


int main(void)
{
    printf("\nCGPOPS TESTING\n\n\n");
    
    int numTestRuns = 1;
    int numDS = 1;
    int sparseCD = 0;
    doubleMatMat cgpopsResultsMatMat(numTestRuns*numDS);
    doubleMat cgpopsResults;
    
    // Derivative supplier settings
    derivativeSupplierG = 0;    // Derivative supplier (default=0)
    scaledG             = 1;    // Scaling flag (default 1=on) (selects if automatic scaling is used for nlp)(setup.scales.method)
    
    // Mesh initialization settings
    int ones[10]   = {[0 ... 9] = 1}; //ones function creates a size 10 array filled with 1s
    numintervalsG   = 10;   // Initial number of mesh intervals per phase (default=10)
    initcolptsG     = 4*ones;    // Initial number of collocation points per interval
                            // (default=4)
    
    // Mesh refinement settings
    meshRefineTypeG = 1;    // Select mesh refinement technique to be used (default=1)  (This is a defult)
    minColPtsG      = 4;    // Mininum number of collocation points used in an interval (mesh.colpointsmin = 4)

    maxColPtsG      = 10;   // Maximum number of collocation points used in an interval (mesh.colpointesmax = 10)

    maxMeshIterG    = 45;   // Maximum number of mesh iterations    (mesh.maxiterations = 45)
    meshTolG        = 1e-6; // Mesh tolerance (default=1e-7)        (mesh.tolerance = le-6)
    
    // Output save settings
    saveIPOPTFlagG       = 1;   // Save IPOPT solution (default=1)
    saveMeshRefineFlagG  = 0;   // Save mesh refinement history (default=0)
    saveHamiltonianG     = 0;   // Save Hamiltonian values (default=0)
    saveLTIHG            = 0;   // Save linear terms in Hamiltonian values (default=0)
    
    // IPOPT settings
    runIPOPTFlagG   = 1;    // Run IPOPT (default=1)
    NLPtolG         = 1e-5; // NLP Solver tolerance (default=1e-7)
    NLPmaxiterG     = 3000; // Maximum number of iterations allowed for NLP solver
    useLTIHDDG = 1;
//    varMeshPtsFracG = 0.05;
//    varMeshPtsNumSegsG = 2;
    varMeshPtsNumColPtsG = 6;
    
    /*-----------------------Changes to global parameter settings-----------------------*/
    
    for (int ds=0; ds<numDS; ds++)
    {
        derivativeSupplierG = sparseCD;
        if (derivativeSupplierG==sparseCD)
//        derivativeSupplierG = ds;
//        if (derivativeSupplierG==ds)
        {
            for (int ni=0; ni<numTestRuns; ni++)
            {
                cgpopsResultsMatMat.mat[ni+ds*numTestRuns] = getDoubleMat(6);
                printf("\nDerivativeSupplier = %d",derivativeSupplierG);
//                numintervalsG = pow(2,ni+1);
                printf("\nNumIntervals = %d",numintervalsG);
                cgpops_go(cgpopsResults);
                cgpopsResultsMatMat.mat[ni+ds*numTestRuns].val[0] = derivativeSupplierG;
                cgpopsResultsMatMat.mat[ni+ds*numTestRuns].val[1] = numintervalsG;
                cgpopsResultsMatMat.mat[ni+ds*numTestRuns].val[2] = cgpopsResults.val[0];
                cgpopsResultsMatMat.mat[ni+ds*numTestRuns].val[3] = cgpopsResults.val[1];
                cgpopsResultsMatMat.mat[ni+ds*numTestRuns].val[4] = cgpopsResults.val[2];
                cgpopsResultsMatMat.mat[ni+ds*numTestRuns].val[5] = cgpopsResults.val[3];
            }
        }
    }
    
    printf4MSCRIPT("cgpopsResultsMatMat",cgpopsResultsMatMat);
    printf("\n\n\n\n\n");
    
    return 0;
}



