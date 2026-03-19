// ********************************************************************
// *            ControlFuncs.h
// ********************************************************************
// ********************************************************************
// * 
// * THIS INFORMATION IS PROPRIETARY TO INVENSYS Powerware Corporation
// * 
// ********************************************************************
// *                                                                        
// *    Copyright (c) 1999, 2000, 2001, 2002, 2003 Invensys Powerware                        
// *                      ALL RIGHTS RESERVED                              
// *                                                                       
// ********************************************************************
// ********************************************************************
// *     FILE NAME:   algos.h
// *                                                                      
// *     DESCRIPTION: C-header for for assembly language algorithms
// *                  
// *     ORIGINATOR:  Kevin VanEyll                                         
// *                                                                      
// *     DATE:        3/12/2004                                            
// *                                                                      
// *     HISTORY:     See visual source safe history                                                    
// ********************************************************************
#ifndef CONTROLFUNC_H
#define CONTROLFUNC_H

extern void copyIIRGainsOnly(stSecondOrderIIRFP* dst, stSecondOrderIIRFP const & src);
extern void phaseAdjustableResonantCompensator(complex<float>* fpole, complex<float>* fzero, float frequency, float phase, float inversePeakGain);
extern void matchedZTransform(stSecondOrderIIRFP* ret, float fline, complex<float> fpole, complex<float> fzero, float dcGain);


#endif
// ********************************************************************
// *            END OF algos.h   
// ********************************************************************
