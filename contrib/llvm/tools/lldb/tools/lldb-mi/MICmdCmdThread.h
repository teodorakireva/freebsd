//===-- MICmdCmdThread.h ----------------------------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

// Overview:    CMICmdCmdThreadInfo interface.
//
//              To implement new MI commands derive a new command class from the command base
//              class. To enable the new command for interpretation add the new command class
//              to the command factory. The files of relevance are:
//                  MICmdCommands.cpp
//                  MICmdBase.h / .cpp
//                  MICmdCmd.h / .cpp
//              For an introduction to adding a new command see CMICmdCmdSupportInfoMiCmdQuery
//              command class as an example.

#pragma once

// In-house headers:
#include "MICmdBase.h"
#include "MICmnMIValueTuple.h"
#include "MICmnMIValueList.h"

//++ ============================================================================
// Details: MI command class. MI commands derived from the command base class.
//          *this class implements MI command "thread-info".
// Gotchas: None.
// Authors: Illya Rudkin 07/03/2014.
// Changes: None.
//--
class CMICmdCmdThreadInfo : public CMICmdBase
{
    // Statics:
  public:
    // Required by the CMICmdFactory when registering *this command
    static CMICmdBase *CreateSelf(void);

    // Methods:
  public:
    /* ctor */ CMICmdCmdThreadInfo(void);

    // Overridden:
  public:
    // From CMICmdInvoker::ICmd
    bool Execute(void) override;
    bool Acknowledge(void) override;
    bool ParseArgs(void) override;
    // From CMICmnBase
    /* dtor */ ~CMICmdCmdThreadInfo(void) override;

    // Typedefs:
  private:
    typedef std::vector<CMICmnMIValueTuple> VecMIValueTuple_t;

    // Attributes:
  private:
    CMICmnMIValueTuple m_miValueTupleThread;
    bool m_bSingleThread;  // True = yes single thread, false = multiple threads
    bool m_bThreadInvalid; // True = invalid, false = ok
    VecMIValueTuple_t m_vecMIValueTuple;
    const CMIUtilString m_constStrArgNamedThreadId;
};
