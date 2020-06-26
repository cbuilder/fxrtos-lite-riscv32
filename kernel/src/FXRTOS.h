#ifndef _FXRTOS_STANDARD_RV32I_GNU_HEADER_
#define _FXRTOS_STANDARD_RV32I_GNU_HEADER_

/** 
  ******************************************************************************
  *  @file   standard-rv32i-gnu.h
  *  @brief  Kernel options.
  ******************************************************************************
  *  Copyright (C) JSC EREMEX, 2008-2020.
  *  Redistribution and use in source and binary forms, with or without 
  *  modification, are permitted provided that the following conditions are met:
  *  1. Redistributions of source code must retain the above copyright notice,
  *     this list of conditions and the following disclaimer.
  *  2. Redistributions in binary form must reproduce the above copyright 
  *     notice, this list of conditions and the following disclaimer in the 
  *     documentation and/or other materials provided with the distribution.
  *  3. Neither the name of the copyright holder nor the names of its 
  *     contributors may be used to endorse or promote products derived from 
  *     this software without specific prior written permission.
  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
  *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
  *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
  *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
  *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  *  POSSIBILITY OF SUCH DAMAGE.
  *****************************************************************************/

#include FX_INTERFACE(HAL_INIT)
#include FX_INTERFACE(HAL_CPU_INTR)
#include FX_INTERFACE(FX_TIMER)
#include FX_INTERFACE(FX_THREAD)
#include FX_INTERFACE(FX_DPC)
#include FX_INTERFACE(FX_SEM)
#include FX_INTERFACE(FX_MUTEX)
#include FX_INTERFACE(FX_MSGQ)
#include FX_INTERFACE(FX_BLOCK_POOL)
#include FX_INTERFACE(FX_EV_FLAGS)
#include FX_INTERFACE(FX_RWLOCK)
#include FX_INTERFACE(FX_COND)
#include FX_INTERFACE(FX_MEM_POOL)

FX_METADATA(({ interface: [FXRTOS, STANDARD_RV32I_GNU] }))

#endif

