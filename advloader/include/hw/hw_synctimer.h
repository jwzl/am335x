/*
* hw_synctimer.h
*
* Register-level header file for SYNCTIMER
*
* Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
#ifndef HW_SYNCTIMER_H_
#define HW_SYNCTIMER_H_

#ifdef __cplusplus
extern "C"
{
#endif


/****************************************************************************************************
* Register Definitions
****************************************************************************************************/
#define SYNCTIMER_SYSCNT_REV                                    (0x0U)
#define SYNCTIMER_SYSCONFIG                                     (0x4U)
#define SYNCTIMER_CR                                            (0x10U)

/****************************************************************************************************
* Field Definition Macros
****************************************************************************************************/

#define SYNCTIMER_SYSCNT_REV_CID_REV_SHIFT                      (0U)
#define SYNCTIMER_SYSCNT_REV_CID_REV_MASK                       (0x000000ffU)

#define SYNCTIMER_SYSCONFIG_IDLEMODE_SHIFT                      (3U)
#define SYNCTIMER_SYSCONFIG_IDLEMODE_MASK                       (0x00000018U)
#define SYNCTIMER_SYSCONFIG_IDLEMODE_FORCE_IDLE                 (0U)
#define SYNCTIMER_SYSCONFIG_IDLEMODE_NO_IDLE                    (1U)

#define SYNCTIMER_CR_CTR_LO_SHIFT                               (0U)
#define SYNCTIMER_CR_CTR_LO_MASK                                (0x0000ffffU)

#define SYNCTIMER_CR_CTR_HI_SHIFT                               (16U)
#define SYNCTIMER_CR_CTR_HI_MASK                                (0xffff0000U)

#ifdef __cplusplus
}
#endif

#endif /* HW_SYNCTIMER_H_ */
