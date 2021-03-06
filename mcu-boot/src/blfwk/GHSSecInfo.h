/*
 * The Clear BSD License
 * Copyright (c) 2013-14, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#if !defined(_GHSSecInfo_h_)
#define _GHSSecInfo_h_

#include "StELFFile.h"
#include "smart_ptr.h"

namespace blfwk
{
/*!
 * \brief Wrapper around the GHS-specific .secinfo ELF section.
 *
 * ELF files produced by the Green Hills MULTI toolset will have a
 * special .secinfo section. For the most part, this section contains
 * a list of address
 * ranges that should be filled by the C runtime startup code. The
 * address ranges correspond to those of ELF sections whose type is
 * #SHT_NOBITS. The GHS runtime uses this table instead of just filling
 * all #SHT_NOBITS sections because the linker command file can
 * be used to optionally not fill individual sections.
 *
 * The isSectionFilled() methods let calling code determine if an ELF
 * section is found in the .secinfo table. If the section is found,
 * then it should be filled.
 */
class GHSSecInfo
{
public:
    //! \brief Default constructor.
    GHSSecInfo(StELFFile *elf);

    //! \brief Returns true if there is a .secinfo section present in the ELF file.
    bool hasSecinfo() const { return m_hasInfo; }
    //! \brief Determines if a section should be filled.
    bool isSectionFilled(uint32_t addr, uint32_t length);

    //! \brief Determines if \a section should be filled.
    bool isSectionFilled(const Elf32_Shdr &section);

protected:
#pragma pack(1)

    /*!
     * \brief The structure of one .secinfo entry.
     */
    struct ghs_secinfo_t
    {
        uint32_t m_clearAddr;       //!< Address to start filling from.
        uint32_t m_clearValue;      //!< Value to fill with.
        uint32_t m_numBytesToClear; //!< Number of bytes to fill.
    };

#pragma pack()

protected:
    StELFFile *m_elf; //!< The parser object for our ELF file.
    bool m_hasInfo;   //!< Whether .secinfo is present in the ELF file.
    smart_array_ptr<ghs_secinfo_t>
        m_info; //!< Pointer to the .secinfo entries. Will be NULL if there is no .secinfo section in the file.
    unsigned m_entryCount; //!< Number of entries in #m_info.
};

}; // namespace blfwk

#endif // _GHSSecInfo_h_
