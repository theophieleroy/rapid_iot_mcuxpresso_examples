/*
 * Copyright (c) 2018 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
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

/*!
 * @file img_bmp_callbacks.h
 * This is the header file for BMP Image callbacks (emWin)
 */

#ifndef _IMG_BMP_CALLBACKS_H_
#define _IMG_BMP_CALLBACKS_H_

/*!
 * @addtogroup img_bmp_callbacks Image BMP Callbacks
 *
 * The img_bmp_callbacks module provdes a callback function that facilitates loading BMP images
 * directly from External SPI Flash using GUI_BMP_DrawEx function.
 *
 * Dependencies
 * -------------------------------------------------------------------------------------------------
 * The img_bmp_callbacks module depends on the following:
 *      - img_program_ext
 *      - emWin library
 *          + NOTE: Designed specifically for use with emWin GUI_BMP_DrawEx.
 *
 * Usage
 * -------------------------------------------------------------------------------------------------
 * @code
 *
 *      // Image address in External SPI Flash 
 *      // See img_program_ext & ui_manager modules for storing images
 *      uint32_t imgBaseAddr = 0x00C00000;
 *
 *      // Draw image from External memory
 *      GUI_BMP_DrawEx(RPK_GUI_get_data, &imgBaseAddr, 0, 0);
 *
 * @endcode
 *
 * @{
 * @brief Callback function to be used with emWin GUI_BMP_DrawEx function to draw BMP from External SPI Flash
 */

/*
 * @brief Callback to read image data out of flash and onto display
 *
 * @param extBaseAddr Pointer to image base address in external memory
 * @param ppData      Reference to pointer used by emWin library to load image onto display
 * @param numBytes    Number of bytes to read from external memory
 * @param offset      Address offset for next chunk of image data
 *
 * @return Returns the number of bytes read. A mismatch with numBytes will indicate and error to emWin.
 *
 */
int RPK_GUI_get_data(void *extBaseAddr, const unsigned char **ppData, unsigned numBytes, long unsigned offset);

/*! @}*/

#endif /* _IMG_BMP_CALLBACKS_H_ */