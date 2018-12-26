;
;* Copyright (c) 2016, Freescale Semiconductor, Inc.
;* All rights reserved.
;*
;* Freescale Confidential and Proprietary - use of this software is
;* governed by the LA_OPT_FSL_OPEN_3RD_PARTY_IP License distributed
;* with this Material.
;*

; =============================================================================
; getRandomWord
; =============================================================================
; input:
;   r0: ptr to RNG context A, B, C, D
; output:
;   r0: random number
; globbered:
;   RNG context updated
;   registers r0 - r3 modified
; register organization:
;   r0:         ptr to RNG context A, B, C, D / retVal
;   r1 - r4:    A, B, C, D
;   r5:         const 0x9e3779b9
; =============================================================================
    
    RSEG    CODE:CODE(2)
    THUMB    
    PUBLIC   getRandomWord    

getRandomWord:
    PUSH    {r4 - r5, lr}           ; backup registers
    LDM     r0, {r1-r4}             ; r1 - r4 = A, B, C, D
    LDR     r5, =0x9e3779b9         ; 2^32 / goldenRatio
    ADD     r1, r5                  ; A += 0x9e3779b9
    ADD     r2, r1, r2, ROR #19     ; B := A + (B >>> 19)
    ADD     r3, r2, r3, ROR #19     ; C := B + (C >>> 19)
    MUL     r3, r3, r5              ; C *= 0x9e3779b9
    ADD     r4, r3, r4, ROR #19     ; D := C + (D >>> 19)
    STM     r0, {r1-r4}             ; write back A, B, C, D
    EOR     r0, r1, r2              ; retVal = A ^ B
    EOR     r0, r0, r3              ; retVal = A ^ B ^ C        ; EOR instead of EORS to avoid unnecessary leakage by CPU flags
    EOR     r0, r0, r4              ; retVal = A ^ B ^ C ^ D    ; EOR instead of EORS to avoid unnecessary leakage by CPU flags
    POP     {r4 - r5, pc}           ; return and restore registers
        

    
    
    
    
; =============================================================================
; secureCopy
; =============================================================================
; input:
;   r0: dst ptr
;   r1: src ptr
;   r2: nBytes, must be a power of 2 with nBytes <= 2^10
;   r3: random 32 bit, used for randomization
; output:
;   none
; globbered:
;   nBytes are copied from src to dst
; register organization:
;   r0:         dst ptr
;   r1:         src ptr
;   r2:         mask = nBytes - 1
;   r3 - r6:    e, s, x, r
;   r7:         temp
;   r12:        counter
; =============================================================================

    PUBLIC   secureCopy

secureCopy:
    ; init permutation
    PUSH    {r4 - r7, lr}           ; backup registers
    ROR     r4, r3, #10             ; r4 := s   ; r3 = e    ; reuse of random bits
    ROR     r5, r3, #20             ; r5 := x
    ORR     r4, r4, #1              ; s := s OR 1   ; odd s
    
    ; check for alignment
    ORR     r7, r0, r1              ; r7 := dstPtr OR srcPtr
    LSLS    r7, #30                 ; r7 contains only two lsb of dstPtr OR srcPtr
    BEQ     _secureCopyWords        ; if word aligned copy word wise
    
_secureCopyBytes
    ; prepare loop variables and mask
    SUBS    r2, #1                  ; r2 := m = nBytes - 1  ; value is known so leakage by CPU flags is ok
    MOV     r12, r2                 ; r12 := counter = nBytes - 1

; -----------------------------------------------------------------------------
_secureCopyBytes_loop               ; for i = 0, ..., p-1 do:
    ; update permutation and compute next r
    ADD     r3, r4                  ; e += s    ; value is sensitive hence no ADDS
    EOR     r6, r3, r5              ; r6 := e XOR x
    AND     r6, r6, r2              ; r6 := r = (e XOR x) AND m
    
    ; copy value from src to dst
    LDRB    r7, [r1, r6]            ; r7 = temp := byte@( srcPtr + r )
    STRB    r7, [r0, r6]            ; byte@( dstPtr + r ) := r7 = temp

    ; next iteration
    SUBS    r12, r12, #1            ; counter -= 1;
    BHS     _secureCopyBytes_loop
    B       _secureCopy_commonEnd
; -----------------------------------------------------------------------------

_secureCopyWords
    ; prepare loop variables and mask
    LSRS    r2, #2                  ; r2 = nWords = nBytes / 4  ; value is known so leakage by CPU flags is ok
    SUBS    r2, #1                  ; r2 := m = nWords - 1      ; value is known so leakage by CPU flags is ok
    MOV     r12, r2                 ; r12 := counter = nWords - 1

; -----------------------------------------------------------------------------
_secureCopyWords_loop               ; for i = 0, ..., p-1 do:
    ; update permutation and compute next r
    ADD     r3, r4                  ; e += s    ; value is sensitive hence no ADDS
    EOR     r6, r3, r5              ; r6 := e XOR x
    AND     r6, r6, r2              ; r6 := r = (e XOR x) AND m
    
    ; copy value from src to dst
    LDR     r7, [r1, r6, LSL #2]    ; r7 = temp := word@( srcPtr + r*4 )
    STR     r7, [r0, r6, LSL #2]    ; word@( dstPtr + r*4 ) := r7 = temp

    ; next iteration
    SUBS    r12, r12, #1            ; counter -= 1;
    BHS     _secureCopyWords_loop
; -----------------------------------------------------------------------------

_secureCopy_commonEnd
    POP     {r4 - r7, pc}           ; return and restore registers
    




    
    
    
    
; =============================================================================
; secureCopy_wordsInterleaved
; =============================================================================
; input:
;   r0: dst ptr
;   r1: src ptr
;   r2: nBytes, must be a power of 2 with 4 <= nBytes <= 2^10
;   r3: random 32 bit, used for randomization
; output:
;   none
; globbered:
;   nBytes are copied from src to dst wordwise
;   word@( dstPtr + 8*i ) = word@( srcPtr + 4*i ), i = 0, ..., nWords - 1; with nWords = nBytes / 4
; register organization:
;   r0:         dst ptr
;   r1:         src ptr
;   r2:         mask = nWords - 1
;   r3 - r6:    e, s, x, r
;   r7:         temp
;   r12:        counter
; =============================================================================

    PUBLIC   secureCopy_wordsInterleaved

secureCopy_wordsInterleaved:
    ; init permutation
    PUSH    {r4 - r7, lr}           ; backup registers
    ROR     r4, r3, #10             ; r4 := s   ; r3 = e    ; reuse of random bits
    ROR     r5, r3, #20             ; r5 := x
    ORR     r4, r4, #1              ; s := s OR 1   ; odd s
    
    ; prepare loop variables and mask
    LSRS    r2, #2                  ; r2 = nWords = nBytes / 4  ; value is known so leakage by CPU flags is ok
    SUBS    r2, #1                  ; r2 := m = nWords - 1      ; value is known so leakage by CPU flags is ok
    MOV     r12, r2                 ; r12 := counter = nWords - 1

; -----------------------------------------------------------------------------
_secureCopy_wordsInterleaved_loop   ; for i = 0, ..., p-1 do:
    ; update permutation and compute next r
    ADD     r3, r4                  ; e += s    ; value is sensitive hence no ADDS
    EOR     r6, r3, r5              ; r6 := e XOR x
    AND     r6, r6, r2              ; r6 := r = (e XOR x) AND m
    
    ; copy value from src to dst
    LDR     r7, [r1, r6, LSL #2]    ; r7 = temp := word@( srcPtr + r*4 )
    STR     r7, [r0, r6, LSL #3]    ; word@( dstPtr + r*8 ) := r7 = temp

    ; next iteration
    SUBS    r12, r12, #1            ; counter -= 1;
    BHS     _secureCopy_wordsInterleaved_loop
; -----------------------------------------------------------------------------

    ; return
    POP     {r4 - r7, pc}           ; return and restore registers
    
    
    
    

    
; =============================================================================
; secureCopy_wordsCombine
; =============================================================================
; input:
;   r0: dst ptr
;   r1: src ptr
;   r2: nBytes, must be a power of 2 with 4 <= nBytes <= 2^10
;   r3: random 32 bit, used for randomization
; output:
;   none
; globbered:
;   nBytes are copied from src to dst wordwise
;   word@( dstPtr + 4*i ) = word@( srcPtr + 8*i ), i = 0, ..., nWords - 1; with nWords = nBytes / 4
; register organization:
;   r0:         dst ptr
;   r1:         src ptr
;   r2:         mask = nWords - 1
;   r3 - r6:    e, s, x, r
;   r7:         temp
;   r12:        counter
; =============================================================================

    PUBLIC   secureCopy_wordsCombine

secureCopy_wordsCombine:
    ; init permutation
    PUSH    {r4 - r7, lr}           ; backup registers
    ROR     r4, r3, #10             ; r4 := s   ; r3 = e    ; reuse of random bits
    ROR     r5, r3, #20             ; r5 := x
    ORR     r4, r4, #1              ; s := s OR 1   ; odd s
    
    ; prepare loop variables and mask
    LSRS    r2, #2                  ; r2 = nWords = nBytes / 4  ; value is known so leakage by CPU flags is ok
    SUBS    r2, #1                  ; r2 := m = nWords - 1      ; value is known so leakage by CPU flags is ok
    MOV     r12, r2                 ; r12 := counter = nWords - 1

; -----------------------------------------------------------------------------
_secureCopy_wordsCombine_loop       ; for i = 0, ..., p-1 do:
    ; update permutation and compute next r
    ADD     r3, r4                  ; e += s    ; value is sensitive hence no ADDS
    EOR     r6, r3, r5              ; r6 := e XOR x
    AND     r6, r6, r2              ; r6 := r = (e XOR x) AND m
    
    ; copy value from src to dst
    LDR     r7, [r1, r6, LSL #3]    ; r7 = temp := word@( srcPtr + r*8 )
    STR     r7, [r0, r6, LSL #2]    ; word@( dstPtr + r*4 ) := r7 = temp

    ; next iteration
    SUBS    r12, r12, #1            ; counter -= 1;
    BHS     _secureCopy_wordsCombine_loop
; -----------------------------------------------------------------------------

    ; return
    POP     {r4 - r7, pc}           ; return and restore registers
    
    
    
    


    
; =============================================================================
; getRandomIndex
; =============================================================================
; input:
;   r0: n, number of possible indexes with n < 2^16
;   r1: random 32 bit, used for randomization
; output:
;   r0: random index in [0, n-1] provided as r || c with random 16-bit r and random 16-bit index c in [0, n-1]
; globbered:
;   r2
; register organization:
;   r0: n, temp and result
;   r1: random to generate c
;   r2: random for extra random bits
; =============================================================================

    PUBLIC   getRandomIndex

getRandomIndex:
    UXTH    r2, r1      ; r2 = 16-bit random
    LSRS    r1, #16     ; r1 = 16-bit random
    MUL     r0, r0, r1  ; r0 = c || ...
    EOR     r0, r0, r2  ; r0 = c || r
    ROR     r0, r0, #16 ; r0 = r || c
    BX      lr          ; return
    
   
    
    
; =============================================================================
; securePointerSelect
; =============================================================================
; input:
;   r0:         index i
;   r1:         reference index provided as r || c with random 16-bit r and random 16-bit index c
;   r2:         ptrA
;   r3:         ptrB
;   stack[0]:   random 32 bit
; output:
;   r0: (i == c) ? ptrA : ptrB in constant time and with minimal side channel leakage
; globbered:
;   none
; register organization:
;   r0: temp for computing AND of all bits of i^c with noise
;   r1: used for selecting lower half of pointers
;   r2: used for selecting higher half of pointers
;   r3: random 32 bit
; =============================================================================

    PUBLIC   securePointerSelect

securePointerSelect:
    ; prepare registers
    MVNS    r0, r0                  ; r0 = ~i               ; to avoid fixed rotations for i == c ...
    EOR     r0, r0, r1              ; r0 = ~r || (~i ^ c)
    UXTH    r1, r2                  ; r1 = ptrA_L
    ORR     r1, r1, r3, LSL #16     ; r1 = ptrB_L || ptrA_L
    BFC     r2, #0, #16             ; r2 = ptrA_H || 0
    ORR     r2, r2, r3, LSR #16     ; r2 = ptrA_H || ptrB_H
    LDR     r3, [ sp ]              ; r3 = random 32 bit
    
    ; compute AND of all bits of ~i^c with noise
    AND     r0, r0, r0, ROR #8      ; AND of 8 bit offsets
    EOR     r0, r0, r3, LSL #8      ; rerandomize
    AND     r0, r0, r0, ROR #4      ; AND of 4 bit offsets
    EOR     r0, r0, r3, LSL #4      ; rerandomize
    AND     r0, r0, r0, ROR #2      ; AND of 2 bit offsets
    EOR     r0, r0, r3, LSL #2      ; rerandomize
    AND     r0, r0, r0, ROR #1      ; AND of 1 bit offsets
    EOR     r0, r0, r3, LSL #1      ; rerandomize   ; r0 = random 31 bits || e, with e == 1 <=> i == c
    
    ; select pointer by random rotations
    ROR     r0, r0, #(32 - 3)       ; r0 <<<= 3     ; r0<3> = e
    ROR     r1, r1, #(32 - 7 - 16)  ; r1 <<<= 7 + 16
    ROR     r2, r2, #(32 - 7 - 16)  ; r2 <<<= 7 + 16
    ROR     r1, r1, r0              ; r1 >>>= ...xexxx
    ROR     r2, r2, r0              ; r2 >>>= ...xexxx
    EOR     r0, r0, #0x07           ; r0 = ...xeyyy
    ROR     r1, r1, r0              ; r1 >>>= ...xeyyy  ; r1 = (ptrB_L || ptrA_L) >>> (16*e + 16)
    ROR     r2, r2, r0              ; r2 >>>= ...xeyyy  ; r2 = (ptrA_H || ptrB_H) >>> (16*e + 16)
    
    ; r0 = ptrX with X = (e == 0) ? A : B
    MOV     r0, r2                  ; r0 = ptrX_H || ptrY_H
    BFI     r0, r1, #0, #16         ; r0 = ptrX_H || ptrX_L = ptrX

    ; return
    BX      lr                      ; return
    
    
    PUBLIC  ltc_decrementer
    
ltc_decrementer:
ltc_decrementer_loop
    SUBS    r0, r0, #1 
    CMP     r0, #0 
    BNE     ltc_decrementer_loop 
    BX      lr    
    
    PUBLIC  ltc_gate_off

; =============================================================================
; ltc_gate_off
; =============================================================================
; input:
;   r0:         SCGC2 register address
;   r1:         r32 random value
;   r2:         number of repeats
;   r3:         number of inner cycles (LTC clock off counter)
; output:
;   r0:         none
; globbered:
;   none
; register organization:
;   r0: 
;   r1:         random 32-bit
;   r2: 
;   r2: 
; =============================================================================
ltc_gate_off:
    CMP         r2, #0
    BEQ         ltc_gate_off_exit
    
    PUSH        {r4-r6, r8}
    MOV         r8, r3
    MOV         r3, r1
    
ltc_gate_off_outer_loop:

    MOV         r6, r8
    
    ; LTC clock off
    CPSID       i
    LDR         r4, [r0]                ; r4 = SIM_SCGC2
    AND         r4, r4, #0xFFF0FFFF     ; r4 = r4 & (~SIM_SCGC2_LTC_MASK)    
    STR         r4, [r0]                ; SIM_SCGC2 = r4
    CPSIE       i
  
ltc_gate_negated:  
    MLA         r1, r1, r3, r1          ; r1 = (r1 * r3) + r1
    ROR         r3, r3, #3    
    CMP         r6, #0
    BEQ         ltc_gate_on
    SUBS        r6, r6, #1
    B           ltc_gate_negated

ltc_gate_on:
    ; LTC clock on
    CPSID       i
    LDR         r4, [r0]                ; r4 = SIM_SCGC2
    ORR         r4, r4, #0x00020000     ; r4 = r4 | (SIM_SCGC2_LTC_MASK)
    STR         r4, [r0]                ; SIM_SCGC2 = r4
    CPSIE       i
    
    SUBS        r2, r2, #1
    CMP         r2, #0
    BNE         ltc_gate_off_outer_loop
        
    POP         {r4-r6, r8}
    
ltc_gate_off_exit:
    BX lr
    
    END
    