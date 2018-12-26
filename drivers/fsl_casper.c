/*
 * The Clear BSD License
 * Copyright 2018 NXP
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

#include "fsl_casper.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.casper"
#endif

#define CASPER_RAM_BASE_NS (FSL_FEATURE_CASPER_RAM_BASE_ADDRESS)
extern uint32_t s_casperRamBase;

#if defined(FSL_FEATURE_CASPER_RAM_IS_INTERLEAVED) && FSL_FEATURE_CASPER_RAM_IS_INTERLEAVED
#define CASPER_RAM_OFFSET (0xE)
#define INTERLEAVE(addr) (((((((addr)>>2) & 0x00000001)<< CASPER_RAM_OFFSET) + (((addr)>>3)<<2) + ((addr) &0x00000003))&0xFFFF)|s_casperRamBase)
#define DEINTERLEAVE(addr) INTERLEAVE(addr)
#define GET_WORD(addr) (*((uint32_t*)DEINTERLEAVE((uint32_t)(addr))))
#define GET_DWORD(addr) (((uint64_t)GET_WORD(addr)) | (((uint64_t)GET_WORD(((uint32_t)(addr))+4))<<32))
#define SET_WORD(addr, value) *((uint32_t*)INTERLEAVE((uint32_t)(addr))) = ((uint32_t)(value))
#define SET_DWORD(addr, value) do{SET_WORD(addr, (uint32_t)(value & 0xFFFFFFFF)); SET_WORD(((uint32_t)(addr))+4, (uint32_t)((value & 0xFFFFFFFF00000000)>>32));}while(0)

/* memcopy is always word aligned */
/* interleaved to interleaved */
static void CASPER_MEMCPY_I2I(void *dst, const void* src, size_t siz)
{
	uint32_t *dst32 = (uint32_t *)dst, *src32 = (uint32_t *)src;
	int i;
	for(i = 0; i < siz/4; i++)
	{
             SET_WORD(&dst32[i], GET_WORD(&src32[i]));
	}
}

/* interleaved to non-interleaved */
static void CASPER_MEMCPY_I2N(void *dst, const void* src, size_t siz)
{
	uint32_t *dst32 = (uint32_t *)dst, *src32 = (uint32_t *)src;
	int i;
	for(i = 0; i < siz/4; i++)
	{
		dst32[i] = GET_WORD(&src32[i]);
	}
}

/* non-interleaved to interleaved */
static void CASPER_MEMCPY_N2I(void *dst, const void* src, size_t siz)
{
	volatile uint32_t *dst32 = (uint32_t *)dst, *src32 = (uint32_t *)src;
	volatile int i;
	for(i = 0; i < siz/4; i++)
	{
		SET_WORD(&dst32[i], src32[i]);
	}
}

#else
#define GET_WORD(addr) (*((uint32_t*)(addr)))
#define GET_DWORD(addr) (*((uint64_t*)(addr)))
#define SET_WORD(addr, value) *((uint32_t*)(addr)) = ((uint32_t)(value))
#define SET_DWORD(addr, value) *((uint64_t*)(addr)) = ((uint64_t)(value))

#define CASPER_MEMCPY_I2I(dst, src, siz) memcpy(dst, src, siz)
#define CASPER_MEMCPY_I2N(dst, src, siz) memcpy(dst, src, siz)
#define CASPER_MEMCPY_N2I(dst, src, siz) memcpy(dst, src, siz)
#endif

/*  The model for this algo is that it can be implemented for a fixed size RSA key */
/*  for max speed. If this is made into a variable (to allow varying size), then */
/*  it will be slower by a bit. */
/*  The file is compiled with N_bitlen passed in as number of bits of the RSA key */
/*  #define N_bitlen 2048 */
#define N_wordlen_max (4096 / 32)
static size_t N_wordlen = 0; /* ! number of words (e.g. 4096/32 is 128 words) */

#define WORK_BUFF_MUL4 (N_wordlen_max * 4 + 2) /* ! working buffer is 4xN_wordlen to allow in place math */
#define N_bytelen (N_wordlen * 4)              /*  for memory copy and the like */
#define N_dwordlen (N_wordlen / 2)

enum
{
    kCASPER_RamOffset_Result = 0x0u,
    kCASPER_RamOffset_Base = (N_wordlen_max + 8u),
    kCASPER_RamOffset_TempBase = (2u * N_wordlen_max + 16u),
    kCASPER_RamOffset_M64 = (kCASPER_RamOffset_TempBase + N_wordlen_max + 4u),
    kCASPER_RamOffset_Modulus = (kCASPER_RamOffset_M64 + 2u),
};

unsigned *msg_ret = (unsigned*)CASPER_RAM_BASE_NS;

#define PreZeroW(i, w_out)             \
    for (i = 0; i < N_wordlen; i += 4) \
    {                                   \
        SET_WORD(&w_out[i+0], 0);       \
        SET_WORD(&w_out[i+1], 0);       \
        SET_WORD(&w_out[i+2], 0);       \
        SET_WORD(&w_out[i+3], 0);       \
    } /*  unrolled partly */
#define PreZeroW2up(i, w_out)                       \
    for (i = N_wordlen; i <= N_wordlen * 2; i += 4) \
    {                                   \
        SET_WORD(&w_out[i+0], 0);       \
        SET_WORD(&w_out[i+1], 0);       \
        SET_WORD(&w_out[i+2], 0);       \
        SET_WORD(&w_out[i+3], 0);       \
    } /*  unrolled partly */

static uint32_t CA_MK_OFF(const void *addr)
{
    return ((uint32_t)addr - s_casperRamBase);
}

static void Accel_done(void)
{
    uint32_t status;
    do
    {
        status = CASPER->STATUS;
    } while (0 == (status & CASPER_STATUS_DONE_MASK));
}

static void Accel_SetABCD_Addr(uint32_t ab, uint32_t cd)
{
    CASPER->CTRL0 = ab | (cd << 16); /* CDoffset << 16 | ABoffset */
}

static void Accel_crypto_mul(uint32_t ctrl1)
{
    CASPER->CTRL1 = ctrl1;
}

static uint32_t Accel_IterOpcodeResaddr(uint32_t iter, uint32_t opcode, uint32_t resAddr)
{
    return CASPER_CTRL1_ITER(iter) | CASPER_CTRL1_MODE(opcode) | (resAddr << 16);
}

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint32_t s_casperRamBase = CASPER_RAM_BASE_NS;

/*******************************************************************************
 * Code
 ******************************************************************************/
 
/*  Compute R`, which is R mod N. This is done using subtraction */
/*  R has 1 in N_wordlen, but we do not fill it in since borrowed. */
/*  Exp-pubkey only used to optimize for exp=3 */
void MultprecMontCalcRp(unsigned Rp[], const unsigned exp_pubkey, const unsigned Nmod[])
{
    int i;

    /*  R is 2^n where n is 1 bit longer than Nmod, so 1 followed by 32 or 64 0 words for example */
    /*  Note that Nmod's upper most bit has to be 1 by definition, so one subtract is enough. We */
    /*  do not set the 1 since it is "borrowed" so no point */
    PreZeroW(i, Rp);
    Accel_SetABCD_Addr(CA_MK_OFF(Nmod), 0);
    Accel_crypto_mul(Accel_IterOpcodeResaddr(N_dwordlen - 1, kCASPER_OpSub64, CA_MK_OFF(Rp)));
    Accel_done();
    /*  final borrow cannot happen since we know we started with a larger number */
}

/*  MultprecMultiply - multiple w=u*v (per Knuth) */
/*  w_out is 2x the size of u and v */
void MultprecMultiply(unsigned w_out[], const unsigned u[], const unsigned v[])
{
    int i, j;

    /*  Knuth 4.3.1 - Algorithm M */
    /*    Compute w = u * v */
    /*  u and v are N bits long in 32 bit word form */
    /*  w is 2*N bits long in 32 bit word form */
    /*  Note: We just multiply in place */

    /*  Step 1. Fill w[t-1:0] with 0s, the upper half will be written as we go */
    PreZeroW(i, w_out);

    /*  We do 1st pass NOSUM so we do not have to 0 output */
    Accel_SetABCD_Addr(CA_MK_OFF(&v[0]), CA_MK_OFF(u));
    Accel_crypto_mul(Accel_IterOpcodeResaddr(N_wordlen / 2 - 1, kCASPER_OpMul6464NoSum, CA_MK_OFF(&w_out[0])));
    Accel_done();
    /*  Step 2. iterate over N words of v using j */
    for (j = 2; j < N_wordlen; j += 2)
    {
        /*  Step 2b. Check for 0 on v word - skip if so since we 0ed already */
        /*  Step 3. Iterate over N words of u using i - perform Multiply-accumulate */
        if (GET_WORD(&v[j]) || GET_WORD(&v[j + 1]))
        {
            Accel_SetABCD_Addr(CA_MK_OFF(&v[j]), CA_MK_OFF(u));
            Accel_crypto_mul(Accel_IterOpcodeResaddr(N_wordlen / 2 - 1, kCASPER_OpMul6464Sum, CA_MK_OFF(&w_out[j])));
            Accel_done();
        }
    }
}

/*  MultprecModulo performs divide to get remainer as needed for RSA */
/*  This performs (q,r) = u/v, but we do not keep q */
/*  r_out is module (remainder) and is 2*N */
/*  u is in r_out (1st N) at start (passed in) */
/*  v is N long */
void MultprecModulo(unsigned r_out[], const unsigned v[], int top)
{
    uint64_t u64;                      /*  use 64 bit math mixed with 32 bit */
    unsigned u32;                      /*  allows us to work on U in 32 bit */
    unsigned u_n, ul16, uh16, *u_shft; /*  u_shft is because r_out is u initially */
    unsigned vl16, vh16, v_Nm1;
    unsigned q_hat, r_hat, q_over;
    unsigned borrow, carry;
    int i, j, tmp;

    /*  Knuth 4.3.1 - Algorithm D */
    /*    Compute q = u / v giving remainder r = u mod v */
    /*    -- we only want r, so we build qhat but do not store the Qs */
    /*  v is N long, with u,q,r 2N long because u is slowly replavced by r. */
    /*  We normalize/unnormlize per Knuth in the buffer (not copied) */

    /*  Step 1. Normalize value so MSb is in v[n-1]. Remember that v is */
    /*  the public key - to call it a 2048 bit number, they cannot have 0 */
    /*  in the MSb (or it would be less than 2048 bits) and so we know we */
    /*  are normalized already. Therefore, u is effectively shifted already. */
    /*  For u, we have it in r_out. u[n] holds any overflow */
    /*  Since divide on CM3/4 is 32/32=32, we break into 16 bit halves, but */
    /*  multiply can be 32x32=64. */
    u_n = 0;
    u_shft = r_out; /*  u (shifted) is in r_out */

    v_Nm1 = GET_WORD(&v[N_wordlen - 1]); /*  MSw of public key */
    vl16 = v_Nm1 & 0xFFFF;    /*  lower 16 */
    vh16 = v_Nm1 >> 16;       /*  upper 16 */
    /*  Step 2. Iterate j from m-n down to 0 (M selected per Knuth as 2*N) */
    for (j = top; j >= 0; j--)
    {
        /*  Step 3. estimate q_hat as (U[j+n]*B + U[j+n-1]) / V[n-1] */
        /*  Note: using subset of Knuth algo since v is 1/2 len of u (which is */
        /*  from multiply or x^2 leading into this). */
        u32 = u_n; /*  pickup u4u3u2, knowing u4 is 0 */
        u64 = ((uint64_t)u_n << 32) | GET_WORD(&u_shft[j + N_wordlen - 1]);
        ul16 = u64 & 0xFFFF;         /*  lower 16 */
        uh16 = (u64 >> 16) & 0xFFFF; /*  upper 16 */

        /*  we see if even possible (u large enough relative to v) */
        if ((u32 - v_Nm1) <= u32)
        {
            u32 -= v_Nm1;
            q_over = 1; /*  overflow from the sub */
        }
        else
            q_over = 0;

        /*  q_hat = u32 / vh16 -- is the upper partial value */
        /*  estimate; if too much, then back down by 1 or 2 */
        q_hat = u32 / vh16;
        r_hat = u32 - (q_hat * vh16);
        /*  see if Q is more than 16 bits or remainder is too large  (over div) */
        if ((q_hat == 0x10000) || ((q_hat * vl16) > ((r_hat << 16) | uh16)))
        {
            /*  too much - undo a division */
            q_hat--;
            r_hat += vh16;
            /*  check if still too much */
            if ((r_hat < 0x10000) && ((q_hat * vl16) > ((r_hat << 16) | uh16)))
                q_hat--; /*  yes, so undo a 2nd */
        }

        /*  compose u3u2uh16, then sub q_hat*v if OK */
        u64 = (((uint64_t)u32 << 16) | uh16) - ((uint64_t)q_hat * v_Nm1);
        if (u64 >> 48)
        {
            /*  no, so add v back */
            u32 = (unsigned)(u64 + v_Nm1);
            q_hat--;
        }
        else
            u32 = (unsigned)u64;

        tmp = q_hat << 16; /*  quotient upper part */

        /*  divide lower part: q = u2uh16ul16 / v. */
        /*  estimate and add back if over divdied */
        q_hat = u32 / vh16;
        r_hat = u32 - (q_hat * vh16);
        if ((q_hat == 0x10000) || ((q_hat * vl16) > ((r_hat << 16) | ul16)))
        {
            /*  too much - undo a division */
            q_hat--;
            r_hat += vh16;
            /*  check if still too much */
            if ((r_hat < 0x10000) && ((q_hat * vl16) > ((r_hat << 16) | ul16)))
                q_hat--; /*  yes, so undo a 2nd */
        }

        /*  compose u2uh16ul16, then sub q_hat*v if OK */
        u64 = (((uint64_t)u32 << 16) | ul16) - ((uint64_t)q_hat * v_Nm1);
        if (u64 >> 48)
        {
            /*  no, so add v back */
            r_hat = (unsigned)(u64 + v_Nm1);
            q_hat--;
        }
        else
            r_hat = (unsigned)u64;

        q_hat |= tmp; /*  other half of the quotient */
        while (q_over ||
               ((uint64_t)q_hat * GET_WORD(&v[N_wordlen - 2])) > ((1LL << 32) * r_hat) + (uint64_t)GET_WORD(&u_shft[j + N_wordlen - 2]))
        { /*  if Qhat>b, then reduce to b-1, then adjust up Rhat */
            q_hat--;
            r_hat += v_Nm1;
            if (r_hat < v_Nm1)
                break; /*  no overflow */
                       /*  else repeat since Rhat >= b */
        }

        /*  Step 4. Multiply and subtract. We know the amount, */
        /*          so we do the schoolboy math. Have to do on */
        /*          the large value. */
        if (q_hat)
        {
            borrow = 0;
            for (i = 0; i < N_wordlen; i++)
            {
                u64 = (uint64_t)q_hat * GET_WORD(&v[i]) + borrow;
                borrow = (unsigned)(u64 >> 32);
                if (GET_WORD(&u_shft[i + j]) < (unsigned)u64)
                    borrow++; /*  carry the overflow */
                SET_WORD(&u_shft[i + j], GET_WORD(&u_shft[i + j]) - (unsigned)u64);
            }
            u_n -= borrow; /*  overflow from shift left does not fit otherwise */
        }

        /*  Store 5. (update Q - we don't), and add back V to remainder if we over-subtracted */
        /*           That restores remainder to correct (we could only be off by 1) */
        /*           This should happen very rarely. */
        if (u_n)
        {
            carry = 0;
            for (i = 0; i < N_wordlen; i++)
            {
                SET_WORD(&u_shft[i + j], GET_WORD(&u_shft[i + j]) + carry);
                carry = (GET_WORD(&u_shft[i + j]) < carry) ? 1 : 0;
                SET_WORD(&u_shft[i + j], GET_WORD(&u_shft[i + j]) + GET_WORD(&v[i]));
                if (GET_WORD(&u_shft[i + j]) < GET_WORD(&v[i]))
                    carry++;
            }
        }
        u_n = GET_WORD(&u_shft[j + N_wordlen - 1]); /*  hold upper part of u to catch overflow (to borrow from) */
    }
    /*  low N bits of r are valid as remainder */
}

/*  We convert X into a Mont form number. Note length of arrays: */
/*  x is N_wordlen, Nmod is N_wordlen */
/*  Rp is N_wordlen (it is R` which is R mod N) */
/*  Xmont_out is N_wordlen*2+1 */
void MultprecMontPrepareX(unsigned Xmont_out[], const unsigned x[], const unsigned Rp[], const unsigned Nmod[])
{
    MultprecMultiply(Xmont_out, x, Rp);
    MultprecModulo(Xmont_out, Nmod, N_wordlen);
}

void MultprecGenNp64(const unsigned *Nmod, unsigned *np64_ret) /*  only pass the low order double word */
{
    uint64_t nprime, Nmod_0;
    Nmod_0 = GET_WORD(&Nmod[0]) | ((uint64_t)GET_WORD(&Nmod[1]) << 32);

#define COMP_NPN_1 ((2 - Nmod_0 * nprime) * nprime) /*  computes N`*N0=1 mod 2^P where P is the partial built up */
    nprime = (((2 + Nmod_0) & 4) << 1) + Nmod_0;    /*  mod 2^4 */
    nprime = COMP_NPN_1;
    nprime = COMP_NPN_1;
    nprime = COMP_NPN_1;
    nprime = COMP_NPN_1;
    /*  8 multiplies of uint64_t */
    *((uint64_t *)np64_ret) = (~0LL - nprime) + 1LL;
}

/*  CIOS Multiply. This is the Coarse Integrated form where the values are */
/*  multiplied and reduced for each step of "i". This uses less memory and */
/*  is faster as a result. Note that this is used to square as well as mul, */
/*  so not as fast as pure squaring could be. */
void MultprecCiosMul(
    unsigned w_out[], const unsigned a[], const unsigned b[], const unsigned Nmod[], const unsigned *Np)
{
    int i, j;
    uint64_t *m64 = (uint64_t *)&msg_ret[kCASPER_RamOffset_M64];
    uint64_t Np64;
    uint64_t carry;
    uint64_t *a64, *b64, *w64, *N64;

    Np64 = *(uint64_t *)Np;

    a64 = (uint64_t *)a;
    b64 = (uint64_t *)b;
    w64 = (uint64_t *)w_out;
    N64 = (uint64_t *)Nmod;

    if (a)
    { /*  if !a, we are reducing only */
        PreZeroW(i, w_out);
    }
    SET_DWORD(&w64[N_dwordlen], 0);
    SET_DWORD(&w64[N_dwordlen + 1], 0);
    /*  with accelerator */

    /*  loop i and then reduce after each j round */
    for (i = 0; i < N_dwordlen; i++)
    {
        /*  Step 3. Iterate over N words of u using i - perform Multiply-accumulate */
        /*  push-pull: we do a*b and then separately m*n (reduce) */
        if (a)
        { /*  if mul&reduce vs. reduce only */
            carry = GET_DWORD(&w64[N_dwordlen]);
            Accel_SetABCD_Addr(CA_MK_OFF(&b64[i]), CA_MK_OFF(a64));
            Accel_crypto_mul(Accel_IterOpcodeResaddr(N_dwordlen - 1, kCASPER_OpMul6464FullSum, CA_MK_OFF(w64)));
            Accel_done();
            /*  max carry is contained since ~0*~0=0xFFFE0001+0xFFFF=0xFFFF0000, */
            /*  so max carry is 0xFFFF and 0xFFFF0000+0xFFFF=0xFFFFFFFF */
            /*  accel took care of w_out[N_wordlen] & +1, so we just take care of the next double word if carry=1 */
            /*  w64[N_dwordlen+1] = g_carry; */
            carry = (GET_DWORD(&w64[N_dwordlen]) < carry);
            SET_DWORD(&w64[N_dwordlen + 1], carry);
        }
        SET_DWORD(&m64[0], GET_DWORD(&w64[0]) * Np64); /*  prime for 1st; modulo a double-word */

        /*  we are reducing, so the 1st [0th] 64 bit value product is tossed, but we */
        /*  need its carry. We let the accel do this separately - really need a mode to */
        /*  do this "reduce" since it is natural */
        carry = GET_DWORD(&w64[N_dwordlen]);
        Accel_SetABCD_Addr(CA_MK_OFF(m64), CA_MK_OFF(&N64[0]));
        Accel_crypto_mul(Accel_IterOpcodeResaddr(N_dwordlen - 1, kCASPER_OpMul6464FullSum, CA_MK_OFF(&w64[0])));
        Accel_done();
        carry = (GET_DWORD(&w64[N_dwordlen]) < carry);

        Accel_SetABCD_Addr(CA_MK_OFF(&w64[1]), 0);
        Accel_crypto_mul(Accel_IterOpcodeResaddr(N_dwordlen - 1, kCASPER_OpCopy, CA_MK_OFF(&w64[0])));

        Accel_done();
        SET_DWORD(&w64[N_dwordlen], GET_DWORD(&w64[N_dwordlen + 1]) + carry);
    }

    /*  now check if need to subtract Nmod */
    if (GET_WORD(&w_out[N_wordlen]))
        j = 1; /*  we have to subtract for sure if carry up */
    else
    {
        j = 0;
        for (i = N_wordlen - 1; i >= 0; i--)
            if (GET_WORD(&w_out[i]) != GET_WORD(&Nmod[i]))
            {
                j = GET_WORD(&w_out[i]) > GET_WORD(&Nmod[i]); /*  if larger sub */
                break;                  /*  we would remove the break if worrying about side channel */
            }
    }
    if (!j)
        return; /*  Is smaller than Nmod, so done. */
    Accel_SetABCD_Addr(CA_MK_OFF(Nmod), 0);
    Accel_crypto_mul(Accel_IterOpcodeResaddr(N_dwordlen - 1, kCASPER_OpSub64, CA_MK_OFF(w_out)));
    Accel_done();
    /*  last borrow is OK since we know it could only be <2N and */
}

/*  RSA_MontSignatureToPlaintextFast: */
/*  MsgRet[] = Message return buffer - must be large enough to hold input and output (4*N+2) */
/*  exp_pubkey = the "e" that the value is raised to. Usually 3 or 0x10001. */
/*  signature = N bitpos len long "message" to process in Montgomery form - so saving conversion (divide) */
/*  pubkey = N bitpos len long public key to process signature with */
/*  returns: 0 */
/*  */
/*  Algo: compute M = signaturen^e mod public_key */
/*        where M is original plaintext, signature is signed value */
/*        note: e is usually either 0x3 or 0x10001 */
int RSA_MontSignatureToPlaintextFast(const unsigned mont_signature[N_wordlen_max],
                                     const unsigned exp_pubkey,
                                     const unsigned pubkey[N_wordlen_max],
                                     unsigned MsgRet[WORK_BUFF_MUL4])
{
    int bidx = 0;
    int bitpos;
    unsigned np64[2];

    /*  MsgRet working area: */
    /*  0..N = RESULT, starting with S` */
    /*  N..N*2 = S` and then working BASE during math. */
    /*  N*2..N*4+2 = temp working area for Mont mul */

    /*  1. Copy sig into MsgRet so we have one working result buffer */
    CASPER_MEMCPY_I2I(&MsgRet[kCASPER_RamOffset_Result], mont_signature, N_bytelen);
    MultprecGenNp64(pubkey, np64);   /*  Generate N` from LSW of N (LSW being lowest 64b word) */
    bitpos = 31 - __CLZ(exp_pubkey); /*  count of bits after the left most 1 */
    while (--bitpos >= 0)
    {
        /*  This operates on: */
        /*    result = 1; */
        /*    base = signature */
        /*    loop while exponent bits from MSb to LSb */
        /*      if (exp bit is 1) */
        /*        result = result * base */
        /*      base = base^2 */
        /*  Because the MSb of exp is always 1 by definition, we can invert this a bit: */
        /*    base = signature` */
        /*    result = base; equivalent to result = 1*base from 1st pass, but now square is needed 1st */
        /*    loop while exponent bits from MSb-1 to LSb */
        /*      base = base^2 */
        /*      if (exp bit is 1) */
        /*        result = result * base */
        /*  This ends up doing the same thing but skips two wasteful steps of multiplying by 1 and */
        /*  a final squaring never used. */
        /*  */
        /*  Next we have the problem that CIOS mul needs a separate dest buffer. So, we bounce */
        /*  base between base and temp, and likewise for result. */
        MultprecCiosMul(&MsgRet[bidx ? kCASPER_RamOffset_Base : kCASPER_RamOffset_TempBase],
                        &MsgRet[bidx ? kCASPER_RamOffset_TempBase : kCASPER_RamOffset_Base],
                        &MsgRet[bidx ? kCASPER_RamOffset_TempBase : kCASPER_RamOffset_Base], pubkey, np64);
        if (exp_pubkey & (1 << bitpos)) /*  where e is 1 */
        {
            /*  result has result, so we need to work into other temp area */
            MultprecCiosMul(&MsgRet[bidx ? kCASPER_RamOffset_TempBase : kCASPER_RamOffset_Base],
                            &MsgRet[kCASPER_RamOffset_Result],
                            &MsgRet[bidx ? kCASPER_RamOffset_Base : kCASPER_RamOffset_TempBase], pubkey, np64);
            /*  we have to copy back to result */

            CASPER_MEMCPY_I2I(&MsgRet[kCASPER_RamOffset_Result],
                   &MsgRet[bidx ? kCASPER_RamOffset_TempBase : kCASPER_RamOffset_Base], N_bytelen);
        }
        else
            bidx = ~bidx;
    }

    /*  final step is one more reduction to get back to normal form (ie. divide R out) */
    MultprecCiosMul(&MsgRet[kCASPER_RamOffset_Result], NULL, NULL, pubkey, np64);
    return (0); /*  always 0 */
}

/*  RSA_SignatureToPlaintextFast: */
/*  MsgRet[] = Message return buffer - must be large enough to hold input and output (4*N+2) */
/*  exp_pubkey = the "e" that the value is raised to. Usually 3 or 0x10001. */
/*  signature = N bitpos len long "message" to process in normal form - so converted to Mont form */
/*  pubkey = N bitpos len long public key to process signature with */
/*  returns: 0 */
/*  */
/*  Algo: compute M = signaturen^e mod public_key */
/*        where M is original plaintext, signature is signed value */
/*        note: e is usually either 0x3 or 0x10001 */
int RSA_SignatureToPlaintextFast(const unsigned signature[N_wordlen_max],
                                 const unsigned exp_pubkey,
                                 const unsigned pubkey[N_wordlen_max],
                                 unsigned MsgRet[WORK_BUFF_MUL4])
{
    /*  MsgRet working area: */
    /*  0..N = RESULT, starting with S`; it is used for R` just during creation of S` */
    /*  N..N*2 = S` and then working BASE during math. Note overflow beyond N*2 when making S` */
    /*  N*2..N*4+2 = temp working area for Mont mul */

    MultprecMontCalcRp(&MsgRet[kCASPER_RamOffset_Result], exp_pubkey, pubkey); /*  calculate R` (=R mod N) */
    MultprecMontPrepareX(&MsgRet[kCASPER_RamOffset_Base], signature, &MsgRet[kCASPER_RamOffset_Result],
                         pubkey); /*  X*R1` mod N */
    return (RSA_MontSignatureToPlaintextFast(&MsgRet[kCASPER_RamOffset_Base], exp_pubkey, pubkey, MsgRet));
}

int CASPER_ModExp(
    CASPER_Type *base, const uint8_t *signature, const uint8_t *pubN, size_t wordLen, uint32_t pubE, uint8_t *plaintext)
{
#define PK_LOC &msg_ret[kCASPER_RamOffset_Modulus]
#define SIG_LOC &msg_ret[kCASPER_RamOffset_Modulus + N_wordlen_max]

    N_wordlen = wordLen; /* set global variable for key length - used by RSA_SignatureToPlaintextFast()  */
    CASPER_MEMCPY_N2I(PK_LOC, pubN, N_bytelen);
    CASPER_MEMCPY_N2I(SIG_LOC, signature, N_bytelen);
    RSA_SignatureToPlaintextFast((const unsigned *)(SIG_LOC), pubE, (const unsigned *)(PK_LOC), msg_ret);

    CASPER_MEMCPY_I2N(plaintext, msg_ret, N_bytelen);
    return 0;
}

void CASPER_Init(CASPER_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    CLOCK_EnableClock(kCLOCK_Casper);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
    RESET_PeripheralReset(kCASPER_RST_SHIFT_RSTn);
    /* If Casper init is called with secure address, use secure addres also for accessing Casper RAM. */
    s_casperRamBase = CASPER_RAM_BASE_NS | ((uint32_t)base & 0x10000000u);
    msg_ret = (unsigned*)s_casperRamBase;
}

void CASPER_Deinit(CASPER_Type *base)
{
    RESET_SetPeripheralReset(kCASPER_RST_SHIFT_RSTn);
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    CLOCK_DisableClock(kCLOCK_Casper);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}
