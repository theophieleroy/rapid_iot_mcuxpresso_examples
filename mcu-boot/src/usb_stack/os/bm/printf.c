/*********************************************************************
* File:        printk.c
* Purpose:          The standard C library routine printf(), but without
*                   all the baggage.
*
* This is a modified version of the file printf.c, which was distributed
* by Motorola as part of the M5407C3BOOT.zip package used to initialize 
* the M5407C3 evaluation board.
*
* Copyright:
*      1999-2000 MOTOROLA, INC. All Rights Reserved.  
*  You are hereby granted a copyright license to use, modify, and
*  distribute the SOFTWARE so long as this entire notice is
*  retained without alteration in any modified and/or redistributed
*  versions, and that such modified versions are clearly identified
*  as such. No licenses are granted by implication, estoppel or
*  otherwise under any patents or trademarks of Motorola, Inc. This 
*  software is provided on an "AS IS" basis and without warranty.
*
*  To the maximum extent permitted by applicable law, MOTOROLA 
*  DISCLAIMS ALL WARRANTIES WHETHER EXPRESS OR IMPLIED, INCLUDING 
*  IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR
*  PURPOSE AND ANY WARRANTY AGAINST INFRINGEMENT WITH REGARD TO THE 
*  SOFTWARE (INCLUDING ANY MODIFIED VERSIONS THEREOF) AND ANY 
*  ACCOMPANYING WRITTEN MATERIALS.
* 
*  To the maximum extent permitted by applicable law, IN NO EVENT
*  SHALL MOTOROLA BE LIABLE FOR ANY DAMAGES WHATSOEVER (INCLUDING 
*  WITHOUT LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS 
*  INTERRUPTION, LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY
*  LOSS) ARISING OF THE USE OR INABILITY TO USE THE SOFTWARE.   
* 
*  Motorola assumes no responsibility for the maintenance and support
*  of this software
********************************************************************/

#include "types.h"
#include "user_config.h"
#if PRINTF_ENABLE
#include <stdarg.h>
#include <math.h>

#ifdef  FALSE
   #undef  FALSE
#endif
#define FALSE (0)

#ifdef  TRUE
   #undef  TRUE
#endif
#define TRUE (1)

extern void out_char (char ch);
extern int32_t strlen (const char *str);
/********************************************************************/

typedef struct
{
    int32_t dest;
    void (*func)(char);
    char *loc;
} PRINTK_INFO;

int32_t
printk (PRINTK_INFO *, const char *, va_list);

/********************************************************************/

#define DEST_CONSOLE    (1)
#define DEST_STRING     (2)

#define FLAGS_MINUS     (0x01)
#define FLAGS_PLUS      (0x02)
#define FLAGS_SPACE     (0x04)
#define FLAGS_ZERO      (0x08)
#define FLAGS_POUND     (0x10)

#define IS_FLAG_MINUS(a)    (a & FLAGS_MINUS)
#define IS_FLAG_PLUS(a)     (a & FLAGS_PLUS)
#define IS_FLAG_SPACE(a)    (a & FLAGS_SPACE)
#define IS_FLAG_ZERO(a)     (a & FLAGS_ZERO)
#define IS_FLAG_POUND(a)    (a & FLAGS_POUND)

#define LENMOD_h        (0x01)
#define LENMOD_l        (0x02)
#define LENMOD_L        (0x04)

#define IS_LENMOD_h(a)  (a & LENMOD_h)
#define IS_LENMOD_l(a)  (a & LENMOD_l)
#define IS_LENMOD_L(a)  (a & LENMOD_L)

#define FMT_d   (0x0001)
#define FMT_o   (0x0002)
#define FMT_x   (0x0004)
#define FMT_X   (0x0008)
#define FMT_u   (0x0010)
#define FMT_c   (0x0020)
#define FMT_s   (0x0040)
#define FMT_p   (0x0080)
#define FMT_n   (0x0100)

#define IS_FMT_d(a)     (a & FMT_d)
#define IS_FMT_o(a)     (a & FMT_o)
#define IS_FMT_x(a)     (a & FMT_x)
#define IS_FMT_X(a)     (a & FMT_X)
#define IS_FMT_u(a)     (a & FMT_u)
#define IS_FMT_c(a)     (a & FMT_c)
#define IS_FMT_s(a)     (a & FMT_s)
#define IS_FMT_p(a)     (a & FMT_p)
#define IS_FMT_n(a)     (a & FMT_n)

/********************************************************************/
static void
printk_putc (int32_t c, int32_t *count, PRINTK_INFO *info)
{
    switch (info->dest)
    {
        case DEST_CONSOLE:
            info->func((char)c);
            break;
        case DEST_STRING:
            *(info->loc) = (unsigned char)c;
            ++(info->loc);
            break;
        default:
            break;
    }
    *count += 1;
}

/********************************************************************/
static int32_t
printk_mknumstr (char *numstr, void *nump, int32_t neg, int32_t radix)
{
    int32_t a,b,c;
    uint32_t ua,ub,uc;

    int32_t nlen;
    char *nstrp;

    nlen = 0;
    nstrp = numstr;
    *nstrp++ = '\0';

    if (neg)
    {
        a = *(int32_t *)nump;
        if (a == 0)
        {
            *nstrp = '0';
            ++nlen;
            goto done;
        }
        while (a != 0)
        {
            b = (int32_t)a / (int32_t)radix;
            c = (int32_t)a - ((int32_t)b * (int32_t)radix);
            if (c < 0)
            {
                c = ~c + 1 + '0';
            }
            else
            {
                c = c + '0';
            }
            a = b;
            *nstrp++ = (char)c;
            ++nlen;
        }
    }
    else
    {
        ua = *(uint32_t *)nump;
        if (ua == 0)
        {
            *nstrp = '0';
            ++nlen;
            goto done;
        }
        while (ua != 0)
        {
            ub = (uint32_t)ua / (uint32_t)radix;
            uc = (uint32_t)ua - ((uint32_t)ub * (uint32_t)radix);
            if (uc < 10)
            {
                uc = uc + '0';
            }
            else
            {
                uc = uc - 10 + 'A';
            }
            ua = ub;
            *nstrp++ = (char)uc;
            ++nlen;
        }
    }
    done:
    return nlen;
}
#if PRINTF_FLOAT_ENABLE
static int32_t
printk_mkfloatnumstr (char *numstr, void *nump, int32_t radix)
{
    int32_t a,b,c,i;
    double r, fractpart, intpart;

    int32_t nlen;
    char *nstrp;
    nlen = 0;
    nstrp = numstr;
    *nstrp++ = '\0';
    r = *(double *)nump;
    if (r == 0)
    {
        *nstrp = '0';
        ++nlen;
        goto done;
    }
    fractpart = modf((double)r , (double *)&intpart);
    /* Process fractional part */
    for (i = 0; i < 6; i++)
    {
        fractpart *= radix;		
    }
    //a = (int32_t)floor(fractpart + (double)0.5);
    a = (int32_t)(fractpart + (double)0.5);
    for (i = 0; i < 6; i++)
    {
        b = (int32_t)a / (int32_t)radix;
        c = (int32_t)a - ((int32_t)b * (int32_t)radix);
        if (c < 0)
        {
            c = ~c + 1 + '0';
        }else
        {		
            c = c + '0';
        }
        a = b;
        *nstrp++ = (char)c;
        ++nlen;
    }
    *nstrp++ = (char)'.';
    ++nlen;	
    a = (int32_t)intpart;
    while (a != 0)
    {
        b = (int32_t)a / (int32_t)radix;
        c = (int32_t)a - ((int32_t)b * (int32_t)radix);
        if (c < 0)
        {
            c = ~c + 1 + '0';
        }else
        {
            c = c + '0';
        }
        a = b;
        *nstrp++ = (char)c;
        ++nlen;
    }
    done:
    return nlen;
}
#endif
/********************************************************************/
static void
printk_pad_zero (int32_t curlen, int32_t field_width, int32_t *count, PRINTK_INFO *info)
{
    int32_t i;

    for (i = curlen; i < field_width; i++)
    {
        printk_putc('0',count, info);
    }
}

/********************************************************************/
static void
printk_pad_space (int32_t curlen, int32_t field_width, int32_t *count, PRINTK_INFO *info)
{
    int32_t i;

    for (i = curlen; i < field_width; i++)
    {
        printk_putc(' ',count, info);
    }
}

/********************************************************************/
#if PRINTF_FLOAT_ENABLE
#ifdef __GNUC__
#pragma GCC optimize ("O0")
#endif
#endif
int32_t
printk (PRINTK_INFO *info, const char *fmt, va_list ap)
{
    /* va_list ap; */
    char *p;
    int32_t c;

    char vstr[33];
    char *vstrp;
    int32_t vlen;

    int32_t done;
    int32_t count = 0;

    int32_t flags_used;
    int32_t field_width;

    int32_t ival;
    int32_t schar, dschar;
    int32_t *ivalp;
    char *sval;
    int32_t cval;
    uint32_t uval;
#if PRINTF_FLOAT_ENABLE
    double fval;
#endif

    /*
     * Start parsing apart the format string and display appropriate
     * formats and data.
     */
    for (p = (char *)fmt; (c = *p) != 0; p++)
    {
        /*
         * All formats begin with a '%' marker.  Special chars like
         * '\n' or '\t' are normally converted to the appropriate
         * character by the __compiler__.  Thus, no need for this
         * routine to account for the '\' character.
         */
        if (c != '%')
        {
            /*
             * This needs to be replaced with something like
             * 'out_char()' or call an OS routine.
             */
            if (c != '\n')
            {
                printk_putc(c, &count, info);
            }
            else
            {
                printk_putc(0x0D /* CR */, &count, info);
                printk_putc(0x0A /* LF */, &count, info);
            }

            /*
             * By using 'continue', the next iteration of the loop
             * is used, skipping the code that follows.
             */
            continue;
        }

        /*
         * First check for specification modifier flags.
         */
        flags_used = 0;
        done = FALSE;
        while (!done)
        {
            switch (/* c = */ *++p)
            {
                case '-':
                    flags_used |= FLAGS_MINUS;
                    break;
                case '+':
                    flags_used |= FLAGS_PLUS;
                    break;
                case ' ':
                    flags_used |= FLAGS_SPACE;
                    break;
                case '0':
                    flags_used |= FLAGS_ZERO;
                    break;
                case '#':
                    flags_used |= FLAGS_POUND;
                    break;
                default:
                    /* we've gone one char too far */
                    --p;
                    done = TRUE;
                    break;
            }
        }

        /*
         * Next check for minimum field width.
         */
        field_width = 0;
        done = FALSE;
        while (!done)
        {
            switch (c = *++p)
            {
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                    field_width = (field_width * 10) + (c - '0');
                    break;
                default:
                    /* we've gone one char too far */
                    --p;
                    done = TRUE;
                    break;
            }
        }

        /*
         * Next check for the width and precision field separator.
         */
        if (/* (c = *++p) */ *++p == '.')
        {
            /* precision_used = TRUE; */

            /*
             * Must get precision field width, if present.
             */
            /* precision_width = 0; */
            done = FALSE;
            while (!done)
            {
                switch (/* c = uncomment if used below */ *++p)
                {
                    case '0':
                    case '1':
                    case '2':
                    case '3':
                    case '4':
                    case '5':
                    case '6':
                    case '7':
                    case '8':
                    case '9':
                        break;
                    default:
                        /* we've gone one char too far */
                        --p;
                        done = TRUE;
                        break;
                }
            }
        }
        else
        {
            /* we've gone one char too far */
            --p;
        }

        /*
         * Check for the length modifier.
         */
        /* length_modifier = 0; */
        switch (/* c = */ *++p)
        {
            case 'h':
                /* length_modifier |= LENMOD_h; */
                break;
            case 'l':
                /* length_modifier |= LENMOD_l; */
                break;
            case 'L':
                /* length_modifier |= LENMOD_L; */
                break;
            default:
                /* we've gone one char too far */
                --p;
                break;
        }

        /*
         * Now we're ready to examine the format.
         */
        switch (c = *++p)
        {
            case 'd':
            case 'i':
                ival = (int32_t)va_arg(ap, int32_t);
                vlen = printk_mknumstr(vstr,&ival,TRUE,10);
                vstrp = &vstr[vlen];

                if (ival < 0)
                {
                    schar = '-';
                    ++vlen;
                }
                else
                {
                    if (IS_FLAG_PLUS(flags_used))
                    {
                        schar = '+';
                        ++vlen;
                    }
                    else
                    {
                        if (IS_FLAG_SPACE(flags_used))
                        {
                            schar = ' ';
                            ++vlen;
                        }
                        else
                        {
                            schar = 0;
                        }
                    }
                }
                dschar = FALSE;
            
                /*
                 * do the ZERO pad.
                 */
                if (IS_FLAG_ZERO(flags_used))
                {
                    if (schar)
                        printk_putc(schar, &count, info);
                    dschar = TRUE;
            
                    printk_pad_zero (vlen, field_width, &count, info);
                    vlen = field_width;
                }
                else
                {
                    if (!IS_FLAG_MINUS(flags_used))
                    {
                        printk_pad_space (vlen, field_width, &count, info);
            
                        if (schar)
                            printk_putc(schar, &count, info);
                        dschar = TRUE;
                    }
                }
            
                /* the string was built in reverse order, now display in */
                /* correct order */
                if (!dschar && schar)
                {
                    printk_putc(schar, &count, info);
                }
                goto cont_xd;
#if PRINTF_FLOAT_ENABLE
            case 'f':
            case 'F':
                fval = (double)va_arg(ap, double);
                vlen = printk_mkfloatnumstr(vstr,&fval,10);
                vstrp = &vstr[vlen];

                if (fval < 0)
                {
                    schar = '-';
                    ++vlen;
                }
                else
                {
                    if (IS_FLAG_PLUS(flags_used))
                    {
                        schar = '+';
                        ++vlen;
                    }
                    else
                    {
                        if (IS_FLAG_SPACE(flags_used))
                        {
                            schar = ' ';
                            ++vlen;
                        }
                        else
                        {
                            schar = 0;
                        }
                    }
                }
                dschar = FALSE;
                if (IS_FLAG_ZERO(flags_used))
                {
                    if (schar)
                        printk_putc(schar, &count, info);
                    dschar = TRUE;
                    printk_pad_zero (vlen, field_width, &count, info);
                    vlen = field_width;
                }
                else
                {
                    if (!IS_FLAG_MINUS(flags_used))
                    {
                        printk_pad_space (vlen, field_width, &count, info);
                        if (schar)
                            printk_putc(schar, &count, info);
                        dschar = TRUE;
                    }
                }
                if (!dschar && schar)
                {
                    printk_putc(schar, &count, info);
                }
                goto cont_xd;
#endif
            case 'x':
            case 'X':
                uval = (uint32_t)va_arg(ap, uint32_t);
                vlen = printk_mknumstr(vstr,&uval,FALSE,16);
                vstrp = &vstr[vlen];

                dschar = FALSE;
                if (IS_FLAG_ZERO(flags_used))
                {
                    if (IS_FLAG_POUND(flags_used))
                    {
                        printk_putc('0', &count, info);
                        printk_putc('x', &count, info);
                        /*vlen += 2;*/
                        dschar = TRUE;
                    }
                    printk_pad_zero (vlen, field_width, &count, info);
                    vlen = field_width;
                }
                else
                {
                    if (!IS_FLAG_MINUS(flags_used))
                    {
                        if (IS_FLAG_POUND(flags_used))
                        {
                            vlen += 2;
                        }
                        printk_pad_space (vlen, field_width, &count, info);
                        if (IS_FLAG_POUND(flags_used))
                        {
                            printk_putc('0', &count, info);
                            printk_putc('x', &count, info);
                            dschar = TRUE;
                        }
                    }
                }

                if ((IS_FLAG_POUND(flags_used)) && !dschar)
                {
                    printk_putc('0', &count, info);
                    printk_putc('x', &count, info);
                    vlen += 2;
                }
                goto cont_xd;

            case 'o':
                uval = (uint32_t)va_arg(ap, uint32_t);
                vlen = printk_mknumstr(vstr,&uval,FALSE,8);
                goto cont_u;
            case 'b':
                uval = (uint32_t)va_arg(ap, uint32_t);
                vlen = printk_mknumstr(vstr,&uval,FALSE,2);
                goto cont_u;
            case 'p':
                uval = (uint32_t)va_arg(ap, void *);
                vlen = printk_mknumstr(vstr,&uval,FALSE,16);
                goto cont_u;
            case 'u':
                uval = (uint32_t)va_arg(ap, uint32_t);
                vlen = printk_mknumstr(vstr,&uval,FALSE,10);

                cont_u:
                    vstrp = &vstr[vlen];

                    if (IS_FLAG_ZERO(flags_used))
                    {
                        printk_pad_zero (vlen, field_width, &count, info);
                        vlen = field_width;
                    }
                    else
                    {
                        if (!IS_FLAG_MINUS(flags_used))
                        {
                            printk_pad_space (vlen, field_width, &count, info);
                        }
                    }

                cont_xd:
                    while (*vstrp)
                        printk_putc(*vstrp--, &count, info);

                    if (IS_FLAG_MINUS(flags_used))
                    {
                        printk_pad_space (vlen, field_width, &count, info);
                    }
                break;

            case 'c':
                cval = (char)va_arg(ap, uint32_t);
                printk_putc(cval,&count, info);
                break;
            case 's':
                sval = (char *)va_arg(ap, char *);
                if (sval)
                {
                    vlen = strlen(sval);
                    if (!IS_FLAG_MINUS(flags_used))
                    {
                        printk_pad_space (vlen, field_width, &count, info);
                    }
                    while (*sval)
                        printk_putc(*sval++,&count, info);
                    if (IS_FLAG_MINUS(flags_used))
                    {
                        printk_pad_space (vlen, field_width, &count, info);
                    }
                }
                break;
            case 'n':
                ivalp = (int32_t *)va_arg(ap, int32_t *);
                *ivalp = count;
                break;
            default:
                printk_putc(c,&count, info);
                break;
        }
    }
    return count;
}
#if PRINTF_FLOAT_ENABLE
#ifdef __GNUC__
#pragma GCC optimize ("O3")
#endif
#endif
/********************************************************************/
int32_t
printf_kinetis (const char *fmt, ...)
{
    va_list ap;
    int32_t rvalue;
    PRINTK_INFO info;


    info.dest = DEST_CONSOLE;
    info.func = &out_char;
    /*
     * Initialize the pointer to the variable length argument list.
     */
    va_start(ap, fmt);
    rvalue = printk(&info, fmt, ap);
    /*
     * Cleanup the variable length argument list.
     */
    va_end(ap);
    return rvalue;
}

/********************************************************************/
int32_t
sprintf_kinetis (char *s, const char *fmt, ...)
{
    va_list ap;
    int32_t rvalue = 0;
    PRINTK_INFO info;

    /*
     * Initialize the pointer to the variable length argument list.
     */
    if (s != 0)
    {
        info.dest = DEST_STRING;
        info.loc = s;
        va_start(ap, fmt);
        rvalue = printk(&info, fmt, ap);
        *info.loc = '\0';
        va_end(ap);
    }
    return rvalue;
}

/********************************************************************/
#else
int32_t
printf_kinetis (const char *fmt, ...)
{
    return 0;
}
#endif
