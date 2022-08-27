/* ******************************************************************** 
* maxqfft.c * * July 01, 2005 * 
* Paul Holden (Paul_Holden@maximhq.com) * Maxim Integrated Products * 
* SOFTWARE COMPILES USING IAR EMBEDDED WORKBENCH FOR MAXQ * 
* NOTE: All fft input/outputs are signed and in Q8.7 notation * 
* Copyright (C) 2005 Maxim/Dallas Semiconductor Corporation, 
* All Rights Reserved. * 
* Permission is hereby granted, free of charge, to any person obtaining a 
* copy of this software and associated documentation files (the "Software"), 
* to deal in the Software without restriction, including without limitation 
* the rights to use, copy, modify, merge, publish, distribute, sublicense, 
* and/or sell copies of the Software, and to permit persons to whom the 
* Software is furnished to do so, subject to the following conditions: * 
* The above copyright notice and this permission notice shall be included 
* in all copies or substantial portions of the Software. * 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
* IN NO EVENT SHALL MAXIM/DALLAS SEMICONDUCTOR BE LIABLE FOR ANY CLAIM, 
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR 
* OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR 
* THE USE OR OTHER DEALINGS IN THE SOFTWARE. * 
* Except as contained in this notice, the name of Maxim/Dallas Semiconductor 
* shall not be used except as stated in the Maxim/Dallas Semiconductor 
* Branding Policy. 

	adapted by G.Dar coglioni umani del cazzo 25/4/2022

* ******************************************************************** */

// INCLUDE STATEMENTS ---------------------------
#include <stdio.h>
//#include "maxqfft.h" WELDED IN HERE

/* DEFINE STATEMENTS */
#define N 256
#define N_DIV_2 128
#define N_DIV_2_PLUS_1 129
#define N_MINUS_1 255
#define LOG_2_N 8

/* cosine Look-Up Table: An LUT for the cosine function in Q8.7. The table was created with the following program: #include <stdio.h> #include <math.h> #define N 256 void main(int argc, char* argv[]) { printf("const int cosLUT[%d] =\n{\n",N/2); for(int i=0; i<N/2; i++) { printf("%+4d",(int)(128*cos(2*M_PI*i/N))); if (i<(N/2-1)) printf(","); if ((i+1)%16 == 0) printf("\n"); } printf("};\n"); }
*/
const int cosLUT[N_DIV_2] = {
	+128,+127,+127,+127,+127,+127,+126,+126,+125,+124,+124,+123,+122,+121,+120,+119,
	+118,+117,+115,+114,+112,+111,+109,+108,+106,+104,+102,+100, +98, +96, +94, +92, +90, +88, +85, +83, +81, +78, +76, +73, +71, +68, +65, +63, +60, +57, +54, +51, +48, +46, +43, +40, +37, +34, +31, +28, +24, +21, +18, +15, +12, +9, +6, +3, +0, -3, -6, -9, -12, -15, -18, -21, -24, -28, -31, -34, -37, -40, -43, -46, -48, -51, -54, -57, -60, -63, -65, -68, -71, -73, -76, -78, -81, -83, -85, -88, -90, -92, -94, -96, -98,-100,-102,-104,-106,-108,-109,-111,-112,-114,-115,-117,
	-118,-119,-120,-121,-122,-123,-124,-124,-125,-126,-126,-127,-127,-127,-127,-127
	};

/* sine Look-Up Table: An LUT for the sine function in Q8.7. The table was created with the following program: #include <stdio.h> #include <math.h> #define N 256 void main(int argc, char* argv[]) { printf("const int sinLUT[%d] =\n{\n",N/2); for(int i=0; i<N/2; i++) { printf("%+4d",(int)(128*sin(2*M_PI*i/N))); if (i<(N/2-1)) printf(","); if ((i+1)%16 == 0) printf("\n"); } printf("};\n"); }
*/
const int sinLUT[N_DIV_2] = { 
	+0, +3, +6, +9, +12, +15, +18, +21, +24, +28, +31, +34, +37, +40, +43, +46, +48, +51, +54, +57, +60, +63, +65, +68, +71, +73, +76, +78, +81, +83, +85, +88, +90, +92, +94, +96, +98,+100,+102,+104,+106,+108,+109,+111,+112,+114,+115,+117,
	+118,+119,+120,+121,+122,+123,+124,+124,+125,+126,+126,+127,+127,+127,+127,+127,
	+128,+127,+127,+127,+127,+127,+126,+126,+125,+124,+124,+123,+122,+121,+120,+119,
	+118,+117,+115,+114,+112,+111,+109,+108,+106,+104,+102,+100, +98, +96, +94, +92, +90, +88, +85, +83, +81, +78, +76, +73, +71, +68, +65, +63, +60, +57, +54, +51, +48, +46, +43, +40, +37, +34, +31, +28, +24, +21, +18, +15, +12, +9, +6, +3
	};

/* Hamming Window Look-Up Table: An LUT for the Hamming Window function in Q8.7. The table was created with the following program: #include <stdio.h> #include <math.h> #define N 256 void main(int argc, char* argv[]) { printf("const char hammingLUT[%d] =\n{\n",N); for(int i=0; i<N; i++) { printf("%+4d",(int)(128*(0.54-0.46*cos(2*M_PI*i/(N-1))))); if (i<(N-1)) printf(","); if ((i+1)%16 == 0) printf("\n"); } printf("};\n"); }
*/
const char hammingLUT[N] = { 
	+10, +10, +10, +10, +10, +10, +10, +11, +11, +11, +12, +12, +12, +13, +13, +14, +14, +15, +15, +16, +17, +17, +18, +19, +20, +21, +21, +22, +23, +24, +25, +26, +27, +28, +29, +30, +31, +33, +34, +35, +36, +37, +39, +40, +41, +42, +44, +45, +46, +48, +49, +50, +52, +53, +55, +56, +57, +59, +60, +62, +63, +65, +66, +68, +69, +70, +72, +73, +75, +76, +78, +79, +81, +82, +83, +85, +86, +88, +89, +90, +92, +93, +94, +96, +97, +98, +99,+101,+102,+103,+104,+105,+106,+107,+109,+110,
	+111,+112,+113,+114,+114,+115,+116,+117,+118,+119,+119,+120,+121,+121,+122,+123,
	+123,+124,+124,+125,+125,+126,+126,+126,+126,+127,+127,+127,+127,+127,+127,+127,
	+127,+127,+127,+127,+127,+127,+127,+126,+126,+126,+126,+125,+125,+124,+124,+123,
	+123,+122,+121,+121,+120,+119,+119,+118,+117,+116,+115,+114,+114,+113,+112,+111,
	+110,+109,+107,+106,+105,+104,+103,+102,+101, +99, +98, +97, +96, +94, +93, +92, +90, +89, +88, +86, +85, +83, +82, +81, +79, +78, +76, +75, +73, +72, +70, +69, +68, +66, +65, +63, +62, +60, +59, +57, +56, +55, +53, +52, +50, +49, +48, +46, +45, +44, +42, +41, +40, +39, +37, +36, +35, +34, +33, +31, +30, +29, +28, +27, +26, +25, +24, +23, +22, +21, +21, +20, +19, +18, +17, +17, +16, +15, +15, +14, +14, +13, +13, +12, +12, +12, +11, +11, +11, +10, +10, +10, +10, +10, +10, +10
	};

/* Hann Window Look-Up Table: An LUT for the Hann Window function in Q8.7. The table was created with the following program: #include <stdio.h> #include <math.h> #define N 256 void main(int argc, char* argv[]) { printf("const char hannLUT[%d] =\n{\n",N); for(int i=0; i<N; i++) { printf("%+4d",(int)(128*(0.5-0.5*cos(2*M_PI*i/(N-1))))); if (i<(N-1)) printf(","); if ((i+1)%16 == 0) printf("\n"); } printf("};\n"); }
*/
const char hannLUT[N] = { 
	+0, +0, +0, +0, +0, +0, +0, +0, +1, +1, +1, +2, +2, +3, +3, +4, +4, +5, +6, +6, +7, +8, +9, +10, +10, +11, +12, +13, +14, +15, +16, +17, +18, +20, +21, +22, +23, +24, +26, +27, +28, +29, +31, +32, +34, +35, +36, +38, +39, +41, +42, +44, +45, +47, +48, +50, +51, +53, +54, +56, +58, +59, +61, +62, +64, +65, +67, +69, +70, +72, +73, +75, +76, +78, +79, +81, +83, +84, +86, +87, +88, +90, +91, +93, +94, +95, +97, +98,+100,+101,+102,+103,+105,+106,+107,+108,
	+109,+110,+111,+112,+113,+114,+115,+116,+117,+118,+119,+120,+120,+121,+122,+122,
	+123,+123,+124,+124,+125,+125,+126,+126,+126,+127,+127,+127,+127,+127,+127,+127,
	+127,+127,+127,+127,+127,+127,+127,+126,+126,+126,+125,+125,+124,+124,+123,+123,
	+122,+122,+121,+120,+120,+119,+118,+117,+116,+115,+114,+113,+112,+111,+110,+109,
	+108,+107,+106,+105,+103,+102,+101,+100, +98, +97, +96, +94, +93, +91, +90, +88, +87, +86, +84, +83, +81, +79, +78, +76, +75, +73, +72, +70, +69, +67, +65, +64, +62, +61, +59, +58, +56, +54, +53, +51, +50, +48, +47, +45, +44, +42, +41, +39, +38, +36, +35, +34, +32, +31, +29, +28, +27, +26, +24, +23, +22, +21, +20, +18, +17, +16, +15, +14, +13, +12, +11, +10, +10, +9, +8, +7, +6, +6, +5, +4, +4, +3, +3, +2, +2, +1, +1, +1, +0, +0, +0, +0, +0, +0, +0, +0
	};

/* Magnitude Look-Up Table: A LUT for determining the magnitude of X(n). The actual magnitude is calculated using the following equation: |X(n)| = sqrt(Re{X(n)}^2 + Im{X(n)}^2) The first LUT index corresponds to the real part of X(n) while the second index corresponds to the imaginary part of X(n). If X(n) is in Q8.7, the absolut value of the real and imaginary parts must be shifted to the right by 11 positions to obtain the 4-MSb (Most Significant bits) needed for each of the two indexed (2^4 = 16). The table was created with the following program: #include <stdio.h> #include <math.h> #define N 256 long magn[16][16] = {{0}}; void main(int argc, char* argv[]) { printf("const unsigned char magnLUT[16][16] =\n{\n"); for(int i=0; i<16; i++) { printf("{"); for(int j=0; j<16; j++) { magn[i][j] = (long)sqrt(pow(i<<12,2) + pow(j<<12,2)); if (magn[i][j] > 0xffff) magn[i][j] = 0xffff; printf("0x%02x",(magn[i][j] >> 8) & 0x000000ff); if (j<15) printf(","); } if (i<15) printf("},\n"); else printf("}\n"); } printf("};\n"); }
*/
const unsigned char magnLUT[16][16] = {
	{0x00,0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80,0x90,0xa0,0xb0,0xc0,0xd0,0xe0,0xf0},
	{0x10,0x16,0x23,0x32,0x41,0x51,0x61,0x71,0x80,0x90,0xa0,0xb0,0xc0,0xd0,0xe0,0xf0},
	{0x20,0x23,0x2d,0x39,0x47,0x56,0x65,0x74,0x83,0x93,0xa3,0xb2,0xc2,0xd2,0xe2,0xf2},
	{0x30,0x32,0x39,0x43,0x50,0x5d,0x6b,0x79,0x88,0x97,0xa7,0xb6,0xc5,0xd5,0xe5,0xf4},
	{0x40,0x41,0x47,0x50,0x5a,0x66,0x73,0x80,0x8f,0x9d,0xac,0xbb,0xca,0xd9,0xe8,0xf8},
	{0x50,0x51,0x56,0x5d,0x66,0x71,0x7c,0x89,0x96,0xa4,0xb2,0xc1,0xd0,0xde,0xed,0xfc},
	{0x60,0x61,0x65,0x6b,0x73,0x7c,0x87,0x93,0xa0,0xad,0xba,0xc8,0xd6,0xe5,0xf3,0xff},
	{0x70,0x71,0x74,0x79,0x80,0x89,0x93,0x9e,0xaa,0xb6,0xc3,0xd0,0xde,0xec,0xfa,0xff},
	{0x80,0x80,0x83,0x88,0x8f,0x96,0xa0,0xaa,0xb5,0xc0,0xcc,0xd9,0xe6,0xf4,0xff,0xff},
	{0x90,0x90,0x93,0x97,0x9d,0xa4,0xad,0xb6,0xc0,0xcb,0xd7,0xe3,0xf0,0xfc,0xff,0xff},
	{0xa0,0xa0,0xa3,0xa7,0xac,0xb2,0xba,0xc3,0xcc,0xd7,0xe2,0xed,0xf9,0xff,0xff,0xff},
	{0xb0,0xb0,0xb2,0xb6,0xbb,0xc1,0xc8,0xd0,0xd9,0xe3,0xed,0xf8,0xff,0xff,0xff,0xff},
	{0xc0,0xc0,0xc2,0xc5,0xca,0xd0,0xd6,0xde,0xe6,0xf0,0xf9,0xff,0xff,0xff,0xff,0xff},
	{0xd0,0xd0,0xd2,0xd5,0xd9,0xde,0xe5,0xec,0xf4,0xfc,0xff,0xff,0xff,0xff,0xff,0xff},
	{0xe0,0xe0,0xe2,0xe5,0xe8,0xed,0xf3,0xfa,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff},
	{0xf0,0xf0,0xf2,0xf4,0xf8,0xfc,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}
	};
// END WELD FROM OLD EXTERNAL HEADER FILE:

// THIS NEEDS TODO:
//#include <intrinsics.h> // Um... research this. currently just elided

// DEFINE STATEMENTS ----------------------------
#define NOP __no_operation()

/* * Windowing: Uncomment one of the following define * statements to enable the corresponding windowing * function on input samples. Comment all to disable * windowing. */
//#define WINDOWING_HAMMING
//#define WINDOWING_HANN

/* * x_n_re * * This array will store the FFT input samples, x(n), * and the real part of the spectrum, X(n). */
//__no_init
int x_n_re[N];
/* * tmp_32 * * a union that allows accessing the individual 16-bit * words of a 32-bit double word as well as the entire * double word. This union is used by the multiplication * macros that make use of the hardware multiplier and * convert the multiplication result to Q8.7 notation. */
//__no_init
union { 
	long tmp_32; 
	struct { 
		int LSW; // Least Significant 16-bit Word 
		int MSW; // Most Significant 16-bit Word 
		} tmp_16;
	};

/* * Hardware Multiplier Macros * * * * * * These macros are used to access the hardware multiplier. For * the MAXQ, registers MA and MB are the hardware multiplier * operands while MC1:MC0 store the hardware multiplier result. * * (1) MUL_1(A,B,C) : C=A*B (result converted to Q8.7 notation) * (2) MUL_2(A,C) : C=A*MB (result converted to Q8.7 notation) * (3) MUL_INIT(B) : MB=B * (4) MUL_NC(A,C) : C=A*MB (result not converted to Q8.7 notation) */
// REWORK THESE FOR STANDARD USE
// THIS NEEDS TODO:
#define MUL_1(A,B,C)
//MA=A;MB=B; NOP; tmp_16.LSW=MC0; tmp_16.MSW=MC1; C=tmp_32>>7
// THIS NEEDS TODO:
#define MUL_2(A,C)
//MA=A; NOP; tmp_16.LSW=MC0; tmp_16.MSW=MC1; C=tmp_32>>7
// THIS NEEDS TODO:
#define MUL_INIT(B)
//MB=B
// THIS NEEDS TODO:
#define MUL_NC(A,C)
//MA=A; NOP; C=MC0

/* * initADC() * * Initializes the ADC to send single channel 8-bit data to the * MAXQ. Refer to the included circuit schematic for connection * information. */
void initADC() {
// THIS NEEDS TODO: //	SETUP THE SOUNDCARD OR IO OF SOME NATURE	//__no_init // unsigned int i;
// // Configure the MAXQ GPIO pins for a write to the ADC
// // configuration register.
// PD0 = 0xFF; PD1 = 0xFF; PD2 = 0x3F;
// PO0 = 0x04; PO1 = 0x00; PO2 = 0xDF;
//
// // Waste time to allow ADC to exit shutdown mode
// for(i=0; i<2048; i++) __no_operation();
//
// // Enable the ADC Channel 0
// PO2_bit.bit2 = 0; NOP; // ADC /CS pin low
// PO2_bit.bit1 = 0; NOP; // ADC /WR pin low
// PO2_bit.bit1 = 1; NOP; // ADC /WR pin high
//
// // Configure the MAXQ GPIO pins to read samples
// // from the ADC
// PD0 = 0x00; PD1 = 0x00;
// PO0 = 0x00;
//
// // Send dummy byte to ADC
// PO2_bit.bit3 = 0; NOP; // ADC CONVST pin low
// PO2_bit.bit3 = 1; NOP; // ADC CONVST pin high
//
// while(!PO2_bit.bit7); // Wait for End-Of-Last-Conversion flag
//
// PO2_bit.bit0 = 0; NOP; // ADC /RD pin low
// PO2_bit.bit0 = 1; NOP; // ADC /RD pin high
//
// PO2_bit.bit2 = 1; // ADC /CS pin high
	}

/* * getSamplesFromADC() * * Captures N 8-bit samples (in 2's complement format) from * the ADC and stores them in the array x_n_re. If windowing * is enabled, the data will be multiplied by the * appropriate function. */
void getSamplesFromADC() { 
// THIS NEEDS TODO:	// rewrite to suck sample from stdio or something /* 1. Init variables */ //__no_init unsigned int i; int *ptr_x_n_re = x_n_re; /* 2. Set ADC /CS pin low to enable interface */ // PO2_bit.bit2 = 0; /* 3. Capture 256 samples from the ADC. This loop does not include any decision structure to ensure that the sampling rate is consistent. Delays are also introduced to force a sampling rate of 200ksps. */ for(i=0; i<256; i++) { // THIS NEEDS TODO: // GRAB THE CHARS _ BITS _ SAMPLES W/E
//
//
// /* 3.1. Acquire sample */
// PO2_bit.bit3 = 0; // ADC CONVST pin low
// NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
// NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
// NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
// NOP;NOP; // Wait tacq
// PO2_bit.bit3 = 1; // ADC CONVST pin high
//
// /* 3.2. Wait for sample to convert */
// NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
// NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
// NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
// NOP;NOP; // Wait for tconv
//
// /* 3.3. Read the sample from the ADC. Pointer notation instead
// of array notation is used for x_n_re to increase sampling
// speed.
// */
// PO2_bit.bit0 = 0; // ADC /RD pin low
// *(ptr_x_n_re++) = PI1; // Read sample
// PO2_bit.bit0 = 1; // ADC /RD pin high
//
// /* 3.4. Wait for 400nS. This will force a sampling rate of 200ksps */
// NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP; } /* 4. Set ADC /CS pin high to disable the ADC digital interface */
// PO2_bit.bit2 = 1; // ADC /CS pin high // THIS NEEDS TODO: /* 5. Perform adjustments to samples: - If the sample is negative (sign bit is 1), convert the sampled byte to the corresponding negative 16-bit word - Multiply sample by windowing function if enabled */ for(i=0; i<256; i++) { if (x_n_re[i]&0x0080) x_n_re[i] += 0xFF00; // Convert to negative 16-bit word (2's comp)

#ifdef WINDOWING_HAMMING MUL_1(x_n_re[i],hammingLUT[i],x_n_re[i]); // x(n) = x(n)*hamming(n);
#endif
#ifdef WINDOWING_HANN MUL_1(x_n_re[i],hannLUT[i]),x_n_re[i]); // x(n) = x(n)*hann(n);
#endif 
		}
	}

/* * main() */
void main() {
 /* 1. Init ADC */ // initADC(); // THIS NEEDS TODO:	
 // SETUP THE INPUT FROM STDIO OR INIT SOUNDCARD - OR SOMETHING 
 /* 2. Init Serial Port */ // PD7_bit.bit0 = 1; 
 // Set TX0 pin as output
// SCON0 = 0x42; // // SMD0 = 0x02; // // PR0 = 0x2F2F; 
// Set baud rate to 115200bps with fsysclk=20MHz 
/* 3. Init Hardware Multiplier */ // MCNT = 0x08; 
// Configure Hardware Multiplier for signed multiply 
// THIS NEEDS TODO: 
//I'm guessing - hoping - that our compiler is smart enough to manage this above section by itself with the right direction	
// we will be rewriting the headers anyways... check this! 

/* 4. FFT Loop */ 
	while (1) { 
/* 4.0. Variable Declaration and Initialization */ 
//__no_init unsigned int i; 
// Misc index int n_of_b = N_DIV_2; 
// Number of butterflies int s_of_b = 1; 
// Size of butterflies int a_index = 0; 
// fft data index int a_index_ref = 0; 
// fft data index reference char stage = 0; 
// Stage of the fft, 0 to (Log2(N)-1) 
//__no_init int nb_index; 
// Number of butterflies index //__no_init int sb_index; 
// Size of butterflies index int x_n_im[N] = {0x0000}; 
// Imaginary part of x(n) and X(n), // initialized to 0 before every fft 
/* 4.1. Get Input Samples from the ADC: Data will be stored in x_n_re */ getSamplesFromADC(); 
/* 4.2. Perform Bit-Reversal: 
Uses an unrolled loop that was created with the following C code: 
	#include <stdio.h> 
	#define N 256 
	#define LOG_2_N 8 
	int bitRev(int a, int nBits) { 
		int rev_a = 0; 
		for (int i=0; i<nBits; i++) { 
			rev_a = (rev_a << 1) | (a & 1); a = a >> 1; 
			} 
		return rev_a; 
		} 
	int main(int argc, char* argv[]) { 
		printf(" unsigned int i;\n"); 
		for(int i=0; i<N; i++) 
			if (bitRev(i,LOG_2_N) > i) { 
				printf(" i=x_n_re[%3d]; " ,i); 
				printf("x_n_re[%3d]=x_n_re[%3d]; ",i,bitRev(i,LOG_2_N)); 
				printf("x_n_re[%3d]=i;\n" ,bitRev(i,LOG_2_N)); 
				} 
				return 0; 
		} 
		*/ 

	i=x_n_re[ 1]; 
	x_n_re[ 1]=x_n_re[128]; 
	x_n_re[128]=i; 
	i=x_n_re[ 2]; 
	x_n_re[ 2]=x_n_re[ 64]; 
	x_n_re[ 64]=i; 
	i=x_n_re[ 3]; 
	x_n_re[ 3]=x_n_re[192]; 
	x_n_re[192]=i; 
	i=x_n_re[ 4]; 
	x_n_re[ 4]=x_n_re[ 32]; 
	x_n_re[ 32]=i; 
	i=x_n_re[ 5]; 
	x_n_re[ 5]=x_n_re[160]; 
	x_n_re[160]=i; 
	i=x_n_re[ 6]; 
	x_n_re[ 6]=x_n_re[ 96]; 
	x_n_re[ 96]=i; 
	i=x_n_re[ 7]; 
	x_n_re[ 7]=x_n_re[224]; 
	x_n_re[224]=i; 
	i=x_n_re[ 8]; 
	x_n_re[ 8]=x_n_re[ 16]; 
	x_n_re[ 16]=i; 
	i=x_n_re[ 9]; 
	x_n_re[ 9]=x_n_re[144]; 
	x_n_re[144]=i; 
	i=x_n_re[ 10]; 
	x_n_re[ 10]=x_n_re[ 80]; 
	x_n_re[ 80]=i; 
	i=x_n_re[ 11]; 
	x_n_re[ 11]=x_n_re[208]; 
	x_n_re[208]=i; 
	i=x_n_re[ 12]; 
	x_n_re[ 12]=x_n_re[ 48]; 
	x_n_re[ 48]=i; 
	i=x_n_re[ 13]; 
	x_n_re[ 13]=x_n_re[176]; 
	x_n_re[176]=i; 
	i=x_n_re[ 14]; 
	x_n_re[ 14]=x_n_re[112]; 
	x_n_re[112]=i; 
	i=x_n_re[ 15]; 
	x_n_re[ 15]=x_n_re[240]; 
	x_n_re[240]=i; 
	i=x_n_re[ 17]; 
	x_n_re[ 17]=x_n_re[136]; 
	x_n_re[136]=i; 
	i=x_n_re[ 18]; 
	x_n_re[ 18]=x_n_re[ 72]; 
	x_n_re[ 72]=i; i=x_n_re[ 19];
	x_n_re[ 19]=x_n_re[200];
	x_n_re[200]=i; 
	i=x_n_re[ 20]; 
	x_n_re[ 20]=x_n_re[ 40]; 
	x_n_re[ 40]=i; 
	i=x_n_re[ 21];
	x_n_re[ 21]=x_n_re[168]; 
	x_n_re[168]=i; 
	i=x_n_re[ 22]; 
	x_n_re[ 22]=x_n_re[104]; 
	x_n_re[104]=i; 
	i=x_n_re[ 23]; x_n_re[ 23]=x_n_re[232];
	x_n_re[232]=i;
	i=x_n_re[ 25];
	x_n_re[ 25]=x_n_re[152]; 
	x_n_re[152]=i; 
	i=x_n_re[ 26]; 
	x_n_re[ 26]=x_n_re[ 88]; 
	x_n_re[ 88]=i; i=x_n_re[ 27]; 
	x_n_re[ 27]=x_n_re[216]; 
	x_n_re[216]=i; i=x_n_re[ 28]; 
	x_n_re[ 28]=x_n_re[ 56]; 
	x_n_re[ 56]=i; i=x_n_re[ 29]; 
	x_n_re[ 29]=x_n_re[184]; 
	x_n_re[184]=i;
	i=x_n_re[ 30]; 
	x_n_re[ 30]=x_n_re[120];
	x_n_re[120]=i; 
	i=x_n_re[ 31]; 
	x_n_re[ 31]=x_n_re[248]; 
	x_n_re[248]=i; 
	i=x_n_re[ 33];
	x_n_re[ 33]=x_n_re[132]; 
	x_n_re[132]=i; 
	i=x_n_re[ 34]; 
	x_n_re[ 34]=x_n_re[ 68]; 
	x_n_re[ 68]=i; 
	i=x_n_re[ 35]; 
	x_n_re[ 35]=x_n_re[196];
	x_n_re[196]=i; i=x_n_re[ 37]; 
	x_n_re[ 37]=x_n_re[164];
	x_n_re[164]=i; 
	i=x_n_re[ 38]; 
	x_n_re[ 38]=x_n_re[100]; 
	x_n_re[100]=i; i=x_n_re[ 39]; 
	x_n_re[ 39]=x_n_re[228]; 
	x_n_re[228]=i; 
	i=x_n_re[ 41]; 
	x_n_re[ 41]=x_n_re[148]; 
	x_n_re[148]=i;
	i=x_n_re[ 42];
	x_n_re[ 42]=x_n_re[ 84]; 
	x_n_re[ 84]=i; 
	i=x_n_re[ 43];
	x_n_re[ 43]=x_n_re[212]; 
	x_n_re[212]=i; 
	i=x_n_re[ 44]; 
	x_n_re[ 44]=x_n_re[ 52]; 
	x_n_re[ 52]=i; 
	i=x_n_re[ 45]; 
	x_n_re[ 45]=x_n_re[180]; 
	x_n_re[180]=i; 
	i=x_n_re[ 46]; 
	x_n_re[ 46]=x_n_re[116]; 
	x_n_re[116]=i; 
	i=x_n_re[ 47]; 
	x_n_re[ 47]=x_n_re[244]; 
	x_n_re[244]=i; 
	i=x_n_re[ 49]; 
	x_n_re[ 49]=x_n_re[140]; 
	x_n_re[140]=i; 
	i=x_n_re[ 50]; 
	x_n_re[ 50]=x_n_re[ 76]; 
	x_n_re[ 76]=i; 
	i=x_n_re[ 51]; 
	x_n_re[ 51]=x_n_re[204]; 
	x_n_re[204]=i; 
	i=x_n_re[ 53]; 
	x_n_re[ 53]=x_n_re[172]; 
	x_n_re[172]=i; 
	i=x_n_re[ 54]; 
	x_n_re[ 54]=x_n_re[108]; 
	x_n_re[108]=i; 
	i=x_n_re[ 55];
	x_n_re[ 55]=x_n_re[236];
	x_n_re[236]=i;
	i=x_n_re[ 57];
	x_n_re[ 57]=x_n_re[156];
	x_n_re[156]=i;
	i=x_n_re[ 58];
	x_n_re[ 58]=x_n_re[ 92];
	x_n_re[ 92]=i;
	i=x_n_re[ 59]; 
	x_n_re[ 59]=x_n_re[220]; 
	x_n_re[220]=i; 
	i=x_n_re[ 61];
	x_n_re[ 61]=x_n_re[188]; 
	x_n_re[188]=i;
	i=x_n_re[ 62]; 
	x_n_re[ 62]=x_n_re[124]; 
	x_n_re[124]=i; 
	i=x_n_re[ 63];
	x_n_re[ 63]=x_n_re[252];
	x_n_re[252]=i; i=x_n_re[ 65];
	x_n_re[ 65]=x_n_re[130]; 
	x_n_re[130]=i; 
	i=x_n_re[ 67]; 
	x_n_re[ 67]=x_n_re[194];
	x_n_re[194]=i; 
	i=x_n_re[ 69]; 
	x_n_re[ 69]=x_n_re[162]; 
	x_n_re[162]=i; 
	i=x_n_re[ 70]; 
	x_n_re[ 70]=x_n_re[ 98];
	x_n_re[ 98]=i; 
	i=x_n_re[ 71]; 
	x_n_re[ 71]=x_n_re[226]; 
	x_n_re[226]=i; 
	i=x_n_re[ 73]; 
	x_n_re[ 73]=x_n_re[146]; 
	x_n_re[146]=i; 
	i=x_n_re[ 74]; 
	x_n_re[ 74]=x_n_re[ 82]; 
	x_n_re[ 82]=i; 
	i=x_n_re[ 75]; 
	x_n_re[ 75]=x_n_re[210]; 
	x_n_re[210]=i; 
	i=x_n_re[ 77]; 
	x_n_re[ 77]=x_n_re[178]; 
	x_n_re[178]=i; 
	i=x_n_re[ 78]; 
	x_n_re[ 78]=x_n_re[114]; 
	x_n_re[114]=i; 
	i=x_n_re[ 79]; 
	x_n_re[ 79]=x_n_re[242]; 
	x_n_re[242]=i; 
	i=x_n_re[ 81]; 
	x_n_re[ 81]=x_n_re[138]; 
	x_n_re[138]=i; 
	i=x_n_re[ 83]; 
	x_n_re[ 83]=x_n_re[202]; 
	x_n_re[202]=i; 
	i=x_n_re[ 85]; 
	x_n_re[ 85]=x_n_re[170]; 
	x_n_re[170]=i; 
	i=x_n_re[ 86]; 
	x_n_re[ 86]=x_n_re[106]; 
	x_n_re[106]=i; 
	i=x_n_re[ 87]; 
	x_n_re[ 87]=x_n_re[234]; 
	x_n_re[234]=i; 
	i=x_n_re[ 89]; 
	x_n_re[ 89]=x_n_re[154]; 
	x_n_re[154]=i; 
	i=x_n_re[ 91]; 
	x_n_re[ 91]=x_n_re[218]; 
	x_n_re[218]=i; 
	i=x_n_re[ 93]; 
	x_n_re[ 93]=x_n_re[186]; 
	x_n_re[186]=i; 
	i=x_n_re[ 94]; 
	x_n_re[ 94]=x_n_re[122]; 
	x_n_re[122]=i; 
	i=x_n_re[ 95]; 
	x_n_re[ 95]=x_n_re[250]; 
	x_n_re[250]=i; 
	i=x_n_re[ 97]; 
	x_n_re[ 97]=x_n_re[134]; 
	x_n_re[134]=i; 
	i=x_n_re[ 99]; 
	x_n_re[ 99]=x_n_re[198]; 
	x_n_re[198]=i; 
	i=x_n_re[101]; 
	x_n_re[101]=x_n_re[166]; 
	x_n_re[166]=i; 
	i=x_n_re[103]; 
	x_n_re[103]=x_n_re[230]; 
	x_n_re[230]=i; 
	i=x_n_re[105]; 
	x_n_re[105]=x_n_re[150]; 
	x_n_re[150]=i; 
	i=x_n_re[107]; 
	x_n_re[107]=x_n_re[214]; 
	x_n_re[214]=i; 
	i=x_n_re[109]; 
	x_n_re[109]=x_n_re[182]; 
	x_n_re[182]=i; 
	i=x_n_re[110]; 
	x_n_re[110]=x_n_re[118]; 
	x_n_re[118]=i; 
	i=x_n_re[111]; 
	x_n_re[111]=x_n_re[246]; 
	x_n_re[246]=i; 
	i=x_n_re[113]; 
	x_n_re[113]=x_n_re[142]; 
	x_n_re[142]=i; 
	i=x_n_re[115];
	x_n_re[115]=x_n_re[206]; 
	x_n_re[206]=i;
	i=x_n_re[117];
	x_n_re[117]=x_n_re[174]; 
	x_n_re[174]=i; 
	i=x_n_re[119];
	x_n_re[119]=x_n_re[238];
	x_n_re[238]=i; 
	i=x_n_re[121];
	x_n_re[121]=x_n_re[158]; 
	x_n_re[158]=i; 
	i=x_n_re[123]; 
	x_n_re[123]=x_n_re[222]; 
	x_n_re[222]=i;
	i=x_n_re[125];
	x_n_re[125]=x_n_re[190];
	x_n_re[190]=i;
	i=x_n_re[127];
	x_n_re[127]=x_n_re[254]; 
	x_n_re[254]=i; 
	i=x_n_re[131]; 
	x_n_re[131]=x_n_re[193]; 
	x_n_re[193]=i;
	i=x_n_re[133];
	x_n_re[133]=x_n_re[161]; 
	x_n_re[161]=i;
	i=x_n_re[135]; 
	x_n_re[135]=x_n_re[225];
	x_n_re[225]=i; i=x_n_re[137];
	x_n_re[137]=x_n_re[145];
	x_n_re[145]=i; 
	i=x_n_re[139];
	x_n_re[139]=x_n_re[209];
	x_n_re[209]=i; 
	i=x_n_re[141]; 
	x_n_re[141]=x_n_re[177]; 
	x_n_re[177]=i; 
	i=x_n_re[143]; 
	x_n_re[143]=x_n_re[241]; 
	x_n_re[241]=i; 
	i=x_n_re[147]; 
	x_n_re[147]=x_n_re[201]; 
	x_n_re[201]=i; 
	i=x_n_re[149]; 
	x_n_re[149]=x_n_re[169]; 
	x_n_re[169]=i; 
	i=x_n_re[151]; 
	x_n_re[151]=x_n_re[233]; 
	x_n_re[233]=i; 
	i=x_n_re[155]; 
	x_n_re[155]=x_n_re[217]; 
	x_n_re[217]=i; 
	i=x_n_re[157]; 
	x_n_re[157]=x_n_re[185]; 
	x_n_re[185]=i; 
	i=x_n_re[159]; 
	x_n_re[159]=x_n_re[249]; 
	x_n_re[249]=i; 
	i=x_n_re[163]; 
	x_n_re[163]=x_n_re[197]; 
	x_n_re[197]=i; 
	i=x_n_re[167]; 
	x_n_re[167]=x_n_re[229]; 
	x_n_re[229]=i; 
	i=x_n_re[171]; 
	x_n_re[171]=x_n_re[213]; 
	x_n_re[213]=i; 
	i=x_n_re[173]; 
	x_n_re[173]=x_n_re[181]; 
	x_n_re[181]=i; 
	i=x_n_re[175]; 
	x_n_re[175]=x_n_re[245]; 
	x_n_re[245]=i; 
	i=x_n_re[179]; 
	x_n_re[179]=x_n_re[205]; 
	x_n_re[205]=i; 
	i=x_n_re[183]; 
	x_n_re[183]=x_n_re[237]; 
	x_n_re[237]=i; 
	i=x_n_re[187]; 
	x_n_re[187]=x_n_re[221]; 
	x_n_re[221]=i; 
	i=x_n_re[191]; 
	x_n_re[191]=x_n_re[253]; 
	x_n_re[253]=i; 
	i=x_n_re[199]; 
	x_n_re[199]=x_n_re[227]; 
	x_n_re[227]=i; 
	i=x_n_re[203]; 
	x_n_re[203]=x_n_re[211]; 
	x_n_re[211]=i; i
	=x_n_re[207]; 
	x_n_re[207]=x_n_re[243]; 
	x_n_re[243]=i; 
	i=x_n_re[215]; 
	x_n_re[215]=x_n_re[235]; 
	x_n_re[235]=i; 
	i=x_n_re[223]; 
	x_n_re[223]=x_n_re[251]; 
	x_n_re[251]=i; 
	i=x_n_re[239]; 
	x_n_re[239]=x_n_re[247]; 
	x_n_re[247]=i; 

/* 4.3. FFT: loop through the 0 to log2(N) stages of the butterfly computations. 
When the FFT begins, the input samples (x(n)) are stored in x_n_re/x_n_im. 
When the FFT is done, the spectrum (X(n)) 
has replaced the input stored in x_n_re/x_n_im. */ 

	for(stage=0; stage<LOG_2_N; stage++) { 
		for(nb_index=0; nb_index<n_of_b; nb_index++) { 
			int tf_index = 0; 
// The twiddle factor index for(sb_index=0; sb_index<s_of_b; sb_index++) { 
//__no_init int resultMulReCos; 
//__no_init int resultMulImCos; 
//__no_init int resultMulReSin; 
//__no_init int resultMulImSin; int b_index = a_index+s_of_b; 
// 2nd fft data index // THIS NEEDS TODO: 
// THIS CALLS THE MACROS THAT NEED TO BE REWRITTEN!!! 
// Line 326 - 329 MUL_1(cosLUT[tf_index],x_n_re[b_index],resultMulReCos); 
			MUL_2(sinLUT[tf_index],resultMulReSin); 
			MUL_1(cosLUT[tf_index],x_n_im[b_index],resultMulImCos); 
			MUL_2(sinLUT[tf_index],resultMulImSin); 
			x_n_re[b_index] = x_n_re[a_index]-resultMulReCos+resultMulImSin; 
			x_n_im[b_index] = x_n_im[a_index]-resultMulReSin-resultMulImCos; 
			x_n_re[a_index] = x_n_re[a_index]+resultMulReCos-resultMulImSin; 
			x_n_im[a_index] = x_n_im[a_index]+resultMulReSin+resultMulImCos; 
			if (((sb_index+1) & (s_of_b-1)) == 0) 
				a_index = a_index_ref; 
			else 
				a_index++; 
			tf_index += n_of_b;
				} 
			a_index = ((s_of_b<<1) + a_index) & N_MINUS_1; 
			a_index_ref = a_index; 
			} 
		n_of_b >>= 1; s_of_b <<= 1;
		} 

/* 4.4. abs(X(n)): 
	Loop through N/2+1 (0 to N/2) FFT results (stored in x_n_re and x_n_im) and make all the values positive. 
	This will be needed for the algorithm used to compute the magnitude of X(n). */ 
	MUL_INIT(-1); 
	for(i=0; i<N_DIV_2_PLUS_1; i++) { 
	// If Re{X(n)} is negative, multiply by -1 if ((x_n_re[i] & 0x8000)!=0x0000) { MUL_NC(x_n_re[i],x_n_re[i]); } 
	// If Im{X(n)} is negative, multiply by -1 if ((x_n_im[i] & 0x8000)!=0x0000) { MUL_NC(x_n_im[i],x_n_im[i]); }
		} 

/* 4.5. |X(n)|: Compute the magniture of X(n) using a LUT. 
	This is possible only because the values of Re{X(n)} and Im{X(n)} are all positive. 
	Note that Im{X(0)} and Im{X(N/2)} contain no data and therefore comput
	ing |X(0)| and |X(N/2)| requires only the real part of X(n). 
	The magniture LUT is declared as: const unsigned char magnLUT[16][16] = {...}; 
	where the first index is abs(Re{X(n)}) and the second index is abs(Im{X(n)}). 
	Since X(n) is stored as Q8.7, 
	and the 4 most significant bits (2^4=16) are to be used as the indexes, 
	the Re{X(n)} and Im{X(n)} must be right shifted 11 positions. */ 
	x_n_re[0] = magnLUT[x_n_re[0]>>11][0]; 
	for(i=1; i<N_DIV_2; i++) 
		x_n_re[i] = magnLUT[x_n_re[i]>>11][x_n_im[i]>>11]; 
	x_n_re[N_DIV_2] = magnLUT[x_n_re[N_DIV_2]>>11][0]; 

/* 4.6. Xmit |X(n)|: 
	Transmits the magnitude of X(n) using UART0 */ 
	for(i=0; i<N_DIV_2_PLUS_1; i++) { 
	// THIS NEEDS TODO:	
	// Empty the output buffer in some way // while(!SCON0_bit.TI); 
	// Wait for empty buffer // SCON0_bit.TI = 0; 
	// Reset empty buffer flag // SBUF0 = x_n_re[i]; // Send magn } }
	}

