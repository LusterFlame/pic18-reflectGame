/*
 * File:   LEDMatrix.c
 * Author: kuras
 *
 * Created on 2019/12/22, 17:38
 */

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>


// CONFIG1H
#pragma config CONFIG1H = 0x08;
//#pragma config OSC = RCIO6      // Oscillator Selection bits (External RC oscillator, port function on RA6)
//#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
//#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// For function use
#define _XTAL_FREQ 1000000
#define true 1
#define false 0

// For First MAX7219
#define CS_Pin1 PORTDbits.RD0
#define DIN_Pin1 PORTDbits.RD2
#define CLK_Pin1 PORTDbits.RD1

// For Second MAX7219
#define CS_Pin2 PORTDbits.RD5
#define DIN_Pin2 PORTDbits.RD6
#define CLK_Pin2 PORTDbits.RD4

#define COMBO_TRIGGER PORTDbits.RD7

void writeByte_1(unsigned short value);
void writeByte_2(unsigned short value);
void init_PIC(void);
void init_MAX_1(void);
void init_MAX_2(void);
void writeDisplay_1(unsigned short col, unsigned short value);
void writeDisplay_2(unsigned short col, unsigned short value);
void fillMatrix(unsigned char pattern);
void writeFullMatrix(unsigned char* matrix);
void rotateLeft(void);
void scrollLeft(void);
void spawn(int laneA, int laneB, int laneC, int laneD);

unsigned char display[16] = {
    /*
          Up -->
    <-- Down
    */
    0B00000000,
    0B00000000,
    0B00000000,
    0B00000000,
    0B00000000,
    0B00000000,
    0B00000000,
    0B00000000,
    0B00000000,
    0B00000000,
    0B00000000,
    0B00000000,
    0B00000000,
    0B00000000,
    0B00000000,
    0B00000000,
};

int combo;

void __interrupt(high_priority) Hi_ISR(void){
    if(INTCONbits.INT0F){
        INTCONbits.INT0F = 0;
        //PORTDbits.RD7 = 1;
        if(PORTCbits.RC7 == 1){
            if((display[2] & 0B11000000) == 0B11000000){
                display[2] = 0;
                ++combo;
            }
            else if((display[1] & 0B11000000) == 0B11000000){
                display[1] = 0;
                ++combo;
            }
            else if((display[3] & 0B11000000) == 0B11000000){
                display[3] = 0;
                ++combo;
            }
            else{
                combo = 0;
                COMBO_TRIGGER = 0;
            }
        }
        else if(PORTCbits.RC6 == 1){
            if((display[2] & 0B00110000) == 0B00110000){
                display[2] = 0;
                ++combo;
            }
            else if((display[1] & 0B00110000) == 0B00110000){
                display[1] = 0;
                ++combo;
            }
            else if((display[3] & 0B00110000) == 0B00110000){
                display[3] = 0;
                ++combo;
            }
            else{
                combo = 0;
                COMBO_TRIGGER = 0;
            }
        }
        else if(PORTCbits.RC5 == 1){
            if((display[2] & 0B00001100) == 0B00001100){
                display[2] = 0;
                ++combo;
            }
            else if((display[1] & 0B00001100) == 0B00001100){
                display[1] = 0;
                ++combo;
            }
            else if((display[3] & 0B00001100) == 0B00001100){
                display[3] = 0;
                ++combo;
            }
            else{
                combo = 0;
                COMBO_TRIGGER = 0;
            }
        }
        else if(PORTCbits.RC4 == 1){
            if((display[2] & 0B00000011) == 0B00000011){
                display[2] = 0;
                ++combo;
            }
            else if((display[1] & 0B00000011) == 0B00000011){
                display[1] = 0;
                ++combo;
            }
            else if((display[3] & 0B00000011) == 0B00000011){
                display[3] = 0;
                ++combo;
            }
            else{
                combo = 0;
                COMBO_TRIGGER = 0;
            }
        }
    }
    
    return;
}

void main(void) {
	unsigned int x;
    int repeat = 0;
    // Initialization
    init_PIC();
    init_MAX_1();
    init_MAX_2();
    srand(100);
    fillMatrix(0xff);
    
    RCONbits.IPEN = 1;
    INTCONbits.GIE = 1;
    INTCONbits.INT0E = 1;
    INTCONbits.INT0F = 0;
    
    combo = 0;
    
    do{
        TMR0H = 0;
        TMR0L = 0;
        writeFullMatrix(display);
        if(!repeat){
            if(rand() % 5 == 0){
                spawn(1, 0, 0, 0);
            }
            else if(rand() % 5 == 0){
                spawn(0, 1, 0, 0);
            }
            else if(rand() % 5 == 0){
                spawn(0, 0, 1, 0);
            }
            else{
                spawn(0, 0, 0, 1);
            }
        }
        if((display[0] & 0B11111111) != 0){
            combo = 0;
        }
        
        if(combo > 3){
            COMBO_TRIGGER = 1;
        }
        else{
            COMBO_TRIGGER = 0;
        }
        
        repeat = (repeat + 1) % 4;
        while((double)(unsigned int)TMR0 / 125000 < 60 / 173 / 4);
        scrollLeft();
    }while(true);
}

void writeByte_1(unsigned short value){
	unsigned short t, Mask, Flag;
    CLK_Pin1 = 0;
    Mask = 128;

	for (t = 0; t < 8; t++) { 
		Flag = value & Mask;
		if(Flag == 0) { 
			DIN_Pin1 = 0; 
		} 
		else { 
			DIN_Pin1 = 1; 
		} 
		CLK_Pin1 = 1; 
		CLK_Pin1 = 0; 
		Mask = Mask >> 1;
        // __delay_ms(2);
	}
}

void writeByte_2(unsigned short value){
	unsigned short t, Mask, Flag;
    CLK_Pin2 = 0;
    Mask = 128;

	for (t = 0; t < 8; t++) { 
		Flag = value & Mask;
		if(Flag == 0) { 
			DIN_Pin2 = 0; 
		} 
		else { 
			DIN_Pin2 = 1; 
		} 
		CLK_Pin2 = 1; 
		CLK_Pin2 = 0; 
		Mask = Mask >> 1;
        // __delay_ms(2);
	}
}


void init_MAX_1(void){
	// Set BCD decode mode
	CS_Pin1 = 0;         // CS pin is pulled LOW
	writeByte_1(0x09);    // Select Decode Mode register
	writeByte_1(0x00);    // Select BCD mode for digits DIG0-DIG7
	CS_Pin1 = 1;         // CS pin is pulled HIGH
	 
	// Set display brighness
	CS_Pin1 = 0;     // CS pin is pulled LOW
	writeByte_1(0x0A);    // Select Intensity register
	writeByte_1(0x05);    // Set brightness
	CS_Pin1 = 1;         // CS pin is pulled HIGH
	 
	// Set display refresh
	CS_Pin1 = 0;         // CS pin is pulled LOW
	writeByte_1(0x0B);    // Select Scan-Limit register
	writeByte_1(0x07);    // Select digits DIG0-DIG3
	CS_Pin1 = 1;         // CS pin is pulled HIGH
	 
	// Turn on the display
	CS_Pin1 = 0;         // CS pin is pulled LOW
	writeByte_1(0x0C);
	writeByte_1(0x01);
	CS_Pin1 = 1;         // CS pin is pulled HIGH
	 
	// Disable Display-Test
	CS_Pin1 = 0;         // CS pin is pulled LOW
	writeByte_1(0x0F);    // Select Display-Test register
	writeByte_1(0x00);    // Disable Display-Test
	CS_Pin1 = 1;         // CS pin is pulled HIGH
}

void init_MAX_2(void){
	// Set BCD decode mode
	CS_Pin2 = 0;         // CS pin is pulled LOW
	writeByte_2(0x09);    // Select Decode Mode register
	writeByte_2(0x00);    // Select BCD mode for digits DIG0-DIG7
	CS_Pin2 = 1;         // CS pin is pulled HIGH
	 
	// Set display brighness
	CS_Pin2 = 0;     // CS pin is pulled LOW
	writeByte_2(0x0A);    // Select Intensity register
	writeByte_2(0x05);    // Set brightness
	CS_Pin2 = 1;         // CS pin is pulled HIGH
	 
	// Set display refresh
	CS_Pin2 = 0;         // CS pin is pulled LOW
	writeByte_2(0x0B);    // Select Scan-Limit register
	writeByte_2(0x07);    // Select digits DIG0-DIG3
	CS_Pin2 = 1;         // CS pin is pulled HIGH
	 
	// Turn on the display
	CS_Pin2 = 0;         // CS pin is pulled LOW
	writeByte_2(0x0C);
	writeByte_2(0x01);
	CS_Pin2 = 1;         // CS pin is pulled HIGH
	 
	// Disable Display-Test
	CS_Pin2 = 0;         // CS pin is pulled LOW
	writeByte_2(0x0F);    // Select Display-Test register
	writeByte_2(0x00);    // Disable Display-Test
	CS_Pin2 = 1;         // CS pin is pulled HIGH
}

void init_PIC(void){
    TRISD = 0x00;      // Port D as outputs.
    TRISB = 0B11111111;
    TRISC = 0B11111111;
    PORTD = 0x00;
    T0CONbits.TMR0ON = 1;
    T0CONbits.T0PS2 = 0;
    T0CONbits.T0PS1 = 0;
    T0CONbits.T0PS0 = 0;
    T0CONbits.T08BIT = 0;
    T0CONbits.T0CS = 0;
    T0CONbits.PSA = 0;
}

void writeDisplay_1(unsigned short col, unsigned short value){
	CS_Pin1 = 0;            // select max7219.
	writeByte_1(col);       // send myColumn value to max7219 (digit place).
	writeByte_1(value); 	// send myValue value to max7219 (digit place).
	CS_Pin1 = 1;            // deselect max7219.
}
 
void writeDisplay_2(unsigned short col, unsigned short value){
	CS_Pin2 = 0;            // select max7219.
	writeByte_2(col);       // send myColumn value to max7219 (digit place).
	writeByte_2(value); 	// send myValue value to max7219 (digit place).
	CS_Pin2 = 1;            // deselect max7219.
}

// This is clear matrix function.
void fillMatrix(unsigned char pattern){
	unsigned short i;
	for(i = 1;i < 9;i++){
		writeDisplay_1(i, pattern);
        writeDisplay_2(i, pattern);
	}
}

void writeFullMatrix(unsigned char* matrix){
    // clearMatrix();
    short index, col;
    for(col = 1;col < 9;++col){
        index = col - 1;
        writeDisplay_1(col, matrix[index]);
        writeDisplay_2(col, matrix[index + 8]);
    }
}

void rotateLeft(void){
    unsigned char temp;
    temp = display[0];
    for(int i = 0, j = 1;j < 16;++i, ++j){
        display[i] = display[j];
    }
    display[15] = temp;
}

void scrollLeft(void){
    for(int i = 0, j = 1;j < 16;++i, ++j){
        display[i] = display[j];
    }
    display[15] = 0;
}

void spawn(int laneA, int laneB, int laneC, int laneD){
    if(laneA != 0){
        display[15] = display[15] | 0B11000000;
    }
    if(laneB != 0){
        display[15] = display[15] | 0B00110000;
    }
    if(laneC != 0){
        display[15] = display[15] | 0B00001100;
    }
    if(laneD != 0){
        display[15] = display[15] | 0B00000011;
    }
    laneA = laneB = laneC = laneD = 0;
}