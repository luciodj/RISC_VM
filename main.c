/**
  Project: RISC-VM

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB® Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC16F1619
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

#include "mcc_generated_files/mcc.h"
#include <stdio.h>

#define serial_available()  ( EUSART_DataReady)
#define ESC         0x09    // tab
#define CMD_SIZE    32
#define FIFO_SIZE   8
#define fifo_available()    ( fifo_len)
#define log           

char cmd_string[ CMD_SIZE];
uint8_t cmd_len = 0;
char fifo_string[ FIFO_SIZE];
uint8_t fifo_len = 0;
uint8_t fifo_head = 0, fifo_tail = 0;

#define RAM_SIZE    50
#define ROM_SIZE    50

// registers
#define LNK         15
#define SP          14
#define BP          13
enum { MOV=0, LSL, ASR, ROR, AND, ANN, IOR, XOR, ADD, SUB, MUL, DIV};
enum { C_MI=0, C_EQ, C_LS=5, C_LE, C_TR, C_NE=9, C_GE=13, C_GT};

// global definition of the VM
uint32_t ROM[ ROM_SIZE];
uint32_t RAM[ RAM_SIZE];
uint32_t R[16], H;           // registers
uint16_t pc;                 // program counter
uint8_t N, Z;                // CPU flags

// VM flags and properties
uint8_t flag_run = false;   // VM flags
uint8_t flag_debug = false;
uint8_t flag_cmd = false;
uint8_t flag_step = false;

uint8_t error = 0;          // last error code 
#define ROM_ADDRESS_OUT_OF_BOUNDS   1
#define RAM_ADDRESS_OUT OF_BOUNDS   2

uint32_t gethex( char *p) {
    uint32_t value = 0;
    int8_t  x;
    while ( x = *p++) { 
        if (x > 'a') x-=32;  // make it lowercase
        if ( ( x < '0') || ( x > 'F'))  // invalid character or string terminated
            return value;
        x -= '0'; if ( x>9) x -= 7;
        value = ( value << 4) + x;
    }
    return value;
}

void fifo_append( char c) {
    if ( fifo_len < FIFO_SIZE){
        fifo_string[ fifo_head++] = c;
        if ( fifo_head >= FIFO_SIZE) fifo_head = 0;
        fifo_len++;
    }
} 

char fifo_get( void) {
    char c;
    if ( fifo_len > 0 ) {
        c = fifo_string[ fifo_tail++];
        if ( fifo_tail >= FIFO_SIZE) fifo_tail = 0;
        fifo_len--;
        return c;
    }
    else return 0;
}

int32_t fifo_getInt( void) {
    char s[4];
    // TODO getInt
    return 1;
}

int32_t _ROR( int32_t r, int32_t n){
    char b;
    while( n-- > 0){
        b = r & 1;
        r >>= 1;
        if ( b) r += 0x80000000;
    }
    return r;
}

void fetch( uint8_t *ir, uint8_t *op, int16_t *im) {
    uint16_t temp;
    if ( pc >= ROM_SIZE) {
        flag_run = false;
        error = ROM_ADDRESS_OUT_OF_BOUNDS;
        return;
    }
    temp = (uint16_t) ( ROM[ pc] >> 16);
    *im = (int16_t) ROM[ pc] ; 
    *ir  = temp >> 8; 
    *op  = (uint8_t) temp; 
    //printf(" ir:%02x op%02x: im:%04x\n", *ir, *op, *im );
    pc += 1;
} //fetch

void resetVM( void) {
    R[ SP] = RAM_SIZE;
    R[ BP] = 0;
    pc = 0;
    error = 0;
} // reset

void runVM( void) {
    uint8_t  ir, op;
    int16_t  im;
    int32_t  A, B, C; 

//     pq
//     00uv |  a(4) || b(4) | opcode(4) ||  xxxx  | c(4) |  # a = opcode( b, c)
//     01uv |  a(4) || b(4) | opcode(4) ||     imm(16)   |  # a = opcode( b, imm)
//     10uv |  a(4) || b(4) |           ||     off(16)   |  # load / store
//     11uv |cond(4)||         imm(8)   ||     imm(16)   |  # a = br(cond, imm)
        
    fetch( &ir, &op, &im);
    if ((ir & 0x80) == 0) {   // p = 0
        // register to register/immediate operations
        log("Register op: ");
        B = R[ op >> 4];
        log("B=%08lx ", B);
        if ( ir & 0x40 == 0) 
            C = R[ im & 0x0f];
        else { 
            if ( ir & 0x10)    // v =1 -> sign extension of im
                C = im;
            else 
                C = (uint16_t) im;  // do not sign extend                
        }
        log("C=%08lx ", C);
        switch( op & 0x0f) { 
            case MOV: A = C;    
                if ( ir & 0x20) A = H;      // u -> fetch H
                break; 
            case LSL: A = B << C; break;
            case ASR: A = B >> C; break;
            case ROR: A = _ROR( B, C); break;
            case AND: A = B & C; break;
            case ANN: A = B & ~C; break;
            case IOR: A = B | C; break;
            case XOR: A = B ^ C; break;
            case ADD: A = B + C; break;
            case SUB: A = B - C; break;
            case MUL: A = B * C; break;
            case DIV: A = B / C; H = B % C; break;
            }
        R[ ir & 0xf] = A;  // write result
        N = ( A < 0); Z = ( A == 0);    // set flags
        }

    else if ( (ir & 0x40) == 0) { // p = 1; q = 0
        // memory load store operations
        int16_t adr = (int16_t)R[ op>>4] + im;
        if ( (ir & 0x20) == 0) {   // u=0 -> load
            log("Load: ");
            if ( adr >= 0) 
                R[ ir & 0xf] = RAM[ adr>>2];
            else {  // special input function
                if ( adr == -4) { // text input
                    R[ ir &0xf] = fifo_getInt();
                }
            }
        }
        else {  // u = 1   -> store
            log( "Store:");
            if ( adr >= 0) 
                RAM[ adr>>2] = R[ ir & 0x0f];
            else { // special output functions
                if ( adr == -4)  printf( "> %08lx\n", R[ ir & 0xf]);
                else if ( adr == -8) putch( R[ ir & 0xf] & 0x7f);
                else if ( adr == -12) puts("");
            }
        }
    }

    else {      // branch instructions
        char condition;
        log("Jump: ");
        switch( ir & 0xf) {  // evaluate conditions
            case C_LS:  // less than, negative
            case C_MI: condition =  ( N ); break;
            case C_GE: condition = !( N ); break;
            case C_EQ: condition =  ( Z ); break;
            case C_NE: condition = !( Z ); break;
            case C_GT: condition = !( Z || N); break;
            case C_LE: condition =  ( Z || N); break;
            case C_TR: condition = true; break;
            default: condition = false; break;
        } 
        if ( condition) {
            if ( ir & 0x10) R[ LNK] = pc << 2;   // v=1 -> save return address 
            if ( ir & 0x20) { // u=1  -> relative jump
                pc += im;
            }
            else // u=0 -> absolute jump (address from register)
                pc = R[ im & 0x0f] >> 2;
        } // if condition
    }
    log("\n");
} // runVM

void cmd_init() {   
    cmd_string[0] = '\0';
    cmd_len = 0;
} // cmd_init

void cmd_append( char c) {
    if ( cmd_len < CMD_SIZE) 
        cmd_string[ cmd_len++] = c;
} // cmd_append

void cmd_interpret( void) {
    static uint16_t adr = 0;
    uint32_t word;
    // terminate command string 
    cmd_string[ cmd_len] = '\0';
    
    switch ( cmd_string[0]) {
        case 'w':   // write to ROM/RAM
            word = gethex( &cmd_string[2]);
            if ( 'a'==cmd_string[1]) {
                printf( "->RAM[%04x]=%08lx\n", adr, word); 
                RAM[adr]= word;
            }
            else if ('r'== cmd_string[1]) {
                adr &= 15;
                printf( "->R[%02x]=%08lx\n", adr, word);
                R[ adr] = word;
            }
            else { // wo
                printf( "->ROM[%04x]=%08lx\n", adr, word);
                ROM[adr]= word;
            }
            adr++;
            break;
        case 'r':   // get from ROM/RAM, example: GA0001 or GO0001
            adr = gethex( &cmd_string[2]);
            if ( 'a'==cmd_string[1]) 
                printf( "RAM[%04x]=%08lx\n", adr, RAM[adr]);
            else if ( 'r' == cmd_string[1])
                printf( "R[%02x]=%08lx\n", adr & 15, R[adr & 15]);
            else 
                printf( "ROM[%04x]=%08lx\n", adr, ROM[adr]);
            break;
        case 'd':   // debug on/off D1/D0
            flag_debug = ( '1'==cmd_string[1]);
            printf( "Debug=%d\n", flag_debug);
            break;
        case 'z':   // reset
            resetVM(); puts("Reset");
            break;
        case 'b':   // breakpoint set/clr
            // TODO implement breakpoints
            break;
        case 'h':   // halt
            flag_run = false; puts( "Halt");
            break;
        case 's':   // single step
            flag_step = true;
            flag_run = true;
            break;
        case 'x':   // execute
            flag_run = true; puts("Run");
            break;
        default:    // sync, return a version number
            puts( "<RISC v0.1>");
            //printf( "Error=%x\n", error);
            break;
    } // switch command                
} // cmd_interpret

void main( void) {
    int i;    
    char c;
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();

    // init RAM/ROM and VM
    for( i=0; i<ROM_SIZE; i++) ROM[ i] = 0;
ROM[0] = 0xE7000000; //	B 		+ 0x0
ROM[1] = 0x40000400; //	MOV 	R0 0x400
ROM[2] = 0xA0D00000; //	STW 	R0 BP [0]
ROM[3] = 0x40000001; //	MOV 	R0 0x1
ROM[4] = 0xA0D00004; //	STW 	R0 BP [4]
ROM[5] = 0x40000000; //	MOV 	R0 0x0
ROM[6] = 0xC7000000; //	B 		R0

    for( i=0; i<RAM_SIZE; i++) RAM[ i] = 0;
    resetVM();
    puts( "RISC VM");
    while( 1){
        if ( serial_available()) {     // check for input
            c = getch();
            if ( flag_cmd) {        // separate commands from text (escape sequence)
                if ( '\n' == c) {
                    flag_cmd = false;   // end of command string 
                    cmd_interpret();    // -> interpret
                } 
                else cmd_append( c);
            } // cmd
            else {                  // normal text input
                if ( ESC == c) { 
                    flag_cmd = true;
                    cmd_init();     // ready to receive command string
                }                    
                else fifo_append( c);  
            } // text
        } // available
        
        if ( flag_run) {            // call vm
            runVM();
            if ( flag_step || ( pc == 0)) {  // perform a single step
                flag_run = false;
                flag_step = false;
                if ( pc == 0) printf("STOP\n");
            }
            if ( flag_debug) { // update watch variables (escaped)
                printf( "PC=%04x\n", pc);
                printf( "N=%x, Z=%x\n", N, Z);
                for( i=0; i<16; i+=4){
                    char j;
                    for( j=0; j<4; j++)
                        printf( "R[%02d]=%08lx ", i+j, R[ i+j]);
                    putchar('\n');
                }
            } // debugging
        } // running
        
    } // while
}