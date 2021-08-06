/* 6502 family Debug monitor for use with NOICE02
// This monitor uses only the basic 6502 instructions.
// No 65C02 extended instructions are used
//
// Copyright (c) 2011 by John Hartman
//
// Modification History:
//   6-Feb-94 JLH ported from Mitsubishi 740 version
//  12-Feb-97 JLH wrong target type!  Change from 1 to 7 for 65(C)02
//  21-Jul-00 JLH change FN_MIN from F7 to F0
//  22-Sep-00 JLH add CALL address to TSTG
//  12-Mar-01 JLH V3.0: improve text about paging, formerly called "mapping"
//  27-Aug-11 JLH ported to Alfred Arnold assembler
//
//============================================================================
// This file has been assembled with the free Alfred Arnold 6502 assembler.
// http://john.ccac.rwth-aachen.de:8000/as/
//
// To customize for a given target, you must change code in the
// hardware equates, the string TSTG, and the routines RESET and REWDT.
// You may or may not need to change GETCHAR, PUTCHAR, depending on
// what UART you use.
//
// For more information, refer to the NoICE help file monitor.htm
//
// To add banked or paged memory support:
// 1) Define page latch port PAGELATCH here
// 2) If PAGELATCH is write only, define or import the latch port's RAM
//    image PAGEIMAGE here (The application code must update PAGEIMAGE
//    before outputing to PAGELATCH)
// 3) Search for and modify PAGELATCH, PAGEIMAGE, and REG_PAGE usage below
// 4) In TSTG below edit "LOW and HIGH LIMIT OF PAGED MEM"
//    to appropriate range (typically 8000H to BFFFH for two-bit MMU)
// For more information, refer to the NoICE help file 2bitmmu.htm
//
// This code was first used on the Mitsubishi MELPS 740 family.
// It should work on the 6502 and 65(C)02 with the following assumptions:
// (These agree with the description of the 6502 given in "6502 Assembly
// Language Programming" by Lance Leventhal, Osborne/McGraw-Hill, 1979)
// - Even though BRK is a one byte instruction, when a BRK occurs the PC
//   is incremented by 2 before being pushed.  If this is not true for your
//   processor, change the "SBC #2" instruction after INT_ENTRY to "SBC #1"
// - When a BRK occurs, the "B" bit is set BEFORE the status register is
//   pushed.  If this is not true for your processor, remove the "pla/pha"
//   after .IRQ and .NMI and repalce them with "PHP/pla"
// - If an interrupt is pending when a BRK is executed, PC will be loaded
//   with the vector for the INTERRUPT, not for BRK.  Thus, both .IRQ
//   and .NMI check for the break bit.  (This occurs on the 740.  According
//   to the Rockwell databook, it occurs on the 6502, but not the 65C02)
//
//============================================================================ */
        .cpu     _65c02
        .file [name="Mon6502.bin", type="bin", segments="Rom"]

// Set true for 16550 UART.  False for 6551.
// Anything else, and you need to provide code for GETCHAR, PUTCHAR, and init.
// #define UART_16550

// HARDWARE PLATFORM CUSTOMIZATIONS

.const RAM_START = $D800          //START OF MONITOR RAM
.const ROM_START = $F800          //START OF MONITOR CODE
.const HARD_VECT = $FFFA          //START OF HARDWARE VECTORS

// STACK RAM (PAGE 1)
// Monitor use is at most 7 bytes.
// This stack is shared with user programs.
//  Adjust INITSTACK as required by your application
.const INITSTACK = $1FF           //TOP OF STACK RAM

//======================================================================
// HARDWARE PLATFORM INDEPENDENT EQUATES and CODE
//
// Communications function codes.
.const FN_GET_STATUS   =  $FF     // reply with device info
.const FN_READ_MEM     =  $FE     // reply with data
.const FN_WRITE_MEM    =  $FD     // reply with status (+/-)
.const FN_READ_REGS    =  $FC     // reply with registers
.const FN_WRITE_REGS   =  $FB     // reply with status
.const FN_RUN_TARGET   =  $FA     // reply (delayed) with registers
.const FN_SET_BYTES    =  $F9     // reply with data (truncate if error)
.const FN_IN           =  $F8     // input from port
.const FN_OUT          =  $F7     // output to port
.const FN_MIN          =  $F0     // MINIMUM RECOGNIZED FUNCTION CODE
.const FN_ERROR        =  $F0     // error reply to unknown op-code

// 6502 OP-CODE EQUATES
.const B               =  $10     // BREAK BIT IN CONDITION CODES
.const lda_OP          =  $AD     // lda AAA
.const sta_OP          =  $8D     // sta AAA
.const CMP_OP          =  $CD     // CMP AAA
.const ldaY_OP         =  $B9     // lda AAA,Y
.const staY_OP         =  $99     // sta AAA,Y
.const CMPY_OP         =  $D9     // CMP AAA,Y
.const RTS_OP          =  $60     // rts

//==========================================================================
#if UART_16550
        //  Equates for 16550 serial port
        //
        .const S16450  =  $3000           //  base of 16450 UART
        .const RXR     =  0               //  Receiver buffer register
        .const TXR     =  0               //  Transmitter buffer register
        .const IER     =  1               //  Interrupt enable register
        .const LCR     =  3               //  Line control register
        .const MCR     =  4               //  Modem control register
        .const LSR     =  5               //  Line status register

        // Define monitor serial port
        .const SERIAL_STATUS   =  S16450+LSR
        .const SERIAL_RXDATA   =  S16450+RXR
        .const SERIAL_TXDATA   =  S16450+TXR
        .const RXRDY           =  $01     // BIT MASK FOR RX BUFFER FULL
        .const TXRDY           =  $20     // BIT MASK FOR TX BUFFER EMPTY

#else
        // Use A//IA (6551) for RS-232
        // The 50747 always reads an address before writing it.  This, of course, 
        // is hard on pending inputs when writing to an ACIA.  If your processor
        // does this, you may wish to doctor the chip select logic so that
        // A2=0 writes to the chip, A2=1 reads from the chip.  Thus we
        // have Read and Write addresses for each register.
        //
        // The hardware connects RS-232 pin 8 (DCD) to 6551 DSR.  This because
        // if a 6551's DCD is not true, we cannot receive anything.  Using DSR
        // permits simple cables to be used for TTY applications.
        //
        // The hardware connects RS-232 pin 20 (DTR) to 6551 RST.  This because
        // the Hayes modem and direct connect cables use DTR for signaling.
        .const SERIAL_TXDATA   =  $3000           // ACIA WRITE DATA
        .const SERIAL_RXDATA   =  $3004           // ACIA READ DATA
        .const SERIAL_RESET    =  $3001           // WRITE (STATUS) TO RESET ACIA
        .const SERIAL_STATUS   =  $3005           // ACIA STATUS
        .const SERIAL_WCMD     =  $3002           // ACIA COMMAND REGISTER
        .const SERIAL_WCTL     =  $3003           // ACIA CONTROL REGISTER
        .const RXRDY           =  $08             // RECEIVE READY
        .const TXRDY           =  $10             // TRANSMIT READY
#endif

// ============================================================================
//  RAM definitions
        .segment Ram[]
        * = RAM_START

//  RAM interrupt vectors (first in segment for easy addressing, else move to
//  their own segment)
RAMVEC:          .fill    2*3, 0

// Target registers: order must match that used by NoICE on the PC
TASK_REGS:
REG_STATE:       .byte    0
REG_PAGE:        .byte    0
REG_SP:          .word    0
REG_Y:           .byte    0
REG_X:           .byte    0
REG_A:           .byte    0
REG_CC:          .byte    0
REG_PC:          .word    0
TASK_REG_END:
.const TASK_REGS_SIZE =  TASK_REG_END-TASK_REGS

// In order that we need no page zero RAM, do memory access via an
// instruction built into RAM.  Build instruction and rts here
CODEBUF:         .dword   0       // ROOM FOR "lda xxxx, rts"

// Store a counter for input timeout
RXTIMER:         .word    0

// Communications buffer
// (Must be at least as long as TASK_REG_SZ.  At least 19 bytes recommended.
// Larger values may improve speed of NoICE memory move commands.)
.const COMBUF_SIZE = 128             // DATA SIZE FOR COMM BUFFER
COMBUF:          .fill    2+COMBUF_SIZE+1, 0 // BUFFER ALSO HAS FN, LEN, and CHECK

RAM_END:                               // ADDRESS OF TOP+1 OF RAM

//===========================================================================
        .segment Rom[]
        * = ROM_START

// Power on reset
RESET:

// Set CPU mode to safe state
        nop                     //DATA BOOK SHOWS THIS - DON'T KNOW WHY
        sei                     //INTERRUPTS OFF
        cld                     //USE BINARY MODE

// INITIALIZE TARGET HARDWARE

// INIT STACK
        ldx     #INITSTACK & $FF
        txs

//===========================================================================
#if UART_16550
// Initialize 16550 UART
//
// Access baud generator, no parity, 1 stop bit, 8 data bits
        lda     #$83    // 10000011B
        sta     S16450+LCR

// Fixed baud rate of 19200: Divisor is clock/(16*baud)
// Our hardware has 3.686400 Mhz.
// Your baud rate value depends on your hardware
        lda     #12                     //fix at 19.2 kbaud
        sta     S16450+RXR              //lsb
        lda     #0
        sta     S16450+RXR+1            //msb=0

// Access data registers, no parity, 1 stop bits, 8 data bits
        lda     #$03    //00000011B
        sta     S16450+LCR

// No loopback, OUT2 on, OUT1 on, rts on, DTR (LED) on
        lda     #$0F    //00001111B
        sta     S16450+MCR

// Disable all interrupts: modem, receive error, transmit, and receive
        lda     #$00    //00000000B
        sta     S16450+IER
#else
// INIT MONITOR ACIA: 19200 BAUD, 8 BITS, NO PARITY, 1 STOP BIT
// Baud rate value depends on your hardware
        lda     #0
        sta     SERIAL_RESET            //PROGRAMMED RESET
        lda     #$1F    //00011111B
        sta     SERIAL_WCTL             //1SB, 8 DATA, 19200 BAUD
        lda     #$0B    //00001011B
        sta     SERIAL_WCMD             //NO PARITY, rts, NO TX OR RX INT
#endif

//===========================================================================
// INIT RAM INTERRUPT VECTORS TO DUMMY HANDLERS
        lda     #_NMIX & $FF      // NMI
        sta     RAMVEC+0
        lda     #_NMIX / $100
        sta     RAMVEC+0+1

        lda     #_IRQX & $FF      // IRQ
        sta     RAMVEC+4
        lda     #_IRQX / $100
        sta     RAMVEC+4+1

// Initialize user registers
        ldx     #INITSTACK & $FF
        stx     REG_SP                  //INIT USER'S STACK POINTER
        ldx     #INITSTACK / $100
        stx     REG_SP+1
        lda     #0
        sta     REG_PC
        sta     REG_PC+1
        sta     REG_A
        sta     REG_X
        sta     REG_Y
        sta     REG_CC
        sta     REG_STATE               //STATE IS 0 = RESET

// Initialize memory paging variables and hardware (if any)
        sta     REG_PAGE                // NO PAGE YET
//      sta     PAGEIMAGE
//      sta     PAGELATCH               // set hardware page

// Set function code for "GO".  Then if we reset after being told to
// GO, we will come back with registers so user can see the crash
        lda     #FN_RUN_TARGET
        sta     COMBUF
        jmp     RETURN_REGS             // DUMP REGS, ENTER MONITOR

// ===========================================================================
//  Get a character to A
// 
//  Return A=char, CY=0 if data received
//         CY=1 if timeout (0.5 seconds)
// 
//  Uses 4 bytes of stack including return address
// 
GETCHAR:
        lda     #0              //LONG TIMEOUT
        sta     RXTIMER
        sta     RXTIMER+1
GC10:   jsr     REWDT           //PREVENT WATCHDOG TIMEOUT
        dec     RXTIMER
        bne     GC20            //BR IF NOT TIMEOUT
        dec     RXTIMER+1       //ELSE DEC HIGH HALF
        beq     GC90            //EXIT IF TIMEOUT
GC20:   lda     SERIAL_STATUS   //READ DEVICE STATUS
        and     #RXRDY
        beq     GC10            //NOT READY YET.

// Data received:  return CY=0. data in A
        clc                     //CY=0
        lda     SERIAL_RXDATA   //READ DATA
        rts

// Timeout:  return CY=1
GC90:   sec                     //CY=1
        rts

//===========================================================================
// Output character in A
//
// Uses 5 bytes of stack including return address
//
PUTCHAR:
        pha
PC10:   jsr     REWDT           //PREVENT WATCHDOG TIMEOUT
        lda     SERIAL_STATUS   //CHECK TX STATUS
        and     #TXRDY          //RX READY ?
        beq     PC10
        pla
        sta     SERIAL_TXDATA   //TRANSMIT CHAR.
        rts

//======================================================================
//
// RESET WATCHDOG TIMER.  MUST BE CALLED AT LEAST ONCE EVERY ? MSEC
// OR PROCESSOR WILL BE RESET
//
// Uses 2 bytes of stack including return address
//
REWDT:  // you know if you need it
        rts

//======================================================================
// Response string for GET TARGET STATUS request
// Reply describes target:
TSTG:   .byte     7                       // 2: PROCESSOR TYPE = 65(C)02
        .byte     COMBUF_SIZE             // 3: SIZE OF COMMUNICATIONS BUFFER
        .byte     $80                     // 4: has CALL
        .word     0                       // 5-8: LOW and HIGH LIMIT OF MAPPED MEM (NONE)
        .byte     B1-B0                   // 9 BREAKPOINT INSTR LENGTH

// Define either the BRK or jsr BRKE instruction for use as breakpoint
// Arnold assembles BRK as two bytes: 00 EA.  We want a ONE byte breakpoint
// so we do it by hand.
B0:     .byte    $00                      // 10+ BREKAPOINT INSTRUCTION
// B0:  jsr     BRKE                    //10+ BREKAPOINT INSTRUCTION
        .encoding "ascii"
B1:     .text    "6502 monitor V3.0"    // DESCRIPTION
        .byte   0                       // NULL terminator
        .byte   0                       // page of CALL breakpoint
        .byte   B0                      // address of CALL breakpoint in native order
B2:
.const TSTG_SIZE = B2 - TSTG            // SIZE OF STRING

// ===========================================================================
//  Enter here via jsr for breakpoint:  PC is stacked.
//  Stacked PC points at jsr+1
// //BRKE: sta     REG_A           //SAVE ACCUM FROM DIRECT ENTRY
// //      PHP                     //SAVE CC'S AS IF AFTER A BRK INSTRUCTION
// //      SEC
// 
//  Common handler for default interrupt handlers
//  Enter with A=interrupt code = processor state
//  PC and CC are stacked.
//  REG_A has pre-interrupt accmulator
//  Stacked PC points at BRK+2 if BRK, else at PC if entry from interrupt
//  A  has state
INT_ENTRY:

// Set CPU mode to safe state
        nop                     // DATA BOOK SHOWS THIS - DON'T KNOW WHY
        sei                     // INTERRUPTS OFF
        cld                     // USE BINARY MODE

// Save registers in reg block for return to master
        sta     REG_STATE       // SAVE MACHINE STATE
        pla                     // GET CONDITION CODES
        sta     REG_CC
        pla                     // GET LSB OF PC OF BREAKPOINT
        sta     REG_PC
        pla                     // GET MSB OF PC OF BREAKPOINT
        sta     REG_PC+1

// If this is a breakpoint (state = 1), then back up PC to point at BRK
        lda     REG_STATE       // SAVED STATUS FOR TESTING
        cmp     #1
        bne     B99             // BR IF NOT BREAKPOINT: PC IS OK

// On the 6502, BRK leaves PC at break address +2:  back it up by 2
// If your core leaves PC at break address +1, back up by 1
        sec
        lda     REG_PC          //BACK UP PC TO POINT AT BREAKPOINT
        sbc     #2
        sta     REG_PC
        lda     REG_PC+1
        sbc     #0
        sta     REG_PC+1
B99:    jmp     ENTER_MON       //REG_PC POINTS AT BREAKPOINT OPCODE
//
//===========================================================================
//
// Main loop:  wait for command frame from master
//
// Uses 4 bytes of stack before jump to functions
//
MAIN:

// Since we have only part of a page for stack, we run on the target's
// stack.  Thus, reset to target SP, rather than our own.
MAI10:  ldx     REG_SP
        txs
        ldx     #0                      //INIT INPUT BYTE COUNT

// First byte is a function code
        jsr     GETCHAR                 //GET A FUNCTION
        bcs     MAI10                   //JIF TIMEOUT: RESYNC
        cmp     #FN_MIN
        bcc     MAI10                   //JIF BELOW MIN: ILLEGAL FUNCTION
        sta     COMBUF,X                //SAVE FUNCTION CODE
        inx

// Second byte is data byte count (may be zero)
        jsr     GETCHAR                 //GET A LENGTH BYTE
        bcs     MAI10                   //JIF TIMEOUT: RESYNC
        cmp     #COMBUF_SIZE+1
        bcs     MAI10                   //JIF TOO LONG: ILLEGAL LENGTH
        sta     COMBUF,X                //SAVE LENGTH
        inx
        cmp     #0
        beq     MAI80                   //SKIP DATA LOOP IF LENGTH = 0

// Loop for data
        tay                             //SAVE LENGTH FOR LOOP
MAI20:  jsr     GETCHAR                 //GET A DATA BYTE
        bcs     MAI10                   //JIF TIMEOUT: RESYNC
        sta     COMBUF,X                //SAVE DATA BYTE
        inx
        dey
        bne     MAI20

// Get the checksum
MAI80:  jsr     GETCHAR                 //GET THE CHECKSUM
        bcs     MAI10                   //JIF TIMEOUT: RESYNC
        sta     CODEBUF                 //SAVE CHECKSUM

// Compare received checksum to that calculated on received buffer
// (Sum should be 0)
        jsr     CHECKSUM
        clc
        adc     CODEBUF
        bne     MAI10                   //JIF BAD CHECKSUM

// Process the message.
        lda     COMBUF+0                //GET THE FUNCTION CODE
        cmp     #FN_GET_STATUS
        beq     TARGET_STATUS
        cmp     #FN_READ_MEM
        beq     JREAD_MEM
        cmp     #FN_WRITE_MEM
        beq     JWRITE_MEM
        cmp     #FN_READ_REGS
        beq     JREAD_REGS
        cmp     #FN_WRITE_REGS
        beq     JWRITE_REGS
        cmp     #FN_RUN_TARGET
        beq     JRUN_TARGET
        cmp     #FN_SET_BYTES
        beq     JSET_BYTES
        cmp     #FN_IN
        beq     JIN_PORT
        cmp     #FN_OUT
        beq     JOUT_PORT

// Error: unknown function.  Complain
        lda     #FN_ERROR
        sta     COMBUF          //SET FUNCTION AS "ERROR"
        lda     #1
        jmp     SEND_STATUS     //VALUE IS "ERROR"

// long jumps to handlers
JREAD_MEM:      jmp     READ_MEM
JWRITE_MEM:     jmp     WRITE_MEM
JREAD_REGS:     jmp     READ_REGS
JWRITE_REGS:    jmp     WRITE_REGS
JRUN_TARGET:    jmp     RUN_TARGET
JSET_BYTES:     jmp     SET_BYTES
JIN_PORT:       jmp     IN_PORT
JOUT_PORT:      jmp     OUT_PORT

//===========================================================================
//
// Target Status:  FN, len
TARGET_STATUS:
        ldx     #0                      //DATA FOR REPLY
        ldy     #TSTG_SIZE              //LENGTH OF REPLY
        sty     COMBUF+1                //SET SIZE IN REPLY BUFFER
TS10:   lda     TSTG,X                  //MOVE REPLY DATA TO BUFFER
        sta     COMBUF+2,X
        inx
        dey
        bne     TS10

// Compute checksum on buffer, and send to master, then return
        jmp     SEND

//===========================================================================
//
// Read Memory:  FN, len, page, Alo, Ahi, Nbytes
//
READ_MEM:

// Set page
////      lda     COMBUF+2
////      sta     PAGEIMAGE
////      sta     PAGELATCH
//
// Build "lda  AAAA,Y" in RAM
        lda     #ldaY_OP
        sta     CODEBUF+0

// Set address of instruction in RAM
        lda     COMBUF+3
        sta     CODEBUF+1
        lda     COMBUF+4
        sta     CODEBUF+2

// Set return after lda
        lda     #RTS_OP
        sta     CODEBUF+3

// Prepare return buffer: FN (unchanged), LEN, DATA
        ldx     COMBUF+5                //NUMBER OF BYTES TO GET
        stx     COMBUF+1                //RETURN LENGTH = REQUESTED DATA
        beq     GLP90                   //JIF NO BYTES TO GET

// Read the requested bytes from local memory
        ldy     #0                      //INITIAL OFFSET
GLP:    jsr     CODEBUF                 //GET BYTE AAAA,Y TO A
        sta     COMBUF+2,Y              //STORE TO RETURN BUFFER
        iny
        dex
        bne     GLP

// Compute checksum on buffer, and send to master, then return
GLP90:  jmp     SEND

//===========================================================================
//
// Write Memory:  FN, len, page, Alo, Ahi, (len-3 bytes of Data)
//
// Uses 2 bytes of stack

WRITE_MEM:

// Set page
////      lda     COMBUF+2
////      sta     PAGEIMAGE
////      sta     PAGELATCH

// Build "sta  AAAA,Y" in RAM
        lda     #staY_OP
        sta     CODEBUF+0

// Set address into RAM
        lda     COMBUF+3
        sta     CODEBUF+1
        lda     COMBUF+4
        sta     CODEBUF+2

// Set return after sta
        lda     #RTS_OP
        sta     CODEBUF+3

// Prepare return buffer: FN (unchanged), LEN, DATA
        ldx     COMBUF+1                //NUMBER OF BYTES TO PUT
        dex                             //LESS PAGE, ADDRLO, ADDRHI
        dex
        dex
        beq     WLP50                   //JIF NO BYTES TO PUT

// Write the specified bytes to local memory
        ldy     #0                      //INITIAL OFFSET
WLP:    lda     COMBUF+5,Y              //GET BYTE TO WRITE
        jsr     CODEBUF                 //STORE THE BYTE AT AAAA,Y
        iny
        dex
        bne     WLP

// Build "CMP  AAAA,Y" in RAM
        lda     #CMPY_OP
        sta     CODEBUF+0

// Compare to see if the write worked
        ldx     COMBUF+1                //NUMBER OF BYTES TO PUT
        dex                             //LESS PAGE, ADDRLO, ADDRHI
        dex
        dex
        ldy     #0                      //INITIAL OFFSET
WLP20:  lda     COMBUF+5,Y              //GET BYTE JUST WRITTEN
        jsr     CODEBUF                 //COMPARE THE BYTE AT AAAA,Y
        bne     WLP80                   //BR IF WRITE FAILED
        iny
        dex
        bne     WLP20

// Write succeeded:  return status = 0
WLP50:  lda     #0                      //RETURN STATUS = 0
        jmp     WLP90

// Write failed:  return status = 1
WLP80:  lda     #1
//
// Return OK status
WLP90:  jmp     SEND_STATUS

//===========================================================================
//
// Read registers:  FN, len=0
//
READ_REGS:

// Enter here from "RUN" and "STEP" to return task registers
RETURN_REGS:
        ldx     #0                      //REGISTER LIVE HERE
        ldy     #TASK_REGS_SIZE         //NUMBER OF BYTES
        sty     COMBUF+1                //SAVE RETURN DATA LENGTH

// Copy the registers
GRLP:   lda     TASK_REGS,X             //GET BYTE TO A
        sta     COMBUF+2,X              //STORE TO RETURN BUFFER
        inx
        dey
        bne     GRLP

// Compute checksum on buffer, and send to master, then return
        jmp     SEND

//===========================================================================
//
// Write registers:  FN, len, (register image)

WRITE_REGS:

        ldx     #0                      //POINTER TO DATA
        ldy     COMBUF+1                //NUMBER OF BYTES
        beq     WRR80                   //JIF NO REGISTERS

// Copy the registers
WRRLP:  lda     COMBUF+2,X              //GET BYTE TO A
        sta     TASK_REGS,X             //STORE TO REGISTER RAM
        inx
        dey
        bne     WRRLP

// Reload SP, in case it has changed
        ldx     REG_SP
        txs

// Return OK status
WRR80:  lda     #0
        jmp     SEND_STATUS

//===========================================================================
//
// Run Target:  FN, len
//
// Uses 3 bytes of stack for user PC and CC before RTI

RUN_TARGET:

// Restore user's page
//////     lda     REG_PAGE                //USER'S PAGE
//////     sta     PAGEIMAGE
//////     sta     PAGELATCH               //set hardware page

// Switch to user stack, if not already running on it
        ldx     REG_SP                  //BACK TO USER STACK
        txs
        lda     REG_PC+1                //SAVE MS USER PC FOR RTI
        pha
        lda     REG_PC                  //SAVE LS USER PC FOR RTI
        pha
        lda     REG_CC                  //SAVE USER CONDITION CODES FOR RTI
        pha

// Restore registers
        ldx     REG_X
        ldy     REG_Y
        lda     REG_A

// Return to user
        rti

//===========================================================================

// Common continue point for all monitor entrances
// REG_STATE, REG_A, REG_CC, REG_PC set// X, Y intact// SP = user stack
ENTER_MON:
        stx     REG_X
        sty     REG_Y
        tsx
        stx     REG_SP          //SAVE USER'S STACK POINTER (LSB)
        lda     #1              //STACK PAGE ALWAYS 1
EM10:   sta     REG_SP+1        //(ASSUME PAGE 1 STACK)

// With only a partial page for the stack, don't switch
//////        ldx  #MONSTACK       //and USE OURS INSTEAD
//////        TXS
//////     lda     PAGEIMAGE       //GET CURRENT USER PAGE
        lda     #0              //... OR ZERO IF UNPAGED TARGET
        sta     REG_PAGE        //SAVE USER'S PAGE

// Return registers to master
        jmp     RETURN_REGS

//===========================================================================
//
// Set target byte(s):  FN, len { (page, alow, ahigh, data), (...)... }
//
// Return has FN, len, (data from memory locations)
//
// If error in insert (memory not writable), abort to return short data
//
// This function is used primarily to set and clear breakpoints
//
// Uses 2 bytes of stack

SET_BYTES:
        ldy     COMBUF+1                //LENGTH = 4*NBYTES
        beq     SB90                    //JIF NO BYTES

// Loop on inserting bytes
        ldx     #0                      //INDEX INTO INPUT BUFFER
        ldy     #0                      //INDEX INTO OUTPUT BUFFER
SB10:

// Set page
////      lda     COMBUF+2,X
////      sta     PAGEIMAGE
////      sta     PAGELATCH

// Build "lda  AAAA" in RAM
        lda     #lda_OP
        sta     CODEBUF+0

// Set address
        lda     COMBUF+3,X
        sta     CODEBUF+1
        lda     COMBUF+4,X
        sta     CODEBUF+2

// Set return after lda
        lda     #RTS_OP
        sta     CODEBUF+3

// Read current data at byte location
        jsr     CODEBUF                 //GET BYTE AT AAAA
        sta     COMBUF+2,Y              //SAVE IN RETURN BUFFER

// Insert new data at byte location

// Build "sta  AAAA" in RAM
        lda     #sta_OP
        sta     CODEBUF+0
        lda     COMBUF+5,X              //BYTE TO WRITE
        jsr     CODEBUF

// Verify write
        lda     #CMP_OP
        sta     CODEBUF+0
        lda     COMBUF+5,X
        jsr     CODEBUF
        bne     SB90                    //BR IF INSERT FAILED: ABORT AT Y BYTES

// Loop for next byte
        iny                             //COUNT ONE INSERTED BYTE
        inx                             //STEP TO NEXT BYTE SPECIFIER
        inx
        inx
        inx
        cpx     COMBUF+1
        bne     SB10                    //LOOP FOR ALL BYTES

// Return buffer with data from byte locations
SB90:   sty     COMBUF+1                //SET COUNT OF RETURN BYTES

// Compute checksum on buffer, and send to master, then return
        jmp     SEND

//===========================================================================
//
// Input from port:  FN, len, PortAddressLo, PAhi (=0)
//
// While the M740 has no input or output instructions, we retain these
// to allow write-without-verify
//
IN_PORT:
//
// Build "lda  AAAA" in RAM
        lda     #lda_OP
        sta     CODEBUF+0
//
// Set port address
        lda     COMBUF+2
        sta     CODEBUF+1
        lda     COMBUF+3
        sta     CODEBUF+2
//
// Set return after lda
        lda     #RTS_OP
        sta     CODEBUF+3
//
// Read the requested byte from local memory
        jsr     CODEBUF                 //GET BYTE TO A
//
// Return byte read as "status"
        jmp     SEND_STATUS

//===========================================================================
//
// Output to port:  FN, len, PortAddressLo, PAhi (=0), data
//
OUT_PORT:
//
// Build "sta  AAAA" in RAM
        lda     #sta_OP
        sta     CODEBUF+0
//
// Set port address
        lda     COMBUF+2
        sta     CODEBUF+1
        lda     COMBUF+3
        sta     CODEBUF+2
//
// Set return after sta
        lda     #RTS_OP
        sta     CODEBUF+3
//
// Get data
        lda     COMBUF+4
//
// Write value to port
        jsr     CODEBUF         //PUT BYTE FROM A
//
// Do not read port to verify (some I/O devices don't like it)
//
// Return status of OK
        lda     #0
        jmp     SEND_STATUS

//===========================================================================
// Build status return with value from "A"
//
SEND_STATUS:
        sta     COMBUF+2                //SET STATUS
        lda     #1
        sta     COMBUF+1                //SET LENGTH
        jmp     SEND

//===========================================================================
// Append checksum to COMBUF and send to master
//
SEND:   jsr     CHECKSUM                //GET A=CHECKSUM, X->checksum location
        eor     #$FF
        clc
        adc     #1
        sta     COMBUF,X                //STORE NEGATIVE OF CHECKSUM
//
// Send buffer to master
        ldx     #0                      //POINTER TO DATA
        ldy     COMBUF+1                //LENGTH OF DATA
        iny                             //PLUS FUNCTION, LENGTH, CHECKSUM
        iny
        iny
SND10:  lda     COMBUF,X
        jsr     PUTCHAR                 //SEND A BYTE
        inx
        dey
        bne     SND10
        jmp     MAIN                    //BACK TO MAIN LOOP

//===========================================================================
// Compute checksum on COMBUF.  COMBUF+1 has length of data,
// Also include function byte and length byte
//
// Returns:
//      A = checksum
//      X = pointer to next byte in buffer (checksum location)
//      Y is scratched
//
CHECKSUM:
        ldx     #0                      //pointer to buffer
        ldy     COMBUF+1                //length of message
        iny                             //plus function, length
        iny
        lda     #0                      //init checksum to 0
CHK10:  clc
        adc     COMBUF,X
        inx
        dey
        bne     CHK10                   //loop for all
        rts                             //return with checksum in A

//**********************************************************************
//
// VECTORS THROUGH RAM
//
// ON THE MELPS740 (from which this was ported...)
// We observe a nasty problem in interrupts during debug with BRK:
// Suppose 2 breakpoints are set with interrupts disabled (such as within an
// interrupt handler).  When "GO" is typed to continue from the first,
// there will most often be another interrupt pending due to the time
// spent in the monitor.  While this interrupt should be held off because
// interrupts are disabled, we find that execution of the second BRK fetches
// the VECTOR FOR THE PENDING INTERRUPT, rather than the BREAK vector.
//
// To allow breakpoints in interrupt handlers, we have installed code
// to check the break bit for ALL interrupts.  If clear, control will be
// passed normally through the RAM vector.  If set, we will enter the
// breakpoint service routine.
//
// According to Rockwell, this will also occur on the 6502, but not on
// the 65C02.
//
//
// ON THE MELPS740 (from which this was ported...)
// It seems that "B" gets set on the stack but NOT in the PS in the
// handler.  Manual claims both get set...
// Not sure how this works on a 6502 or a 65C02.
// If your breakpoints or single-step don't work, then change this code
// to look at PS rather than stack.
//
// IRQ/BREAK.  VECTOR THROUGH RAM
// First, check for BRK (breakpoint) interrupt
_IRQ:   cld
        sta     REG_A           //SAVE A

// Test B bit in STACKED condition codes
        pla
        pha                     //GET COPY OF PRE-INT STATUS REG

// Test B bit in CURRENT condition codes
////      PHP
////      pla                     //GET COPY OF CURRENT STATUS REG

        and     #B
        bne     GOBREAKP        //SET: DO BREAKPOINT CODE
        lda     REG_A           //ELSE RESTORE ACC.
        jmp     (RAMVEC+4)      //JUMP THROUGH RAM VECTOR

// IRQ NOT USED (ADDRESS plaCED IN RAMVEC+04H BY INIT)
_IRQX:  lda     #2              //TARGET STOP TYPE
        jmp     GOBREAK         //COMplaIN, ENTER MONITOR

// NMI.  VECTOR THROUGH RAM
// First, check for BRK (breakpoint) interrupt
_NMI:   cld
        sta     REG_A           //SAVE A

// Test B bit in STACKED condition codes
        pla
        pha                     //GET COPY OF PRE-INT STATUS REG

// Test B bit in CURRENT condition codes
////      PHP
////      pla                     //GET COPY OF CURRENT STATUS REG

        and     #B
        bne     GOBREAKP        //SET: DO BREAKPOINT CODE
        lda     REG_A           //ELSE RESTORE ACC.
        jmp     (RAMVEC+0)      //JUMP THROUGH RAM VECTOR

// NMI NOT USED (ADDRESS plaCED IN RAMVEC+00H BY INIT)
_NMIX:  lda     #3              //TARGET STOP TYPE
        jmp     GOBREAK         //ALWAYS DO BREAKPOINT CODE

// BREAK ENTRY.
GOBREAKP:
        lda     #1              //STATE IS "BREAKPOINT"
GOBREAK:
        jmp     INT_ENTRY       //ENTER MONITOR

// INTERRUPT VECTORS
        * = HARD_VECT
        .word     _NMI            //FFFA NMI
        .word     RESET           //FFFC RESET
        .word     _IRQ            //FFFE IRQ, BRK

