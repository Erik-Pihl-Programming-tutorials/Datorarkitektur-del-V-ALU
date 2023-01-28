/********************************************************************************
* program_memory.c: Contains function definitions and macro definitions for
*                   implementation of a 0.75 kB program memory, capable of 
*                   storing 256 24-bit instructions. Since C doesn't support
*                   unsigned 24-bit integers (without using structs or unions),
*                   the program memory is set to 32 bits data width, but only
*                   24 bits are used.
********************************************************************************/
#include "program_memory.h"

/* Macro definitions: */
#define main           1  /* Start address for subroutine main. */
#define main_loop      2  /* Start address for loop in subroutine main. */
#define led1_off       5 /* Start address for subroutine led1_off. */
#define led1_on        9 /* Start address for subroutine led1_on. */
#define setup          13 /* Start address for subroutine setup. */
#define end            18 /* End address for current program. */

#define LED1 PORTB0    /* LED 1 connected to pin 8 (PORTB0). */
#define BUTTON1 PORTB5 /* Button 1 connected to pin 13 (PORTB5). */

/* Static functions: */
static uint32_t assemble(const uint8_t op_code,
                         const uint8_t op1,
                         const uint8_t op2);

/********************************************************************************
* data: Program memory with capacity for storing 256 instructions at address
*       0 - 255. 
********************************************************************************/
static uint32_t data[PROGRAM_MEMORY_ADDRESS_WIDTH];

/********************************************************************************
* program_memory_write: Writes machine code to the program memory by converting
*                       from assembly code via an assembler. This function
*                       should only be called once when the program starts.
********************************************************************************/
void program_memory_write(void)
{
   static bool program_memory_initialized = false;
   if (program_memory_initialized) return;

   /********************************************************************************
   * RESET_vect: Reset vector and start address for the program. A jump is made
   *             to the main subroutine in order to start the program.
   ********************************************************************************/
   data[0]  = assemble(JMP, main, 0x00);
   
   /********************************************************************************
   * main: Initiates the I / O ports at start.The program is running as long as
   *       voltage is supplied.BUTTON1 is continuously polled.When BUTTON1
   *       is pressed, LED1 is enabled, otherwise it's disabled.
   *********************************************************************************/
   data[1]  = assemble(CALL, setup, 0x00);
   data[2]  = assemble(IN, R16, PINB);
   data[3]  = assemble(ANDI, R16, (1 << BUTTON1));
   data[4]  = assemble(BRNE, led1_on, 0x00);

   /********************************************************************************
   * led1_off: Disables LED1. 
   *********************************************************************************/
   data[5]  = assemble(IN, R16, PORTB);
   data[6]  = assemble(ANDI, R16, ~(1 << LED1));
   data[7]  = assemble(OUT, PORTB, R16);
   data[8]  = assemble(JMP, main_loop, 0x00);

   /********************************************************************************
   * led1_on: Enables LED1.
   *********************************************************************************/
   data[9]  = assemble(IN, R16, PORTB);
   data[10] = assemble(ORI, R16, (1 << LED1));
   data[11] = assemble(OUT, PORTB, R16);
   data[12] = assemble(JMP, main_loop, 0x00);

   /********************************************************************************
   * setup: Sets the pin connected to LED1 to output and enables the internal 
   *        pull-up resistor for the pin connected to BUTTON1 so that the input
   *        signal always reads as high (1) or low (0).
   *********************************************************************************/
   data[13] = assemble(LDI, R16, (1 << LED1));
   data[14] = assemble(OUT, DDRB, R16);
   data[15] = assemble(LDI, R16, (1 << BUTTON1));
   data[16] = assemble(OUT, PORTB, R16);
   data[17] = assemble(RET, 0x00, 0x00);

   program_memory_initialized = true; 
   return;
}

/********************************************************************************
* program_memory_read: Returns the instruction at specified address. If an
*                      invalid address is specified (should be impossible as
*                      long as the program memory address width isn't increased)
*                      no operation (0x00) is returned.
*
*                      - address: Address to instruction in program memory.
********************************************************************************/
uint32_t program_memory_read(const uint8_t address)
{
   if (address < PROGRAM_MEMORY_ADDRESS_WIDTH)
   {
      return data[address];
   }
   else
   {
      return 0x00;
   }
}

/********************************************************************************
* program_memory_subroutine_name: Returns the name of the subroutine at
*                                 specified address.
*
*                                 - address: Address within the subroutine.
********************************************************************************/
const char* program_memory_subroutine_name(const uint8_t address)
{
   if (address >= RESET_vect && address < main)       return "RESET_vect";
   else if (address >= main && address < led1_off)    return "main";
   else if (address >= led1_off && address < led1_on) return "led1_off";
   else if (address >= led1_on && address < setup)    return "led1_on";
   else if (address >= setup && address < end)        return "setup";
   else                                               return "Unknown";
}

/********************************************************************************
* assemble: Returns instruction assembled from specified OP code and operands.
*           In the instruction, the OP code is placed at bit 23 down to 16,
*           operand 1 is placed at bit 15 down to 8 and operand 2 is placed
*           at bit 7 down to 0.
* 
*           - op_code: OP code of the instruction (operation to perform).
*           - op1    : First operand (usually a destination).
*           - op2    : Second operand (usually a constant or a read location).
********************************************************************************/
static uint32_t assemble(const uint8_t op_code,
                         const uint8_t op1,
                         const uint8_t op2)
{
   const uint32_t instruction = (op_code << 16) | (op1 << 8) | op2;
   return instruction;
}