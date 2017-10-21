
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <sys/mman.h>

#define LED 6
#define LED2 5
#define BUTTON 19
#define POSITIONS 3
#define COLOURS 3
#define MAX_ATTEMPTS 10

#define STRB_PIN 24
#define RS_PIN 25
#define DATA0_PIN 23
#define DATA1_PIN 17
#define DATA2_PIN 27
#define DATA3_PIN 22

// char data for the CGRAM, i.e. defining new characters for the display
static unsigned char newChar [8] = 
{
  0b11111,
  0b10001,
  0b10001,
  0b10101,
  0b11111,
  0b10001,
  0b10001,
  0b11111,
} ;

/* bit pattern to feed into lcdCharDef to define a new character */
static unsigned char hawoNewChar [8] = 
{
  0b11111,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
} ;

// data structure holding data on the representation of the LCD
struct lcdDataStruct
{
  int bits, rows, cols ;
  int rsPin, strbPin ;
  int dataPins [8] ;
  int cx, cy ;
} ;

static int lcdControl ;

/* ***************************************************************************** */
/* INLINED fcts from wiringPi/devLib/lcd.c: */
// HD44780U Commands (see Fig 11, p28 of the Hitachi HD44780U datasheet)

#define	LCD_CLEAR	0x01
#define	LCD_HOME	0x02
#define	LCD_ENTRY	0x04
#define	LCD_CTRL	0x08
#define	LCD_CDSHIFT	0x10
#define	LCD_FUNC	0x20
#define	LCD_CGRAM	0x40
#define	LCD_DGRAM	0x80

// Bits in the entry register
#define	LCD_ENTRY_SH		0x01
#define	LCD_ENTRY_ID		0x02

// Bits in the control register
#define	LCD_BLINK_CTRL		0x01
#define	LCD_CURSOR_CTRL		0x02
#define	LCD_DISPLAY_CTRL	0x04

// Bits in the function register
#define	LCD_FUNC_F	0x04
#define	LCD_FUNC_N	0x08
#define	LCD_FUNC_DL	0x10

#define	LCD_CDSHIFT_RL	0x04

// =======================================================

#ifndef	TRUE
#define	TRUE	(1==1)
#define	FALSE	(1==2)
#endif

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

#define	INPUT	0
#define	OUTPUT	1

static volatile unsigned int gpiobase ;
static uint32_t *gpio ;


// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices
#define	PI_GPIO_MASK	(0xFFFFFFC0)

// protos
int failure (int fatal, const char *message, ...);
void waitForEnter (void);

/* inlined ARM function to set the pinMode to input or output.
 * this function takes the pin number and the INPUT/OUUTPUT(0 or 1) 
 * value as parameters, calculates the function select and the shifting
 * and it writes in the corect place in memory 0 or 1 value. */
void setPinMode_ASM(int piNNum, int inOut)
{
	int value1, fselect, shifting;
	value1 = piNNum;
	shifting = (value1 % 10) *3;
	value1 = value1 / 10;
	fselect = value1 % 10;
	
	if(inOut == 1){
		asm("_start_fpMod: NOP\n"  //just a label for debugging
			"\tMOV R1, %[sel]\n"	
			"\tMOV R6, #4\n"   //the offset has to be multiplied by 4
			"\tMUL R1, R6\n"
			"\tLDR R5, [%[gpi], R1]\n" //load the value into register
			"\tMOV R2, #7\n"   // 7 is 111 in assembly
			"\tMOV R3, #1\n"		
			"\tLSL R2, %[shf]\n"  //shifting
			"\tBIC R5, R2\n"  //the BIC instruction is the NAND operation
			"\tLSL R3, %[shf]\n"  //shifting
			"\tORR R5, R3\n"
			"\tSTR R5, [%[gpi], R1]\n"  // store the value in the address (gpio + fselect)
			: // no registers need to be modified
			: [sel] "r" (fselect), [gpi] "r" (gpio), [shf] "r" (shifting)
			: "r1", "r2", "r3", "r5", "r6", "cc"); 
	}	
	else{
		asm("_start_pMIn: NOP\n" 
			"\tMOV R1, %[sel]\n"
			"\tMOV R6, #4\n"  
			"\tMUL R1, R6\n"
			"\tLDR R5, [%[gpi], R1]\n" 
			"\tMOV R2, #7\n" 
			"\tLSL R2, %[shf]\n" 
			"\tBIC R5, R2\n" 
			"\tSTR R5, [%[gpi], R1]\n" 
		: // no registers need to be modified
		: [sel] "r" (fselect), [gpi] "r" (gpio), [shf] "r" (shifting)
		: "r1", "r2", "r6", "r5", "cc"); 
	}	
}

/* ARM inlined function to set the pin ON or OFF.
 * it takes the pinNumber and the value(0 or 1) as parameters.
 * if the value is 0 we choose the clear register, else we
 * choose  the set register, we then set correct bit
 * corresponding to the correct pin to 1.*/
void sendBitSetClear_ASM(int pinN, int value){

int offse;

if(value ==0){
	if(pinN < 32)
		offse = 10; //clear register number 1
	else
		offse = 11; //clear register number 2
	}
else{
	if(pinN < 32)
		offse = 7; //set register number 1	
	else
		offse = 8; //set register number 2	
	}
	
asm("_start_dWrite: NOP\n" 
		"\tMOV R1, %[sOfft]\n"
		"\tMOV R5, #4\n"  
		"\tMUL R1, R5\n"
		"\tMOV R3, #1\n"
		"\tMOV R2, %[pinNum]\n"
		"\tAND R2, #31\n"		
		"\tLSL R3, R2\n"		
		"\tSTR R3, [%[gp], R1]\n"
		: // no registers need to be modified
		: [pinNum] "r" (pinN), [gp] "r" (gpio), [sOfft] "r" (offse)
		: "r1", "r2", "r3", "cc"); 
}


/* ARM inlined function to read the value of the corresponding
 * pin(in our case the button). the value will be 1 if the button
 * is pressed.*/
int getButtonValue_ASM(int butt){
	
int ret = 0;

asm("_start_bValue: NOP\n"
		"\tLDR R3, [%[gp], #52]\n" // 52 is 13*4
		"\tMOV R1, #1\n"
		"\tMOV R2, %[butt1]\n"
		"\tAND R2, #31\n"		
		"\tLSL R1, R2\n"		
		"\tAND R3, R1\n"
		"\tMOV %[ret1], R3\n" // store the result in ret
		: [ret1] "=r" (ret)
		: [gp] "r" (gpio), [butt1] "r" (butt)
		: "r1", "r2", "r3", "cc"); 

return ret;

}	

/* timer function to insert the delay. */
void delay(unsigned int t){
	
	struct timespec sleeper, dummy ;
            
            sleeper.tv_sec  = (time_t)(t / 1000) ;
            sleeper.tv_nsec = (long)(t % 1000) * 1000000 ;
            
            nanosleep (&sleeper, &dummy) ;
}

/* timer function to insert the delay in microseconds */ 
void delayMicroseconds (unsigned int howLong)
{
  struct timespec sleeper ;
  unsigned int uSecs = howLong % 1000000 ;
  unsigned int wSecs = howLong / 1000000 ;

if (howLong ==   0)
    return ;
#if 0
  else if (howLong  < 100)
    delayMicrosecondsHard (howLong) ;
#endif
  else
  {
    sleeper.tv_sec  = wSecs ;
    sleeper.tv_nsec = (long)(uSecs * 1000L) ;
    nanosleep (&sleeper, NULL) ;
  }
}

/* ------------------------------------------------------- */

void strobe (const struct lcdDataStruct *lcd)
{
  sendBitSetClear_ASM( lcd->strbPin, 1) ; delayMicroseconds (50) ;
  sendBitSetClear_ASM( lcd->strbPin, 0) ; delayMicroseconds (50) ;
}

/*
 * sentDataCmd:
 *	Send data or command byte to the display.
 *********************************************************************************
 */
void sendDataCmd (const struct lcdDataStruct *lcd, unsigned char data)
{
  register unsigned char myData = data ;
  unsigned char          i, d4 ;

  if (lcd->bits == 4)
  {
    d4 = (myData >> 4) & 0x0F;
    for (i = 0 ; i < 4 ; ++i)
    {
      sendBitSetClear_ASM( lcd->dataPins [i], (d4 & 1)) ;
      d4 >>= 1 ;
    }
    strobe (lcd) ;

    d4 = myData & 0x0F ;
    for (i = 0 ; i < 4 ; ++i)
    {
      sendBitSetClear_ASM( lcd->dataPins [i], (d4 & 1)) ;
      d4 >>= 1 ;
    }
  }
  else
  {
    for (i = 0 ; i < 8 ; ++i)
    {
      sendBitSetClear_ASM( lcd->dataPins [i], (myData & 1)) ;
      myData >>= 1 ;
    }
  }
  strobe (lcd) ;
}

/*
 * lcdPutCommand:
 *	Send a command byte to the display
 *********************************************************************************
 */
void lcdPutCommand (const struct lcdDataStruct *lcd, unsigned char command)
{
#ifdef DEBUG
  fprintf(stderr, "lcdPutCommand: digitalWrite(%d,%d) and sendDataCmd(%d,%d)\n", lcd->rsPin,   0, lcd, command);
#endif
  sendBitSetClear_ASM( lcd->rsPin,   0) ;
  sendDataCmd  (lcd, command) ;
  delay (2) ;
}
/* ------------------------------------------------------- */

void lcdPut4Command (const struct lcdDataStruct *lcd, unsigned char command)
{
  register unsigned char myCommand = command ;
  register unsigned char i ;

  sendBitSetClear_ASM( lcd->rsPin,   0) ;

  for (i = 0 ; i < 4 ; ++i)
  {
    sendBitSetClear_ASM( lcd->dataPins [i], (myCommand & 1)) ;
    myCommand >>= 1 ;
  }
  strobe (lcd) ;
}

/*
 * lcdHome: lcdClear:
 *	Home the cursor or clear the screen.
 *********************************************************************************
 */

void lcdHome (struct lcdDataStruct *lcd)
{
  lcdPutCommand (lcd, LCD_HOME) ;
  lcd->cx = lcd->cy = 0 ;
  delay (5) ;
}
/* ------------------------------------------------------- */

void lcdClear (struct lcdDataStruct *lcd)
{
#ifdef DEBUG
  fprintf(stderr, "lcdClear: lcdPutCommand(%d,%d) and lcdPutCommand(%d,%d)\n", lcd, LCD_CLEAR, lcd, LCD_HOME);
#endif
  lcdPutCommand (lcd, LCD_CLEAR) ;
  lcdPutCommand (lcd, LCD_HOME) ;
  lcd->cx = lcd->cy = 0 ;
  delay (5) ;
}

/*
 * lcdPosition:
 *	Update the position of the cursor on the display.
 *	Ignore invalid locations.
 *********************************************************************************
 */
void lcdPosition (struct lcdDataStruct *lcd, int x, int y)
{
  // struct lcdDataStruct *lcd = lcds [fd] ;

  if ((x > lcd->cols) || (x < 0))
    return ;
  if ((y > lcd->rows) || (y < 0))
    return ;

  lcdPutCommand (lcd, x + (LCD_DGRAM | (y>0 ? 0x40 : 0x00)  /* rowOff [y] */  )) ;

  lcd->cx = x ;
  lcd->cy = y ;
}


/*
 * lcdDisplay: lcdCursor: lcdCursorBlink:
 *	Turn the display, cursor, cursor blinking on/off
 *********************************************************************************
 */
void lcdDisplay (struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |=  LCD_DISPLAY_CTRL ;
  else
    lcdControl &= ~LCD_DISPLAY_CTRL ;

  lcdPutCommand (lcd, LCD_CTRL | lcdControl) ; 
}
/* ------------------------------------------------------- */

void lcdCursor (struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |=  LCD_CURSOR_CTRL ;
  else
    lcdControl &= ~LCD_CURSOR_CTRL ;

  lcdPutCommand (lcd, LCD_CTRL | lcdControl) ; 
}
/* ------------------------------------------------------- */

void lcdCursorBlink (struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |=  LCD_BLINK_CTRL ;
  else
    lcdControl &= ~LCD_BLINK_CTRL ;

  lcdPutCommand (lcd, LCD_CTRL | lcdControl) ; 
}

/*
 * lcdPutchar:
 *	Send a data byte to be displayed on the display. We implement a very
 *	simple terminal here - with line wrapping, but no scrolling. Yet.
 *********************************************************************************
 */
void lcdPutchar (struct lcdDataStruct *lcd, unsigned char data)
{
  sendBitSetClear_ASM( lcd->rsPin, OUTPUT) ;
  sendDataCmd  (lcd, data) ;

  if (++lcd->cx == lcd->cols)
  {
    lcd->cx = 0 ;
    if (++lcd->cy == lcd->rows)
      lcd->cy = 0 ;
    
    // TODO: inline computation of address and eliminate rowOff
    lcdPutCommand (lcd, lcd->cx + (LCD_DGRAM | (lcd->cy>0 ? 0x40 : 0x00)   /* rowOff [lcd->cy] */  )) ;
  }
}


/*
 * lcdPuts:
 *	Send a string to be displayed on the display
 *********************************************************************************
 */
void lcdPuts (struct lcdDataStruct *lcd, const char *string)
{
  while (*string)
    lcdPutchar (lcd, *string++) ;
}

/* function to make an LED blink. it uses the ARM inlined function
 * defined earlier. */
void blink(int n, int sdelay, int led){
	int i;
	for(i=0; i<n; i++){
        sendBitSetClear_ASM(led, 1);
        delay(sdelay);
        sendBitSetClear_ASM(led, 0);
        delay(sdelay);
	}
}

/* ------------------------------------------------------- */

int failure (int fatal, const char *message, ...)
{
    va_list argp ;
    char buffer [1024] ;
    
    if (!fatal) //  && wiringPiReturnCodes)
        return -1 ;
    
    va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
    va_end (argp) ;
    
    fprintf (stderr, "%s", buffer) ;
    exit (EXIT_FAILURE) ;
    
    return 0 ;
}

/* check wether an int(val) is into an array(arr) of size(size) */
int isvalueinarray(int val, int *arr, int size){
	int i;
	for(i=0; i< size; i++){
		if(arr[i] == val)
			return 1;
	}
	return 0;
}

/* function to check how many correct and partially correct answers
 * have been given. */
int *findRight(int pos, int *col, int *sec){
	int l, t, z;
	int right = 0;
    int wrong = 0; 
    int temp[5]= {-1,-1,-1,-1,-1};
	int tempindex =0;   
	
	for(l=0; l<pos; l++){
		if(col[l] == sec[l]){
			right ++;
		}
	}
	
	for(t = 0; t < pos; t++){
		for(z = 0; z < pos; z++){
			if(sec[t] == col[z] && isvalueinarray(z, temp, 5) == 0){
				temp[tempindex]= z;
				tempindex++;
				wrong ++;
				break;
			}
		}
	}
	wrong = wrong-right;
	
	int ret[2] = {right, wrong};
	int *returner = ret;
	
	return returner;
}

/* Main ----------------------------------------------------------------------------- */

int main (int argc, char ** argv)
{
	int debugMode;
	char *dbm = "-d";
    int fd, rando;
    int i, j, k;
    int count, keep, attempts;
    int flag;
    int right, wrong;
    int secret[3];
    int colours[3];
	struct lcdDataStruct *lcd ;
	int bits, rows, cols ;
	unsigned char func ;
	struct tm *t ;
	time_t tim ;

	char buf [32] ;

	// hard-coded: 16x2 display, using a 4-bit connection
	bits = 4; 
	cols = 16; 
	rows = 2; 
    
    if (geteuid () != 0)
        fprintf (stderr, "setup: Must be root. (Did you forget sudo?)\n") ;
    
    // constant for RPi2
    gpiobase = 0x3F200000 ;
    
    // -----------------------------------------------------------------------------
    // memory mapping
    // Open the master /dev/memory device
    
    if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
        return failure (FALSE, "setup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
    
    // GPIO:
    gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, gpiobase) ;
    if ((int32_t)gpio == -1)
        return failure (FALSE, "setup: mmap (GPIO) failed: %s\n", strerror (errno)) ;
    
    // -----------------------------------------------------------------------------
    // setting the mode
    
    // controlling LED pin 6   
    setPinMode_ASM(LED, OUTPUT); //set the mode (asm)
    
    // controlling LED pin 5 
    setPinMode_ASM(LED2, OUTPUT); //set the mode (asm)
    
    // controlling button pin 19
    setPinMode_ASM(BUTTON, INPUT); //set the mode (asm)
    
    // -----------------------------------------------------------------------------
    
    lcd = (struct lcdDataStruct *)malloc (sizeof (struct lcdDataStruct));
    
	if (lcd == NULL)
		return -1 ;

	// hard-wired GPIO pins
	lcd->rsPin   = RS_PIN ;
	lcd->strbPin = STRB_PIN ;
	lcd->bits    = 4 ;
	lcd->rows    = rows ;  // # of rows on the display
	lcd->cols    = cols ;  // # of cols on the display
	lcd->cx      = 0 ;     // x-pos of cursor
	lcd->cy      = 0 ;     // y-pos of curosr

	lcd->dataPins [0] = DATA0_PIN ;
	lcd->dataPins [1] = DATA1_PIN ;
	lcd->dataPins [2] = DATA2_PIN ;
	lcd->dataPins [3] = DATA3_PIN ;
  
	sendBitSetClear_ASM( lcd->rsPin,   0); setPinMode_ASM(lcd->rsPin,   OUTPUT) ;
	sendBitSetClear_ASM( lcd->strbPin, 0); setPinMode_ASM(lcd->strbPin, OUTPUT) ;

	for (i = 0 ; i < bits ; ++i)
	{
		sendBitSetClear_ASM( lcd->dataPins [i], 0) ;
		setPinMode_ASM(lcd->dataPins [i], OUTPUT) ;
	}
	delay (35) ; // mS
  
	if (bits == 4)
	{
		func = LCD_FUNC | LCD_FUNC_DL ;			// Set 8-bit mode 3 times
		lcdPut4Command (lcd, func >> 4) ; delay (35) ;
		lcdPut4Command (lcd, func >> 4) ; delay (35) ;
		lcdPut4Command (lcd, func >> 4) ; delay (35) ;
		func = LCD_FUNC ;					// 4th set: 4-bit mode
		lcdPut4Command (lcd, func >> 4) ; delay (35) ;
		lcd->bits = 4 ;
	}
	else
	{
		failure(TRUE, "setup: only 4-bit connection supported\n");
		func = LCD_FUNC | LCD_FUNC_DL ;
		lcdPutCommand  (lcd, func     ) ; delay (35) ;
		lcdPutCommand  (lcd, func     ) ; delay (35) ;
		lcdPutCommand  (lcd, func     ) ; delay (35) ;
	}

	if (lcd->rows > 1)
	{
		func |= LCD_FUNC_N ;
		lcdPutCommand (lcd, func) ; delay (35) ;
	}

	// Rest of the initialisation sequence
	lcdDisplay     (lcd, TRUE) ;
	lcdCursor      (lcd, FALSE) ;
	lcdCursorBlink (lcd, FALSE) ;
	lcdClear       (lcd) ;

	lcdPutCommand (lcd, LCD_ENTRY   | LCD_ENTRY_ID) ;    // set entry mode to increment address counter after write
	lcdPutCommand (lcd, LCD_CDSHIFT | LCD_CDSHIFT_RL) ;  // set display shift to right-to-left
  	
	//create the random sequence
    srand(time(NULL));
	for(j=0; j<POSITIONS; j++){
		rando = (rand() % POSITIONS)+1;
		secret[j] = rando;
	}
	
	//check if we are in debug mode
	if(argc == 2)
		if(strcmp(argv[1], dbm) == 0)
			debugMode = 1;
			
	fprintf(stderr,"Printing welcome message...\n");

	lcdPosition (lcd, 0, 0) ; lcdPuts (lcd, "  welcome to  ") ;
	lcdPosition (lcd, 0, 1) ; lcdPuts (lcd, "  mastermind  ") ;
	
	delay(3000);
	lcdClear(lcd) ;
	
	fprintf(stderr,">>>GAME STARTED<<<\n");
	
	if(debugMode)
		printf("SECRET: %d %d %d\n", secret[0], secret[1], secret[2]);
    
    attempts = 1;
    keep = 0;
    flag = 0;
    while(attempts <= MAX_ATTEMPTS){ //max attempts
		
	if(debugMode)			
		printf("ATTEMPT NUMBER : %d\n", attempts);
	
	k = 0;	
	
    while(k < POSITIONS){
		count = 0 ;
    while (1)
    {
        if (getButtonValue_ASM(BUTTON)){ //GPLEV0
			while(getButtonValue_ASM(BUTTON)){ //GPLEV0
				}			
				if(count < COLOURS){
					count++;
					flag = 1; // button has been pressed at least once
					keep = 0; // restart the couter
				}
			}
        delay(10);
        keep ++;
        if(flag == 1 && keep > 400){ 
			flag = 0;
			keep = 0;
			break; 
		}
    }
	
	if(debugMode)
		printf("entered : %d\n", count);

	blink(1, 800, LED2);
    blink(count, 500, LED);
	
	colours[k] = count;
	k++;
	
	}
	
	blink(2, 500, LED2);
	delay(500);
 
	//check how many right answers
    int *tmp = findRight(POSITIONS, colours, secret);   
        
    right = tmp[0];
    wrong = tmp[1];    
        
    if(debugMode){
		printf("------------\n");
		printf("RIGHT : %d\n",right);
		printf("PARTIAL : %d\n",wrong);
		printf("------------\n");
	}		
    
    //write how many right answers on the display 
	char str[16];
	strcpy(str,"RIGHT: ");
	char rightS =(char) (right + (int)'0');
	char *a = (char*) malloc(2*sizeof(char));
	a[0] = rightS;
	a[1] = '\0';
	strcat(str,a);
	
	//write how many partially right answers on the display
	char str2[16];
	strcpy(str2,"PARTIAL: ");
	char wrongS =(char) (wrong + (int)'0');
	char *b = (char*) malloc(2*sizeof(char));
	b[0] = wrongS;
	b[1] = '\0';
	strcat(str2,b);
	lcdPosition (lcd, 0, 0) ; lcdPuts (lcd, str) ;
	lcdPosition (lcd, 0, 1) ; lcdPuts (lcd, str2) ;
	
	if(right == 3){ //if the code has been guessed right
		lcdClear(lcd) ;		
		char str3[16];
		strcpy(str3," attempts: ");
		char attS =(char) (attempts + (int)'0');
		char *ch = (char*) malloc(2*sizeof(char));
		ch[0] = attS;
		ch[1] = '\0';
		strcat(str3,ch);	
		lcdPosition (lcd, 0, 0) ; lcdPuts (lcd, " SUCCESS!") ;
		lcdPosition (lcd, 0, 1) ; lcdPuts (lcd, str3) ;
		sendBitSetClear_ASM(LED2, 1);
		blink(3, 500, LED);
		sendBitSetClear_ASM(LED2, 0);
		delay(5000);
		lcdClear(lcd) ;		
		exit(0);
	}
     
    blink(right, 500, LED);   
	delay(500);
	blink(1, 500, LED2);
	delay(500);
	blink(wrong, 500, LED);
	delay(500);
	blink(3, 500, LED2);	
	
	attempts ++;
}
    //if the code has not been guessed within the max number of attempts
    sendBitSetClear_ASM(LED, 1);
    lcdClear(lcd) ;
    blink(5, 1000, LED2);
    sendBitSetClear_ASM(LED, 0);
    lcdPosition (lcd, 0, 0); lcdPuts (lcd, "FAILURE") ;
    lcdClear(lcd) ;
}
