
// ***** Declaracion de funciones **** //
void LCD_Ini(void);													//inicializa display 8bits
void LCD_Display_Off(void);											//apagar display
void LCD_Display_On(void);											//prender display
void LCD_Clear(void);												//limpiar display
void LCD_Home(void);												//poner cursor en posicion org
void LCD_Cursor(char ren, char col);								//poner cursor en renglon,columna
void LCD_Cursor_On(void);											//mostrar cursor
void LCD_Cursor_Off(void);											//ocultar cursor
void LCD_Blink_On(void);											//prender blink
void LCD_Shift_RALL(void);											//recorrer todo display derecha
void LCD_Shift_LALL(void);											//recorrer todo display izquierda
void LCD_Shift_R(void);												//recorrer uno derecha
void LCD_Shift_L(void); 											//recorrer uno izquierda
void LCD_Caracter(char Data);										//mandar caracter
void LCD_Cadena_F(char ren, char col, const unsigned char *cadena);	//mandar cadena FLASH
void LCD_Cadena(char ren, char col, char *cadena);					//mandar cadena RAM
unsigned char LCD_Busy ( void );				  					//retorna el valor "busy flag"
void LCD_WriteControl (unsigned char CMD); 	 						//manda una instruccion de control


// ***** Declaracion de macros de control ***** //
#define PORT_CTRL 			GPIOE->BSRR
#define LCD_OP_PORT			GPIOE->ODR
//														 			 RS RW E - Datos
//Pines de control								    //15 14 13 12-11 10 9  8 - 7654-3210 bit
#define SET_LCD_E			PORT_CTRL = 0x00000100  //0  0  0  0 -0  0  0  1 - 0000-0000
#define CLEAR_LCD_E			PORT_CTRL = 0x01000000  //                     0
#define SET_LCD_DATA		PORT_CTRL = 0x00000400  //0  0  0  0 -0  1  0  0 - 0000-0000
#define SET_LCD_CMD			PORT_CTRL = 0x04000000  //               0
#define SET_LCD_READ		PORT_CTRL = 0x00000200  //0  0  0  0 -0  0  1  0 - 0000-0000
#define SET_LCD_WRITE		PORT_CTRL = 0x02000000  //                  0



// ***** Declaracion de constantes ***** //
#define LCD_FUNCTION_SET	0x3C
#define LCD_OFF				0x08
#define LCD_CLEAR			0x01
// LCD_MODE_SET 0x07 para corrimientos y 0x06 normal
#define LCD_MODE_SET		0x06
#define LCD_ON				0x0C
#define LCD_HOME			0x02
#define LCD_CURS_ON			0x0E
#define LCD_BLINK_ON		0x0D
#define LCD_SHITF_RA		0x18
#define LCD_SHITF_LA		0x1C
#define LCD_SHITF_R			0x14
#define LCD_SHITF_L			0x10


/********************************* FUNCIONES DEL DISPLAY *******************************/
void LCD_Ini(void)				   			 //inicializar display en 8 bits
{
	uint32_t tmpreg=0x00000000;
	//activar periferico (reloj)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	// Delay after an RCC peripheral clock enabling
	tmpreg = RCC->AHB1ENR & RCC_AHB1ENR_GPIOEEN;

	GPIOE->MODER   &= ~ 0x003FFFFF; 	    //apagar bits 10-0     			0b0000-0000-0011-1111-1111-1111-1111-1111
	GPIOE->MODER   |=   0x00155555; 		//Output,						0b0000-0000-0001-0101-0101-0101-0101-0101
	GPIOE->OTYPER  &= ~ 0x000007FF; 	    //Push pull, apagar bits 10-0   0b0000-0000-0000-0000-0000-0111-1111-1111
	GPIOE->PUPDR   &= ~ 0x000007FF; 		//No Pull, apagar               0b0000-0000-0000-0000-0000-0111-1111-1111
	GPIOE->OSPEEDR &= ~ 0x003FFFFF; 	    //apagar bits 10-0              0b0000-0000-0011-1111-1111-1111-1111-1111

	LCD_WriteControl (LCD_FUNCTION_SET);		 //
	LCD_WriteControl (LCD_FUNCTION_SET);		 //
	LCD_WriteControl (LCD_FUNCTION_SET);		 //
	LCD_WriteControl (LCD_OFF);				 //
	LCD_WriteControl (LCD_CLEAR);				 //
	LCD_WriteControl (LCD_MODE_SET);			 //
	LCD_WriteControl (LCD_ON);					 //
	LCD_WriteControl (LCD_HOME);				 //

	tmpreg=tmpreg; 								//para que no genere warning por falta de uso
}

unsigned char LCD_Busy ( void )				 //retorna el valor "busy flag"
{
 unsigned int i;
 for(i=100000; i>0; )
 {
  i--;
 }
 return 0x00;								 // retornar busy flag
}

void LCD_WriteControl (unsigned char CMD) 	 //manda una instruccion de control
{
 while (LCD_Busy()& 0X80);			 		 // checar bandera busy
 CLEAR_LCD_E;	   							 // apagar E
 SET_LCD_WRITE; 							 // poner RW en 0
 SET_LCD_CMD;								 // poner RS en 0

 LCD_OP_PORT &= ~ 0x000000FF; 	             //apagar bits 7-0     			0b0000-0000-0000-0000-0000-0000-1111-1111
 LCD_OP_PORT |=   (uint32_t)CMD;             // mandar comando

 SET_LCD_E;	   								 // prender E
 SET_LCD_E;	   								 // prender E
 CLEAR_LCD_E;	   							 // apagar E
}

void LCD_Caracter (char Data)		 		 //manda un caracter al display
{
 while (LCD_Busy() & 0X80); 				 // checar bandera busy
 CLEAR_LCD_E;	   							 // apagar E
 SET_LCD_WRITE ;							 // poner RW en 0
 SET_LCD_DATA;								 // poner RS en 1

 LCD_OP_PORT &= ~ 0x000000FF; 	             //apagar bits 7-0     			0b0000-0000-0000-0000-0000-0000-1111-1111
 LCD_OP_PORT |=   (uint32_t)Data;            // mandar caracter

 SET_LCD_E;	   								 // prender E
 SET_LCD_E;	   								 // prender E
 CLEAR_LCD_E;	   							 // apagar E
}

void LCD_Clear(void)						 //limpiar display y dejar cursor inicial
{
 LCD_WriteControl(LCD_CLEAR);
}

void LCD_Home(void)							 //mandar cursor a posicion original
{
 LCD_WriteControl(LCD_HOME);
}

void LCD_Cursor_On (void)					 //poner cursor visible en el display
{
 LCD_WriteControl (LCD_CURS_ON);
}

void LCD_Cursor_Off (void)					 //quitar cursor del display
{
 LCD_WriteControl (LCD_ON);
}

void LCD_Display_Off (void)					 //apagar display
{
 LCD_WriteControl(LCD_OFF);
}

void LCD_Display_On (void)					 //prender display
{
 LCD_WriteControl(LCD_ON);
}

void LCD_Blink_On(void)
{
 LCD_WriteControl(LCD_BLINK_ON);
}

void LCD_Shift_RALL(void)
{
 LCD_WriteControl(LCD_SHITF_RA);
}

void LCD_Shift_LALL(void)
{
 LCD_WriteControl(LCD_SHITF_LA);
}

void LCD_Shift_R(void)
{
 LCD_WriteControl(LCD_SHITF_R);
}

void LCD_Shift_L(void)
{
 LCD_WriteControl(LCD_SHITF_L);
}

void LCD_Cursor (char ren, char col)		 //posiciona el cursor en renglon,columna
{
 switch(ren)
 {
  case 1: LCD_WriteControl (0x80 + col - 1); break;
  case 2: LCD_WriteControl (0xc0 + col - 1); break;
  default: break;
 }
}

void LCD_Cadena_F (char ren, char col ,const unsigned char *cadena)
{
 LCD_Cursor (ren, col);
 while (*cadena)
 LCD_Caracter (*cadena++);
}

void LCD_Cadena (char ren, char col ,char *cadena)
{
 LCD_Cursor (ren, col);
 while (*cadena)
 LCD_Caracter (*cadena++);
}
