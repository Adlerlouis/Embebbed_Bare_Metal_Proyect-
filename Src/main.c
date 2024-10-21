#include <stm32f4xx.h>
#include "LCD_Lib.c"

//********DECLARACION DE FUNCIONES***********
void botones(void);
void pendulo(void);
void ConfiguraPerifericos(void);
void izquierda(void);
void derecha(void);
void stop(void);
void start(void);
void delay_X(uint32_t nTime);
void delay_ms(uint32_t nTime);
char concaA(int num);
//RELOJ
void seg(void);
void min(void);
void hhrs(void);
void display(void);
int SS=0;
int MM=0;
int HH=0;
#define PuertoLED   GPIOE->ODR
//STATIC VOID DELAY_US(uint32_t nTime)*****

//*******DECLARACION DE VARIABLES***************

uint8_t  pend=0x80;
uint8_t  cont=0;
uint8_t  del=1;
int    var1=0;
int    var2;
int main (void)

{
	        LCD_Ini();
			ConfiguraPerifericos();
			del=4;
            var1=0;
            var2;
		    LCD_Cadena(1,1,"ENNCENDIDO ");
		    LCD_Clear();
		    LCD_Cadena(2,3,":");
		    LCD_Cadena(2,6,":");


	while(1)
	{
 {
        display();
	   switch(var1)
	    {
       case 1: stop();break;
       case 2:start();break;

	   default:;
		}

	}

}
}










void ConfiguraPerifericos(void)
{
	uint32_t tmpreg =  0x00;


	//-----------------------PC0 Y PC3 como salidas del SERVO MOTOR COMO SALIDA y 2 LEDS INDICADORES DE ABIERTO O CERRADO
	//Activa periferico para (reloj)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	//DELAY AFTER AN RCC PERIFERAL CLOCK ENABLING
	tmpreg = RCC->AHB1ENR & RCC_AHB1ENR_GPIOCEN;                                   //MODO , PIN//
	GPIOC->MODER    |=    0x00000555; //  0000,0000,0000,0000,0000,0101,0101,0101  //01 C3//01 C2//01 C1//01 C0
	GPIOC->OTYPER   &=  ~ 0x0000002F; //  0000,0000,0000,0000,0000,0000,0011,1111  //1 C3//1 C2//1 C1//1 C0
	GPIOC->PUPDR    &=  ~ 0x00000FFF; //  0000,0000,0000,0000,0000,1111,1111,1111  //11 C3//11 C2//11 C1//11 C0
	GPIOC->OSPEEDR  |=    0x00000555; //  0000,0000,0000,0000,0000,0101,0101,0101  //01 C3//01 C2//01 C1//01 C0

                                     //       BR PARTE ALTA    //   BS PARTE BAJA
	GPIOC->BSRR      =    0x002F0000; //  0000,0000,0011,1111,0000,0000,0000,0000 //11 C3//11 C2 //11 C1 //11 C0






	//-----------------------PUERTOS DE INTERRUPCION PB0,PB4,PB8,PB12 DEBIDO A LA INTERRUPCION (SWITCH) COMO ENTRADADA PARA RELOJ

   //ACTIVARV PERIFERICOS E RELOJ
  //ACTIVAMOS EL PUERTO A
	RCC ->AHB1ENR  |= RCC_AHB1ENR_GPIOBEN;
	//DELAY AFTER  RCC PERIFERIAL
	tmpreg =  RCC->AHB1ENR & RCC_AHB1ENR_GPIOBEN;

	GPIOB->MODER  &=  ~   0x03030303;//  0000,0011,0000,0011,0000,0011,0000,0011  //11 B12//11 B8//11 B4//11 B0

	GPIOB->PUPDR  |=      0x01010101;//  0000,0001,0000,0001,0000,0001,0000,0001  //01 B12//01 B8//11 B4//11 B0

	//-----------------------PUERTOS DE INTERRUPCION PD0 Y PD4 DEBIDO A LA INTERRUPCION (SWITCH) COMO ENTRADADA

	RCC ->AHB1ENR  |= RCC_AHB1ENR_GPIODEN;
		//DELAY AFTER  RCC PERIFERIAL
	tmpreg =  RCC->AHB1ENR & RCC_AHB1ENR_GPIODEN;


	GPIOD->MODER  &=  ~   0x00000303;//  0000,0000,0000,0000,0000,0011,0000,0011  11 D4//11 D0

	GPIOD->PUPDR  |=      0x00000101;//  0000,0001,0000,0000,0000,0001,0000,0001  //01 D4//01 D0



	///configuracion de las interrupciones externas pB0,PB4,PB8,PB12
	RCC ->APB2ENR  |= RCC_APB2ENR_SYSCFGEN;
		//DELAY AFTER  RCC PERIFERIAL
    tmpreg =  RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN;
    //SELECTOR DE LOS  SYSC
    SYSCFG->EXTICR[0] |=  0X0000001; //  0000,0000,0000,0000,0000,0000,0000,0001  //0001:  PARA GPIOB EN EXTI0 [3:0]EXTICR1
    SYSCFG->EXTICR[1] |=  0X0000001; //  0000,0000,0000,0000,0000,0000,0000,0001  //0001:  PARA GPIOB EN EXTI4 [3:0]EXTICR2
    SYSCFG->EXTICR[2] |=  0X0000001; //  0000,0000,0000,0000,0000,0000,0000,0001  //0001:  PARA GPIOB EN EXTI8 [3:0]EXTICR3
    SYSCFG->EXTICR[3] |=  0X0000001; //  0000,0000,0000,0000,0000,0000,0000,0001  //0001:  PARA GPIOB EN EXTI12[3:0]EXTICR4
    //SELECTOR DE LOS ENMASCARAMIENTOS

    EXTI->IMR        |=     0X00001111;//  0000,0000,0000,0000,0001,0001,0001,0001   //01 B12//01 B8//01 B4//01 B0 //1=INTERRRUPCION DE LAS MASCARA
    EXTI->EMR        &=  ~  0X00001111;//  0000,0000,0000,0000,0001,0001,0001,0001   //01 B12//01 B8//01 B4//01 B0 //EVENT NO MAKED
    EXTI->RTSR       &=  ~  0X00001111;//  0000,0000,0000,0000,0001,0001,0001,0001   //01 B12//01 B8//01 B4//01 B0 //EVENT NO MAKED
    EXTI->FTSR       |=     0X00001111;//  0000,0000,0000,0000,0001,0001,0001,0001   //01 B12//01 B8//01 B4//01 B0 //EVENT NO MAKED



    //CONFIGURACION DE  LAS INTERRUPCIONES EXTERNAS PD1 Y PD2


    //ACTIVAMOS LOS PUERTOS  PD1,PD2
           RCC ->AHB1ENR  |= RCC_AHB1ENR_GPIODEN;
           //DELAY AFTER  RCC PERIFERIAL
           tmpreg =  RCC->AHB1ENR & RCC_AHB1ENR_GPIODEN;
           GPIOD->MODER  &=  ~   0x0000003C;//  0000,0000,0000,0000,0000,0000,0011,1100 //11 B15//11 B12//11 B8//11 B4//11 B0
           GPIOD->PUPDR  |=      0x00000014;//  0000,0000,0000,0000,0000,0000,0001,0100  //01 B15//01 B12//01 B8//11 B4//11 B0

           ///configuracion de las interrupciones externas PB0,PB4,PB8,PB12,PB15
           RCC ->APB2ENR  |= RCC_APB2ENR_SYSCFGEN;
           //DELAY AFTER  RCC PERIFERIAL
           tmpreg =  RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN;
           //SELECTOR DE LOS  SYSC                //RESERVADO        // ULTILIZABLE
           SYSCFG->EXTICR[0] |=  0X00000030; //  0000,0000,0000,0000,0000,0000,0011,0000  //0001:  PARA GPIOB EN EXTI0 [3:0]EXTICR1
           SYSCFG->EXTICR[0] |=  0X00000300; //  0000,0000,0000,0000,0000,0000,0011,0000  //0001:  PARA GPIOB EN EXTI0 [3:0]EXTICR1

           //SELECTOR DE LOS ENMASCARAMIENTOS

                   EXTI->IMR        |=     0X00000006;//  0000,0000,0000,0000,0000,0000,0000,0110                  // 1 B4// 1 B0 //1=INTERRRUPCION DE LAS MASCARA
                   EXTI->EMR        &=  ~  0X00000006;//  0000,0000,0000,0000,0000,0000,0000,0110                  // 1 B8// 1 B4// 1 B0 //EVENT NO MAKED
                   EXTI->RTSR       &=  ~  0X00000006;//  0000,0000,0000,0000,0000,0000,0000,0110                  // 1 B8// 1 B4// 1 B0 //EVENT NO MAKED
                   EXTI->FTSR       |=     0X00000006;//  0000,0000,0000,0000,0000,0000,0000,0110                  // 1 B8// 1 B4// 1 B0 //EVENT NO MAKED


        //CONFIGURACION DEL TIMER PARA RELOJ


        ///-----------CONFIUGRACION DE LOS  TIMER
        	RCC ->APB1ENR  |= RCC_APB1ENR_TIM3EN;
        	//------DELAY AFTER PERHIPAL  CLOCK ENABLING
            tmpreg=RCC->APB1ENR & RCC_APB1ENR_TIM3EN;
            tmpreg=tmpreg/2;
            //SELECTOR DE LOS TIMERS
            TIM3->PSC  = 0X000003E7;//PRESCALADOR DE 9999  270f
            TIM3->ARR  = 0X00015f90; //PERIODO DE 8999 2337
            TIM3->DIER = 0X00000001;//ENABLE INTERRUPT



            //NVIC configuracion de los timers en este caso el timer 3
               tmpreg= NVIC_GetPriorityGrouping();//CONFIGURAMOS EL NVIC DEL EXTI8
                  NVIC_SetPriority(TIM3_IRQn,NVIC_EncodePriority(tmpreg,0,0));
                  NVIC_EnableIRQ(TIM3_IRQn);
                  TIM3->CR1 = 0X00000001; //ENABLE DEL TIM3

    //nvic configuracion.
    tmpreg= NVIC_GetPriorityGrouping();  //CONFIGURAMOS EL EL NVIC DEL EXTI0
       NVIC_SetPriority(EXTI0_IRQn,13);
       NVIC_EnableIRQ(EXTI0_IRQn);

       tmpreg= NVIC_GetPriorityGrouping();  //CONFIGURAMOS EL EL NVIC DEL EXTI1
               NVIC_SetPriority(EXTI1_IRQn,14);
               NVIC_EnableIRQ(EXTI1_IRQn);

          tmpreg= NVIC_GetPriorityGrouping();  //CONFIGURAMOS EL EL NVIC DEL EXTI2
               NVIC_SetPriority(EXTI2_IRQn,15);
               NVIC_EnableIRQ(EXTI2_IRQn);

    tmpreg= NVIC_GetPriorityGrouping(); //CONFIGURAMOS EL EL NVIC DEL EXTI4
       NVIC_SetPriority(EXTI4_IRQn,17);
       NVIC_EnableIRQ(EXTI4_IRQn);

    tmpreg= NVIC_GetPriorityGrouping();//CONFIGURAMOS EL NVIC DEL EXTI8
       NVIC_SetPriority(EXTI9_5_IRQn,NVIC_EncodePriority(tmpreg,0,0));
       NVIC_EnableIRQ(EXTI9_5_IRQn);

    tmpreg= NVIC_GetPriorityGrouping();//CONFIGURAMOS EL NVIC DEL EXTI12
       NVIC_SetPriority(EXTI15_10_IRQn,NVIC_EncodePriority(tmpreg,0,0));
       NVIC_EnableIRQ(EXTI15_10_IRQn);

    tmpreg=tmpreg;




}

char concaA(int num)
{
	return num + '0';
}


 void delay_X(uint32_t nTime)  { while (nTime--);}
 void delay_ms(uint32_t nTime) { while (nTime--) {delay_X(58823);}}




 void  TIM3_IRQHandler (void)     //void del timer
       {
        uint32_t tmpreg=0x0000000;
 	   if((GPIOA-> IDR & 0X0000001)==0X00000001)
 	   {
 		   GPIOA -> BSRR =0X0000001;
 		   SS++;
 	   }
 	   else GPIOA->BSRR =0X00010000;

 	   {
           SS++;
       }
 	   TIM3->SR = 0X00000000;
 	   tmpreg = TIM3->SR;
 	   tmpreg =tmpreg/2;
       }

 void EXTI0_IRQHandler(void)
 {
    var1=1;
 	EXTI -> PR =   ((uint32_t)0x00000001);


 }



 void EXTI4_IRQHandler(void)
 {
       var1=2;
	   EXTI -> PR =((uint32_t)0x00000010);


 }

 void EXTI1_IRQHandler(void) //void de la interrupcion EXT0 boton izquerda d1
     {
	 if((EXTI->PR & 0X00000002)==0X00000002)
		  {
			MM++;
		  }
	    	EXTI -> PR =   ((uint32_t)0x00000002);


     }

   void EXTI2_IRQHandler(void) //void de la interrupcion EXT0 boton izquerda d2
     {
	   if((EXTI->PR & 0X00000004)==0X00000004)
	   	 	  {
	   	 		HH++;
	   	 	  }
	     	   EXTI -> PR =   ((uint32_t)0x00000004);

     }



 void EXTI9_5_IRQHandler(void)
 {
       var2=3;
	   EXTI -> PR =   ((uint32_t)0x0000100);
 }


 void EXTI15_10_IRQHandler(void)
  {

       var2=4;
	   EXTI -> PR =   ((uint32_t)0x0001000);


  }

 void izquierda(void)
 {
	 LCD_Cadena(1,1,"izquierda ");
	 LCD_Cadena(2,3,":");
	 LCD_Cadena(2,6,":");
	 LCD_Home();
     LCD_Clear();
           	  GPIOC->ODR  =(uint32_t)0x00000001;
	          delay_ms(del);
	          GPIOC->ODR  =(uint32_t)0x00000002;
	          delay_ms(del);
	          GPIOC->ODR  =(uint32_t)0x00000004;
	          delay_ms(del);
	          GPIOC->ODR  =(uint32_t)0x00000008;

 }


 void derecha (void)
 {
	 LCD_Cadena(1,1,"derecha ");
	 LCD_Cadena(2,3,":");
     LCD_Cadena(2,6,":");
	 LCD_Home();
	 LCD_Clear();
           	  GPIOC->ODR  =(uint32_t)0x00000008;
	          delay_ms(del);
	          GPIOC->ODR  =(uint32_t)0x00000004;
	          delay_ms(del);
	          GPIOC->ODR  =(uint32_t)0x00000002;
	          delay_ms(del);
	          GPIOC->ODR  =(uint32_t)0x00000001;
 }

 void stop (void)
 {
	 LCD_Cadena(1,1,"parar ");
	 LCD_Cadena(2,3,":");
	 LCD_Cadena(2,6,":");


	          GPIOC-> ODR= (uint32_t)0X00000010;

 }


void start(void)
{

	LCD_Cadena(1,1,"inicio ");
    GPIOC-> ODR= (uint32_t)0X00000020;

	switch(var2)
	{
    case 3: izquierda();break;
    case 4: derecha();break;
	   default:;
	}
	}



	 void display(void)  //FUNCIONAMIENTO PRINCIPAL
	  {
		           seg();
		 		   min();
		 		  hhrs();


		 		 if(SS==59)
		 		 {
		 			 SS=0;
		 			 concaA(SS);
		 			 MM++
		 			 ;
		 		 }if(MM>60)
		 		 {
		 		     MM=0;
		 		     HH++;

		 		 }if(HH>23)
		 		 {
		 			 HH=0;
		 		 }
		 		 }




	 void seg(void) //DISPLAY SEGUNDOS
	   {

		   switch(SS)
		   		        {

		   	    case 0: LCD_Cadena(2,7,"00"); break;
		   	    case 1: LCD_Cadena(2,7,"01"); break;
		   	    case 2: LCD_Cadena(2,7,"02"); break;
		        case 3: LCD_Cadena(2,7,"03"); break;
		   	    case 4: LCD_Cadena(2,7,"04"); break;
		   	    case 5: LCD_Cadena(2,7,"05"); break;
		   	    case 6: LCD_Cadena(2,7,"06"); break;
		   	    case 7: LCD_Cadena(2,7,"07"); break;
		        case 8: LCD_Cadena(2,7,"08"); break;
		   	    case 9: LCD_Cadena(2,7,"09"); break;
		   	    case 10: LCD_Cadena(2,7,"10"); break;
		   	    case 11: LCD_Cadena(2,7,"11"); break;
		   	    case 12: LCD_Cadena(2,7,"12"); break;
		        case 13: LCD_Cadena(2,7,"13"); break;
		   	    case 14: LCD_Cadena(2,7,"14"); break;
		        case 15: LCD_Cadena(2,7,"15"); break;
		        case 16: LCD_Cadena(2,7,"16"); break;
		   	    case 17: LCD_Cadena(2,7,"17"); break;
		        case 18: LCD_Cadena(2,7,"18"); break;
		        case 19: LCD_Cadena(2,7,"19"); break;
		   	    case 20: LCD_Cadena(2,7,"20"); break;
		        case 21: LCD_Cadena(2,7,"21"); break;
		   	    case 22: LCD_Cadena(2,7,"22"); break;
		   	    case 23: LCD_Cadena(2,7,"23"); break;
		   	    case 24: LCD_Cadena(2,7,"24"); break;
		   	    case 25: LCD_Cadena(2,7,"25"); break;
		   	    case 26: LCD_Cadena(2,7,"26"); break;
		   	    case 27: LCD_Cadena(2,7,"27"); break;
		   	    case 28: LCD_Cadena(2,7,"28"); break;
		   	    case 29: LCD_Cadena(2,7,"29"); break;
		   	    case 30: LCD_Cadena(2,7,"30"); break;
		   		case 31: LCD_Cadena(2,7,"31"); break;
		        case 32: LCD_Cadena(2,7,"32"); break;
		   	    case 33: LCD_Cadena(2,7,"33"); break;
		   	    case 34: LCD_Cadena(2,7,"34"); break;
		   	    case 35: LCD_Cadena(2,7,"35"); break;
		   	    case 36: LCD_Cadena(2,7,"36"); break;
		   	    case 37: LCD_Cadena(2,7,"37"); break;
		   	    case 38: LCD_Cadena(2,7,"38"); break;
		        case 39: LCD_Cadena(2,7,"39"); break;
		        case 40: LCD_Cadena(2,7,"40"); break;
		        case 41: LCD_Cadena(2,7,"41"); break;
		        case 42: LCD_Cadena(2,7,"42"); break;
		        case 43: LCD_Cadena(2,7,"43"); break;
		        case 45: LCD_Cadena(2,7,"44"); break;
		        case 46: LCD_Cadena(2,7,"45"); break;
		        case 47: LCD_Cadena(2,7,"46"); break;
		        case 48: LCD_Cadena(2,7,"47"); break;
		        case 49: LCD_Cadena(2,7,"48"); break;
		        case 50: LCD_Cadena(2,7,"50"); break;
		        case 51: LCD_Cadena(2,7,"51"); break;
		        case 52: LCD_Cadena(2,7,"52"); break;
		        case 53: LCD_Cadena(2,7,"53"); break;
		   	    case 54: LCD_Cadena(2,7,"54"); break;
		   	    case 55: LCD_Cadena(2,7,"55"); break;
	            case 56: LCD_Cadena(2,7,"56"); break;
		        case 57: LCD_Cadena(2,7,"57"); break;
		        case 58: LCD_Cadena(2,7,"58"); break;
	    	    case 59: LCD_Cadena(2,7,"59"); break;
		   		        }
	   }



	   void min(void)
	   {
		    switch(MM)   //DISPLAY MINUTOS
				           {
				        	    case 0: LCD_Cadena(2,4,"00"); break;
				        	    case 1: LCD_Cadena(2,4,"01"); break;
				        	    case 2: LCD_Cadena(2,4,"02"); break;
				                case 3: LCD_Cadena(2,4,"03"); break;
				        	    case 4: LCD_Cadena(2,4,"04"); break;
				        	    case 5: LCD_Cadena(2,4,"05"); break;
				        	    case 6: LCD_Cadena(2,4,"06"); break;
				        	    case 7: LCD_Cadena(2,4,"07"); break;
				                case 8: LCD_Cadena(2,4,"08"); break;
				        	    case 9: LCD_Cadena(2,4,"09"); break;
				        	    case 10: LCD_Cadena(2,4,"10"); break;
				        	    case 11: LCD_Cadena(2,4,"11"); break;
				        	    case 12: LCD_Cadena(2,4,"12"); break;
				                case 13: LCD_Cadena(2,4,"13"); break;
				        	    case 14: LCD_Cadena(2,4,"14"); break;
				                case 15: LCD_Cadena(2,4,"15"); break;
				                case 16: LCD_Cadena(2,4,"16"); break;
				        	    case 17: LCD_Cadena(2,4,"17"); break;
				                case 18: LCD_Cadena(2,4,"18"); break;
				                case 19: LCD_Cadena(2,4,"19"); break;
				        	    case 20: LCD_Cadena(2,4,"20"); break;
				                case 21: LCD_Cadena(2,4,"21"); break;
				        	    case 22: LCD_Cadena(2,4,"22"); break;
				        	    case 23: LCD_Cadena(2,4,"23"); break;
				        	    case 24: LCD_Cadena(2,4,"24"); break;
				        	    case 25: LCD_Cadena(2,4,"25"); break;
				        	    case 26: LCD_Cadena(2,4,"26"); break;
				        	    case 27: LCD_Cadena(2,4,"27"); break;
				        	    case 28: LCD_Cadena(2,4,"28"); break;
				        	    case 29: LCD_Cadena(2,4,"29"); break;
				        	    case 30: LCD_Cadena(2,4,"30"); break;
				        		case 31: LCD_Cadena(2,4,"31"); break;
				                case 32: LCD_Cadena(2,4,"32"); break;
				        	    case 33: LCD_Cadena(2,4,"33"); break;
				        	    case 34: LCD_Cadena(2,4,"34"); break;
				        	    case 35: LCD_Cadena(2,4,"35"); break;
				        	    case 36: LCD_Cadena(2,4,"36"); break;
				        	    case 37: LCD_Cadena(2,4,"37"); break;
				        	    case 38: LCD_Cadena(2,4,"38"); break;
				                case 39: LCD_Cadena(2,4,"39"); break;
				                case 40: LCD_Cadena(2,4,"40"); break;
				                case 41: LCD_Cadena(2,4,"41"); break;
				                case 42: LCD_Cadena(2,4,"42"); break;
				                case 43: LCD_Cadena(2,4,"43"); break;
				                case 45: LCD_Cadena(2,4,"44"); break;
				                case 46: LCD_Cadena(2,4,"45"); break;
				                case 47: LCD_Cadena(2,4,"46"); break;
				                case 48: LCD_Cadena(2,4,"47"); break;
				                case 49: LCD_Cadena(2,4,"48"); break;
				                case 50: LCD_Cadena(2,4,"50"); break;
				                case 51: LCD_Cadena(2,4,"51"); break;
				                case 52: LCD_Cadena(2,4,"52"); break;
				                case 53: LCD_Cadena(2,4,"53"); break;
				        		case 54: LCD_Cadena(2,4,"54"); break;
				        	    case 55: LCD_Cadena(2,4,"55"); break;
				        	    case 56: LCD_Cadena(2,4,"56"); break;
				                case 57: LCD_Cadena(2,4,"57"); break;
				                case 58: LCD_Cadena(2,4,"58"); break;
				        	    case 59: LCD_Cadena(2,4,"59"); break;


	   }
	   }



	   void hhrs(void)
	   {

		   switch(HH)//DISPLAY HORAS
				        		        {

				        	    case 0: LCD_Cadena(2,1,"00"); break;
				        	    case 1: LCD_Cadena(2,1,"01"); break;
				        	    case 2: LCD_Cadena(2,1,"02"); break;
				                case 3: LCD_Cadena(2,1,"03"); break;
				        	    case 4: LCD_Cadena(2,1,"04"); break;
				        	    case 5: LCD_Cadena(2,1,"05"); break;
				        	    case 6: LCD_Cadena(2,1,"06"); break;
				        	    case 7: LCD_Cadena(2,1,"07"); break;
				                case 8: LCD_Cadena(2,1,"08"); break;
				        	    case 9: LCD_Cadena(2,1,"09"); break;
				        	    case 10: LCD_Cadena(2,1,"10"); break;
				        	    case 11: LCD_Cadena(2,1,"11"); break;
				        	    case 12: LCD_Cadena(2,1,"12"); break;
				                case 13: LCD_Cadena(2,1,"13"); break;
				        	    case 14: LCD_Cadena(2,1,"14"); break;
				                case 15: LCD_Cadena(2,1,"15"); break;
				                case 16: LCD_Cadena(2,1,"16"); break;
				        	    case 17: LCD_Cadena(2,1,"17"); break;
				                case 18: LCD_Cadena(2,1,"18"); break;
				                case 19: LCD_Cadena(2,1,"19"); break;
				        	    case 20: LCD_Cadena(2,1,"20"); break;
				                case 21: LCD_Cadena(2,1,"21"); break;
				        	    case 22: LCD_Cadena(2,1,"22"); break;
				        	    case 23: LCD_Cadena(2,1,"23"); break;

				       }

}



