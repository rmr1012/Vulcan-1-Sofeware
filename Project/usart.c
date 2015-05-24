#include "usart.h"

//////////////////////////////////////////////////////////////////
//printf function set up	  
#if 1
#pragma import(__use_no_semihosting)                              
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef¡¯ d in stdio.h. */ 
FILE __stdout;       
//define_sys_exit()to avoid semi-master mode    
_sys_exit(int x) 
{ 
	x = x; 
} 
//redefine fputc function 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//loop until done   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 
//end
//////////////////////////////////////////////////////////////////

