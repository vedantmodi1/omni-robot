#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#define radius 26
#define a  21.92802820068454
#define b  0.2479251815690904
#define val 0.0174555555555  

int count=0, top1, top2, top3;
float velocity_x=0, velocity_y=0, req_angle;
float omega=0;
float velocity_1, velocity_2, velocity_3;
bool i=0;
bool j=0; 
bool imu_next_byte;
double distance;
int imu_data, an_integer, n=0; bool imu_next_byte; 
float imu_angle;
float last_error=0, integral=0, derivative=0; float error;
float last_error_distance=0, integral_distance=0, derivative_distance=0; float error_distance;
float req_change;
int k=1;
int l=0;
int required_distance=150;
int sv2[4];

float correction_velocity;

void pid()
{   
	float kp=0.001, ki=0.000005, kd=0.0000;
	int current_angle = imu_angle;
	
	if (current_angle < 180)
		error = current_angle;
	else
		error = current_angle - 360;
		
	if ((error<4.5 && error>(-4.5)) && error!=0)
	{
		omega=0;
		//kp=0;
		integral = integral + error; 
		if(integral>(50/ki))
			integral=50/ki;
		omega = (kp * error) + (ki*integral) + (kd*derivative);
		goto end;
	}
    else
	{
		integral=0;
	}
	//derivative = (error - last_error);
    omega = (kp * error) + (ki*integral) + (kd*derivative);
	
    end: omega=-omega;
}
void pid_distance()
{
	k=0;
	float kp_distance=0.001, ki_distance=0.000000, kd_distance=0.0000;
	
	error_distance = required_distance-distance; 
		if ((error_distance<5 && error_distance>(-5)) && error_distance!=0)
	{
		
		
		integral_distance = integral_distance + error_distance;
		if(integral_distance>(10/ki_distance))
		integral_distance=10/ki_distance;
		velocity_x = (kp_distance * error_distance) + (ki_distance*integral_distance) + (kd_distance*derivative_distance);
		
		goto end2;
		//kp=0;
		
	}
	else
	{
		integral_distance=0;
	}
	//derivative = (error - last_error);
	velocity_x = (kp_distance * error) + (ki_distance*integral) + (kd_distance*derivative);
	end2:	
	req_angle=0;
}


int main(void)
{   
	//***FOR IFM*** ADC4
	DDRC=0b00000000;											 //for adc4
	ADMUX=(1<<REFS0)|(1<<MUX2);                                  //Vref 5V, ADC4                                                      
	ADCSRA=(1<<ADSC)|(1<<ADEN)|(1<<ADATE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);     //enable adc,enabling auto triggering, starting conversation, prescaler 128
	
	//***FOR IMU*** UART
	DDRD|=(1<<PIND1)|(0<<PIND0);                                 // D1=TXD1 D0=RXD1, transmitter pin set as O/P, receiver as I/P
	UCSR0B=(1<<RXEN0)|(1<<RXCIE0);;                              //enabling transmitter receiver, enabling RX complete interrupt      ***FOR IMU***
	UCSR0C=(1<<UCSZ01)|(1<<UCSZ00);                              //character size is 8
    UBRR0=25;                                                    //baud rate=32800
	                                                    
	sei();
	
	//PWM ***MOTOR 1&2***   ***MOTOR 3***
	DDRB|=(1<<PINB0)|(1<<PINB3)|(1<<PINB2);               //M1 BRAKE, M3 PWM, OC1B NOT USING
	DDRD|=(1<<PIND3)|(1<<PIND6)|(1<<PIND7)|(1<<PIND5);	  //M2 PWM,	M3 DIR,  M3 BRAKE, M1 PWM
	DDRC|=(1<<PINC0)|(1<<PINC1)|(1<<PINC2);               //M2 BRAKE, M1 DIR, M2 DIR
	//PWM ON 0B, 2A, 2B, TIMER 1 FOR PID2222226
	
	TCCR0A=(1<<COM0B1)|(1<<WGM01)|(1<<WGM00);                    //non inverting, top=0xFF B3W
	TCCR0B=(1<<CS20);                                            //62500Hz no prescalar  8bit
	
	TCCR2A=(1<<COM2A1)|(1<<COM2B1)|(1<<WGM21)|(1<<WGM20);        //non inverting, top=0xFF B3W                                                     
	TCCR2B=(1<<CS20);                                            //62500Hz no prescalar  8bit
	// w/o direction set 1:200, 3:186, 2:200.
	// with direction set 2:206, 1:208 , 3:196.
	                                                  
													  
	TCCR1B=(1<<CS12)|(1<<CS10);                                  //1024 prescaler
	int sv[6], p=0;	
	int n=0;					//far to close not working
												             
	while (1)
	{  
		//***DISTANCE FROM IFM***
		if(!((ADCSRA&0b01000000)==(0b00000000)))
		{
			distance = (a + b*ADC);
		}
		if (k==0)
		{
			
			pid();
			pid_distance();
			req_angle=0;
			velocity_y=0;
		}
		if (k==1)
		{
			
         pid();
		 
		 
		 if(distance<50)
		 {
			 if(TCNT1>156 && p<6)                           //0.01s
	 		 { 
		 		 sv[p]=distance; p++; TCNT1=0;
			 } 
			 else if((sv[1]<50 && sv[1]!=0) && (sv[2]<50 && sv[2]!=0) && (sv[3]<50 && sv[3]!=0) && (sv[4]<50 && sv[4]!=0) && (sv[5]<50 && sv[5]!=0))
	 		 {   
				  if(req_angle!=30)
				  {
					  n=n+1;
				  }
		    	 req_angle=30;  //n=n+1;
		 		 TCNT1=0; sv[0]=0; sv[1]=0; sv[2]=0; sv[3]=0; sv[4]=0; sv[5]=0;   
				 p=0;
			 }
			 if(p==6)
				p=0;
		 }	
		 
		 
		 if(distance>150)
		 {
			 if(TCNT1>156 && p<6)                           //0.01s
			 {	
				sv[p]=distance; p++; TCNT1=0;
			 }
			 else if((sv[1]>150) && (sv[2]>150) && (sv[3]>150) && (sv[4]>150) && (sv[5]>150))
			 {   
				 if(req_angle!=135)
				 {
					 n=n+1;
				 }
				 req_angle=135;  //n=n+1;
				 TCNT1=0; sv[0]=0; sv[1]=0; sv[2]=0; sv[3]=0; sv[4]=0; sv[5]=0;
				 p=0;
			 }
			 if(p==6)
				p=0;
		 } 
		 
		 
		 if(n>=4)
		 {
			  if(distance<150)
			  {
				  if(TCNT1>156 && p<4)                           //0.01s
				  {
					  sv[p]=distance; p++; TCNT1=0;
				  }
				  else if((sv[1]<150 && sv[1]!=0) && (sv[2]<150 && sv[2]!=0) && (sv[3]<150 && sv[3]!=0) && (sv[4]<150 && sv[4]!=0) && (sv[5]<150 && sv[5]!=0))
				  {   
					  pid_distance();
				  }
			  }
		 }
		 
velocity_x=cos(req_angle*val);   velocity_y=sin(req_angle*val); 		 
					
	}
		

		
		velocity_1 =  - velocity_x*(sin(imu_angle*val))           +    velocity_y*(cos(imu_angle*val))          +  radius*omega;
	    velocity_2 =  - velocity_x*(sin((imu_angle + 120)*val))   +    velocity_y*(cos((imu_angle + 120)*val))  +  radius*omega;
		velocity_3 =  - velocity_x*(sin((imu_angle + 240)*val))   +    velocity_y*(cos((imu_angle + 240)*val))  +  radius*omega;
		//OCR2B=255; OCR0B=255; OCR2A=255;
		//OCR2B=(255*velocity_1); OCR0B=(255*velocity_2); OCR2A=(255*velocity_3);
		
		//***DIRECTION***	
		if(velocity_1>0)
			PORTC=PORTC|0b00000010;
		if(velocity_1<0)
			PORTC=PORTC&0b11111101;	
		if(velocity_2>0)
			PORTC=PORTC|0b00000100;
		if(velocity_2<0)
			PORTC=PORTC&0b11111011;
		if(velocity_3>0)
			PORTD=PORTD|0b01000000;
		if(velocity_3<0)
			PORTD=PORTD&0b10111111;   

		top1=abs(150*velocity_1);
			OCR0B=top1;
		top2=abs(150*velocity_2);
			OCR2B=top2;
		top3=abs(150*velocity_3);
			OCR2A=top3;
	} 

}
		 
		 


//***RECEIVING IMU DATA***
ISR (USART_RX_vect)
{
	imu_data=UDR0;
	if (imu_data & 0x80)								 //if imu_data between 80 and 99, imu_data & 0x80=0b 1000 0000
	{
		imu_data = (imu_data & 0b01111111);              //setting bit 7 as 0
		an_integer = (imu_data << 7);					 //left shifting imu_data by 7
		imu_next_byte = true;
	}
	else if (imu_next_byte)
	{
		an_integer = (an_integer + imu_data);
		imu_angle = (an_integer / 10.0);
		imu_next_byte = false;
	}
}