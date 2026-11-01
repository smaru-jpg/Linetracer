#include <mega128.h>
// Alphanumeric LCD Module functions
#asm
   .equ __lcd_port=0x12 ;PORTD
#endasm
#include <alcd.h>
#include <delay.h>
#include <stdio.h>
#include "AccTable.h"

//About Switch
#define Left_switch_on    (!PINE.0)
#define Middle_switch_on  (!PINE.1)
#define Right_switch_on   (!PINE.2)
#define Left_switch_off   (PINE.0)
#define Middle_switch_off (PINE.1)
#define Right_switch_off  (PINE.2)

//About Motor
#define Motor_on  TCCR1B = 0x09, TCCR3B = 0x09
#define Motor_off TCCR1B = 0x00, TCCR3B = 0x00

//About Timer
#define Timer_on  TCCR0 = 0x07
#define Timer_off TCCR0 = 0x00
#define Timer_set TCNT0 = 0x64 
                      
//About menu
int menu = 0; 

//About LCD
unsigned char lcd_data[40];

//About ADC
unsigned char adc_data[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //adc변환 이후 adc값이 저장되는 배열 
unsigned char adc_max[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //tuning에서 최대값 및 최소값을 넣기 위한 배열
unsigned char adc_min[8] = {255, 255, 255, 255, 255, 255, 255, 255};
unsigned char adc_normal[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //실제로 사용하는 adc값 
unsigned char mux = 0; 

//About position                                        
signed int position = 0;
//signed int position_array[8] = {0, 25, 15, 5, -5, -15, -25, 0}; //position이 반대인 경우 이 배열을 바꾸세요  
signed int position_array[8] = {0, -25, -15, -5, 5, 15, 25, 0};
unsigned char Pgain=44;  
//unsigned char Pgain2=44;    
                               
//About Motor                               
//unsigned char RMotorPhase[8] = {0x10, 0x50, 0x40, 0x60, 0x20, 0xa0, 0x80, 0x90};    
unsigned char RMotorPhase[8] = {0x90, 0x80, 0xa0, 0x20, 0x60, 0x40, 0x50, 0x10};     
//unsigned char LMotorPhase[8] = {0x09, 0x08, 0x0a, 0x02, 0x06, 0x04, 0x05, 0x01};
unsigned char LMotorPhase[8] = {0x01, 0x05, 0x04, 0x06, 0x02, 0x0a, 0x08, 0x09};
unsigned char RPhaseIndex = 0; //motor상을 바꾸어 주기 위한 변수
unsigned char LPhaseIndex = 0;
signed long  RaccTableIndex = 0; //모터 속더 관련 변수들
signed long  LaccTableIndex = 0;
signed long  SearchTableIndex = 950;
signed long  TableIndexTarget = 1000;
signed long  Speed = 1000;
signed long  Speed_2nd = 2500;
signed long  OCRr = 65535;
signed long  OCRl = 65535;
signed long  templ=0;   
signed long  tempr=0;
signed int  denominator = 0;
unsigned long base_vel = 0;
unsigned long acc_vel = 35;
signed long  SL_percentage = 600;
signed long S_L = 3000;


//About MarkSensor
unsigned char KindsOfSections[150]; //구간의 종류를 구분하기 위한 변수
unsigned char CrossFlag = 0; //cross를 구분하기 위한 변수
unsigned char right_turn_mark_cnt = 0 ; //turn mark를 인식하기 위한 변수
unsigned char left_turn_mark_cnt = 0 ;          
unsigned char end_mark_cnt = 0 ;
unsigned char turn_mark = 0 ;
unsigned char turn_mark_cnt = 0 ;
unsigned char flag = 0 ;
unsigned char mark_adc_max = 20;
unsigned char End_cnt = 20;
unsigned char End = 20;
unsigned char Mark_cnt = 25;
unsigned char Mark1 = 25;
unsigned char step = 0;
unsigned char motor_con = 0; //end mark를 찍은 이후 멈추기 위한 변수
unsigned char minustop = 3;
unsigned int SectionLength[150];
unsigned int SectionLength2[150]; //구간의 길이를 넣기 위한 변수들
int index = 0;

//About Timer
unsigned char temp = 0; //시간을 제기 위한 변수들
unsigned char sec = 0;
unsigned char min = 0;

//About Control flag
unsigned char Timer_flag = 1;

// Timer 0 overflow interrupt service routine
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
// Place your code here
    Timer_set;
    temp = temp + 1;
    
    if(temp == 100)
    {
        temp = 0;
        sec = sec + 1;
        
        if(sec == 60) min = min + 1;
    }
    //따로 주석을 달지 않겟습니다.
    //스스로 공부해보세요
    //HW
}

// Timer 1 output compare A interrupt service routine
interrupt [TIM1_COMPA] void timer1_compa_isr(void)
{
    RPhaseIndex++;
    RPhaseIndex &= 0x07;
    //HW 만약 RPhaseIndex = 0x80이 되어서 and 시키면 어떻케 될까요? 이 구문이 하는 역활?        
         
    if(TableIndexTarget > RaccTableIndex) RaccTableIndex++;
    if(TableIndexTarget < RaccTableIndex)
    {
        if(motor_con == 1) RaccTableIndex -= minustop;
        else RaccTableIndex--;
    }
    if(RaccTableIndex <= 0) RaccTableIndex = 0;
    //목표의 속도에 도달하도록 함
    //예를들어 TableIndexTarget = 400인 경우
    //RaccTableIndex이 작은 경우 ++
    //RaccTableIndex이 큰 경우 --          
    
    OCRr = tempr;
    OCR1AH = OCRr >> 8;
    OCR1AL = OCRr;
    //계산 되어진 OCR값을 넣어 줌
    //OCR1AH = OCR >> 8의 의미는
    //HW 답 써놨는데 송규섭이 숙제로 내래요
    
    PORTC = RMotorPhase[RPhaseIndex] | LMotorPhase[LPhaseIndex];
    //HW 2개의 상을 OR시켜서 PORTC에 넣어 주는 이유는 ?
    
    step++;
}

// Timer 3 output compare A interrupt service routine
interrupt [TIM3_COMPA] void timer3_compa_isr(void)
{
    LPhaseIndex++;
    LPhaseIndex &= 0x07;  
    
    if(TableIndexTarget > LaccTableIndex) LaccTableIndex++;
    if(TableIndexTarget < LaccTableIndex)
    {
        if(motor_con == 1) LaccTableIndex -= minustop;
        else LaccTableIndex--;
    }
    if(LaccTableIndex <= 0) LaccTableIndex = 0;  
    
    OCRl = templ;
    OCR3AH = OCRl>>8;
    OCR3AL = OCRl;
    PORTC = RMotorPhase[RPhaseIndex] | LMotorPhase[LPhaseIndex];
    
    step++;
}                

// ADC interrupt service routine 
unsigned char emit[8] = {0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x01};
interrupt [ADC_INT] void adc_isr(void)
{   
    PORTA = emit[mux];
    //발광부를 켜줌 항상 켜두는 것이 아니라 필요한 것을 켜둠
    adc_data[mux] = ADCH;
    //ADC값의 high값을 사용함
    mux++;
    mux &= 0x07;//if(mux >= 8) mux=0;
    
    ADMUX = mux | 0x60;  
    ADCSRA |= 0x40;  
}               

void initiation()
{
    //타이머 관련 초기화
    OCRr = OCRl = 65535;
    TCNT1H = TCNT1L = 0x00;
    TCNT3H = TCNT3L = 0x00;
    
    //mark 관련 초기화 
    turn_mark = 0;
    right_turn_mark_cnt = 0;
    left_turn_mark_cnt = 0;
    end_mark_cnt = 0;
    step = 0;

    //timer 관련 초기화    
    Timer_set;
    temp = 0;
    sec = 0;
    min = 0;
    
    position = 0;
} 

//About Position
void update_position(void)
{
    signed long sigma_p_an=0;
    signed long sigma_an=0;
    unsigned char adc = 0;   
    
    for(adc=0; adc < 8; adc++)
    {
        if(adc_data[adc] < adc_min[adc]) adc_normal[adc] = 0;
        else if(adc_data[adc] > adc_max[adc]) adc_normal[adc] = 100;
        else adc_normal[adc] = ((unsigned long)adc_data[adc] - adc_min[adc]) * 100
                                / ((unsigned long)adc_max[adc] - adc_min[adc]);
        //adc_min 보다 작은 경우 adc_mormal = 0
        //adc_max 보다 큰 겨우 adc_normal = 100
        //그 사이의 값일 경우에는
        //(현재 ADC값(adc_data) - ADC 최소값(adc_min)
        // / ADC 최대값(adc_max) - ADC 최소값(adc_min)) * 100
        //을 통해 ADC값 표준화
          
        //이해가 안되면 직접 손으로 써보세요
          
        sigma_p_an += (signed long)position_array[adc] * adc_normal[adc];
        //위에서 구한 각 표준화 된 값과
        //각 센서위치의 포지션값을 곱해주어 전부 더함 
          
        sigma_an += adc_normal[adc];
        //표준화된 센서 값만 전부 다시 더 함 
    
    position = sigma_p_an / sigma_an;
    }
    //포지션값 * 센서의 표준화값의 합 / 센서의 표준화 값의 합
    //을 통해 포지션 값을 구함
}

void update_PID(void)
{
    signed long temp;
    
    temp = (signed long)Pgain * position;
    
    if(temp > 1111) temp = 1111;
    else if(temp < -1111) temp = -1111;

    temp = temp * 9;
    denominator = temp;

    tempr = (((unsigned long)table[RaccTableIndex]) * 10000 / (denominator+10000));
    if(tempr>65534) tempr = 65534;
	if(tempr<1) tempr = 1;
    //table[]이란 베열은 위의 선언에 없지만 참조한 AccTable.h에 있음
    //HW if(tempr > 65534)나 밑의 if 문이 필요한 이유?
	
	
    templ = (((unsigned long)table[LaccTableIndex]) * 10000 / (-denominator+10000));
    if(templ>65534) templ = 65534;
    if(templ<1) templ = 1;

}                  

//About Motor
void motor_off()
{
    motor_con = 1;
    
    while((LaccTableIndex>0) || (RaccTableIndex>0))
    {
        TableIndexTarget = 0;
        update_PID();
    }
    //TableIndexTarget이 0이 될떄까지 기다림
    
    motor_con = 0; 

    //모터 관련된 레지스터 끄기
    Motor_off;
    TIMSK = 0x00;
    ETIMSK= 0x00;
    PORTC = 0x00; 
    PORTG = 0x00;
    
    //Timer off
    Timer_off;
    
    TableIndexTarget = SearchTableIndex;
}
 
//About line out
void lineout(void)
{
    if((adc_normal[1] < 20) && (adc_normal[2] < 20) && (adc_normal[3] < 20) && (adc_normal[4] < 20) && (adc_normal[5] < 20) && (adc_normal[6] < 20))
    {
    //mark sensor를 제외한 나머지 sensor가 모두 20(ADC값이)보다 작아진 경우에는 linout
    //즉 적어도 한개 이상의 센서가 라인 위에 있어야 된다.
        motor_off();
        lcd_clear();
        while(Left_switch_off)
        {
            lcd_gotoxy(0, 0);
            lcd_putsf("LineOut");
            lcd_gotoxy(0, 1);
            sprintf(lcd_data, "%dmarks", turn_mark);
            lcd_puts(lcd_data);
            delay_ms(100);
        }
    }
}

void mark(void) //스타트 마크를 지나면 flag = 1, 왼,오,엔드 마크 카운터 들을 0으로 초기화
{ 
    KindsOfSections[0] = 1;
    //처음 출발할때는 항상 직선이므로 1

    if((adc_normal[7] > mark_adc_max) && (adc_normal[0] > mark_adc_max) && (CrossFlag == 0))
    {
        end_mark_cnt += step >> 1;
        right_turn_mark_cnt = 0;
        left_turn_mark_cnt = 0;    
        PORTG=0x00;  
    }
    else if((adc_normal[7] > mark_adc_max) && (adc_normal[0] < mark_adc_max) && (CrossFlag == 0))
    {  
        right_turn_mark_cnt += step >> 1;  
        PORTG=0xFF;
    }     
    else if((adc_normal[0] > mark_adc_max) && (adc_normal[7] < mark_adc_max) && (CrossFlag == 0))
    {  
        left_turn_mark_cnt += step >> 1;   
        PORTG=0xFF;   
    }
    else if(end_mark_cnt >= End_cnt)
    {
        if(flag == 0)
        {   
	        flag = 1;
            end_mark_cnt = 0;    
            turn_mark_cnt = 0;
            SectionLength[0] = 0;
        }
        else if(flag == 1)
        {            
            flag = 0; 
            motor_off();
	        
	        while(Left_switch_off)
        	{
                if(Timer_flag)
                {
                    lcd_gotoxy(0, 0);
                    sprintf(lcd_data, "%d marks", turn_mark);
                    lcd_puts(lcd_data);
                    lcd_gotoxy(0, 1);
                    sprintf(lcd_data, "%dm %ds", min, sec);
                    lcd_puts(lcd_data);
                }
                else
                {
                    lcd_gotoxy(0, 0);
                    lcd_putsf("Finish");
                    lcd_gotoxy(0, 1);
                    sprintf(lcd_data, "%d marks", turn_mark);
                    lcd_puts(lcd_data);
                }
        	}
            PORTG=0x00;              
        }
    }                                     
    else if((right_turn_mark_cnt >= Mark_cnt) && (KindsOfSections[turn_mark] == 3)) //right 2 직선
    {                   
        turn_mark++;
        flag = 1;   
        KindsOfSections[turn_mark] = 1;    
        SectionLength[turn_mark] = 0;
        right_turn_mark_cnt = 0;
        left_turn_mark_cnt = 0;
        end_mark_cnt = 0;
        PORTG=0x00;     
    }                              
    else if((left_turn_mark_cnt >= Mark_cnt) && (KindsOfSections[turn_mark] == 2)) //left 2 직선
    {                   
        turn_mark++;
        flag = 1;   
        KindsOfSections[turn_mark] = 1;    
        SectionLength[turn_mark] = 0;
        right_turn_mark_cnt = 0;
        left_turn_mark_cnt = 0;
        end_mark_cnt  = 0; 
        PORTG=0x00;  
    }
    else if(right_turn_mark_cnt >= Mark_cnt) //right
    {                   
        turn_mark++;
        flag = 1;   
        KindsOfSections[turn_mark] = 3;    
        SectionLength[turn_mark] = 0;
        right_turn_mark_cnt = 0;
        left_turn_mark_cnt = 0;
        end_mark_cnt = 0;
        PORTG=0xFF;
    }       
    else if(left_turn_mark_cnt >= Mark_cnt) //left
    {                   
        turn_mark++;
        flag = 1;   
        KindsOfSections[turn_mark] = 2;   
        SectionLength[turn_mark] = 0;
        right_turn_mark_cnt = 0;
        left_turn_mark_cnt = 0;
        end_mark_cnt = 0;
        PORTG=0xFF;   
    }
    else
    {
        right_turn_mark_cnt = 0;
        left_turn_mark_cnt = 0;
        end_mark_cnt = 0;
        PORTG=0x00;   
    }
          
    if(flag == 1) SectionLength[turn_mark] += step >> 1;
     
    step = 0;  
} 

void mark2(void)
{ 
    KindsOfSections[0] = 1;
    //처음 출발할때는 항상 직선이므로 1

    if((adc_normal[7] > mark_adc_max) && (adc_normal[0] > mark_adc_max) && (CrossFlag == 0))
    {
        end_mark_cnt += step >> 1;
        right_turn_mark_cnt = 0;
        left_turn_mark_cnt = 0;
        PORTG=0x00;      
    }
    else if((adc_normal[7] > mark_adc_max) && (adc_normal[0] < mark_adc_max) && (CrossFlag == 0))
    {  
        right_turn_mark_cnt += step >> 1;
        PORTG=0xFF;      
    }     
    else if((adc_normal[0] > mark_adc_max) && (adc_normal[7] < mark_adc_max) && (CrossFlag == 0))
    {  
        left_turn_mark_cnt += step >> 1; 
        PORTG=0xFF;     
    }
    else if(end_mark_cnt >= End_cnt)
    {
        if(flag == 0)
        {   
	        flag = 1;
            end_mark_cnt = 0;    
            turn_mark_cnt = 0;
            SectionLength2[0] = 0;
            PORTG=0x00;
        }
        else if(flag == 1)
        {            
            flag = 0; 
            motor_off();
	        
	        while(Left_switch_off)
        	{
                if(Timer_flag)
                {
                    lcd_gotoxy(0, 0);
                    sprintf(lcd_data, "%d marks", turn_mark);
                    lcd_puts(lcd_data);
                    lcd_gotoxy(0, 1);
                    sprintf(lcd_data, "%dm %ds", min, sec);
                    lcd_puts(lcd_data);
                }
                else
                {
                    lcd_gotoxy(0, 0);
                    lcd_putsf("Finish");
                    lcd_gotoxy(0, 1);
                    sprintf(lcd_data, "%d marks", turn_mark);
                    lcd_puts(lcd_data);
                }
        	}
            PORTG=0x00;              
        }
    }                                     
    else if((right_turn_mark_cnt >= Mark_cnt) && (KindsOfSections[turn_mark] == 3)) //right 2 직선
    {                   
        turn_mark++;
        flag = 1;   
        KindsOfSections[turn_mark] = 1;    
        SectionLength2[turn_mark] = 0;
        right_turn_mark_cnt = 0;
        left_turn_mark_cnt = 0;
        end_mark_cnt = 0;  
        PORTG=0x00;   
    }                              
    else if((left_turn_mark_cnt >= Mark_cnt) && (KindsOfSections[turn_mark] == 2)) //left 2 직선
    {                   
        turn_mark++;
        flag = 1;   
        KindsOfSections[turn_mark] = 1;    
        SectionLength2[turn_mark] = 0;
        right_turn_mark_cnt = 0;
        left_turn_mark_cnt = 0;
        end_mark_cnt  = 0;
        PORTG=0x00;    
    }
    else if(right_turn_mark_cnt >= Mark_cnt) //right
    {                   
        turn_mark++;
        flag = 1;   
        KindsOfSections[turn_mark] = 3;    
        SectionLength2[turn_mark] = 0;
        right_turn_mark_cnt = 0;
        left_turn_mark_cnt = 0;
        end_mark_cnt = 0; 
        PORTG=0xFF;
    }       
    else if(left_turn_mark_cnt >= Mark_cnt) //left
    {                   
        turn_mark++;
        flag = 1;   
        KindsOfSections[turn_mark] = 2;   
        SectionLength2[turn_mark] = 0;
        right_turn_mark_cnt = 0;
        left_turn_mark_cnt = 0;
        end_mark_cnt = 0; 
        PORTG=0xFF;  
    }
    else
    {
        right_turn_mark_cnt = 0;
        left_turn_mark_cnt = 0;
        end_mark_cnt = 0; 
        PORTG=0x00;  
    }
          
    if(flag == 1) SectionLength2[turn_mark] += step >> 1;
     
    step = 0;  
} 

void detect_cross() //앞에서 미리 주행 센서들로 인식한 크로스라인을 엔드, 스타트 마크를 인식하지 않기위해 사용함
{
    static signed int cross_length_cnt = 0;
    static signed int AutoClearCnt = 0;
                           
    if((adc_normal[1] > 15) && (adc_normal[2] > 15) && (adc_normal[3] > 15) && (adc_normal[4] > 15) && (adc_normal[5] > 15) && (adc_normal[6] > 15))
    {
        cross_length_cnt += step >> 1;
    }
    else if(cross_length_cnt > 3)
    {
        cross_length_cnt = 0;
        CrossFlag = 1;
    }
    else
    {
        cross_length_cnt = 0;
    }
    
    if(CrossFlag == 1)
    {
        AutoClearCnt += step >> 1;
        if(AutoClearCnt > 300)
        {
            CrossFlag = 0;
            AutoClearCnt = 0;
        }
    }
}   

//About Sensor Tuning
void sensor_tuning(void)
{
     unsigned char adc = 0;  
     
     delay_ms(500); //채터링 현상때문에 반복문이 한번밖에 실행안되게 됨
    
     lcd_clear();
     lcd_gotoxy(0, 0);
     lcd_putsf("Tuning!");
     delay_ms(100);       
     
     for(adc = 0; adc < 8; adc++) // 배열초기화
     {
          adc_min[adc] = 255;
          adc_max[adc] = 0;                                    
     }                     
     adc = 0; 
     
     while(Left_switch_off)
     {
         if(adc_data[adc] < adc_min[adc]) adc_min[adc] = adc_data[adc];
         if(adc_data[adc] > adc_max[adc]) adc_max[adc] = adc_data[adc];
         adc++;
         if(adc > 7) adc=0;
     }
}   

void set_timer(void)
{
    delay_ms(100);
    while(Left_switch_off)
    {
        if(Middle_switch_on || Right_switch_on) Timer_flag = Timer_flag ^ 0x01;
        
        lcd_clear();
        lcd_gotoxy(0, 0);
        lcd_putsf("Timer");        
        lcd_gotoxy(0, 1);
        if(Timer_flag) lcd_putsf("ON");
        else lcd_putsf("OFF");
        delay_ms(100);                                                                                    
    }
}  
 
//About race
void navigate(void)
{  
    lcd_clear();
     TableIndexTarget = Speed;    
        delay_ms(500);
    if(Left_switch_off)
    {
        initiation();
        
        if(Timer_flag) Timer_on;
        else Timer_off;
        
        Motor_on;
        TIMSK = 0x10 | Timer_flag;
        ETIMSK = 0x10;        
  
        while(Left_switch_off)
        {  
            update_position();
            update_PID();
            detect_cross();
            mark();
            lineout(); 
        }
        
        motor_off();
    }
}


void accel(void)
{
    if(position <= 0 && position >=-3)
    {
         TableIndexTarget = 65*position + Speed +330;
    }
    else if(position <= -3 && position >= -6)
    {
         TableIndexTarget = 25*position + Speed +195;
    }
    else if(position <= -6 && position >= -9)
    {
         TableIndexTarget = 13*position + Speed +123;
   }  
    else if(position <= -9 && position >= -12)
   {
         TableIndexTarget = 5*position + Speed +51;
   }                                          
    
     else if(position >= 0 && position <=3)
   {
         TableIndexTarget = -65*position + Speed +330;
    }
    else if(position >= 3 && position <= 6)
     {
         TableIndexTarget = -25*position + Speed +195;
    }
    else if(position >= 6 && position <= 9)
   {
         TableIndexTarget = -13*position + Speed +123;
   }  
    else if(position >= 9 && position <= 12)
    {
         TableIndexTarget = -5*position + Speed +51;
   }              
   else 
   {
        TableIndexTarget = Speed;
   }                    
}

void accel_2(void)
{
    if(position <= 0 && position >=-3)
    {
         TableIndexTarget = 50*position + Speed +270;
    }
    else if(position <= -3 && position >= -6)
    {
         TableIndexTarget = 25*position + Speed +195;
    }
    else if(position <= -6 && position >= -9)
    {
         TableIndexTarget = 10*position + Speed +105;
   }  
    else if(position <= -9 && position >= -12)
   {
         TableIndexTarget = 5*position + Speed +60;
   }        
     else if(position >= 0 && position <= 3)
   {
         TableIndexTarget = -50*position + Speed +270;
    }
    else if(position >= 3 && position <= 6)
     {
         TableIndexTarget = -25*position + Speed +195;
    }
    else if(position >= 6 && position <= 9)
   {
         TableIndexTarget = -13*position + Speed +105;
   }  
    else if(position >= 9 && position <= 12)
    {
         TableIndexTarget = -5*position + Speed +60;
   }                  
   else 
   {
        TableIndexTarget = Speed;
   }                                    
}

void accelate(void)
{  
    lcd_clear();
     TableIndexTarget = Speed;    
        delay_ms(500);
    if(Left_switch_off)
    {
        initiation();
        
        if(Timer_flag) Timer_on;
        else Timer_off;
        
        Motor_on;
        TIMSK = 0x10 | Timer_flag;
        ETIMSK = 0x10;        
  
        while(Left_switch_off)
        {  
            update_position();
            update_PID(); 
            accel();
            detect_cross();
            mark();
            lineout(); 
        }
        
        motor_off();
    }
}

void accelate_2(void)
{  
    lcd_clear();
     TableIndexTarget = Speed;    
        delay_ms(500);
    if(Left_switch_off)
    {
        initiation();
        
        if(Timer_flag) Timer_on;
        else Timer_off;
        
        Motor_on;
        TIMSK = 0x10 | Timer_flag;
        ETIMSK = 0x10;        
  
        while(Left_switch_off)
        {  
            update_position();
            update_PID(); 
            accel_2();
            detect_cross();
            mark();
            lineout(); 
        }
        
        motor_off();
    }
}

void set_speed(void)
{
    delay_ms(500);   
    
    while(Left_switch_off)
    { 
     lcd_clear();
     lcd_gotoxy(0, 0);
     lcd_putsf("Speed");
     lcd_gotoxy(0, 1); 
     sprintf(lcd_data, "%d", Speed);
     lcd_puts(lcd_data);
     delay_ms(100);
     if(Middle_switch_on) Speed += 5;
     if(Right_switch_on) Speed -= 5; 
     }
}

void set_Pgain(void)
{
    delay_ms(500);   
    
    while(Left_switch_off)
    { 
     lcd_clear();
     lcd_gotoxy(0, 0);
     lcd_putsf("Pgain");
     lcd_gotoxy(0, 1); 
     sprintf(lcd_data, "%d", Pgain);
     lcd_puts(lcd_data);
     delay_ms(100);
     if(Middle_switch_on) Pgain += 1;
     if(Right_switch_on) Pgain -= 1; 
     }
}

void set_Speed_2nd(void)
{
    delay_ms(500);   
    
    while(Left_switch_off)
    { 
     lcd_clear();
     lcd_gotoxy(0, 0);
     lcd_putsf("2nd_Speed");
     lcd_gotoxy(0, 1); 
     sprintf(lcd_data, "%d", Speed_2nd);
     lcd_puts(lcd_data);
     delay_ms(100);
     if(Middle_switch_on) Speed_2nd += 100;
     if(Right_switch_on) Speed_2nd -= 100; 
     }
}

void set_2nd(void)
{   
    if(KindsOfSections[turn_mark]=1 && SectionLength[turn_mark] >= S_L 
    && SectionLength2[turn_mark] <= SectionLength[turn_mark]*(SL_percentage*0.001)
     && SectionLength2[turn_mark] >= SectionLength[turn_mark]*0.05)
     {
      TableIndexTarget = Speed_2nd;
     }           
     else if(KindsOfSections[turn_mark]=1 && S_L > SectionLength[turn_mark] >= 1500 
    && SectionLength2[turn_mark] <= SectionLength[turn_mark]*(SL_percentage*0.001)
     && SectionLength2[turn_mark] >= SectionLength[turn_mark]*0.05) 
     {
      TableIndexTarget = 2000;
     }   
     else   
  {
    if(position <= 0 && position >=-3)
    {
         TableIndexTarget = 50*position + Speed +270;
    }
    else if(position <= -3 && position >= -6)
    {
         TableIndexTarget = 25*position + Speed +195;
    }
    else if(position <= -6 && position >= -9)
    {
         TableIndexTarget = 10*position + Speed +105;
   }  
    else if(position <= -9 && position >= -12)
   {
         TableIndexTarget = 5*position + Speed +60;
   }        
     else if(position >= 0 && position <= 3)
   {
         TableIndexTarget = -50*position + Speed +270;
    }
    else if(position >= 3 && position <= 6)
     {
         TableIndexTarget = -25*position + Speed +195;
    }
    else if(position >= 6 && position <= 9)
   {
         TableIndexTarget = -13*position + Speed +105;
   }  
    else if(position >= 9 && position <= 12)
    {
         TableIndexTarget = -5*position + Speed +60;
   }                  
   else 
   {
        TableIndexTarget = Speed;
   }                                             
}                 
}   

void set_2nd_2(void)
{   
    if(KindsOfSections[turn_mark]=1 && SectionLength[turn_mark] >= S_L 
    && SectionLength2[turn_mark] <= SectionLength[turn_mark]*(SL_percentage*0.001)
     && SectionLength2[turn_mark] >= SectionLength[turn_mark]*0.06)
     {
      TableIndexTarget = Speed_2nd;
     }           
     else if(KindsOfSections[turn_mark]=1 && S_L >= SectionLength[turn_mark] >= 1500 
    && SectionLength2[turn_mark] <= SectionLength[turn_mark]*(SL_percentage*0.001)
     && SectionLength2[turn_mark] >= SectionLength[turn_mark]*0.06) 
     {
      TableIndexTarget = 2000;
     }      
     else 
     {
     TableIndexTarget = Speed;
     }
}
      
void Navigate_2(void)
{  
    lcd_clear();
     TableIndexTarget = Speed;    
        delay_ms(500);
    if(Left_switch_off)
    {
        initiation();
        
        if(Timer_flag) Timer_on;
        else Timer_off;
        
        Motor_on;
        TIMSK = 0x10 | Timer_flag;
        ETIMSK = 0x10;        
  
        while(Left_switch_off)
        {  
            update_position();
            update_PID(); 
            set_2nd();
            detect_cross();
            mark2();
            lineout(); 
        }
        
        motor_off();
    }
}

void Navigate_3(void)
{  
    lcd_clear();
     TableIndexTarget = Speed;    
        delay_ms(500);
    if(Left_switch_off)
    {
        initiation();
        
        if(Timer_flag) Timer_on;
        else Timer_off;
        
        Motor_on;
        TIMSK = 0x10 | Timer_flag;
        ETIMSK = 0x10;        
  
        while(Left_switch_off)
        {  
            update_position();
            update_PID(); 
            set_2nd_2();
            detect_cross();
            mark2();
            lineout(); 
        }
        
        motor_off();
    }
}
void set_brake()
{
         delay_ms(500);   
    
    while(Left_switch_off)
    { 
     lcd_clear();
     lcd_gotoxy(0, 0);
     lcd_putsf("brake");
     lcd_gotoxy(0, 1); 
     sprintf(lcd_data, "%d", SL_percentage);
     lcd_puts(lcd_data);
     delay_ms(100);
     if(Middle_switch_on) SL_percentage += 25;
     if(Right_switch_on) SL_percentage -= 25; 
     }
}

void set_Section(void)
{
     delay_ms(500);   
    
    while(Left_switch_off)
    { 
     lcd_clear();
     lcd_gotoxy(0, 0);
     lcd_putsf("Length");
     lcd_gotoxy(0, 1); 
     sprintf(lcd_data, "%d",SectionLength[index] );
     lcd_puts(lcd_data);
     delay_ms(100);
      if(Middle_switch_on && index < 149) index++ ;
     if(Right_switch_on && index > 0) index-- ;
     }
}

// Declare your global variables here

void set_S_L(void)
{
     delay_ms(500);   
    
    while(Left_switch_off)
    { 
     lcd_clear();
     lcd_gotoxy(0, 0);
     lcd_putsf("S_L");
     lcd_gotoxy(0, 1); 
     sprintf(lcd_data, "%d", S_L);
     lcd_puts(lcd_data);
     delay_ms(100);
     if(Middle_switch_on) S_L += 250;
     if(Right_switch_on) S_L -= 250; 
     }
}

void set_minustop(void)
{
     delay_ms(500);   
    
    while(Left_switch_off)
    { 
     lcd_clear();
     lcd_gotoxy(0, 0);
     lcd_putsf("minustop");
     lcd_gotoxy(0, 1); 
     sprintf(lcd_data, "%d", minustop);
     lcd_puts(lcd_data);
     delay_ms(100);
     if(Middle_switch_on) minustop += 1;
     if(Right_switch_on) minustop -= 1; 
     }
}

void set_Mark(void)
{
     delay_ms(500);   
    
    while(Left_switch_off)
    { 
     lcd_clear();
     lcd_gotoxy(0, 0);
     lcd_putsf("Mark");
     lcd_gotoxy(0, 1); 
     sprintf(lcd_data, "%d", Mark1);
     lcd_puts(lcd_data);
     delay_ms(100);
     if(Middle_switch_on) Mark1 += 1;
     if(Right_switch_on) Mark1 -= 1; 
     }
}

void set_End(void)
{
     delay_ms(500);   
    
    while(Left_switch_off)
    { 
     lcd_clear();
     lcd_gotoxy(0, 0);
     lcd_putsf("End");
     lcd_gotoxy(0, 1); 
     sprintf(lcd_data, "%d", End);
     lcd_puts(lcd_data);
     delay_ms(100);
     if(Middle_switch_on) End += 1;
     if(Right_switch_on) End -= 1; 
     }
}

void main(void)
{
// Declare your local variables here
// Input/Output Ports initialization
// Port A initialization
// Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In 
// State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T 
PORTA=0x00;
DDRA=0xFF;
PORTB=0x00;
DDRB=0x00;
PORTC=0x00;
DDRC=0xFF;
PORTD=0x00;
DDRD=0x00;
PORTE=0x00;
DDRE=0xC0;
PORTF=0x00;
DDRF=0x00;
// Port G initialization
// Func4=In Func3=In Func2=In Func1=In Func0=In 
// State4=T State3=T State2=T State1=T State0=T 
PORTG=0x00;
DDRG=0xFF;
// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: Timer 0 Stopped
// Mode: Normal top=FFh
// OC0 output: Disconnected
ASSR=0x00;
TCCR0=0x00;
TCNT0=0x00;
OCR0=0x00;
// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 20000.000 kHz
// Mode: CTC top=OCR1A
// OC1A output: Discon.
// OC1B output: Discon.
// OC1C output: Discon.
// Noise Canceler: Off
// Input Capture on Falling Edge
TCCR1A=0x00;
TCCR1B=0x00; //TCCR1B=0x09;
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;
OCR1CH=0x00;
OCR1CL=0x00;
// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: Timer 2 Stopped
// Mode: Normal top=FFh
// OC2 output: Disconnected
TCCR2=0x00;
TCNT2=0x00;
OCR2=0x00;
// Timer/Counter 3 initialization
// Clock source: System Clock
// Clock value: 20000.000 kHz
// Mode: CTC top=OCR3A
// Noise Canceler: Off
// Input Capture on Falling Edge
// OC3A output: Discon.
// OC3B output: Discon.
// OC3C output: Discon.
TCCR3A=0x00;
TCCR3B=0x00; //TCCR3B=0x09;
TCNT3H=0x00;
TCNT3L=0x00;
ICR3H=0x00;
ICR3L=0x00;
OCR3AH=0x00;
OCR3AL=0x00;
OCR3BH=0x00;
OCR3BL=0x00;
OCR3CH=0x00;
OCR3CL=0x00;
// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// INT2: Off
// INT3: Off
// INT4: Off
// INT5: Off
// INT6: Off
// INT7: Off
EICRA=0x00;
EICRB=0x00;
EIMSK=0x00;
// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=0x00;
ETIMSK=0x00;
// Analog Comparator initialization
// Analog Comparator: Off
// Analog Comparator Input Capture by Timer/Counter 1: Off
// Analog Comparator Output: Off
ACSR=0x80;
SFIOR=0x00;
// ADC initialization
// ADC Clock frequency: 156.250 kHz
// ADC Voltage Reference: AREF pin
// ADC High Speed Mode: Off
// Only the 8 most significant bits of
// the AD conversion result are used
ADMUX=0x20;
ADCSRA=0xCF;
// LCD module initialization
lcd_init(8);
// Global enable interrupts
#asm("sei")
while (1)
      {
        if(Middle_switch_on) menu++;
        if(Right_switch_on) menu--; 
        if(menu > 15) menu = 0;
        if(menu < 0) menu = 15;
        
        switch(menu)
        { 
            case 0: 
                    lcd_clear();
                    lcd_gotoxy(0, 0);
                    lcd_putsf("1.Sensor Tuning");
                    delay_ms(100);
                    if(Left_switch_on) sensor_tuning();  
                    break;

            case 1:
                    lcd_clear();
                    lcd_gotoxy(0, 0);
                    lcd_putsf("2.Timer");
                    delay_ms(100);
                    if(Left_switch_on) set_timer();  
                    break;
                                        
            case 2:
                    lcd_clear();
                    lcd_gotoxy(0, 0);
                    lcd_putsf("3.Navigate");
                    delay_ms(100);
                    if(Left_switch_on) navigate();  
                    break;   
                      
            case 3:
                    lcd_clear();
                    lcd_gotoxy(0, 0);
                    lcd_putsf("4.Accelate_330");
                    delay_ms(100);
                    if(Left_switch_on) accelate(); 
                    break;  
                    
            case 4:
                    lcd_clear();
                    lcd_gotoxy(0, 0);
                    lcd_putsf("5.Accelate_270");
                    delay_ms(100);
                    if(Left_switch_on) accelate_2(); 
                    break;   
                  
            case 5: 
                    lcd_clear();
                    lcd_gotoxy(0, 0);
                    lcd_putsf("6.Navigate_SL");
                    delay_ms(100);
                    if(Left_switch_on) Navigate_3();
                    break; 
                      
            case 6: 
                    lcd_clear();
                    lcd_gotoxy(0, 0);
                    lcd_putsf("7.2nd Navigate");
                    delay_ms(100);
                    if(Left_switch_on) Navigate_2();
                    break;      
                    
            
                    
            case 7:
                    lcd_clear();
                    lcd_gotoxy(0, 0);
                    lcd_putsf("8.Speed");
                    delay_ms(100);
                    if(Left_switch_on) set_speed(); 
                    break;
            
            case 8:                                    
                    lcd_clear();
                    lcd_gotoxy(0, 0);
                    lcd_putsf("9.Pgain");
                    delay_ms(100);
                    if(Left_switch_on) set_Pgain();
                    break;                               
            
            case 9:
                    lcd_clear();
                    lcd_gotoxy(0, 0);
                    lcd_putsf("10. brake");
                    delay_ms(100);
                    if(Left_switch_on) set_brake();
                    break;  
                    
            case 10:
                    lcd_clear();
                    lcd_gotoxy(0, 0);
                    lcd_putsf("11. Length");
                    delay_ms(100);
                    if(Left_switch_on) set_Section();
                    break; 
                    
             case 11:
                    lcd_clear();
                    lcd_gotoxy(0, 0);
                    lcd_putsf("12. S_L");
                    delay_ms(100);
                    if(Left_switch_on) set_S_L();
                    break;     
                    
             case 12:
                    lcd_clear();
                    lcd_gotoxy(0, 0);
                    lcd_putsf("13. Mark");
                    delay_ms(100);
                    if(Left_switch_on) set_Mark();
                    break;  
                    
             case 13:
                    lcd_clear();
                    lcd_gotoxy(0, 0);
                    lcd_putsf("14. End");
                    delay_ms(100);
                    if(Left_switch_on) set_End();
                    break;  
                    
             case 14:
                    lcd_clear();
                    lcd_gotoxy(0, 0);
                    lcd_putsf("15. Speed_2nd");
                    delay_ms(100);
                    if(Left_switch_on) set_Speed_2nd();
                    break;      
                    
             case 15:
                    lcd_clear();
                    lcd_gotoxy(0, 0);
                    lcd_putsf("16. minustop");
                    delay_ms(100);
                    if(Left_switch_on) set_minustop();
                    break;   
         }
         delay_ms(200);  
      }
}
