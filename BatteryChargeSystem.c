
//  Period ve duty ayarlamalar ile ilgili değişkenler
int dutyRatio=150;
unsigned int period;
float cycle;
int in;

char method;
int overVoltage_counter=0;
int overCurrent_counter=0;

// UART ile ilgili değişkenler
char buf[8],buf1[8],buf2[8],buf3[8],buf4[8];


// voltage okuma ile ilgili değişkenler
float okunan_voltage=0;
float set_voltage=0;
int toplam_adc_voltage=0;
int temp_adc_voltage=0;
int okunan_adc_voltage=0;
int count_voltage=0;
int voltage_offset = 5;

// current okuma ile ilgili değişkenler
float okunan_current=0;
int toplam_adc_current=0;
int temp_adc_current=0;
int okunan_adc_current=0;
float hesap1_current;
float ortalama_current;
float offset = 393.5;   //372 idi   //offset 0 akımdaki adc değerine ve akımdaki lineerliğe göre
float mv_Amp=15.65;     //14 idi    // bu da deneysel ama düz batarya deşarjda 1A dan 10A ya kadar max +-10mA sapma
char count_current=0;
int current_offset = 25;

// referans voltage ayarlama ile ilgili değişkenler
float referans_voltage=0;
float toplam_referans_voltage=0;
long int referans_adc_voltage=0;
float referansGerilimAyar=164;

// referans current ayarlama ile ilgili değişkenler
float referans_current=0;
float toplam_referans_current=0;
long int referans_current_adc=0;
float referansAkimAyar=819;

// Pid değişken tanımlama
float Kp=0;
float Ki=0;
float Kd=0;

float error_current;
float previous_current_error=0;
float integral_current=0;
float difference_current;

void GerilimOku() {
          int i=0;


                   for(i=0;i<10; i++)
                    {
                       temp_adc_voltage = ADC1_Get_Sample(1);
                       if (temp_adc_voltage<(okunan_adc_voltage+voltage_offset)&&(temp_adc_voltage>(okunan_adc_voltage-voltage_offset))) { okunan_adc_voltage=temp_adc_voltage; count_voltage=0; }
                       else {count_voltage++;}
                       if (count_voltage >5)  { okunan_adc_voltage=temp_adc_voltage; count_voltage=0; }
                       set_voltage=(float)okunan_adc_voltage;
                       toplam_adc_voltage+=set_voltage;
                    }

                    okunan_voltage = toplam_adc_voltage/(10*44);
                    if(okunan_voltage<1) { okunan_voltage=0; }
                    toplam_adc_voltage=0;
}

void ReferansGerilimSet()
{              int i=0;
                    toplam_referans_voltage=0;
                     for(i=0;i<10; i++)
                        {
                         referans_adc_voltage =  ADC2_Get_Sample(2);     // a2 deki değeri oku
                         toplam_referans_voltage+= referans_adc_voltage;
                         }

                    referans_voltage = toplam_referans_voltage /(10*referansGerilimAyar);
}

 void GerilimKontrol()
      {
        if(okunan_voltage<referans_voltage)
        {
            dutyRatio++;
            if(dutyRatio>600) { dutyRatio=600;}
        }

        if(okunan_voltage>referans_voltage)
        {
            dutyRatio--;
             if(dutyRatio<0) { dutyRatio=0;}
        }

      }
void AkimOlcum()
 {       int i=0;


                        toplam_adc_current=0;
                       for(i=0;i<20; i++)
                    {
                        temp_adc_current = ADC3_Get_Sample(3);
                       if (temp_adc_current<(okunan_adc_current+current_offset)&&(temp_adc_current>(okunan_adc_current-current_offset))) { okunan_adc_current=temp_adc_current; count_current=0; }
                       else {count_current++;}
                       if (count_current>5)  { okunan_adc_current=temp_adc_current; count_current=0; }
                       toplam_adc_current+=okunan_adc_current;
                    }
                       ortalama_current=(float)toplam_adc_current /20;
                       hesap1_current= (ortalama_current*0.7726);        // 3000/4095 oranı
                       okunan_current=hesap1_current - offset;
                       if(okunan_current<0.15) {okunan_current=0;}           //1A ' nın altında pek bi işe yaramadığı için
                       okunan_current=okunan_current/mv_Amp;


                      /* okunan_i=(float)okunan_itemp /20;
                       okunan_i= ((okunan_i*0.773) - offset)/mv_Amp;      // mv_Amp=20
                       okunan_i= okunan_i-1.85;  */
 }


 void ReferansAkimSet()
      {    int i=0;
                     toplam_referans_current=0;
                     for(i=0;i<10; i++)
                     {
                     referans_current_adc =  ADC1_Get_Sample(4);
                     toplam_referans_current+= referans_current_adc;
                     }

                     referans_current = toplam_referans_current /(10*referansAkimAyar);
      }

 void AkimKontrol()
 {
        if(okunan_current<referans_current)
        {
            dutyRatio++;
            if(dutyRatio>850) { dutyRatio=850;}
        }

        if(okunan_current>referans_current)
        {
            dutyRatio--;
             if(dutyRatio<0) { dutyRatio=0;}
        }
}
void PidAkimKontrol()
{   error_current =  okunan_current  - referans_current;
    integral_current += error_current;
    difference_current = error_current - previous_current_error;
    previous_current_error = error_current;
    dutyRatio = Kp * error_current + (Ki * integral_current * Ts) + (Kd * difference_current /Ts);


}

void MethodSecim()
{

if((okunan_voltage>=0 && okunan_voltage<=5) ) {dutyRatio=100;}

       else
       {
          if(okunan_voltage<48 || okunan_voltage<referans_voltage)
          {
          AkimKontrol();  method = "Akim Kontrol" ;
          }
          else                    // şuanki halde buraya hiç girmeez
          {
          GerilimKontrol(); method = "Gerilim Kontrol";}
          }
 }


void SendData()
{

   sprintf(buf, "%d.%02u", (int)okunan_voltage, (okunan_voltage-((int)okunan_voltage)) * 100);
   UART3_Write_Text(buf);
   UART3_Write(44);
   
  sprintf(buf1, "%d.%02u", (int)referans_voltage, (referans_voltage-((int)referans_voltage)) * 100);
   UART3_Write_Text(buf1);
    UART3_Write(44);
    
   sprintf(buf2, "%d.%02u", (int)okunan_current, (okunan_current-((int)okunan_current)) * 100);
   UART3_Write_Text(buf2);
   
  UART3_Write(44);         // virgül
  sprintf(buf3, "%d.%02u", (int)referans_current, (referans_current-((int)referans_current)) * 100);
  UART3_Write_Text(buf3);

  UART3_Write(44);
  sprintf(buf4, "%d", dutyRatio);
  UART3_Write_Text(buf4);

UART3_Write(13);
UART3_Write(10);

}


void InitTimer2(){
  RCC_APB1ENR.TIM2EN = 1;
  TIM2_CR1.CEN = 0;
  TIM2_PSC = 2;
  TIM2_ARR = 55999;
  NVIC_IntEnable(IVT_INT_TIM2);
  TIM2_DIER.UIE = 1;
  TIM2_CR1.CEN = 1;
}

void Timer2_interrupt() iv IVT_INT_TIM2 {
  TIM2_SR.UIF = 0;
      PWM_TIM4_Set_Duty(cycle*dutyRatio,_PWM_NON_INVERTED,_PWM_CHANNEL2);
           GPIO_Digital_Output(&GPIOD_ODR,
                                      _GPIO_PINMASK_14|
                                      _GPIO_PINMASK_15);


      if(okunan_voltage>48)
      {overVoltage_counter++;
       GPIOD_ODR=(1<<14);
       dutyRatio=dutyRatio-1;

      }
      else
      {
       overVoltage_counter =0;
       GPIOD_ODR=(0<<14);
      }

      if(okunan_current>4)
      {
       overCurrent_counter++;
       GPIOD_ODR=(1<<15);
       dutyRatio=dutyRatio-2;

      }
      else
      {
       overCurrent_counter=0;
       GPIOD_ODR=(0<<15);
      }

}

void InitTimer3(){   // Pid için örnekleme  süresini oluşturmak için yaptım Ts=1ms = 0.001
  RCC_APB1ENR.TIM3EN = 1;
  TIM3_CR1.CEN = 0;
  TIM3_PSC = 2;
  TIM3_ARR = 55999;
  NVIC_IntEnable(IVT_INT_TIM3);
  TIM3_DIER.UIE = 1;
  TIM3_CR1.CEN = 1;
}

void Timer3_interrupt() iv IVT_INT_TIM3 {
  TIM3_SR.UIF = 0;

}



 void kurulum()
 {
   UART3_Init_Advanced(9600,_UART_8_BIT_DATA,_UART_NOPARITY,_UART_ONE_STOPBIT,&_GPIO_MODULE_USART3_PB10_11);
   InitTimer2();
  GPIO_Digital_Output(&GPIOD_ODR,_GPIO_PINMASK_13);


  ADC1_Init();
  ADC_Set_Input_Channel(_ADC_CHANNEL_1|_ADC_CHANNEL_4);   //PA1
  ADC2_Init();
  ADC_Set_Input_Channel(_ADC_CHANNEL_2);   //PA2
  ADC3_Init();
  ADC_Set_Input_Channel(_ADC_CHANNEL_3);   //PA3


  period = PWM_TIM4_Init(40000);
  PWM_TIM4_Set_Duty(period*1,_PWM_NON_INVERTED,_PWM_CHANNEL2);
  PWM_TIM4_Start(_PWM_CHANNEL2,&_GPIO_MODULE_TIM4_CH2_PD13);
   cycle = (float)period/1000;
 }
void main()
{
 kurulum();


 while(1)
 {



           GerilimOku();
           ReferansGerilimSet();
          // GerilimKontrol();
          // AkimKontrol();
           AkimOlcum();
           ReferansAkimSet();
            MethodSecim();
           SendData();



 }

}