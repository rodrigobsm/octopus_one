/*********************************************************
* PROJETO ESTATOSFERA UNIFEOB - www.unifeob.edu.br
* Este codigo fonte faz parte do computador de bordo
* Octopus One e seu codigo fonte pode ser usado livremente
* Prof. Resp.: Rodrigo Santa Maria
* rodrigo.maria@unifeob.edu.br e demais alunos/desenvolvedores
* Ultima atualizacao: 17/06/2015
*********************************************************/
#include <LiquidCrystal.h>
#include <Servo.h>
#include <StopWatch.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <SdFat.h>
#include "DHT.h"
#include <Wire.h>
#include <LSM303.h>
#include <SFE_BMP180.h>
/*******************************************************/

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
Servo servo_foto;
long int tempo;  // tempo millis()
StopWatch tempo_missao;  // tempo missao

// CONTROLE DO MENU
int menu_atual = 1;  // menu raiz
int menu_max   = 9;
int lcd_key    = 0;

/* OPCOES DO MENU */
char* menu_txt[]={"< ESTRATOSFERA >",
                  "<    MISSAO    >",
                  "<  INTER FOTO  >",
                  "< LUMINOSIDADE >",
                  "< UMIDADE EXT  >",
                  "< PRESSAO ATM  >",
                  "< TEMP INTERNA >",
                  "< TEMP EXTERNA >",
                  "< ACELEROMETRO >",
                  "<   BUSSOLA    >"};

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

// definindo valores padra
#define INTEVALO_GRAVACAO_DADOS 5000 // 5 segundos
#define TEMPO_BUZZER_ATIVO      300  // 5 minutos em segundos
#define PINO_BUZZER             A7   // analog 2
#define PINO_TEMP_INTERNA       49   // digital 52
#define PINO_LUMINOSIDADE       A13  // analog 6
#define PINO_UMIDADE            A8   // digital 50

// CONTROLES E VARIAVEIS GLOBAIS
int        adc_key_in  = 0;
bool       missao_ativa = true;
int        segs_intervalo_fotos = 30; // em segundos
int        buzzer_state = 0;
long       previousMillis = 0;
long       previousMillisBuzzer = 0;
float      temp_interna_C;
float      temp_externa_C;
float      umidade_externa;
int        sensor_luminosidade;
int        x, y, z;     //triple axis data acelerometro
float      direcao;
LSM303     bussola;
SFE_BMP180 pressure;
char       status;      // sensor pressao
double     T,P,p0,a;    // temp e pressao
const int  chipSelect = 53;
SdFat      sd;
SdFile     myFile;
float      hours, minutes, seconds;
long int   total_coords_salvas=0;

// Temp Interna
OneWire oneWire_temp_int(PINO_TEMP_INTERNA);
DallasTemperature sensor_temp_interna(&oneWire_temp_int);

// Umidade Externa
DHT sensor_umidade_externa(PINO_UMIDADE, DHT11);

void setup()
{
 lcd.begin(16, 2);              // start the library
 lcd.setCursor(0,0);
 lcd.print("= ESTRATOSFERA ="); // print a simple message
 lcd.setCursor(0,1);
 lcd.print("  CARREGANDO... "); // print a simple message
 delay(1000);
 lcd.setCursor(0,0);
 lcd.print("= ADS  UNIFEOB ="); // print a simple message
 delay(1000);
 lcd.clear();
 
 // inicia variaveis
 servo_foto.attach(14);
 servo_foto.write(0);  // modo zerado
 
 // inicia serial
 Serial.begin(9600);
 while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
 }
 Serial.println("Projeto Estratosfera Carregado!");
 
 // seta portas 
 pinMode(PINO_BUZZER, OUTPUT);
 
 // inicia bussola
 Wire.begin();
 bussola.init();
 bussola.enableDefault();
 bussola.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
 bussola.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
 
 // inicia pressao
 pressure.begin();
  
  // Initialize SdFat or print a detailed error message and halt
  // Use half speed like the native library.
  // change to SPI_FULL_SPEED for more performance.
  pinMode(chipSelect, OUTPUT);
  digitalWrite(chipSelect, HIGH);
    
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    if (sd.card()->errorCode()) {
        Serial.println("Erro ao inicializar SD!");
    }
    
  
}
 
void loop()
{
 
  //////////// FLUXO CRITICO DO SISTEMA ////////////////
  
  tempo = millis();
  
  if (missao_ativa) {
  
          // atualiza relogio
          unsigned long over;
          hours=int(tempo_missao.elapsed()/3600000);
          over=tempo_missao.elapsed()%3600000;
          minutes=int(over/60000);
          over=over%60000;
          seconds=int(over/1000);
          
          // TODO CONTROLE DA MISSAO PASSA POR AQUI
          
         // CONTROLE DO BUZZER
         if (tempo_missao.elapsed()/1000 <= TEMPO_BUZZER_ATIVO) {
               beep_buzzer();
         } else {
               analogWrite(PINO_BUZZER, 0); // desliga buzzer
         }
          
         // TEMPERATURA INTERNA
         ler_temp_interna();
          
         // LUMINOSIDADE
         ler_luminosidade(); 
         
         // UMIDADE
         ler_umidade();
         
         // ACELEROMETRO
         ler_acelerometro();
         
         // BUSSOLA
         ler_bussola();
         
         // processa timer fotos
         tirar_foto_timer();
         
         // ler pressao
         ler_pressao();
         
         // POR FIM, GRAVA DADOS
         gravar_dados_sd();
 
  }  
  
 // imprime menu atual 
 lcd.setCursor(0,0);
 lcd.print(menu_txt[menu_atual]); 
 //lcd.setCursor(0,1);    
 //lcd.print(menu_atual);
     
 // se ta no menu raiz, joga tempo na tela e status msg
 switch (menu_atual) {
  
   // MENU RAIZ - ESTRATOSFERA
   case 0:
       lcd.setCursor(0,1);    
       lcd.print("  TEMPO: ");
       tempo_ativo();
   break;
   
   // MENU MISSAO
   case 1:
       lcd.setCursor(0,1);    
       if (missao_ativa)
           lcd.print(" MISSAO ATIVA   ");
       else
           lcd.print(" MISSAO PAUSADA ");   
   break;
   
   // MENU INTERVALO FOTOS
   case 2:
       lcd.setCursor(0,1);    
       lcd.print("A CADA "); 
       lcd.print(segs_intervalo_fotos); 
       lcd.print(" SEGS     "); 
   break;
   
   // MENU LUMINOSIDADE
   case 3:
       lcd.setCursor(0,1);    
       lcd.print("  "); 
       lcd.print(sensor_luminosidade); 
       lcd.print("               "); 
   break;
   
   // MENU UMIDADE EXT
   case 4:
       lcd.setCursor(0,1);    
       lcd.print("  "); 
       lcd.print(umidade_externa); 
       lcd.print(" %            "); 
   break;   
   
   // MENU PRESSAO ATMOSFERICA
   case 5:
       lcd.setCursor(0,1);    
       //lcd.print("  "); 
       lcd.print(P,2); 
       lcd.print(" mb ");
       lcd.print(P*0.0295333727,2);
       lcd.println(" inHg"); 
   break;   
   
   // MENU TEMP INTERNA
   case 6:
       lcd.setCursor(0,1);
       lcd.print("  ");   
       lcd.print(temp_interna_C);
       lcd.print(" C       "); 
   break;
   
   // MENU TEMP EXTERNA
   case 7:
       lcd.setCursor(0,1);
       lcd.print("  ");   
       lcd.print(temp_externa_C);
       lcd.print(" C       "); 
   break;
   
   // MENU ACELEROMETRO
   case 8:
       lcd.setCursor(0,1);
       lcd.print(" ");   
       lcd.print(x);
       lcd.print(" "); 
       lcd.print(y);
       lcd.print(" ");
       lcd.print(z);
       lcd.print("     ");
   break;
   
   // MENU BUSSOLA
   case 9:
       lcd.setCursor(0,1);
       lcd.print(" Direcao: ");   
       lcd.print(direcao);
       lcd.println("       "); 
   break;
   
 }

 //lcd.setCursor(0,1);            // move to the begining of the second line
 processar_menu_botoes();
  
}

//////////// FUNCOES AUXILIARES ////////////////////

void ler_pressao() {
  
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      Serial.print("Temperatura Ext Barometrica: ");
      Serial.print(T,2);
      Serial.print(" deg C, ");
      Serial.print((9.0/5.0)*T+32.0,2);
      Serial.println(" deg F");
      
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.print("Pressao Absoluta: ");
          Serial.print(P,2);
          Serial.print(" mb, ");
          Serial.print(P*0.0295333727,2);
          Serial.println(" inHg");

        }
        else Serial.println("Erro ao receber leitura da pressao\n");
      }
      else Serial.println("Erro ao iniciar leitura da pressao\n");
    }
    else Serial.println("Erro ao receber leitura da temperatura\n");
  }
  else Serial.println("Erro iniciando leitura da temperatura\n");
  
}

void ler_bussola() {
  bussola.read();
  direcao = bussola.heading();
  Serial.print("Bussola Direcao: ");
  Serial.println(direcao);
}

void ler_acelerometro() {
  
  // le dados acelerometro
  bussola.read();

  x = bussola.m.x;
  y = bussola.m.y;
  z = bussola.m.z;
  
  // imprime na serial
  Serial.print("Acelerometro X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);
  
}

void ler_umidade() {
 umidade_externa = sensor_umidade_externa.readHumidity(); 
 Serial.print("Umidade: ");
 Serial.print(umidade_externa);
 Serial.println(" %");
}

void gravar_dados_sd() {
  
  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > INTEVALO_GRAVACAO_DADOS) {  // 5 segundos
    // save the last time you blinked the LED 
    previousMillis = currentMillis;  
    
      // GRAVA OS DADOS
      // open the file for write at end like the Native SD library
            if (!myFile.open("dados.csv", O_RDWR | O_CREAT | O_AT_END)) {
              sd.errorHalt("Erro ao abrir o arquivo dados.csv para escrita!");
            }
            
            
            total_coords_salvas++;
            
            
            // if the file opened okay, write to it:
            Serial.print("\nGravando dados no SD... Tempo: ");
            Serial.print((int)hours);
            Serial.print(":");
            Serial.print((int)minutes);
            Serial.print(":");
            Serial.print((int)seconds);
            Serial.print(" - Total Registros: ");
            Serial.println(total_coords_salvas);
           
            
            
            // GRAVA DADOS CSV
            // MODELO ARQUIVO
            // luminosidade, umidade, pressao, temp_interna, temp_externa,
            // acelerometro_x, acelerometro_y, acelerometro_z, bussola
            
            myFile.print((int)hours);
            myFile.print(":");
            myFile.print((int)minutes);
            myFile.print(":");
            myFile.print((int)seconds);
            myFile.print(";");
            myFile.print(sensor_luminosidade);
            myFile.print(";");
            myFile.print(umidade_externa);
            myFile.print(";");
            myFile.print(P);
            myFile.print(";");
            myFile.print(temp_interna_C);
            myFile.print(";");
            myFile.print(temp_externa_C);
            myFile.print(";");
            myFile.print(T);  // temperatura externa do barometro
            myFile.print(";");
            myFile.print(x);
            myFile.print(";");
            myFile.print(y);
            myFile.print(";");
            myFile.print(z);
            myFile.print(";");
            myFile.print(x);
            myFile.print(";");
            myFile.print(direcao);
            // fim de linha
            myFile.print("\n");
          
            // close the file:
            myFile.close();
            Serial.println("Dados gravados!\n");
          
            // re-open the file for reading:
            /*
            if (!myFile.open("test.txt", O_READ)) {
              sd.errorHalt("opening test.txt for read failed");
            }
            Serial.println("test.txt:");
          
            // read from the file until there's nothing else in it:
            int data;
            while ((data = myFile.read()) >= 0) {
              Serial.write(data);
            }
            */
            
            // close the file:
            myFile.close();
      
  }  
  
}

void tempo_ativo() {
  lcd.setCursor(9,1);
  lcd.print(hours,0);
  lcd.print(":");
  lcd.print(minutes,0);
  lcd.print(":");
  lcd.print(seconds,0);
  lcd.print("       ");
}

void processar_menu_botoes() {
   lcd_key = read_LCD_buttons();  // read the buttons
   switch (lcd_key)               // depending on which button was pushed, we perform an action
   {
     case btnRIGHT:
       {
         //lcd.print("RIGHT ");
         menu_atual++;
         if (menu_atual>menu_max) menu_atual = 0;
         break;
       }
     case btnLEFT:
       {
         //lcd.print("LEFT   ");
         menu_atual--;
         if (menu_atual<0) menu_atual = menu_max;
         break;
       }
     case btnUP:
       {
         // PROCESSA COMANDOS DE AUMENTO
         switch (menu_atual) {
        
           // MENU MISSAO
           case 2:
               segs_intervalo_fotos = segs_intervalo_fotos + 5;
           break;
         
         }
         
         break;
       }
     case btnDOWN:
       {
         // PROCESSA COMANDOS DE DECREMENTO
         switch (menu_atual) {
        
           // MENU MISSAO
           case 2:
               segs_intervalo_fotos = segs_intervalo_fotos - 5;
               if (segs_intervalo_fotos <= 0) segs_intervalo_fotos = 5;
           break;
         
         }
         
         break;
       }
     case btnSELECT:
       {
         
         // PROCESSA COMANDOS DE SELECAO
         switch (menu_atual) {
        
           // MENU MISSAO
           case 1:
               missao_ativa = !missao_ativa;
               if (missao_ativa == true) {
                 tempo_missao.start();
               } else {
                 tempo_missao.stop();
               }
           break;
         
       }
         
         break;
       }
     case btnNONE:
       {
         //lcd.print("NONE  ");
         break;
       }
   }
   delay(250);
}

// read the buttons
int read_LCD_buttons()
{
   adc_key_in = analogRead(0);      // read the value from the sensor 
   // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
   // we add approx 50 to those values and check to see if we are close
   if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
   // For V1.1 us this threshold
   if (adc_key_in < 50)   return btnRIGHT;  
   if (adc_key_in < 250)  return btnUP; 
   if (adc_key_in < 450)  return btnDOWN; 
   if (adc_key_in < 650)  return btnLEFT; 
   if (adc_key_in < 850)  return btnSELECT;  
   return btnNONE;  // when all others fail, return this...
   adc_key_in = 0;
   lcd_key = 0;
}

///////////////// FUNCOES LEITURA DE SENSORES ////////////////////////

///////////////// FUNCOES DE ATUADORES ///////////////////////////////

void ler_luminosidade() {
  sensor_luminosidade = analogRead(PINO_LUMINOSIDADE);
  Serial.print("Luminosidade: ");
  Serial.println(sensor_luminosidade);
}

void ler_temp_interna() {
  
  sensor_temp_interna.requestTemperatures();
  temp_interna_C = sensor_temp_interna.getTempCByIndex(1);
  temp_externa_C = sensor_temp_interna.getTempCByIndex(0);
  
  Serial.print("Temp. Interna: ");
  Serial.println(temp_interna_C);
  
  Serial.print("Temp. Externa: ");
  Serial.println(temp_externa_C);  
  
}

void tirar_foto_timer() {

  unsigned long currentMillis = millis();
 
  if (currentMillis - previousMillis > (segs_intervalo_fotos*1000)) {  // 1 segundo

    previousMillis = currentMillis;  
    tirar_foto();
    
  }  
  
}

void tirar_foto() {
  servo_foto.write(0);                  // sets the servo position according to the scaled value
  servo_foto.write(20);    
  delay(200); 
  servo_foto.write(0); 
}

void beep_buzzer() {
  unsigned long currentMillisBuzzer = millis();
 
  if(currentMillisBuzzer - previousMillisBuzzer > 200) {  // 1 segundo
    // save the last time you blinked the LED 
    previousMillisBuzzer = currentMillisBuzzer;  
    // if the LED is off turn it on and vice-versa:
    if (buzzer_state == 0) {
      Serial.println("BEEEEP!");
      buzzer_state = 1024;
    } else {
      buzzer_state = 0;
    }
    // set the LED with the ledState of the variable:
    analogWrite(PINO_BUZZER, buzzer_state);
  }  
}

