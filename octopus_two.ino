/*********************************************************
* PROJETO ESTATOSFERA UNIFEOB - www.unifeob.edu.br
* Este codigo fonte faz parte do computador de bordo
* Octopus One e seu codigo fonte pode ser usado livremente
* Prof. Resp.: Rodrigo Santa Maria
* rodrigo.maria@unifeob.edu.br e demais alunos/desenvolvedores
* Ultima atualizacao: 17/06/2015
*********************************************************/
#include <StopWatch.h>
#include <SPI.h>
#include <SD.h>
#include "DHT.h"
#include <Wire.h>
#include <HMC5883L.h>
#include <SFE_BMP180.h>
/*******************************************************/

long int tempo;  // tempo millis()
StopWatch tempo_missao;  // tempo missao

// definindo valores padra
#define INTEVALO_GRAVACAO_DADOS 5000 // 5 segundos
#define PINO_LUMINOSIDADE       A3  // analog 6
#define PINO_UMIDADE_TEMP_INT   9   // digital 5
#define PINO_UMIDADE_TEMP_EXT   8   // digital 6

// CONTROLES E VARIAVEIS GLOBAIS
bool       missao_ativa = false;
int        segs_intervalo_fotos = 30; // em segundos
int        buzzer_state = 0;
long       previousMillis = 0;
long       previousMillisBuzzer = 0;  
float      temp_interna_C;
float      temp_externa_C;
float      umidade_externa;
float      umidade_interna;
int        sensor_luminosidade;
int        x, y, z;     //triple axis data acelerometro
float      direcao;
HMC5883L   bussola;
SFE_BMP180 pressure;
char       status;      // sensor pressao
double     T,P,p0,a;    // temp e pressao
const int  chipSelect = 53;
float      hours, minutes, seconds;
long int   total_coords_salvas=0;

File myFile;

// Umidade Externa
DHT sensor_umidade_externa(PINO_UMIDADE_TEMP_EXT, DHT11);
DHT sensor_umidade_interna(PINO_UMIDADE_TEMP_INT, DHT11);


void setup()
{

 // inicia serial
 Serial.begin(9600);
 while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
 }
 Serial.println("Projeto Estratosfera Carregado!");

 
 // inicia bussola
 Wire.begin();
 bussola = HMC5883L();
 bussola.SetScale(1.3); // Set the scale of the compass.
 bussola.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous

 // inicia pressao
 pressure.begin();

 // inicia SD
 if (!SD.begin(4)) {
    Serial.println("Erro ao iniciar SD! :(");
    return;
  } 
  Serial.println("SD iniciado com sucesso!");


  // INICIA MISSAO
  inicia_para_missao();
  
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
         
         // TEMPERATURA INTERNA
         ler_temp_umi_int_ext();
          
         // LUMINOSIDADE
         ler_luminosidade(); 
         
         // ACELEROMETRO
         ler_acelerometro();
         
         // BUSSOLA
         ler_bussola();
         
         // ler pressao
         ler_pressao();
         
         // POR FIM, GRAVA DADOS
         gravar_dados_sd();

         delay (50);
 
  }  


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

  MagnetometerRaw raw = bussola.ReadRawAxis();
  MagnetometerScaled scaled = bussola.ReadScaledAxis();
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  float declinationAngle = 0.0457;
  heading += declinationAngle;
  if(heading < 0) heading += 2*PI;
  if(heading > 2*PI) heading -= 2*PI;
  float headingDegrees = heading * 180/M_PI; 
  direcao = headingDegrees;
  
  Serial.print("Bussola Direcao: ");
  Serial.println(direcao);
  
}

void ler_acelerometro() {

  MagnetometerRaw raw = bussola.ReadRawAxis();
   x = raw.XAxis;
   y = raw.YAxis;
   z = raw.ZAxis;
  
  // imprime na serial
  Serial.print(" X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);
  
}

void gravar_dados_sd() {
  
  unsigned long currentMillis = millis();
 
  if (currentMillis - previousMillis > INTEVALO_GRAVACAO_DADOS) {  // 5 segundos
    
    // save the last time you blinked the LED 
    previousMillis = currentMillis;
    
      // GRAVA OS DADOS
      // open the file for write at end like the Native SD library
            myFile = SD.open("dados.csv", FILE_WRITE);
            if (!myFile) {
               Serial.print("\nErro ao gravar dados SD!\n");
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
            myFile.print(umidade_interna);
            myFile.print(";");            
            myFile.print(umidade_externa);
            myFile.print(";");
            myFile.print(P);  // pressao
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
            myFile.print(direcao);  // bussola
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


///////////////// FUNCOES LEITURA DE SENSORES ////////////////////////

///////////////// FUNCOES DE ATUADORES ///////////////////////////////

void inicia_para_missao() {
  
  missao_ativa = !missao_ativa;
  if (missao_ativa == true) {
      tempo_missao.start();
  } else {
      tempo_missao.stop();
  }
  
}

void ler_luminosidade() {
  sensor_luminosidade = analogRead(PINO_LUMINOSIDADE);
  Serial.print("Luminosidade: ");
  Serial.println(sensor_luminosidade);
}

void ler_temp_umi_int_ext() {
  
  umidade_externa  = sensor_umidade_externa.readHumidity();
  umidade_interna  = sensor_umidade_interna.readHumidity();
  temp_externa_C   =  sensor_umidade_externa.readTemperature(); // T;
  temp_interna_C   = sensor_umidade_interna.readTemperature();
  
  
  Serial.print("Temp. Interna: ");
  Serial.println(temp_interna_C);

  Serial.print("Umidade Interna: ");
  Serial.print(umidade_interna);
  Serial.println(" %");
  
  Serial.print("Temp. Externa: ");
  Serial.println(temp_externa_C);  
  
  Serial.print("Umidade Externa: ");
  Serial.print(umidade_externa);
  Serial.println(" %");

}

