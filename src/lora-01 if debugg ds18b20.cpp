#include "LoRaWan_APP.h"
#include "Arduino.h"

//la température sonde DS18B20    EXterne au boitier
#include "OneWire.h"
#include "DallasTemperature.h"
OneWire oneWire(ADC3);
DallasTemperature ds(&oneWire);

// Paramètre debugg pour contrôler les print
int debugg = 0; //1 // Mettre à 0 pour désactiver les print, 1 pour les activer

/* OTAA para c'est ce OTAA paramêtre qui est utilisé */
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

//lora-01
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0xB1, 0xE8 };
uint8_t appKey[] = { 0x64, 0x15, 0x08, 0x32, 0x47, 0x90, 0x93, 0x43, 0x1D, 0x26, 0xB5, 0xFD, 0x34, 0xF8, 0xF0, 0xB8 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };

uint32_t devAddr =  ( uint32_t )0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 900000;  // 900000=15mn

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;

/* Number of trials to transmit the frame */
uint8_t confirmedNbTrials = 4;  // Ajouté ici

/*!
   \brief   Prepares the payload of the frame
*/

static void prepareTxFrame(uint8_t port)
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(3000);

  // Mesure avec la sonde DS18B20
  if (debugg == 1) {
    Serial.println("ds18b20 test!");
  }
  delay(1000);  
  ds.begin();          // sonde activée
  delay(1000);
  ds.requestTemperatures();
  int tDs = ds.getTempCByIndex(0);
  int8_t tDs_byte = (tDs + 35) * 2;  // Conversion de la température

  if (debugg == 1) {
    Serial.print(tDs);
    Serial.println(" C DS18B20  ");
    Serial.print(tDs_byte, HEX);
    Serial.println(" Byte DS18B20   ");
  }

  // Lecture de la tension batterie
  uint16_t batteryVoltage = getBatteryVoltage();
  
  if (debugg == 1) {
    Serial.print(batteryVoltage);
    Serial.println(" mV ");
  }

  appDataSize = 4;  // Mettre à jour la taille du tableau d'octets envoyé
  
  // Envoi de la tension batterie et de la température DS18B20
  appData[0] = (uint8_t)(batteryVoltage >> 8);
  appData[1] = (uint8_t)batteryVoltage;
  appData[2] = (uint8_t)(tDs_byte >> 8);
  appData[3] = (uint8_t)tDs_byte;

  digitalWrite(Vext, HIGH);
}

////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);

#if(AT_SUPPORT)
  enableAt();
#endif
  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();

  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);  // Activer l'alimentation des capteurs branchés sur Vext
  delay(500);

  prepareTxFrame(appPort);  // Appel de la fonction pour préparer les données
  digitalWrite(Vext, HIGH); // Désactiver les capteurs branchés sur Vext
}

////////////////////////////////////////////////////////////

void loop()
{
  switch (deviceState)
  {
    case DEVICE_STATE_INIT:
    {
#if(LORAWAN_DEVEUI_AUTO)
      LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
      getDevParam();
#endif
      printDevParam();
      LoRaWAN.init(loraWanClass, loraWanRegion);
      deviceState = DEVICE_STATE_JOIN;
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      prepareTxFrame(appPort);
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      LoRaWAN.sleep();
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}
