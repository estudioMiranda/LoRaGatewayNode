/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * Este ejemplo envía un paquete LoraWAN con los datos "payload[5]"
 * del sensor DHT11 / DHT22, usando la configuración de frecuencia 
 * y encriptación de The Things Network.
 *
 * Este usa ABP (Activación Por (by) Personalización), donde DevAddr y
 * Session keys estan preconfiguradas (diferente a OTAA, donde DevEUI 
 * application key es configurada, mientras la DevAddr y Session keys 
 * estan asignadas/generadas en el proceso over-the-air-activation. 
 *
 * Note: LoRaWAN por sub-banda duty-cycle estan limitadas. 
 * Aplica la limitación (1% en g1, 0.1% en g2), pero no la política de uso justo de TTN 
 * (que probablemente sea violado por este boceto cuando se deja funcionando durante más tiempo)!
 * 
 * Para usar este sketch/ejemplo, primero registre su aplicación y dispositivo con
 * The Things Network, para establecer o generar un DevAddr, NwkSKey y AppSKey. 
 * Cada dispositivo debe tener sus propios valores únicos para estos los campos.
 *
 * Biblioteca => MCCI LoRaWAN LMIC library (gestor de bibliotecas de arduino)
 * No olvide definir el tipo de radio BAND correctamente en la biblioteca 
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

//////////////////////////// SENSOR DHT11 / DHT22  ////////////////////////////////
// Biblioteca sensor DHT 11 / 22
#include "DHT.h"

// Tipo de sensor DHT11 / DHT22
#define DHTTYPE    DHT11
// Pin del sensor DHT
#define DHTPIN 10

DHT dht(DHTPIN, DHTTYPE);

////////////////////////////////////////////////////////////////////////

//
// Para uso normal, requerimos que edite el sketch/ejemplo para reemplazar FILLMEIN
// con valores asignados por la consola TTN. Sin embargo, para las pruebas de regresión,
// queremos poder compilar estos scripts. Las pruebas de regresión definen
// COMPILE_REGRESSION_TEST, y en ese caso definimos FILLMEIN como un
// trabajo pero con un valor inofensivo.
//
//#ifdef COMPILE_REGRESSION_TEST
//# define FILLMEIN 0
//#else
//# warning "Tu debes reemplazar el valor FILLMEIN con los valores proporcionados por tu panel de control TTN!"
//# define FILLMEIN (#No edites esto, edita las lineas que usan edit FILLMEIN)
//#endif

// LoRaWAN NwkSKey, Clave de sessión network
// Esto debería estar en big-endian (aka msb).
static const PROGMEM u1_t NWKSKEY[16] = { 0xDE, 0x11, 0x4D, 0xA5, 0x85, 0xC9, 0xA0, 0xEC, 0x68, 0x86, 0xA6, 0x8D, 0xC3, 0x4A, 0xB8, 0x27 };

// LoRaWAN AppSKey, Clave de sesión de la aplicación
// Debería estar en big-endian también (aka msb).
static const u1_t PROGMEM APPSKEY[16] = { 0xA9, 0x4E, 0x5E, 0x09, 0xF4, 0x73, 0x9F, 0xE1, 0x6B, 0xB1, 0x8B, 0x4E, 0xFE, 0xBF, 0x89, 0xE0 };

// LoRaWAN dirección del end-device (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// La biblioteca convierte la dirección al orden de bytes de red según sea necesario, por lo que debe estar en big-endian (aka msb).
static const u4_t DEVADDR = 0x260B6C32 ; // <-- Esta dirección cambia para cada nodoChange this address for every node!

// Estas devoluciones de llamada solo se utilizan en la activación por aire, 
// por lo que aquí se han dejado vacías (no podemos omitirlos por completo a menos que
// DISABLE_JOIN se estableca en arduino-lmic/project_config/lmic_project_config.h,
// de lo contrario, el enlazador dará un error).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//////////////////////////////////////////////////////////////////////////////////////////////////

// Variable donde guardamos los datos del sensor
static uint8_t payload[5];

//////////////////////////////////////////////////////////////////////////////////////////////////
static osjob_t sendjob;

// Programa TX cada tantos segundos (puede ser más largo debido al deber
// limitaciones del ciclo).
const unsigned TX_INTERVAL = 60;

// Mapeo de pines de la placa de desarrollo esta es de TTGO LoRa ESP32 OLED V.1.0
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26,33,32}
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Recibido ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Recibido "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes del payload"));
            }
            // Programa la próxima transión
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // datos recibidos en ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            Serial.print(F("Evento desconocido: "));
            Serial.println((unsigned) ev);
            break;
    }
}
/////////////////////////////////////////////////////////////////////////////////////

// Función envio de datos del sendor DHT11 / DHT22
void do_send(osjob_t* j){
    // Comprueba si hay un trabajo TX/RX en ejecución
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, sin poder enviar"));
    } else {
        // Lee la temperatura del sensor DHT11 / DHT22
        float temperature = 25.6;
        //float rHumidity = dht.readTemperature();
        Serial.print("Temperatura: "); Serial.print(temperature);
        Serial.println(" *C");
        // Ajusta el dato f2sflt16 al rango (-1 to 1)
        temperature = temperature / 100;

        // Lee la humedad del sensor DHT11 / DHT22
        float rHumidity = 50;
        //float rHumidity = dht.readHumidity();
        Serial.print("Humedad: ");
        Serial.println(rHumidity);
        // Ajusta el dato f2sflt16 al rango (-1 to 1)
        rHumidity = rHumidity / 100;

        // float -> int
        // Nota: este usa el dato sflt16 
        // (https://github.com/mcci-catena/arduino-lmic#sflt16)
        uint16_t payloadTemp = LMIC_f2sflt16(temperature);
        // int -> bytes
        byte tempLow = lowByte(payloadTemp);
        byte tempHigh = highByte(payloadTemp);
        // Coloca los bytes en la variable payload[5]
        payload[0] = tempLow;
        payload[1] = tempHigh;

        // float -> int
        uint16_t payloadHumid = LMIC_f2sflt16(rHumidity);
        // int -> bytes
        byte humidLow = lowByte(payloadHumid);
        byte humidHigh = highByte(payloadHumid);
        payload[2] = humidLow;
        payload[3] = humidHigh;

        //  Prepara la transmisión de datos en sentido ascendente en el próximo momento posible.
        //  Transmite en el puerto 1 (el primer parámetro); puede utilizar cualquier valor entre 1 y 223 (los demás están reservados).
        //  No solicita un ack (el último parámetro, si no es cero, solicita un ack de la red).
        //  Recuerde, los acks consumen muchos recursos de red; no pida un reconocimiento a menos que realmente lo necesite.
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
    }
        //  La próxima transmisión está programada después del evento TX_COMPLETE.
}
//////////////////////////////////////////////////////////////////////////////////////

void setup() {

    while (!Serial); // Espera a que el monitor serie se inicie
    Serial.begin(115200);
    delay(100);     // Codigó de ejemplo prueba RF_95
    Serial.println(F("Iniciando"));

    #ifdef VCC_ENABLE
    // Para placas de desarrollo Pinoccio Scout
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // Inicia LMIC
    os_init();
    // Resetea el estado de la MAC. Se descartan las transferencias de datos de sesión y pendientes.
    LMIC_reset();

    //  Establece los parámetros de sesión estáticos. En lugar de establecer una sesión de forma dinámica
    //  al unirse a la red, se proporcionan parámetros de sesión precalculados.
    #ifdef PROGMEM
    //  En AVR, estos valores se almacenan en la memoria flash y solo se copian en la RAM
    //  una vez. Cópielos en un búfer temporal aquí, LMIC_setSession
    //  cópielos en su propio búfer nuevamente.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
    #else
    // Si no está ejecutando un AVR con PROGMEM, simplemente use las matrices directamente
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    //  Configure los canales utilizados por Things Network, que corresponda
    //  a los valores predeterminados de la mayoría de las puertas de enlace. Sin esto, solo tres bases
    //  se utilizan canales de la especificación LoRaWAN, que ciertamente
    //  funciona, por lo que es bueno para depurar, pero puede sobrecargar esas
    //  frecuencias, así que asegúrese de configurar el rango de frecuencia completo de
    //  su red aquí (a menos que su red los configure automáticamente).
    //  La configuración de canales debería ocurrir después de LMIC_setSession, ya que
    //  configura el conjunto de canales mínimo. El LMIC no te deja cambiar
    //  las tres configuraciones básicas, pero las mostramos aquí.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN define un canal adicional a 869.525Mhz usando SF9 para la clase B
    // ranuras de ping de los dispositivos. LMIC no tiene una manera fácil de definir establecer esta.
    // La frecuencia y el soporte para la clase B son irregulares y no han sido probados, por lo que
    // la frecuencia no está configurada aquí.
    #elif defined(CFG_us915) || defined(CFG_au915)
    //  Los canales NA-US y AU 0-71 se configuran automáticamente
    //  pero solo un grupo de 8 debería (una subbanda) debería estar activo
    //  TTN recomienda la segunda subbanda, 1 en un recuento basado en cero.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #elif defined(CFG_as923)
    //  Configure los canales utilizados en su país. Solo dos están definidos por defecto,
    //  y no se pueden cambiar. Utilice BAND_CENTI para indicar el ciclo de trabajo del 1%.
    // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

    // ... extra definitions for channels 2..n here
    #elif defined(CFG_kr920)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    // ... definiciones adicionales para los canales 3..n aquí.
    #elif defined(CFG_in866)
    // Configure los canales utilizados en su país. Tres están definidos por defecto,
    // y no se pueden cambiar. El ciclo de trabajo no importa, pero es convencional
    // BAND_MILLI.
    // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    #else
    # error Region not supported
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN usa SF9 para RX2.
    LMIC.dn2Dr = DR_SF9;

    // Establece la velocidad de datos y la potencia de transmisión para el enlace ascendente.
    LMIC_setDrTxpow(DR_SF7,14);

    // Inica el envio
    do_send(&sendjob);
}

void loop() {
    unsigned long now;
    now = millis();
    if ((now & 512) != 0) {
      digitalWrite(13, HIGH);
    }
    else {
      digitalWrite(13, LOW);
    }

    os_runloop_once();

}
