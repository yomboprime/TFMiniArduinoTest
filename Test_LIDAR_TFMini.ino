/*
 * TFMini LIDAR Sensor test skecth for ESP32
 * 
 * Juan Jose Luna Espinosa 2018
 * https://github.com/yomboprime/TFMiniArduinoTest
 * 
 * Released under public domain
 * 
 * 
 * Connections:
 * Green cable to RX in the ESP32 (UART 2, pin 16)
 * Red cable to 5V
 * Black cable to GND
 * White cable is not needed.
 * 
 * 9 byte frame of the sensor:
 * Byte 0: Always is 0x59 
 * Byte 1: Always is 0x59 
 * Byte 2: Distance in cm, low byte
 * Byte 3: Distance in cm, high byte
 * Byte 4: Signal strength, low byte
 * Byte 5: Signal strength, high byte
 * Byte 6: Reserved byte
 * Byte 7: Raw signal quality
 * Byte 8: Checksum of the 8 previous bytes.
 */

HardwareSerial Serial1( 2 );

void setup() {

  // Debug serial
  Serial.begin( 115200 );

  // Serial connected to LIDAR sensor
  Serial1.begin( 115200 );

  delay( 1000 );
  Serial.println( "Starting..." );

}

void loop() {

  // If a byte arrives from debug Serial, perform a readings per seconds test
  if ( Serial.available() > 0 ) {

    speedTest();

  }
  else {

    // Perform one distance reading and show it on Serial
  
    unsigned int distance = readLIDAR( 2000 );

    if ( distance > 0 ) {
      Serial.printf( "Distance (cm): %d\n", distance );
    }
    else {
      Serial.println( "Timeout reading LIDAR" );
    }
  }
  
}

void speedTest() {

  while ( Serial.available() > 0 ) {
    Serial.read();
  }

  Serial.printf( "\n\nPerforming speed test...\n" );

  long t0 = millis();

  #define NUM_READINGS 1000

  long accum = 0;

  for ( int i = 0; i < NUM_READINGS; i++ ) {

    accum += readLIDAR( 2000 );
  
  }

  long t1 = millis();

  float readingsPerSecond = NUM_READINGS * 1000.0f / ( t1 - t0 );

  float meanDistance = ((float)accum) / NUM_READINGS;

  Serial.println( "\n\nSpeed test:" );
  Serial.printf( "%f readings per second.\n", readingsPerSecond );
  Serial.printf( "%f mean read distance.\n", meanDistance );

  Serial.println( "\n\nHit another key to continue reading the sensor distance." );

  while ( Serial.available() == 0 ) {
    delay( 10 );
  }
  while ( Serial.available() > 0 ) {
    Serial.read();
  }

}

/*
 * This function reads the Serial1 until a valid packet is found or timeout passed.
 * Param timeout: Timeout in milliseconds.
 * Returns distance in cm or 0 if timeout happened.
 */
unsigned int readLIDAR( long timeout ) {

  unsigned char readBuffer[ 9 ];

  long t0 = millis();

  while ( Serial1.available() < 9 ) {

    if ( millis() - t0 > timeout ) {
      // Timeout
      return 0;
    }

    delay( 10 );
  }

  for ( int i = 0; i < 9; i++ ) {
    readBuffer[ i ] = Serial1.read();
  }

  while ( ! detectFrame( readBuffer ) ) {

    if ( millis() - t0 > timeout ) {
      // Timeout
      return 0;
    }

    while ( Serial1.available() == 0 ) {
      delay( 10 );
    }

    for ( int i = 0; i < 8; i++ ) {
      readBuffer[ i ] = readBuffer[ i + 1 ];
    }

    readBuffer[ 8 ] = Serial1.read();

  }

  // Distance is in bytes 2 and 3 of the 9 byte frame.
  unsigned int distance = ( (unsigned int)( readBuffer[ 2 ] ) ) |
                          ( ( (unsigned int)( readBuffer[ 3 ] ) ) << 8 );

  return distance;

}

bool detectFrame( unsigned char *readBuffer ) {

  return  readBuffer[ 0 ] == 0x59 &&
          readBuffer[ 1 ] == 0x59 &&
          (unsigned char)(
            0x59 +
            0x59 +
            readBuffer[ 2 ] + 
            readBuffer[ 3 ] + 
            readBuffer[ 4 ] +
            readBuffer[ 5 ] + 
            readBuffer[ 6 ] + 
            readBuffer[ 7 ]
          ) == readBuffer[ 8 ];
}

