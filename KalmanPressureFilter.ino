// Kalman filter variables
float x_estimated = 0;   // Initial estimated value
float P = 1;             // Initial estimation error covariance
float Q = 0.1;           // Process noise covariance
float R = 10;            // Measurement noise covariance

const int sensorPin = A0;  // Analog pin connected to the sensor
const int ledPin = 9;      // LED pin

void KalmanFilter(float z_measurement) {
  // Prediction
  float x_predicted = x_estimated;           // Predicted state estimate
  float P_predicted = P + Q;                // Predicted estimate covariance
  
  // Update
  float K = P_predicted / (P_predicted + R); // Kalman gain
  x_estimated = x_predicted + K * (z_measurement - x_predicted);  // Updated (corrected) state estimate
  P = (1 - K) * P_predicted;                 // Updated (corrected) estimate covariance
}

void setup() {
  pinMode(ledPin, OUTPUT);  // Set LED pin as an output
  Serial.begin(9600);       // Initialize serial communication for debugging purposes
}

void loop() {
  // Read sensor value
  float sensorReading = analogRead(sensorPin);

  // Applying the Kalman filter to smooth the sensor reading
  KalmanFilter(sensorReading);

  // Adjust LED brightness based on the filtered sensor value
  int brightness = map(x_estimated, 0, 1023, 0, 255); // Map filtered value to LED brightness
  analogWrite(ledPin, brightness);  // Set LED brightness

  // Print hashtags proportional to LED brightness
  for (int i = 0; i < brightness; ++i) {
    Serial.print("#");
  }
  Serial.println(); // Move to the next line after printing the hashtags


  delay(100);  // Adjust delay according to your application
}
