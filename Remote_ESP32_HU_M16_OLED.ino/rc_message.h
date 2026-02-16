#ifndef RC_MESSAGE_H
#define RC_MESSAGE_H

// RC message format (packed so wire layout is stable)
typedef struct __attribute__((packed)) {
  float roll_deg;     // desired roll angle in degrees
  float pitch_deg;    // desired pitch angle in degrees
  float yaw_rate_dps; // desired yaw rate in deg/s
  float throttle;     // 0.0 .. 1.0 normalized
  uint32_t seq;
  uint8_t arm;        // 0/1 arm flag
} rc_message_t;

#endif // RC_MESSAGE_H