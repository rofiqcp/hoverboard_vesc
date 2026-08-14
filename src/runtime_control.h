#ifndef RUNTIME_CONTROL_H
#define RUNTIME_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "esc_protocol.h"
#include "motor_sensor.h"

/* Konfigurasi outer position PID Q16.16. Runtime mirrors these fields into
 * mc_configuration; the active controller output is Iq target (VESC style). */
typedef struct {
    int32_t kp_q16;
    int32_t ki_q16;
    int32_t kd_q16;
    int32_t position_min;
    int32_t position_max;
    uint16_t deadband_ticks;
} PositionPidConfig;


/* Mode FOC efektif per motor yang dibaca interrupt motor.c. */
extern volatile uint8_t controlModeLeftFoc;
extern volatile uint8_t controlModeRightFoc;

/* Setpoint runtime host dan command akhir ke FOC. */
extern volatile int32_t runtimeSetpointLeft;
extern volatile int32_t runtimeSetpointRight;
extern volatile int16_t runtimeCommandLeft;
extern volatile int16_t runtimeCommandRight;
/* Gate output per motor: bit0=LEFT, bit1=RIGHT. Global legacy `enable` tetap
 * menjadi master safety, tetapi motor.c juga wajib melihat mask ini sehingga
 * fault/ARM satu sisi tidak mematikan sisi lain. */
extern volatile uint8_t runtimeMotorEnableMask;

/* Runtime deadline monitor untuk current-control ISR 16 kHz. Jika satu siklus
 * FOC belum selesai ketika DMA ADC berikutnya tiba, counter bertambah. Fault
 * dilatch per motor agar overload tidak berubah menjadi interrupt starvation
 * yang mematikan UART/main-loop. */
extern volatile uint16_t motorControlIsrOverrunCount;
extern volatile uint8_t motorControlIsrOverrunFaultMask;
extern volatile uint8_t motorControlOvercurrentFaultMask;
extern volatile uint32_t motorControlIsrLastCycles;
extern volatile uint32_t motorControlIsrMaxCycles;
extern volatile uint32_t motorControlIsrDeadlineCycles;
/* Pengganti telemetry "peak load %" tanpa DWT->CYCCNT di ISR: skala streak
 * overrun saat ini terhadap MOTOR_ISR_OVERRUN_LIMIT. Aman dipanggil dari
 * main-loop/telemetry (bukan dari ISR 16 kHz). */
uint8_t MotorControl_GetOverrunStreakPct(void);
void MotorControl_ClearIsrOverrunFault(uint8_t clear_mask);
void MotorControl_ClearOvercurrentFault(uint8_t clear_mask);

/* Parameter runtime OPEN forced-phase dari GUI. Frekuensi adalah electrical Hz
 * dalam milli-Hz; durasi 0 berarti continuous. Mask bit0=LEFT bit1=RIGHT
 * menandakan host memakai format OPEN extended. */
extern volatile uint32_t runtimeOpenFrequencyLeftMilliHz;
extern volatile uint32_t runtimeOpenFrequencyRightMilliHz;
extern volatile int32_t runtimeOpenPhaseStepLeftQ16;
extern volatile int32_t runtimeOpenPhaseStepRightQ16;
extern volatile uint32_t runtimeOpenDurationLeftMs;
extern volatile uint32_t runtimeOpenDurationRightMs;
extern volatile uint8_t runtimeOpenParamsValidMask;

/* Override SVPWM open-loop untuk auto-detect sensor. Dibaca langsung ISR motor.c. */
extern volatile uint8_t sensorCalibrationOpenLoopMask;
/* Bit set only for VESC Hall/Encoder detect that uses forced phase + closed-loop
 * D-axis current regulation. Encoder power-on alignment remains a separate
 * voltage-override path so the two commissioning semantics can never alias. */
extern volatile uint8_t sensorCalibrationCurrentControlMask;
extern volatile uint16_t sensorCalibrationPhaseLeftQ16;
extern volatile uint16_t sensorCalibrationPhaseRightQ16;
extern volatile int16_t sensorCalibrationCurrentLeft;
extern volatile int16_t sensorCalibrationCurrentRight;
extern volatile int16_t sensorCalibrationVoltageLeft;
extern volatile int16_t sensorCalibrationVoltageRight;
/* Fast commissioning current-tracking guard. Set in the 8-kHz motor slot,
 * consumed by the slow commissioning state machine. This is NOT a runtime
 * motor fault and therefore does not trigger the 3-s buzzer fault-stop. */
extern volatile uint8_t sensorCalibrationFastCurrentFaultMask;

extern PositionPidConfig positionPidConfigLeft;
extern PositionPidConfig positionPidConfigRight;

extern MotorRuntimeConfig motorConfigLeft;
extern MotorRuntimeConfig motorConfigRight;

typedef struct {
    uint16_t current_threshold_centi_amp;
    int16_t search_command;
} HomingMotorConfig;

extern HomingMotorConfig homingConfigLeft;
extern HomingMotorConfig homingConfigRight;
extern uint16_t homingTimeoutMs;
extern uint16_t homingDebounceMs;
extern bool homingOnBootLeft;
extern bool homingOnBootRight;

typedef struct {
    int32_t right_zero_ticks; /* logical 0 degrees, refreshed by homing */
    int32_t span_ticks;       /* signed ticks from 0 to 360 degrees */
    uint8_t calibrated;       /* span is valid and persisted */
    uint8_t homed;            /* runtime absolute reference is valid this boot */
} SteeringCalibration;

extern SteeringCalibration steeringCalibrationLeft;
extern SteeringCalibration steeringCalibrationRight;

/* Board position domain: exactly one mechanical revolution / calibrated
 * steering span. 0 and 360 are distinct endpoints; no nearest-turn wrapping. */
bool RuntimeControl_PositionTargetTicks(bool left, float deg, int32_t *target_ticks);
float RuntimeControl_PositionDeg(bool left);
float RuntimeControl_PositionErrorDeg(bool left);
int32_t RuntimeControl_PositionSpanTicks(bool left);

/* Encoder AB synchronization is separate from commissioning. It aligns the
 * incremental electrical reference at zero speed without changing mechanical
 * position or the stored detect parameters. */
bool RuntimeControl_RequestEncoderSync(bool left);

/* Full mechanical calibration searches both hard stops, stores 0..360 span and
 * returns to center. Normal homing only searches the right/0-degree stop and
 * reuses the stored span. */
bool RuntimeControl_StartHomingCalibration(bool left);
bool RuntimeControl_StartHomingOne(bool left);
/* Persisted power-on homing switch. Enabling requires an already calibrated
 * mechanical span; boot only re-finds the 0-degree stop and never performs a
 * destructive full two-stop calibration automatically. */
bool RuntimeControl_SetHomingOnBoot(bool left, bool on);
bool RuntimeControl_HomingOnBoot(bool left);
bool RuntimeControl_SteeringReady(bool left);

/* Inisialisasi runtime mode, PID posisi, EEPROM dan state link. */
void RuntimeControl_Init(void);

/* Dipanggil parser untuk setiap command valid 64-byte. */
void RuntimeControl_OnCommand(const EscCommandFrame *frame);
/* VESC protocol bridge: updates one motor without disturbing the other motor's
 * requested mode/setpoint/arm state. esc_mode uses ESC_MODE_* and setpoint uses
 * the existing host-domain expected by RuntimeControl. */
void RuntimeControl_VescSetOne(bool left, uint8_t esc_mode, int32_t setpoint, bool arm);
void RuntimeControl_VescReleaseAll(void);
void RuntimeControl_VescAlive(void);

/* Asynchronous VESC Tool commissioning bridge. The VESC protocol starts one
 * blocking-style detect request, while the existing bare-metal slow loop performs
 * the sweep without an RTOS. Poll returns true only after a terminal result. */
typedef struct {
    uint8_t state;
    uint8_t result_code;
    uint8_t sensor_type;
    uint8_t hall_table[8];       /* VESC FOC format: 0..200 angle, 255 invalid */
    uint16_t encoder_cpr;
    uint16_t encoder_offset_deg;
    uint8_t encoder_inverted;
    uint8_t pole_pairs;
} RuntimeVescDetectResult;

bool RuntimeControl_VescStartSensorDetect(bool left, uint8_t sensor_type, int16_t detect_current_internal);
int16_t RuntimeControl_SensorCalibrationTargetCurrentInternal(void);
int16_t RuntimeControl_SensorCalibrationMeasuredCurrentInternal(void);
uint16_t RuntimeControl_SensorCalibrationElectricalPhaseQ16(void);
int8_t RuntimeControl_SensorCalibrationSweepDirection(void);
uint8_t RuntimeControl_SensorCalibrationForwardCycles(void);
uint8_t RuntimeControl_SensorCalibrationReverseCycles(void);
uint16_t RuntimeControl_SensorCalibrationCompletedCycles(void);
bool RuntimeControl_SensorCalibrationMotionDetected(void);
uint32_t RuntimeControl_SensorCalibrationMotionCounter(void);
uint16_t RuntimeControl_SensorCalibrationMotionAgeMs(void);
uint8_t RuntimeControl_SensorCalibrationObservedHallMask(void);
uint8_t RuntimeControl_SensorCalibrationObservedEncoderMask(void);
int32_t RuntimeControl_SensorCalibrationEncoderDelta(void);
int32_t RuntimeControl_SensorCalibrationEncoderForwardDelta(void);
uint32_t RuntimeControl_SensorCalibrationEncoderDirectionNormalScore(void);
uint32_t RuntimeControl_SensorCalibrationEncoderDirectionInvertedScore(void);
bool RuntimeControl_SensorCalibrationEncoderDirectionProved(void);
bool RuntimeControl_VescPollSensorDetect(bool left, uint8_t sensor_type, RuntimeVescDetectResult *result);
bool RuntimeControl_VescGetLastSensorDetect(bool left, RuntimeVescDetectResult *result, bool *encoder_ratio_fallback_used);

/* Dipanggil main loop setiap ~5 ms: watchdog, arming dan PID posisi. */
void RuntimeControl_UpdateSlow(uint32_t dt_ms);

/* Membentuk lalu mengirim telemetry bila periodenya tiba. */
void RuntimeControl_ServiceTelemetry(void);

/* Monotonic runtime clock with DMA-16kHz + SysTick fallback. */
uint32_t RuntimeControl_MonotonicMs(void);

/* Simpan/muat semua gain PID, limit posisi dan parameter motor ke EEPROM emulasi. */
bool RuntimeSettings_Save(void);
bool RuntimeSettings_Load(void);

/* Status runtime untuk GUI/ROS. */
bool RuntimeControl_LinkActive(void);
bool RuntimeControl_Armed(void);
bool RuntimeControl_ArmedLeft(void);
bool RuntimeControl_ArmedRight(void);
uint8_t RuntimeControl_ArmRejectLeft(void);
uint8_t RuntimeControl_ArmRejectRight(void);
/* Detailed VESC diagnostic extension helpers. Read-only and safe while running. */
uint16_t RuntimeControl_FaultStopRemainingMs(bool left);
uint8_t RuntimeControl_SensorCalibrationState(void);
uint8_t RuntimeControl_SensorCalibrationMotor(void);
uint8_t RuntimeControl_SensorCalibrationType(void);
uint8_t RuntimeControl_SensorCalibrationResultCode(void);
bool RuntimeControl_EncoderAlignmentActive(bool left);
int16_t RuntimeControl_EncoderAlignmentProbeDelta(bool left);
bool RuntimeControl_EncoderAlignmentDirectionProved(bool left);
bool RuntimeControl_EncoderElectricalReady(bool left);
uint16_t RuntimeControl_LinkAgeMs(void);
uint32_t RuntimeControl_GetReconnectCount(void);
uint16_t RuntimeSettings_GetStoredCrc(void);
uint16_t RuntimeSettings_GetGeneration(void);
uint16_t RuntimeSettings_GetVerifyFailures(void);
bool RuntimeSettings_Verified(void);
/* Error efektif mencakup diagnostic FOC + health sensor terpilih di EEPROM.
 * OPEN mode tetap boleh ARM walau error sensor ada untuk commissioning. */
uint8_t RuntimeControl_ErrorLeft(void);
uint8_t RuntimeControl_ErrorRight(void);
bool RuntimeControl_HasBlockingFault(void);
/* Buzzer fault hanya aktif untuk fault yang terjadi saat ARM/percobaan ARM;
 * readiness sensor saat DISARM tetap dilaporkan ke GUI tetapi tidak membuat buzzer terus berbunyi. */
bool RuntimeControl_ShouldSoundFaultBuzzer(void);

/* Status homing untuk GUI/telemetry. */
uint8_t RuntimeControl_HomingStateLeft(void);
uint8_t RuntimeControl_HomingStateRight(void);
uint8_t RuntimeControl_HomingActiveMask(void);
int16_t RuntimeControl_HomingPeakCurrentLeft(void);
int16_t RuntimeControl_HomingPeakCurrentRight(void);

/* Nol-kan posisi signed tanpa membuat wrap. */
void MotorOdometry_Zero(uint8_t motor_select);

#endif /* RUNTIME_CONTROL_H */
