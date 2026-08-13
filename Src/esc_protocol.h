#ifndef ESC_PROTOCOL_H
#define ESC_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define ESC_PROTOCOL_VERSION 5U
#define ESC_FRAME_SIZE       64U
#define ESC_FRAME_START      0xABCDU

/* Jenis pesan host -> STM32. Semua frame command berukuran tetap 64 byte. */
typedef enum {
    ESC_MSG_HELLO            = 1,
    ESC_MSG_CONTROL          = 2,
    ESC_MSG_PID_CONFIG       = 3,
    ESC_MSG_TELEMETRY_CONFIG = 4,
    ESC_MSG_SAVE_EEPROM      = 5,
    ESC_MSG_LOAD_EEPROM      = 6,
    ESC_MSG_ZERO_POSITION    = 7,
    ESC_MSG_DISARM           = 8,
    ESC_MSG_REQUEST_CONFIG   = 9,
    ESC_MSG_MOTOR_CONFIG     = 10,
    ESC_MSG_REQUEST_MOTOR_CONFIG = 11,
    ESC_MSG_HOMING_CONFIG    = 12,
    ESC_MSG_REQUEST_HOMING_CONFIG = 13,
    ESC_MSG_START_HOMING     = 14,
    ESC_MSG_ABORT_HOMING     = 15,
    ESC_MSG_START_SENSOR_AUTODETECT = 16,
    ESC_MSG_START_SENSOR_MANUAL_CAL = 17,
    ESC_MSG_ABORT_SENSOR_CAL = 18,
    ESC_MSG_REQUEST_SENSOR_CAL = 19,
    /* Menghapus proof-of-calibration (encoder_calibrated/hall_calibrated) dan
     * LUT/sequence terkait untuk motor terpilih, kembali ke default pabrik,
     * TANPA menyentuh sensor_type/CPR/offset/inverted. aux_value adalah bitmask:
     * bit0 = reset Hall LUT, bit1 = reset Encoder sequence/CPR-proof. flags boleh
     * memakai ESC_FLAG_SAVE_AFTER_APPLY agar hasil reset langsung ditulis EEPROM. */
    ESC_MSG_RESET_SENSOR_CAL = 20,
    ESC_MSG_FOC_ADV_CONFIG   = 21, /* VESC MTPA/FW + advanced outer-loop options */
    ESC_MSG_REQUEST_FOC_ADV_CONFIG = 22,
    ESC_MSG_TELEMETRY        = 0x80
} EscMessageType;

/* Motor target pada command parameter. */
typedef enum {
    ESC_MOTOR_LEFT  = 0,
    ESC_MOTOR_RIGHT = 1,
    ESC_MOTOR_BOTH  = 2
} EscMotorSelect;

/* Mode runtime. SPD/POS adalah outer loop VESC-style yang menghasilkan target Iq. */
typedef enum {
    ESC_MODE_OPEN = 0,
    ESC_MODE_VLT  = 1,
    ESC_MODE_SPD  = 2,
    ESC_MODE_TRQ  = 3,
    ESC_MODE_POS  = 4,
    /* VESC COMM_SET_DUTY gets its own runtime mode. Keep legacy values stable. */
    ESC_MODE_DUTY = 5,
    /* V18: do not collapse VESC brake / handbrake into signed torque mode. */
    ESC_MODE_BRAKE = 6,
    ESC_MODE_HANDBRAKE = 7
} EscControlMode;

/* Error code motor yang dikirim pada FeedbackFrame.error_left/right.
 * 0..7 dipertahankan untuk diagnostic legacy FOC. Sensor-specific code memakai
 * range terpisah agar GUI/ROS dapat membedakan Hall dan Encoder dari EEPROM. */
typedef enum {
    ESC_MOTOR_ERROR_NONE = 0,
    ESC_MOTOR_ERROR_HALL_NOT_DETECTED = 0x10,
    ESC_MOTOR_ERROR_HALL_LUT_INVALID = 0x11,
    ESC_MOTOR_ERROR_ENCODER_NOT_CALIBRATED = 0x20,
    ESC_MOTOR_ERROR_ENCODER_NO_SIGNAL = 0x21,
    ESC_MOTOR_ERROR_ENCODER_SIGNAL_INVALID = 0x22,
    ESC_MOTOR_ERROR_CONTROL_ISR_OVERRUN = 0x30,
    ESC_MOTOR_ERROR_ABS_OVER_CURRENT = 0x31
} EscMotorErrorCode;

/* Loop PID yang dapat dituning dari interface. */
typedef enum {
    ESC_PID_CURRENT_D = 0,  /* Id */
    ESC_PID_TORQUE_Q  = 1,  /* Iq; ini adalah loop TRQ pada algoritma asli */
    ESC_PID_SPEED     = 2,
    ESC_PID_POSITION  = 3
} EscPidLoop;

/* Halaman telemetry. Interface hanya meminta data yang sedang diamati. */

/* Status state-machine homing mekanik. */
typedef enum {
    ESC_HOMING_IDLE = 0,
    ESC_HOMING_SEARCHING = 1,       /* legacy/single-stop search */
    ESC_HOMING_HOMED = 2,
    ESC_HOMING_TIMEOUT = 3,
    ESC_HOMING_ABORTED = 4,
    ESC_HOMING_FAULT = 5,
    ESC_HOMING_SYNC_ELECTRICAL = 6,
    ESC_HOMING_CAL_RIGHT = 7,
    ESC_HOMING_CAL_LEFT = 8,
    ESC_HOMING_RETURN_CENTER = 9,
    ESC_HOMING_READY = 10
} EscHomingState;

typedef enum {
    ESC_TELEM_BASIC  = 0,
    ESC_TELEM_FOC    = 1,
    ESC_TELEM_PID    = 2,
    ESC_TELEM_RAW    = 3,
    ESC_TELEM_CONFIG = 4,  /* one-shot response untuk membaca gain/limit aktif */
    ESC_TELEM_MOTOR_CONFIG = 5, /* one-shot arah motor + sensor */
    ESC_TELEM_HOMING_CONFIG = 6, /* one-shot config/status homing */
    ESC_TELEM_SENSOR_CAL = 7,    /* one-shot/progress auto detect & calibration */
    ESC_TELEM_FOC_ADV_CONFIG = 8 /* one-shot VESC MTPA/FW/outer advanced config */
} EscTelemetryPage;


/* Commissioning sensor. AUTO menggerakkan motor memakai forced electrical
 * phase + SVPWM; MANUAL membiarkan output mati dan hanya merekam sensor. */
typedef enum {
    ESC_SENSOR_CAL_IDLE = 0,
    ESC_SENSOR_CAL_RUNNING = 1,
    ESC_SENSOR_CAL_SUCCESS = 2,
    ESC_SENSOR_CAL_FAILED_SEQUENCE = 3,
    ESC_SENSOR_CAL_FAILED_TIMEOUT = 4,
    ESC_SENSOR_CAL_OVERCURRENT = 5,
    ESC_SENSOR_CAL_ABORTED = 6,
    ESC_SENSOR_CAL_PERSIST_FAILED = 7
} EscSensorCalibrationState;

/* Detail result_code pada EscTelemetrySensorCalibration.result_code. */
#define ESC_SENSOR_CAL_RESULT_NO_MOTION             7U
#define ESC_SENSOR_CAL_RESULT_CONTROL_ISR_OVERRUN   8U
#define ESC_SENSOR_CAL_RESULT_START_REJECTED        9U

typedef enum {
    ESC_SENSOR_CAL_METHOD_NONE = 0,
    ESC_SENSOR_CAL_METHOD_AUTO = 1,
    ESC_SENSOR_CAL_METHOD_MANUAL = 2
} EscSensorCalibrationMethod;

#define ESC_FLAG_ARM              (1U << 0) /* legacy: ARM kedua motor */
#define ESC_FLAG_SAVE_AFTER_APPLY (1U << 1)
#define ESC_FLAG_OPEN_PARAMS_VALID (1U << 2)
#define ESC_FLAG_OPEN_RESTART      (1U << 3)
/* v11.9 extension tanpa mengubah frame/protocol version. Jika salah satu bit
 * side-arm dipakai, bit legacy ARM diabaikan dan request menjadi independen. */
#define ESC_FLAG_ARM_LEFT          (1U << 4)
#define ESC_FLAG_ARM_RIGHT         (1U << 5)

/* aux_value bitmask untuk ESC_MSG_RESET_SENSOR_CAL. */
#define ESC_RESET_SENSOR_HALL     (1U << 0)
#define ESC_RESET_SENSOR_ENCODER  (1U << 1)

typedef enum {
    ESC_ARM_REJECT_NONE = 0,
    ESC_ARM_REJECT_UNSAFE_SETPOINT = 1,
    ESC_ARM_REJECT_SENSOR_OR_FOC = 2,
    ESC_ARM_REJECT_CALIBRATION_BUSY = 3,
    ESC_ARM_REJECT_ALIGNMENT_BUSY = 4,
    ESC_ARM_REJECT_LINK_TIMEOUT = 5,
    ESC_ARM_REJECT_HOMING_BUSY = 6,
    ESC_ARM_REJECT_CONTROL_OVERRUN = 7,
    ESC_ARM_REJECT_CURRENT_OFFSET_NOT_READY = 8
} EscArmRejectReason;
#define ESC_STATUS_LINK_OK         (1U << 0)
#define ESC_STATUS_ARMED           (1U << 1)
#define ESC_STATUS_EEPROM_OK       (1U << 2)
#define ESC_STATUS_TIMEOUT         (1U << 3)
#define ESC_STATUS_EEPROM_VERIFIED (1U << 4)
#define ESC_STATUS_SETTINGS_DIRTY   (1U << 5)
#define ESC_STATUS_HOMING_ACTIVE    (1U << 6)
#define ESC_STATUS_HOMING_COMPLETE  (1U << 7)

#if defined(__GNUC__)
#define ESC_PACKED __attribute__((packed))
#else
#define ESC_PACKED
#pragma pack(push, 1)
#endif

/*
 * Command 64 byte. Gain PID dikirim sebagai Q16.16 sehingga host dapat
 * mengirim nilai pecahan tanpa float pada STM32. Untuk loop FOC lama,
 * nilai dikonversi ke gain raw uint16 yang sudah dipakai algoritma asli.
 */
typedef struct ESC_PACKED {
    uint16_t start;
    uint8_t  version;
    uint8_t  type;
    uint16_t sequence;
    uint8_t  flags;
    uint8_t  motor;
    uint8_t  mode_left;
    uint8_t  mode_right;
    uint8_t  loop;
    uint8_t  telemetry_page;
    uint16_t telemetry_rate_hz;
    int32_t  setpoint_left;
    int32_t  setpoint_right;
    int32_t  kp_q16;
    int32_t  ki_q16;
    int32_t  kd_q16;
    int32_t  i_limit_q16;
    int32_t  output_min;
    int32_t  output_max;
    int32_t  position_min;
    int32_t  position_max;
    uint32_t telemetry_mask;
    uint16_t config_word; /* motor config word, atau 0 untuk command lain */
    uint16_t aux_value;   /* deadband POS atau encoder CPR sesuai message type */
    uint16_t checksum;
} EscCommandFrame;

/* Feedback selalu 64 byte. Isi payload ditentukan oleh field page. */
typedef struct ESC_PACKED {
    uint16_t start;
    uint8_t  version;
    uint8_t  type;
    uint16_t sequence;
    uint8_t  page;
    uint8_t  status;
    uint32_t uptime_ms;
    uint8_t  mode_left;
    uint8_t  mode_right;
    uint8_t  error_left;
    uint8_t  error_right;
    uint8_t  payload[46];
    uint16_t checksum;
} EscFeedbackFrame;

/* Payload halaman BASIC (46 byte, memenuhi payload frame). */
typedef struct ESC_PACKED {
    int32_t position_left;
    int32_t position_right;
    int32_t setpoint_left;
    int32_t setpoint_right;
    int16_t speed_left;
    int16_t speed_right;
    int16_t battery_centi_volt;
    int16_t temperature_deci_c;
    int16_t dc_current_left_centi_amp;
    int16_t dc_current_right_centi_amp;
    int16_t command_left;
    int16_t command_right;
    uint16_t link_age_ms;
    uint16_t telemetry_rate_hz;
    /* v11.9: status ARM per motor dan alasan request terakhir ditolak. Spare
     * payload BASIC dipakai sehingga frame tetap 64 byte dan host lama aman. */
    uint8_t arm_state_mask;       /* bit0 LEFT armed, bit1 RIGHT armed */
    uint8_t arm_request_mask;     /* request host yang masih aktif */
    uint8_t output_enable_mask;   /* gate PWM efektif sebelum current chopping */
    uint8_t arm_reject_left;
    uint8_t arm_reject_right;
    uint8_t motor_isr_peak_load_pct; /* max cycle sejak clear terhadap budget 16 kHz */
    /* v11.11 runtime survival diagnostics. Payload BASIC tetap tepat 46 byte. */
    uint8_t motor_isr_overrun_count; /* saturasi 0..255 untuk diagnosis */
    uint8_t boot_cpu_fault_code; /* 0 normal, 1..5 lihat stm32f1xx_it.h */
    uint8_t boot_reset_flags;    /* bit0 PIN,1 POR,2 SW,3 IWDG,4 WWDG,5 LPWR */
    uint8_t uart_recovery_flags; /* bit0 TX DMA error pernah terjadi, bit1 RX DMA direstart */
} EscTelemetryBasic;

_Static_assert(sizeof(EscTelemetryBasic) == 46U, "BASIC telemetry payload must stay 46 bytes");

/* Payload halaman FOC (46 byte, memenuhi seluruh payload frame). */
typedef struct ESC_PACKED {
    int16_t id_left;
    int16_t iq_left;
    int16_t id_right;
    int16_t iq_right;
    int16_t phase_a_left;
    int16_t phase_b_left;
    int16_t phase_b_right;
    int16_t phase_c_right;
    int16_t dc_link_left;
    int16_t dc_link_right;
    int16_t duty_u_left;
    int16_t duty_v_left;
    int16_t duty_w_left;
    int16_t duty_u_right;
    int16_t duty_v_right;
    int16_t duty_w_right;
    int16_t electrical_angle_left;
    int16_t electrical_angle_right;
    int16_t speed_left;
    int16_t speed_right;
    uint16_t hall_bits;
    /* Arus motor/DC-link engineering centi-amp. Empat byte sisa payload FOC
     * dipakai agar Iq/Id/Hall/Imotor dapat diplot pada page yang sama. */
    int16_t dc_current_left_centi_amp;
    int16_t dc_current_right_centi_amp;
} EscTelemetryFoc;

/* Payload halaman PID (44 byte). */
typedef struct ESC_PACKED {
    uint8_t loop;
    uint8_t reserved;
    int32_t setpoint_left;
    int32_t setpoint_right;
    int32_t measured_left;
    int32_t measured_right;
    int32_t error_left;
    int32_t error_right;
    int16_t p_left;
    int16_t i_left;
    int16_t d_left;
    int16_t output_left;
    int16_t p_right;
    int16_t i_right;
    int16_t d_right;
    int16_t output_right;
    uint16_t antiwindup_flags;
} EscTelemetryPid;

/* Payload raw ADC/Hall untuk debugging dan kalibrasi. */
typedef struct ESC_PACKED {
    int16_t adc_phase_a_left;
    int16_t adc_phase_b_left;
    int16_t adc_phase_b_right;
    int16_t adc_phase_c_right;
    int16_t adc_dc_left;
    int16_t adc_dc_right;
    int16_t adc_battery;
    int16_t adc_temperature;
    uint8_t hall_left;
    uint8_t hall_right;
    uint16_t pwm_period;
    uint32_t main_loop_counter;
    uint32_t valid_frames;
    uint32_t bad_frames;
    uint32_t reconnect_counter;
    int16_t core_speed_left;
    int16_t core_speed_right;
    uint16_t eeprom_crc;
    uint16_t eeprom_generation;
    uint16_t eeprom_verify_failures;
} EscTelemetryRaw;

/* Payload one-shot untuk membaca tuning yang sedang aktif dari STM32. */
typedef struct ESC_PACKED {
    uint8_t motor;
    uint8_t loop;
    int32_t kp_q16;
    int32_t ki_q16;
    int32_t kd_q16;
    int32_t i_limit_q16;
    int32_t output_min;
    int32_t output_max;
    int32_t position_min;
    int32_t position_max;
    uint16_t position_deadband_ticks;
    uint16_t eeprom_crc;
    uint16_t eeprom_generation;
    uint16_t eeprom_verify_failures;
    uint8_t eeprom_verified;
    uint8_t settings_dirty;
    uint8_t reserved2;
    uint8_t reserved3;
} EscTelemetryConfig;

/* Payload one-shot konfigurasi/status homing mekanik berbasis kenaikan arus. */
typedef struct ESC_PACKED {
    uint16_t left_current_threshold_centi_amp;
    uint16_t right_current_threshold_centi_amp;
    int16_t  left_search_command;
    int16_t  right_search_command;
    uint16_t timeout_ms;
    uint16_t debounce_ms;
    uint8_t  left_state;
    uint8_t  right_state;
    uint8_t  active_mask;
    uint8_t  reserved0;
    int16_t  left_current_centi_amp;
    int16_t  right_current_centi_amp;
    int16_t  left_peak_current_centi_amp;
    int16_t  right_peak_current_centi_amp;
    int32_t  left_position;
    int32_t  right_position;
    uint16_t eeprom_crc;
    uint16_t eeprom_generation;
    uint16_t eeprom_verify_failures;
    uint8_t  eeprom_verified;
    uint8_t  settings_dirty;
    /* EEPROM v8: pilih per motor apakah hard-stop homing dijalankan otomatis
     * setelah power-on/boot. */
    uint8_t  auto_home_boot_left;
    uint8_t  auto_home_boot_right;
} EscTelemetryHomingConfig;

/* Payload progress/result commissioning Hall/Encoder (tepat 46 byte).
 * v11.7 menukar debug-only delta/open angle dengan proof urutan encoder 4-state
 * LEFT/RIGHT agar GUI dapat menampilkan hasil yang benar-benar tersimpan.
 * Saat RUNNING/gagal, slot sequence bernilai 0xFF belum teramati pada sesi ini. */
typedef struct ESC_PACKED {
    uint8_t state;
    uint8_t method;
    uint8_t motor;
    uint8_t sensor_type;
    uint16_t progress_permille;
    uint16_t elapsed_ms;
    uint16_t samples;
    uint16_t invalid_samples;
    uint8_t hall_sequence_left[6];
    uint8_t hall_sequence_right[6];
    uint8_t encoder_sequence_left[4];
    uint8_t encoder_sequence_right[4];
    uint8_t hall_lut_valid_left;
    uint8_t hall_lut_valid_right;
    uint8_t encoder_sequence_valid_left;
    uint8_t encoder_sequence_valid_right;
    uint16_t encoder_cpr_left;
    uint16_t encoder_cpr_right;
    uint16_t encoder_offset_left_deg;
    uint16_t encoder_offset_right_deg;
    uint8_t result_code;
    /* bit0/1=encoder calibrated L/R, bit2/3=sensor inverted L/R,
     * bit4/5=Hall sequence detected/calibrated L/R,
     * bit6=sequence payload adalah kandidat sesi, bukan LUT aktif. */
    uint8_t calibration_flags;
} EscTelemetrySensorCalibration;


/* Advanced VESC control page, exactly 46 bytes. Engineering-unit transport is
 * used so the GUI does not need board ADC scaling details. */
typedef struct ESC_PACKED {
    uint8_t  motor;
    uint8_t  mtpa_mode;
    uint16_t flags; /* bit0: speed PID braking allowed */
    uint32_t flux_linkage_uwb;
    int32_t  ld_lq_diff_uh;
    uint16_t fw_current_centi_amp;
    uint16_t fw_duty_start_permille;
    uint16_t fw_ramp_time_ms;
    uint16_t fw_q_current_factor_permille;
    uint16_t fw_backoff_permille;
    uint16_t speed_kd_filter_permille;
    int32_t  speed_min_erpm;
    int32_t  speed_ramp_erpms_s;
    int32_t  pos_kd_proc_q16;
    uint16_t pos_kd_filter_permille;
    uint32_t pos_gain_dec_ticks;
    int16_t  actual_fw_current_centi_amp;
    int16_t  actual_mtpa_id_centi_amp;
} EscTelemetryFocAdvanced;

_Static_assert(sizeof(EscTelemetryFocAdvanced) == 46U, "FOC advanced telemetry must be 46 bytes");

/* Payload one-shot konfigurasi arah motor dan sensor. */
typedef struct ESC_PACKED {
    uint16_t left_config_word;
    uint16_t right_config_word;
    uint16_t left_encoder_cpr;
    uint16_t right_encoder_cpr;
    int32_t  left_sensor_position;
    int32_t  right_sensor_position;
    uint8_t  left_sensor_encoding;
    uint8_t  right_sensor_encoding;
    uint16_t eeprom_crc;
    uint16_t eeprom_generation;
    uint16_t eeprom_verify_failures;
    uint8_t  eeprom_verified;
    uint8_t  settings_dirty;
} EscTelemetryMotorConfig;

#if !defined(__GNUC__)
#pragma pack(pop)
#endif

/* CRC16-CCITT-FALSE untuk 62 byte pertama frame. */
uint16_t EscProtocol_Crc16(const uint8_t *data, uint16_t length);

/* Mulai RX circular DMA USART3. */
void EscProtocol_StartRx(void);

/* Poll RX circular DMA serta recover RX error tanpa HAL UART state-machine. */
void EscProtocol_ServiceIo(void);

/* Eksekusi command queue di main-loop; operasi berat tidak dijalankan dari IRQ. */
void EscProtocol_ServiceCommands(void);
/* Bounded variant untuk scheduler bare-metal: mencegah burst command membuat
 * telemetry tertunda. Return jumlah frame yang dieksekusi. */
uint8_t EscProtocol_ServiceCommandsBudget(uint8_t max_frames);

/* Dipanggil dari main-loop untuk memproses byte baru dari circular DMA. */
void usart3_rx_check(void);

/* Mengirim satu feedback frame bila DMA TX sedang bebas. */
bool EscProtocol_TxReady(void);
bool EscProtocol_SendFeedback(const EscFeedbackFrame *frame);
/* Handler direct DMA1 Channel2 TX; sangat singkat dan tidak memanggil HAL UART. */
void EscProtocol_TxDmaIrqHandler(void);

/* Statistik parser untuk telemetry/debug. */
uint32_t EscProtocol_GetValidFrameCount(void);
uint32_t EscProtocol_GetBadFrameCount(void);
uint32_t EscProtocol_GetTxDmaErrorCount(void);
uint32_t EscProtocol_GetRxDmaRecoveryCount(void);

#endif /* ESC_PROTOCOL_H */
