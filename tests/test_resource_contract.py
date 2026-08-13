#!/usr/bin/env python3
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
setup = (root/'Src/setup.c').read_text()
motor = (root/'Src/motor.c').read_text()
enc = (root/'Src/left_encoder.c').read_text()
util = (root/'Src/util.c').read_text()
conf = (root/'Src/vesc_config_compat.c').read_text()
runtime = (root/'Src/runtime_control.c').read_text()
defines = (root/'Src/defines.h').read_text()

# Current-loop scan remains three dual-regular pairs. PA2/PA3 must not lengthen it.
assert setup.count('hadc1.Init.NbrOfConversion       = 3') == 1
assert setup.count('hadc2.Init.NbrOfConversion       = 3') == 1
assert 'ADC_CHANNEL_2;  /* PA2 / VESC ADC1 */' in setup
assert 'ADC_CHANNEL_3;  /* PA3 / VESC ADC2 */' in setup
assert 'adcAppDivider = 32U' in setup
assert 'ADC2->JDR1' in setup and 'ADC2->JDR2' in setup
assert 'ADC_Slow_TriggerFromCurrentISR();' in motor

# V10 restores the original board's low-side-shunt sample geometry: ADC /4,
# 7.5-cycle phase rank, TIM8 offset = 80 timer ticks. These three values are one
# physical contract and must not drift independently.
config_h = (root/'Src/config.h').read_text()
main_c_early = (root/'Src/main.c').read_text()
assert '#define ADC_CONV_CLOCK_CYCLES    ADC_CONV_TIME_7C5' in config_h
assert '#define ADC_CLOCK_DIV            4' in config_h
assert 'RCC_ADCPCLK2_DIV4' in main_c_early
assert 'LEFT_TIM->CNT' in setup and 'ADC_TOTAL_CONV_TIME' in setup

# USART3 is the permanent VESC Tool transport; USART2 must not be initialized.
assert 'USART3' in util and 'UART3_Init();' in util
assert 'GPIO_PIN_10' in setup and 'GPIO_PIN_11' in setup
assert 'USART2' not in util
assert 'USART2' not in setup
assert 'PA2' in defines and 'PA3' in defines

# LEFT encoder hardware contract: PB6/PB7 TIM4, PB5 index. No RIGHT encoder backend.
assert 'PB6 = TIM4_CH1 / A' in enc and 'PB7 = TIM4_CH2 / B' in enc and 'PB5 = Z index / EXTI5' in enc
assert 'TIM_ENCODERMODE_TI12' in enc
assert 'MotorSensor_UpdateHardwareEncoder(&motorConfigLeft' in motor
assert 'MotorSensor_UpdateHardwareEncoder(&motorConfigRight' not in motor
assert re.search(r'MotorSensor_Update\(&motorConfigRight', motor)

# RIGHT remains Hall-only after both VESC writes and EEPROM migration.
assert 'r->sensor_type = MOTOR_SENSOR_HALL_UVW;' in conf
assert 'motor_right.sensor_type = MOTOR_SENSOR_HALL_UVW;' in runtime


# USART3 has a single runtime transport. The retired custom UART implementation
# must not exist; only the small CRC/type compatibility shim may remain.
assert not (root/'Src/esc_protocol.c').exists()
assert (root/'Src/esc_protocol_compat.c').exists()

# Virtual CAN semantics are local + 1 and physical CAN is not referenced in protocol.
protocol = (root/'Src/vesc_protocol.c').read_text()
assert 'C_FORWARD_CAN' in protocol and 'C_PING_CAN' in protocol
assert 'process_ctx(d+1' in protocol.replace(' ', '')
assert 'vesc_ctx_right' in protocol and 'vesc_ctx_left' in protocol
assert 'CAN1' not in protocol

# VESC commissioning and command/fault contract.
protocol = (root/'Src/vesc_protocol.c').read_text()
esc_h = (root/'Src/esc_protocol.h').read_text()
foc_h = (root/'Src/foc_motor.h').read_text()
motor_c = (root/'Src/motor.c').read_text()
assert 'C_DETECT_ENCODER=27' in protocol and 'C_DETECT_HALL_FOC=28' in protocol
assert 'RuntimeControl_VescStartSensorDetect' in protocol
assert 'RuntimeControl_VescPollSensorDetect' in protocol
assert 'ESC_MODE_DUTY' in esc_h and 'CONTROL_MODE_DUTY' in foc_h
assert 'ESC_MODE_DUTY' in protocol
assert 'FAULT_REPORT_DURATION_MS 3000U' in runtime
assert 'rearmRelease' not in runtime
assert 'motorControlOvercurrentFaultMask' in motor_c
# V15 keeps only sample-local DC current chopping; no persistent ISR OC latch threshold.
assert 'MOTOR_OVERCURRENT_ISR_LIMIT' not in motor_c
assert 'ESC_MOTOR_ERROR_ABS_OVER_CURRENT' in protocol
# Hall table wire values use VESC's 0..200 electrical-angle convention.
assert '* 200U) / 12U' in runtime
assert '* 200U) / 12U' in conf
# VESC encoder detect must persist inferred pole pairs and keep watchdog alive.
assert 'SENSOR_CAL_VESC_ENCODER_CYCLES' in runtime
assert 'detected_pole_pairs' in runtime
assert 'EEPROM_LEFT_POLE_PAIRS' in runtime and 'EEPROM_RIGHT_POLE_PAIRS' in runtime
assert 'RuntimeControl_VescAlive();' in protocol

# V5 zero-current calibration contract: hardware ADC self-cal runs before PWM
# timers; offset acquisition uses settle + arithmetic mean, never the old
# recursive startup-dependent filter.
main_c = (root/'Src/main.c').read_text()
assert 'HAL_ADCEx_Calibration_Start(&hadc1)' in main_c
assert 'HAL_ADCEx_Calibration_Start(&hadc2)' in main_c
assert main_c.index('HAL_ADCEx_Calibration_Start(&hadc2)') < main_c.index('MX_TIM_Init();')
assert 'MotorControl_RequestCurrentOffsetCalibration();' in main_c
assert 'CURRENT_CAL_COLLECT_SAMPLES 2048U' in motor_c
assert 'CURRENT_CAL_SETTLE_SAMPLES 512U' in motor_c
assert 'currentCal.sum[n] += raw[n];' in motor_c
assert '(adc_buffer.rlA+offsetrlA)/2' not in motor_c.replace(' ', '')
# V12: current offset must be learned in the SAME active LOW-FET zero-vector
# common-mode as runtime phase-current sampling. V10's MOE-OFF calibration is
# prohibited because it created a ~700-count false phase-current step in hardware.
assert 'set_current_zero_vector_left();' in motor_c
assert 'set_current_zero_vector_right();' in motor_c
cal_begin = motor_c[motor_c.index('static void current_cal_begin_isr(void)'):motor_c.index('static bool current_cal_service_isr(void)')]
assert 'LEFT_TIM->BDTR |= TIM_BDTR_MOE;' in cal_begin
assert 'RIGHT_TIM->BDTR |= TIM_BDTR_MOE;' in cal_begin
assert 'bridge_release_left();' not in cal_begin and 'bridge_release_right();' not in cal_begin
assert 'BRIDGE_CURRENT_WARMUP_ADC_SAMPLES 1U' in motor_c
assert 'bridge_current_domain_service' in motor_c
assert 'left_domain_ready ? curL_phaA : 0' in motor_c
assert 'right_domain_ready ? curR_phaB : 0' in motor_c

# ISR overrun must be measured with the Cortex-M3 cycle counter, not by
# re-reading the circular DMA transfer-complete flag after the FOC work.
assert 'DWT->CYCCNT' in motor_c
assert 'motorControlIsrDeadlineCycles' in motor_c
assert 'cycles >= motorControlIsrDeadlineCycles' in motor_c
assert 'if (DMA1->ISR & DMA_ISR_TCIF1)' not in motor_c

# HBTS debug must expose both current calibration evidence and measured ISR
# execution cycles for the next hardware troubleshooting run.
assert '#define HBTS_DIAG_VERSION          15U' in protocol
assert 'motorControlIsrLastCycles' in protocol
assert 'motorControlIsrMaxCycles' in protocol
assert 'motorControlIsrDeadlineCycles' in protocol
assert 'MotorControl_GetCurrentOffsetDebug' in protocol
assert 'runtimeBuzzerReason' in protocol
assert 'runtimeBuzzerLastReason' in protocol
assert 'sensorCalibrationFastCurrentFaultMask' in protocol
assert 'motor_current_amp' not in protocol
# Internal sensor/commissioning errors must never masquerade as VESC DRV fault.
fault_seg = protocol[protocol.index('static uint8_t fault_code'):protocol.index('static uint8_t node_id')]
assert 'return 3U' not in fault_seg
assert 'ESC_MOTOR_ERROR_ABS_OVER_CURRENT' in fault_seg and 'return 4U' in fault_seg
assert 'vesc_last_set_host_raw_left' in protocol
assert 'vesc_last_set_normalized_left' in protocol
assert 'vesc_last_set_run_left' in protocol

# V15 removes the extra commissioning fast-guard from the timing-sensitive DMA ISR.
# The function/snapshot can remain for backwards-compatible diagnostics, but the hot
# ISR must rely only on the stock DC-link chop plus the closed-loop current limiter.
isr_seg = motor_c[motor_c.index('void DMA1_Channel1_IRQHandler(void)'): ]
assert 'commissioning_current_guard(' not in isr_seg
assert 'abs_s16_saturated(curL_DC) <= curDC_max' in isr_seg
assert 'abs_s16_saturated(curR_DC) <= curDC_max' in isr_seg
assert 'MotorCommissioningFaultSnapshot' in motor_c
assert 'MotorControl_GetCommissioningFaultSnapshot' in protocol

# Current zero calibration must use robust block-mean stability. Raw single-sample
# peak-to-peak remains diagnostic only and must not be the pass/fail criterion.
assert 'CURRENT_CAL_BLOCK_SAMPLES' in motor_c
assert 'CURRENT_CAL_MAX_BLOCK_MEAN_SPAN_COUNTS' in motor_c
assert 'block_min_mean' in motor_c and 'block_max_mean' in motor_c
assert 'span > CURRENT_CAL_MAX_SPAN_COUNTS' not in motor_c
assert 'candidate_mean' in motor_c
assert 'const bool current_offsets_ready = current_cal_service_isr();' in motor_c
assert 'const bool current_offsets_ready = current_cal_service_isr();' in motor_c
assert 'if (!current_offsets_ready)' in motor_c

# Current semantics must remain VESC-like: motor current from Id/Iq vector,
# per-node input current from its physical DC-link sensor, whole-board battery
# current only from DCL+DCR diagnostics. Never derive battery current by summing
# motor-current magnitudes.
assert 'hypotf(id,iq)' in protocol.replace(' ', '')
assert 'right_dc_curr:left_dc_curr' in protocol.replace(' ', '')
assert 'vesc_buf_append_i16(out,dc_curr,&i);' in protocol.replace(' ', '')
assert 'left_dc_curr = MotorControl_GetDcInputCentiAmp(true);' in main_c
assert 'right_dc_curr = MotorControl_GetDcInputCentiAmp(false);' in main_c
assert 'int32_t dc_sum = (int32_t)left_dc_curr + (int32_t)right_dc_curr;' in main_c

# V7 sensor detect must be forced-phase CLOSED-LOOP D-axis current, never the
# V6 arbitrary voltage mapping. One requested amp maps to 800 internal units.
runtime_c = (root/'Src/runtime_control.c').read_text()
assert 'SENSOR_CAL_CURRENT_DEFAULT_INTERNAL 800' in runtime_c
assert 'SENSOR_CAL_CURRENT_MIN_INTERNAL     200' in runtime_c
assert 'SENSOR_CAL_CURRENT_MAX_INTERNAL    1600' in runtime_c
assert 'sensorCalibrationCurrentControlMask' in motor_c
assert 'mc_foc_set_current_commissioning(&motorLeft, true);' in motor_c
assert 'motorLeft.m_id_set = sensorCalibrationCurrentLeft;' in motor_c
assert 'motorLeft.m_iq_set = 0;' in motor_c
assert 'detect_current_internal' in protocol
assert 'detect_open_voltage' not in protocol
assert 'a * (float)units' in protocol
assert 'MotorControl_GetDcRawCentiAmp' in protocol
assert 'MotorControl_GetDcTelemetryRejects' in protocol

# V10 standard VESC current is actuation telemetry only. Released-bridge current
# must be zero per node, raw passive observation remains HBTS-only, and the short
# back-to-back GET_VALUES hold must expire instead of becoming stuck forever.
assert 'MotorControl_BridgeActive(left)' in protocol
assert 'current_avg_reset(a, true);' in protocol
assert 'VESC_CURRENT_HOLD_MAX_MS 25U' in protocol
assert 'last_valid_ms' in protocol
assert 'passive_l1_limit' not in protocol
assert 'passive_rejects' in protocol
assert 'motor_current_amp' not in protocol
start_seg = runtime_c[runtime_c.index('static bool sensor_cal_start('):runtime_c.index('bool RuntimeControl_VescStartSensorDetect(')]
assert 'cfg->hall_calibrated = 0U;' not in start_seg
assert 'cfg->encoder_calibrated = 0U;' not in start_seg
stop_seg = runtime_c[runtime_c.index('static void sensor_cal_stop_output('):runtime_c.index('static int16_t sensor_cal_trig_q14(')]
assert 'motorLeft.m_id_set = 0;' in stop_seg and 'motorRight.m_id_set = 0;' in stop_seg
assert 'mc_foc_set_current_commissioning(&motorLeft, false);' in stop_seg

# Safe current recalibration is available from VESC Tool Terminal and cannot
# directly arm either power bridge.
assert 'C_TERMINAL_CMD=20' in protocol and 'C_PRINT=21' in protocol
assert 'hb_current_cal' in protocol and 'hb_current_status' in protocol
assert 'RuntimeControl_VescReleaseAll();' in protocol
assert 'MotorControl_RequestCurrentOffsetCalibration()' in protocol


# V18 control profile: PWM/ADC/sensor observation stay 16 kHz while the two
# current loops are deterministically interleaved at 8 kHz per motor. This is the
# explicit timing contract that fixes V16/V17 sample-count RPM distortion.
profile = (root/'Src/control_profile.h').read_text()
assert 'CONTROL_PWM_FREQUENCY_HZ             16000U' in profile
assert 'CONTROL_SENSOR_SAMPLE_FREQUENCY_HZ   16000U' in profile
assert 'CONTROL_FOC_MOTOR_FREQUENCY_HZ        8000U' in profile
assert 'CONTROL_CURRENT_ADC_COUNTS_PER_A      50U' in profile
assert 'CONTROL_CURRENT_INTERNAL_PER_A' in profile
assert 'motorFocSlotRight' in motor_c
isr_seg = motor_c[motor_c.index('void DMA1_Channel1_IRQHandler(void)'): ]
assert isr_seg.index('MotorSensor_UpdateHardwareEncoder') < isr_seg.index('if (motorIsrShedNext)')
assert isr_seg.index('MotorSensor_Update(&motorConfigRight') < isr_seg.index('if (motorIsrShedNext)')
assert 'if (!run_right_slot)' in isr_seg and 'mc_foc_run_current_control(&motorLeft' in isr_seg
assert 'mc_foc_run_current_control(&motorRight' in isr_seg
assert 'conf->foc_current_units_per_amp = CONTROL_CURRENT_INTERNAL_PER_A;' in (root/'Src/foc_motor.c').read_text()
assert 'VescProtocol_CurrentTelemetrySample();' in main_c
assert 'current_avg_take' in protocol
assert 'read-reset average' in protocol


# V12 active-domain debug contract
assert "final_active_raw" in motor_c, "V12 must preserve final active current sample"
assert "final_active_residual" in motor_c, "V12 must expose active-domain offset residual"
tester = (root/'tools/vesc_full_test.py').read_text()
assert "set_command_trace.csv" in tester, "V12 tester must emit exact SET trace"

print('RESOURCE_PIN_ADC_CONTRACT_TESTS_PASS')
