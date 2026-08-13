/*
 * VESC Tool 6.00 protocol facade for the dual hoverboard controller.
 * LEFT is the local VESC. RIGHT is a local virtual-CAN device (ID local+1).
 * Packet framing/CRC/command layout matches vedderb/bldc tag 6.00.
 * No RTOS: RX is circular DMA, parser/commands run in main context, TX is DMA.
 */
#include "vesc_protocol.h"
#include "vesc_packet.h"
#include "vesc_buffer.h"
#include "vesc_config_compat.h"
#include "vesc_app.h"
#include "runtime_control.h"
#include "motor_sensor.h"
#include "foc_motor.h"
#include "setup.h"
#include "defines.h"
#include "motor_current_cal.h"
#include "config.h"
#include "stm32f1xx_hal.h"
#include <math.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include <stdio.h>

extern UART_HandleTypeDef huart3;
extern volatile adc_buf_t adc_buffer;
extern int16_t left_dc_curr,right_dc_curr,dc_curr,batVoltageCalib,board_temp_deci_c;
extern mc_foc_output_t motorOutputLeft,motorOutputRight;
extern MotorSensorSample motorSensorSampleLeft,motorSensorSampleRight;
extern MotorRuntimeConfig motorConfigLeft,motorConfigRight;
extern MotorSensorState motorSensorStateLeft,motorSensorStateRight;
extern mc_configuration motorConfLeft,motorConfRight;
extern volatile uint8_t runtimeBuzzerReason;
extern volatile uint8_t runtimeBuzzerLastReason;
extern volatile uint16_t runtimeBuzzerEventCount;

/* VESC 6.00 COMM_PACKET_ID values used here. */
enum {
 C_FW_VERSION=0,C_GET_VALUES=4,C_SET_DUTY=5,C_SET_CURRENT=6,C_SET_CURRENT_BRAKE=7,
 C_SET_RPM=8,C_SET_POS=9,C_SET_HANDBRAKE=10,C_SET_DETECT=11,C_SET_MCCONF=13,C_GET_MCCONF=14,
 C_GET_MCCONF_DEFAULT=15,C_SET_APPCONF=16,C_GET_APPCONF=17,C_GET_APPCONF_DEFAULT=18,
 C_TERMINAL_CMD=20,C_PRINT=21,C_ROTOR_POSITION=22,C_DETECT_ENCODER=27,C_DETECT_HALL_FOC=28,C_REBOOT=29,C_ALIVE=30,C_GET_DECODED_ADC=32,C_FORWARD_CAN=34,C_CUSTOM_APP_DATA=36,
 C_GET_VALUES_SETUP=47,C_GET_VALUES_SELECTIVE=50,C_GET_VALUES_SETUP_SELECTIVE=51,
 C_DETECT_APPLY_ALL_FOC=58,C_PING_CAN=62,C_SET_CURRENT_REL=84
};

#define RX_DMA_SIZE 1024U
#define TX_QUEUE_DEPTH 6U
#define VESC_FW_MAJOR 6U
#define VESC_FW_MINOR 0U
#define UART_OVERRIDE_MS 300U

typedef enum {
    VESC_SIDE_LEFT = 0,
    VESC_SIDE_RIGHT = 1
} vesc_side_t;

typedef struct {
    vesc_side_t side;
    bool right;
} vesc_motor_ctx_t;

static const vesc_motor_ctx_t vesc_ctx_left = {VESC_SIDE_LEFT, false};
static const vesc_motor_ctx_t vesc_ctx_right = {VESC_SIDE_RIGHT, true};
static uint32_t vesc_route_packets_left = 0U;
static uint32_t vesc_route_packets_right = 0U;
static uint32_t vesc_set_packets_left = 0U;
static uint32_t vesc_set_packets_right = 0U;
static uint8_t vesc_last_set_command_left = 0xFFU;
static uint8_t vesc_last_set_command_right = 0xFFU;
/* V12 SET provenance: exact wire int32 plus the normalized RuntimeControl
 * setpoint that was actually submitted. This makes all four VESC Tool control
 * families auditable without guessing from wheel motion. */
static int32_t vesc_last_set_host_raw_left = 0;
static int32_t vesc_last_set_host_raw_right = 0;
static int32_t vesc_last_set_normalized_left = 0;
static int32_t vesc_last_set_normalized_right = 0;
static uint8_t vesc_last_set_run_left = 0U;
static uint8_t vesc_last_set_run_right = 0U;
/* Standard VESC telemetry wire provenance. */
static uint32_t vesc_values_reply_count_left = 0U;
static uint32_t vesc_values_reply_count_right = 0U;
static uint8_t vesc_last_values_fault_left = 0xFFU;
static uint8_t vesc_last_values_fault_right = 0xFFU;
static uint8_t vesc_last_values_command_left = 0xFFU;
static uint8_t vesc_last_values_command_right = 0xFFU;
static uint16_t vesc_last_values_payload_len_left = 0U;
static uint16_t vesc_last_values_payload_len_right = 0U;

static uint8_t rx_dma[RX_DMA_SIZE];
static uint16_t rx_old=0;
static VescPacketParser parser;
static uint8_t tx_frames[TX_QUEUE_DEPTH][VESC_PACKET_MAX_FRAME];
static uint16_t tx_len[TX_QUEUE_DEPTH];
static volatile uint8_t tx_head=0,tx_tail=0;
static volatile bool tx_busy=false;
static uint32_t rx_packets=0,crc_or_parser_errors=0,tx_drops=0,last_uart_control_ms=0;

typedef struct {
    bool active;
    uint8_t command;
    bool right;
    uint8_t sensor_type;
    bool immediate_failure;
    uint16_t reply_retries;
} VescPendingDetect;
static VescPendingDetect pending_detect;
/* VESC Tool Rotor Position display mode is selected with COMM_SET_DETECT and
 * then streamed asynchronously with COMM_ROTOR_POSITION at about 100 Hz. */
static uint8_t display_position_mode_left = 0U;
static uint8_t display_position_mode_right = 0U;
static uint32_t display_position_last_left_ms = 0U;
static uint32_t display_position_last_right_ms = 0U;

typedef enum {
    AUTO_DETECT_IDLE = 0, AUTO_DETECT_WAIT_CURRENT_CAL, AUTO_DETECT_LEFT_ENCODER,
    AUTO_DETECT_RIGHT_HALL, AUTO_DETECT_LEFT_SYNC, AUTO_DETECT_REPLY
} auto_detect_stage_t;
typedef struct {
    bool active;
    auto_detect_stage_t stage;
    int16_t result;
    uint16_t reply_retries;
} VescAutoDetect;
static VescAutoDetect auto_detect;
static bool protocol_initialized = false;

static uint8_t second_id(void){uint8_t n=vescAppConfig.controller_id;return n>=254U?0U:(uint8_t)(n+1U);}
uint8_t VescProtocol_LocalCanId(void){return vescAppConfig.controller_id;}
uint8_t VescProtocol_RightVirtualCanId(void){return second_id();}
uint32_t VescProtocol_RxPackets(void){return rx_packets;}
uint32_t VescProtocol_RxCrcErrors(void){return crc_or_parser_errors;}
uint32_t VescProtocol_TxDrops(void){return tx_drops;}
static uint8_t tx_queue_used(void){return tx_head>=tx_tail?(uint8_t)(tx_head-tx_tail):(uint8_t)(TX_QUEUE_DEPTH-tx_tail+tx_head);}

static void tx_start_next(void){if(tx_busy||tx_tail==tx_head)return;uint8_t q=tx_tail;DMA1_Channel2->CCR&=~DMA_CCR_EN;DMA1->IFCR=DMA_IFCR_CGIF2;DMA1_Channel2->CPAR=(uint32_t)(uintptr_t)&USART3->DR;DMA1_Channel2->CMAR=(uint32_t)(uintptr_t)tx_frames[q];DMA1_Channel2->CNDTR=tx_len[q];DMA1_Channel2->CCR=DMA_CCR_DIR|DMA_CCR_MINC|DMA_CCR_TCIE|DMA_CCR_TEIE;SET_BIT(USART3->CR3,USART_CR3_DMAT);tx_busy=true;DMA1_Channel2->CCR|=DMA_CCR_EN;}
static bool send_payload_internal(const uint8_t*p,uint16_t len,bool count_queue_full){uint8_t next=(uint8_t)((tx_head+1U)%TX_QUEUE_DEPTH);if(next==tx_tail){if(count_queue_full)++tx_drops;return false;}uint16_t n=VescPacket_Encode(p,len,tx_frames[tx_head],VESC_PACKET_MAX_FRAME);if(!n){++tx_drops;return false;}tx_len[tx_head]=n;tx_head=next;tx_start_next();return true;}
static bool send_payload(const uint8_t*p,uint16_t len){return send_payload_internal(p,len,true);}
/* Terminal commissioning replies are retried until accepted by the queue. A
 * temporary full queue is therefore backpressure, not packet loss. */
static bool send_payload_terminal(const uint8_t*p,uint16_t len){return send_payload_internal(p,len,false);}
void VescProtocol_TxDmaIrqHandler(void){uint32_t f=DMA1->ISR;if(f&(DMA_ISR_TCIF2|DMA_ISR_TEIF2)){DMA1_Channel2->CCR&=~DMA_CCR_EN;DMA1->IFCR=DMA_IFCR_CGIF2;CLEAR_BIT(USART3->CR3,USART_CR3_DMAT);if(tx_busy){tx_busy=false;tx_tail=(uint8_t)((tx_tail+1U)%TX_QUEUE_DEPTH);}tx_start_next();}}

static float motor_amp(bool r,int16_t raw){mc_configuration*c=r?&motorConfRight:&motorConfLeft;uint16_t u=c->foc_current_units_per_amp?c->foc_current_units_per_amp:1U;return(float)raw/(float)u;}

typedef struct {
    int32_t id_sum;
    int32_t iq_sum;
    int32_t dc_centi_amp_sum;
    int32_t vd_mv_sum;
    int32_t vq_mv_sum;
    uint16_t samples;
    uint16_t dc_samples;
    int16_t last_id;
    int16_t last_iq;
    int16_t last_dc_centi_amp;
    int16_t last_vd_mv;
    int16_t last_vq_mv;
    uint16_t passive_rejects;
    uint16_t generation;
    uint32_t last_valid_ms;
} vesc_current_avg_t;

static vesc_current_avg_t current_avg_left;
static vesc_current_avg_t current_avg_right;

#define VESC_CURRENT_HOLD_MAX_MS 25U

static void current_avg_reset(vesc_current_avg_t *a, bool clear_last)
{
    if (a == NULL) return;
    a->id_sum = 0;
    a->iq_sum = 0;
    a->dc_centi_amp_sum = 0;
    a->vd_mv_sum = 0;
    a->vq_mv_sum = 0;
    a->samples = 0U;
    a->dc_samples = 0U;
    if (clear_last) {
        a->last_id = 0;
        a->last_iq = 0;
        a->last_dc_centi_amp = 0;
        a->last_vd_mv = 0;
        a->last_vq_mv = 0;
        a->last_valid_ms = 0U;
    }
}

static int16_t foc_mod_q14_to_mv(int16_t mod_q14)
{
    /* VESC defines mod_d/q = Vd/q * 1.5 / Vbus, therefore
     * Vd/q = mod_d/q * (2/3) * Vbus. Our mod_d/q use Q14.
     * batVoltageCalib is centivolts; this runs at 200 Hz, never in DMA ISR. */
    const int64_t num = (int64_t)mod_q14 * (int64_t)batVoltageCalib * 20LL;
    const int64_t den = 3LL * 16384LL;
    int64_t mv = num >= 0 ? (num + den / 2LL) / den : (num - den / 2LL) / den;
    if (mv > INT16_MAX) mv = INT16_MAX;
    if (mv < INT16_MIN) mv = INT16_MIN;
    return (int16_t)mv;
}

void VescProtocol_CurrentTelemetrySample(void)
{
    /* VESC standard current is ACTUATION telemetry, not a raw shunt scope.
     * This stock hoverboard uses low-side shunts whose passive/back-EMF pickup can
     * move while the inverter is released. Therefore GET_VALUES must never turn a
     * released motor's raw observation into Motor Current / Id / Iq. Raw phase/DC
     * evidence remains available in HBTS for diagnostics. This also guarantees
     * that LEFT motion cannot appear as RIGHT virtual-CAN current while RIGHT MOE
     * is off. */
    vesc_current_avg_t *avgs[2] = {&current_avg_left, &current_avg_right};
    const mc_foc_output_t *outs[2] = {&motorOutputLeft, &motorOutputRight};
    const motor_all_state_t *motors[2] = {&motorLeft, &motorRight};

    for (uint8_t n = 0U; n < 2U; ++n) {
        vesc_current_avg_t *a = avgs[n];
        const bool left = n == 0U;
        const bool bridge_active = MotorControl_BridgeActive(left);

        if (!MotorControl_CurrentOffsetsValid() || !bridge_active ||
            !MotorControl_CurrentMeasurementValid(left)) {
            /* Count suppressed passive observations only as a diagnostic. */
            int32_t id_abs = outs[n]->id; if (id_abs < 0) id_abs = -id_abs;
            int32_t iq_abs = outs[n]->iq; if (iq_abs < 0) iq_abs = -iq_abs;
            if ((id_abs + iq_abs) > 0 && a->passive_rejects != UINT16_MAX)
                ++a->passive_rejects;
            current_avg_reset(a, true);
            continue;
        }

        if (a->samples >= 4096U) {
            a->id_sum >>= 1;
            a->iq_sum >>= 1;
            a->vd_mv_sum >>= 1;
            a->vq_mv_sum >>= 1;
            a->samples >>= 1;
        }
        a->id_sum += outs[n]->id;
        a->iq_sum += outs[n]->iq;
        a->vd_mv_sum += foc_mod_q14_to_mv(motors[n]->m_motor_state.mod_d);
        a->vq_mv_sum += foc_mod_q14_to_mv(motors[n]->m_motor_state.mod_q);
        ++a->samples;

        if (MotorControl_CurrentMeasurementValid(left)) {
            if (a->dc_samples >= 4096U) {
                a->dc_centi_amp_sum >>= 1;
                a->dc_samples >>= 1;
            }
            a->dc_centi_amp_sum += MotorControl_GetDcInputCentiAmp(left);
            ++a->dc_samples;
        }
    }
}

static void current_avg_take(bool right, int16_t *id, int16_t *iq, int16_t *dc_centi_amp,
                             int16_t *vd_mv, int16_t *vq_mv)
{
    vesc_current_avg_t *a = right ? &current_avg_right : &current_avg_left;
    const bool left = !right;

    /* Never publish a stale nonzero standard current after this motor's bridge is
     * released. This is the key cross-side/stuck-current invariant for virtual CAN. */
    if (!MotorControl_BridgeActive(left) || !MotorControl_CurrentOffsetsValid() ||
        !MotorControl_CurrentMeasurementValid(left)) {
        current_avg_reset(a, true);
        *id = 0;
        *iq = 0;
        *dc_centi_amp = 0;
        *vd_mv = 0;
        *vq_mv = 0;
        return;
    }

    const uint32_t now = RuntimeControl_MonotonicMs();
    if (a->samples != 0U) {
        const int32_t half = (int32_t)(a->samples >> 1);
        a->last_id = (int16_t)((a->id_sum >= 0 ? a->id_sum + half : a->id_sum - half) /
                               (int32_t)a->samples);
        a->last_iq = (int16_t)((a->iq_sum >= 0 ? a->iq_sum + half : a->iq_sum - half) /
                               (int32_t)a->samples);
        a->last_vd_mv = (int16_t)((a->vd_mv_sum >= 0 ? a->vd_mv_sum + half : a->vd_mv_sum - half) /
                                  (int32_t)a->samples);
        a->last_vq_mv = (int16_t)((a->vq_mv_sum >= 0 ? a->vq_mv_sum + half : a->vq_mv_sum - half) /
                                  (int32_t)a->samples);
        if (a->dc_samples != 0U) {
            const int32_t dc_half = (int32_t)(a->dc_samples >> 1);
            a->last_dc_centi_amp = (int16_t)((a->dc_centi_amp_sum >= 0 ?
                                      a->dc_centi_amp_sum + dc_half : a->dc_centi_amp_sum - dc_half) /
                                      (int32_t)a->dc_samples);
        } else {
            a->last_dc_centi_amp = 0;
        }
        current_avg_reset(a, false);
        a->last_valid_ms = now;
        if (a->generation != UINT16_MAX) ++a->generation;
    } else if ((uint32_t)(now - a->last_valid_ms) > VESC_CURRENT_HOLD_MAX_MS) {
        /* GET_VALUES and GET_VALUES_SETUP can be polled back-to-back. A short hold
         * avoids alternating zero values between the two commands, but never lets
         * one accepted sample become permanently 'stuck' in VESC Tool. */
        a->last_id = 0;
        a->last_iq = 0;
        a->last_dc_centi_amp = 0;
        a->last_vd_mv = 0;
        a->last_vq_mv = 0;
    }

    *id = a->last_id;
    *iq = a->last_iq;
    *dc_centi_amp = a->last_dc_centi_amp;
    *vd_mv = a->last_vd_mv;
    *vq_mv = a->last_vq_mv;
}

static float motor_current_from_raw(bool right, int16_t id_raw, int16_t iq_raw)
{
    const float id = motor_amp(right, id_raw);
    const float iq = motor_amp(right, iq_raw);
    float mag = hypotf(id, iq);
    /* Match VESC 6.00 mcpwm_foc_get_tot_current_filtered_motor semantics:
     * sign = SIGN(Vq * Iq), while magnitude includes BOTH Id and Iq. SIGN(0)
     * is positive upstream, so a stationary D-axis commissioning current must
     * still appear as Imotor instead of being forced to zero. */
    const int32_t vq = right ? motorRight.m_motor_state.vq : motorLeft.m_motor_state.vq;
    if (((int64_t)vq * (int64_t)iq_raw) < 0) mag = -mag;
    return mag;
}
static float duty(bool r){uint16_t q=r?motorOutputRight.duty_abs_q15:motorOutputLeft.duty_abs_q15;float d=(float)q/32767.0f;int16_t iq=r?motorOutputRight.iq:motorOutputLeft.iq;return iq<0?-d:d;}
static int32_t erpm(bool r){
    mc_configuration*c=r?&motorConfRight:&motorConfLeft;
    motor_all_state_t*m=r?&motorRight:&motorLeft;
    /* GET_VALUES RPM is electrical RPM. Keep the sensor Q4 mechanical-speed
     * precision until after multiplying by pole-pairs; the V16 path first
     * rounded to whole mechanical RPM, quantizing telemetry in 15-eRPM steps. */
    int64_t q4=(int64_t)m->m_speed_rpm_q4*(int64_t)c->foc_motor_pole_pairs;
    if(q4>=0)q4+=8;else q4-=8;
    q4/=16;
    if(q4>INT32_MAX)q4=INT32_MAX;
    if(q4<INT32_MIN)q4=INT32_MIN;
    return(int32_t)q4;
}
static int32_t tach(bool r){MotorRuntimeConfig*cfg=r?&motorConfigRight:&motorConfigLeft;mc_configuration*c=r?&motorConfRight:&motorConfLeft;int32_t p=r?motorSensorSampleRight.position_ticks:motorSensorSampleLeft.position_ticks;uint32_t cpr=(cfg->sensor_type==MOTOR_SENSOR_ENCODER_AB&&!r)?cfg->encoder_cpr:(uint32_t)6U*c->foc_motor_pole_pairs;if(cpr==0)return 0;int64_t t=(int64_t)p*(int64_t)(6U*c->foc_motor_pole_pairs);return(int32_t)(t/(int64_t)cpr);}
static float pos_deg(bool r){return RuntimeControl_PositionDeg(!r);}
static uint8_t fault_code(bool r){
    const uint8_t f=r?RuntimeControl_ErrorRight():RuntimeControl_ErrorLeft();
    if(f==ESC_MOTOR_ERROR_NONE)return 0U;
    if(f==ESC_MOTOR_ERROR_ABS_OVER_CURRENT)return 4U; /* FAULT_CODE_ABS_OVER_CURRENT */
    /* V12: never forge FAULT_CODE_DRV (3). This board has no DRV830x-style
     * standard fault source. Sensor readiness, encoder/hall commissioning and ISR
     * deadline faults remain enforced internally and are exported losslessly by
     * HBTS, but VESC Tool must not label them as a gate-driver failure. */
    return 0U;
}
static uint8_t node_id(bool r){return r?second_id():vescAppConfig.controller_id;}
static void record_values_wire(bool r, uint8_t cmd, uint8_t fault, uint16_t payload_len)
{
    uint32_t *count = r ? &vesc_values_reply_count_right : &vesc_values_reply_count_left;
    if (*count != UINT32_MAX) ++(*count);
    if (r) {
        vesc_last_values_fault_right = fault;
        vesc_last_values_command_right = cmd;
        vesc_last_values_payload_len_right = payload_len;
    } else {
        vesc_last_values_fault_left = fault;
        vesc_last_values_command_left = cmd;
        vesc_last_values_payload_len_left = payload_len;
    }
}

/* HBTS diagnostic/commissioning extension for the one-shot Python bench tester.
 * It remains inside standard VESC COMM_CUSTOM_APP_DATA, so normal VESC Tool is
 * unaffected. V20 uses HBTS diagnostic layout version 15. It carries raw-vs-validated
 * current evidence, exact command provenance and active-domain calibration proof;
 * the only write is guarded zero-current recalibration. */
#define HBTS_DIAG_VERSION          15U
#define HBTS_DIAG_GET              1U
#define HBTS_CURRENT_RECAL         2U
#define HBTS_DIAG_REPLY            0x81U
#define HBTS_CURRENT_RECAL_REPLY   0x82U
static bool hbts_magic(const uint8_t *d, uint16_t l)
{
    return d != NULL && l >= 6U && d[0]=='H' && d[1]=='B' && d[2]=='T' && d[3]=='S' &&
           d[4]==HBTS_DIAG_VERSION;
}
static void hbts_diag(bool r)
{
    /* HBTS v15 is 453 bytes before VESC framing. Keep explicit headroom for
     * append-only debug growth; V19 used undersized debug/FW buffers and could
     * corrupt the stack during the very packets used for troubleshooting. */
    uint8_t out[512];
    int32_t i=0;
    const MotorRuntimeConfig *cfg=r?&motorConfigRight:&motorConfigLeft;
    const MotorSensorSample *sample=r?&motorSensorSampleRight:&motorSensorSampleLeft;
    const MotorSensorState *state=r?&motorSensorStateRight:&motorSensorStateLeft;
    const mc_foc_output_t *foc=r?&motorOutputRight:&motorOutputLeft;
    const mc_configuration *conf=r?&motorConfRight:&motorConfLeft;
    MotorCurrentOffsetDebug cdbg;
    MotorControl_GetCurrentOffsetDebug(&cdbg);
    const bool armed=r?RuntimeControl_ArmedRight():RuntimeControl_ArmedLeft();
    const uint8_t reject=r?RuntimeControl_ArmRejectRight():RuntimeControl_ArmRejectLeft();
    const uint8_t err=r?RuntimeControl_ErrorRight():RuntimeControl_ErrorLeft();
    const bool calibrated=(cfg->sensor_type==MOTOR_SENSOR_ENCODER_AB)?
        (cfg->encoder_calibrated!=0U):(cfg->hall_calibrated!=0U && cfg->hall_lut_valid!=0U);
    const bool enc_aligned=(cfg->sensor_type==MOTOR_SENSOR_ENCODER_AB)?state->encoder_electrical_aligned:true;
    uint16_t flags=0U;
    if(armed)flags|=(1U<<0);
    if((runtimeMotorEnableMask&(r?0x02U:0x01U))!=0U)flags|=(1U<<1);
    if(RuntimeControl_LinkActive())flags|=(1U<<2);
    if(sample->feedback_valid)flags|=(1U<<3);
    if(calibrated)flags|=(1U<<4);
    if(enc_aligned)flags|=(1U<<5);
    if(RuntimeSettings_Verified())flags|=(1U<<6);
    const uint8_t cal_live_state=RuntimeControl_SensorCalibrationState();
    const uint8_t cal_live_motor=RuntimeControl_SensorCalibrationMotor();
    const bool cal_owned=(cal_live_motor==(r?ESC_MOTOR_RIGHT:ESC_MOTOR_LEFT)) && cal_live_state!=ESC_SENSOR_CAL_IDLE;
    if(pending_detect.active && pending_detect.right==r)flags|=(1U<<7);
    if(cal_owned && cal_live_state==ESC_SENSOR_CAL_RUNNING)flags|=(1U<<8);
    if((RuntimeControl_HomingActiveMask()&(r?0x02U:0x01U))!=0U)flags|=(1U<<9);
    if(RuntimeControl_EncoderAlignmentActive(!r))flags|=(1U<<10);
    if(cdbg.valid)flags|=(1U<<11);
    const bool current_measurement_valid=MotorControl_CurrentMeasurementValid(!r);
    const bool bridge_moe=r?((RIGHT_TIM->BDTR&TIM_BDTR_MOE)!=0U):((LEFT_TIM->BDTR&TIM_BDTR_MOE)!=0U);
    if(current_measurement_valid)flags|=(1U<<12);
    if(bridge_moe)flags|=(1U<<13);
    if((sensorCalibrationCurrentControlMask&(r?0x02U:0x01U))!=0U)flags|=(1U<<14);

    out[i++]=C_CUSTOM_APP_DATA;
    out[i++]='H';out[i++]='B';out[i++]='T';out[i++]='S';
    out[i++]=HBTS_DIAG_VERSION;
    out[i++]=HBTS_DIAG_REPLY;
    out[i++]=node_id(r);
    out[i++]=r?1U:0U;
    vesc_buf_append_u16(out,flags,&i);
    out[i++]=cfg->sensor_type;
    out[i++]=reject;
    out[i++]=err;
    out[i++]=fault_code(r);
    out[i++]=r?controlModeRightFoc:controlModeLeftFoc;
    out[i++]=sample->raw_hall_encoding;
    out[i++]=sample->encoder_ab;
    out[i++]=conf->foc_motor_pole_pairs;
    vesc_buf_append_u16(out,cfg->encoder_cpr,&i);
    vesc_buf_append_u16(out,RuntimeControl_FaultStopRemainingMs(!r),&i);
    vesc_buf_append_u16(out,RuntimeControl_LinkAgeMs(),&i);
    vesc_buf_append_i32(out,sample->position_ticks,&i);
    vesc_buf_append_i16(out,sample->mechanical_angle_q4,&i);
    vesc_buf_append_u16(out,sample->electrical_phase_q16,&i);
    vesc_buf_append_i16(out,sample->mechanical_speed_q4,&i);
    vesc_buf_append_i32(out,foc->speed_rpm,&i);
    vesc_buf_append_i16(out,foc->id,&i);
    vesc_buf_append_i16(out,foc->iq,&i);
    vesc_buf_append_u16(out,foc->duty_abs_q15,&i);
    vesc_buf_append_i16(out,r?right_dc_curr:left_dc_curr,&i);
    vesc_buf_append_i16(out,batVoltageCalib,&i);
    vesc_buf_append_u16(out,conf->foc_current_units_per_amp,&i);
    vesc_buf_append_i32(out,r?runtimeSetpointRight:runtimeSetpointLeft,&i);
    vesc_buf_append_i16(out,r?runtimeCommandRight:runtimeCommandLeft,&i);
    vesc_buf_append_u16(out,motorControlIsrOverrunCount,&i);
    vesc_buf_append_u32(out,motorControlIsrLastCycles,&i);
    vesc_buf_append_u32(out,motorControlIsrMaxCycles,&i);
    vesc_buf_append_u32(out,motorControlIsrDeadlineCycles,&i);
    out[i++]=motorControlIsrOverrunFaultMask;
    out[i++]=motorControlOvercurrentFaultMask;
    vesc_buf_append_u32(out,state->encoder_valid_edges,&i);
    vesc_buf_append_u32(out,state->encoder_invalid_transitions,&i);
    vesc_buf_append_u32(out,rx_packets,&i);
    vesc_buf_append_u32(out,crc_or_parser_errors,&i);
    vesc_buf_append_u32(out,tx_drops,&i);
    vesc_buf_append_u16(out,RuntimeSettings_GetGeneration(),&i);
    vesc_buf_append_u16(out,RuntimeSettings_GetVerifyFailures(),&i);
    vesc_buf_append_u16(out,adc_buffer.pa2Analog,&i);
    vesc_buf_append_u16(out,adc_buffer.pa3Analog,&i);
    vesc_buf_append_i32(out,VescApp_GetDecoded1Micro(),&i);
    vesc_buf_append_i32(out,VescApp_GetDecoded2Micro(),&i);
    out[i++]=cal_owned?cal_live_state:ESC_SENSOR_CAL_IDLE;
    out[i++]=cal_owned?cal_live_motor:(r?ESC_MOTOR_RIGHT:ESC_MOTOR_LEFT);
    out[i++]=cal_owned?RuntimeControl_SensorCalibrationType():cfg->sensor_type;
    out[i++]=r?RuntimeControl_HomingStateRight():RuntimeControl_HomingStateLeft();
    out[i++]=vescAppConfig.app_to_use;
    out[i++]=vescAppConfig.adc_ctrl_type;
    out[i++]=vescAppConfig.multi_esc;
    vesc_buf_append_u32(out,vescAppConfig.timeout_ms,&i);

    /* Current evidence. Imotor remains the FOC phase-vector magnitude;
     * Ibattery_total is the sum of the two validated DC-link currents. */
    out[i++]=cdbg.state;
    out[i++]=cdbg.valid;
    out[i++]=cdbg.adc1_hw_cal_ok;
    out[i++]=cdbg.adc2_hw_cal_ok;
    vesc_buf_append_u16(out,cdbg.collected_samples,&i);
    vesc_buf_append_u16(out,cdbg.target_samples,&i);
    vesc_buf_append_u16(out,cdbg.failure_mask,&i);
    vesc_buf_append_u16(out,cdbg.generation,&i);
    for(uint8_t n=0U;n<6U;++n)vesc_buf_append_u16(out,cdbg.raw[n],&i);
    for(uint8_t n=0U;n<6U;++n)vesc_buf_append_u16(out,cdbg.candidate_mean[n],&i);
    for(uint8_t n=0U;n<6U;++n)vesc_buf_append_u16(out,cdbg.offset[n],&i);
    for(uint8_t n=0U;n<6U;++n)vesc_buf_append_u16(out,cdbg.raw_span[n],&i);
    for(uint8_t n=0U;n<6U;++n)vesc_buf_append_u16(out,cdbg.block_span[n],&i);
    for(uint8_t n=0U;n<6U;++n)vesc_buf_append_i16(out,cdbg.residual[n],&i);
    vesc_buf_append_i16(out,dc_curr,&i);
    const float imotor_ca_f=motor_current_from_raw(r,foc->id,foc->iq)*100.0f;
    int32_t imotor_ca=(int32_t)lrintf(imotor_ca_f);
    if(imotor_ca>INT16_MAX)imotor_ca=INT16_MAX;
    if(imotor_ca<INT16_MIN)imotor_ca=INT16_MIN;
    vesc_buf_append_i16(out,(int16_t)imotor_ca,&i);
    out[i++]=RuntimeControl_SensorCalibrationResultCode();

    /* V7 measurement/commissioning evidence. Standard VESC current fields are
     * validated/gated; these explicit raw values make troubleshooting lossless. */
    vesc_buf_append_i16(out,MotorControl_GetDcInputCentiAmp(!r),&i);
    vesc_buf_append_i16(out,MotorControl_GetDcRawCentiAmp(!r),&i);
    vesc_buf_append_u16(out,MotorControl_GetDcTelemetryRejects(!r),&i);
    const bool cal_this_motor=cal_owned && cal_live_state==ESC_SENSOR_CAL_RUNNING;
    vesc_buf_append_i16(out,cal_this_motor?RuntimeControl_SensorCalibrationTargetCurrentInternal():0,&i);
    vesc_buf_append_i16(out,cal_this_motor?RuntimeControl_SensorCalibrationMeasuredCurrentInternal():0,&i);
    vesc_buf_append_u16(out,cal_this_motor?RuntimeControl_SensorCalibrationElectricalPhaseQ16():0U,&i);
    vesc_buf_append_i16(out,foc->id_target,&i);
    vesc_buf_append_i16(out,foc->iq_target,&i);

    /* V8 current-loop proof: enough evidence to distinguish scaling/polarity,
     * PI runaway and telemetry filtering without another firmware guess. */
    const motor_state_t *ms = r ? &motorRight.m_motor_state : &motorLeft.m_motor_state;
    const vesc_current_avg_t *avg = r ? &current_avg_right : &current_avg_left;
    vesc_buf_append_i16(out,ms->vd,&i);
    vesc_buf_append_i16(out,ms->vq,&i);
    vesc_buf_append_i16(out,ms->mod_d,&i);
    vesc_buf_append_i16(out,ms->mod_q,&i);
    vesc_buf_append_u16(out,avg->passive_rejects,&i);
    out[i++]=sensorCalibrationFastCurrentFaultMask;
    vesc_buf_append_i32(out,conf->foc_current_kp_q16,&i);
    vesc_buf_append_i32(out,conf->foc_current_ki_dt_q16,&i);

    /* V8 buzzer-source evidence. This is observation only; it never changes the
     * safety state. The last reason survives after the audible pattern ends. */
    out[i++]=runtimeBuzzerReason;
    out[i++]=runtimeBuzzerLastReason;
    vesc_buf_append_u16(out,runtimeBuzzerEventCount,&i);

    /* Per-node standard-current provenance (retained from V10). This explicitly proves that
     * standard VESC current is tied to THIS motor's bridge and cannot be copied
     * from the opposite virtual-CAN context or held forever after release. */
    const uint32_t now_ms = RuntimeControl_MonotonicMs();
    const uint32_t avg_age = avg->last_valid_ms == 0U ? UINT32_MAX :
        (uint32_t)(now_ms - avg->last_valid_ms);
    const bool avg_has_live_or_held = avg->samples != 0U ||
        (avg->last_valid_ms != 0U && avg_age <= VESC_CURRENT_HOLD_MAX_MS);
    out[i++]=bridge_moe?1U:0U;
    out[i++]=(bridge_moe && current_measurement_valid && avg_has_live_or_held)?1U:0U;
    vesc_buf_append_u16(out,avg->generation,&i);
    vesc_buf_append_u16(out,avg_age > UINT16_MAX ? UINT16_MAX : (uint16_t)avg_age,&i);
    vesc_buf_append_i16(out,avg->last_id,&i);
    vesc_buf_append_i16(out,avg->last_iq,&i);
    vesc_buf_append_i16(out,avg->last_dc_centi_amp,&i);

    /* First-sample commissioning trip evidence. Unlike live detect fields above,
     * this snapshot survives abort cleanup and therefore captures the exact raw
     * ADC/offset/current/PWM state that caused FAST CURRENT GUARD. */
    MotorCommissioningFaultSnapshot fs;
    MotorControl_GetCommissioningFaultSnapshot(!r,&fs);
    out[i++]=fs.valid;
    out[i++]=fs.fault_mask;
    out[i++]=fs.streak_at_trip;
    out[i++]=fs.left;
    vesc_buf_append_u16(out,fs.generation,&i);
    vesc_buf_append_i16(out,fs.target_internal,&i);
    vesc_buf_append_i16(out,fs.id_internal,&i);
    vesc_buf_append_i16(out,fs.iq_internal,&i);
    vesc_buf_append_i16(out,fs.phase_current_1_delta,&i);
    vesc_buf_append_i16(out,fs.phase_current_2_delta,&i);
    vesc_buf_append_i16(out,fs.dc_delta,&i);
    vesc_buf_append_u16(out,fs.adc_phase_1_raw,&i);
    vesc_buf_append_u16(out,fs.adc_phase_2_raw,&i);
    vesc_buf_append_u16(out,fs.adc_dc_raw,&i);
    vesc_buf_append_u16(out,fs.offset_phase_1,&i);
    vesc_buf_append_u16(out,fs.offset_phase_2,&i);
    vesc_buf_append_u16(out,fs.offset_dc,&i);
    vesc_buf_append_u16(out,fs.forced_phase_q16,&i);
    vesc_buf_append_i16(out,fs.duty_a,&i);
    vesc_buf_append_i16(out,fs.duty_b,&i);
    vesc_buf_append_i16(out,fs.duty_c,&i);
    vesc_buf_append_u16(out,fs.duty_abs_q15,&i);

    /* Routing provenance (retained from V10). These counters make virtual-CAN isolation auditable:
     * a command forwarded to RIGHT must increment RIGHT context only, while a
     * local command must stay in LEFT context. */
    vesc_buf_append_u32(out,r?vesc_route_packets_right:vesc_route_packets_left,&i);
    vesc_buf_append_u32(out,r?vesc_route_packets_left:vesc_route_packets_right,&i);
    vesc_buf_append_u32(out,r?vesc_set_packets_right:vesc_set_packets_left,&i);
    out[i++]=r?vesc_last_set_command_right:vesc_last_set_command_left;

    /* V12 current-domain + command decoder proof. Appended to preserve every
     * earlier HBTS field offset. */
    out[i++]=cdbg.sampling_mode;
    out[i++]=cdbg.bridge_active_mask;
    out[i++]=cdbg.bridge_warmup_left;
    out[i++]=cdbg.bridge_warmup_right;
    vesc_buf_append_u16(out,cdbg.bridge_transition_left,&i);
    vesc_buf_append_u16(out,cdbg.bridge_transition_right,&i);
    vesc_buf_append_i32(out,r?vesc_last_set_host_raw_right:vesc_last_set_host_raw_left,&i);
    vesc_buf_append_i32(out,r?vesc_last_set_normalized_right:vesc_last_set_normalized_left,&i);
    out[i++]=r?vesc_last_set_run_right:vesc_last_set_run_left;
    for(uint8_t n=0U;n<6U;++n)vesc_buf_append_u16(out,cdbg.final_active_raw[n],&i);
    for(uint8_t n=0U;n<6U;++n)vesc_buf_append_i16(out,cdbg.final_active_residual[n],&i);

    /* V12 commissioning + standard-wire proof. Appended only. */
    out[i++]=cal_owned?(uint8_t)RuntimeControl_SensorCalibrationSweepDirection():0U;
    out[i++]=cal_owned?RuntimeControl_SensorCalibrationForwardCycles():0U;
    out[i++]=cal_owned?RuntimeControl_SensorCalibrationReverseCycles():0U;
    vesc_buf_append_u16(out,cal_owned?RuntimeControl_SensorCalibrationCompletedCycles():0U,&i);
    out[i++]=(cal_owned&&RuntimeControl_SensorCalibrationMotionDetected())?1U:0U;
    vesc_buf_append_u32(out,cal_owned?RuntimeControl_SensorCalibrationMotionCounter():0U,&i);
    vesc_buf_append_u16(out,cal_owned?RuntimeControl_SensorCalibrationMotionAgeMs():0U,&i);
    out[i++]=cal_owned?RuntimeControl_SensorCalibrationObservedHallMask():0U;
    out[i++]=cal_owned?RuntimeControl_SensorCalibrationObservedEncoderMask():0U;
    vesc_buf_append_i32(out,cal_owned?RuntimeControl_SensorCalibrationEncoderDelta():0,&i);
    vesc_buf_append_u16(out,avg->samples,&i);
    vesc_buf_append_u16(out,avg->dc_samples,&i);
    const int16_t comm_vmax=mc_foc_commissioning_voltage_limit();
    vesc_buf_append_u16(out,(uint16_t)comm_vmax,&i);
    int32_t vd_abs=ms->vd; if(vd_abs<0)vd_abs=-vd_abs;
    int32_t vq_abs=ms->vq; if(vq_abs<0)vq_abs=-vq_abs;
    out[i++]=(vd_abs >= ((int32_t)comm_vmax-16))?1U:0U;
    out[i++]=(vq_abs >= ((int32_t)comm_vmax-16))?1U:0U;
    vesc_buf_append_u32(out,r?vesc_values_reply_count_right:vesc_values_reply_count_left,&i);
    out[i++]=r?vesc_last_values_fault_right:vesc_last_values_fault_left;
    out[i++]=r?vesc_last_values_command_right:vesc_last_values_command_left;
    vesc_buf_append_u16(out,r?vesc_last_values_payload_len_right:vesc_last_values_payload_len_left,&i);

    /* V14 transaction evidence. A terminal result is per motor and survives the
     * opposite motor's detect. Queue depth/owner/retry count makes a dropped or
     * starved blocking reply diagnosable from one log. */
    RuntimeVescDetectResult last_detect;
    bool ratio_fallback=false;
    const bool last_detect_valid=RuntimeControl_VescGetLastSensorDetect(!r,&last_detect,&ratio_fallback);
    out[i++]=tx_queue_used();
    out[i++]=pending_detect.active?(pending_detect.right?2U:1U):0U;
    vesc_buf_append_u16(out,pending_detect.reply_retries,&i);
    out[i++]=last_detect_valid?1U:0U;
    out[i++]=last_detect_valid?last_detect.state:0U;
    out[i++]=last_detect_valid?last_detect.result_code:0U;
    out[i++]=last_detect_valid?last_detect.sensor_type:cfg->sensor_type;
    out[i++]=(last_detect_valid&&ratio_fallback)?1U:0U;
    out[i++]=last_detect_valid?last_detect.pole_pairs:conf->foc_motor_pole_pairs;
    out[i++]=last_detect_valid?last_detect.encoder_inverted:cfg->sensor_inverted;
    vesc_buf_append_u16(out,last_detect_valid?last_detect.encoder_cpr:cfg->encoder_cpr,&i);

    /* V17 encoder-direction commissioning proof. These appended fields make a
     * wrong speed-feedback sign diagnosable without guessing A/B polarity. */
    vesc_buf_append_i32(out,cal_owned?RuntimeControl_SensorCalibrationEncoderForwardDelta():0,&i);
    vesc_buf_append_u32(out,cal_owned?RuntimeControl_SensorCalibrationEncoderDirectionNormalScore():0U,&i);
    vesc_buf_append_u32(out,cal_owned?RuntimeControl_SensorCalibrationEncoderDirectionInvertedScore():0U,&i);
    out[i++]=(cal_owned&&RuntimeControl_SensorCalibrationEncoderDirectionProved())?1U:0U;

    /* V19 integrated detect/sync/homing/position proof. Keep append-only so old
     * field offsets remain stable for archived hardware logs. */
    const SteeringCalibration *steer = r ? &steeringCalibrationRight : &steeringCalibrationLeft;
    out[i++]=cfg->encoder_ratio;
    out[i++]=RuntimeControl_EncoderElectricalReady(!r)?1U:0U;
    out[i++]=steer->calibrated;
    out[i++]=steer->homed;
    vesc_buf_append_i32(out,steer->right_zero_ticks,&i);
    vesc_buf_append_i32(out,steer->span_ticks,&i);
    vesc_buf_append_i32(out,(int32_t)lrintf(RuntimeControl_PositionDeg(!r)*1000.0f),&i);
    vesc_buf_append_i32(out,r?motorRight.m_pos_pid_set:motorLeft.m_pos_pid_set,&i);
    vesc_buf_append_i16(out,r?motorRight.m_duty_cycle_set_q15:motorLeft.m_duty_cycle_set_q15,&i);
    out[i++]=RuntimeControl_HomingOnBoot(!r)?1U:0U;
    out[i++]=(RuntimeControl_HomingActiveMask()&(r?0x02U:0x01U))?1U:0U;
    out[i++]=r?display_position_mode_right:display_position_mode_left;
    out[i++]=auto_detect.active?1U:0U;
    out[i++]=(uint8_t)auto_detect.stage;
    vesc_buf_append_i16(out,auto_detect.result,&i);
    vesc_buf_append_u16(out,auto_detect.reply_retries,&i);

    /* V20 LEFT electrical-sync + bounded position proof. Slow/debug only. */
    vesc_buf_append_i16(out,RuntimeControl_EncoderAlignmentProbeDelta(!r),&i);
    out[i++]=RuntimeControl_EncoderAlignmentDirectionProved(!r)?1U:0U;
    int32_t session_zero=0;
    (void)RuntimeControl_PositionTargetTicks(!r,0.0f,&session_zero);
    vesc_buf_append_i32(out,session_zero,&i);
    out[i++]=0U; /* real flux-observer unavailable in this port */
    send_payload(out,(uint16_t)i);
}

static void hbts_current_recal(void)
{
    uint8_t out[12];
    int32_t i=0;
    /* Release is deterministic and never bypasses safety. Refuse while another
     * commissioning transaction, homing or encoder alignment owns the bridge. */
    bool accepted=false;
    if(RuntimeControl_SensorCalibrationState()!=ESC_SENSOR_CAL_RUNNING &&
       RuntimeControl_HomingActiveMask()==0U &&
       !RuntimeControl_EncoderAlignmentActive(true) &&
       !RuntimeControl_EncoderAlignmentActive(false)){
        RuntimeControl_VescReleaseAll();
        accepted=MotorControl_RequestCurrentOffsetCalibration();
    }
    out[i++]=C_CUSTOM_APP_DATA;
    out[i++]='H';out[i++]='B';out[i++]='T';out[i++]='S';
    out[i++]=HBTS_DIAG_VERSION;
    out[i++]=HBTS_CURRENT_RECAL_REPLY;
    out[i++]=accepted?1U:0U;
    out[i++]=MotorControl_CurrentOffsetCalState();
    send_payload(out,(uint16_t)i);
}

static void custom_app_data(const uint8_t *d,uint16_t l,bool r)
{
    if(!hbts_magic(d,l))return;
    if(d[5]==HBTS_DIAG_GET)hbts_diag(r);
    else if(d[5]==HBTS_CURRENT_RECAL)hbts_current_recal();
}

static void append_values(uint8_t cmd,const uint8_t*data,uint16_t len,bool r)
{
    uint8_t out[180];
    int32_t i=0;
    uint32_t mask=0xFFFFFFFFUL;
    uint8_t wire_fault=0xFFU;
    out[i++]=cmd;
    if(cmd==C_GET_VALUES_SELECTIVE){
        if(len<4)return;
        int32_t j=0;
        mask=vesc_buf_get_u32(data,&j);
        vesc_buf_append_u32(out,mask,&i);
    }

    /* VESC 6.00 uses read-reset average motor/input/Id/Iq values. Reproduce that
     * behavior here so VESC Tool does not display 8-kHz phase-current snapshots. */
    int16_t avg_id=0, avg_iq=0, avg_dc_ca=0, avg_vd_mv=0, avg_vq_mv=0;
    current_avg_take(r,&avg_id,&avg_iq,&avg_dc_ca,&avg_vd_mv,&avg_vq_mv);

    if(mask&(1UL<<0))vesc_buf_append_float16(out,(float)board_temp_deci_c/10.0f,10,&i);
    if(mask&(1UL<<1))vesc_buf_append_float16(out,0,10,&i);
    if(mask&(1UL<<2))vesc_buf_append_float32(out,motor_current_from_raw(r,avg_id,avg_iq),100,&i);
    if(mask&(1UL<<3))vesc_buf_append_float32(out,(float)avg_dc_ca/100.0f,100,&i);
    if(mask&(1UL<<4))vesc_buf_append_float32(out,motor_amp(r,avg_id),100,&i);
    if(mask&(1UL<<5))vesc_buf_append_float32(out,motor_amp(r,avg_iq),100,&i);
    if(mask&(1UL<<6))vesc_buf_append_float16(out,duty(r),1000,&i);
    if(mask&(1UL<<7))vesc_buf_append_float32(out,(float)erpm(r),1,&i);
    if(mask&(1UL<<8))vesc_buf_append_float16(out,(float)batVoltageCalib/100.0f,10,&i);
    for(uint8_t bit=9;bit<=12;bit++)if(mask&(1UL<<bit))vesc_buf_append_float32(out,0,10000,&i);
    if(mask&(1UL<<13))vesc_buf_append_i32(out,tach(r),&i);
    if(mask&(1UL<<14)){int32_t t=tach(r);if(t<0)t=-t;vesc_buf_append_i32(out,t,&i);}
    if(mask&(1UL<<15)){wire_fault=fault_code(r);out[i++]=wire_fault;}
    if(mask&(1UL<<16))vesc_buf_append_float32(out,pos_deg(r),1000000,&i);
    if(mask&(1UL<<17))out[i++]=node_id(r);
    if(mask&(1UL<<18)){vesc_buf_append_float16(out,(float)board_temp_deci_c/10.0f,10,&i);vesc_buf_append_float16(out,(float)board_temp_deci_c/10.0f,10,&i);vesc_buf_append_float16(out,(float)board_temp_deci_c/10.0f,10,&i);}
    if(mask&(1UL<<19))vesc_buf_append_float32(out,(float)avg_vd_mv/1000.0f,1000,&i);
    if(mask&(1UL<<20))vesc_buf_append_float32(out,(float)avg_vq_mv/1000.0f,1000,&i);
    if(mask&(1UL<<21))out[i++]=RuntimeControl_LinkActive()?0U:1U;
    record_values_wire(r,cmd,wire_fault,(uint16_t)i);
    send_payload(out,(uint16_t)i);
}

static void append_setup(uint8_t cmd, const uint8_t *data, uint16_t len, bool right)
{
    uint8_t out[160];
    int32_t i = 0;
    uint32_t mask = 0xFFFFFFFFUL;
    uint8_t wire_fault = 0xFFU;

    out[i++] = cmd;
    if (cmd == C_GET_VALUES_SETUP_SELECTIVE) {
        if (len < 4U) return;
        int32_t j = 0;
        mask = vesc_buf_get_u32(data, &j);
        vesc_buf_append_u32(out, mask, &i);
    }

    int16_t avg_id=0, avg_iq=0, avg_dc_ca=0, avg_vd_mv=0, avg_vq_mv=0;
    current_avg_take(right,&avg_id,&avg_iq,&avg_dc_ca,&avg_vd_mv,&avg_vq_mv);
    (void)avg_vd_mv; (void)avg_vq_mv;

    if (mask & (1UL << 0)) {
        vesc_buf_append_float16(out, (float)board_temp_deci_c / 10.0f, 10, &i);
    }
    if (mask & (1UL << 1)) {
        vesc_buf_append_float16(out, 0.0f, 10, &i); /* motor temp not fitted */
    }
    if (mask & (1UL << 2)) {
        vesc_buf_append_float32(out, motor_current_from_raw(right,avg_id,avg_iq), 100, &i);
    }
    if (mask & (1UL << 3)) {
        vesc_buf_append_float32(out,(float)avg_dc_ca/100.0f,100,&i);
    }
    if (mask & (1UL << 4)) {
        vesc_buf_append_float16(out, duty(right), 1000, &i);
    }
    if (mask & (1UL << 5)) {
        vesc_buf_append_float32(out, (float)erpm(right), 1, &i);
    }
    if (mask & (1UL << 6)) {
        vesc_buf_append_float32(out, 0.0f, 1000, &i); /* speed m/s unavailable */
    }
    if (mask & (1UL << 7)) {
        vesc_buf_append_float16(out, (float)batVoltageCalib / 100.0f, 10, &i);
    }
    if (mask & (1UL << 8)) {
        vesc_buf_append_float16(out, 0.5f, 1000, &i); /* battery SOC not estimated */
    }
    for (int bit = 9; bit <= 14; ++bit) {
        if (mask & (1UL << bit)) {
            vesc_buf_append_float32(out, 0.0f, (bit <= 12) ? 10000 : 1000, &i);
        }
    }
    if (mask & (1UL << 15)) {
        vesc_buf_append_float32(out, pos_deg(right), 1000000, &i);
    }
    if (mask & (1UL << 16)) { wire_fault=fault_code(right); out[i++] = wire_fault; }
    if (mask & (1UL << 17)) out[i++] = node_id(right);
    if (mask & (1UL << 18)) out[i++] = 2U; /* two physical motor phases/controllers */
    if (mask & (1UL << 19)) {
        vesc_buf_append_float32(out, 0.0f, 1000, &i);
    }
    if (mask & (1UL << 20)) {
        vesc_buf_append_u32(out, 0U, &i); /* odometer in meters not calibrated */
    }
    if (mask & (1UL << 21)) {
        vesc_buf_append_u32(out, RuntimeControl_MonotonicMs(), &i);
    }

    record_values_wire(right,cmd,wire_fault,(uint16_t)i);
    send_payload(out, (uint16_t)i);
}

static int16_t current_permille(bool r,float a){mc_configuration*c=r?&motorConfRight:&motorConfLeft;float pos=fabsf(motor_amp(r,c->l_current_max));float neg=fabsf(motor_amp(r,c->l_current_min));float lim=a<0.0f?neg:pos;if(lim<0.01f)return 0;float p=a*1000.0f/lim;if(p>1000)p=1000;if(p<-1000)p=-1000;return(int16_t)lrintf(p);}
static int16_t brake_current_permille(bool r,float a){mc_configuration*c=r?&motorConfRight:&motorConfLeft;float lim=fabsf(motor_amp(r,c->l_current_min));if(lim<0.01f)lim=fabsf(motor_amp(r,c->l_current_max));if(lim<0.01f)return 0;float p=fabsf(a)*1000.0f/lim;if(p>1000)p=1000;return(int16_t)lrintf(p);}
static int16_t erpm_permille(bool r,int32_t e){mc_configuration*c=r?&motorConfRight:&motorConfLeft;float m=((float)c->l_max_speed_rpm_q4/16.0f)*(float)c->foc_motor_pole_pairs;if(m<1)m=1;float p=(float)e*1000.0f/m;if(p>1000)p=1000;if(p<-1000)p=-1000;return(int16_t)lrintf(p);}
static void record_set_wire(bool r,int32_t raw){if(r)vesc_last_set_host_raw_right=raw;else vesc_last_set_host_raw_left=raw;}
static void record_set_runtime(bool r,int32_t normalized,bool run){if(r){vesc_last_set_normalized_right=normalized;vesc_last_set_run_right=run?1U:0U;}else{vesc_last_set_normalized_left=normalized;vesc_last_set_run_left=run?1U:0U;}}
/* Protocol facade follows VESC commands.c: decode wire units, route to the
 * selected controller and refresh timeout. Runtime/core owns actuation. */
static void set_duty(bool r,float d){
    if (d > 1.0f) d = 1.0f;
    if (d < -1.0f) d = -1.0f;
    const int32_t normalized=(int32_t)lrintf(d*1000.0f);
    const bool run=normalized!=0;
    record_set_runtime(r,normalized,run);
    RuntimeControl_VescSetOne(!r,ESC_MODE_DUTY,normalized,run);
    RuntimeControl_VescAlive();last_uart_control_ms=RuntimeControl_MonotonicMs();
}
static void set_current(bool r,float a){
    const int16_t v=current_permille(r,a);const bool run=v!=0;
    record_set_runtime(r,v,run);RuntimeControl_VescSetOne(!r,ESC_MODE_TRQ,v,run);
    RuntimeControl_VescAlive();last_uart_control_ms=RuntimeControl_MonotonicMs();
}
static void set_brake(bool r,float a){
    const int16_t v=brake_current_permille(r,fabsf(a));const bool run=v!=0;
    record_set_runtime(r,v,run);RuntimeControl_VescSetOne(!r,ESC_MODE_BRAKE,v,run);
    RuntimeControl_VescAlive();last_uart_control_ms=RuntimeControl_MonotonicMs();
}
static void set_handbrake(bool r,float a){
    const int16_t v=current_permille(r,a);const bool run=v!=0;
    record_set_runtime(r,v,run);RuntimeControl_VescSetOne(!r,ESC_MODE_HANDBRAKE,v,run);
    RuntimeControl_VescAlive();last_uart_control_ms=RuntimeControl_MonotonicMs();
}
static void set_rpm(bool r,int32_t e){
    int16_t v = erpm_permille(r,e);
    if (e != 0 && v == 0) v = e > 0 ? 1 : -1;
    const bool run = e != 0;
    record_set_runtime(r,v,run);RuntimeControl_VescSetOne(!r,ESC_MODE_SPD,v,run);
    RuntimeControl_VescAlive();last_uart_control_ms=RuntimeControl_MonotonicMs();
}
static void set_pos(bool r,float deg){
    int32_t ticks=0;
    if (!RuntimeControl_PositionTargetTicks(!r,deg,&ticks)) return;
    record_set_runtime(r,ticks,true);
    RuntimeControl_VescSetOne(!r,ESC_MODE_POS,ticks,true);
    RuntimeControl_VescAlive();
    last_uart_control_ms=RuntimeControl_MonotonicMs();
}

static int16_t detect_current_internal(bool right, float current_a)
{
    const mc_configuration *c = right ? &motorConfRight : &motorConfLeft;
    const uint16_t units = c->foc_current_units_per_amp ? c->foc_current_units_per_amp : 800U;
    float a = fabsf(current_a);
    if (!isfinite(a) || a < 0.01f) a = 1.0f;
    if (a < 0.25f) a = 0.25f;
    if (a > 5.0f) a = 5.0f;

    int32_t internal = (int32_t)lrintf(a * (float)units);
    int32_t max_internal = c->l_current_max;
    if (max_internal < 0) max_internal = -max_internal;
    /* Keep commissioning below both the explicit 5-A cap and the configured
     * motor current limit. */
    if (internal > max_internal) internal = max_internal;
    if (internal < (int32_t)(units / 4U)) internal = (int32_t)(units / 4U);
    if (internal > INT16_MAX) internal = INT16_MAX;
    return (int16_t)internal;
}


static bool send_encoder_detect_failure(bool terminal_retry)
{
    uint8_t o[10];int32_t k=0;o[k++]=C_DETECT_ENCODER;
    vesc_buf_append_float32(o,1001.0f,1000000.0f,&k);
    vesc_buf_append_float32(o,0.0f,1000000.0f,&k);
    o[k++]=0U;
    return terminal_retry ? send_payload_terminal(o,(uint16_t)k) : send_payload(o,(uint16_t)k);
}

static void start_detect(uint8_t cmd,const uint8_t *data,uint16_t len,bool right)
{
    if(pending_detect.active)return; /* VESC blocking-command semantic. */
    int32_t j=0;
    const float current=(len>=4U)?vesc_buf_get_float32(data,1000.0f,&j):0.0f;
    const uint8_t sensor=(cmd==C_DETECT_ENCODER)?MOTOR_SENSOR_ENCODER_AB:MOTOR_SENSOR_HALL_UVW;

    pending_detect.active=true;
    pending_detect.command=cmd;
    pending_detect.right=right;
    pending_detect.sensor_type=sensor;
    pending_detect.immediate_failure=false;
    pending_detect.reply_retries=0U;
    last_uart_control_ms=RuntimeControl_MonotonicMs();

    /* RIGHT has no encoder hardware. Treat every start rejection as a terminal
     * asynchronous transaction rather than fire-and-forget. This makes even a
     * busy/full TX queue lossless: detect_service() will retry the failure reply. */
    if(right&&sensor==MOTOR_SENSOR_ENCODER_AB){
        pending_detect.immediate_failure=true;
        return;
    }
    RuntimeControl_VescAlive();
    if(!RuntimeControl_VescStartSensorDetect(!right,sensor,detect_current_internal(right,current))){
        pending_detect.immediate_failure=true;
        return;
    }
}

static void detect_service(void)
{
    if(!pending_detect.active)return;
    /* COMM_DETECT_* is blocking in upstream VESC. The bare-metal implementation
     * is asynchronous, so keep the watchdog alive and, once terminal, keep the
     * result pending until the UART TX ring has actually accepted the reply. */
    RuntimeControl_VescAlive();
    bool sent=false;
    if(pending_detect.immediate_failure){
        if(pending_detect.command==C_DETECT_ENCODER){
            sent=send_encoder_detect_failure(true);
        }else{
            uint8_t o[10];o[0]=C_DETECT_HALL_FOC;memset(o+1,255,8);o[9]=1U;
            sent=send_payload_terminal(o,10);
        }
        if(sent){pending_detect.active=false;}
        else if(pending_detect.reply_retries!=UINT16_MAX){++pending_detect.reply_retries;}
        return;
    }

    RuntimeVescDetectResult res;
    if(!RuntimeControl_VescPollSensorDetect(!pending_detect.right,pending_detect.sensor_type,&res))return;
    if(pending_detect.command==C_DETECT_HALL_FOC){
        uint8_t o[10];o[0]=C_DETECT_HALL_FOC;memcpy(o+1,res.hall_table,8);
        o[9]=(res.state==ESC_SENSOR_CAL_SUCCESS)?0U:1U;
        sent=send_payload_terminal(o,10);
    }else{
        if(res.state!=ESC_SENSOR_CAL_SUCCESS||res.encoder_cpr<MOTOR_ENCODER_CPR_MIN){
            sent=send_encoder_detect_failure(true);
        }else{
            uint8_t o[10];int32_t k=0;o[k++]=C_DETECT_ENCODER;
            vesc_buf_append_float32(o,(float)res.encoder_offset_deg,1000000.0f,&k);
            vesc_buf_append_float32(o,(float)res.pole_pairs,1000000.0f,&k);
            o[k++]=res.encoder_inverted?1U:0U;
            sent=send_payload_terminal(o,(uint16_t)k);
        }
    }
    if(sent){
        pending_detect.active=false;
    }else if(pending_detect.reply_retries!=UINT16_MAX){
        ++pending_detect.reply_retries;
    }
}


static float phase_deg_q16(uint16_t phase)
{
    return ((float)phase * 360.0f) / 65536.0f;
}

static float angle_diff_deg(float a, float b)
{
    float d = a - b;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

static bool rotor_position_value(bool right, uint8_t mode, float *value)
{
    if (value == NULL) return false;
    const motor_all_state_t *m = right ? &motorRight : &motorLeft;
    const MotorSensorSample *sample = right ? &motorSensorSampleRight : &motorSensorSampleLeft;
    const MotorRuntimeConfig *cfg = right ? &motorConfigRight : &motorConfigLeft;

    switch (mode) {
    case 1U: /* Inductance/detect: upstream only emits while MC_STATE_DETECTING. */
        if (RuntimeControl_SensorCalibrationState() == ESC_SENSOR_CAL_RUNNING &&
            RuntimeControl_SensorCalibrationMotor() == (right ? ESC_MOTOR_RIGHT : ESC_MOTOR_LEFT)) {
            *value = phase_deg_q16(RuntimeControl_SensorCalibrationElectricalPhaseQ16());
            return true;
        }
        if (RuntimeControl_EncoderAlignmentActive(!right)) {
            *value = phase_deg_q16(m->m_phase_now_override);
            return true;
        }
        return false;

    case 2U: /* Observer: do NOT alias sensor/active phase. This port has no flux observer yet. */
        return false;

    case 3U: /* Encoder raw mechanical angle, exactly the source class VESC uses. */
        if (cfg->sensor_type != MOTOR_SENSOR_ENCODER_AB) return false;
        *value = (float)sample->mechanical_angle_q4 / 16.0f;
        return true;

    case 4U: /* PID position now: logical/homed 0..360 coordinate. */
        *value = RuntimeControl_PositionDeg(!right);
        return true;

    case 5U: /* PID position error. */
        *value = RuntimeControl_PositionErrorDeg(!right);
        return true;

    case 6U: /* Observer vs Encoder requires a real observer. */
    case 7U: /* Observer vs Hall requires a real observer. */
        return false;

    default:
        return false;
    }
}

static bool send_rotor_position(bool right, uint8_t mode)
{
    float value = 0.0f;
    if (!rotor_position_value(right, mode, &value)) return false;
    uint8_t o[5]; int32_t k=0;
    o[k++]=C_ROTOR_POSITION;
    vesc_buf_append_float32(o,value,100000.0f,&k);
    return send_payload(o,(uint16_t)k);
}

static void rotor_position_stream_service(void)
{
    const uint32_t now=RuntimeControl_MonotonicMs();
    if(display_position_mode_left!=0U && (uint32_t)(now-display_position_last_left_ms)>=10U){
        if(send_rotor_position(false,display_position_mode_left))display_position_last_left_ms=now;
    }
    if(display_position_mode_right!=0U && (uint32_t)(now-display_position_last_right_ms)>=10U){
        if(send_rotor_position(true,display_position_mode_right))display_position_last_right_ms=now;
    }
}

static void auto_detect_reply_service(void)
{
    if(!auto_detect.active || auto_detect.stage!=AUTO_DETECT_REPLY)return;
    uint8_t o[3];int32_t k=0;o[k++]=C_DETECT_APPLY_ALL_FOC;vesc_buf_append_i16(o,auto_detect.result,&k);
    if(send_payload_terminal(o,(uint16_t)k))auto_detect.active=false;
    else if(auto_detect.reply_retries!=UINT16_MAX)++auto_detect.reply_retries;
}

static void auto_detect_start(bool right)
{
    if(right || auto_detect.active || pending_detect.active ||
       RuntimeControl_SensorCalibrationState()==ESC_SENSOR_CAL_RUNNING ||
       RuntimeControl_HomingActiveMask()!=0U){
        auto_detect.active=true;auto_detect.stage=AUTO_DETECT_REPLY;auto_detect.result=-1;auto_detect.reply_retries=0U;return;
    }
    RuntimeControl_VescReleaseAll();
    memset(&auto_detect,0,sizeof(auto_detect));
    auto_detect.active=true;
    auto_detect.stage=AUTO_DETECT_WAIT_CURRENT_CAL;
    auto_detect.result=-1;
    if(!MotorControl_RequestCurrentOffsetCalibration() && MotorControl_CurrentOffsetsValid()){
        if(RuntimeControl_VescStartSensorDetect(true,MOTOR_SENSOR_ENCODER_AB,detect_current_internal(false,0.5f)))
            auto_detect.stage=AUTO_DETECT_LEFT_ENCODER;
        else auto_detect.stage=AUTO_DETECT_REPLY;
    }
}

static void auto_detect_service(void)
{
    if(!auto_detect.active)return;
    RuntimeControl_VescAlive();
    RuntimeVescDetectResult res;
    switch(auto_detect.stage){
    case AUTO_DETECT_WAIT_CURRENT_CAL:
        if(MotorControl_CurrentOffsetCalState()==MOTOR_CURRENT_CAL_FAILED){auto_detect.stage=AUTO_DETECT_REPLY;break;}
        if(MotorControl_CurrentOffsetsValid()){
            if(RuntimeControl_VescStartSensorDetect(true,MOTOR_SENSOR_ENCODER_AB,detect_current_internal(false,0.5f)))
                auto_detect.stage=AUTO_DETECT_LEFT_ENCODER;
            else auto_detect.stage=AUTO_DETECT_REPLY;
        }
        break;
    case AUTO_DETECT_LEFT_ENCODER:
        if(RuntimeControl_VescPollSensorDetect(true,MOTOR_SENSOR_ENCODER_AB,&res)){
            if(res.state!=ESC_SENSOR_CAL_SUCCESS){auto_detect.stage=AUTO_DETECT_REPLY;break;}
            if(RuntimeControl_VescStartSensorDetect(false,MOTOR_SENSOR_HALL_UVW,detect_current_internal(true,0.5f)))
                auto_detect.stage=AUTO_DETECT_RIGHT_HALL;
            else auto_detect.stage=AUTO_DETECT_REPLY;
        }
        break;
    case AUTO_DETECT_RIGHT_HALL:
        if(RuntimeControl_VescPollSensorDetect(false,MOTOR_SENSOR_HALL_UVW,&res)){
            if(res.state!=ESC_SENSOR_CAL_SUCCESS){auto_detect.stage=AUTO_DETECT_REPLY;break;}
            if(RuntimeControl_RequestEncoderSync(true))auto_detect.stage=AUTO_DETECT_LEFT_SYNC;
            else if(RuntimeControl_EncoderElectricalReady(true)){
                auto_detect.result=RuntimeSettings_Save()?2:-1;
                auto_detect.stage=AUTO_DETECT_REPLY;
            } else auto_detect.stage=AUTO_DETECT_REPLY;
        }
        break;
    case AUTO_DETECT_LEFT_SYNC:
        if(!RuntimeControl_EncoderAlignmentActive(true)){
            if(RuntimeControl_EncoderElectricalReady(true)){
                /* Terminal success means runtime state AND EEPROM persistence
                 * succeeded. Never report result=2 and silently drop calibration. */
                auto_detect.result=RuntimeSettings_Save()?2:-1;
            }
            auto_detect.stage=AUTO_DETECT_REPLY;
        }
        break;
    case AUTO_DETECT_REPLY:
        auto_detect_reply_service();
        break;
    default:break;
    }
}

static void send_print(const char *text)
{
    if (text == NULL) return;
    uint8_t out[160];
    size_t n = strlen(text);
    if (n > sizeof(out) - 1U) n = sizeof(out) - 1U;
    out[0] = C_PRINT;
    memcpy(&out[1], text, n);
    send_payload(out, (uint16_t)(n + 1U));
}

static void terminal_command(const uint8_t *data, uint16_t len)
{
    char cmd[48];
    size_t n = len;
    if (n >= sizeof(cmd)) n = sizeof(cmd) - 1U;
    if (n > 0U && data != NULL) memcpy(cmd, data, n);
    cmd[n] = '\0';

    /* Strip trailing CR/LF and leading spaces. VESC Tool sends the terminal
     * command payload without requiring a trailing NUL. */
    while (n > 0U && (cmd[n - 1U] == '\r' || cmd[n - 1U] == '\n' || cmd[n - 1U] == ' ')) {
        cmd[--n] = '\0';
    }
    char *c = cmd;
    while (*c == ' ') ++c;

    if (strcmp(c, "hb_current_cal") == 0 || strcmp(c, "foc_dc_cal") == 0) {
        /* Safe VESC-Tool terminal entry point: never calibrate while a bridge is
         * commanded. Release both sides first, then let the ADC-DMA ISR perform
         * settling + 2048-sample zero-current averaging with MOE forced OFF. */
        RuntimeControl_VescReleaseAll();
        if (RuntimeControl_SensorCalibrationState() == ESC_SENSOR_CAL_RUNNING ||
            RuntimeControl_HomingActiveMask() != 0U ||
            RuntimeControl_EncoderAlignmentActive(true) ||
            RuntimeControl_EncoderAlignmentActive(false)) {
            send_print("HB current calibration rejected: commissioning active\n");
            return;
        }
        if (MotorControl_RequestCurrentOffsetCalibration()) {
            send_print("HB current calibration started; bridges released\n");
        } else {
            send_print("HB current calibration rejected: bridge/override active\n");
        }
        return;
    }

    if (strcmp(c, "hb_current_status") == 0) {
        switch (MotorControl_CurrentOffsetCalState()) {
        case MOTOR_CURRENT_CAL_IDLE:
            send_print("HB current calibration: IDLE\n");
            break;
        case MOTOR_CURRENT_CAL_SETTLING:
            send_print("HB current calibration: SETTLING\n");
            break;
        case MOTOR_CURRENT_CAL_COLLECTING:
            send_print("HB current calibration: COLLECTING\n");
            break;
        case MOTOR_CURRENT_CAL_VALID:
            send_print("HB current calibration: VALID\n");
            break;
        case MOTOR_CURRENT_CAL_FAILED:
            send_print("HB current calibration: FAILED (run the bundled Python full tester for details)\n");
            break;
        default:
            send_print("HB current calibration: UNKNOWN\n");
            break;
        }
        return;
    }

    if (strcmp(c, "hb_encoder_sync") == 0 || strcmp(c, "hb_encoder_sync_left") == 0) {
        RuntimeControl_VescReleaseAll();
        send_print(RuntimeControl_RequestEncoderSync(true) ?
            "HB LEFT encoder electrical sync started\n" :
            "HB LEFT encoder sync rejected (detect/config/busy)\n");
        return;
    }
    if (strcmp(c, "hb_home_cal") == 0 || strcmp(c, "hb_home_cal_left") == 0) {
        RuntimeControl_VescReleaseAll();
        send_print(RuntimeControl_StartHomingCalibration(true) ?
            "HB LEFT full 0..360 homing calibration started\n" :
            "HB LEFT homing calibration rejected\n");
        return;
    }
    if (strcmp(c, "hb_home") == 0 || strcmp(c, "hb_home_left") == 0) {
        RuntimeControl_VescReleaseAll();
        send_print(RuntimeControl_StartHomingOne(true) ?
            "HB LEFT one-stop homing started\n" :
            "HB LEFT homing rejected\n");
        return;
    }
    if (strcmp(c, "hb_home_cal_right") == 0) {
        RuntimeControl_VescReleaseAll();
        send_print(RuntimeControl_StartHomingCalibration(false) ?
            "HB RIGHT full 0..360 homing calibration started\n" :
            "HB RIGHT homing calibration rejected\n");
        return;
    }
    if (strcmp(c, "hb_home_right") == 0) {
        RuntimeControl_VescReleaseAll();
        send_print(RuntimeControl_StartHomingOne(false) ?
            "HB RIGHT one-stop homing started\n" :
            "HB RIGHT homing rejected\n");
        return;
    }
    if (strncmp(c, "hb_home_on ", 11) == 0) {
        const long on = strtol(c + 11, NULL, 10);
        RuntimeControl_VescReleaseAll();
        if (on != 0 && on != 1) send_print("HB LEFT homing-on expects 0 or 1\n");
        else send_print(RuntimeControl_SetHomingOnBoot(true, on != 0) ?
            (on ? "HB LEFT power-on homing ENABLED + saved\n" : "HB LEFT power-on homing DISABLED + saved\n") :
            "HB LEFT homing-on rejected (calibrate 0..360 first / busy / EEPROM)\n");
        return;
    }
    if (strncmp(c, "hb_home_on_right ", 17) == 0) {
        const long on = strtol(c + 17, NULL, 10);
        RuntimeControl_VescReleaseAll();
        if (on != 0 && on != 1) send_print("HB RIGHT homing-on expects 0 or 1\n");
        else send_print(RuntimeControl_SetHomingOnBoot(false, on != 0) ?
            (on ? "HB RIGHT power-on homing ENABLED + saved\n" : "HB RIGHT power-on homing DISABLED + saved\n") :
            "HB RIGHT homing-on rejected (calibrate 0..360 first / busy / EEPROM)\n");
        return;
    }
    if (strcmp(c, "hb_home_status") == 0) {
        char line[180];
        (void)snprintf(line, sizeof(line),
            "HB HOME L:cal=%u homed=%u on=%u zero=%ld span=%ld | R:cal=%u homed=%u on=%u zero=%ld span=%ld\n",
            (unsigned)steeringCalibrationLeft.calibrated, (unsigned)steeringCalibrationLeft.homed,
            RuntimeControl_HomingOnBoot(true) ? 1U : 0U,
            (long)steeringCalibrationLeft.right_zero_ticks, (long)steeringCalibrationLeft.span_ticks,
            (unsigned)steeringCalibrationRight.calibrated, (unsigned)steeringCalibrationRight.homed,
            RuntimeControl_HomingOnBoot(false) ? 1U : 0U,
            (long)steeringCalibrationRight.right_zero_ticks, (long)steeringCalibrationRight.span_ticks);
        send_print(line);
        return;
    }
    if (strcmp(c, "hb_auto_detect") == 0) {
        RuntimeControl_VescReleaseAll();
        auto_detect_start(false);
        send_print(auto_detect.active && auto_detect.stage != AUTO_DETECT_REPLY ?
            "HB integrated detect started: current-cal -> LEFT encoder -> RIGHT Hall -> LEFT sync -> EEPROM\n" :
            "HB integrated detect rejected/busy\n");
        return;
    }
    if (strncmp(c, "hb_set_pole_pairs ", 18) == 0) {
        long pp = strtol(c + 18, NULL, 10);
        if (pp < 1 || pp > 60 || RuntimeControl_Armed()) {
            send_print("HB pole-pairs rejected: use 1..60 while disarmed\n");
        } else {
            motorConfLeft.foc_motor_pole_pairs = (uint8_t)pp;
            motorConfRight.foc_motor_pole_pairs = (uint8_t)pp;
            MotorSensor_PrepareRuntime(&motorConfigLeft,&motorSensorStateLeft,motorConfLeft.foc_motor_pole_pairs);
            MotorSensor_PrepareRuntime(&motorConfigRight,&motorSensorStateRight,motorConfRight.foc_motor_pole_pairs);
            send_print(RuntimeSettings_Save() ? "HB pole-pairs saved (VESC Motor Poles = 2x)\n" : "HB pole-pairs changed in RAM, EEPROM save failed\n");
        }
        return;
    }
    if (strcmp(c, "hb_help") == 0) {
        send_print("HB: hb_current_cal/status, hb_auto_detect, hb_encoder_sync, hb_home, hb_home_cal, hb_home_on 0|1, hb_home_status, hb_set_pole_pairs N\n");
        return;
    }

    send_print("Unknown HB terminal command. Type hb_help\n");
}

static bool fw_append_cstr(uint8_t *dst, size_t cap, int32_t *index, const char *src)
{
    if (dst == NULL || index == NULL || src == NULL || *index < 0) return false;
    const size_t len = strlen(src) + 1U;
    if ((size_t)*index + len > cap) return false;
    memcpy(&dst[*index], src, len);
    *index += (int32_t)len;
    return true;
}

static void fw_version(bool r)
{
    /* V19 used 80 bytes with a 46-byte FW name and provably overflowed the
     * stack on COMM_FW_VERSION. Keep generous headroom and bounded appends. */
    uint8_t o[128];
    int32_t i=0;
    o[i++]=C_FW_VERSION;
    o[i++]=VESC_FW_MAJOR;
    o[i++]=VESC_FW_MINOR;
    if(!fw_append_cstr(o,sizeof(o),&i,"HOVERBOARD_DUAL_FOC")) return;
    volatile const uint32_t*uid=(volatile const uint32_t*)0x1FFFF7E8UL;
    if ((size_t)i + 12U + 8U >= sizeof(o)) return;
    for(int w=0;w<3;w++){
        uint32_t v=uid[w];
        o[i++]=(uint8_t)v;o[i++]=(uint8_t)(v>>8);o[i++]=(uint8_t)(v>>16);o[i++]=(uint8_t)(v>>24);
    }
    if(r)o[i-1]++;
    o[i++]=1; /* pairing done */
    o[i++]=0; /* test version */
    o[i++]=0; /* HW_TYPE_VESC */
    o[i++]=0; /* custom configs */
    o[i++]=0; /* phase filters */
    o[i++]=0; /* qml hw */
    o[i++]=0; /* qml app */
    o[i++]=0; /* nrf flags */
    if(!fw_append_cstr(o,sizeof(o),&i,"hoverboard-vesc6-v20")) return;
    send_payload(o,(uint16_t)i);
}

static void process_ctx(const uint8_t*p,uint16_t n,const vesc_motor_ctx_t *ctx){
 if(!p||n==0||ctx==NULL)return;
 const bool r=ctx->right;
 if(r)++vesc_route_packets_right; else ++vesc_route_packets_left;
 uint8_t cmd=p[0];
 if(cmd==C_SET_DUTY||cmd==C_SET_CURRENT||cmd==C_SET_CURRENT_BRAKE||
    cmd==C_SET_RPM||cmd==C_SET_POS||cmd==C_SET_HANDBRAKE||cmd==C_SET_CURRENT_REL){
     if(r){
         if(vesc_set_packets_right!=UINT32_MAX)++vesc_set_packets_right;
         vesc_last_set_command_right=cmd;
     }else{
         if(vesc_set_packets_left!=UINT32_MAX)++vesc_set_packets_left;
         vesc_last_set_command_left=cmd;
     }
 }
 const uint8_t*d=p+1;uint16_t l=n-1;int32_t j=0;switch(cmd){
 case C_FW_VERSION:fw_version(r);break;
 case C_GET_VALUES:case C_GET_VALUES_SELECTIVE:append_values(cmd,d,l,r);break;
 case C_GET_VALUES_SETUP:case C_GET_VALUES_SETUP_SELECTIVE:append_setup(cmd,d,l,r);break;
 case C_SET_DUTY:if(l>=4){int32_t raw=vesc_buf_get_i32(d,&j);record_set_wire(r,raw);set_duty(r,(float)raw/100000.0f);}break;
 case C_SET_CURRENT:if(l>=4){int32_t raw=vesc_buf_get_i32(d,&j);record_set_wire(r,raw);set_current(r,(float)raw/1000.0f);}break;
 case C_SET_CURRENT_BRAKE:if(l>=4){int32_t raw=vesc_buf_get_i32(d,&j);record_set_wire(r,raw);set_brake(r,(float)raw/1000.0f);}break;
 case C_SET_HANDBRAKE:if(l>=4){int32_t raw=vesc_buf_get_i32(d,&j);record_set_wire(r,raw);set_handbrake(r,(float)raw/1000.0f);}break;
 case C_SET_RPM:if(l>=4){int32_t raw=vesc_buf_get_i32(d,&j);record_set_wire(r,raw);set_rpm(r,raw);}break;
 case C_SET_POS:if(l>=4){int32_t raw=vesc_buf_get_i32(d,&j);record_set_wire(r,raw);set_pos(r,(float)raw/1000000.0f);}break;
 case C_SET_DETECT:if(l>=1){uint8_t mode=d[0];if(mode>7U)mode=0U;if(r){display_position_mode_right=mode;display_position_last_right_ms=0U;if(mode!=0U)display_position_mode_left=0U;}else{display_position_mode_left=mode;display_position_last_left_ms=0U;if(mode!=0U)display_position_mode_right=0U;}RuntimeControl_VescAlive();}break;
 case C_DETECT_ENCODER:case C_DETECT_HALL_FOC:start_detect(cmd,d,l,r);break;
 case C_DETECT_APPLY_ALL_FOC:auto_detect_start(r);break;
 case C_TERMINAL_CMD:terminal_command(d,l);break;
 case C_SET_CURRENT_REL:if(l>=4){int32_t raw=vesc_buf_get_i32(d,&j);record_set_wire(r,raw);float rel=(float)raw/100000.0f;mc_configuration*c=r?&motorConfRight:&motorConfLeft;set_current(r,rel*fabsf(motor_amp(r,c->l_current_max)));}break;
 case C_ALIVE:RuntimeControl_VescAlive();break;
 case C_ROTOR_POSITION:(void)send_rotor_position(r,r?display_position_mode_right:display_position_mode_left);break;
 case C_GET_DECODED_ADC:{uint8_t o[17];int32_t k=0;o[k++]=C_GET_DECODED_ADC;vesc_buf_append_i32(o,VescApp_GetDecoded1Micro(),&k);vesc_buf_append_i32(o,VescApp_GetVoltage1MicroV(),&k);vesc_buf_append_i32(o,VescApp_GetDecoded2Micro(),&k);vesc_buf_append_i32(o,VescApp_GetVoltage2MicroV(),&k);send_payload(o,(uint16_t)k);}break;
 case C_GET_MCCONF:case C_GET_MCCONF_DEFAULT:{uint8_t o[VESC_PACKET_MAX_PAYLOAD];o[0]=cmd;int32_t k=VescConfig_SerializeMc(o+1,r,cmd==C_GET_MCCONF_DEFAULT);if(k>0)send_payload(o,(uint16_t)(k+1));}break;
 case C_SET_MCCONF:{if(VescConfig_DeserializeMc(d,l,r,true)){uint8_t o[1]={C_SET_MCCONF};send_payload(o,1);}}break;
 case C_GET_APPCONF:case C_GET_APPCONF_DEFAULT:{uint8_t o[VESC_PACKET_MAX_PAYLOAD];o[0]=cmd;int32_t k=VescConfig_SerializeApp(o+1,r,cmd==C_GET_APPCONF_DEFAULT);if(k>0)send_payload(o,(uint16_t)(k+1));}break;
 case C_SET_APPCONF:{if(VescConfig_DeserializeApp(d,l,r,true)){uint8_t o[1]={C_SET_APPCONF};send_payload(o,1);}}break;
 case C_CUSTOM_APP_DATA:custom_app_data(d,l,r);break;
 case C_FORWARD_CAN:
     /* Mirror VESC dual-motor semantics: only the virtual second ID switches
      * context locally. Never recurse through a naked bool that can accidentally
      * mix LEFT/RIGHT telemetry or setpoints. */
     if(!r && l>=1 && d[0]==second_id())
         process_ctx(d+1,(uint16_t)(l-1),&vesc_ctx_right);
     break;
 case C_PING_CAN:{uint8_t o[2]={C_PING_CAN,second_id()};send_payload(o,2);}break;
 default:break;}}

static void rx_cb(const uint8_t*p,uint16_t n){++rx_packets;process_ctx(p,n,&vesc_ctx_left);}
void VescProtocol_Init(void){VescPacket_Init(&parser);memset(rx_dma,0,sizeof(rx_dma));rx_old=0;tx_head=tx_tail=0;tx_busy=false;if(!protocol_initialized){pending_detect.active=false;auto_detect.active=false;display_position_mode_left=display_position_mode_right=0U;protocol_initialized=true;}DMA1_Channel2->CCR&=~DMA_CCR_EN;DMA1->IFCR=DMA_IFCR_CGIF2;DMA1_Channel2->CPAR=(uint32_t)(uintptr_t)&USART3->DR;DMA1_Channel2->CNDTR=0;DMA1_Channel2->CCR=DMA_CCR_DIR|DMA_CCR_MINC|DMA_CCR_TCIE|DMA_CCR_TEIE;DMA1_Channel3->CCR&=~DMA_CCR_EN;DMA1->IFCR=DMA_IFCR_CGIF3;DMA1_Channel3->CPAR=(uint32_t)(uintptr_t)&USART3->DR;DMA1_Channel3->CMAR=(uint32_t)(uintptr_t)rx_dma;DMA1_Channel3->CNDTR=RX_DMA_SIZE;DMA1_Channel3->CCR=DMA_CCR_MINC|DMA_CCR_CIRC;SET_BIT(USART3->CR3,USART_CR3_DMAR);DMA1_Channel3->CCR|=DMA_CCR_EN;}
void VescProtocol_Service(void){uint32_t f=DMA1->ISR;if(tx_busy&&(f&(DMA_ISR_TCIF2|DMA_ISR_TEIF2)))VescProtocol_TxDmaIrqHandler();if((f&DMA_ISR_TEIF3)||(DMA1_Channel3->CCR&DMA_CCR_EN)==0){++crc_or_parser_errors;VescProtocol_Init();return;}detect_service();auto_detect_service();uint16_t pos=(uint16_t)(RX_DMA_SIZE-DMA1_Channel3->CNDTR);while(rx_old!=pos){VescPacket_Feed(&parser,rx_dma[rx_old],rx_cb);rx_old++;if(rx_old>=RX_DMA_SIZE)rx_old=0;}detect_service();auto_detect_service();rotor_position_stream_service();tx_start_next();}
void VescProtocol_AdcSetNormalized(int16_t p,bool speed){
    uint32_t now=RuntimeControl_MonotonicMs();
    if((uint32_t)(now-last_uart_control_ms)<UART_OVERRIDE_MS)return;
    if (p > 1000) p = 1000;
    if (p < -1000) p = -1000;
    uint8_t mode=speed?ESC_MODE_SPD:
        ((vescAppConfig.adc_ctrl_type>=VESC_ADC_DUTY&&
          vescAppConfig.adc_ctrl_type<=VESC_ADC_DUTY_REV_BUTTON)?ESC_MODE_DUTY:ESC_MODE_TRQ);
    const bool run=(p>1||p<-1);
    RuntimeControl_VescSetOne(true,mode,p,run);
    if(vescAppConfig.multi_esc)RuntimeControl_VescSetOne(false,mode,p,run);
}
