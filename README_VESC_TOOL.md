# VESC Tool Notes — V20

## Motor poles

Hoverboard motor physical pole-pairs remain 15, therefore VESC Tool **Motor Poles
= 30**. Encoder ratio is an independent value and is normally 15 for this motor.

## Rotor Position buttons

V20 sources are intentionally distinct:

- **Inductance / Detect**: phase being forced while commissioning is actually active.
  Outside detection there is no fabricated inductance position stream.
- **Observer**: no stream in V20. A VESC observer is a real flux-model state; V20
  does not alias Hall/encoder phase and call it observer.
- **Encoder**: raw mechanical encoder angle on the LEFT encoder context.
- **PID Pos**: current logical 0..360 position.
- **PID Error**: target minus current logical position.
- **Obs vs Enc / Obs vs Hall**: no stream until a real observer exists.

The stream uses standard `COMM_ROTOR_POSITION`, degree * 100000, at about 100 Hz.

## Vd / Vq

V20 implements standard GET_VALUES Vd/Vq. They are FOC d/q voltages computed from
the controller's d/q modulation and measured DC bus voltage, not a separate BEMF
sensor input. They are sampled/averaged in the slow telemetry path.

## Position

`SET_POS` receives degrees from VESC Tool and maps 0..360 to a signed mechanical
sensor span exactly once. Motor-direction inversion is not applied a second time
to the absolute raw target.

## Detection Result

Sensor labels are meaningful:
- ID 10: Encoder
- ID 11: Hall Sensors

The displayed R/L/flux fields are not yet measurements from a full VESC R/L/flux
auto-detect. V20's integrated operation is the board sensor/current-offset
commissioning path.
