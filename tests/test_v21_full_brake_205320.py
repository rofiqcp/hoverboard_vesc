from pathlib import Path
R=Path(__file__).resolve().parents[1]
def s(p): return (R/p).read_text()
def test_zero_duty_output_mapping():
 f=s("Src/foc_motor.c"); m=s("Src/motor.c"); h=s("Src/foc_motor.h")
 assert "bool zero_duty_phase_brake" in h
 assert "output->zero_duty_phase_brake = true" in f
 assert "if (out->zero_duty_phase_brake)" in m
 assert "set_current_zero_vector_left();" in m and "set_current_zero_vector_right();" in m
