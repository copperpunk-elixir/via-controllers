defmodule FixedWing.SpeedCourseAltitudeSideslip.AttitudeOutputTest do
  use ExUnit.Case
  require Logger
  alias ViaControllers.FixedWing.SpeedCourseAltitudeSideslip
  require ViaUtils.Constants, as: VC

  setup do
    {:ok, []}
  end

  test "Attitude Output Test" do
    controller_config = [
      tecs_energy: [
        ki: 0.25,
        kd: 0,
        altitude_kp: 1.0,
        energy_rate_scalar: 0.004,
        integrator_range: 100,
        feed_forward_speed_max_mps: 60.0,
        output_min: -1.0,
        output_max: 1.0,
        output_neutral: -1.0
      ],
      tecs_balance: [
        ki: 0.1,
        kd: 0.0,
        altitude_kp: 0.75,
        balance_rate_scalar: 0.5,
        time_constant: 2.0,
        integrator_range: 0.4,
        integrator_factor: 5.0,
        min_airspeed_for_climb_mps: 10,
        output_min: -0.78,
        output_max: 0.52,
        output_neutral: 0.0
      ],
      roll_course: [
        kp: 0.25,
        ki: 0.0,
        integrator_range: 0.052,
        integrator_airspeed_min: 5.0,
        output_min: -0.78,
        output_max: 0.78,
        output_neutral: 0.0
      ]
    ]

    controller = SpeedCourseAltitudeSideslip.new(controller_config)

    values = %{
      course_rad: 0,
      groundspeed_mps: 0,
      altitude_m: 0,
      vertical_velocity_mps: 0,
      yaw_rad: 0
    }

    commands = %{
      course_rad: 0.1,
      groundspeed_mps: 10,
      altitude_m: 5,
      sideslip_rad: -0.1
    }

    airspeed_mps = 0
    dt_s = 0.1

    {controller, output} =
      ViaControllers.FixedWing.SpeedCourseAltitudeSideslip.update(
        controller,
        commands,
        values,
        airspeed_mps,
        dt_s
      )

    Logger.debug("output: #{ViaUtils.Format.eftb_map(output, 3)}")

    assert output.roll_rad > 0
    assert output.pitch_rad > 0
    assert output.deltayaw_rad < 0
    assert output.thrust_scaled > -1
    Process.sleep(100)
  end
end
