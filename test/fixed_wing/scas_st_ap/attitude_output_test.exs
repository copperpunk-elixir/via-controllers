defmodule FixedWing.ScasTsPa.AttitudeOutputTest do
  use ExUnit.Case
  require Logger
  alias ViaControllers.FixedWing.ScasStAp
  require ViaUtils.Shared.ValueNames, as: SVN
  require ViaUtils.Shared.GoalNames, as: SGN

  setup do
    {:ok, []}
  end

  test "Attitude Output Test" do
    controller_config = [
      min_airspeed_for_climb_mps: 20,
      feed_forward_speed_max_mps: 65.0,
      speed_thrust: [
        kp: 0.0,
        ki: 0.1,
        kd: 0,
        output_min: -1.0,
        output_neutral: -1.0,
        output_max: 1.0,
        feed_forward_speed_max_mps: 65,
        integrator_range: 0.26,
        integrator_airspeed_min_mps: 5.0
      ],
      altitude_pitch: [
        kp: 0.0,
        ki: 0.0,
        kd: 0,
        output_min: -0.78,
        output_neutral: 0,
        output_max: 0.52,
        integrator_range: 0.26,
        integrator_airspeed_min_mps: 5.0
      ],
      roll_course: [
        kp: 0.25,
        ki: 0.0,
        kd: 0,
        time_constant_s: 4,
        integrator_range: 0.052,
        integrator_airspeed_min_mps: 5.0,
        output_min: -0.78,
        output_max: 0.78,
        output_neutral: 0.0
      ]
    ]

    controller = ScasStAp.new(controller_config)

    dt_s = 0.1

    values = %{
      SVN.course_rad() => 0,
      SVN.yaw_rad() => 0,
      SVN.groundspeed_mps() => 0,
      SVN.airspeed_mps() => 30,
      SVN.dt_s() => dt_s,
      SVN.vertical_velocity_mps() => 0,
      SVN.altitude_m() => 0
    }

    commands = %{
      SGN.sideslip_rad() => -0.1,
      SGN.course_rad() => 0.1,
      SGN.altitude_m() => 5,
      SGN.groundspeed_mps() => 50
    }

    {_controller, output} = ScasStAp.update(controller, commands, values)

    Logger.debug("output: #{ViaUtils.Format.eftb_map(output, 3)}")

    assert output.roll_rad > 0
    assert output.pitch_rad > 0
    assert output.deltayaw_rad < 0
    assert output.thrust_scaled > -1
    Process.sleep(100)
  end
end
