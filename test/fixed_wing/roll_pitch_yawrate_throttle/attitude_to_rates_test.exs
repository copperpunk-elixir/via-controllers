defmodule FixedWing.RollPitchDeltayawThrust.RateControlTest do
  use ExUnit.Case
  require Logger
  require ViaUtils.Shared.GoalNames, as: SGN
  require ViaUtils.Shared.ValueNames, as: SVN
  alias ViaControllers.FixedWing.RollPitchDeltayawThrust

  setup do
    {:ok, []}
  end

  test "Rate Control Test" do
    full_config = [
      roll: [
        output_min: -6.0,
        output_neutral: 0,
        output_max: 6.0,
        multiplier: 2.0,
        command_rate_max: 0.5,
        initial_command: 0
      ],
      pitch: [
        output_min: -3.0,
        output_neutral: 0,
        output_max: 3.0,
        multiplier: 2.0,
        command_rate_max: 0.5,
        initial_command: 0
      ],
      deltayaw: [
        output_min: -3.0,
        output_neutral: 0,
        output_max: 3.0,
        multiplier: 1.0,
        command_rate_max: 1.5,
        initial_command: 0
      ],
      thrust: [
        output_min: 0.0,
        output_neutral: 0.0,
        output_max: 1.0,
        multiplier: 1.0,
        command_rate_max: 0.5,
        initial_command: 0
      ]
    ]

    rpyt = RollPitchDeltayawThrust.new(full_config)

    dt = 1.0

    commands = %{
      SGN.roll_rad() => 0.4,
      SGN.pitch_rad() => -0.2,
      SGN.deltayaw_rad() => 2.0,
      SGN.thrust_scaled() => 0.7
    }

    values = %{
      SVN.roll_rad() => 0,
      SVN.pitch_rad() => 0,
      SVN.yaw_rad() => 0,
      SVN.dt_s() => dt,
      SVN.airspeed_mps() => 0
    }

    {rpyt, output} = RollPitchDeltayawThrust.update(rpyt, commands, values)
    # Two outputs that were not rate limited
    assert output.rollrate_rps ==
             (Map.fetch!(commands, SGN.roll_rad) - Map.fetch!(values, SVN.roll_rad)) * full_config[:roll][:multiplier]

    assert output.pitchrate_rps ==
             (Map.fetch!(commands, SGN.pitch_rad) - Map.fetch!(values, SVN.pitch_rad)) * full_config[:pitch][:multiplier]

    # And two that were
    assert output.yawrate_rps == full_config[:deltayaw][:command_rate_max]
    assert output.throttle_scaled == full_config[:thrust][:command_rate_max]
    Logger.debug(inspect(output))
  end
end
