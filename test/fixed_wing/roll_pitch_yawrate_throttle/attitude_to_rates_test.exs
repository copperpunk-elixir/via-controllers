defmodule FixedWing.RollPitchYawrateThrottle.RateControlTest do
  use ExUnit.Case
  require Logger
  alias ViaControllers.FixedWing.RollPitchYawrateThrottle

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
      yawrate: [
        output_min: -3.0,
        output_neutral: 0,
        output_max: 3.0,
        multiplier: 1.0,
        command_rate_max: 1.5,
        initial_command: 0
      ],
      throttle: [
        output_min: 0.0,
        output_neutral: 0.0,
        output_max: 1.0,
        multiplier: 1.0,
        command_rate_max: 0.5,
        initial_command: 0
      ]
    ]

    rpyt =
      RollPitchYawrateThrottle.new(full_config)

    commands = %{
      roll_rad: 0.4,
      pitch_rad: -0.2,
      yawrate_rps: 2.0,
      throttle_scaled: 0.7
    }

    values = %{
      roll_rad: 0,
      pitch_rad: 0,
      yawrate_rps: 0,
      throttle_scaled: 0
    }

    dt = 1.0
    rpyt = RollPitchYawrateThrottle.update(rpyt, commands, values, 0, dt)
    output = RollPitchYawrateThrottle.output(rpyt)
    # Two outputs that were not rate limited
    assert output.rollrate_rps == (commands.roll_rad - values.roll_rad) * full_config[:roll][:multiplier]

    assert output.pitchrate_rps ==
             (commands.pitch_rad - values.pitch_rad) * full_config[:pitch][:multiplier]

    # And two that were
    assert output.yawrate_rps == full_config[:yawrate][:command_rate_max]
    assert output.throttle_scaled == full_config[:throttle][:command_rate_max]
    Logger.debug(inspect(output))
  end
end
