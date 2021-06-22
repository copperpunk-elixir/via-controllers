defmodule ViaControllers.FixedWing.RollPitchYawrateThrottle do
  require Logger

  defstruct roll_scalar: nil, pitch_scalar: nil, yawrate_scalar: nil, throttle_scalar: nil

  @spec new(list()) :: struct()
  def new(full_config) do
    roll_config = Keyword.fetch!(full_config, :roll)
    pitch_config = Keyword.fetch!(full_config, :pitch)
    yawrate_config = Keyword.fetch!(full_config, :yawrate)
    throttle_config = Keyword.fetch!(full_config, :throttle)
    new(roll_config, pitch_config, yawrate_config, throttle_config)
  end

  @spec new(list(), list(), list(), list()) :: struct()
  def new(roll_config, pitch_config, yawrate_config, throttle_config) do
    roll_scalar = Scalar.new(roll_config)
    pitch_scalar = Scalar.new(pitch_config)
    yawrate_scalar = Scalar.new(yawrate_config)
    throttle_scalar = Scalar.new(throttle_config)

    %ViaControllers.FixedWing.RollPitchYawrateThrottle{
      roll_scalar: roll_scalar,
      pitch_scalar: pitch_scalar,
      yawrate_scalar: yawrate_scalar,
      throttle_scalar: throttle_scalar
    }
  end

  @spec update(struct(), map(), map(), number(), number()) :: tuple()
  def update(rpyt, commands, values, _airspeed, dt) do
    roll_scalar = Scalar.update(rpyt.roll_scalar, commands.roll_rad, values.roll_rad, dt)
    pitch_scalar = Scalar.update(rpyt.pitch_scalar, commands.pitch_rad, values.pitch_rad, dt)

    yawrate_scalar =
      Scalar.update(rpyt.yawrate_scalar, commands.yawrate_rps, values.yawrate_rps, dt)

    throttle_scalar =
      Scalar.update(rpyt.throttle_scalar, commands.throttle_scaled, values.throttle_scaled, dt)

    {%{
       rpyt
       | roll_scalar: roll_scalar,
         pitch_scalar: pitch_scalar,
         yawrate_scalar: yawrate_scalar,
         throttle_scalar: throttle_scalar
     }, output(rpyt)}
  end

  @spec output(struct()) :: map()
  def output(rpyt) do
    %{
      rollrate_rps: Scalar.output(rpyt.roll_scalar),
      pitchrate_rps: Scalar.output(rpyt.pitch_scalar),
      yawrate_rps: Scalar.output(rpyt.yawrate_scalar),
      throttle_scaled: Scalar.output(rpyt.throttle_scalar)
    }
  end
end
