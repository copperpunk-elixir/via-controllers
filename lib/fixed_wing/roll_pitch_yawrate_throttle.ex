defmodule ViaControllers.FixedWing.RollPitchDeltayawThrottle do
  require Logger

  defstruct roll_scalar: nil, pitch_scalar: nil, deltayaw_scalar: nil, throttle_scalar: nil

  @spec new(list()) :: struct()
  def new(full_config) do
    roll_config = Keyword.fetch!(full_config, :roll)
    pitch_config = Keyword.fetch!(full_config, :pitch)
    deltayaw_config = Keyword.fetch!(full_config, :deltayaw)
    throttle_config = Keyword.fetch!(full_config, :throttle)
    new(roll_config, pitch_config, deltayaw_config, throttle_config)
  end

  @spec new(list(), list(), list(), list()) :: struct()
  def new(roll_config, pitch_config, deltayaw_config, throttle_config) do
    roll_scalar = Scalar.new(roll_config)
    pitch_scalar = Scalar.new(pitch_config)
    deltayaw_scalar = Scalar.new(deltayaw_config)
    throttle_scalar = Scalar.new(throttle_config)

    %ViaControllers.FixedWing.RollPitchDeltayawThrottle{
      roll_scalar: roll_scalar,
      pitch_scalar: pitch_scalar,
      deltayaw_scalar: deltayaw_scalar,
      throttle_scalar: throttle_scalar
    }
  end

  @spec update(struct(), map(), map(), number(), number()) :: tuple()
  def update(controller, commands, values, _airspeed, dt) do
    {roll_scalar, rollrate_output_rps} = Scalar.update(controller.roll_scalar, commands.roll_rad, values.roll_rad, dt)
    {pitch_scalar, pitchrate_output_rps} = Scalar.update(controller.pitch_scalar, commands.pitch_rad, values.pitch_rad, dt)

    {deltayaw_scalar, yawrate_output_rps} =
      Scalar.update(controller.deltayaw_scalar, commands.deltayaw_rad, 0, dt)

    {throttle_scalar, throttle_output_scaled} =
      Scalar.update(controller.throttle_scalar, commands.throttle_scaled, 0, dt)

    output = %{
      rollrate_rps: rollrate_output_rps,
      pitchrate_rps: pitchrate_output_rps,
      yawrate_rps: yawrate_output_rps,
      throttle_scaled: throttle_output_scaled
    }
    {%{
       controller
       | roll_scalar: roll_scalar,
         pitch_scalar: pitch_scalar,
         deltayaw_scalar: deltayaw_scalar,
         throttle_scalar: throttle_scalar
     }, output}
  end

  @spec output(struct()) :: map()
  def output(controller) do
    %{
      rollrate_rps: Scalar.output(controller.roll_scalar),
      pitchrate_rps: Scalar.output(controller.pitch_scalar),
      yawrate_rps: Scalar.output(controller.deltayaw_scalar),
      throttle_scaled: Scalar.output(controller.throttle_scalar)
    }
  end
end
