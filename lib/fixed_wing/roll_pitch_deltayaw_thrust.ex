defmodule ViaControllers.FixedWing.RollPitchDeltayawThrust do
  require Logger

  defstruct roll_scalar: nil, pitch_scalar: nil, deltayaw_scalar: nil, thrust_scalar: nil

  @spec new(list()) :: struct()
  def new(full_config) do
    roll_config = Keyword.fetch!(full_config, :roll)
    pitch_config = Keyword.fetch!(full_config, :pitch)
    deltayaw_config = Keyword.fetch!(full_config, :deltayaw)
    thrust_config = Keyword.fetch!(full_config, :thrust)
    new(roll_config, pitch_config, deltayaw_config, thrust_config)
  end

  @spec new(list(), list(), list(), list()) :: struct()
  def new(roll_config, pitch_config, deltayaw_config, thrust_config) do
    roll_scalar = Scalar.new(roll_config)
    pitch_scalar = Scalar.new(pitch_config)
    deltayaw_scalar = Scalar.new(deltayaw_config)
    thrust_scalar = Scalar.new(thrust_config)

    %ViaControllers.FixedWing.RollPitchDeltayawThrust{
      roll_scalar: roll_scalar,
      pitch_scalar: pitch_scalar,
      deltayaw_scalar: deltayaw_scalar,
      thrust_scalar: thrust_scalar
    }
  end

  @spec update(struct(), map(), map(), number(), number()) :: tuple()
  def update(controller, commands, values, _airspeed, dt) do
    {roll_scalar, rollrate_output_rps} =
      Scalar.update(controller.roll_scalar, commands.roll_rad, values.roll_rad, dt)

    {pitch_scalar, pitchrate_output_rps} =
      Scalar.update(controller.pitch_scalar, commands.pitch_rad, values.pitch_rad, dt)

    {deltayaw_scalar, yawrate_output_rps} =
      Scalar.update(controller.deltayaw_scalar, commands.deltayaw_rad, 0, dt)

    {thrust_scalar, throttle_output_scaled} =
      Scalar.update(controller.thrust_scalar, commands.thrust_scaled, 0, dt)

    output =
      Map.drop(commands, [:roll_rad, :pitch_rad, :deltayaw_rad, :thrust_scaled])
      |> Map.merge(%{
        rollrate_rps: rollrate_output_rps,
        pitchrate_rps: pitchrate_output_rps,
        yawrate_rps: yawrate_output_rps,
        throttle_scaled: throttle_output_scaled
      })

    {%{
       controller
       | roll_scalar: roll_scalar,
         pitch_scalar: pitch_scalar,
         deltayaw_scalar: deltayaw_scalar,
         thrust_scalar: thrust_scalar
     }, output}
  end

  @spec output(struct()) :: map()
  def output(controller) do
    %{
      rollrate_rps: Scalar.output(controller.roll_scalar),
      pitchrate_rps: Scalar.output(controller.pitch_scalar),
      yawrate_rps: Scalar.output(controller.deltayaw_scalar),
      throttle_scaled: Scalar.output(controller.thrust_scalar)
    }
  end
end
