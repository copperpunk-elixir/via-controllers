defmodule ViaControllers.FixedWing.RollPitchDeltayawThrust do
  require ViaUtils.Shared.ValueNames, as: SVN
  require ViaUtils.Shared.GoalNames, as: SGN
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

  @spec update(struct(), map(), map()) :: tuple()
  def update(controller, commands, values) do
    %{SVN.roll_rad() => roll_rad, SVN.pitch_rad() => pitch_rad, SVN.dt_s() => dt_s} = values

    %{
      SGN.roll_rad() => cmd_roll_rad,
      SGN.pitch_rad() => cmd_pitch_rad,
      SGN.deltayaw_rad() => cmd_deltayaw_rad,
      SGN.thrust_scaled() => cmd_thrust_scaled
    } = commands

    {roll_scalar, rollrate_output_rps} =
      Scalar.update(controller.roll_scalar, cmd_roll_rad, roll_rad, dt_s)

    {pitch_scalar, pitchrate_output_rps} =
      Scalar.update(controller.pitch_scalar, cmd_pitch_rad, pitch_rad, dt_s)

    {deltayaw_scalar, yawrate_output_rps} =
      Scalar.update(controller.deltayaw_scalar, cmd_deltayaw_rad, 0, dt_s)

    {thrust_scalar, throttle_output_scaled} =
      Scalar.update(controller.thrust_scalar, cmd_thrust_scaled, 0, dt_s)

    output =
      Map.drop(commands, [
        SGN.roll_rad(),
        SGN.pitch_rad(),
        SGN.deltayaw_rad(),
        SGN.thrust_scaled()
      ])
      |> Map.merge(%{
        SGN.rollrate_rps() => rollrate_output_rps,
        SGN.pitchrate_rps() => pitchrate_output_rps,
        SGN.yawrate_rps() => yawrate_output_rps,
        SGN.throttle_scaled() => throttle_output_scaled
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
      SGN.rollrate_rps() => Scalar.output(controller.roll_scalar),
      SGN.pitchrate_rps() => Scalar.output(controller.pitch_scalar),
      SGN.yawrate_rps() => Scalar.output(controller.deltayaw_scalar),
      SGN.throttle_scaled() => Scalar.output(controller.thrust_scalar)
    }
  end
end
