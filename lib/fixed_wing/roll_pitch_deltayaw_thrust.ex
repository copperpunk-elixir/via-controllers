defmodule ViaControllers.FixedWing.RollPitchDeltayawThrust do
  require ViaUtils.Shared.ValueNames, as: SVN
  require ViaUtils.Shared.GoalNames, as: SGN
  require ViaUtils.Shared.ControlTypes, as: SCT
  require Logger

  defstruct roll_rollrate_scalar: nil,
            pitch_pitchrate_scalar: nil,
            deltayaw_yawrate_scalar: nil,
            thrust_throttle_scalar: nil

  @spec new(list()) :: struct()
  def new(full_config) do
    roll_config = Keyword.fetch!(full_config, SCT.roll_rollrate_scalar())
    pitch_config = Keyword.fetch!(full_config, SCT.pitch_pitchrate_scalar())
    deltayaw_config = Keyword.fetch!(full_config, SCT.deltayaw_yawrate_scalar())
    thrust_config = Keyword.fetch!(full_config, SCT.thrust_throttle_scalar())
    new(roll_config, pitch_config, deltayaw_config, thrust_config)
  end

  @spec new(list(), list(), list(), list()) :: struct()
  def new(roll_config, pitch_config, deltayaw_config, thrust_config) do
    roll_rollrate_scalar = Scalar.new(roll_config)
    pitch_pitchrate_scalar = Scalar.new(pitch_config)
    deltayaw_yawrate_scalar = Scalar.new(deltayaw_config)
    thrust_throttle_scalar = Scalar.new(thrust_config)

    %ViaControllers.FixedWing.RollPitchDeltayawThrust{
      roll_rollrate_scalar: roll_rollrate_scalar,
      pitch_pitchrate_scalar: pitch_pitchrate_scalar,
      deltayaw_yawrate_scalar: deltayaw_yawrate_scalar,
      thrust_throttle_scalar: thrust_throttle_scalar
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

    %{
      roll_rollrate_scalar: roll_rollrate_scalar,
      pitch_pitchrate_scalar: pitch_pitchrate_scalar,
      deltayaw_yawrate_scalar: deltayaw_yawrate_scalar,
      thrust_throttle_scalar: thrust_throttle_scalar
    } = controller

    {roll_rollrate_scalar, rollrate_output_rps} =
      Scalar.update(roll_rollrate_scalar, cmd_roll_rad, roll_rad, dt_s)

    {pitch_pitchrate_scalar, pitchrate_output_rps} =
      Scalar.update(pitch_pitchrate_scalar, cmd_pitch_rad, pitch_rad, dt_s)

    {deltayaw_yawrate_scalar, yawrate_output_rps} =
      Scalar.update(deltayaw_yawrate_scalar, cmd_deltayaw_rad, 0, dt_s)

    {thrust_throttle_scalar, throttle_output_scaled} =
      Scalar.update(thrust_throttle_scalar, cmd_thrust_scaled, 0, dt_s)

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
       | roll_rollrate_scalar: roll_rollrate_scalar,
         pitch_pitchrate_scalar: pitch_pitchrate_scalar,
         deltayaw_yawrate_scalar: deltayaw_yawrate_scalar,
         thrust_throttle_scalar: thrust_throttle_scalar
     }, output}
  end

  @spec output(struct()) :: map()
  def output(controller) do
    %{
      roll_rollrate_scalar: roll_rollrate_scalar,
      pitch_pitchrate_scalar: pitch_pitchrate_scalar,
      deltayaw_yawrate_scalar: deltayaw_yawrate_scalar,
      thrust_throttle_scalar: thrust_throttle_scalar
    } = controller

    %{
      SGN.rollrate_rps() => Scalar.output(roll_rollrate_scalar),
      SGN.pitchrate_rps() => Scalar.output(pitch_pitchrate_scalar),
      SGN.yawrate_rps() => Scalar.output(deltayaw_yawrate_scalar),
      SGN.throttle_scaled() => Scalar.output(thrust_throttle_scalar)
    }
  end
end
