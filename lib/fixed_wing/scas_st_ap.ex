defmodule ViaControllers.FixedWing.ScasStAp do
  require Logger
  require ViaUtils.Constants, as: VC
  require ViaUtils.Shared.ValueNames, as: SVN
  require ViaUtils.Shared.GoalNames, as: SGN
  require ViaUtils.Shared.ControlTypes, as: SCT

  defstruct [
    :speed_thrust_pid,
    :altitude_pitch_pid,
    :course_roll_pid,
    :min_airspeed_for_climb_mps
  ]

  @spec new(list()) :: struct()
  def new(config) do
    # Speed-Thrust
    speed_thrust_config = config[SCT.speed_thrust_pid()]
    ff_speed_max_mps = Keyword.get(speed_thrust_config, SCT.feed_forward_speed_max_mps(), 0)

    speed_thrust_ff_function =
      if ff_speed_max_mps > 0 do
        Logger.debug("good speed max")

        fn cmd, _airspeed ->
          # Logger.debug("speed_cmd, as: #{cmd}/#{airspeed}")
          cmd = min(cmd, ff_speed_max_mps)
          # Logger.debug("speed_cmd, as: #{cmd}")
          :math.sqrt(cmd / ff_speed_max_mps)
        end
      else
        fn _, _ -> 0 end
      end

    speed_thrust_config =
      Keyword.put(speed_thrust_config, SCT.feed_forward_function(), speed_thrust_ff_function)

    speed_thrust_pid = ViaControllers.Pid.new(speed_thrust_config)

    # Altitude-Pitch
    altitude_pitch_ff_function = fn vv_cmd, airspeed ->
      # Logger.debug("vv_cmd/as: #{vv_cmd}/#{airspeed}")
      if vv_cmd != 0, do: :math.asin(min(vv_cmd / airspeed, 1)), else: 0
    end

    altitude_pitch_config =
      config[SCT.altitude_pitch_pid()]
      |> Keyword.put(SCT.feed_forward_function(), altitude_pitch_ff_function)

    altitude_pitch_pid = ViaControllers.Pid.new(altitude_pitch_config)

    # Roll-Course
    course_time_constant_s = config[SCT.course_roll_pid()][SCT.time_constant_s()]

    course_roll_ff_function = fn cmd, airspeed ->
      :math.atan(cmd / course_time_constant_s * airspeed / VC.gravity())
    end

    course_roll_config =
      config[SCT.course_roll_pid()]
      |> Keyword.put(SCT.feed_forward_function(), course_roll_ff_function)

    course_roll_pid = ViaControllers.Pid.new(course_roll_config)

    # Other params
    min_airspeed_for_climb_mps = Keyword.get(config, SCT.min_airspeed_for_climb_mps, 0)

    %ViaControllers.FixedWing.ScasStAp{
      speed_thrust_pid: speed_thrust_pid,
      altitude_pitch_pid: altitude_pitch_pid,
      course_roll_pid: course_roll_pid,
      min_airspeed_for_climb_mps: min_airspeed_for_climb_mps
    }
  end

  @spec update(struct(), map(), map()) :: tuple()
  def update(controller, commands, values) do
    %{
      SVN.course_rad() => course_rad,
      SVN.yaw_rad() => yaw_rad,
      SVN.groundspeed_mps() => groundspeed_mps,
      SVN.airspeed_mps() => airspeed_mps,
      SVN.dt_s() => dt_s,
      SVN.vertical_velocity_mps() => vv_mps,
      SVN.altitude_m() => altitude_m
    } = values

    %{
      SGN.sideslip_rad() => cmd_sideslip_rad,
      SGN.course_rad() => cmd_course_rad,
      SGN.altitude_m() => cmd_altitude_m,
      SGN.groundspeed_mps() => cmd_groundspeed_mps
    } = commands

    # Steering
    # dyaw = cmd_sideslip_rad + course_rad - yaw_rad
    # dyaw_left_right = ViaUtils.Motion.turn_left_or_right_for_correction(dyaw)

    # Logger.debug(
    #   "yaw/sdslp_cmd/dyaw/dyaw_lr: #{ViaUtils.Format.eftb_deg(yaw_rad, 1)}/#{ViaUtils.Format.eftb_deg(cmd_sideslip_rad, 1)}/#{ViaUtils.Format.eftb_deg(dyaw, 1)}/#{ViaUtils.Format.eftb_deg(dyaw_left_right, 1)}"
    # )

    # Logger.debug(
    #   "crs/crs_cmd: #{ViaUtils.Format.eftb_deg(course_rad, 1)}/#{ViaUtils.Format.eftb_deg(cmd_course_rad, 1)}"
    # )

    %{
      speed_thrust_pid: speed_thrust_pid,
      altitude_pitch_pid: altitude_pitch_pid,
      course_roll_pid: course_roll_pid,
      min_airspeed_for_climb_mps: min_airspeed_for_climb_mps
    } = controller

    deltayaw_rad =
      ViaUtils.Motion.turn_left_or_right_for_correction(cmd_sideslip_rad + course_rad - yaw_rad)

    delta_course_cmd_rad =
      ViaUtils.Motion.turn_left_or_right_for_correction(cmd_course_rad - course_rad)

    {course_roll_pid, roll_output_rad} =
      ViaControllers.Pid.update(
        course_roll_pid,
        delta_course_cmd_rad,
        0,
        airspeed_mps,
        dt_s
      )

    # Logger.debug("vv: #{ViaUtils.Format.eftb(vv_mps, 2)}")

    # Logger.debug("alt cmd/value/err: #{Common.Utils.eftb(alt_cmd,1)}/#{Common.Utils.eftb(altitude,1)}/#{Common.Utils.eftb(alt_cmd - altitude,1)}")
    excess_airspeed_mps =
      ViaUtils.Math.constrain(airspeed_mps - min_airspeed_for_climb_mps, -5, 5)

    altitude_corr_mult = 0.5 * (:math.sin(excess_airspeed_mps * :math.pi() / 10) + 1)

    altitude_corr = cmd_altitude_m - altitude_m

    # Logger.debug(
    #   "corr/exs_ap/mult/pre: #{ViaUtils.Format.eftb(altitude_corr, 1)}/#{ViaUtils.Format.eftb(excess_airspeed_mps, 3)}/#{ViaUtils.Format.eftb(altitude_corr_mult, 3)}/#{ViaUtils.Format.eftb(altitude_corr_mult * altitude_corr, 1)}"
    # )

    cmd_altitude_rate_mps =
      (altitude_corr_mult * altitude_corr) |> ViaUtils.Math.constrain(-5.0, 5.0)

    # Speed-Thrust
    {speed_thrust_pid, thrust_output_scaled} =
      ViaControllers.Pid.update(
        speed_thrust_pid,
        cmd_groundspeed_mps,
        groundspeed_mps,
        airspeed_mps,
        dt_s,
        false
      )

    # Altitude-Pitch
    {altitude_pitch_pid, pitch_output_rad} =
      ViaControllers.Pid.update(
        altitude_pitch_pid,
        cmd_altitude_rate_mps,
        vv_mps,
        airspeed_mps,
        dt_s,
        false
      )

    output = %{
      SGN.roll_rad() => roll_output_rad,
      SGN.pitch_rad() => pitch_output_rad,
      SGN.deltayaw_rad() => deltayaw_rad,
      SGN.thrust_scaled() => thrust_output_scaled
    }

    # Logger.debug("thrust cmd: #{ViaUtils.Format.eftb(output.thrust_scaled, 3)}")

    controller = %{
      controller
      | speed_thrust_pid: speed_thrust_pid,
        altitude_pitch_pid: altitude_pitch_pid,
        course_roll_pid: course_roll_pid
    }

    {controller, output}
  end
end
