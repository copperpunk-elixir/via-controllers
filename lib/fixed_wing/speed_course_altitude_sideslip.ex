defmodule ViaControllers.FixedWing.SpeedCourseAltitudeSideslip do
  require Logger
  require ViaUtils.Constants, as: VC
  require ViaUtils.Shared.ValueNames, as: SVN
  require ViaUtils.Shared.GoalNames, as: SGN

  defstruct [:tecs_energy, :tecs_balance, :roll_course_pid]

  @spec new(list()) :: struct()
  def new(config) do
    tecs_energy = ViaControllers.FixedWing.Tecs.Energy.new(config[:tecs_energy])
    tecs_balance = ViaControllers.FixedWing.Tecs.Balance.new(config[:tecs_balance])

    course_time_constant_s = config[:roll_course][:time_constant_s]

    roll_course_ff_function = fn cmd, airspeed ->
      :math.atan(cmd / course_time_constant_s * airspeed / VC.gravity())
    end

    roll_course_config =
      config[:roll_course] |> Keyword.put(:ff_function, roll_course_ff_function)

    roll_course_pid = ViaControllers.Pid.new(roll_course_config)

    %ViaControllers.FixedWing.SpeedCourseAltitudeSideslip{
      tecs_energy: tecs_energy,
      tecs_balance: tecs_balance,
      roll_course_pid: roll_course_pid
    }
  end

  @spec update(struct(), map(), map()) :: tuple()
  def update(controller, commands, values) do
    # Commands:
    # groundspeed_mps
    # course_rad
    # altitude_m
    # sideslip_ra   d

    # Values:
    # groundspeed_mps
    # course_rad
    # altitude_m
    # yaw_rad
    # vertical_velocity_mps
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

    deltayaw_rad =
      ViaUtils.Motion.turn_left_or_right_for_correction(cmd_sideslip_rad + course_rad - yaw_rad)

    delta_course_cmd_rad =
      ViaUtils.Motion.turn_left_or_right_for_correction(cmd_course_rad - course_rad)

    {roll_course_pid, roll_output_rad} =
      ViaControllers.Pid.update(
        controller.roll_course_pid,
        delta_course_cmd_rad,
        0,
        airspeed_mps,
        dt_s
      )

    # Speeds are [m/s]
    # Altitudes are [m]
    # Logger.debug("vv: #{ViaUtils.Format.eftb(vv_mps, 2)}")
    # CMDs

    # Logger.debug("alt cmd/value/err: #{Common.Utils.eftb(alt_cmd,1)}/#{Common.Utils.eftb(altitude,1)}/#{Common.Utils.eftb(alt_cmd - altitude,1)}")
    # Energy Cals
    potential_energy = VC.gravity() * altitude_m
    kinetic_energy = 0.5 * groundspeed_mps * groundspeed_mps

    dV = ViaUtils.Math.constrain(cmd_groundspeed_mps - groundspeed_mps, -5.0, 5.0)
    groundspeed_sp = cmd_groundspeed_mps
    potential_energy_sp = VC.gravity() * cmd_altitude_m
    kinetic_energy_sp = 0.5 * groundspeed_sp * groundspeed_sp

    # Logger.info("pe/pe_sp: #{Common.Utils.eftb(potential_energy,1)}/#{Common.Utils.eftb(potential_energy_sp,1)}")

    # energy = potential_energy + kinetic_energy
    # energy_sp = potential_energy_sp + kinetic_energy_sp
    speed_dot_sp = dV * dt_s

    kinetic_energy_rate_sp = groundspeed_mps * speed_dot_sp
    potential_energy_rate = vv_mps * VC.gravity()

    # TECS calcs
    # Energy (thrust)
    energy_cmds = %{
      potential_energy: potential_energy_sp,
      kinetic_energy: kinetic_energy_sp,
      kinetic_energy_rate: kinetic_energy_rate_sp,
      altitude_corr_m: cmd_altitude_m - altitude_m,
      groundspeed_mps: cmd_groundspeed_mps
    }

    energy_values = %{
      potential_energy: potential_energy,
      kinetic_energy: kinetic_energy,
      potential_energy_rate: potential_energy_rate,
      groundspeed_mps: groundspeed_mps,
      # airspeed: airspeed_mps,
      dt_s: dt_s
    }

    {tecs_energy, thrust_output_scaled} =
      ViaControllers.FixedWing.Tecs.Energy.update(
        controller.tecs_energy,
        energy_cmds,
        energy_values
      )

    # Balance (pitch)
    balance_cmds = %{
      # kinetic_energy: kinetic_energy_sp,
      # kinetic_energy_rate: kinetic_energy_rate_sp,
      altitude_corr: cmd_altitude_m - altitude_m
      # groundspeed_mps: cmd_groundspeed_mps
    }

    balance_values = %{
      # kinetic_energy: kinetic_energy,
      potential_energy: potential_energy,
      potential_energy_rate: potential_energy_rate,
      airspeed_mps: airspeed_mps,
      # groundspeed_mps: groundspeed_mps,
      # airspeed_mps: airspeed_mps,
      dt_s: dt_s
    }

    {tecs_balance, pitch_output_rad} =
      ViaControllers.FixedWing.Tecs.Balance.update(
        controller.tecs_balance,
        balance_cmds,
        balance_values
      )

    output = %{
      roll_rad: roll_output_rad,
      pitch_rad: pitch_output_rad,
      deltayaw_rad: deltayaw_rad,
      thrust_scaled: thrust_output_scaled
    }

    # Logger.debug("roll cmd: #{ViaUtils.Format.eftb_deg(roll_output_rad, 1)}")

    controller = %{
      controller
      | tecs_energy: tecs_energy,
        tecs_balance: tecs_balance,
        roll_course_pid: roll_course_pid
    }

    {controller, output}
  end
end
